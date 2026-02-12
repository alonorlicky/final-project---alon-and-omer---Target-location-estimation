%% 1. Data Loading & Synchronization
data_path = '';
file_name = 'nonlinear_ship_static_target_exp.csv';

% Create full file path
fullFilePath = fullfile(data_path, file_name);

% Define import options
opts = detectImportOptions(fullFilePath);
opts.VariableNamingRule = 'modify'; 
data_table = readtable(fullFilePath, opts);

% Filter rows with missing GPS data
idx_valid = ~isnan(data_table.locationLatitude_WGS84_) & ~isnan(data_table.locationLongitude_WGS84_);
valid_data = data_table(idx_valid, :);

% Extract time (Time Parsing)
gpsTimes = datetime(valid_data.loggingTime_txt_, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSZ', 'TimeZone', 'UTC');
gpsTimes.TimeZone = 'local'; 

% Extract location data
lat = valid_data.locationLatitude_WGS84_;
lon = valid_data.locationLongitude_WGS84_;
alt = valid_data.locationAltitude_m_;

% Fix Dynamic detection of Heading/Course column
allVars = valid_data.Properties.VariableNames;
% Function to find a variable containing a specific name but not 'Accuracy'
findCol = @(name) allVars(contains(allVars, name, 'IgnoreCase', true) & ~contains(allVars, 'Accuracy', 'IgnoreCase', true));

headingNames = findCol('TrueHeading');
courseNames = findCol('Course');

if ~isempty(headingNames)
    % Found Compass data (True Heading)
    colName = headingNames{1};
    disp(['Using Heading from column: ', colName]);
    bearing_gps = valid_data.(colName);
elseif ~isempty(courseNames)
    % Compass not found, using GPS movement direction
    colName = courseNames{1};
    disp(['Using Course from column: ', colName]);
    bearing_gps = valid_data.(colName);
else
    error('Could not find TrueHeading or Course column in the CSV file.');
end

% Fill missing heading data
bearing_gps = fillmissing(bearing_gps, 'nearest');

% Convert to mathematical coordinate system
bearing_math = 90 - bearing_gps; 

%% 2. Coordinate Transformation (LLA -> ENU)
wgs84 = wgs84Ellipsoid("meter");

% Convert ship location to local coordinate system
ref_lat = lat(1);
ref_lon = lon(1);
ref_alt = alt(1);

[Xe, Ye, Ze] = geodetic2ecef(wgs84, lat, lon, alt); 
[X, Y, ~] = ecef2enu(Xe, Ye, Ze, ref_lat, ref_lon, ref_alt, wgs84);

% Create central Timetable
mainTT = timetable(gpsTimes, X, Y, deg2rad(bearing_math), 'VariableNames', {'X', 'Y', 'Bearing'});

% Handle time duplicates
[mainTT, ~] = unique(mainTT);

% Resample/Synchronize to 1Hz
mainTT = retime(mainTT, 'regular', 'linear', 'SampleRate', 1); 

%% 3. Static Target & Theoretical Geometry
target_lat = 32.06738; target_long = 34.76281; target_alt = 0;

% Convert target location to ENU
[tXe, tYe, tZe] = geodetic2ecef(wgs84, target_lat, target_long, target_alt);
[tX, tY, ~] = ecef2enu(tXe, tYe, tZe, ref_lat, ref_lon, ref_alt, wgs84);

% Geometric calculation
angleToTarget_Ref = unwrap(atan2(tY - mainTT.Y, tX - mainTT.X)); 
target_dist = hypot(tX, tY); 

%% 4. Calibration & Scaling Logic
beta_measured = mainTT.Bearing;

% Calculate Scale Factor
range_theo = max(angleToTarget_Ref) - min(angleToTarget_Ref);
range_mes = max(beta_measured) - min(beta_measured);

if range_mes == 0
    scale_factor = 1;
    disp('Warning: Measurement range is zero. Scale factor set to 1.');
else
    scale_factor = range_theo / range_mes;
end

% Final calibration
calibrated = ((beta_measured - mean(beta_measured, 'omitnan')) * scale_factor) + mean(angleToTarget_Ref, 'omitnan');

%% 5. Simulink Export Preparation
num_reps = 1000;
bearing_inf = repmat(calibrated, num_reps, 1);
t_axis = (0:length(bearing_inf)-1)' * seconds(1); 

% Final Variables for Simulink
simulink_matrix_Bearing = [seconds(t_axis), bearing_inf];
simulink_matrix_X = [seconds(t_axis), repmat(mainTT.X, num_reps, 1)];
simulink_matrix_Y = [seconds(t_axis), repmat(mainTT.Y, num_reps, 1)];

TimetableSimulinkOutput = timetable(t_axis, bearing_inf, 'VariableNames', {'Relative_Bearing'});

disp(['Target located at ENU: X=', num2str(tX), ' Y=', num2str(tY)]);
disp('Simulink input matrices (Bearing, X, Y) are ready in workspace.');

%% 6. Visualization 
figure('Name', 'Trajectory Analysis', 'Color', 'w'); % Set background to white

% Style Configurations
pathColor = [0, 0.447, 0.741];        
targetColor = [0.929, 0.694, 0.125];  
startColor = [0.466, 0.674, 0.188];   
endColor = [0.635, 0.078, 0.184];     
targetMarker = 'h';                  

% Plotting
% 1. Plot the Ship's Path
plot(mainTT.X, mainTT.Y, 'Color', pathColor, 'LineWidth', 2, 'LineStyle', '-');
hold on;

% 2. Plot Start & End Points
scatter(mainTT.X(1), mainTT.Y(1), 80, 'o', ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', startColor, 'LineWidth', 1.2);
scatter(mainTT.X(end), mainTT.Y(end), 80, 's', ... % Square shape for end
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', endColor, 'LineWidth', 1.2);

% 3. Plot the Target
scatter(tX, tY, 400, targetMarker, ...
    'MarkerEdgeColor', [0.2 0.2 0.2], ...
    'MarkerFaceColor', targetColor, ...
    'LineWidth', 1.5);

% Aesthetics & Scaling
title('Ship Trajectory relative to Static Target', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('East [m]', 'FontSize', 12);
ylabel('North [m]', 'FontSize', 12);

% Create a clean legend
legend({'Ship Path', 'Start Point', 'End Point', 'Target'}, ...
       'Location', 'bestoutside', 'FontSize', 11, 'Box', 'off');

grid on;
grid minor; 
axis equal; % Ensures 1 meter North = 1 meter East visually

% Dynamic Zoom/Scaling 
padding = 5; % meters buffer around the plot
all_X = [mainTT.X; tX];
all_Y = [mainTT.Y; tY];
xlim([min(all_X)-padding, max(all_X)+padding]);
ylim([min(all_Y)-padding, max(all_Y)+padding]);

hold off;