%% 1. Data Loading & Synchronization (Target - CSV)
% Update the path and filename to your CSV location
data_path = '/Users/alonorlicky/Documents/MATLAB/final project/new final project/expiriment/exp2/good_simulation/'; 
file_name = 'linear_ship_static_target_exp2.csv'; % Changed to .csv

% Create full file path
fullFilePath = fullfile(data_path, file_name);

% Define import options
opts = detectImportOptions(fullFilePath);
opts.VariableNamingRule = 'modify'; 
data_table = readtable(fullFilePath, opts);

% Filter rows missing GPS data
idx_valid = ~isnan(data_table.locationLatitude_WGS84_) & ~isnan(data_table.locationLongitude_WGS84_);
valid_data = data_table(idx_valid, :);

% Extract time (Time Parsing)
gpsTimes = datetime(valid_data.loggingTime_txt_, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSZ', 'TimeZone', 'UTC');
gpsTimes.TimeZone = 'local'; 

% Extract location data
lat = valid_data.locationLatitude_WGS84_;
lon = valid_data.locationLongitude_WGS84_;
alt = valid_data.locationAltitude_m_;

%% 2. Coordinate Transformation (LLA -> ENU)
wgs84 = wgs84Ellipsoid("meter");

% Convert target position to local coordinate system (meters)
% Note: Using the first point as reference (0,0,0) for the ENU frame
ref_lat = lat(1);
ref_lon = lon(1);
ref_alt = alt(1);

[Xe, Ye, Ze] = geodetic2ecef(wgs84, lat, lon, alt); 
[X, Y, ~] = ecef2enu(Xe, Ye, Ze, ref_lat, ref_lon, ref_alt, wgs84);

% Create a single central Timetable for the Target
mainTT_Target = timetable(gpsTimes, X, Y, 'VariableNames', {'X', 'Y'});

% Handle time duplicates (if any exist in CSV)
[mainTT_Target, ~] = unique(mainTT_Target);

% Synchronize to 1Hz
mainTT_Target = retime(mainTT_Target, 'regular', 'linear', 'SampleRate', 1); 

%% 3. Manual Calibration & Adjustments
% Applying specific offsets as defined in experimental setup
offset_X = 5; 
offset_Y = -3.4;

% Apply polarity correction and offsets
target_X_calibrated = 1 * (mainTT_Target.X + offset_X);
target_Y_calibrated = -1 * (mainTT_Target.Y - offset_Y);

%% 4. Simulink Export Preparation
% Replicate data 1000 times to create a long series
num_reps = 1000;
target_X_inf = repmat(target_X_calibrated, num_reps, 1);
target_Y_inf = repmat(target_Y_calibrated, num_reps, 1);
t_axis = (0:length(target_X_inf)-1)' * seconds(1);

% --- Final Variables for Simulink 'From Workspace' Blocks ---
% Keeping the Matrix format [Time, Data] because the Ship script expects column 2
simulink_target_X = [seconds(t_axis), target_X_inf];
simulink_target_Y = [seconds(t_axis), target_Y_inf];

disp('Target data processed (CSV Source).');
disp('Variables created: simulink_target_X, simulink_target_Y');
disp('You can now run the Ship script.');

%% 5. Visualization
figure;
plot(target_X_calibrated, target_Y_calibrated);
hold on;
scatter(target_X_calibrated(1), target_Y_calibrated(1), 'g','filled');     % Start position
scatter(target_X_calibrated(end), target_Y_calibrated(end), 'r','filled'); % End position
title('Dynamic Target Trajectory in ENU Coordinates');
xlabel('East (meters)');
ylabel('North (meters)');
legend('path','start','end','FontSize',12);
grid on;
axis equal;
hold off;