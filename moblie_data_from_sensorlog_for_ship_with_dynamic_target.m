%% 1. Data Loading & Synchronization (Ship - CSV)
data_path = '/Users/alonorlicky/Documents/MATLAB/final project/new final project/expiriment/exp2/SensorLogFiles_my_iOS_device_260115_15-08-08/';
file_name = '2026-01-15_14_14_16_my_iOS_device.csv';

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

% --- Fix: Dynamic detection of Heading/Course column ---
allVars = valid_data.Properties.VariableNames;

% Function to find a variable containing a specific name but not 'Accuracy'
findCol = @(name) allVars(contains(allVars, name, 'IgnoreCase', true) & ~contains(allVars, 'Accuracy', 'IgnoreCase', true));

headingNames = findCol('TrueHeading');
courseNames = findCol('Course');

if ~isempty(headingNames)
    % Found Compass data (True Heading) - using it
    colName = headingNames{1};
    disp(['Using Heading from column: ', colName]);
    bearing_gps = valid_data.(colName);
elseif ~isempty(courseNames)
    % Compass not found, using GPS movement direction (Course)
    colName = courseNames{1};
    disp(['Using Course from column: ', colName]);
    bearing_gps = valid_data.(colName);
else
    error('Could not find TrueHeading or Course column in the CSV file.');
end

% Fill missing heading data
bearing_gps = fillmissing(bearing_gps, 'nearest');

% Convert to mathematical coordinate system (East = 0)
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

% Synchronize/Resample to 1Hz
mainTT = retime(mainTT, 'regular', 'linear', 'SampleRate', 1); 

%% 3. Dynamic Target Integration & Geometry
% Note: This section assumes 'simulink_target_X' and 'Y' exist from the Target script

% Verify that target data exists
if ~exist('simulink_target_X', 'var') || ~exist('simulink_target_Y', 'var')
    error('Target data (simulink_target_X/Y) not found in workspace. Please run the Target script first.');
end

% Crop data to the shortest common length
N = min([height(mainTT), length(simulink_target_X)]);

% Crop data to common length
ship_X = mainTT.X(1:N);
ship_Y = mainTT.Y(1:N);
ship_Bearing = mainTT.Bearing(1:N);

% Column 2 contains the values in the Simulink matrix format [Time, Data]
target_X_vals = simulink_target_X(1:N, 2); 
target_Y_vals = simulink_target_Y(1:N, 2);

% Geometric calculation (Theoretical Bearing to Dynamic Target)
delta_X = target_X_vals - ship_X;
delta_Y = target_Y_vals - ship_Y;
angleToTarget_Ref = atan2(delta_Y, delta_X); 

%% 4. Calibration & Scaling Logic
% Normalize angles to Pi range for comparison
beta_measured_norm = wrapToPi(ship_Bearing);
beta_theoretical_norm = wrapToPi(angleToTarget_Ref);

% Calculate fixed offset (Average error)
angle_offset = mean(beta_measured_norm - beta_theoretical_norm);

% Apply correction
bearing_corrected = wrapToPi(beta_measured_norm - angle_offset);

%% 5. Simulink Export Preparation
% Duplicate data 1000 times for continuous simulation
num_reps = 1000;

% Duplicate data
bearing_inf = repmat(bearing_corrected, num_reps, 1);
ship_X_inf = repmat(ship_X, num_reps, 1);
ship_Y_inf = repmat(ship_Y, num_reps, 1);

% Calculate time vector in seconds (0, 1, 2...)
total_samples = length(bearing_inf);
t_axis_seconds = (0:total_samples-1)'; 

% --- Final Variables for Simulink: Using TIMESERIES Objects ---
simulink_matrix_Bearing = timeseries(bearing_inf, t_axis_seconds);
simulink_matrix_Bearing.Name = 'Relative_Bearing';
simulink_matrix_X = timeseries(ship_X_inf, t_axis_seconds);
simulink_matrix_X.Name = 'Ship_X';
simulink_matrix_Y = timeseries(ship_Y_inf, t_axis_seconds);
simulink_matrix_Y.Name = 'Ship_Y';

disp('Dynamic Ship analysis complete.');
disp(['Total simulation time available: ', num2str(max(t_axis_seconds)), ' seconds.']);
disp('Variables ready as timeseries objects: simulink_matrix_Bearing, simulink_matrix_X, simulink_matrix_Y');
% Correct use of gpsTimes
plot(mainTT.gpsTimes(1:N), rad2deg(bearing_corrected));
title('Corrected Relative Bearing'); xlabel('Time'); ylabel('Bearing (deg)');
grid on;