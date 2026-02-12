%% 1. Data Loading (Ship - CSV)
data_path = ''; 
file_name = '';

% Create Full Path
fullFilePath = fullfile(data_path, file_name);
if ~isfile(fullFilePath)
    error(['ERROR: Ship file not found: ' fullFilePath]);
end

opts = detectImportOptions(fullFilePath);
opts.VariableNamingRule = 'modify'; 
data_table = readtable(fullFilePath, opts);

% Auto-detect Columns
allVars = data_table.Properties.VariableNames;
findCol = @(keyword) allVars(find(contains(lower(allVars), lower(keyword)), 1));
timeName = findCol('loggingtime');
if isempty(timeName), timeName = findCol('time'); end

latName = findCol('latitude');
lonName = findCol('longitude');
idx_valid = ~isnan(data_table.(char(latName))) & ~isnan(data_table.(char(lonName)));
valid_data = data_table(idx_valid, :);

% Parse Time
try
    gpsTimes = datetime(valid_data.(char(timeName)), 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSZ', 'TimeZone', 'UTC');
catch
    gpsTimes = datetime(valid_data.(char(timeName)), 'TimeZone', 'UTC');
end
gpsTimes.TimeZone = 'local';

% LOGIC FOR CUTTING DATA (SYNC)
if exist('target_start_abs', 'var')
    disp('Syncing: Cutting Ship data before Target start time...');
    idx_keep = gpsTimes >= target_start_abs;
    
    if sum(idx_keep) < 10
        error('Sync Error: Ship recording ended before Target started (or no overlap). Check file times.');
    end
    
    valid_data = valid_data(idx_keep, :);
    gpsTimes = gpsTimes(idx_keep);
    disp(['Ship data cropped. New Start Time: ', char(gpsTimes(1))]);
else
    disp('First Run: Using all Ship data (Target start unknown yet).');
end

% Extract Data
ship_lat = valid_data.(char(latName));
ship_lon = valid_data.(char(lonName));
altName = findCol('altitude');
ship_alt = valid_data.(char(altName));

% Extract Orientation
headingNames = findCol('TrueHeading');
courseNames = findCol('Course');
if ~isempty(headingNames)
    bearing_raw = valid_data.(headingNames{1});
elseif ~isempty(courseNames)
    bearing_raw = valid_data.(courseNames{1});
else
    error('No Heading column found');
end

% Smoothing
bearing_resampled = fillmissing(bearing_raw, 'spline');

%% 2. Coordinate Transformation
bearing_resampled_east = 90 - bearing_resampled;
bearing_radians = deg2rad(bearing_resampled_east);
wgs84 = wgs84Ellipsoid("meter");
ref_lat = ship_lat(1);
ref_lon = ship_lon(1);
ref_alt = ship_alt(1);
[X_e, Y_e, Z_e] = geodetic2ecef(wgs84, ship_lat, ship_lon, ship_alt);
[X, Y, Z] = ecef2enu(X_e, Y_e, Z_e, ref_lat, ref_lon, ref_alt, wgs84);
X_ship = X;
Y_ship = Y;
Bearing_vec = bearing_radians;

if ~exist('simulink_target_X', 'var')
    warning('Target data not found! Run Target script, then run this Ship script again.');
    return;
end

%% 3. Sync & Calculations
t_target_in = simulink_target_X(:, 1);
x_target_in = simulink_target_X(:, 2);
y_target_in = simulink_target_Y(:, 2);
t_ship = seconds(gpsTimes - gpsTimes(1));
[t_ship_unique, idx_unique] = unique(t_ship); 
X_ship = X_ship(idx_unique);
Y_ship = Y_ship(idx_unique);
Bearing_vec = Bearing_vec(idx_unique);
t_ship = t_ship_unique;
target_X_aligned = interp1(t_target_in, x_target_in, t_ship, 'linear', 'extrap');
target_Y_aligned = interp1(t_target_in, y_target_in, t_ship, 'linear', 'extrap');
delta_X = target_X_aligned - X_ship;
delta_Y = target_Y_aligned - Y_ship;
beta_theoretical = atan2(delta_Y, delta_X);
beta_measured_norm = wrapToPi(Bearing_vec);
beta_theoretical_norm = wrapToPi(beta_theoretical);
angle_offset = mean(beta_measured_norm - beta_theoretical_norm);
beta_measured_corrected = wrapToPi(beta_measured_norm - angle_offset);

%% 4. Prepare for Simulink
t_final = (0:1:floor(t_ship(end)))';
beta_sim = interp1(t_ship, beta_measured_corrected, t_final, 'pchip');
ship_X_sim = interp1(t_ship, X_ship, t_final, 'pchip');
ship_Y_sim = interp1(t_ship, Y_ship, t_final, 'pchip');
target_X_sim = interp1(t_ship, target_X_aligned, t_final, 'pchip');
target_Y_sim = interp1(t_ship, target_Y_aligned, t_final, 'pchip');
num_loops = 1000;
t_total = (0:(length(t_final)*num_loops)-1)' * 1.0; 
simulink_matrix_Bearing = timeseries(repmat(beta_sim, num_loops, 1), t_total);
simulink_matrix_Bearing.Name = 'Relative_Bearing';
simulink_matrix_X = timeseries(repmat(ship_X_sim, num_loops, 1), t_total);
simulink_matrix_X.Name = 'Ship_X';
simulink_matrix_Y = timeseries(repmat(ship_Y_sim, num_loops, 1), t_total);
simulink_matrix_Y.Name = 'Ship_Y';
simulink_target_X = [t_total, repmat(target_X_sim, num_loops, 1)];
simulink_target_Y = [t_total, repmat(target_Y_sim, num_loops, 1)];

disp('Sync Complete. Data Ready.');
figure('Name', 'Final Trajectory');
plot(X_ship, Y_ship, 'b'); hold on;
plot(target_X_aligned, target_Y_aligned, 'r');
legend('Ship', 'Target'); grid on; axis equal;