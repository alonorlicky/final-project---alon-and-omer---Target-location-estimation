%% 1. Data Loading (Target - CSV)
% Update Path to your folder
data_path = ''; 
file_name =''; 

% Create Full Path
fullFilePath = fullfile(data_path, file_name);
if ~isfile(fullFilePath)
    error(['ERROR: The file was not found at this path: ' newline fullFilePath newline 'Please check the file name digits (02 vs 08?)']);
end

% Load CSV
opts = detectImportOptions(fullFilePath);
opts.VariableNamingRule = 'modify'; % Make headers MATLAB-safe
data_table = readtable(fullFilePath, opts);

% Auto-detect correct column names 
allVars = data_table.Properties.VariableNames;
findCol = @(keyword) allVars(find(contains(lower(allVars), lower(keyword)), 1));

% 1. LATITUDE
latName = allVars(find(contains(lower(allVars), 'latitude') & ~contains(lower(allVars), 'accuracy'), 1));
if isempty(latName), latName = findCol('lat'); end 

% 2. LONGITUDE
lonName = allVars(find(contains(lower(allVars), 'longitude') & ~contains(lower(allVars), 'accuracy'), 1));
if isempty(lonName), lonName = findCol('lon'); end 

% 3. ALTITUDE
altName = findCol('altitude');
if isempty(altName), altName = findCol('alt'); end

% 4. TIME
timeName = findCol('loggingtime');
if isempty(timeName), timeName = findCol('time'); end

if isempty(latName) || isempty(timeName)
    disp('Available columns:'); disp(allVars');
    error('Could not find Lat/Time columns. Check names above.');
end

disp(['Target: Using columns: ', char(latName), ', ', char(timeName)]);

% Extract Data
idx_valid = ~isnan(data_table.(char(latName))) & ~isnan(data_table.(char(lonName)));
valid_data = data_table(idx_valid, :);

% Parse Time
try
    gpsTimes = datetime(valid_data.(char(timeName)), 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSZ', 'TimeZone', 'UTC');
catch
    warning('Standard time format failed, trying auto-detection...');
    gpsTimes = datetime(valid_data.(char(timeName)), 'TimeZone', 'UTC');
end
gpsTimes.TimeZone = 'local';

% Export Absolute Start Time
target_start_abs = gpsTimes(1); 

% Extract Position
target_lat = valid_data.(char(latName));
target_lon = valid_data.(char(lonName));
target_alt = valid_data.(char(altName));

%% 2. Processing
wgs84 = wgs84Ellipsoid("meter");
[X_e_target, Y_e_target, Z_e_target] = geodetic2ecef(wgs84, target_lat, target_lon, target_alt); 

% Coordinate Sync
if exist('ref_lat', 'var')
    disp('Target: Using Ship reference for Coordinates.');
    [X_target, Y_target, Z_target] = ecef2enu(X_e_target, Y_e_target, Z_e_target, ref_lat, ref_lon, ref_alt, wgs84);
else
    warning('Target: Ship reference not found! Using Target start (0,0) temporarily.');
    [X_target, Y_target, Z_target] = ecef2enu(X_e_target, Y_e_target, Z_e_target, target_lat(1), target_lon(1), target_alt(1), wgs84);
end

% Time Vector (Always starts at 0 relative to itself)
t_target = seconds(gpsTimes - target_start_abs);

% Apply offsets
X_target = 1*(X_target+5);
Y_target = -1*(Y_target-3.4);

%EXPORT TO SIMULINK (Matrix Format)
simulink_target_X = [t_target, X_target];
simulink_target_Y = [t_target, Y_target];

disp(['Target Data Processed. Absolute Start: ', char(target_start_abs)]);
disp('Now run the Ship script again to crop the early ship data.');