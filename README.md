# final-project---alon-and-omer---Target-location-estimation
target Location Estimation Using Self-Coordinates and Measured Angle with RLS, EKF, and UKF
this project includes:
  1. simulation files (.slx)
  2. csv files from expiriments
  3. MATLAB scripts for processing the CSV files
  4. Simulink models for experiment data processing

Running Simulations for Different Scenarios
     
To simulate the different scenarios, run the following Simulink files:
  1.simulation_linear_ship_static_target_ALL_shortdistance.slx
  2.simulation_Nonlinear_ship_moving_target_ALL_shortdistance.slx
  3.simulation_Nonlinear_ship_static_target_ALL_shortdistance.slx

Processing Data from a Static Target Experiment

  1.Use one of the following CSV files, for example:
    a.linear_ship_static_target_exp.csv
    b.nonlinear_ship_static_target_exp.csv
  2.Configure the file path inside the script and run:
    mobile_data_sensorlog_to_simulink_static_target.m
  3.Adjust the target coordinates (x, y) inside the model and run:
    expiriment_static_target.slx
    
Processing Data from a Dynamic Target Experiment

  1.Clear the MATLAB workspace and command window.
  2.Configure the file path inside the script and run:
    moblie_data_from_sensorlog_for_ship_with_dynamic_target.m
  3.Configure the file path inside the script and run:
    mobile_data_sensorlog_to_simulink_target_for_dynamic_target.m
  4.run again this matlab code:
    moblie_data_from_sensorlog_for_ship_with_dynamic_target.m
  5.run this slx file: expiriment_dynamic_target.slx
  
