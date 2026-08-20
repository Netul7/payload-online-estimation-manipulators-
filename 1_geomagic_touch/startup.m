% startup.m
disp('Setting up Geomagic Touch Simulation Environment...');

% 1. Add all subdirectories to the MATLAB path automatically
addpath(genpath(pwd));

% 2. Run the initialization script to load parameters into the workspace
run('datosGT.m');

% 3. Open the Simulink model automatically
open_system('Geo_ma_T_v3.slx');

disp('Environment ready. You can now run the Simulink model.');