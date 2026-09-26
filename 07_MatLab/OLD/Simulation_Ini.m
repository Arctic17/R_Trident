%% DeltaRobot_Cell Simscape Physical Parameters
clear; clc;

% Units: [mm], [kg], [kg/m^3], [degrees]
params.rho_carbon_fiber = 1600; 
params.rho_aluminum = 2700;     
params.rho_steel = 7850;        

% Bicep
params.bicep_length = 170; 
params.bicep_width  = 16;   
params.bicep_height = 30;
params.bicep_geo =  [params.bicep_length params.bicep_width params.bicep_height];

% Forearm 
params.forearm_length = 400;
params.rod_radius   = 6; 

% End Effector & Base
params.plate_thickness = 10;
params.plate_radius    = 45; 
params.e = params.plate_radius;   
params.f = 125.0;  

% Estimated Masses 
bicep_vol = (params.bicep_length * params.bicep_width * params.bicep_height) * 1e-9; 
params.bicep_mass = bicep_vol * params.rho_aluminum;

rod_vol = (pi * (params.rod_radius/2)^2 * params.forearm_length) * 1e-9; 
params.forearm_mass = (rod_vol * params.rho_carbon_fiber) * 2; 

% Motor Configuration
params.phi = [0, 120, 240]; 
params.theta_max = 115;   
params.theta_min = -0;  

% Belt Drive & Inertia Parameters
params.enable_belt = 1;            
params.R_belt = 3.0;               
params.I_rotor = 0.60e-4;          
params.I_pulley_motor = 1.0e-5;    
params.I_pulley_arm = 9.0e-5;