%% R-Trident Rotary Delta Geometric Parameters
clear; clc;


%% R-Trident Simscape Physical Parameters
% Units: [mm], [kg], [kg/m^3], [degrees]

% Material Densities 
params.rho_carbon_fiber = 1600; 
params.rho_aluminum = 2700;     
params.rho_steel = 7850;        

% Bicep
params.bicep_length = 150; 
params.bicep_width  = 16;   
params.bicep_height = 30;
params.bicep_geo =  [params.bicep_length params.bicep_width params.bicep_height];

% Forearm 
params.forearm_length = 335;
params.rod_radius   = 6; 

% End Effector
params.plate_thickness = 10;
params.plate_radius    = 35; 

% End-Effector Geometry
params.e = params.plate_radius;   % tool plate radius

% Base Geometry
params.f = 80.0;  % radius base

% Estimated Masses 
bicep_vol = (params.bicep_length * params.bicep_width * params.bicep_height) * 1e-9; % m^3
params.bicep_mass = bicep_vol * params.rho_aluminum;

rod_vol = (pi * (params.rod_radius/2)^2 * params.forearm_length) * 1e-9; % m^3
params.forearm_mass = (rod_vol * params.rho_carbon_fiber) * 2; % 2 rods per arm

% Motor mounting angles 
params.phi = [0, 120, 240]; 

% Motor mechanical limits (to prevent the arm from hitting the frame)
params.theta_max = 115;   
params.theta_min = -0;  

%% Calculated Constants (Used for the math blocks later)
% These constants speed up the PLC execution time
params.sqrt3 = sqrt(3);
params.sin120 = params.sqrt3 / 2;
params.cos120 = -0.5;

% Effective radius difference (The "Delta" offset)
params.wb = params.f;             % Base offset
params.up = params.e;             % Tool offset
params.sp = (params.wb - params.up); % Effective reach offset

%% Servo Drive Scaling (A6-RS Parameters)
params.pulses_per_rev = 2^17; 
params.gear_ratio = 1;          
params.deg_to_pulses = (params.pulses_per_rev * params.gear_ratio) / 360;

%% --- PID TUNING ---
Kp = 40;    
Ki = 1.5;  
Kd = 4;
Nd = 100;
% ------------------------------------
%% Display Summary
fprintf('--- R-Trident Configuration Loaded ---\n');
fprintf('Reach Radius (sp): %.2f mm\n', params.sp);
fprintf('Total Arm Length: %.2f mm\n', params.bicep_length + params.forearm_length);
fprintf('Servo Scaling: %.2f pulses/degree\n', params.deg_to_pulses);