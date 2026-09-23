%% R-Trident Rotary Delta Geometric Parameters
clear; clc;


% %% R-Trident Simscape Physical Parameters
% % Units: [mm], [kg], [kg/m^3], [degrees]
% 
% % Material Densities 
% params.rho_carbon_fiber = 1600; 
% params.rho_aluminum = 2700;     
% params.rho_steel = 7850;        
% 
% % Bicep
% params.bicep_length = 150; 
% params.bicep_width  = 16;   
% params.bicep_height = 30;
% params.bicep_geo =  [params.bicep_length params.bicep_width params.bicep_height];
% 
% % Forearm 
% params.forearm_length = 335;
% params.rod_radius   = 6; 
% 
% % End Effector
% params.plate_thickness = 10;
% params.plate_radius    = 35; 
% 
% % End-Effector Geometry
% params.e = params.plate_radius;   % tool plate radius
% 
% % Base Geometry
% params.f = 80.0;  % radius base
% 
% % Estimated Masses 
% bicep_vol = (params.bicep_length * params.bicep_width * params.bicep_height) * 1e-9; % m^3
% params.bicep_mass = bicep_vol * params.rho_aluminum;
% 
% rod_vol = (pi * (params.rod_radius/2)^2 * params.forearm_length) * 1e-9; % m^3
% params.forearm_mass = (rod_vol * params.rho_carbon_fiber) * 2; % 2 rods per arm
% 
% % Motor mounting angles 
% params.phi = [0, 120, 240]; 
% 
% % Motor mechanical limits (to prevent the arm from hitting the frame)
% params.theta_max = 115;   
% params.theta_min = -0;  

% % PID TUNING 
% Kp = 40;    
% Ki = 1.5;  
% Kd = 4;
% Nd = 100;

%% DeltaRobot_Cell Simscape Physical Parameters
% Units: [mm], [kg], [kg/m^3], [degrees]

% Material Densities 
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

% End Effector
params.plate_thickness = 10;
params.plate_radius    = 45; 

% End-Effector Geometry
params.e = params.plate_radius;   % tool plate radius

% Base Geometry
params.f = 125.0;  % radius base

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


% PID TUNING 
Kp = 40;    
Ki = 1.5;  
Kd = 4;
Nd = 100;

%% Calculated Constants (Used for the math blocks later)
% These constants speed up the PLC execution time
params.sqrt3 = sqrt(3);
params.sin120 = params.sqrt3 / 2;
params.cos120 = -0.5;

% Effective radius difference (The "Delta" offset)
params.wb = params.f;             % Base offset
params.up = params.e;             % Tool offset
params.sp = (params.wb - params.up); % Effective reach offset


%% Temp
%% R-Trident Workspace Boundary Calculator
% Change arm lengths here to match your active configuration:
L1 = params.bicep_length;    % e.g. 150
L2 = params.forearm_length;  % e.g. 400 (or 350)
sp = params.f - params.e;    % 80 - 35 = 45 mm

th_min = params.theta_min;   % 0 deg
th_max = params.theta_max;   % 115 deg
phi_deg = params.phi;        % [0, 120, 240]

% Discretization
r_test = 0:1:350;            % Radius search (mm)
z_test = -500:2:-100;        % Z depth search below base (mm)
az_deg = 0:15:345;           % 24 sample points around each circular slice

max_r_overall = 0;
best_z = NaN;

for r = r_test
    found_valid_z_for_this_r = false;
    
    for z = z_test
        circle_reachable = true;
        
        for az = az_deg * (pi/180)
            x = r * cos(az);
            y = r * sin(az);
            
            % Check all 3 arms
            for i = 1:3
                phi = phi_deg(i) * (pi/180);
                
                % Rotate (x, y) into arm frame
                xL =  x * cos(phi) + y * sin(phi);
                yL = -x * sin(phi) + y * cos(phi);
                
                % Target point relative to the knee pivot in local 2D plane:
                % (L1*cos(th) - (xL - sp))^2 + (-L1*sin(th) - z)^2 = L2^2 - yL^2
                rem_sq = L2^2 - yL^2;
                if rem_sq <= 0
                    circle_reachable = false; 
                    break; 
                end
                
                dx = xL - sp;
                % Equation: L1^2 + dx^2 + z^2 - rem_sq - 2*L1*dx*cos(th) + 2*L1*z*sin(th) = 0
                % Let E = -2*L1*dx,  F = 2*L1*z,  G = L1^2 + dx^2 + z^2 - rem_sq
                % E*cos(th) + F*sin(th) + G = 0
                E = -2 * L1 * dx;
                F =  2 * L1 * z;
                G = L1^2 + dx^2 + z^2 - rem_sq;
                
                % Solve via t = tan(th/2): (G - E)*t^2 + 2*F*t + (G + E) = 0
                qa = G - E;
                qb = 2 * F;
                qc = G + E;
                disc = qb^2 - 4 * qa * qc;
                
                if disc < 0
                    circle_reachable = false; 
                    break; 
                end
                
                % Two solutions for theta
                t1 = (-qb + sqrt(disc)) / (2 * qa);
                t2 = (-qb - sqrt(disc)) / (2 * qa);
                
                sol1 = 2 * atan(t1) * 180 / pi;
                sol2 = 2 * atan(t2) * 180 / pi;
                
                % Standard delta arm operates in elbow-down (smallest positive / active branch)
                valid1 = (sol1 >= th_min && sol1 <= th_max);
                valid2 = (sol2 >= th_min && sol2 <= th_max);
                
                % If neither solution falls within [0, 115], arm cannot reach
                if ~(valid1 || valid2)
                    circle_reachable = false;
                    break;
                end
            end
            
            if ~circle_reachable
                break;
            end
        end
        
        if circle_reachable
            found_valid_z_for_this_r = true;
            if r > max_r_overall
                max_r_overall = r;
                best_z = z;
            end
            break; % Found at least one working Z for this radius
        end
    end
    
    if ~found_valid_z_for_this_r && r > 10
        % Continuous circular boundary reached
        break;
    end
end

%% Servo Drive Scaling (A6-RS Parameters)
params.pulses_per_rev = 2^17; 
params.gear_ratio = 1;          
params.deg_to_pulses = (params.pulses_per_rev * params.gear_ratio) / 360;

%% Display Summary
fprintf('--- R-Trident Configuration Loaded ---\n');
fprintf('Configuration: L1 = %d mm, L2 = %d mm\n', L1, L2);
fprintf('Theta Limits : [%d deg, %d deg]\n', th_min, th_max);
fprintf('Max Usable Continuous Radius: %.2f mm (at Z = %.1f mm)\n', max_r_overall, best_z);
fprintf('Usable Cylinder Diameter    : %.2f mm\n', 2 * max_r_overall);
fprintf('Total Arm Length: %.2f mm\n', params.bicep_length + params.forearm_length);
fprintf('Servo Scaling: %.2f pulses/degree\n', params.deg_to_pulses);