%% Master_Run.m
% Executes the Base Config, Analytical Math, and Simscape models modularly.
clear; clc; close all;

%% 1. Load the pristine baseline parameters
disp('Loading Base Configuration...');
run('BaseSkript.m');

%% 2. Overrides (For this specific test only)
params.m_payload = 0.0;            

% --- BELT DRIVE & INERTIA PARAMETERS ---
params.enable_belt = 1;            % 1 = Belt Drive, 0 = Direct Drive
params.R_belt = 3.0;               % Mechanical reduction ratio (e.g., 3:1)

% Rotational Inertias [kg*m^2]
params.I_rotor = 0.40e-4;          % 400W Servo motor rotor inertia
params.I_pulley_motor = 1.0e-5;    % Estimated driving pulley inertia (small)
params.I_pulley_arm = 9.0e-5;      % Estimated driven pulley inertia (large)

% Trajectory specific variables required by Math_Model
params.cycle_time = 0.7;                  
params.sim_time = 12.0;                    
params.z_surface = -360;
params.z_lift = 0;
params.h_arch = 40;
params.r_arch = 100;
params.enable_vertical_drops = 0;
params.time_drop = 0.1;

%% 3. Execute Analytical Math Model (Sub-Script)
disp('Executing Math Model...');
run('Math_Model.m'); % Generates pos_math, tau_math, and t_sim

%% 4. Execute 3D Simscape Physics
disp('Executing Simscape model...');
try
    out = sim('SimulinkModel', 'StopTime', num2str(params.sim_time));
    disp('Simulation Complete.');
catch ME
    error('Simulation failed: %s', ME.message);
end

%% 5. Data Extraction & Interpolation
t_tau_raw = out.tau_motor.Time;
t_pos_raw = out.pos_cmd.Time;
tau_raw   = out.tau_motor.Data;
pos_c_raw = out.pos_cmd.Data;
pos_r_raw = out.EndEff_Pos.Data;

% Flatten matrices if Simulink exports 3D arrays
if ndims(tau_raw) == 3;   tau_raw = squeeze(tau_raw)';     end
if ndims(pos_c_raw) == 3; pos_c_raw = squeeze(pos_c_raw)'; end
if ndims(pos_r_raw) == 3; pos_r_raw = squeeze(pos_r_raw)'; end

% Interpolate variable-step Simscape data onto the fixed Math timeline
tau_sim_interp = interp1(t_tau_raw, tau_raw, t_sim, 'linear', 'extrap');
pos_c_interp   = interp1(t_pos_raw, pos_c_raw, t_sim, 'linear', 'extrap');
pos_r_interp   = interp1(t_pos_raw, pos_r_raw, t_sim, 'linear', 'extrap');

% Calculate following error delta [mm]
delta_pos = pos_r_interp - pos_c_interp;

%% 6. Plotting - Figure 1: Kinematics (Real vs Cmd & Delta)
c_light = [1.0 0.4 0.4; 0.4 1.0 0.4; 0.4 0.6 1.0]; % Light Solid (Cmd)
c_dark  = [0.6 0.0 0.0; 0.0 0.6 0.0; 0.0 0.0 0.6]; % Dark Dashed (Real)
figure('Name', 'Kinematics: Position & Delta Error', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% Top: Position (Real vs Commanded)
subplot(2, 1, 1);
for i = 1:3
    plot(t_sim, pos_c_interp(:,i), '-', 'Color', c_light(i,:), 'LineWidth', 1.5); hold on;
    plot(t_sim, pos_r_interp(:,i), '--', 'Color', c_dark(i,:), 'LineWidth', 2);
end
grid on; title('Cartesian Path (Real vs Commanded)'); 
xlabel('Time [s]'); ylabel('Position [mm]');
legend('X Cmd', 'X Real', 'Y Cmd', 'Y Real', 'Z Cmd', 'Z Real', 'Location', 'eastoutside');
xlim([0 params.sim_time]);

% Bottom: Error Delta
subplot(2, 1, 2);
for i = 1:3
    plot(t_sim, delta_pos(:,i), 'Color', c_dark(i,:), 'LineWidth', 1.5); hold on;
end
grid on; title('Cartesian Following Error (Real - Commanded)');
xlabel('Time [s]'); ylabel('\Delta Error [mm]');
legend('\Delta X', '\Delta Y', '\Delta Z', 'Location', 'eastoutside');
xlim([0 params.sim_time]);

%% 7. Plotting - Figure 2: Dynamics (Sim vs Math Torques)
figure('Name', 'Dynamics: Torque Validation', 'Color', 'w', 'Position', [1120, 100, 700, 800]);
labels = {'Motor 1', 'Motor 2', 'Motor 3'};
for i = 1:3
    subplot(3, 1, i);
    plot(t_sim, tau_math(i,:), '-', 'Color', c_light(i,:), 'LineWidth', 1.5); hold on; grid on;
    plot(t_sim, tau_sim_interp(:,i), '-', 'Color', c_dark(i,:), 'LineWidth', 2);
    title(sprintf('%s Torque', labels{i}));
    ylabel('Torque [Nm]');
    legend('Theoretical (Math)', 'Simscape (Real)', 'Location', 'eastoutside');
    xlim([0 params.sim_time]);
    if i == 3; xlabel('Time [s]'); end
end

%% 8. Plotting - Figure 3: 3D Trajectory Comparison
figure('Name', '3D Trajectory Comparison');
hold on; grid on; view(3);

% 1. Theoretical Math Model (pos_math is a 3xN matrix in meters)
plot3(pos_math(1,:), pos_math(2,:), pos_math(3,:), ...
    'b--', 'LineWidth', 1.5, 'DisplayName', 'Theoretical (Math)');

% 2. Commanded Trajectory from Simulink (Converted from mm to m)
plot3(pos_c_interp(:,1)*1e-3, pos_c_interp(:,2)*1e-3, pos_c_interp(:,3)*1e-3, ...
    'g-', 'LineWidth', 1.5, 'DisplayName', 'Commanded (Simulink)');

% 3. Real/Measured Trajectory from Simscape (Converted from mm to m)
plot3(pos_r_interp(:,1)*1e-3, pos_r_interp(:,2)*1e-3, pos_r_interp(:,3)*1e-3, ...
    'r-', 'LineWidth', 1.5, 'DisplayName', 'Real (Simscape)');

% Formatting for 3D inspection
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('End-Effector Trajectory Comparison');
legend('Location', 'best');
axis equal;       
rotate3d on;      
hold off;



