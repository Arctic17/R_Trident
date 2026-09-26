%% Master Delta Robot Script: Analytical Math vs. Simscape (Non-Destructive)
clear; clc; close all;

%% 1. Initialization & Overrides
% Load the pristine baseline parameters
run('BaseSkript.m'); 

% --- OVERRIDE SECTION ---
params.enable_belt = 0;            % 0 = Direct-Drive, 1 = Belt-Drive
params.I_rotor = 1.20e-4;          % Test different rotor inertia
params.m_payload = 0.5;            % 500g payload added
cycle_time = 0.5;                  % Trajectory cycle speed
sim_time = 10.0;                   % Total run time

% Standardize geometric parameters to meters for the math engine
params.L = params.bicep_length * 1e-3;
params.l = params.forearm_length * 1e-3;
params.f = params.f * 1e-3;
params.e = params.e * 1e-3;

% Recalculate effective dynamics with the new payload override
m_elbow = 0.5 * params.forearm_mass;
plate_vol = pi * (params.e)^2 * (params.plate_thickness * 1e-3);
m_plate = plate_vol * params.rho_aluminum; 
m_ee_eff = m_plate + params.m_payload + 3 * m_elbow;
I_bicep = (1/3) * params.bicep_mass * params.L^2;
I_eff_arm = I_bicep + m_elbow * (params.L^2);

%% 2. Calculate Analytical Mathematical Baseline
disp('Calculating theoretical mathematical dynamics...');
t_sim = 0:0.002:sim_time;
N = length(t_sim);
pos_math = zeros(3, N);

% 2a. Minimum-Jerk Trajectory Generation
for k = 1:N
    [x, y, z] = trajectory_fcn(t_sim(k), -320, cycle_time);
    pos_math(:, k) = [x; y; z] * 1e-3; 
end

dt = t_sim(2) - t_sim(1);
vel_math = [zeros(3,1), diff(pos_math, 1, 2) / dt];
acc_math = [zeros(3,1), diff(vel_math, 1, 2) / dt];

% 2b. Virtual Work Dynamics Engine
theta = zeros(3, N);
theta_dot = zeros(3, N);
tau_math = zeros(3, N);

for k = 1:N
    P = pos_math(:, k);
    V = vel_math(:, k);
    A = acc_math(:, k);
    
    q = delta_IK(P, params);
    theta(:, k) = q;
    
    [J_inv, J] = delta_Jacobian(q, P, params);
    
    q_dot = J_inv * V;
    theta_dot(:, k) = q_dot;
    
    F_ee = m_ee_eff * (A - params.g);
    tau_ee = J' * F_ee; % Reflected platform torque
    
    if k == 1
        q_ddot = zeros(3,1);
    else
        q_ddot = (theta_dot(:, k) - theta_dot(:, k-1)) / dt;
    end
    
    % Select inertia profile based on drive type
    if params.enable_belt == 1
        I_total = I_eff_arm + params.I_pulley_arm + ((params.I_rotor + params.I_pulley_motor) * params.R_belt^2);
        tau_arm_iner = I_total * q_ddot;
        tau_arm_g = (params.bicep_mass * (params.L/2) + m_elbow * params.L) * 9.81 * cos(q);
        tau_math(:, k) = (tau_ee + tau_arm_iner + tau_arm_g) / params.R_belt;
    else
        I_total = I_eff_arm + params.I_rotor;
        tau_arm_iner = I_total * q_ddot;
        tau_arm_g = (params.bicep_mass * (params.L/2) + m_elbow * params.L) * 9.81 * cos(q);
        tau_math(:, k) = tau_ee + tau_arm_iner + tau_arm_g;
    end
end

%% 3. Execute Simscape Physics Engine
disp('Starting Simscape execution with temporary overrides...');
assignin('base', 'params', params);
assignin('base', 'cycle_time', cycle_time);

try
    out = sim('SimulinkModel', 'StopTime', num2str(sim_time));
    disp('Simulation Complete.');
catch ME
    error('Simulation failed: %s', ME.message);
end

%% 4. Extract & Interpolate Simscape Data
% Extract raw timeseries
t_tau_raw = out.tau_motor.Time;
t_pos_raw = out.pos_cmd.Time;

if length(t_tau_raw) < 2
    error('Simulation aborted! Check the Simulink Diagnostic Viewer.');
end

% Flatten data arrays in case Simulink exported 3D Mux tensors
tau_raw   = out.tau_motor.Data;
pos_c_raw = out.pos_cmd.Data;
pos_r_raw = out.EndEff_Pos.Data;

if ndims(tau_raw) == 3;   tau_raw = squeeze(tau_raw)';     end
if ndims(pos_c_raw) == 3; pos_c_raw = squeeze(pos_c_raw)'; end
if ndims(pos_r_raw) == 3; pos_r_raw = squeeze(pos_r_raw)'; end

% Interpolate variable-step Simscape data onto the uniform math time vector (t_sim)
tau_sim_interp = interp1(t_tau_raw, tau_raw, t_sim, 'linear', 'extrap');
pos_c_interp   = interp1(t_pos_raw, pos_c_raw, t_sim, 'linear', 'extrap');
pos_r_interp   = interp1(t_pos_raw, pos_r_raw, t_sim, 'linear', 'extrap');

% Ensure Simscape output is converted to mm for Cartesian plots if it outputs meters natively
if max(abs(pos_r_interp(:))) < 2.0 
    pos_r_interp = pos_r_interp * 1000; 
end

%% 5. Display Superimposed Wide Layout
c_light = [0.4 0.6 1.0; 0.4 1.0 0.4; 1.0 0.4 0.4]; % X/1=Blue, Y/2=Green, Z/3=Red (Light)
c_dark  = [0.0 0.0 0.6; 0.0 0.6 0.0; 0.6 0.0 0.0]; % X/1=Blue, Y/2=Green, Z/3=Red (Dark)

figure('Name', 'Math vs Simscape Validation', 'Color', 'w', 'Position', [100, 100, 1400, 900]);

% --- Plot 1: Command vs Real Positions ---
subplot(2, 1, 1);
plot(t_sim, pos_c_interp(:,1), '-', 'Color', c_light(1,:), 'LineWidth', 1.5); hold on; grid on;
plot(t_sim, pos_r_interp(:,1), '--', 'Color', c_dark(1,:),  'LineWidth', 2);
plot(t_sim, pos_c_interp(:,2), '-', 'Color', c_light(2,:), 'LineWidth', 1.5);
plot(t_sim, pos_r_interp(:,2), '--', 'Color', c_dark(2,:),  'LineWidth', 2);
plot(t_sim, pos_c_interp(:,3), '-', 'Color', c_light(3,:), 'LineWidth', 1.5);
plot(t_sim, pos_r_interp(:,3), '--', 'Color', c_dark(3,:),  'LineWidth', 2);
title('Cartesian Trajectory: Command (Solid) vs Real Simscape (Dashed)'); 
xlabel('Time [s]'); ylabel('Position [mm]'); 
legend('X Cmd', 'X Real', 'Y Cmd', 'Y Real', 'Z Cmd', 'Z Real', 'Location', 'eastoutside');
xlim([0 sim_time]);

% --- Plot 2: Mathematical Torque vs Simscape Torque ---
subplot(2, 1, 2);
plot(t_sim, tau_math(1,:), '-', 'Color', c_light(1,:), 'LineWidth', 1.5); hold on; grid on;
plot(t_sim, tau_sim_interp(:,1), '--', 'Color', c_dark(1,:), 'LineWidth', 2);
plot(t_sim, tau_math(2,:), '-', 'Color', c_light(2,:), 'LineWidth', 1.5);
plot(t_sim, tau_sim_interp(:,2), '--', 'Color', c_dark(2,:), 'LineWidth', 2);
plot(t_sim, tau_math(3,:), '-', 'Color', c_light(3,:), 'LineWidth', 1.5);
plot(t_sim, tau_sim_interp(:,3), '--', 'Color', c_dark(3,:), 'LineWidth', 2);
title(sprintf('Motor Torques: Mathematical Baseline (Solid) vs Simscape Reality (Dashed) | Payload: %.1f kg', params.m_payload)); 
xlabel('Time [s]'); ylabel('Torque [Nm]');
legend('Math \tau_1', 'Sim \tau_1', 'Math \tau_2', 'Sim \tau_2', 'Math \tau_3', 'Sim \tau_3', 'Location', 'eastoutside');
xlim([0 sim_time]);

%% 6. Helper Functions
function [x, y, z] = trajectory_fcn(t, z_surface, cycle_time)
    R = 80;          
    z_lift = 30;     
    h_arch = 20;     
    t_init = 1.0;    
    
    phi0 = 0;
    xA0 = R * cos(phi0);
    yA0 = R * sin(phi0);
    
    if t < t_init
        u = t / t_init;
        s = u^3 * (10 - 15*u + 6*u^2); 
        x = s * xA0; y = s * yA0; z = z_surface;
        return;
    end
    
    t_run = t - t_init;
    cycle_idx = floor(t_run / cycle_time);
    diag_idx = mod(cycle_idx, 6);
    
    phi_pick = diag_idx * (pi / 3);
    phi_place = phi_pick + pi;
    phi_next = (diag_idx + 1) * (pi / 3);
    
    xA = R * cos(phi_pick); yA = R * sin(phi_pick);
    xB = R * cos(phi_place); yB = R * sin(phi_place);
    xNext = R * cos(phi_next); yNext = R * sin(phi_next);
    
    tau = mod(t_run, cycle_time) / cycle_time;
    total_clearance = z_lift + h_arch;
    
    if tau < 0.50
        u = tau / 0.50;
        s_xy = u^3 * (10 - 15*u + 6*u^2);
        x = xA + s_xy * (xB - xA);
        y = yA + s_xy * (yB - yA);
        z = z_surface + total_clearance * (64 * u^3 * (1 - u)^3);
    else
        u = (tau - 0.50) / 0.50;
        s_tr = u^3 * (10 - 15*u + 6*u^2);
        x = xB + s_tr * (xNext - xB);
        y = yB + s_tr * (yNext - yB);
        z = z_surface + total_clearance * (64 * u^3 * (1 - u)^3);
    end
end

function q = delta_IK(P, params)
    q = zeros(3,1);
    for i = 1:3
        phi = params.phi(i) * pi / 180;
        X =  P(1)*cos(phi) + P(2)*sin(phi);
        Y = -P(1)*sin(phi) + P(2)*cos(phi);
        Z =  P(3);
        
        Y_eff = Y - (params.f - params.e);
        A = -2 * params.L * Y_eff;
        B =  2 * params.L * Z;
        C = params.l^2 - params.L^2 - X^2 - Y_eff^2 - Z^2;
        
        disc = A^2 + B^2 - C^2;
        t_val = (B + sqrt(max(0, disc))) / (A + C);
        q(i) = 2 * atan(t_val);
    end
end

function [J_inv, J] = delta_Jacobian(q, P, params)
    J_p = zeros(3,3); J_th = zeros(3,3);
    for i = 1:3
        phi = params.phi(i) * pi / 180;
        R_z = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
        
        A_loc = [0; params.f + params.L * cos(q(i)); -params.L * sin(q(i))];
        A_glob = R_z * A_loc;
        P_loc = R_z' * P + [0; params.e; 0];
        P_glob = R_z * P_loc;
        
        L_rod = P_glob - A_glob;
        J_p(i, :) = L_rod';
        dA_dq = R_z * [0; -params.L * sin(q(i)); -params.L * cos(q(i))];
        J_th(i, i) = dot(L_rod, dA_dq);
    end
    J_inv = J_th \ J_p; J = inv(J_inv);
end