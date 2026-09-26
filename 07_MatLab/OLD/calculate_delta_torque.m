%% Validated Delta Robot Dynamics & Minimum-Jerk Torque Script (10s Wide Layout)
clear; clc; close all;

%% 1. Parameters (Hardware matched to user specs)
params.f = 125.0 * 1e-3;            % Base radius [m][cite: 19]
params.e = 45.0  * 1e-3;            % Tool plate radius [m][cite: 19]
params.L = 170.0 * 1e-3;            % Bicep length [m][cite: 19]
params.l = 400.0 * 1e-3;            % Forearm length [m][cite: 19]
params.phi_deg = [0, 120, 240];     % Motor mounting angles [deg]
params.g = [0; 0; -9.81];           % Gravity [m/s^2]

% Material Densities
params.rho_carbon_fiber = 1600; 
params.rho_aluminum = 2700;     

% Mass Calculations from Simscape dimensions
bicep_vol = (170 * 16 * 30) * 1e-9; 
m_bicep = bicep_vol * params.rho_aluminum;
I_bicep = (1/3) * m_bicep * params.L^2;

rod_radius = 6; 
rod_vol = (pi * (rod_radius/2)^2 * 400) * 1e-9; 
m_forearm = (rod_vol * params.rho_carbon_fiber) * 2; % 2 rods per arm

plate_vol = (pi * (params.e * 1000)^2 * 10) * 1e-9;
m_plate = plate_vol * params.rho_aluminum; 
m_payload = 0.0; 

% Lumped Equivalent Masses
m_elbow = 0.5 * m_forearm;
m_ee_eff = m_plate + m_payload + 3 * (0.5 * m_forearm);
I_eff_arm = I_bicep + m_elbow * (params.L^2);

% --- BELT DRIVE & INERTIA PARAMETERS ---
params.enable_belt = 1;            % 1 = Belt Drive, 0 = Direct Drive
params.R_belt = 3.0;               % Mechanical reduction ratio (e.g., 3:1)

% Rotational Inertias [kg*m^2]
params.I_rotor = 0.60e-4;          % 400W Servo motor rotor inertia
params.I_pulley_motor = 1.0e-5;    % Estimated driving pulley inertia (small)
params.I_pulley_arm = 9.0e-5;      % Estimated driven pulley inertia (large)

%% 2. Generate Smooth C2-Continuous Trajectory (10 Second Run)
cycle_time = 0.5;            % Cycle time [s]
z_surface = -350;            % Operating depth [mm]
h_arch = 60;                 % Parabola Arch Height [mm] (-350 + 60 = -290)

t_sim = 0:0.002:10.0;        % Simulation time vector [s] (Extended to 10s)
N = length(t_sim);
pos = zeros(3, N);

for k = 1:N
    [x, y, z] = trajectory_fcn(t_sim(k), z_surface, cycle_time, h_arch);
    pos(:, k) = [x; y; z] * 1e-3; % Convert mm to m
end

dt = t_sim(2) - t_sim(1);
vel = [zeros(3,1), diff(pos, 1, 2) / dt];
acc = [zeros(3,1), diff(vel, 1, 2) / dt];

%% 3. Dynamic Model (Principle of Virtual Work)
theta = zeros(3, N);
theta_dot = zeros(3, N);
tau = zeros(3, N);

for k = 1:N
    P = pos(:, k);
    V = vel(:, k);
    A = acc(:, k);
    
    q = delta_IK(P, params);
    theta(:, k) = q;
    
    [J_inv, J] = delta_Jacobian(q, P, params);
    
    q_dot = J_inv * V;
    theta_dot(:, k) = q_dot;
    
    F_ee = m_ee_eff * (A - params.g);
    tau_ee = J' * F_ee;
    
    if k == 1
        q_ddot = zeros(3,1);
    else
        q_ddot = (theta_dot(:, k) - theta_dot(:, k-1)) / dt;
    end
    
    % Base arm inertia (calculated in Section 1)
    I_arm_base = I_eff_arm; 
    
    if params.enable_belt == 1
        % Reflect motor and small pulley inertia to the joint by R^2
        I_reflected = I_arm_base + params.I_pulley_arm + ((params.I_rotor + params.I_pulley_motor) * params.R_belt^2);
        
        tau_arm_iner = I_reflected * q_ddot;
        tau_arm_g = (m_bicep * (params.L/2) + m_elbow * params.L) * 9.81 * cos(q);
        
        % Total required torque at the arm joint
        tau_joint = tau_ee + tau_arm_iner + tau_arm_g;
        
        % Torque required at the motor shaft (mechanical advantage)
        tau(:, k) = tau_joint / params.R_belt;
        
    else
        % Direct Drive (Including motor rotor inertia for accuracy)
        I_total_direct = I_arm_base + params.I_rotor; 
        
        tau_arm_iner = I_total_direct * q_ddot;
        tau_arm_g = (m_bicep * (params.L/2) + m_elbow * params.L) * 9.81 * cos(q);
        
        % Torque required directly at the motor shaft
        tau(:, k) = tau_ee + tau_arm_iner + tau_arm_g;
    end
end

%% 4. Display Results (Wide 2-Row Layout)
figure('Name', 'Delta Robot Dynamics: 2D Positions & Torque', 'Color', 'w', 'Position', [100, 100, 1400, 800]);

% Plot 1: 2D Cartesian Trajectories Over Time
subplot(2, 1, 1);
plot(t_sim, pos(1,:)*1e3, 'r-', 'LineWidth', 1.5); hold on; grid on;
plot(t_sim, pos(2,:)*1e3, 'g-', 'LineWidth', 1.5);
plot(t_sim, pos(3,:)*1e3, 'b-', 'LineWidth', 1.5);
title('Cartesian Trajectory (X, Y, Z) over Time'); 
xlabel('Time [s]'); 
ylabel('Position [mm]'); 
legend('X Position', 'Y Position', 'Z Position', 'Location', 'eastoutside');
xlim([0 10]);

% Plot 2: Required Torques Over Time
subplot(2, 1, 2);
plot(t_sim, tau(1,:), 'r-', 'LineWidth', 1.5); hold on; grid on;
plot(t_sim, tau(2,:), 'g-', 'LineWidth', 1.5);
plot(t_sim, tau(3,:), 'b-', 'LineWidth', 1.5);
title('Required Motor Torques [Nm]'); 
xlabel('Time [s]'); 
ylabel('Torque [Nm]');
legend('\tau_1', '\tau_2', '\tau_3', 'Location', 'eastoutside');
xlim([0 10]);

%% 5. Continuous Parabolic Arch Trajectory
function [x, y, z] = trajectory_fcn(t, z_surface, cycle_time, h_arch)
    R = 80;          
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
    
    if tau < 0.50
        u = tau / 0.50;
        s_xy = u^3 * (10 - 15*u + 6*u^2);
        x = xA + s_xy * (xB - xA);
        y = yA + s_xy * (yB - yA);
        z = z_surface + h_arch * (64 * u^3 * (1 - u)^3);
    else
        u = (tau - 0.50) / 0.50;
        s_tr = u^3 * (10 - 15*u + 6*u^2);
        x = xB + s_tr * (xNext - xB);
        y = yB + s_tr * (yNext - yB);
        z = z_surface + h_arch * (64 * u^3 * (1 - u)^3);
    end
end

%% 6. Kinematics & Analytical Jacobian
function q = delta_IK(P, params)
    q = zeros(3,1);
    for i = 1:3
        phi = params.phi_deg(i) * pi / 180;
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
    J_p = zeros(3,3);
    J_th = zeros(3,3);
    for i = 1:3
        phi = params.phi_deg(i) * pi / 180;
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
    J_inv = J_th \ J_p;
    J = inv(J_inv);
end