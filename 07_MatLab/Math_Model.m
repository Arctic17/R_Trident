%% Math_Model.m
% Calculates theoretical Virtual Work torques and trajectories.
% Inherits 'params', 'cycle_time', 'z_surface', 'h_arch', 'sim_time', 'enable_vertical_drops', and 'trajectory_type' from Master_Run.

% 1. Convert geometric dimensions to meters for physics calculations
m_params.L   = params.bicep_length * 1e-3;
m_params.l   = params.forearm_length * 1e-3;
m_params.f   = params.f * 1e-3;
m_params.e   = params.e * 1e-3;
m_params.phi = params.phi;

% 2. Calculate Equivalent Masses & Inertias (All in SI units: kg, m, kg*m^2)
m_elbow = 0.5 * params.forearm_mass;
plate_vol = (pi * (params.plate_radius)^2 * params.plate_thickness) * 1e-9; % m^3
m_plate   = plate_vol * params.rho_aluminum; 
m_ee_eff  = m_plate + params.m_payload + 3 * m_elbow;
I_eff_arm = (params.bicep_mass * (m_params.L^2) / 3) + m_elbow * (m_params.L^2);

% 3. Generate Trajectory Profile
t_sim = 0:0.002:params.sim_time;
N = length(t_sim);
pos_math = zeros(3, N);

m_params.z_surface = params.z_surface;
m_params.cycle_time = params.cycle_time;
m_params.h_arch = params.h_arch;
m_params.enable_vertical_drops = params.enable_vertical_drops;
m_params.r_arch = params.r_arch;
m_params.z_lift = params.z_lift;
m_params.time_drop = params.time_drop;
% Add trajectory selection toggle (Default to 1 if not set in Master_Run)
if isfield(params, 'trajectory_type')
    m_params.trajectory_type = params.trajectory_type;
else
    m_params.trajectory_type = 1; % 1 = Default Star/Circular, 2 = Butterfly
end

for k = 1:N
    [x, y, z] = math_trajectory_fcn(t_sim(k), m_params.z_surface, m_params.cycle_time, m_params.h_arch, m_params.enable_vertical_drops, m_params.r_arch, m_params.z_lift, m_params.time_drop, m_params.trajectory_type);
    pos_math(:, k) = [x; y; z] * 1e-3; % Convert mm to m
end

dt = t_sim(2) - t_sim(1);
vel_math = [zeros(3,1), diff(pos_math, 1, 2) / dt];
acc_math = [zeros(3,1), diff(vel_math, 1, 2) / dt];

% 4. Dynamic Virtual Work Engine
tau_math  = zeros(3, N);
theta_dot = zeros(3, N);

for k = 1:N
    P = pos_math(:, k);
    V = vel_math(:, k);
    A = acc_math(:, k);
    
    q = math_delta_IK(P, m_params);
    [J_inv, J] = math_delta_Jacobian(q, P, m_params);
    
    q_dot = J_inv * V;
    theta_dot(:, k) = q_dot;
    
    % End-effector inertial and gravity forces
    F_ee = m_ee_eff * (A - [0; 0; -9.81]);
    tau_ee = J' * F_ee;
    
    if k == 1
        q_ddot = zeros(3,1);
    else
        q_ddot = (theta_dot(:, k) - theta_dot(:, k-1)) / dt;
    end
    
    % Joint-space dynamics 
    tau_arm_g = (params.bicep_mass * (m_params.L/2) + m_elbow * m_params.L) * 9.81 * cos(q);
    
    if isfield(params, 'enable_belt') && params.enable_belt == 1
        I_reflected = I_eff_arm + params.I_pulley_arm + ((params.I_rotor + params.I_pulley_motor) * params.R_belt^2);
        tau_arm_iner = I_reflected * q_ddot;
        tau_joint = tau_ee + tau_arm_iner + tau_arm_g;
        tau_math(:, k) = tau_joint / params.R_belt;
    else
        I_total = I_eff_arm + params.I_rotor;
        tau_arm_iner = I_total * q_ddot;
        tau_math(:, k) = tau_ee + tau_arm_iner + tau_arm_g;
    end
end

% --- HELPER FUNCTIONS ---
function [x, y, z] = math_trajectory_fcn(t, z_surface, cycle_time, h_arch, enable_vertical_drops, r_arch, z_lift, time_drop, trajectory_type)
    t_init = 1.0; 
    
    if trajectory_type == 1
        % --- ORIGINAL STAR MODE ---
        R = r_arch;          
        phi0 = 0; xA0 = R * cos(phi0); yA0 = R * sin(phi0);
        
        if t < t_init
            u = t / t_init; s = u^3 * (10 - 15*u + 6*u^2); 
            x = s * xA0; y = s * yA0; z = z_surface; return;
        end
        
        t_run = t - t_init;
        T_current = cycle_time; t_accum = 0;            
        
        for p = 1:100 
            T_pattern = 6 * T_current;
            if t_run < (t_accum + T_pattern); break; end
            t_accum = t_accum + T_pattern;
            T_current = max(0.05, T_current - time_drop); 
        end
        
        t_local = t_run - t_accum;
        raw_cycles = (t_local / T_current) + 1e-10;
        cycle_idx = floor(raw_cycles);
        diag_idx = mod(cycle_idx, 6);
        tau = raw_cycles - cycle_idx;
        
        phi_pick = diag_idx * (pi / 3); phi_place = phi_pick + pi; phi_next = (diag_idx + 1) * (pi / 3);
        xA = R * cos(phi_pick); yA = R * sin(phi_pick);
        xB = R * cos(phi_place); yB = R * sin(phi_place);
        xNext = R * cos(phi_next); yNext = R * sin(phi_next);
        
        if enable_vertical_drops == 1
            if tau < 0.15
                u = tau / 0.15; s = u^3 * (10 - 15*u + 6*u^2);
                x = xA; y = yA; z = z_surface + s * z_lift;
            elseif tau < 0.65
                u = (tau - 0.15) / 0.50; s_xy = u^3 * (10 - 15*u + 6*u^2);
                x = xA + s_xy * (xB - xA); y = yA + s_xy * (yB - yA);
                z = z_surface + z_lift + h_arch * (sin(pi * u))^2;
            elseif tau < 0.80
                u = (tau - 0.65) / 0.15; s = u^3 * (10 - 15*u + 6*u^2);
                x = xB; y = yB; z = (z_surface + z_lift) - s * z_lift;
            else
                u = (tau - 0.80) / 0.20; s_tr = u^3 * (10 - 15*u + 6*u^2);
                x = xB + s_tr * (xNext - xB); y = yB + s_tr * (yNext - yB);
                z = z_surface + (z_lift + 10) * (sin(pi * u))^2;
            end
        else
            total_clearance = z_lift + h_arch;
            if tau < 0.50
                u = tau / 0.50; s_xy = u^3 * (10 - 15*u + 6*u^2);
                x = xA + s_xy * (xB - xA); y = yA + s_xy * (yB - yA);
                z = z_surface + total_clearance * (sin(pi * u))^2;
            else
                u = (tau - 0.50) / 0.50; s_tr = u^3 * (10 - 15*u + 6*u^2);
                x = xB + s_tr * (xNext - xB); y = yB + s_tr * (yNext - yB);
                z = z_surface + total_clearance * (sin(pi * u))^2;
            end
        end
        
    elseif trajectory_type == 2
        % --- BUTTERFLY MODE ---
        W = 200; 
        H_rect = 80; 
        H_hop = h_arch; % <--- NO LONGER HARDCODED TO 90
        
        P1 = [-W/2,  H_rect/2]; P2 = [ W/2,  H_rect/2];
        P3 = [-W/2, -H_rect/2]; P4 = [ W/2, -H_rect/2];
        points = [P1; P2; P3; P4; P1];
        
        if t < t_init
            u = t / t_init; s = u^3 * (10 - 15*u + 6*u^2); 
            x = s * P1(1); y = s * P1(2); z = z_surface; return;
        end
        
        t_run = t - t_init;
        T_current = cycle_time; t_accum = 0;            
        
        for p = 1:100 
            T_pattern = 4 * T_current;
            if t_run < (t_accum + T_pattern); break; end
            t_accum = t_accum + T_pattern;
            T_current = max(0.05, T_current - time_drop); 
        end
        
        t_local = t_run - t_accum;
        
        raw_cycles = (t_local / T_current) + 1e-10;
        cycle_idx = floor(raw_cycles);
        seg_idx = mod(cycle_idx, 4) + 1;
        tau = raw_cycles - cycle_idx;
        
        if seg_idx < 1; seg_idx = 1; end
        if seg_idx > 4; seg_idx = 4; end
        
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        
        p_start = points(seg_idx, :); p_end = points(seg_idx + 1, :);
        
        x = p_start(1) + (p_end(1) - p_start(1)) * s;
        y = p_start(2) + (p_end(2) - p_start(2)) * s;
        z = z_surface + H_hop * (0.5 * (1 - cos(2 * pi * tau)));
    end
end

function q = math_delta_IK(P, m_params)
    q = zeros(3,1);
    for i = 1:3
        phi_ang = m_params.phi(i) * pi / 180;
        X =  P(1)*cos(phi_ang) + P(2)*sin(phi_ang);
        Y = -P(1)*sin(phi_ang) + P(2)*cos(phi_ang);
        Z =  P(3);
        
        Y_eff = Y - (m_params.f - m_params.e);
        A = -2 * m_params.L * Y_eff;
        B =  2 * m_params.L * Z;
        C = m_params.l^2 - m_params.L^2 - X^2 - Y_eff^2 - Z^2;
        
        disc = A^2 + B^2 - C^2;
        t_val = (B + sqrt(max(0, disc))) / (A + C);
        q(i) = 2 * atan(t_val);
    end
end

function [J_inv, J] = math_delta_Jacobian(q, P, m_params)
    J_p = zeros(3,3); J_th = zeros(3,3);
    for i = 1:3
        phi_ang = m_params.phi(i) * pi / 180;
        R_z = [cos(phi_ang), -sin(phi_ang), 0; sin(phi_ang), cos(phi_ang), 0; 0, 0, 1];
        
        A_loc = [0; m_params.f + m_params.L * cos(q(i)); -m_params.L * sin(q(i))];
        A_glob = R_z * A_loc;
        P_loc = R_z' * P + [0; m_params.e; 0];
        P_glob = R_z * P_loc;
        
        L_rod = P_glob - A_glob;
        J_p(i, :) = L_rod';
        dA_dq = R_z * [0; -m_params.L * sin(q(i)); -m_params.L * cos(q(i))];
        J_th(i, i) = dot(L_rod, dA_dq);
    end
    J_inv = J_th \ J_p;
    J = inv(J_inv);
end