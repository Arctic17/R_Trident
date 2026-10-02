%% Master_Run.m - Single Simulation Deep-Dive & Justification Suite
% Executes a single high-fidelity run with all combination variables,
% dense data logging, and full kinematics/dynamics/3D trajectory plots.
clear; clc; close all;

%% 1. Load Baseline Parameters
disp('Loading Base Configuration...');
run('BaseSkript.m');

%% 2. Output Directory Setup
base_out_folder = fullfile('MotorValidation', 'Single_Run_Results');
if ~exist(base_out_folder, 'dir')
    mkdir(base_out_folder);
end

%% 3. Motor Catalog & Active Configuration Overrides
% Motor Database: [I_rotor (kg*m^2), Tau_Continuous (Nm), Tau_Peak (Nm)]
motor_labels = {
    'EC 90 flat Open Rotor 48V', ...
    'EC 60 flat Ventilated 48V', ...
    'EC-i 52 High Torque 48V', 'EC 60 brushless 48V', 'ec90 160'
}; 
motor_opts = [
    31.65e-5, 1.220, 9.57;   % Motor 1
    8.32e-5,  0.581, 2.65;   % Motor 2
    2.64e-5,  0.627, 7.83;    % Motor 3
    8.31e-5, 0.460, 4.820;
    8.21e-5, 0.441, 2.650
];

% =========================================================================
% DESIGN SELECTION CONFIGURATION
% =========================================================================
m_idx                 = 1;     % Motor Index: 1 (EC 90), 2 (EC 60), 3 (EC-i 52)
params.bicep_length   = 160;   % [mm] Swept options: 200, 180, 160
params.forearm_length = 380;   % [mm] Swept options: 420, 400, 380, 360, 340
params.R_belt         = 2.0;   % Reduction ratio: 1.0 (Direct Drive), 2.0, 3.0
params.enable_belt    = (params.R_belt > 1.0);

% Build clean configuration and display tags
config_tag = sprintf('L1-%d_L2-%d_R-%.1f_%s', ...
    params.bicep_length, params.forearm_length, params.R_belt, motor_labels{m_idx});

%% 4. Dynamic Geometric, Mass & Transmission Calculations
% Dynamic mass recalculation based on geometry and materials
params.rod_diameter = 5.0; % [mm]
bicep_vol = (params.bicep_length * params.bicep_width * params.bicep_height) * 1e-9;
params.bicep_mass = bicep_vol * params.rho_aluminum;
rod_vol = (pi * (params.rod_diameter / 2)^2 * params.forearm_length) * 1e-9;
params.forearm_mass = (rod_vol * params.rho_carbon_fiber) * 2; % 2 carbon rods per arm

% Optimal Z-surface heuristic
params.z_surface = -0.85 * params.forearm_length;

% Dynamic PID scaling derivative filter coefficient
Nd = 100;

% Motor inertia & torque boundaries
params.I_rotor = motor_opts(m_idx, 1);
tau_cont_limit = motor_opts(m_idx, 2); 
tau_peak_limit = motor_opts(m_idx, 3); 

% Dynamic transmission inertias (T2.5, 10mm Aluminum Pulleys)
if params.R_belt == 1.0
    params.I_pulley_motor = 0.0;     
    params.I_pulley_arm   = 0.0;
elseif params.R_belt == 2.0
    params.I_pulley_motor = 1.0e-7;  
    params.I_pulley_arm   = 4.5e-5;  
else 
    params.I_pulley_motor = 1.2e-7;  
    params.I_pulley_arm   = 9.0e-5;  
end

%% 5. Trajectory & High-Fidelity Simulation Settings
params.m_payload             = 0.0;            
params.cycle_time            = 0.5;                  
params.sim_time              = 13;                    
params.z_lift                = 0;
params.h_arch                = 50;  % Updated for new trajectory jump
params.r_arch                = 200; % Updated reference radius/width
params.enable_vertical_drops = 0;
params.time_drop             = 0.05;

% --- NEW TRAJECTORY TOGGLE ---
% 1 = Original Star/Circular Pick-and-Place
% 2 = Butterfly Cross Path (200x80mm, 90mm Z-hop)
params.trajectory_type       = 2;  
% -----------------------------

err_tolerance                = 10.0; % Max allowed tracking error [mm]

% Push updated variables to base workspace for Simulink access
assignin('base', 'params', params);
assignin('base', 'Nd', Nd);

% Define Subfolder Structure: 07_MatLab\Single_Run_Results\R[r_arch]_[config_tag]
subfolder_name = sprintf('R%d_%s', round(params.r_arch), config_tag);
run_dir = fullfile(base_out_folder, subfolder_name);
if ~exist(run_dir, 'dir')
    mkdir(run_dir);
end

% Standardized file prefix for all exports: R[radius]_[config_tag]_...
file_prefix = sprintf('R%d_%s', round(params.r_arch), config_tag);

% Initialize logging buffer
log_lines = {};
log_lines{end+1} = '==================================================';
log_lines{end+1} = sprintf('Selected Configuration: %s', config_tag);
log_lines{end+1} = sprintf('Trajectory Mode: %d', params.trajectory_type);
log_lines{end+1} = '==================================================';

fprintf('\n%s\n', log_lines{end-3});
fprintf('%s\n', log_lines{end-2});
fprintf('%s\n', log_lines{end-1});
fprintf('%s\n', log_lines{end});

%% 6. Execute Analytical Math Model
log_lines{end+1} = 'Running Analytical Math Model...';
fprintf('%s\n', log_lines{end});
run('Math_Model.m'); % Computes pos_math, vel_math, acc_math, tau_math, t_sim

%% 7. Execute 3D Simscape Physics (High Sampling Density)
log_lines{end+1} = 'Executing 3D Simscape Model...';
fprintf('%s\n', log_lines{end});
try
    out = sim('SimulinkModel', 'StopTime', num2str(params.sim_time), 'MaxStep', '0.001');
    log_lines{end+1} = 'Simscape Simulation Complete.';
    fprintf('%s\n', log_lines{end});
catch ME
    error('Simscape execution failed: %s', ME.message);
end

%% 8. Data Extraction, Matrix Squeeze & Resampling
t_tau_raw = out.tau_motor.Time; 
tau_raw   = out.tau_motor.Data;
t_pos_raw = out.pos_cmd.Time;   
pos_c_raw = out.pos_cmd.Data; 
pos_r_raw = out.EndEff_Pos.Data;

if ndims(tau_raw) == 3;   tau_raw = squeeze(tau_raw)';     end
if ndims(pos_c_raw) == 3; pos_c_raw = squeeze(pos_c_raw)'; end
if ndims(pos_r_raw) == 3; pos_r_raw = squeeze(pos_r_raw)'; end

if max(abs(pos_r_raw(:))) < 2.0; pos_r_raw = pos_r_raw * 1000; end
if max(abs(pos_c_raw(:))) < 2.0; pos_c_raw = pos_c_raw * 1000; end

tau_sim_interp = interp1(t_tau_raw, tau_raw, t_sim, 'linear', 'extrap');
pos_c_interp   = interp1(t_pos_raw, pos_c_raw, t_sim, 'linear', 'extrap');
pos_r_interp   = interp1(t_pos_raw, pos_r_raw, t_sim, 'linear', 'extrap');

delta_pos = pos_r_interp - pos_c_interp;

%% 9. Speed Tier Evaluation & Segment Boundary Tracking
t_accum = 1.0; 
T_current = params.cycle_time;
best_cycle_time = NaN;
best_err = NaN;
segment_boundaries = t_accum; 

log_lines{end+1} = '';
log_lines{end+1} = 'Evaluating Speed Tiers across simulation timeline:';
fprintf('\n%s\n', log_lines{end});

for p = 1:15
    % Adjust pattern check to match new 4-segment butterfly loop
    if params.trajectory_type == 2
        T_pattern = 4 * T_current; % 4 segments in the new path
    else
        T_pattern = 6 * T_current; % 6 segments in the original star path
    end
    
    t_start = t_accum;
    t_end = min(t_accum + T_pattern, params.sim_time);
    
    window_idx = (t_sim >= t_start) & (t_sim < t_end);
    if ~any(window_idx); break; end
    
    settle_idx = (t_sim >= (t_start + 0.1)) & (t_sim < t_end);
    if ~any(settle_idx); settle_idx = window_idx; end 
    
    err_window = max(vecnorm((pos_r_interp(settle_idx, :) - pos_c_interp(settle_idx, :))'));
    tau_window = tau_sim_interp(settle_idx, :);
    
    max_tau_peak = max(abs(tau_window(:)));
    max_rms_tau  = max(rms(tau_window));
    
    is_failed = (max_tau_peak > tau_peak_limit) || (max_rms_tau > tau_cont_limit) || (err_window > err_tolerance);
    
    if is_failed
        tier_str = sprintf('  Tier %2d (CT = %.2fs): FAILED -> Peak: %5.2f Nm (lim %.2f), RMS: %4.2f Nm (lim %.2f), Err: %5.2f mm (lim %.1f)', ...
            p, T_current, max_tau_peak, tau_peak_limit, max_rms_tau, tau_cont_limit, err_window, err_tolerance);
        log_lines{end+1} = tier_str;
        fprintf('%s\n', tier_str);
        segment_boundaries(end+1) = t_end; 
        break; 
    else
        tier_str = sprintf('  Tier %2d (CT = %.2fs): PASSED -> Peak: %5.2f Nm, RMS: %4.2f Nm, Err: %5.2f mm', ...
            p, T_current, max_tau_peak, max_rms_tau, err_window);
        log_lines{end+1} = tier_str;
        fprintf('%s\n', tier_str);
        best_cycle_time = T_current; 
        best_err = err_window;
    end
    
    t_accum = t_accum + T_pattern;
    segment_boundaries(end+1) = t_accum; 
    T_current = max(0.05, T_current - params.time_drop);
    if t_accum >= params.sim_time; break; end
end

summary_str = sprintf('\nEvaluation Summary: Fastest Successful CT = %.2f s | Max Error = %.2f mm', best_cycle_time, best_err);
log_lines{end+1} = summary_str;
fprintf('%s\n', summary_str);
% --- WORKSPACE CALCULATION ---
[max_r, opt_z] = Calculate_Workspace(params);
fprintf('    --> Theoretical Max Reach: %.1f mm (Radius) at Z = %.1f mm\n', max_r, opt_z);

% Write formatted log to .txt inside the subfolder
txt_filename = fullfile(run_dir, sprintf('%s_Log.txt', file_prefix));
fid = fopen(txt_filename, 'w');
if fid ~= -1
    for l = 1:length(log_lines)
        fprintf(fid, '%s\n', log_lines{l});
    end
    fclose(fid);
end

%% 10. PLOT 1: Comprehensive Kinematics (Position, Following Error, Acceleration)
c_light = [1.0 0.4 0.4; 0.4 1.0 0.4; 0.4 0.6 1.0]; 
c_dark  = [0.6 0.0 0.0; 0.0 0.6 0.0; 0.0 0.0 0.6]; 

fig_kin = figure('Name', 'Kinematics: Path, Error & Acceleration', 'Color', 'w', ...
    'Position', [50, 50, 1400, 950]); 

subplot(3, 1, 1);
for j = 1:3
    plot(t_sim, pos_c_interp(:, j), '-',  'Color', c_light(j, :), 'LineWidth', 1); hold on;
    plot(t_sim, pos_r_interp(:, j), '--', 'Color', c_dark(j, :),  'LineWidth', 1.5);
end
grid on; title(sprintf('Cartesian Path Tracking [%s]', config_tag), 'Interpreter', 'none'); 
ylabel('Position [mm]'); 
legend('X Cmd', 'X Real', 'Y Cmd', 'Y Real', 'Z Cmd', 'Z Real', 'Location', 'eastoutside');
xlim([0, params.sim_time]);

subplot(3, 1, 2);
for j = 1:3
    plot(t_sim, delta_pos(:, j), 'Color', c_dark(j, :), 'LineWidth', 1.5); hold on;
end
grid on; title('Cartesian Following Error (Real - Commanded)'); 
ylabel('\Delta Error [mm]'); 
legend('\Delta X', '\Delta Y', '\Delta Z', 'Location', 'eastoutside');
xlim([0, params.sim_time]);
ylim([-20, 20]);

subplot(3, 1, 3);
for j = 1:3
    plot(t_sim, acc_math(j, :), '-', 'Color', c_light(j, :), 'LineWidth', 1.5); hold on;
end
grid on; title('Theoretical Cartesian Acceleration (XYZ)'); 
ylabel('Acc [m/s^2]'); xlabel('Time [s]');
legend('Acc X', 'Acc Y', 'Acc Z', 'Location', 'eastoutside');
xlim([0, params.sim_time]);
ylim([-80, 80]);

for sp = 1:3
    subplot(3, 1, sp); hold on;
    for sb = 1:length(segment_boundaries)
        xline(segment_boundaries(sb), 'k--', 'LineWidth', 1, 'Alpha', 0.6, 'HandleVisibility', 'off');
    end
end

% Save Kinematics in SVG and PNG inside the subfolder
print(fig_kin, fullfile(run_dir, sprintf('%s_Kinematics.png', file_prefix)), '-dpng', '-r200');
print(fig_kin, fullfile(run_dir, sprintf('%s_Kinematics.svg', file_prefix)), '-dsvg');

%% 11. PLOT 2: Dynamics Validation & Motor Limits
fig_dyn = figure('Name', 'Dynamics: Torque Validation & Limits', 'Color', 'w', ...
    'Position', [100, 100, 1300, 900]);
motor_names = {'Motor 1', 'Motor 2', 'Motor 3'};

for j = 1:3
    subplot(3, 1, j);
    plot(t_sim, tau_math(j, :), '-',  'Color', c_light(j, :), 'LineWidth', 1); hold on; grid on;
    plot(t_sim, tau_sim_interp(:, j), '-', 'Color', c_dark(j, :),  'LineWidth', 1.5);
    
    yline(tau_cont_limit,  'r--', 'Continuous Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    yline(-tau_cont_limit, 'r--', 'HandleVisibility', 'off');
    yline(tau_peak_limit,  'r-',  'Stall Limit',      'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    yline(-tau_peak_limit, 'r-',  'HandleVisibility', 'off');
    
    title(sprintf('%s Dynamic Torque [%s]', motor_names{j}, config_tag), 'Interpreter', 'none');
    ylabel('Torque [Nm]');
    legend('Theoretical (Math)', 'Simscape (Real)', 'Location', 'eastoutside');
    xlim([0, params.sim_time]);
    ylim([-max(10, tau_peak_limit * 1.15), max(10, tau_peak_limit * 1.15)]);
    
    for sb = 1:length(segment_boundaries)
        xline(segment_boundaries(sb), 'k--', 'LineWidth', 1, 'Alpha', 0.6, 'HandleVisibility', 'off');
    end
    if j == 3; xlabel('Time [s]'); end
end

print(fig_dyn, fullfile(run_dir, sprintf('%s_Dynamics.png', file_prefix)), '-dpng', '-r200');
print(fig_dyn, fullfile(run_dir, sprintf('%s_Dynamics.svg', file_prefix)), '-dsvg');

%% 11.1. PLOT 2 (Single Motor): Dynamic Torque Validation & Motor Limits
target_motor = 1;

fig_dyn_single = figure('Name', sprintf('Dynamics: Motor %d Torque Validation', target_motor), ...
    'Color', 'w', 'Position', [100, 100, 1200, 550]);

plot(t_sim, tau_math(target_motor, :), '-',  'Color', c_light(target_motor, :), 'LineWidth', 1); hold on; grid on;
plot(t_sim, tau_sim_interp(:, target_motor), '-', 'Color', c_dark(target_motor, :),  'LineWidth', 1.5);

yline(tau_cont_limit,  'r--', 'Continuous Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-tau_cont_limit, 'r--', 'HandleVisibility', 'off');
yline(tau_peak_limit,  'r-',  'Stall Limit',      'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-tau_peak_limit, 'r-',  'HandleVisibility', 'off');

for sb = 1:length(segment_boundaries)
    xline(segment_boundaries(sb), 'k--', 'LineWidth', 1, 'Alpha', 0.6, 'HandleVisibility', 'off');
end

title(sprintf('%s Dynamic Torque Validation [%s]', motor_names{target_motor}, config_tag), 'Interpreter', 'none');
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend('Theoretical (Math)', 'Simscape (Real)', 'Location', 'eastoutside');
xlim([0, params.sim_time]);
ylim([-2, 2]);

print(fig_dyn_single, fullfile(run_dir, sprintf('%s_Dynamics_Motor%d.png', file_prefix, target_motor)), '-dpng', '-r200');
print(fig_dyn_single, fullfile(run_dir, sprintf('%s_Dynamics_Motor%d.svg', file_prefix, target_motor)), '-dsvg');

%% 12. PLOT 3: 3D End-Effector Trajectory Comparison
fig_3d = figure('Name', '3D Trajectory Comparison', 'Color', 'w', ...
    'Position', [150, 150, 1050, 800]);
hold on; grid on; view(3);

if max(abs(pos_math(:))) < 2.0
    p_math_m = pos_math;
else
    p_math_m = pos_math * 1e-3;
end
p_cmd_m  = pos_c_interp * 1e-3;
p_real_m = pos_r_interp * 1e-3;

plot3(p_math_m(1, :), p_math_m(2, :), p_math_m(3, :), ...
    'b-', 'LineWidth', 1.5, 'DisplayName', 'Theoretical (Math)');
plot3(p_cmd_m(:, 1), p_cmd_m(:, 2), p_cmd_m(:, 3), ...
    'g-',  'LineWidth', 1.5, 'DisplayName', 'Commanded (Simulink)');
plot3(p_real_m(:, 1), p_real_m(:, 2), p_real_m(:, 3), ...
    'r-',  'LineWidth', 1.5, 'DisplayName', 'Real (Simscape)');

xlabel('X Position [m]');
ylabel('Y Position [m]');
zlabel('Z Position [m]');
title(sprintf('End-Effector 3D Trajectory Comparison [%s]', config_tag), 'Interpreter', 'none');
legend('Location', 'best');
axis equal;       
rotate3d on;      

print(fig_3d, fullfile(run_dir, sprintf('%s_Trajectory3D.png', file_prefix)), '-dpng', '-r200');
print(fig_3d, fullfile(run_dir, sprintf('%s_Trajectory3D.svg', file_prefix)), '-dsvg');

fprintf('\nAll assets (SVGs, PNGs, and Log TXT) successfully saved to:\n%s\n', run_dir);