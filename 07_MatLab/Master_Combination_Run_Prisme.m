%% Master_Combination_Run.m (PARALLEL OPTIMIZATION SUITE)
clear; clc; close all;
disp('Loading Base Configuration...');
run('BaseSkript.m');

%% 1. Optimization Setup & Root Folder Generation
root_out_folder = fullfile('07_MatLab', 'Optimization_Results');
if ~exist(root_out_folder, 'dir')
    mkdir(root_out_folder);
end

% Start the Parallel Pool if it isn't running
if isempty(gcp('nocreate'))
    disp('Starting Parallel Pool... This takes a few seconds.');
    parpool; 
end

%% 2. Define Parameter Sweep Arrays
L_bicep_opts   = [160, 180]; 
L_forearm_opts = [360, 380, 400]; 
ratio_opts     = [1.0 2.0, 3.0]; 

motor_labels = {
    'EC 90 flat Open Rotor 48V', ...
    'EC-i 52 High Torque 48V', ...
    'EC 60 brushless 48V'
}; 
motor_opts = [
    47.65e-5, 1.220, 9.57;   
    2.64e-5,  0.627, 7.83;   
    8.31e-5,  0.843, 6.820   
];

[B, F, R, M] = ndgrid(L_bicep_opts, L_forearm_opts, ratio_opts, 1:size(motor_opts,1));
num_tests = numel(B);

results = zeros(num_tests, 5); 
config_strings = cell(num_tests, 1);
math_data = cell(num_tests, 1); % To hold theoretical math data for plots later
err_tolerance = 10; 

%% 3. PHASE 1: PREPARE PARALLEL INPUTS (Very Fast)
disp(['Preparing ', num2str(num_tests), ' Simulation Inputs...']);

% Pre-allocate the simulation input array
simInputs(num_tests) = Simulink.SimulationInput('SimulinkModel');

for i = 1:num_tests
    params.R_belt = R(i);
    params.enable_belt = (params.R_belt > 1.0); 
    params.bicep_length   = B(i);
    params.forearm_length = F(i);
    m_idx = M(i);
    
    raw_tag = sprintf('L1-%d_L2-%d_R-%.1f_%s', B(i), F(i), R(i), motor_labels{m_idx});
    config_strings{i} = raw_tag;
    
    % Geometry & Mass
    params.rod_diameter = 5.0; 
    bicep_vol = (params.bicep_length * params.bicep_width * params.bicep_height) * 1e-9;
    params.bicep_mass = bicep_vol * params.rho_aluminum;
    rod_vol = (pi * (params.rod_diameter/2)^2 * params.forearm_length) * 1e-9;
    params.forearm_mass = (rod_vol * params.rho_carbon_fiber) * 2; 

    params.z_surface = -0.85 * params.forearm_length;
    Nd = 100; 
    
    % Motors
    params.I_rotor = motor_opts(m_idx, 1);
    if params.R_belt == 1.0
        params.I_pulley_motor = 0.0;     params.I_pulley_arm = 0.0;
    elseif params.R_belt == 2.0
        params.I_pulley_motor = 1.0e-5;  params.I_pulley_arm = 4.5e-5;  
    else 
        params.I_pulley_motor = 1.2e-5;  params.I_pulley_arm = 9.0e-5;  
    end

    % Trajectory
    params.m_payload = 0.0;            
    params.cycle_time = 0.7;                  
    params.sim_time = 15.0;                    
    params.z_lift = 0;
    params.h_arch = 50;
    params.y_span = 50;  
    params.r_arch = 200;
    params.trajectory_type = 2; 
    params.enable_vertical_drops = 0;
    params.time_drop = 0.1;
    
    % RUN MATH MODEL LOCALLY & SAVE RESULTS
    try
        run('Math_Model.m'); 
        math_data{i}.tau_math = tau_math;
        math_data{i}.acc_math = acc_math;
        math_data{i}.t_sim = t_sim;
    catch
        math_data{i} = []; % Failsafe
    end

    % CONFIGURE THE PARALLEL WORKER PACKAGE
    simInputs(i) = simInputs(i).setVariable('params', params);
    simInputs(i) = simInputs(i).setVariable('Nd', Nd);
    
    % Set to Accelerator Mode for extra speed
    simInputs(i) = simInputs(i).setModelParameter('SimulationMode', 'accelerator');
    simInputs(i) = simInputs(i).setModelParameter('Solver', 'ode23t');
    simInputs(i) = simInputs(i).setModelParameter('StopTime', num2str(params.sim_time));
end

%% 4. PHASE 2: EXECUTE PARALLEL SIMULATIONS (Heavy Lifting)
disp('==================================================');
disp('🚀 Launching parsim! Opening Simulation Manager...');
disp('==================================================');

% This command blasts the array to all your CPU cores.
% 'TransferBaseWorkspaceVariables', 'off' keeps memory clean.
simOut = parsim(simInputs, 'ShowProgress', 'on', 'ShowSimulationManager', 'on', 'TransferBaseWorkspaceVariables', 'off');

%% 5. PHASE 3: POST-PROCESSING & PLOTTING
disp('Simulations Finished. Processing data and saving graphs...');

for i = 1:num_tests
    raw_tag = config_strings{i};
    safe_tag = strrep(strrep(raw_tag, ' ', '_'), ':', '-');
    
    run_dir = fullfile(root_out_folder, safe_tag);
    if ~exist(run_dir, 'dir'); mkdir(run_dir); end
    
    % Check if simulation crashed or math model failed
    if ~isempty(simOut(i).ErrorMessage) || isempty(math_data{i})
        fprintf('Run %d FAILED/CRASHED.\n', i);
        results(i, :) = [B(i), F(i), R(i), NaN, NaN];
        continue;
    end
    
    % Extract Math & Simscape Data
    t_sim = math_data{i}.t_sim;
    tau_math = math_data{i}.tau_math;
    acc_math = math_data{i}.acc_math;
    
    t_tau_raw = simOut(i).tau_motor.Time; tau_raw = simOut(i).tau_motor.Data;
    t_pos_raw = simOut(i).pos_cmd.Time; pos_c_raw = simOut(i).pos_cmd.Data; pos_r_raw = simOut(i).EndEff_Pos.Data;

    if ndims(tau_raw) == 3;   tau_raw = squeeze(tau_raw)';     end
    if ndims(pos_c_raw) == 3; pos_c_raw = squeeze(pos_c_raw)'; end
    if ndims(pos_r_raw) == 3; pos_r_raw = squeeze(pos_r_raw)'; end

    tau_sim_interp = interp1(t_tau_raw, tau_raw, t_sim, 'linear', 'extrap');
    pos_c_interp   = interp1(t_pos_raw, pos_c_raw, t_sim, 'linear', 'extrap');
    pos_r_interp   = interp1(t_pos_raw, pos_r_raw, t_sim, 'linear', 'extrap');
    delta_pos_full = pos_r_interp - pos_c_interp;
    
    % Limits
    m_idx = M(i);
    tau_cont_limit = motor_opts(m_idx, 2); 
    tau_peak_limit = motor_opts(m_idx, 3);
    
    % Evaluate Windows
    t_accum = 1.0; 
    T_current = 0.7; % params.cycle_time
    best_cycle_time = NaN; best_err = NaN; segment_boundaries = t_accum; 

    for p = 1:15  
        T_pattern = 4 * T_current; % Trajectory Type 2
        t_start = t_accum; t_end = min(t_accum + T_pattern, 15.0);
        
        window_idx = (t_sim >= t_start) & (t_sim < t_end);
        if ~any(window_idx); break; end
        
        settle_idx = (t_sim >= (t_start + 0.1)) & (t_sim < t_end);
        if ~any(settle_idx); settle_idx = window_idx; end 
        
        err_window = max(vecnorm((pos_r_interp(settle_idx, :) - pos_c_interp(settle_idx, :))'));
        tau_window = tau_sim_interp(settle_idx, :);
        max_tau_peak = max(abs(tau_window(:))); max_rms_tau = max(rms(tau_window));
        
        if max_tau_peak > tau_peak_limit || max_rms_tau > tau_cont_limit || err_window > err_tolerance 
            segment_boundaries(end+1) = t_end; break; 
        else
            best_cycle_time = T_current; best_err = err_window;
        end
        
        t_accum = t_accum + T_pattern; segment_boundaries(end+1) = t_accum; 
        T_current = max(0.05, T_current - 0.1); % params.time_drop
        if t_accum >= 15.0; break; end
    end
    results(i, :) = [B(i), F(i), R(i), best_cycle_time, best_err];

    % --- PLOT GENERATION ---
    c_light = [1.0 0.4 0.4; 0.4 1.0 0.4; 0.4 0.6 1.0]; 
    c_dark  = [0.6 0.0 0.0; 0.0 0.6 0.0; 0.0 0.0 0.6]; 
    
    fig_kin = figure('Visible', 'off', 'Position', [100, 100, 1400, 900]); 
    subplot(3, 1, 1);
    for j = 1:3
        plot(t_sim, pos_c_interp(:,j), '-', 'Color', c_light(j,:), 'LineWidth', 1); hold on;
        plot(t_sim, pos_r_interp(:,j), '--', 'Color', c_dark(j,:), 'LineWidth', 1.5);
    end
    grid on; title(sprintf('Cartesian Path [%s]', raw_tag), 'Interpreter', 'none'); 
    ylabel('Position [mm]'); 
    
    subplot(3, 1, 2);
    for j = 1:3; plot(t_sim, delta_pos_full(:,j), 'Color', c_dark(j,:), 'LineWidth', 1.5); hold on; end
    grid on; title('Following Error'); ylabel('\Delta Error [mm]'); ylim([-20, 20]);
    
    subplot(3, 1, 3);
    for j = 1:3; plot(t_sim, acc_math(j,:), '-', 'Color', c_light(j,:), 'LineWidth', 1.5); hold on; end
    grid on; title('Theoretical Acceleration'); ylabel('Acc [m/s^2]'); xlabel('Time [s]'); ylim([-80, 80]);

    for sp = 1:3; subplot(3, 1, sp); hold on;
        for sb = 1:length(segment_boundaries)
            xline(segment_boundaries(sb), 'k--', 'LineWidth', 1, 'Alpha', 0.6, 'HandleVisibility', 'off');
        end
    end
    print(fig_kin, fullfile(run_dir, sprintf('%s_Kinematics.png', safe_tag)), '-dpng', '-r150'); close(fig_kin);
    
    fig_dyn = figure('Visible', 'off', 'Position', [100, 100, 1400, 900]); 
    for j = 1:3
        subplot(3, 1, j);
        plot(t_sim, tau_math(j,:), '-', 'Color', c_light(j,:), 'LineWidth', 1); hold on; grid on;
        plot(t_sim, tau_sim_interp(:,j), '-', 'Color', c_dark(j,:), 'LineWidth', 1.5);
        yline(tau_cont_limit, 'r--', 'HandleVisibility', 'off'); yline(-tau_cont_limit, 'r--', 'HandleVisibility', 'off');
        yline(tau_peak_limit, 'r-', 'HandleVisibility', 'off'); yline(-tau_peak_limit, 'r-', 'HandleVisibility', 'off');
        title(sprintf('Motor %d Torque [%s]', j, raw_tag), 'Interpreter', 'none'); ylabel('Torque [Nm]');
        ylim([-max(10, tau_peak_limit*1.15), max(10, tau_peak_limit*1.15)]);
        for sb = 1:length(segment_boundaries); xline(segment_boundaries(sb), 'k--', 'Alpha', 0.6, 'HandleVisibility', 'off'); end
    end
    print(fig_dyn, fullfile(run_dir, sprintf('%s_Dynamics.png', safe_tag)), '-dpng', '-r150'); close(fig_dyn);
end

%% 6. FINAL MASTER CSV & PARETO
summary_table = table(B(:), F(:), R(:), config_strings, results(:,4), results(:,5), ...
    'VariableNames', {'Bicep_Length_mm', 'Forearm_Length_mm', 'Reduction_Ratio', 'Configuration', 'Fastest_Cycle_Time_s', 'Max_Tracking_Error_mm'});
writetable(summary_table, fullfile(root_out_folder, 'Master_Optimization_Summary.csv'));
disp('All Processing Complete! Summary CSV saved.');