%% Master_Run.m (Complete Optimization Suite)
clear; clc; close all;
disp('Loading Base Configuration...');
run('BaseSkript.m');

%% 1. Optimization Setup & Folder Generation
out_folder = 'Optimization_Graphs';
if ~exist(out_folder, 'dir')
    mkdir(out_folder);
end

%% 2. Define Parameter Sweep Arrays
so

% Motor Options: [I_rotor (kg*m^2), Tau_Continuous, Tau_Peak]
motor_labels = {'EC 90 flat Open Rotor 48V', 'EC 60 flat Ventilated 48V', 'EC-i 52 High Torque 48V'}; 
motor_opts = [
    47.65e-5, 1.220, 9.57;
    8.32e-5, 0.581, 2.65;  
    2.64e-5, 0.627, 7.83
];

[B, F, R, M] = ndgrid(L_bicep_opts, L_forearm_opts, ratio_opts, 1:size(motor_opts,1));
num_tests = numel(B);

% Log: [Bicep, Forearm, Ratio, Best_Cycle_Time, Best_Cycle_Error]
results = zeros(num_tests, 5); 
config_strings = cell(num_tests, 1); 
err_tolerance = 10; % Max allowed tracking error (mm)

disp(['Starting parameter sweep of ', num2str(num_tests), ' configurations...']);

%% 3. Execute Optimization Loop
for i = 1:num_tests
    params.R_belt = R(i);
    params.enable_belt = (params.R_belt > 1.0); 
    params.bicep_length   = B(i);
    params.forearm_length = F(i);
    
    m_idx = M(i);
    config_strings{i} = sprintf('L1-%d L2-%d R:%.1f (%s)', B(i), F(i), R(i), motor_labels{m_idx});
    
    fprintf('Testing %d/%d (L1:%d, L2:%d, Ratio:%.1f, %s)...\n', ...
        i, num_tests, params.bicep_length, params.forearm_length, params.R_belt, motor_labels{m_idx});

    % --- 3a. DYNAMIC MASS RECALCULATION ---
    params.rod_diameter = 5.0; 
    bicep_vol = (params.bicep_length * params.bicep_width * params.bicep_height) * 1e-9;
    params.bicep_mass = bicep_vol * params.rho_aluminum;
    rod_vol = (pi * (params.rod_diameter/2)^2 * params.forearm_length) * 1e-9;
    params.forearm_mass = (rod_vol * params.rho_carbon_fiber) * 2; 

    % --- 3b. OPTIMAL Z-SURFACE HEURISTIC ---
    params.z_surface = -0.85 * params.forearm_length;
    
    % --- 3c. DYNAMIC PID SCALING ---
    Nd = 100; 
    
    % --- 3d. APPLY SETTINGS & MOTOR LIMITS ---
    params.I_rotor        = motor_opts(m_idx, 1);
    tau_cont_limit        = motor_opts(m_idx, 2); 
    tau_peak_limit        = motor_opts(m_idx, 3); 

    % --- DYNAMIC TRANSMISSION INERTIAS (T2.5, 10mm Aluminum) ---
    if params.R_belt == 1.0
        params.I_pulley_motor = 0.0;     
        params.I_pulley_arm   = 0.0;
    elseif params.R_belt == 2.0
        params.I_pulley_motor = 1.0e-5;  
        params.I_pulley_arm   = 4.5e-5;  
    else 
        params.I_pulley_motor = 1.2e-5;  
        params.I_pulley_arm   = 9.0e-5;  
    end

    params.m_payload = 0.0;            
    params.cycle_time = 0.7;                  
    params.sim_time = 18.0;                    
    params.z_lift = 0;
    params.h_arch = 50;
    params.r_arch = 200;
    params.enable_vertical_drops = 0;
    params.time_drop = 0.1;

    % Execute Math Model
    try
        run('Math_Model.m'); 
    catch
        results(i, :) = [B(i), F(i), R(i), NaN, NaN];
        continue; 
    end

    % Execute Simscape Model
    try
        out = sim('SimulinkModel', 'StopTime', num2str(params.sim_time));
        
        t_tau_raw = out.tau_motor.Time; tau_raw = out.tau_motor.Data;
        t_pos_raw = out.pos_cmd.Time; pos_c_raw = out.pos_cmd.Data; pos_r_raw = out.EndEff_Pos.Data;

        if ndims(tau_raw) == 3;   tau_raw = squeeze(tau_raw)';     end
        if ndims(pos_c_raw) == 3; pos_c_raw = squeeze(pos_c_raw)'; end
        if ndims(pos_r_raw) == 3; pos_r_raw = squeeze(pos_r_raw)'; end

        tau_sim_interp = interp1(t_tau_raw, tau_raw, t_sim, 'linear', 'extrap');
        pos_c_interp   = interp1(t_pos_raw, pos_c_raw, t_sim, 'linear', 'extrap');
        pos_r_interp   = interp1(t_pos_raw, pos_r_raw, t_sim, 'linear', 'extrap');
        delta_pos_full = pos_r_interp - pos_c_interp;
        
        % --- WINDOWED PATTERN EVALUATION (ACTIVE CHECKING) ---
        t_accum = 1.0; 
        T_current = params.cycle_time;
        best_cycle_time = NaN;
        best_err = NaN;
        segment_boundaries = t_accum; 

        fprintf('    --> Evaluating speed tiers for %s:\n', config_strings{i});
        for p = 1:15  % <-- INCREASED FROM 10 to 15
            T_pattern = 6 * T_current;
            t_start = t_accum;
            t_end = min(t_accum + T_pattern, params.sim_time);
            
            window_idx = (t_sim >= t_start) & (t_sim < t_end);
            if ~any(window_idx); break; end
            
            settle_idx = (t_sim >= (t_start + 0.1)) & (t_sim < t_end);
            if ~any(settle_idx); settle_idx = window_idx; end 
            
            err_window = max(vecnorm((pos_r_interp(settle_idx, :) - pos_c_interp(settle_idx, :))'));
            tau_window = tau_sim_interp(settle_idx, :);
            
            max_tau_peak = max(abs(tau_window(:)));
            max_rms_tau = max(rms(tau_window));
            
            if max_tau_peak > tau_peak_limit || max_rms_tau > tau_cont_limit || err_window > err_tolerance 
                fprintf('        [Speed Tiers] Cycle Time %.1fs: FAILED (Peak Tau: %.2f Nm, Error: %.2f mm)\n', ...
                    T_current, max_tau_peak, err_window);
                segment_boundaries(end+1) = t_end; 
                break; 
            else
                fprintf('        [Speed Tiers] Cycle Time %.1fs: PASSED (Peak Tau: %.2f Nm, Error: %.2f mm)\n', ...
                    T_current, max_tau_peak, err_window);
                best_cycle_time = T_current; 
                best_err = err_window;
            end
            
            t_accum = t_accum + T_pattern;
            segment_boundaries(end+1) = t_accum; 
            T_current = max(0.05, T_current - params.time_drop);
            if t_accum >= params.sim_time; break; end
        end

        results(i, :) = [B(i), F(i), R(i), best_cycle_time, best_err];

        %% --- GENERATE & SAVE RUN GRAPHS (WIDER & DETAILED) ---
        file_name = strrep(config_strings{i}, ' ', '_');
        file_name = strrep(file_name, ':', '-');
        file_name = strrep(file_name, '(', '');  % Remove opening parenthesis
        file_name = strrep(file_name, ')', '');  % Remove closing parenthesis
        
        c_light = [1.0 0.4 0.4; 0.4 1.0 0.4; 0.4 0.6 1.0]; 
        c_dark  = [0.6 0.0 0.0; 0.0 0.6 0.0; 0.0 0.0 0.6]; 
        
        % 1. Kinematics Graph [Position, Error, Acceleration]
        fig_kin = figure('Visible', 'off', 'Position', [100, 100, 2000, 1000]); 
        
        subplot(3, 1, 1);
        for j = 1:3
            plot(t_sim, pos_c_interp(:,j), '-', 'Color', c_light(j,:), 'LineWidth', 1.5); hold on;
            plot(t_sim, pos_r_interp(:,j), '--', 'Color', c_dark(j,:), 'LineWidth', 2);
        end
        grid on; title(sprintf('Cartesian Path [%s]', config_strings{i}), 'Interpreter', 'none'); 
        ylabel('Position [mm]'); legend('X Cmd', 'X Real', 'Y Cmd', 'Y Real', 'Z Cmd', 'Z Real', 'Location', 'eastoutside');
        
        subplot(3, 1, 2);
        for j = 1:3
            plot(t_sim, delta_pos_full(:,j), 'Color', c_dark(j,:), 'LineWidth', 1.5); hold on;
        end
        grid on; title('Following Error (Real - Commanded)'); ylabel('\Delta Error [mm]'); 
        legend('\Delta X', '\Delta Y', '\Delta Z', 'Location', 'eastoutside');
        ylim([-20, 20]);
        
        subplot(3, 1, 3);
        plot(t_sim, acc_math(1,:), '-', 'Color', c_light(1,:), 'LineWidth', 1.5); hold on;
        plot(t_sim, acc_math(2,:), '-', 'Color', c_light(2,:), 'LineWidth', 1.5); % FIXED LINE
        plot(t_sim, acc_math(3,:), '-', 'Color', c_light(3,:), 'LineWidth', 1.5);
        grid on; title('Commanded Theoretical Acceleration (XYZ)'); ylabel('Acc [m/s^2]'); xlabel('Time [s]');
        legend('Acc X', 'Acc Y', 'Acc Z', 'Location', 'eastoutside');
        ylim([-80, 80]);
        % === 2. ADD VERTICAL LINES TO KINEMATICS SUBPLOTS ===
        for sp = 1:3
            subplot(3, 1, sp);
            hold on;
            for sb = 1:length(segment_boundaries)
                xline(segment_boundaries(sb), 'k--', 'LineWidth', 1, 'Alpha', 0.6, 'HandleVisibility', 'off');
            end
        end
        
        print(fig_kin, fullfile(out_folder, sprintf('Kinematics_%s.png', file_name)), '-dpng', '-r150');
        print(fig_kin, fullfile(out_folder, sprintf('Kinematics_%s.svg', file_name)), '-dsvg');
        close(fig_kin);
        
        % 2. Dynamics Graph
        fig_dyn = figure('Visible', 'off', 'Position', [100, 100, 1400, 1000]); 
        labels = {'Motor 1', 'Motor 2', 'Motor 3'};
        for j = 1:3
            subplot(3, 1, j);
            plot(t_sim, tau_math(j,:), '-', 'Color', c_light(j,:), 'LineWidth', 1.5); hold on; grid on;
            plot(t_sim, tau_sim_interp(:,j), '--', 'Color', c_dark(j,:), 'LineWidth', 2);
            
            yline(tau_cont_limit, 'r--', 'Continuous Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
            yline(-tau_cont_limit, 'r--', 'HandleVisibility', 'off');
            yline(tau_peak_limit, 'r-', 'Stall Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
            yline(-tau_peak_limit, 'r-', 'HandleVisibility', 'off');
            
            ylim([-10, 10]);

            title(sprintf('%s Torque [%s]', labels{j}, config_strings{i}), 'Interpreter', 'none');
            ylabel('Torque [Nm]');
            legend('Theoretical', 'Simscape (Real)', 'Location', 'eastoutside');
            if j == 3; xlabel('Time [s]'); end
        end

        % === 3. ADD VERTICAL LINES TO DYNAMICS SUBPLOTS ===
        for sp = 1:3
            subplot(3, 1, sp);
            hold on;
            for sb = 1:length(segment_boundaries)
                xline(segment_boundaries(sb), 'k--', 'LineWidth', 1, 'Alpha', 0.6, 'HandleVisibility', 'off');
            end
        end
        
        print(fig_dyn, fullfile(out_folder, sprintf('Dynamics_%s.png', file_name)), '-dpng', '-r150');
        print(fig_dyn, fullfile(out_folder, sprintf('Dynamics_%s.svg', file_name)), '-dsvg');
        close(fig_dyn);
        
    catch ME
        fprintf('  Simulation failed: %s\n', ME.message);
        results(i, :) = [B(i), F(i), R(i), NaN, NaN];
    end
end
disp('Sweep Complete.');

%% 3.5. Export Summary Results to CSV (Option A)
summary_table = table(B(:), F(:), R(:), config_strings, results(:,4), results(:,5), ...
    'VariableNames', {'Bicep_Length_mm', 'Forearm_Length_mm', 'Reduction_Ratio', 'Configuration', 'Fastest_Cycle_Time_s', 'Max_Tracking_Error_mm'});
csv_filename = fullfile(out_folder, 'R_Trident_Optimization_Summary.csv');
writetable(summary_table, csv_filename);
fprintf('Summary results successfully exported to: %s\n', csv_filename);

%% 4. Plotting - Final Annotated Pareto Front
valid_idx = ~isnan(results(:,4)) & (results(:,4) > 0);
valid_results = results(valid_idx, :);
valid_labels = config_strings(valid_idx);

if isempty(valid_results)
    disp('--------------------------------------------------');
    disp('WARNING: 0 configurations survived the evaluation limits!');
    disp('--------------------------------------------------');
else
    figure('Name', 'Design Space Optimization', 'Color', 'w', 'Position', [200, 200, 1200, 800]); 
    
    idx_direct = valid_results(:,3) == 1.0;
    idx_belt = valid_results(:,3) > 1.0;
    
    hold on;
    % Plot Belt Drive points
    if any(idx_belt)
        scatter(valid_results(idx_belt, 4), valid_results(idx_belt, 5), 100, valid_results(idx_belt, 3), 'filled', 'MarkerEdgeColor', 'k');
        colormap('jet'); cb = colorbar; cb.Label.String = 'Reduction Ratio';
    end
    
    % Plot Direct Drive points
    if any(idx_direct)
        scatter(valid_results(idx_direct, 4), valid_results(idx_direct, 5), 150, 'k', 'Marker', 'd', ...
            'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w', 'DisplayName', 'Direct Drive');
    end
    
    % Annotate every point with its configuration string
    for k = 1:size(valid_results, 1)
        text(valid_results(k, 4), valid_results(k, 5), ['  ' valid_labels{k}], ...
            'VerticalAlignment', 'middle', 'Interpreter', 'none', 'FontSize', 9);
    end
    
    % Formatting
    xlabel('Fastest Successful Cycle Time [s] (Lower is Better)', 'FontWeight', 'bold');
    ylabel('Maximum Tracking Error during that Cycle [mm]', 'FontWeight', 'bold');
    title('R-Trident Performance Capabilities', 'FontSize', 14);
    set(gca, 'XDir', 'reverse'); 
    grid on;
    
    if any(idx_direct) && any(idx_belt)
        legend('Belt Drive', 'Direct Drive', 'Location', 'best');
    elseif any(idx_direct)
        legend('Direct Drive', 'Location', 'best');
    end
end