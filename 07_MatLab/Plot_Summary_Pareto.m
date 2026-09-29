%% Plot_Summary_Pareto.m
clear; clc; close all;

csv_filename = ['R_Trident_Optimization_Summary.csv'];
opts = detectImportOptions(csv_filename);
summary_table = readtable(csv_filename, opts);

% Extract fields
cycle_time = summary_table.Fastest_Cycle_Time_s;
err_mm     = summary_table.Max_Tracking_Error_mm;
r_ratio    = summary_table.Reduction_Ratio;
config_str = summary_table.Configuration;

% Filter valid configurations
valid_idx = ~isnan(cycle_time) & (cycle_time > 0);

if ~any(valid_idx)
    disp('WARNING: 0 configurations found in CSV!');
    return;
end

valid_time   = cycle_time(valid_idx);
valid_err    = err_mm(valid_idx);
valid_ratio  = r_ratio(valid_idx);
valid_labels = config_str(valid_idx);

% Plotting
figure('Name', 'Design Space Optimization', 'Color', 'w', 'Position', [200, 200, 1200, 800]); 
hold on;

idx_direct = (valid_ratio == 1.0);
idx_belt   = (valid_ratio > 1.0);

% Plot Belt Drive points
if any(idx_belt)
    scatter(valid_time(idx_belt), valid_err(idx_belt), 100, valid_ratio(idx_belt), 'filled', 'MarkerEdgeColor', 'k');
    colormap('jet'); 
    cb = colorbar; 
    cb.Label.String = 'Reduction Ratio';
end

% Plot Direct Drive points
if any(idx_direct)
    scatter(valid_time(idx_direct), valid_err(idx_direct), 150, 'k', 'Marker', 'd', ...
        'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w', 'DisplayName', 'Direct Drive');
end

% Annotate points
for k = 1:numel(valid_time)
    text(valid_time(k), valid_err(k), ['  ' valid_labels{k}], ...
        'VerticalAlignment', 'middle', 'Interpreter', 'none', 'FontSize', 9);
end

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