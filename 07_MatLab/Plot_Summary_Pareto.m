%% Plot_Summary_Pareto.m - Granular Design Space Pareto Front
clear; clc; close all;

%% 1. Import Optimization Summary
csv_filename = 'R_Trident_Optimization_Summary.csv';
if ~isfile(csv_filename)
    error('File "%s" not found in active directory.', csv_filename);
end

opts = detectImportOptions(csv_filename);
summary_table = readtable(csv_filename, opts);

% Extract fields
cycle_time = summary_table.Fastest_Cycle_Time_s;
err_mm     = summary_table.Max_Tracking_Error_mm;
r_ratio    = summary_table.Reduction_Ratio;
config_str = summary_table.Configuration;

% Filter valid configurations
valid_idx = ~isnan(cycle_time) & (cycle_time > 0) & ~isnan(err_mm);
if ~any(valid_idx)
    disp('WARNING: 0 valid configurations found in CSV!');
    return;
end

valid_time   = cycle_time(valid_idx);
valid_err    = err_mm(valid_idx);
valid_ratio  = r_ratio(valid_idx);
valid_labels = cellstr(config_str(valid_idx)); % Ensure robust cellstr indexing

%% 2. Figure Setup (GitHub High-Contrast Resolution)
fig = figure('Name', 'Design Space Optimization', ...
             'Color', [1 1 1], ...
             'Position', [100, 100, 1400, 850], ...
             'InvertHardcopy', 'off'); 

ax = gca;
hold(ax, 'on');
grid(ax, 'on');
ax.GridLineStyle = ':';
ax.GridAlpha = 0.4;
ax.LineWidth = 1.0;
ax.FontName = 'Helvetica';
ax.FontSize = 10;

%% 3. Swapped Axes for Granular Error Discrimination
idx_direct = (valid_ratio == 1.0);
idx_belt   = (valid_ratio > 1.0);

% Belt Drive points colored by gear ratio
if any(idx_belt)
    scatter(valid_err(idx_belt), valid_time(idx_belt), 130, valid_ratio(idx_belt), ...
        'filled', 'MarkerEdgeColor', [0.15 0.15 0.15], 'LineWidth', 1.0, ...
        'DisplayName', 'Belt Drive');
    
    % Native MATLAB perceptually uniform colormap (colorblind & dark-mode safe)
    try
        colormap(ax, 'parula');
    catch
        colormap(ax, 'jet');
    end
    
    cb = colorbar(ax); 
    cb.Label.String = 'Reduction Ratio (R_{belt})';
    cb.Label.FontWeight = 'bold';
    cb.Label.FontSize = 10;
    cb.LineWidth = 1.0;
end

% Direct Drive points marked as distinct dark diamonds
if any(idx_direct)
    scatter(valid_err(idx_direct), valid_time(idx_direct), 150, 'd', ...
        'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.2, 'DisplayName', 'Direct Drive (1:1)');
end

%% 4. Angled Non-Overlapping Labels (45° Upward Tilt)
for k = 1:numel(valid_time)
    text(valid_err(k), valid_time(k), sprintf('  %s', valid_labels{k}), ...
        'Rotation', 45, ...
        'FontSize', 8.5, ...
        'FontName', 'Helvetica', ...
        'FontWeight', 'normal', ...
        'Color', [0.15 0.15 0.15], ...
        'Interpreter', 'none', ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'bottom');
end

%% 5. Formatting & Boundary Margins
xlabel('Maximum Dynamic Following Error [mm] (Lower \rightarrow Higher Precision)', ...
    'FontWeight', 'bold', 'FontSize', 11);
ylabel('Fastest Attainable Cycle Time [s] (Lower \rightarrow Higher Speed)', ...
    'FontWeight', 'bold', 'FontSize', 11);
title('R-Trident Parametric Design Optimization Front', ...
    'FontSize', 14, 'FontWeight', 'bold');

% Margins tailored to your data range (allows full room for 45° labels)
min_err = min(valid_err);
max_err = max(valid_err);
min_t   = min(valid_time);
max_t   = max(valid_time);

xlim([9, 10.2]);
ylim([min_t, 0.8]);

box(ax, 'on');

if any(idx_direct) && any(idx_belt)
    legend('Belt Drive', 'Direct Drive (1:1)', 'Location', 'northwest', 'Box', 'on');
elseif any(idx_direct)
    legend('Direct Drive (1:1)', 'Location', 'northwest', 'Box', 'on');
end

%% 6. GitHub-Ready Exports (Opaque White Background)
out_dir = 'docs/img';
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

svg_target = fullfile(out_dir, 'Performance_Capabilities_05.svg');
png_target = fullfile(out_dir, 'Performance_Capabilities_05.png');

try
    exportgraphics(fig, svg_target, 'ContentType', 'vector', 'BackgroundColor', 'white');
    exportgraphics(fig, png_target, 'Resolution', 300, 'BackgroundColor', 'white');
catch
    % Fallback for older MATLAB releases
    print(fig, svg_target, '-dsvg');
    print(fig, png_target, '-dpng', '-r300');
end

fprintf('\nPlot generation complete!\nSaved:\n  -> %s\n  -> %s\n', svg_target, png_target);