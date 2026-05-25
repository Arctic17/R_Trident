% MATLAB Script: Bode Diagram of Various Regulators and Combinations
clear; clc; close all;

%% 1. Define Controller Parameters
Kp = 2;      % Proportional gain
Ti = 1;      % Integral time constant
Td = 0.1;      % Derivative time constant
Tf = 0.01;     % Filter time constant (for Df, PDf, PIDf)

% Define the Laplace variable 's'
s = tf('s');

%% 2. Define Individual Regulators and Combinations
% Using a containers.Map to store the transfer functions with their labels
regulators = containers.Map();

% Base building blocks
regulators('P')    = tf(Kp);
regulators('I')    = 1 / (Ti * s);
regulators('D')    = Td * s;
regulators('Df')   = (Td * s) / (1 + Tf * s);

% Combinations
regulators('PI')   = regulators('P') + regulators('I');
regulators('PD')   = regulators('P') + regulators('D');
regulators('PDf')  = regulators('P') + regulators('Df');
regulators('PID')  = regulators('P') + regulators('I') + regulators('D');
regulators('PIDf') = regulators('P') + regulators('I') + regulators('Df');

%% 3. Plotting Setup
% Define frequency range for clarity (e.g., 10^-2 to 10^3 rad/s)
w = logspace(-2, 3, 1000);

% List of keys to plot sequentially or in groups
keys_to_plot = {'P', 'I', 'D', 'Df', 'PI', 'PD', 'PDf', 'PID', 'PIDf'};

%% 4. Generate the Bode Plots
figure('Name', 'Regulator Bode Comparisons', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 600]);

% Option A: Overlaid on a single plot for direct comparison
hold on;
for i = 1:length(keys_to_plot)
    name = keys_to_plot{i};
    bode(regulators(name), w);
end
hold off;

% Formatting the overlaid plot
grid on;
title('Bode Diagram of Controller Regulators and Combinations');
legend(keys_to_plot, 'Location', 'best');

% Adjust line widths for better visibility
all_lines = findall(gcf, 'Type', 'Line');
set(all_lines, 'LineWidth', 1.5);