%% Delta Robot Non-Destructive Test Runner
clear; clc; close all;

% 1. Load the pristine baseline parameters
% (This runs your original script and populates the 'params' struct)
run(['BaseSkript.m']); 

% =========================================================================
% 2. OVERRIDE SECTION (Modify parameters for this specific test only)
% =========================================================================

% Test a Direct-Drive configuration instead of the baseline Belt-Drive
params.enable_belt = 0;            

% Test a different motor rotor inertia 
params.I_rotor = 1.20e-4;          

% Set test-specific cycle times or payload
cycle_time = 0.5;                  
params.m_payload = 0.5; % Adding a 500g payload to the end-effector

% =========================================================================
% 3. EXECUTE SIMULATION
% =========================================================================
disp('Starting Simscape execution with temporary test parameters...');

% Run the model (Ensure the model name matches your actual .slx file)
sim_time = 10.0;
try
    out = sim('SimulinkModel', 'StopTime', num2str(sim_time));
    disp('Simulation Complete.');
catch ME
    error('Simulation failed: %s', ME.message);
end

% =========================================================================
%% 4. EXTRACT & PLOT MUXED DATA (WITH INDEPENDENT TIME VECTORS)
% =========================================================================
% Extract specific time vectors for each signal
t_tau   = out.tau_motor.Time;
t_pos_c = out.pos_cmd.Time;
t_pos_r = out.EndEff_Pos.Time;

% Failsafe: Did the simulation crash instantly?
if length(t_tau) < 2
    error('Simulation aborted! Check the Simulink Diagnostic Viewer.');
end

% Extract Data
tau_raw   = out.tau_motor.Data;
pos_c_raw = out.pos_cmd.Data;
pos_r_raw = out.EndEff_Pos.Data;

% Failsafe: Flatten [3 x 1 x N] arrays to [N x 3] if Simulink exported 3D arrays
if ndims(tau_raw) == 3;   tau   = squeeze(tau_raw)';   else; tau   = tau_raw;   end
if ndims(pos_c_raw) == 3; pos_c = squeeze(pos_c_raw)'; else; pos_c = pos_c_raw; end
if ndims(pos_r_raw) == 3; pos_r = squeeze(pos_r_raw)'; else; pos_r = pos_r_raw; end

% Display Wide 2-Row Layout
figure('Name', 'Test Results: Pos & Torque', 'Color', 'w', 'Position', [100, 100, 1400, 800]);

% Plot 1: 2D Cartesian Trajectories
subplot(2, 1, 1);
plot(t_pos_c, pos_c(:,1), 'r--', 'LineWidth', 1); hold on; grid on;
plot(t_pos_r, pos_r(:,1), 'r-',  'LineWidth', 1.5);
plot(t_pos_c, pos_c(:,3), 'b--', 'LineWidth', 1);
plot(t_pos_r, pos_r(:,3), 'b-',  'LineWidth', 1.5);
title('Cartesian Trajectory: Command vs Real'); 
xlabel('Time [s]'); ylabel('Position [mm]'); 
legend('X Cmd', 'X Real', 'Z Cmd', 'Z Real', 'Location', 'eastoutside');
xlim([0 sim_time]);

% Plot 2: Required Torques
subplot(2, 1, 2);
plot(t_tau, tau(:,1), 'r-', 'LineWidth', 1.5); hold on; grid on;
plot(t_tau, tau(:,2), 'g-', 'LineWidth', 1.5);
plot(t_tau, tau(:,3), 'b-', 'LineWidth', 1.5);
title(sprintf('Required Motor Torques (Belt: %d, Payload: %.1f kg)', params.enable_belt, params.m_payload)); 
xlabel('Time [s]'); ylabel('Torque [Nm]');
legend('\tau_1', '\tau_2', '\tau_3', 'Location', 'eastoutside');
xlim([0 sim_time]);