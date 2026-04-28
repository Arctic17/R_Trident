%% R-Trident 
clear; clc;

%% Geometry (mm) 
Rs = 210;    % Radius to motor center
Re = 45;     % Radius to effector joint
Lb = 150;    % Bicep length
Lf = 300;    % Forearm length
Rd = Rs - Re; % Effective delta radius

%% Mass Properties (kg) 
m_bicep = 0.250;    % 250g machined aluminum arm
m_forearm = 0.080;  % 80g carbon fiber rods + joints
m_effector = 0.400; % 400g (Hotend, fans, mount)

% Total mass per arm for gravity calculations
m_total_moving = m_effector + (3 * (m_forearm / 2)); 

%% Motor Specifications (A6M60-400H2B1) 
T_rated = 1.27;      % Rated Torque (Nm)
T_peak = 4.45;       % Peak Torque (Nm)
J_motor = 0.27e-4;   % Rotor Inertia (kg·m²) - From Datasheet
Kt = 0.51;           % Torque Constant (Nm/Amp)
V_max = 3000;        % Rated Speed (RPM)

%% Derived Inertia (kg·m²) 
I_bicep = (1/3) * m_bicep * (Lb/1000)^2;
I_load = (m_total_moving / 3) * (Lb/1000)^2; 

% Total System Inertia per Motor
J_total = J_motor + I_bicep + I_load;

%% Simulation Parameters
g = 9.81;            % Gravity (m/s^2)
Ts = 0.001;          % Controller Sample Time (1ms for industrial servos)