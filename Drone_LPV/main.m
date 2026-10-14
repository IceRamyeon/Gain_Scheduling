close all; clc; clear;

%% 0. Simulation Configuration (CFG)
cfg = struct();

cfg.start_with_yaw = true;    % If true, the drone will start with a yaw angle aligned to the initial trajectory direction. If false, it will start with a yaw of 0 rad.

% 기본 설정
cfg.dt = 0.01;                % Time step
cfg.target_yaw = [];          % Target yaw angle (rad) - If empty, dynamic yaw will be calculated
cfg.target_speed = 4;              % Target speed (m/s)

% PID Table Generation Parameters / Gain Scheduling Initiation
cfg.Feedforward = true;  % If 1 : Feedforward Path curvature and velocity, 0 : No Feedforward
cfg.Z_Gain_Scheduling = true; % If 0 : Fixed PID + Feedforward, 1 : LPV Gain Scheduling for z-axis


% Simulation 환경 및 저장 설정 추가
cfg.play_animation = false;          % 시뮬레이션 애니메이션 재생 여부
cfg.auto_save = false;               % 자동 저장 여부 (true / false)
cfg.save_dir  = './sim_results/261012/2/v=4,Feedforward,Scheduling,alpha=0.15';    % 저장할 디렉토리 경로 지정

% 초기 상태: [x, y, z, dx, dy, dz, phi, theta, psi, p, q, r]'
cfg.drone1_init_states = [ 0.0, 0.0, -6.0, ...
     cfg.target_speed, 0, 0, ...
     0, 0, 0, ...
     0, 0, 0]';

% Drone Parameters
cfg.m = 1.25;                 % Drone mass (kg)
cfg.armLength = 0.265;        % Arm length (m)
cfg.Ixx = 0.0232;             % Moment of inertia around x-axis (kg*m^2)
cfg.Iyy = 0.0232;             % Moment of inertia around y-axis (kg*m^2)
cfg.Izz = 0.0468;             % Moment of inertia around z-axis (kg*m^2)

% Position Control Gains
cfg.posGain = containers.Map(...
     {'P_x','I_x','D_x', ...
     'P_y','I_y','D_y', ...
     'P_z','I_z','D_z'},...
     {3.17, 0.001, 0.81, ...
     3.17, 0.001, 0.81, ...
     4.0, 0.001, 0.0});

% Attitude Control Gains
cfg.attGain = containers.Map(...
     {'P_phi','I_phi','D_phi', ...
     'P_theta','I_theta','D_theta', ...
     'P_psi','I_psi','D_psi', ...
     'P_zdot','I_zdot','D_zdot'},...
     {18.10, 0.001, 0.93, ...
     18.10, 0.001, 0.93, ...
     36.51, 0.001, 1.87, ...
     25.0, 0.001, 0.0});

%% 1. Initialization
[drone1] = init_drone_and_env(cfg);

%% 2. Run Simulation
disp('Running Simulation Loop.');
[log_data, valid_len] = run_simulation_loop(drone1, cfg);
disp('Running Simulation Loop Finished.');

%% 3. Play Animation
if cfg.play_animation
     disp('Playing Simulation Animation.');
     play_simulation_graphics(drone1, log_data, cfg);
else
     disp('Animation Playback Skipped.');
end

%% 4. Final Plot & Auto Save
finalize_and_save_results(log_data, cfg);