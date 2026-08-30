close all; clc; clear;

%% 0. Simulation Configuration (CFG)
cfg = struct();

% 기본 설정
cfg.simTime = 30;           % [sec]
cfg.dt = 0.01;              % Time step
cfg.target_yaw = 0.0;

% 저장 설정 추가
cfg.auto_save = true;               % 자동 저장 여부 (true / false)
cfg.save_dir  = './sim_results';    % 저장할 디렉토리 경로 지정

% Position Control Gains
cfg.posGain = containers.Map(...
	{'P_x','I_x','D_x', ...
     'P_y','I_y','D_y', ...
     'P_z','I_z','D_z'},...
    {1.2, 0.001, 1.8, ...
     1.2, 0.001, 1.8, ...
     2.0, 0.003, 0.9});

% Attitude Control Gains
cfg.attGain = containers.Map(...
	{'P_phi','I_phi','D_phi', ...
     'P_theta','I_theta','D_theta', ...
     'P_psi','I_psi','D_psi', ...
     'P_zdot','I_zdot','D_zdot'},...
    {7.0, 0.006, 1.0, ...
     7.0, 0.006, 1.0, ...
     7.0, 0.006, 1.0, ...
     8.0, 0.01, 1.0});

%% 1. Initialization
[drone1] = init_drone_and_env(cfg);

%% 2. Run Simulation
disp('Running Simulation Loop.');
[log_data, valid_len] = run_simulation_loop(drone1, cfg);
disp('Running Simulation Loop Finished.');

%% 3. Play Animation
play_simulation_graphics(drone1, log_data, cfg);

%% 4. Final Plot & Auto Save
finalize_and_save_results(log_data, cfg);