close all; clc; clear;

%% 0. Simulation Configuration (CFG)
cfg = struct();

cfg.start_with_yaw = true;    % If true, the drone will start with a yaw angle aligned to the initial trajectory direction. If false, it will start with a yaw of 0 rad.

% 기본 설정
cfg.dt = 0.01;                % Time step
cfg.target_yaw = [];          % Target yaw angle (rad) - If empty, dynamic yaw will be calculated
cfg.kappa_lookahead = 5.0;         % Feedforward 곡률 예견 거리 [m] (전방 이 지점의 순간 곡률 사용, [] 이면 전방 5초 평균)
cfg.kappa_tau = 0.2;               % 전방 곡률 댐핑 시정수 [s] (1차 지연, 0 이면 댐핑 없음)
cfg.target_speed = 4;              % Target speed (m/s)            

% 경로 종류
cfg.path_type = 'circle';          % 'circle' : 정지 -> 직선 가속 -> 원 궤적 -> 직선 탈출,  'spline' : 기존 pathGen 경유점 궤적
cfg.curvature = 0.2;               % 원 곡률 [1/m] (반지름 = 1/|kappa|, 부호: + 는 +y 쪽 선회, - 는 -y 쪽, 0 = 직선 모드)
cfg.n_laps = 5;                    % 원 바퀴 수 (다 돌면 초반 직선과 같은 길이를 직진한 뒤 시뮬레이션 종료)
cfg.accel_dist_per_speed = 3.0;    % 가속 구간 길이 = 이 값 x target_speed [m]  (가속도 = v/(2*이 값), 가속 시간 = 2*이 값 [s])
cfg.cruise_time = 3.0;             % 가속 후 원 진입 전 등속 직선 시간 [s] (목표 속력으로 안정된 뒤 원 진입)

% PID Table Generation Parameters / Gain Scheduling Initiation
cfg.Calculate_PID_Table = 0;  % If true, the PID table will be generated based on the specified speed and curvature ranges.
cfg.Feedforward = true;  % If 1 : Feedforward Path curvature and velocity, 0 : No Feedforward
cfg.Z_Gain_Scheduling = true; % If 0 : Fixed PID + Feedforward, 1 : LPV Gain Scheduling for z-axis

% 고도 계단 실험 (CircleTrajectory): 기준 고도(-z0, 예: 6 m)에서 시작해 n_laps 바퀴마다 한 단계씩 계단 변화
%   'up' = +height_step, 'down' = -height_step, 마지막 고도를 유지한 채 탈출 직선 후 종료
%   원 전체 바퀴 수 = n_laps x (numel(height_order) + 1). 비워 두면({}) 고도 고정 (기존 실험과 같음)
%   주의: 셀 배열 {'up','up','down'} 이나 문자열 배열 ["up","up","down"] 로 쓸 것 (['up','down'] 은 'updown' 한 덩어리가 됨)
cfg.height_step  = 0.3;            % 계단 크기 [m] (포화 없는 선형 범위: 추력 여유 / K_P,z 이하)
cfg.height_order = {};             % 예: {'up', 'up', 'down'} -> 6.0 > 6.3 > 6.6 > 6.3 m
cfg.height_settle_band = 0.10;     % 정착시간 판정 band (계단 크기 대비 비율)
cfg.Z_D_on_measurement = true;     % z 미분항: true = 측정 속도 기반 -K_D*vz (계단 시 미분 킥 없음), false = 기존 오차 미분
cfg.Use_NDI = false;               % true: 기존 비선형 역산(NDI) 분기. false(기본): 평형점 + 선형 보정 구조
                                   %   이 구조에서 Feedforward = false -> 호버 평형점 고정 = 피드포워드 없는 고정 PID
cfg.straight_distance = 30;        % curvature = 0 (직선 모드)일 때 고도 단계 하나당 직진 거리 [m] (n_laps 대신 사용)


% Simulation 환경 및 저장 설정 추가
cfg.play_animation = false;          % 시뮬레이션 애니메이션 재생 여부
cfg.auto_save = false;               % 자동 저장 여부 (true / false)
cfg.save_dir  = './sim_results/261012/2/v=4,Feedforward,Scheduling,alpha=0.15';    % 저장할 디렉토리 경로 지정

% 초기 상태: [x, y, z, dx, dy, dz, phi, theta, psi, p, q, r]'
if strcmpi(cfg.path_type, 'circle'), v0 = 0; else, v0 = cfg.target_speed; end   % 원 궤적: 정지에서 출발
cfg.drone1_init_states = [ 0.0, 0.0, -6.0, ...                                      
                          v0, 0, 0, ...                                                                
                          0, 0, 0, ...                                                                
                          0, 0, 0]';

% Drone Parameters
% Gazebo x500 모델 값으로 고정 (PX4-gazebo-models x500_base/model.sdf, 근거는 PX4/px4_main.m 참고)
% (이전 값: m 1.25, armLength 0.265, Ixx = Iyy 0.0232, Izz 0.0468)
cfg.m = 2.064;                % Drone mass (kg)
cfg.armLength = 0.246;        % Arm length (m), 로터 위치 (±0.174, ±0.174) m
cfg.Ixx = 0.021667;           % Moment of inertia around x-axis (kg*m^2)
cfg.Iyy = 0.021667;           % Moment of inertia around y-axis (kg*m^2)
cfg.Izz = 0.040;              % Moment of inertia around z-axis (kg*m^2)
% [사용 안 함] F450 실측값 (2026-11, 배터리 포함, 프롭 4개 기준)
% cfg.m = 0.95896;            % Drone mass (kg)
% cfg.armLength = 0.225;      % Arm length (m), 대각 450 mm / 2
% cfg.Ixx = 0.01187;          % Moment of inertia around x-axis (kg*m^2)
% cfg.Iyy = 0.01272;          % Moment of inertia around y-axis (kg*m^2)
% cfg.Izz = 0.02440;          % Moment of inertia around z-axis (kg*m^2)

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

%% 0. Calculate PID Table
if cfg.Calculate_PID_Table
     disp('Calculating PID Table...');
     [Kp_x, Ki_x, Kd_x, ...
      Kp_y, Ki_y, Kd_y, ...
      Kp_z, Ki_z, Kd_z] = Build_PID_Table(cfg.m);
end

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