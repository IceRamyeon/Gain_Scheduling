% px4_main.m
% Gazebo SITL (PX4 x500) 등가 MATLAB 시뮬레이터 메인 실행 스크립트
% - 250 Hz 물리/내부 각속도 루프, 50 Hz 2-DOF 위치 제어기(ZOH), 10 Hz M10 GPS + 8 Hz EKF2
% - 0.04s (10 스텝) MAVLink + EKF FIFO 순수 시간 지연 링 버퍼
% - 비대칭 1차 지연 모터 동역학 (tau_up = 12.5ms, tau_down = 25ms) 및 4-로터 믹서 클리핑 (8.6 N)

close all; clc; clear;

[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
root_dir = fileparts(px4_dir);
addpath(root_dir);
addpath(px4_dir);

fprintf('========================================================================\n');
fprintf('  Gazebo-Equivalent MATLAB Simulator (PX4 x500 SITL Digital Twin)       \n');
fprintf('========================================================================\n');

%% 1. 시뮬레이션 및 기체/제어기 환경 설정 (cfg)
cfg = struct();
cfg.sim_mode = 'PX4_SITL';

% [다중 주기 및 통신 지연 설정 (요구사항 3)]
cfg.dt_sim         = 0.004; % 물리 및 내부 각속도 루프 주기: 250 Hz
cfg.dt_pos         = 0.020; % 외부 위치 제어기 주기: 50 Hz (5스텝마다 1회, ZOH 유지)
cfg.dt             = cfg.dt_sim; % 기존 finalize_and_save_results 호환용
cfg.use_comm_delay = true;  % MAVLink + EKF 순수 지연 사용 여부
cfg.delay_steps    = 10;    % 250 Hz 기준 10 스텝 = 0.04 s 순수 시간 지연

% [기체 물리량 및 비대칭 모터/믹서 파라미터 (요구사항 1: Gazebo x500 기준)]
cfg.m               = 2.064;                 % 질량 [kg]
cfg.armLength       = 0.246;                 % 암 길이 L [m] (로터 위치 ±0.174, ±0.174 m)
cfg.Ixx             = 0.021667;              % X축 관성모멘트 [kg*m^2]
cfg.Iyy             = 0.021667;              % Y축 관성모멘트 [kg*m^2]
cfg.Izz             = 0.040000;              % Z축 관성모멘트 [kg*m^2]
cfg.MPC_THR_HOVER   = 0.60;                  % 호버링 정규화 추력
cfg.f_max_per_rotor = 8.60;                  % 로터당 최대 추력 [N] (4개 합산 34.4 N)
cfg.km_yaw          = 0.05;                  % 로터 요 토크 계수 [m]
cfg.tau_up          = 0.0125;                % 모터 가속 시정수 [s]
cfg.tau_down        = 0.0250;                % 모터 감속 시정수 [s]
cfg.drag_coeff      = [0.10; 0.10; 0.20];    % 선형 공기항력 계수 [dx; dy; dz]

% [PX4 이중 루프 자세 제어기 설정 (요구사항 2)]
% att_mode = 1: 시스템 식별 등가 모델 (sysid_order = 1: 1차 a/(s+a), sysid_order = 2: 2차 캐스케이드)
% att_mode = 2: 전체 이중 루프 모델 (외부 P + ±220 deg/s 클리핑 + 내부 250Hz 각속도 PID)
cfg.att_mode    = 2;
cfg.sysid_order = 2;
cfg.sysid_a     = 4.0;  % [rad/s]
cfg.K_att       = 4.0;  % [rad/s]
cfg.tau_rate    = 0.04; % [s]

% [2-DOF 위치 제어기 설정 (요구사항 4: D-on-Measurement + Feedforward)]
% a = 4.0 rad/s 기준 위상 여유 >= 40 deg, Mp <= 4% 만족 보수적 기본 게인 (각도 환산 기준)
cfg.pos_Kp_ang        = 0.10;  % 수평 위치 비례 게인 (가속도 환산 시 * g)
cfg.pos_Ki_ang        = 0.001; % 수평 위치 적분 게인 (가속도 환산 시 * g)
cfg.pos_Kd_ang        = 0.17;  % 수평 속도 댐핑 게인 (D-on-Measurement, 가속도 환산 시 * g)
cfg.pos_Kp_z          = 4.0;   % 수직 위치 비례 게인
cfg.pos_Ki_z          = 0.2;   % 수직 위치 적분 게인
cfg.pos_Kd_z          = 3.5;   % 수직 속도 댐핑 게인
cfg.k_ff_v            = 1.0;   % 목표 속도 피드포워드 가중치 (0 또는 1)
cfg.k_ff_a            = 1.0;   % 목표 가속도 피드포워드 가중치 (0 또는 1)
cfg.Z_Gain_Scheduling = true;  % 기울기에 따른 Z축 추력/게인 보상
cfg.max_angle_deg     = 30.0;  % 최대 롤/피치 제한 [deg]

% [센서 노이즈 및 EKF2 위상 지연 스위치 (요구사항 5)]
cfg.use_sensor_noise = false;  % true 설정 시 10Hz M10 GPS 랜덤워크(1.5m) + 8Hz EKF2 LPF 활성화
cfg.gps_drift_bound  = 1.5;    % GPS 드리프트 바운드 [m]

% [비행 궤적 및 시뮬레이션 출력 설정 (요구사항 6)]
cfg.target_speed   = 4.0;      % 목표 비행 속력 [m/s]
cfg.target_yaw     = [];       % 빈 배열 시 궤적 접선 방향 동적 Yaw 추적
cfg.start_with_yaw = true;
cfg.play_animation = false;
cfg.auto_save      = true;
cfg.close_figures  = false;
cfg.save_dir       = fullfile(px4_dir, 'sim_results_px4');
cfg.file_prefix    = sprintf('PX4_SITL_Mode%d_v%.1f_Noise%d', ...
    cfg.att_mode, cfg.target_speed, double(cfg.use_sensor_noise));

% 초기 상태: [X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r]'
cfg.drone_init_states = [0.0, 0.0, -6.0, cfg.target_speed, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]';

%% 2. 웨이포인트 생성 및 PX4 드론 객체 초기화
[wp_x, wp_y, wp_z] = pathGen(false);
waypoints = [wp_x; wp_y; wp_z];

drone_px4 = Drone_PX4(cfg, waypoints);

%% 3. 다중 주기 시뮬레이션 실행
fprintf('시뮬레이션 실행 중 (250 Hz 물리/자세 루프, 50 Hz 위치 루프, %.0f ms 지연)...\n', ...
    cfg.delay_steps * cfg.dt_sim * 1000);
[log_data, valid_len] = run_px4_simulation_loop(drone_px4, cfg);

%% 4. 애니메이션 및 결과 시각화/저장
if cfg.play_animation
    play_simulation_graphics(drone_px4, log_data, cfg);
end

finalize_px4_results(log_data, cfg);

