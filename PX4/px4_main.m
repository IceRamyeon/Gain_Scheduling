%% PX4(Gazebo SITL / 실기체) + MATLAB PositionCtrl 실행 (A 방식)
%  - 위치 제어: PID_Drone/PositionCtrl (Trim Feedforward + Z 게인 스케줄링) 을 MATLAB 에서 실행
%  - 자세 제어: PX4 (mc_att_control / mc_rate_control)
%  - 경로:      PID_Drone/pathGen + TrajectoryPlanner (시뮬레이션과 동일)
%  - 결과:      비행 후 plot_px4_result.m 으로 ULog + 이 스크립트의 기록을 합쳐서 플롯
%
%  실행 순서
%   1) PX4 SITL:  make px4_sitl gz_x500        (QGroundControl 은 모니터링용으로 켜 두기)
%   2) 이 스크립트 실행 -> ①~⑤ 자동 진행
%   3) plot_px4_result.m 실행
%
%  PID_Drone 폴더의 파일은 수정하지 않고 addpath 로 불러서 그대로 사용한다.

close all; clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'PID_Drone'));

%% 0. 시뮬레이션과 같은 설정 (PID_Drone/main.m 과 맞춰 둘 것)
cfg = struct();

% 제어 모드 (③ 프리롤 + ④ 경로 추종 구간)
%  'MATLAB' : PID_Drone/PositionCtrl 이 자세+추력 명령 (A 방식, 제안 제어기)
%  'PX4'    : PX4 기본 위치제어기(mc_pos_control)에 위치·속도·가속도 셋포인트만 전달 (비교군)
%             -> 게인은 PX4 파라미터(MPC_XY_P, MPC_XY_VEL_*, MPC_Z_*)를 그대로 사용
%  'PPO'    : Sehyun 브랜치 Drone_LPV 의 LPV 위치제어기 + PPO 잔차 게인 보정 (PX4/ppo)
%             -> 경로·planner·위치제어기·게인은 PPO 학습 조건(Drone_LPV)과 같게, 설정은 아래 cfg.ppo
cfg.mode = 'PX4';
cfg.start_with_yaw = true;
cfg.dt = 0.01;                    % 기준값 표 계산 간격 (시뮬레이션 dt 와 동일하게)
cfg.target_yaw = [];
cfg.target_speed = 3;             % [m/s]
cfg.kappa_lookahead = 5.0;        % Feedforward 곡률 예견 거리 [m] (전방 이 지점의 순간 곡률 사용, [] 이면 전방 5초 평균)

cfg.Feedforward = true;
cfg.Z_Gain_Scheduling = true;

cfg.auto_save = false;
cfg.play_animation = false;
cfg.save_dir = '';

% 경로 시작점 = 시뮬레이션 초기 위치 (경로 좌표계)
cfg.drone1_init_states = [0.0, 0.0, -6.0, ...           % x, y, z [m]
                        cfg.target_speed, 0, 0, ...     % vx, vy, vz [m/s]
                        0, 0, 0, ...                    % roll, pitch, yaw [rad]
                        0, 0, 0]';                      % p, q, r [rad/s]

% 기체 파라미터: Gazebo x500 모델(PX4-gazebo-models x500_base/model.sdf) 기준
%  - 질량: base_link 2.0 kg + 로터 0.016077 kg x 4 = 2.064 kg
%  - 관성: base_link ixx = iyy = 0.021667, izz = 0.040 [kg m^2]
%  - 팔 길이: 로터 위치 (±0.174, ±0.174) m -> sqrt(2) x 0.174 = 0.246 m
%  (정규화 추력 변환에서는 m 이 상쇄됨. U1[N] 로그 값과 PositionCtrl 의 U1_eq 크기에만 반영)
%  (MATLAB 시뮬레이션 값: m 1.25, armLength 0.265, Ixx = Iyy 0.0232, Izz 0.0468)
cfg.m = 2.064; 
cfg.armLength = 0.246; 
cfg.Ixx = 0.021667; 
cfg.Iyy = 0.021667; 
cfg.Izz = 0.040;

% xy 위치 PID: PX4 자세 루프(1차 극점 a = 4 rad/s)를 넣은 4차 특성방정식으로 설계
%   s^4 + a s^3 + g a KD s^2 + g a KP s + g a KI = 0,  극점 -1, -1, -0.5, -1.5 (합 = -a)
%   -> KP = 3.5/(g a) = 0.089 [rad/m], KI = 0.75/(g a) = 0.019 [rad/(m s)], KD = 5.75/(g a) = 0.147 [rad s/m]
%   교차주파수 약 1.4 rad/s, 위상 여유 46 deg (각속도 루프 0.04 s + 지연 0.04 s 가정 시 39 deg)
%   (이전 값: P 3.17, I 0.001, D 0.81)
%   주의: Feedforward 모드의 PositionCtrl 은 z 게인도 같은 계수(P_x, I_x, D_x)에서 만든다
%         (kP_z = P_x * g * m / cos(phi_eq) 등) -> 이 값이면 z 루프도 느려진다 (극점 약 -0.5 +- 0.43i, -0.42)
%   P_z, I_z, D_z 는 Feedforward = false (NDI) 일 때만 쓰임
cfg.posGain = containers.Map( ...
    {'P_x','I_x','D_x', ...
     'P_y','I_y','D_y', ...
     'P_z','I_z','D_z'}, ...
    {0.089, 0.019, 0.147, ...
     0.089, 0.019, 0.147, ...
     4.0, 0.001, 0.0});

%% 1. PX4 연결·실행 설정
cfg.px4.local_port     = 14540;   % SITL offboard 포트 (실기체는 텔레메트리/컴패니언 연결에 맞게)
cfg.px4.rate_hz        = 50;      % MATLAB 제어 루프 주기 [Hz]
cfg.px4.path_origin    = [0; 0; 0];  % 경로 좌표계 원점의 NED 위치 [m] (SITL: 이륙 지점 = 0)
cfg.px4.path_rot_deg   = 0;       % 경로 좌표계 x축의 NED 방위각 [deg] (0 = 북쪽)
cfg.px4.preroll_acc    = 2.0;     % 프리롤 가속도 [m/s^2]
cfg.px4.preroll_margin = 2.0;     % 프리롤 등속 여유 거리 [m]
cfg.px4.thr_hover      = NaN;     % NaN 이면 PX4 파라미터 MPC_THR_HOVER 를 읽어서 사용
cfg.px4.thr_limits     = [0.1 0.9];
cfg.px4.abort_err      = 8.0;     % 위치 오차가 이보다 크면 중단 -> PX4 호버·착륙 [m]
cfg.px4.abort_tilt_deg = 60;      % 기울기가 이보다 크면 중단 [deg]
cfg.px4.result_dir     = fullfile(here, 'results');

%% 1-1. PPO 모드 설정 (Sehyun 브랜치 Drone_LPV/RL_DroneEnv_LPV, run_simulation_loop 과 같은 값)
cfg.ppo.agent_file     = fullfile(here, 'ppo', 'Agent3000.mat');  % 학습된 에이전트 (saved_agent 또는 agent)
cfg.ppo.rate_hz        = 10;      % PPO 개입 주기 (학습: 0.1 s)
cfg.ppo.lookahead_dt   = 0.5;     % 미래 곡률 예견 시간 [s]
cfg.ppo.range_P_xy     = 1.0;     % 잔차 게인 스케일 (action [-1, 1] -> 오프셋)
cfg.ppo.range_D_xy     = 1.2;
cfg.ppo.range_P_z      = 1.5;
cfg.ppo.range_D_z      = 1.0;
cfg.ppo.path_random    = false;   % false: Drone_LPV 고정 벤치마크 경로, true: 랜덤 경로
cfg.ppo.path_max_kappa = 0.30;    % 랜덤 경로 최대 곡률 (학습 환경 max_kappa)
cfg.ppo.path_seed      = 1;       % 랜덤 시드 ([] 이면 매번 다름)
cfg.ppo.posGain = containers.Map( ...                 % 학습에 쓰인 LPV 기준 게인 (극점 계수 a2 = D, a1 = P, a0 = I)
    {'P_x','I_x','D_x', 'P_y','I_y','D_y', 'P_z','I_z','D_z'}, ...
    {13.5, 5.0, 5.5,  13.5, 5.0, 5.5,  16.88, 6.25, 6.88});

isPPO = strcmpi(cfg.mode, 'PPO');
if isPPO, addpath(fullfile(here, 'ppo')); end

%% 2. 기준값 표 (시뮬레이션과 같은 순서·dt 로 TrajectoryPlanner 호출)
if isPPO
    ref = build_reference_table_lpv(cfg);              % Drone_LPV 경로 + planner (kappa_now, kappa_future)
else
    ref = build_reference_table(cfg);
end
fprintf('경로 길이 %.1f m, 추종 시간 %.1f s\n', ref.traj.total_length, ref.tf);

%% 3. 제어기 (PID_Drone/PositionCtrl 그대로)
params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'}, ...
    {cfg.m, cfg.armLength, cfg.Ixx, cfg.Iyy, cfg.Izz});
agent = [];
if isPPO
    posCtrl = PositionCtrl_LPV(cfg.ppo.posGain, params, cfg);   % Drone_LPV 위치제어기 (잔차 오프셋 입력 가능)
    S = load(cfg.ppo.agent_file);
    if isfield(S, 'agent'), agent = S.agent; else, agent = S.saved_agent; end
    fprintf('PPO 에이전트: %s (관측 %d차원)\n', cfg.ppo.agent_file, agent.ObservationInfo.Dimension(1));
else
    posCtrl = PositionCtrl(cfg.posGain, params, cfg);
end

%% 4. PX4 연결 후 ①~⑤ 실행
link = Px4Link(cfg.px4.local_port);
cleanupObj = onCleanup(@() delete(link));
meta = run_px4_mission(link, cfg, ref, posCtrl, agent);

%% 5. 비행 기록 저장 (ULog 와 합칠 때 사용)
if ~exist(cfg.px4.result_dir, 'dir'), mkdir(cfg.px4.result_dir); end
metaFile = fullfile(cfg.px4.result_dir, sprintf('px4_run_%s_%s.mat', ...
    char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')), cfg.mode));
save(metaFile, 'meta');
fprintf('비행 기록 저장: %s\n', metaFile);
disp('ULog 위치(SITL): ~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/<날짜>/<시간>.ulg');

plot_px4_result;