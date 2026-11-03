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
cfg.start_with_yaw = true;
cfg.dt = 0.01;                    % 기준값 표 계산 간격 (시뮬레이션 dt 와 동일하게)
cfg.target_yaw = [];
cfg.target_speed = 3;             % [m/s]

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

cfg.posGain = containers.Map( ...
    {'P_x','I_x','D_x', ...
     'P_y','I_y','D_y', ...
     'P_z','I_z','D_z'}, ...
    {0.1, 0, 0.1, ...
     0.1, 0, 0.1, ...
     1.0, 0, 0.0});

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

%% 2. 기준값 표 (시뮬레이션과 같은 순서·dt 로 TrajectoryPlanner 호출)
ref = build_reference_table(cfg);
fprintf('경로 길이 %.1f m, 추종 시간 %.1f s\n', ref.traj.total_length, ref.tf);

%% 3. 제어기 (PID_Drone/PositionCtrl 그대로)
params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'}, ...
    {cfg.m, cfg.armLength, cfg.Ixx, cfg.Iyy, cfg.Izz});
posCtrl = PositionCtrl(cfg.posGain, params, cfg);

%% 4. PX4 연결 후 ①~⑤ 실행
link = Px4Link(cfg.px4.local_port);
cleanupObj = onCleanup(@() delete(link));
meta = run_px4_mission(link, cfg, ref, posCtrl);

%% 5. 비행 기록 저장 (ULog 와 합칠 때 사용)
if ~exist(cfg.px4.result_dir, 'dir'), mkdir(cfg.px4.result_dir); end
metaFile = fullfile(cfg.px4.result_dir, sprintf('px4_run_%s.mat', datetime("now", 'yyyymmdd_HHMMSS')));
save(metaFile, 'meta');
fprintf('비행 기록 저장: %s\n', metaFile);
disp('ULog 위치(SITL): ~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/<날짜>/<시간>.ulg');
disp('다음 단계: plot_px4_result.m 실행 (가장 최근 비행 기록과 그 비행의 ULog 를 자동으로 찾음)');

plot_px4_result;