%% PX4(Gazebo SITL / 실기체) + MATLAB PositionCtrl 실행 (A 방식)
%  - 위치 제어: PID_Drone/PositionCtrl (Trim Feedforward + Z 게인 스케줄링) 을 MATLAB 에서 실행
%  - 자세 제어: PX4 (mc_att_control / mc_rate_control)
%  - 경로:      cfg.path_type = 'circle' -> PID_Drone/CircleTrajectory (정지 -> 직선 가속 -> 원 n바퀴 -> 직선 탈출)
%               cfg.path_type = 'spline' -> PID_Drone/pathGen + TrajectoryPlanner (시뮬레이션과 동일)
%  - 결과:      비행 후 plot_px4_result.m 으로 ULog + 이 스크립트의 기록을 합쳐서 플롯
%
%  실행 순서
%   1) PX4 SITL:  make px4_sitl gz_x500        (QGroundControl 은 모니터링용으로 켜 두기)
%   2) 이 스크립트 실행 -> ①~⑤ 자동 진행
%   3) plot_px4_result.m 실행
%
%  PositionCtrl, pathGen, TrajectoryPlanner, CircleTrajectory, finalize_and_save_results 는
%  PID_Drone 에서 PX4 폴더로 복사해 둔 사본을 사용한다 (PID_Drone 은 addpath 하지 않음).

close all; clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(here);

%% 0. 시뮬레이션과 같은 설정 (PID_Drone/main.m 과 맞춰 둘 것)
cfg = struct();

% 제어 모드 (③ 프리롤 + ④ 경로 추종 구간)
%  'MATLAB' : PID_Drone/PositionCtrl 이 자세+추력 명령 (A 방식, 제안 제어기)
%  'PX4'    : PX4 기본 위치제어기(mc_pos_control)에 위치·속도·가속도 셋포인트만 전달 (비교군)
%             -> 게인은 PX4 파라미터(MPC_XY_P, MPC_XY_VEL_*, MPC_Z_*)를 그대로 사용
%  'PPO'    : Sehyun 브랜치 PX4 전용 PPO (Drone_LPV/PX4/RL_DroneEnv_PX4 학습) -> PX4/ppo
%             -> PositionCtrl_PX4 (PX4 mc_pos_control 과 같은 2-DOF 구조, 잔차 0 이면 PX4 기준선과 같은 꼴)
%                + PPO 잔차 게인 오프셋 (10 Hz), 경로는 위 cfg.path_type 그대로, 설정은 아래 cfg.ppo
cfg.mode = 'PPO';
cfg.start_with_yaw = true;
cfg.dt = 0.01;                      % 기준값 표 계산 간격 (시뮬레이션 dt 와 동일하게)
cfg.target_yaw = [];
cfg.target_speed = 10;               % [m/s]
cfg.kappa_lookahead = 0.0;          % Feedforward 곡률 예견 거리 [m] (전방 이 지점의 순간 곡률 사용, [] 이면 전방 5초 평균)
cfg.kappa_tau = 0;                  % 전방 곡률 댐핑 시정수 [s] (1차 지연, 0 이면 댐핑 없음)

% 경로 종류 (PID_Drone/main.m 과 동일)
cfg.path_type = 'circle';          % 'circle' : 정지 -> 직선 가속 -> 원 궤적 -> 직선 탈출,  'spline' : 기존 pathGen 경유점 궤적
cfg.curvature = 0.069;               % 원 곡률 [1/m] (반지름 = 1/|kappa|, 부호: + 는 +y 쪽 선회, - 는 -y 쪽, 0 = 직선 모드)
cfg.n_laps = 3;                    % 원 바퀴 수 (다 돌면 초반 직선과 같은 길이를 직진한 뒤 비행 종료)
cfg.accel_dist_per_speed = 3.0;    % 가속 구간 길이 = 이 값 x target_speed [m]  (가속도 = v/(2*이 값), 가속 시간 = 2*이 값 [s])
cfg.cruise_time = 5.0;             % 가속 후 원 진입 전 등속 직선 시간 [s] (목표 속력으로 안정된 뒤 원 진입)

cfg.Feedforward = true;
cfg.Z_Gain_Scheduling = true;

% 고도 계단 실험 (CircleTrajectory): 기준 고도(-z0, 예: 6 m)에서 시작해 n_laps 바퀴마다 한 단계씩 계단 변화
%   'up' = +height_step, 'down' = -height_step, 마지막 고도를 유지한 채 탈출 직선 후 종료
%   원 전체 바퀴 수 = n_laps x (numel(height_order) + 1). 비워 두면({}) 고도 고정 (기존 실험과 같음)
%   주의: 셀 배열 {'up','up','down'} 이나 문자열 배열 ["up","up","down"] 로 쓸 것 (['up','down'] 은 'updown' 한 덩어리가 됨)
cfg.height_step  = 0.3;            % 계단 크기 [m] (포화 없는 선형 범위: 추력 여유 / K_P,z 이하)
cfg.height_order = {"up","down","up","down","up","down"};             % 예: {'up', 'up', 'down'} -> 6.0 > 6.3 > 6.6 > 6.3 m
cfg.height_settle_band = 0.10;     % 정착시간 판정 band (계단 크기 대비 비율)
cfg.Z_D_on_measurement = true;     % z 미분항: true = 측정 속도 기반 -K_D*vz (계단 시 미분 킥 없음), false = 기존 오차 미분
cfg.Use_NDI = false;               % true: 기존 비선형 역산(NDI) 분기. false(기본): 평형점 + 선형 보정 구조
                                   %   이 구조에서 Feedforward = false -> 호버 평형점 고정 = 피드포워드 없는 고정 PID
cfg.max_angle_deg = 50;            % PositionCtrl 롤·피치 명령 제한 [deg] (phi_eq 35~40 deg 선회용, 기존 고정값 30)
cfg.Err_Yaw = 'desired';           % 오차 회전 yaw: 'desired'(목표 yaw, PX4 자세 명령 좌표계와 일치) / 'measured'(기존)
cfg.Eq_Delay_Comp = false;         % 평형점 지연 보정: 피치 += |v kappa| sin|phi_eq| / K_att
cfg.K_att = 6.5;                   % PX4 자세 각도 P (MC_PITCH_P, SITL 에서 param show 로 확인)
cfg.straight_distance = 120;        % curvature = 0 (직선 모드)일 때 고도 단계 하나당 직진 거리 [m] (n_laps 대신 사용)

cfg.auto_save = true;
cfg.play_animation = false;
cfg.save_dir = '';

% 경로 시작점 = 시뮬레이션 초기 위치 (경로 좌표계)
if strcmpi(cfg.path_type, 'circle'), v0 = 0; else, v0 = cfg.target_speed; end   % 원 궤적: 정지에서 출발
cfg.drone1_init_states = [0.0, 0.0, -6.0, ...           % x, y, z [m]
                        v0, 0, 0, ...                   % vx, vy, vz [m/s]
                        0, 0, 0, ...                    % roll, pitch, yaw [rad]
                        0, 0, 0]';                      % p, q, r [rad/s]

% 기체 파라미터: Gazebo x500 모델로 고정 (F450 실기체 값은 사용하지 않음, 아래 주석으로 보관)
%  (정규화 추력 변환에서는 m 이 상쇄됨. U1[N] 로그 값과 PositionCtrl 의 U1_eq 크기에만 반영)
%  (MATLAB 시뮬레이션 값: m 1.25, armLength 0.265, Ixx = Iyy 0.0232, Izz 0.0468)
cfg.vehicle = 'x500';
% Gazebo x500 모델(PX4-gazebo-models x500_base/model.sdf) 기준
%  - 질량: base_link 2.0 kg + 로터 0.016077 kg x 4 = 2.064 kg
%  - 관성: base_link ixx = iyy = 0.021667, izz = 0.040 [kg m^2]
%  - 팔 길이: 로터 위치 (±0.174, ±0.174) m -> sqrt(2) x 0.174 = 0.246 m
cfg.m = 2.064;
cfg.armLength = 0.246;
cfg.Ixx = 0.021667;
cfg.Iyy = 0.021667;
cfg.Izz = 0.040;

% [사용 안 함] F450 실기체 측정값
%   실기체 F450 측정값 (2026-11, 배터리 포함, 프롭 4개 기준)
%    - 질량: 측정 925.45 g (배터리 포함, 프롭 1개만 장착) + 프롭 11.17 g x 3 = 958.96 g
%    - 관성: 낙하추 회전스탠드 측정값 (측정 당시 프롭 3개 미장착)
%        측정값      Ixx 0.01143, Iyy 0.01228, Izz 0.02270
%        높이 보정   Ixx, Iyy 에서 m d^2 = 0.92545 x 0.021^2 = 0.00041 차감
%                    (회전축 높이 6.3 cm, 무게중심 높이 4.2 cm 가정)
%        프롭 3개    점질량 11.17 g, 축간 대각 450 mm 기준
%                    Ixx, Iyy: + 3 x 0.01117 x 0.1591^2 = +0.00085
%                    Izz     : + 3 x 0.01117 x 0.225^2  = +0.00170
%    - 팔 길이: 대각 450 mm / 2 = 0.225 m
% cfg.m = 0.95896;
% cfg.armLength = 0.225;
% cfg.Ixx = 0.01187;
% cfg.Iyy = 0.01272;
% cfg.Izz = 0.02440;

% 위치 PID 게인: Gazebo SITL(x500)에서 PX4 가 쓰는 위치 제어기 게인을 등가 단일 루프 PID 로 환산
%   PX4 main 기본값 (x500 airframe 4001_gz_x500 은 아래 값을 바꾸지 않음):
%     xy: MPC_XY_P 0.95, MPC_XY_VEL_P_ACC 1.8, MPC_XY_VEL_I_ACC 0.4, MPC_XY_VEL_D_ACC 0.2
%     z : MPC_Z_P  1.0,  MPC_Z_VEL_P_ACC  4.0, MPC_Z_VEL_I_ACC  2.0, MPC_Z_VEL_D_ACC  0.0
%   환산 (PX4: 위치 P -> 속도 PID, 속도 오차 = Kp*e + de/dt 로 보면)
%     a = (Kvp*Kp + Kvi)*e + (Kvi*Kp)*int(e) + (Kvp + Kvd*Kp)*de/dt   [m/s^2]  (Kvd 의 d2e/dt2 항은 생략)
%     xy: KP = 1.8*0.95 + 0.4 = 2.11, KI = 0.4*0.95 = 0.38, KD = 1.8 + 0.2*0.95 = 1.99   [m/s^2 단위]
%         -> Feedforward 모드 PositionCtrl 은 각도 단위(rad)라서 g 로 나눔: 0.215, 0.0387, 0.203
%     z : KP = 4*1 + 2 = 6, KI = 2*1 = 2, KD = 4 [m/s^2 단위] -> NDI 모드의 U1 [N] 단위로 m 을 곱함
%   주의: Feedforward 모드의 PositionCtrl 은 z 게인을 xy 와 같은 계수(P_x, I_x, D_x)에서 만든다
%         (kP_z = P_x * g * m / cos(phi_eq)) -> z 는 가속도 단위 2.11 / 0.38 / 1.99 가 되어 PX4 z(6 / 2 / 4)보다 느리다
%   P_z, I_z, D_z 는 Use_NDI = true 일 때만 쓰임 (NDI 분기는 P_x 등도 가속도 단위로 씀 -> 지금 값(rad 단위)과 g 배 차이)
%
%   [이전 값 1] 4차 특성방정식 설계 (PX4 자세 루프 a = 4 rad/s, 극점 -1, -1, -0.5, -1.5)
%     xy: P 0.089, I 0.019, D 0.147 [rad 단위],  z: P 4.0, I 0.001, D 0.0
%   [이전 값 2] 최초 값
%     xy: P 3.17,  I 0.001, D 0.81,              z: P 4.0, I 0.001, D 0.0
cfg.posGain = containers.Map( ...
    {'P_x','I_x','D_x', ...
     'P_y','I_y','D_y', ...
     'P_z','I_z','D_z'}, ...
    {0.215, 0.0387, 0.203, ...
     0.215, 0.0387, 0.203, ...
     6.0*cfg.m, 2.0*cfg.m, 4.0*cfg.m});

%% 1. PX4 연결·실행 설정
cfg.px4.local_port     = 14540;   % SITL offboard 포트 (실기체는 텔레메트리/컴패니언 연결에 맞게)
cfg.px4.rate_hz        = 50;      % MATLAB 제어 루프 주기 [Hz]
cfg.px4.path_origin    = [0; 0; 0];  % 경로 좌표계 원점의 NED 위치 [m] (SITL: 이륙 지점 = 0)
cfg.px4.path_rot_deg   = 0;       % 경로 좌표계 x축의 NED 방위각 [deg] (0 = 북쪽)
cfg.px4.preroll_acc    = 2.0;     % 프리롤 가속도 [m/s^2]
cfg.px4.preroll_margin = 2.0;     % 프리롤 등속 여유 거리 [m]
cfg.px4.thr_hover      = NaN;     % NaN 이면 PX4 파라미터 MPC_THR_HOVER 를 읽어서 사용
cfg.px4.thr_limits     = [0.1 0.9];
cfg.px4.tiltmax_air_deg = 50;     % PX4 MPC_TILTMAX_AIR [deg] 로 설정 (기본 45, NaN 이면 건드리지 않음). SITL 은 값이 저장되어 유지됨
cfg.px4.abort_err      = 8.0;     % 위치 오차가 이보다 크면 중단 -> PX4 호버·착륙 [m]
cfg.px4.abort_tilt_deg = 60;      % 기울기가 이보다 크면 중단 [deg]
cfg.px4.end_mode       = 'land_plot_return';  % 주행 종료 후 동작
                                   %   'land_plot_return': 그 자리 착륙 -> disarm 후 ULog 결과창·저장 -> 다시 이륙해 프리롤 시작점 복귀 -> 착륙·disarm
                                   %   'land_plot'       : 그 자리 착륙 -> ULog 결과창·저장 (예전 방식)
                                   %   'return_land_plot': 착륙하지 않고 공중에서 프리롤 시작점으로 복귀 -> 착륙 -> ULog 결과창·저장 (직진 실험용)
                                   %   'hold_return'     : PX4 Hold 로 프리롤 시작점 복귀·호버 (착륙 안 함, 결과는 MATLAB 기록 기준)
cfg.px4.return_speed   = 20.0;    % 복귀·② 장거리 이동 최대 속도 [m/s] (px4_smooth_transit, PX4 MPC_XY_VEL_MAX 는 이동 중에만 올림)
cfg.px4.return_acc     = 2.0;     % 복귀·② 장거리 이동 최대 가속도 [m/s^2] (사인 램프 가감속, 20 m/s 까지 약 16 s)
cfg.px4.start_here     = false;   % true: 지금 서 있는 곳을 프리롤 시작점으로 (경로 원점 수평 이동, ② 장거리 이동 생략). 배치 실행에서 사용
cfg.px4.ulog_close_wait = 4;     % disarm 후 ULog 파일이 닫힐 때까지 기다리는 시간 [s]
cfg.px4.result_dir     = fullfile(here, 'results');

%% 1-1. PPO 모드 설정 (Sehyun 브랜치 Drone_LPV/PX4/RL_DroneEnv_PX4 + PositionCtrl_PX4 와 같은 값, 커밋 0bd9cfd)
cfg.ppo.agent_file   = fullfile(here, 'ppo', 'saved_gs_altstep_ppo_agent.mat');   % 학습된 에이전트 (agent 또는 saved_agent)
cfg.ppo.rate_hz      = 10;        % PPO 개입 주기 (학습: 0.1 s)
cfg.ppo.lookahead_L  = 2.0;       % 관측 kappa_spatial 의 예견 거리 [m] (학습: dt = L / v_des)
cfg.ppo.range_P_xy   = 0.8;       % 잔차 게인 스케일 (action [-1, 1] -> 오프셋, 가속도 단위 m/s^2)
cfg.ppo.range_D_xy   = 1.0;
cfg.ppo.range_P_z    = 1.5;
cfg.ppo.range_D_z    = 1.0;
cfg.ppo.on_mode      = 'always';  % 'always': ④ 시작(정지·가속 구간 포함)부터 전체 구간에서 PPO 잔차 적용 (가속 구간은 학습 분포 밖)
                                  % 'speed' : 목표 속력 도달(가속 끝, t >= t_acc)부터 적용 -> 등속 직선 cruise_time 뒤 원 진입
                                  % 'circle': 원 진입(t >= t_acc + cruise_time)부터 적용
% PositionCtrl_PX4 기준 게인 (학습 환경 cfg 와 동일)
cfg.ppo.ctrl.pos_Kp_ang        = 0.215;    % xy [rad/m] (내부에서 x g -> 2.109 m/s^2 per m)
cfg.ppo.ctrl.pos_Ki_ang        = 0.0387;
cfg.ppo.ctrl.pos_Kd_ang        = 0.203;
cfg.ppo.ctrl.pos_Kp_z          = 6.0;      % z [m/s^2 단위]
cfg.ppo.ctrl.pos_Ki_z          = 2.0;
cfg.ppo.ctrl.pos_Kd_z          = 4.0;
cfg.ppo.ctrl.k_ff_v            = 1.0;      % 목표 속도 feedforward
cfg.ppo.ctrl.k_ff_a            = 1.0;      % 목표 가속도 feedforward
cfg.ppo.ctrl.Z_Gain_Scheduling = true;     % 기울기 보상 (1 / cos(phi) cos(theta))
cfg.ppo.ctrl.max_angle_deg     = cfg.max_angle_deg;
cfg.ppo.ctrl.MPC_THR_HOVER     = 0.73;     % 임시값: run_px4_mission 이 ② 측정 호버 추력으로 덮어씀 (학습 환경 값은 0.6)

%% 1-2. 일괄 실험 덮어쓰기 (run_px4_batch 에서 호출될 때만 적용, 직접 실행 시에는 비어 있음)
cfg_ov = getappdata(0, 'px4_cfg_override');
if ~isempty(cfg_ov)
    cfg = apply_cfg_override(cfg, cfg_ov);
    fprintf('[일괄 실험] cfg 덮어쓰기 적용: %s\n', strjoin(fieldnames(cfg_ov)', ', '));
end
clear cfg_ov;

isPPO = strcmpi(cfg.mode, 'PPO');
if isPPO, addpath(fullfile(here, 'ppo')); end

%% 2. 기준값 표 (시뮬레이션과 같은 순서·dt 로 CircleTrajectory / TrajectoryPlanner 호출)
ref = build_reference_table(cfg);                      % PPO 모드면 kappa_now / kappa_spatial 도 계산
fprintf('경로 길이 %.1f m, 추종 시간 %.1f s\n', ref.traj.total_length, ref.tf);

%% 3. 제어기 (PID_Drone/PositionCtrl 그대로)
params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'}, ...
    {cfg.m, cfg.armLength, cfg.Ixx, cfg.Iyy, cfg.Izz});
agent = [];
if isPPO
    pc = cfg.ppo.ctrl;
    pc.m = cfg.m;
    pc.dt_pos = 1 / cfg.px4.rate_hz;                   % 학습: 50 Hz (0.02 s), 비행 중에는 실제 루프 주기로 갱신
    posCtrl = PositionCtrl_PX4(pc);
    S = load(cfg.ppo.agent_file);
    if isfield(S, 'agent'), agent = S.agent; else, agent = S.saved_agent; end
    if agent.ObservationInfo.Dimension(1) ~= 11
        error('PPO 에이전트 관측 차원이 %d 입니다 (PX4 전용 학습은 11).', agent.ObservationInfo.Dimension(1));
    end
    try
        agent.UseExplorationPolicy = false;            % 평가: 확률 정책의 샘플 대신 평균 행동 사용
    catch
        warning('UseExplorationPolicy 를 설정하지 못했습니다 (MATLAB 버전). getAction 이 샘플링된 행동을 낼 수 있습니다.');
    end
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

%% 6. 결과 그래프·저장 + (선택) 재이륙·복귀
switch meta.end_mode
    case {'land_plot_return', 'land_plot', 'return_land_plot'}
        % 착륙 -> disarm 되면 ULog 가 닫힘 -> ULog 기준 결과 (plot_px4_result)
        disp('착륙·disarm 대기 중...');
        if link.waitDisarmed(120)
            pause(cfg.px4.ulog_close_wait);
            try
                plot_px4_result;
            catch err
                warning('ULog 결과 처리 실패: %s', err.message);
            end
        else
            warning('disarm 을 확인하지 못했습니다 (120 s). 결과 처리를 건너뜁니다.');
        end
        if strcmp(meta.end_mode, 'land_plot_return')
            px4_return_to_start(link, meta.cfg, meta);       % 다시 이륙해 프리롤 시작점 복귀 -> 착륙·disarm
        end
    case 'hold_return'
        % 착륙 전이라 ULog 가 닫히지 않으므로 MATLAB 비행 기록(meta.log)으로 바로 그린다
        if isnan(meta.t4_start) || isnan(meta.t4_end)
            warning('④ 경로 추종 구간 기록이 없어 결과 그래프를 그리지 않습니다.');
        else
            log_data = meta_to_log_data(meta, 0.02);
            run_time = datetime('now');
            cfg_plot = meta.cfg;
            cfg_plot.save_dir = fullfile(cfg.px4.result_dir, 'plots', ...
                char(datetime(run_time, 'Format', 'yyyy-MM-dd')), char(datetime(run_time, 'Format', 'HH_mm_ss')));
            disp('※ 결과는 MATLAB 비행 기록 기준입니다 (U1 = PositionCtrl 추력 명령, U2~U4 없음). ULog 기준 그래프는 착륙 후 plot_px4_result.');
            finalize_and_save_results(log_data, cfg_plot);
        end
end
