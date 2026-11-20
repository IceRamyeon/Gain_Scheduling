function [log_data, valid_len] = run_px4_simulation_loop(drone, cfg, agent, rl_env)
% RUN_PX4_SIMULATION_LOOP Gazebo SITL 등가 다중 주기(250Hz / 50Hz / 10Hz) 및
% MAVLink+EKF 순수 시간 지연 FIFO Ring Buffer(10스텝 = 0.04s) 실행 루프
% - agent 미전달 시: 순수 PX4 2-DOF + LPV 제어 실행
% - agent 전달 시: 10 Hz (25스텝마다 1회) PPO 잔차 게인 오프셋 추론 및 주입

use_ppo = (nargin >= 3) && ~isempty(agent);
if use_ppo && (nargin < 4 || isempty(rl_env))
    rl_env = struct('lookahead_L', 2.0, 'range_P_xy', 0.8, 'range_D_xy', 1.0, ...
        'range_P_z', 1.5, 'range_D_z', 1.0);
end

sim_tf    = drone.tf;
dt_sim    = cfg.dt_sim; % 0.004 s (250 Hz)
dt_pos    = cfg.dt_pos; % 0.020 s (50 Hz)
pos_ratio = round(dt_pos / dt_sim); % 5 스텝마다 1회 위치 제어기 실행
rl_ratio  = round(0.10 / dt_sim);   % 25 스텝마다 1회(10 Hz) PPO 실행

num_steps = ceil(sim_tf / dt_sim);
valid_len = num_steps;

% 1. 로그 데이터 메모리 사전 할당 (기존 finalize_and_save_results.m 완벽 호환 + PX4 확장 필드)
log_data = struct();
log_data.t_hist           = zeros(1, num_steps);
log_data.state_hist       = zeros(12, num_steps);
log_data.pos_des_hist     = zeros(3, num_steps);
log_data.vel_des_hist     = zeros(3, num_steps);
log_data.acc_des_hist     = zeros(3, num_steps);
log_data.att_des_hist     = zeros(4, num_steps); % [phi_cmd; theta_cmd; psi_cmd; U1_phys]
log_data.att_delayed_hist = zeros(4, num_steps); % 0.04s 지연 버퍼 통과 후 명령
log_data.thr_cmd_hist     = zeros(1, num_steps); % 정규화 추력 명령 [0.1, 0.9]
log_data.u_hist           = zeros(4, num_steps); % 실제 인가된 [U1_act; M1_act; M2_act; M3_act]
log_data.f_cmd_hist       = zeros(4, num_steps); % 4-로터 목표 추력 (클리핑 후) [N]
log_data.f_act_hist       = zeros(4, num_steps); % 4-로터 실제 추력 (비대칭 지연 후) [N]
log_data.p_est_hist       = zeros(3, num_steps); % EKF2 위치 추정치 (50Hz LOCAL_POSITION_NED)
log_data.v_est_hist       = zeros(3, num_steps); % EKF2 속도 추정치 (50Hz LOCAL_POSITION_NED)
log_data.PID_Pos_hist     = zeros(3, num_steps); % [kP_z; kI_z; kD_z]
log_data.kappa_hist       = zeros(1, num_steps);
log_data.sat_rot_hist     = zeros(1, num_steps); % 로터 클리핑 발생 여부
log_data.action_hist      = zeros(4, num_steps); % PPO 잔차 액션 이력

current_act = zeros(4, 1);
if use_ppo
    obs_dim     = agent.ObservationInfo.Dimension(1);
    lookahead_L = rl_env.lookahead_L;
    dt_spatial  = lookahead_L / max(cfg.target_speed, 0.5);
else
    dt_spatial  = 0.5;
end

% 2. 초기 Yaw 정렬
[~, init_vel, ~] = drone.trajCtrl.get_position(0);
if cfg.start_with_yaw && isempty(cfg.target_yaw)
    if norm(init_vel(1:2)) > 1e-3
        init_yaw = atan2(init_vel(2), init_vel(1));
    else
        init_yaw = 0.0;
    end
elseif ~isempty(cfg.target_yaw)
    init_yaw = cfg.target_yaw;
else
    init_yaw = 0.0;
end
drone.x(9)     = init_yaw;
drone.euler(3) = init_yaw;
prev_yaw       = init_yaw;

% 3. Zero-Order Hold (ZOH) 변수 및 FIFO 통신 지연 링 버퍼 (10스텝 = 0.04s) 초기화
init_att_cmd = [0.0; 0.0; init_yaw; cfg.MPC_THR_HOVER];
zoh_att_cmd  = init_att_cmd;
zoh_U1_phys  = drone.m * drone.g;

delay_steps = max(1, round(cfg.delay_steps)); % 기본 10 스텝 (0.04 s @ 250 Hz)
fifo_buffer = repmat(init_att_cmd, 1, delay_steps);
buf_idx     = 1;

% 4. 다중 주기 시뮬레이션 루프 (기본 250 Hz)
for i = 1:num_steps
    state_true   = drone.GetState();
    current_time = drone.t;
    is_50hz_tick = (mod(i - 1, pos_ratio) == 0);

    % [A] 궤적 생성기 조회 (목표 위치, 목표 속도, 현재 곡률, 공간 예견 곡률, 목표 가속도)
    [traj_pos, traj_vel, kappa_avg, kappa_spatial, traj_acc] = drone.trajCtrl.get_position(current_time, dt_spatial);

    % [B] 센서 노이즈 (10Hz GPS 랜덤워크) + EKF2 위상 지연 (8Hz LPF) -> 50Hz LOCAL_POSITION_NED
    [p_est, v_est] = drone.sensorEKF.Update(state_true(1:3), state_true(4:6), is_50hz_tick);

    % [B-1] 10 Hz PPO 잔차 게인 오프셋 추론 (Gazebo와 동일하게 EKF2 추정치 p_est, v_est 관측)
    if use_ppo && mod(i - 1, rl_ratio) == 0
        obs_vec = [
            traj_pos(1) - p_est(1);
            traj_pos(2) - p_est(2);
            traj_pos(3) - p_est(3);
            v_est(1:3);
            state_true(7:8);
            cfg.target_speed;
            kappa_avg;
            kappa_spatial
            ];
        action = getAction(agent, obs_vec(1:obs_dim));
        if iscell(action), current_act = action{1}; else, current_act = action; end

        dP_xy = rl_env.range_P_xy * current_act(1);
        dD_xy = max(0, rl_env.range_D_xy * current_act(2));
        dP_z  = rl_env.range_P_z  * current_act(3);
        dD_z  = max(0, rl_env.range_D_z  * current_act(4));
        drone.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);
    end

    % [C] 50 Hz 위치 제어기 실행 (5스텝마다 1회 갱신, 중간 스텝은 Zero-Order Hold 유지)
    if is_50hz_tick
        % 동적 Yaw 산출 (랩어라운드 방지)
        if isempty(cfg.target_yaw)
            speed_xy = norm(traj_vel(1:2));
            if speed_xy > 1e-3
                raw_yaw  = atan2(traj_vel(2), traj_vel(1));
                yaw_diff = raw_yaw - prev_yaw;
                while yaw_diff >  pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - prev_yaw; end
                while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - prev_yaw; end
                dynamic_target_yaw = raw_yaw;
                prev_yaw = dynamic_target_yaw;
            else
                dynamic_target_yaw = prev_yaw;
            end
        else
            dynamic_target_yaw = cfg.target_yaw;
        end

        % 50 Hz 2-DOF 위치 제어기 업데이트 (반드시 dt_pos = 0.02s 기준 동작)
        [zoh_att_cmd, zoh_U1_phys] = drone.posCtrl.Update(...
            traj_pos, traj_vel, traj_acc, dynamic_target_yaw, ...
            p_est, v_est, state_true(7:9));
    end

    % [D] MAVLink + EKF 순수 시간 지연 FIFO Ring Buffer (0.04 s = 10 steps @ 250 Hz)
    if cfg.use_comm_delay && delay_steps > 0
        delayed_att_cmd = fifo_buffer(:, buf_idx);
        fifo_buffer(:, buf_idx) = zoh_att_cmd;
        buf_idx = mod(buf_idx, delay_steps) + 1;
    else
        delayed_att_cmd = zoh_att_cmd;
    end

    % [E] 250 Hz PX4 이중 루프 자세 제어기 실행 (Mode 1 또는 Mode 2)
    ctrl_in = drone.attCtrl.Update(delayed_att_cmd, state_true);

    % [F] 250 Hz 4-로터 믹서 클리핑 + 비대칭 모터 동역학 + 6-DOF 물리 적분
    [u_actual, f_cmd, f_act] = drone.UpdateState(ctrl_in);

    % [G] 데이터 로깅
    log_data.t_hist(i)             = current_time;
    log_data.state_hist(:, i)      = state_true;
    log_data.pos_des_hist(:, i)    = traj_pos;
    log_data.vel_des_hist(:, i)    = traj_vel;
    log_data.acc_des_hist(:, i)    = traj_acc;
    log_data.att_des_hist(:, i)    = [zoh_att_cmd(1:3); zoh_U1_phys];
    log_data.att_delayed_hist(:,i) = delayed_att_cmd;
    log_data.thr_cmd_hist(i)       = zoh_att_cmd(4);
    log_data.u_hist(:, i)          = u_actual;
    log_data.f_cmd_hist(:, i)      = f_cmd;
    log_data.f_act_hist(:, i)      = f_act;
    log_data.p_est_hist(:, i)      = p_est;
    log_data.v_est_hist(:, i)      = v_est;
    log_data.PID_Pos_hist(:, i)    = drone.posCtrl.GetZ_PID();
    log_data.kappa_hist(i)         = kappa_avg;
    log_data.sat_rot_hist(i)       = double(drone.is_saturated);
    log_data.action_hist(:, i)     = current_act;
end
end

