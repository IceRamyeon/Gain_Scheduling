function [log_data, valid_len] = run_simulation_loop(drone1, cfg, agent, rl_env)
    % RUN_SIMULATION_LOOP LPV 단독 제어 및 PPO 잔차 제어 공용 시뮬레이션 루프
    % - agent 인자가 없거나 비어 있으면: 순수 LPV / PID 제어 시뮬레이션 수행
    % - agent 및 rl_env가 주어지면: 10스텝(10 Hz) 주기로 PPO 잔차 게인 오프셋 추론 및 주입

    use_ppo = (nargin >= 3) && ~isempty(agent);
    if use_ppo && (nargin < 4 || isempty(rl_env))
        rl_env = struct('lookahead_dt', 0.5, 'range_P_xy', 1.0, 'range_D_xy', 1.2, ...
                        'range_P_z', 1.5, 'range_D_z', 1.0);
    end

    sim_tf    = drone1.tf;
    num_steps = ceil(sim_tf / cfg.dt);
    valid_len = num_steps;

    % 로그 데이터 메모리 사전 할당
    log_data = struct();
    log_data.t_hist       = zeros(1, num_steps);
    log_data.state_hist   = zeros(12, num_steps);
    log_data.pos_des_hist = zeros(3, num_steps);
    log_data.att_des_hist = zeros(4, num_steps);
    log_data.u_hist       = zeros(4, num_steps);
    log_data.PID_Pos_hist = zeros(3, num_steps);
    log_data.kappa_hist   = zeros(1, num_steps);
    log_data.action_hist  = zeros(4, num_steps);

    % 초기 Yaw 설정
    prev_yaw = 0.0;
    if isfield(cfg, 'start_with_yaw') && cfg.start_with_yaw
        if ~isfield(cfg, 'target_yaw') || isempty(cfg.target_yaw)
            [~, init_vel, ~] = drone1.trajCtrl.get_position(0);
            if norm(init_vel(1:2)) > 1e-3
                prev_yaw = atan2(init_vel(2), init_vel(1));
            end
        else
            prev_yaw = cfg.target_yaw;
        end
        drone1.x(9)     = prev_yaw;
        drone1.euler(3) = prev_yaw;
    end

    N_sub       = 10; % PPO 잔차 개입 주기 (100 Hz / 10 = 10 Hz)
    current_act = zeros(4, 1);
    if use_ppo
        obs_dim     = agent.ObservationInfo.Dimension(1);
        lookahead_t = rl_env.lookahead_dt;
    else
        lookahead_t = 0.5;
    end

    for i = 1:num_steps
        drone1_state = drone1.GetState();
        current_time = drone1.t;

        [traj_pos, traj_vel, kappa_now, kappa_future] = ...
            drone1.trajCtrl.get_position(current_time, lookahead_t);

        % [A] PPO 잔차 액션 추론 (use_ppo == true 일 때 10스텝마다)
        if use_ppo && mod(i - 1, N_sub) == 0
            obs_full = [
                traj_pos(1) - drone1_state(1);
                traj_pos(2) - drone1_state(2);
                traj_pos(3) - drone1_state(3);
                drone1_state(4:8);
                cfg.target_speed;
                kappa_now;
                kappa_future
            ];
            action = getAction(agent, obs_full(1:obs_dim));
            if iscell(action), current_act = action{1}; else, current_act = action; end

            dP_xy = rl_env.range_P_xy * current_act(1);
            dD_xy = max(0, rl_env.range_D_xy * current_act(2));
            dP_z  = rl_env.range_P_z  * current_act(3);
            dD_z  = max(0, rl_env.range_D_z  * current_act(4));
            drone1.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);
        end

        % [B] 동적 Yaw 계산 (랩어라운드 방지)
        if ~isfield(cfg, 'target_yaw') || isempty(cfg.target_yaw)
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

        % [C] 위치/자세 제어기 및 동역학 업데이트
        current_cmd = [traj_pos; dynamic_target_yaw; cfg.target_speed; kappa_now];
        att_cmd = drone1.posCtrl.Update(current_cmd, drone1_state);
        u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);
        drone1.UpdateState(u_motor);

        % [D] 로깅
        log_data.t_hist(i)         = current_time;
        log_data.state_hist(:, i)  = drone1_state;
        log_data.pos_des_hist(:,i) = traj_pos;
        log_data.att_des_hist(:,i) = att_cmd;
        log_data.u_hist(:, i)      = u_motor;
        log_data.PID_Pos_hist(:,i) = drone1.posCtrl.GetZ_PID();
        log_data.kappa_hist(i)     = kappa_now;
        log_data.action_hist(:, i) = current_act;
    end
end
