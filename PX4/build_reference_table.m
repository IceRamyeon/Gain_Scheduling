function ref = build_reference_table(cfg)
% BUILD_REFERENCE_TABLE  경로 기준값 표를 비행 전에 미리 계산한다.
%   PID_Drone/TrajectoryPlanner 를 시뮬레이션(run_simulation_loop)과 같은 순서·같은 dt로 호출한다.
%   - TrajectoryPlanner 의 곡률 LPF(lpf_alpha)는 "호출 1번 = cfg.dt" 를 가정하므로,
%     비행 중 루프 주기(약 50 Hz)로 직접 부르면 시뮬레이션과 다른 κ가 나온다.
%   - 그래서 시뮬레이션과 똑같이 cfg.dt 간격으로 미리 계산해 두고, 비행 중에는 시간으로 보간한다.
%   - 전방 5초 곡률 계산이 무거워서 실시간 루프 부담도 줄어든다.
%
%   ref.t      : 1xN [s]  (0 ~ tf)
%   ref.pos    : 3xN [m]  목표 위치 (경로 좌표계)
%   ref.vel    : 3xN [m/s] 목표 속도
%   ref.acc    : 3xN [m/s^2] 목표 가속도 (ref.vel 의 수치 미분, PX4 모드 feedforward 용)
%   ref.kappa  : 1xN      Feedforward 곡률 (cfg.kappa_lookahead 지정 시: 전방 지점 곡률 + 댐핑, 아니면 전방 5초 평균)
%   ref.kappa_raw : 1xN   댐핑 전 전방 곡률 (기록용)
%   ref.yaw    : 1xN [rad] 목표 yaw (run_simulation_loop 과 같은 랩어라운드 처리, 연속값)
%   (PPO 모드) ref.kappa_now, ref.kappa_spatial : PPO 관측 곡률, ref.ppo_t_on : PPO 잔차 적용 시작 시각 [s]

    if isfield(cfg, 'kappa_lookahead'), kappa_lk = cfg.kappa_lookahead; else, kappa_lk = []; end
    if isfield(cfg, 'kappa_tau'), kappa_tau = cfg.kappa_tau; else, kappa_tau = 0; end
    if isfield(cfg, 'path_type') && strcmpi(cfg.path_type, 'circle')
        % 정지 -> 직선 가속 -> 원 궤적 (PID_Drone/Drone.m 과 같은 분기)
        if isfield(cfg, 'cruise_time'), cruise_t = cfg.cruise_time; else, cruise_t = 0; end
        traj = CircleTrajectory(cfg.drone1_init_states(1:3), cfg.target_speed, cfg.curvature, ...
                                cfg.accel_dist_per_speed, cfg.n_laps, kappa_lk, kappa_tau, cruise_t);
    else
        [wp_x, wp_y, wp_z] = pathGen();
        traj = TrajectoryPlanner(cfg.drone1_init_states(1:3), [wp_x; wp_y; wp_z], cfg.target_speed, kappa_lk, kappa_tau);
    end

    num_steps = ceil(traj.tf / cfg.dt);
    t = (0:num_steps-1) * cfg.dt;

    % run_simulation_loop 과 동일: 루프 전에 get_position(0) 을 한 번 호출 (LPF 상태도 같이 진행됨)
    [~, init_vel, ~] = traj.get_position(0);
    if norm(init_vel(1:2)) > 1e-3
        prev_yaw = atan2(init_vel(2), init_vel(1));
    else
        prev_yaw = 0.0;
    end
    if ~isempty(cfg.target_yaw)
        prev_yaw = cfg.target_yaw;
    end

    pos = zeros(3, num_steps); vel = zeros(3, num_steps);
    kappa = zeros(1, num_steps); kappa_raw = zeros(1, num_steps); yaw = zeros(1, num_steps);

    for i = 1:num_steps
        [p, v, k, k_raw] = traj.get_position(t(i));
        kappa_raw(i) = k_raw;

        if isempty(cfg.target_yaw)
            if norm(v(1:2)) > 1e-3
                raw_yaw = atan2(v(2), v(1));
                while raw_yaw - prev_yaw > pi,  raw_yaw = raw_yaw - 2*pi; end
                while raw_yaw - prev_yaw < -pi, raw_yaw = raw_yaw + 2*pi; end
                prev_yaw = raw_yaw;
            end
            yaw(i) = prev_yaw;
        else
            yaw(i) = cfg.target_yaw;
        end

        pos(:, i) = p; vel(:, i) = v; kappa(i) = k;
    end

    acc = [gradient(vel(1, :), cfg.dt); gradient(vel(2, :), cfg.dt); gradient(vel(3, :), cfg.dt)];

    ref = struct('t', t, 'pos', pos, 'vel', vel, 'acc', acc, 'kappa', kappa, 'kappa_raw', kappa_raw, 'yaw', yaw, ...
                 'tf', t(end), 'traj', traj);

    % PPO 관측용 곡률 (RL_DroneEnv_PX4 와 같은 정의, 댐핑 없는 순간 곡률)
    %   kappa_now     : 현재 경로 지점의 곡률
    %   kappa_spatial : 전방 lookahead_L [m] 지점의 곡률 (학습: dt = L / v_des 뒤의 곡률 = L m 앞)
    if isfield(cfg, 'mode') && strcmpi(cfg.mode, 'PPO')
        Lk = cfg.ppo.lookahead_L;
        ref.kappa_now = zeros(1, num_steps); ref.kappa_spatial = zeros(1, num_steps);
        for i = 1:num_steps
            if isa(traj, 'CircleTrajectory')
                s_i = traj.arc_at_time(t(i));
            else
                s_i = min(traj.speed * t(i), traj.total_length);
            end
            ref.kappa_now(i)     = traj.curvature_at(s_i);
            ref.kappa_spatial(i) = traj.curvature_at(min(s_i + Lk, traj.total_length));
        end
        % PPO 적용 시작 시각 (원 궤적)
        %   'speed'  : 가속이 끝나 목표 속력에 도달한 시각 t_acc (이후 등속 직선 cruise_time 동안 PPO 가 먼저 적응)
        %   'circle' : 원 진입 시각 t_circ0
        %   'always' : 0
        ref.ppo_t_on = 0;
        if isa(traj, 'CircleTrajectory') && isfield(cfg.ppo, 'on_mode')
            if strcmpi(cfg.ppo.on_mode, 'speed'),  ref.ppo_t_on = traj.t_acc;   end
            if strcmpi(cfg.ppo.on_mode, 'circle'), ref.ppo_t_on = traj.t_circ0; end
        end
    end
end
