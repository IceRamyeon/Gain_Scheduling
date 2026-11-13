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
%   ref.kappa  : 1xN      전방 5초 평균 곡률 (LPF 후)
%   ref.yaw    : 1xN [rad] 목표 yaw (run_simulation_loop 과 같은 랩어라운드 처리, 연속값)

    [wp_x, wp_y, wp_z] = pathGen();
    if isfield(cfg, 'kappa_lookahead'), kappa_lk = cfg.kappa_lookahead; else, kappa_lk = []; end
    traj = TrajectoryPlanner(cfg.drone1_init_states(1:3), [wp_x; wp_y; wp_z], cfg.target_speed, kappa_lk);

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
    kappa = zeros(1, num_steps); yaw = zeros(1, num_steps);

    for i = 1:num_steps
        [p, v, k] = traj.get_position(t(i));

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

    ref = struct('t', t, 'pos', pos, 'vel', vel, 'acc', acc, 'kappa', kappa, 'yaw', yaw, ...
                 'tf', t(end), 'traj', traj);
end
