function ref = build_reference_table_lpv(cfg)
% BUILD_REFERENCE_TABLE_LPV  PPO 모드용 기준값 표 (Sehyun 브랜치 Drone_LPV 경로·planner 기준)
%   PPO 에이전트는 Drone_LPV 의 직선 + 원호 경로(pathGen_LPV)와 TrajectoryPlanner_LPV 로 학습됐으므로,
%   관측값(kappa_now, kappa_future)과 PositionCtrl_LPV 의 kappa 입력을 같은 정의로 맞추기 위해 이 planner 를 쓴다.
%   - Drone_LPV/run_simulation_loop 과 같은 순서·dt 로 get_position(t, lookahead) 를 호출
%   - 출력 구조는 PX4/build_reference_table 과 같고, kappa_future 가 추가됨
%
%   ref.t, ref.pos, ref.vel, ref.acc, ref.yaw, ref.tf : build_reference_table 과 동일
%   ref.kappa        : 현재 곡률 kappa_now (원호 구간 고정 곡률, 평균·LPF 없음)
%   ref.kappa_future : cfg.ppo.lookahead_dt 초 뒤의 곡률
%   ref.segments     : 경로 정의 [seg_r; seg_theta; seg_z]

    P = cfg.ppo;
    if P.path_random
        if ~isempty(P.path_seed), rng(P.path_seed); end
        [seg_r, seg_theta, seg_z] = pathGen_LPV(true, P.path_max_kappa);
    else
        [seg_r, seg_theta, seg_z] = pathGen_LPV(false);
    end
    seg = [seg_r; seg_theta; seg_z];

    traj = TrajectoryPlanner_LPV(cfg.drone1_init_states(1:3), seg, cfg.target_speed, P.lookahead_dt);

    num_steps = ceil(traj.tf / cfg.dt);
    t = (0:num_steps-1) * cfg.dt;

    % 초기 yaw (Drone_LPV/run_simulation_loop 과 동일)
    prev_yaw = 0.0;
    if ~isempty(cfg.target_yaw)
        prev_yaw = cfg.target_yaw;
    else
        [~, init_vel] = traj.get_position(0, P.lookahead_dt);
        if norm(init_vel(1:2)) > 1e-3, prev_yaw = atan2(init_vel(2), init_vel(1)); end
    end

    pos = zeros(3, num_steps); vel = zeros(3, num_steps); acc = zeros(3, num_steps);
    kappa = zeros(1, num_steps); kappa_f = zeros(1, num_steps); yaw = zeros(1, num_steps);

    for i = 1:num_steps
        [p, v, k_now, k_fut, a] = traj.get_position(t(i), P.lookahead_dt);
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
        pos(:, i) = p; vel(:, i) = v; acc(:, i) = a; kappa(i) = k_now; kappa_f(i) = k_fut;
    end

    ref = struct('t', t, 'pos', pos, 'vel', vel, 'acc', acc, 'kappa', kappa, 'kappa_future', kappa_f, ...
                 'yaw', yaw, 'tf', t(end), 'traj', traj, 'segments', seg);

    % 비행 가능 여부 점검: 필요 기울기 = atan(v^2 |kappa| / g)
    phi_req = rad2deg(atan(cfg.target_speed^2 * max(abs(kappa)) / 9.81));
    fprintf('PPO 경로: 구간 %d개, 길이 %.1f m, 최대 |kappa| %.3f -> 필요 기울기 %.1f deg\n', ...
        size(seg, 2), traj.total_length, max(abs(kappa)), phi_req);
    fprintf('          x 범위 %.1f ~ %.1f m, y 범위 %.1f ~ %.1f m\n', ...
        min(pos(1, :)), max(pos(1, :)), min(pos(2, :)), max(pos(2, :)));
    if phi_req > 45
        warning('필요 기울기 %.1f deg 가 PositionCtrl_LPV 의 max_angle 45 deg 를 넘습니다. 속도를 낮추세요.', phi_req);
    end
end
