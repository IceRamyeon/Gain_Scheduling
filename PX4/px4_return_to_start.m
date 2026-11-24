function px4_return_to_start(link, cfg, meta)
    % PX4_RETURN_TO_START  착륙(disarm) 상태에서 다시 이륙해 프리롤 시작점으로 복귀한 뒤 Hold 로 호버 유지
    %   ① 현재 위치에서 프리롤 시작점 고도로 이륙 (오프보드 위치제어)
    %   ② 프리롤 시작점까지 직선 이동 셋포인트를 cfg.px4.return_speed 로 옮기며 이동 (진행 방향을 보며)
    %   ③ 시작 yaw 로 정렬, 정지 확인 후 AUTO.LOITER(Hold) 로 전환 -> 착륙하지 않고 떠 있음
    %   meta.p_pre_ned, meta.yaw_pre : run_px4_mission 이 ② 에서 기록한 프리롤 시작점 (NED) 과 yaw
    px = cfg.px4;
    Ts = 1 / px.rate_hz;
    if isfield(px, 'return_speed') && ~isempty(px.return_speed), v_ret = px.return_speed; else, v_ret = 3.0; end
    if ~isfield(meta, 'p_pre_ned')
        warning('meta 에 프리롤 시작점(p_pre_ned)이 없어 복귀하지 않습니다.');
        return;
    end
    p_goal = meta.p_pre_ned(:); yaw_goal = meta.yaw_pre;

    [x, ~, ok] = link.getState(); t0 = tic;
    while ~ok
        if toc(t0) > 10, error('PX4 상태를 받지 못했습니다.'); end
        pause(0.1); [x, ~, ok] = link.getState();
    end

    disp('재이륙: 프리롤 시작점 고도로 상승');
    p_to = [x(1); x(2); p_goal(3)]; yaw_now = x(9);
    for k = 1:px.rate_hz                               % OFFBOARD 전환 전 1초 선송신
        link.sendPosition(p_to, yaw_now); pause(Ts);
    end
    link.setOffboard(); pause(0.1);
    link.arm();
    hold_until(p_to, yaw_now, 0.3, 1.0, 40);

    dp = p_goal - p_to; D = norm(dp);
    fprintf('프리롤 시작점 복귀: %.1f m, %.1f m/s\n', D, v_ret);
    if D > 0.5
        u = dp / D;
        yaw_m = atan2(u(2), u(1));
        if hypot(u(1), u(2)) < 0.3, yaw_m = yaw_now; end
        hold_until(p_to, yaw_m, 0.5, 0.5, 15);         % 진행 방향으로 회전
        T_ret = D / v_ret; tr = tic;
        while toc(tr) < T_ret
            link.sendPosition(p_to + v_ret * toc(tr) * u, yaw_m, v_ret * u);
            pause(Ts);
        end
    end
    hold_until(p_goal, yaw_goal, 0.3, 2.0, 60);
    link.setHold();
    disp('프리롤 시작점 도착 -> Hold (호버 유지). 다음 비행은 px4_main 재실행, 착륙은 QGC 등에서 직접');

    function hold_until(p_ned, yaw_ned, tol_pos, hold_s, timeout_s)
        th = tic; t_in = [];
        while true
            link.sendPosition(p_ned, yaw_ned);
            [xs, ~, okk] = link.getState();
            if okk
                yaw_err = abs(atan2(sin(xs(9) - yaw_ned), cos(xs(9) - yaw_ned)));
                near = norm(xs(1:3) - p_ned) < tol_pos && norm(xs(4:6)) < 0.3 && yaw_err < deg2rad(5);
                if near
                    if isempty(t_in), t_in = tic; end
                    if toc(t_in) >= hold_s, break; end
                else
                    t_in = [];
                end
            end
            if toc(th) > timeout_s
                error('목표점 도달 시간 초과 (%.0f s). PX4 상태를 확인하세요.', timeout_s);
            end
            pause(Ts);
        end
    end
end
