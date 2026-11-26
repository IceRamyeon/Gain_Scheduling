function px4_return_to_start(link, cfg, meta)
    % PX4_RETURN_TO_START  착륙(disarm) 상태에서 다시 이륙해 프리롤 시작점으로 복귀한 뒤 착륙·disarm
    %   ① 현재 위치에서 프리롤 시작점 고도로 이륙 (오프보드 위치제어)
    %   ② 프리롤 시작점까지 px4_smooth_transit 로 이동 (최대 cfg.px4.return_speed, 사인 램프 가감속, 진행 방향을 보며)
    %   ③ 시작 yaw 로 정렬, 정지 확인 후 AUTO.LAND -> disarm 될 때까지 대기
    %   meta.p_pre_ned, meta.yaw_pre : run_px4_mission 이 ② 에서 기록한 프리롤 시작점 (NED) 과 yaw
    px = cfg.px4;
    Ts = 1 / px.rate_hz;
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
    % 시간 제한 없음: 도달 여부만 판단. 단, disarm 직후 재시동이 거부되면(상승이 안 됨) 3 s 마다 OFFBOARD·ARM 재요청
    hold_until(p_to, yaw_now, 0.3, 1.0, Inf, x(3));

    % 프리롤 시작점까지 부드러운 속도 프로파일로 이동 (최대 return_speed, 최대 가속도 return_acc, 사인 램프 가감속)
    px4_smooth_transit(link, p_to, p_goal, px);
    hold_until(p_goal, yaw_goal, 0.3, 1.0, Inf);       % 시간 제한 없음: 도달 여부만 판단
    disp('프리롤 시작점 도착 -> 착륙');
    link.setLand();
    if link.waitDisarmed(120)
        disp('착륙·disarm 완료. 다음 비행은 px4_main 을 다시 실행하면 됩니다.');
    else
        warning('disarm 을 확인하지 못했습니다 (120 s). PX4/QGC 에서 상태를 확인하세요.');
    end

    function hold_until(p_ned, yaw_ned, tol_pos, hold_s, timeout_s, z_ground)
        % z_ground 를 주면 재이륙 감시: 지면 고도에서 0.3 m 이상 올라가기 전까지 3 s 마다 OFFBOARD·ARM 재요청
        if nargin < 6, z_ground = NaN; end
        th = tic; t_in = []; t_re = tic; n_re = 0;
        while true
            link.sendPosition(p_ned, yaw_ned);
            [xs, ~, okk] = link.getState();
            if ~isnan(z_ground) && okk && (z_ground - xs(3)) < 0.3 && toc(t_re) > 3
                n_re = n_re + 1;
                fprintf('재이륙이 시작되지 않아 OFFBOARD·ARM 재요청 (%d회)\n', n_re);
                link.setOffboard(); pause(0.05); link.arm();
                t_re = tic;
            end
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
