function meta = run_px4_mission(link, cfg, ref, posCtrl)
% RUN_PX4_MISSION  PX4(SITL/실기체)에서 단계 ①~⑤를 실행한다 (A 방식).
%
%   ① 이륙·상승      : PX4 기본 위치제어기   z = 0 -> 경로 시작 고도
%   ② 시작점 이동    : PX4 기본 위치제어기   프리롤 시작점(경로 시작점 - L·접선)에서 호버, yaw 정렬
%   ③ 프리롤 가속    : MATLAB PositionCtrl   직선 가속, 경로 시작점에서 속도 v 도달
%   ④ 경로 추종      : MATLAB PositionCtrl   TrajectoryPlanner t = 0 ~ tf   <- RMSE 평가 구간
%   ⑤ 호버·착륙      : PX4 기본 위치제어기 -> AUTO.LAND
%
%   ③④ 에서는 PID_Drone/PositionCtrl 의 출력 [phi, theta, psi, U1] 을
%   SET_ATTITUDE_TARGET(자세 쿼터니언 + 정규화 추력)으로 보내고, 자세 제어는 PX4가 한다.
%
%   meta.t4_start / meta.t4_end : ④ 구간의 PX4 부팅 후 시간 [s]  -> ULog 를 자를 때 사용
%   meta.log                    : 루프마다 기록한 행렬 (열 정의는 meta.log_cols)

    px  = cfg.px4;
    if ~isfield(cfg, 'mode'), cfg.mode = 'MATLAB'; end
    usePX4 = strcmpi(cfg.mode, 'PX4');                % true: ③④ 를 PX4 위치제어기가 수행 (비교군)
    fprintf('제어 모드: %s\n', cfg.mode);
    Ts  = 1 / px.rate_hz;
    g   = 9.81;
    rot = deg2rad(px.path_rot_deg);

    %% 호버 추력 (U1[N] -> 정규화 추력 변환에 사용)
    %  - cfg.px4.thr_hover 에 값이 있으면 그 값을 쓴다.
    %  - NaN 이면 ② 호버 중 PX4 가 실제로 쓰는 추력(ATTITUDE_TARGET.thrust)을 측정해서 쓴다.
    %    (MPC_THR_HOVER 파라미터는 초기값일 뿐이고, 실제 호버 추력과 다르면 ③ 진입 후 고도가 떨어진다)
    %  - 측정에 실패하면 MPC_THR_HOVER 파라미터로 대체한다.
    thr_hover = px.thr_hover;
    thr_param = link.readParam('MPC_THR_HOVER');
    fprintf('MPC_THR_HOVER (파라미터) = %.3f\n', thr_param);
    if isnan(thr_hover), thr_hover = thr_param; end
    if isnan(thr_hover), thr_hover = 0.5; end          % 임시값: ② 이후 측정값으로 교체됨
    % 정규화 추력 = thr_hover * U1 / (m g)
    %  -> PositionCtrl 의 U1 은 질량 m 에 비례(U1_eq, Z 게인 모두 m 포함)하므로 비율로 바꾸면 m 이 상쇄된다.
    %  -> 단, 실제 추력은 명령에 선형이 아니라서(THR_MDL_FAC) 호버 근처에서만 정확한 근사다.
    U1_to_thr = @(U1) min(max(thr_hover * U1 / (cfg.m * g), px.thr_limits(1)), px.thr_limits(2));

    %% 경로 시작점·접선과 프리롤 기하
    p_start = ref.pos(:, 1);
    d = ref.vel(:, 1) / norm(ref.vel(:, 1));          % 경로 시작 접선 방향 (경로 좌표계)
    yaw0 = ref.yaw(1);
    v = cfg.target_speed;
    a = px.preroll_acc;
    t_acc = v / a;                                     % 가속 시간
    s_acc = v^2 / (2*a);                               % 가속 거리
    L = s_acc + px.preroll_margin;                     % 프리롤 길이
    t_pre = t_acc + px.preroll_margin / v;             % 프리롤 총 시간
    p_pre = p_start - L * d;                           % 프리롤 시작점
    fprintf('프리롤: L = %.1f m, 가속 %.1f s + 등속 %.1f s\n', L, t_acc, t_pre - t_acc);

    %% 로그
    meta.log_cols = {'t_boot', 'phase', ...
        'x','y','z','vx','vy','vz','phi','theta','psi','p','q','r', ...          % 경로 좌표계 상태
        'x_des','y_des','z_des','psi_des','v_des','kappa', ...                    % PositionCtrl 입력
        'phi_cmd','theta_cmd','psi_cmd','U1_cmd', 'thr_norm', ...                 % PositionCtrl 출력
        'kP_z','kI_z','kD_z'};
    LOG = nan(50000, numel(meta.log_cols)); nlog = 0;
    meta.aborted = false; meta.abort_reason = '';
    meta.thr_hover = thr_hover; meta.preroll_L = L;
    meta.t4_start = NaN; meta.t4_end = NaN;

    %% 0) 상태 메시지 주기 요청 (제어 루프 주기와 맞춤) + 첫 상태 수신
    link.setMessageRate(32, px.rate_hz);               % LOCAL_POSITION_NED
    link.setMessageRate(30, px.rate_hz);               % ATTITUDE
    link.setMessageRate(83, px.rate_hz);               % ATTITUDE_TARGET (호버 추력 측정용)
    pause(0.5);
    [x_ned, ~, ok] = link.getState();
    t0 = tic;
    while ~ok
        if toc(t0) > 10, error('PX4 상태(LOCAL_POSITION_NED/ATTITUDE)를 받지 못했습니다.'); end
        pause(0.1); [x_ned, ~, ok] = link.getState();
    end

    %% ① 이륙·상승 (PX4 위치제어)
    z_alt = path_to_ned(p_start, cfg); z_alt = z_alt(3);
    p_to = [x_ned(1); x_ned(2); z_alt];
    yaw_now = x_ned(9);
    disp('① 이륙·상승');
    for k = 1:px.rate_hz                                % OFFBOARD 전환 전 1초 선송신
        link.sendPosition(p_to, yaw_now); pause(Ts);
    end
    link.setOffboard(); pause(0.1);
    link.arm();
    hold_until(p_to, yaw_now, 0.3, 1.0, 40);

    %% ② 프리롤 시작점 이동 (PX4 위치제어)
    disp('② 프리롤 시작점 이동');
    p_pre_ned = path_to_ned(p_pre, cfg);
    hold_until(p_pre_ned, yaw0 + rot, 0.3, 2.0, 60);

    %% 호버 추력 측정 (② 호버 상태에서 1초 동안 PX4 추력 명령의 중앙값)
    if isnan(px.thr_hover)
        thr_s = nan(1, px.rate_hz);
        for kk = 1:px.rate_hz
            link.sendPosition(p_pre_ned, yaw0 + rot);
            thr_s(kk) = link.getThrustSetpoint();
            pause(Ts);
        end
        thr_meas = median(thr_s, 'omitnan');
        if ~isnan(thr_meas) && thr_meas > 0.05 && thr_meas < 0.95
            thr_hover = thr_meas;
            fprintf('호버 추력 (측정) = %.3f  -> 사용\n', thr_hover);
        else
            if isnan(thr_param)
                error('호버 추력을 측정·읽지 못했습니다. cfg.px4.thr_hover 에 값을 직접 넣어 주세요.');
            end
            thr_hover = thr_param;
            warning('호버 추력 측정 실패 -> MPC_THR_HOVER = %.3f 사용', thr_hover);
        end
    end
    meta.thr_hover = thr_hover;
    U1_to_thr = @(U1) min(max(thr_hover * U1 / (cfg.m * g), px.thr_limits(1)), px.thr_limits(2));

    %% ③ 프리롤 가속 + ④ 경로 추종 (MATLAB PositionCtrl 또는 PX4 위치제어기)
    disp('③ 프리롤 가속');
    posCtrl.Reset();
    phase = 3;
    [~, t3_start] = link.getState();
    t_prev = t3_start - Ts;                            % 첫 루프에서 바로 PositionCtrl 이 계산되도록
    last_att = [0 0 yaw0 cfg.m * g];
    loop_t = tic; k = 0;
    while true
        k = k + 1;
        [x_ned, t_boot, ok] = link.getState();
        if ~ok, wait_tick(); continue; end
        x = ned_to_path(x_ned, cfg);

        % 기준값
        if phase == 3
            tau = t_boot - t3_start;
            if tau >= t_pre
                phase = 4; meta.t4_start = t_boot;
                disp('④ 경로 추종 (평가 구간)');
            else
                if tau < t_acc, s = -L + 0.5 * a * tau^2;       v_des = a * tau * d; a_des = a * d;
                else,           s = -L + s_acc + v * (tau - t_acc); v_des = v * d;     a_des = [0; 0; 0];
                end
                cmd = [p_start + s * d; yaw0; v; 0];
            end
        end
        if phase == 4
            tau = t_boot - meta.t4_start;
            if tau >= ref.tf
                meta.t4_end = t_boot;
                break;
            end
            p_des = interp1(ref.t, ref.pos', tau)';
            v_des = interp1(ref.t, ref.vel', tau)';
            a_des = interp1(ref.t, ref.acc', tau)';
            cmd = [p_des; interp1(ref.t, ref.yaw, tau); v; interp1(ref.t, ref.kappa, tau)];
        end

        % 안전 감시
        if norm(x(1:3) - cmd(1:3)) > px.abort_err || max(abs(x(7:8))) > deg2rad(px.abort_tilt_deg)
            meta.aborted = true;
            meta.abort_reason = sprintf('phase %d, 위치오차 %.1f m, 기울기 %.0f deg', ...
                phase, norm(x(1:3) - cmd(1:3)), rad2deg(max(abs(x(7:8)))));
            warning('중단: %s', meta.abort_reason);
            meta.t4_end = t_boot;
            break;
        end

        if usePX4
            % 비교군: PX4 위치제어기에 위치·속도·가속도 셋포인트 전달 (자세·추력은 PX4 가 계산)
            link.sendPosition(path_to_ned(cmd(1:3), cfg), cmd(4) + rot, ...
                              path_to_ned(v_des, cfg, false), path_to_ned(a_des, cfg, false));
            last_att = nan(1, 4); thr = NaN; kz = nan(1, 3);
        else
            % PositionCtrl (실제 dt 반영: 적분·미분이 루프 주기와 무관하게 시뮬레이션과 같은 의미가 되도록)
            dt = t_boot - t_prev;
            if dt > 1e-4
                posCtrl.dt = min(dt, 0.1);
                att = posCtrl.Update(cmd, x);          % [phi; theta; psi; U1]
                last_att = att(:)';
                t_prev = t_boot;
            end
            thr = U1_to_thr(last_att(4));
            link.sendAttitude(last_att(1), last_att(2), last_att(3) + rot, thr);
            kz = posCtrl.GetZ_PID()';
        end

        % 로그 (PX4 모드에서는 자세 명령·추력·Z 게인 열이 NaN -> ULog 의 PX4 자세 셋포인트로 대체)
        nlog = nlog + 1;
        LOG(nlog, :) = [t_boot, phase, x', cmd', last_att, thr, kz];
        wait_tick();
    end

    %% ⑤ 호버·착륙 (PX4 위치제어 -> AUTO.LAND)
    disp('⑤ 호버·착륙');
    [x_ned, ~] = link.getState();
    t5 = tic;
    while toc(t5) < 3
        link.sendPosition(x_ned(1:3), x_ned(9)); pause(Ts);
    end
    link.setLand();
    pause(1);

    meta.log = LOG(1:nlog, :);
    meta.ref = rmfield(ref, 'traj');
    meta.cfg = cfg;
    meta.t3_start = t3_start;
    if ~meta.aborted
        fprintf('④ 구간: t_boot %.2f ~ %.2f s (%.1f s)\n', meta.t4_start, meta.t4_end, meta.t4_end - meta.t4_start);
    end

    %% ---- nested helpers ----
    function wait_tick()
        % 고정 주기 유지 (MATLAB은 실시간 OS가 아니라서 몇 ms 흔들림은 남는다)
        rem_t = k * Ts - toc(loop_t);
        if rem_t > 0, pause(rem_t); end
    end

    function hold_until(p_ned, yaw_ned, tol_pos, hold_s, timeout_s)
        % PX4 위치 셋포인트를 보내면서, 목표 근처에 hold_s 초 머무를 때까지 기다린다.
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
