function log_data = ulog_to_log_data(ulgFile, meta, dt_grid)
% ULOG_TO_LOG_DATA  PX4 ULog(.ulg) -> PID_Drone 시뮬레이션과 같은 형식의 log_data
%   ④ 경로 추종 구간(meta.t4_start ~ meta.t4_end)만 잘라서, dt_grid 간격으로 다시 샘플링한다.
%   결과는 PID_Drone/finalize_and_save_results(log_data, cfg) 에 그대로 넣을 수 있다.
%
%   출처
%   - state_hist (실제 상태)          : ULog  vehicle_local_position / vehicle_attitude / vehicle_angular_velocity
%   - pos_des, att_des, PID, kappa    : MATLAB 이 비행 중 기록한 meta.log (ULog 에는 PositionCtrl 내부값이 없음)
%   - u_hist(1)  추력 [N]             : ULog  vehicle_thrust_setpoint (정규화값 -> N 근사 변환)
%   - u_hist(2:4) 토크                : ULog  vehicle_torque_setpoint (정규화값 -1 ~ 1, N·m 아님)
%
%   필요: UAV Toolbox (ulogreader)

    if nargin < 3, dt_grid = 0.01; end
    cfg = meta.cfg;
    if isnan(meta.t4_start) || isnan(meta.t4_end)
        error('meta 에 ④ 구간 시간이 없습니다 (중단된 비행일 수 있음).');
    end

    ulog = ulogreader(ulgFile);
    lp   = local_topic(ulog, 'vehicle_local_position');
    att  = local_topic(ulog, 'vehicle_attitude');
    angv = local_topic(ulog, 'vehicle_angular_velocity');

    tg = (meta.t4_start : dt_grid : meta.t4_end)';     % PX4 부팅 후 시간 [s]
    N  = numel(tg);

    %% 실제 상태 (NED -> 경로 좌표계)
    t_lp = local_time(lp);
    if meta.t4_start < t_lp(1) || meta.t4_end > t_lp(end)
        error(['④ 구간(%.2f ~ %.2f s)이 ULog 시간 범위(%.2f ~ %.2f s) 밖입니다. ' ...
               '다른 비행의 ULog 이거나, 시간 기준(부팅 후 시간)이 다를 수 있습니다.'], ...
               meta.t4_start, meta.t4_end, t_lp(1), t_lp(end));
    end
    P =interp1(t_lp, [lp.x, lp.y, lp.z], tg, 'linear', 'extrap');
    V = interp1(t_lp, [lp.vx, lp.vy, lp.vz], tg, 'linear', 'extrap');

    q = double(att.q);                                 % [w x y z]
    eul = local_quat2eul(q);                           % [phi theta psi]
    eul(:, 3) = unwrap(eul(:, 3));
    E = interp1(local_time(att), eul, tg, 'linear', 'extrap');

    W = interp1(local_time(angv), double(angv.xyz), tg, 'linear', 'extrap');

    state_hist = zeros(12, N);
    for i = 1:N
        x_ned = [P(i, :)'; V(i, :)'; E(i, :)'; W(i, :)'];
        state_hist(:, i) = ned_to_path(x_ned, cfg);
    end

    %% MATLAB 기록값 (④ 구간)
    C = meta.log;
    col = @(name) find(strcmp(meta.log_cols, name), 1);
    sel = C(:, col('phase')) == 4;
    C = C(sel, :);
    tl = C(:, col('t_boot'));
    [tl, iu] = unique(tl); C = C(iu, :);
    pick = @(names) interp1(tl, C(:, cellfun(col, names)), tg, 'linear', 'extrap')';

    pos_des_hist = pick({'x_des', 'y_des', 'z_des'});
    if isfield(cfg, 'mode') && strcmpi(cfg.mode, 'PX4')
        % PX4 모드: 자세 명령은 PX4 위치제어기가 만든 vehicle_attitude_setpoint 에서 가져온다
        asp = local_topic(ulog, 'vehicle_attitude_setpoint');
        eul_d = local_quat2eul(double(asp.q_d));
        eul_d(:, 3) = unwrap(eul_d(:, 3)) - deg2rad(cfg.px4.path_rot_deg);
        U1_d = -double(asp.thrust_body(:, 3)) / meta.thr_hover * cfg.m * 9.81;   % 정규화 추력 -> N 근사
        att_des_hist = interp1(local_time(asp), [eul_d, U1_d], tg, 'linear', 'extrap')';
    else
        att_des_hist = pick({'phi_cmd', 'theta_cmd', 'psi_cmd', 'U1_cmd'});
    end
    pid_hist     = pick({'kP_z', 'kI_z', 'kD_z'});
    kappa_hist   = pick({'kappa'});
    if any(strcmp(meta.log_cols, 'kappa_raw'))
        kappa_raw_hist = pick({'kappa_raw'});                  % 댐핑 전 전방 곡률
    else
        kappa_raw_hist = kappa_hist;
    end
    if any(strcmp(meta.log_cols, 'a_P_xy'))
        action_hist = pick({'a_P_xy', 'a_D_xy', 'a_P_z', 'a_D_z'});   % PPO 액션 (PPO 모드 외에는 NaN)
    else
        action_hist = nan(4, N);
    end

    %% 제어 입력 (PX4 출력)
    u_hist = nan(4, N);
    try
        thr = local_topic(ulog, 'vehicle_thrust_setpoint');
        thr_z = double(thr.xyz(:, 3));                 % 정규화, 위쪽 추력이 음수
        u_hist(1, :) = interp1(local_time(thr), -thr_z / meta.thr_hover * cfg.m * 9.81, tg, 'linear', 'extrap');
    catch err
        warning('vehicle_thrust_setpoint 를 읽지 못했습니다: %s', err.message);
    end
    try
        trq = local_topic(ulog, 'vehicle_torque_setpoint');
        u_hist(2:4, :) = interp1(local_time(trq), double(trq.xyz), tg, 'linear', 'extrap')';
    catch err
        warning('vehicle_torque_setpoint 를 읽지 못했습니다: %s', err.message);
    end

    %% 시뮬레이션과 같은 구조체
    log_data.t_hist       = (tg - meta.t4_start)';
    log_data.state_hist   = state_hist;
    log_data.pos_des_hist = pos_des_hist;
    log_data.att_des_hist = att_des_hist;
    log_data.u_hist       = u_hist;
    log_data.PID_Pos_hist = pid_hist;
    log_data.kappa_hist   = kappa_hist;
    log_data.kappa_raw_hist = kappa_raw_hist;
    log_data.action_hist  = action_hist;
end

%% ---- local functions ----
function TT = local_topic(ulog, name)
    d = readTopicMsgs(ulog, 'TopicNames', {name}, 'InstanceID', {0});
    if isempty(d) || height(d) == 0
        error('ULog 에 %s 토픽이 없습니다.', name);
    end
    TT = d.TopicMessages{1};
end

function t = local_time(TT)
    % ulogreader 의 행 시간(부팅 후 duration) -> [s]
    t = seconds(TT.Properties.RowTimes);
end

function eul = local_quat2eul(q)
    % 쿼터니언 [w x y z] -> ZYX 오일러각 [phi theta psi]
    w = q(:, 1); x = q(:, 2); y = q(:, 3); z = q(:, 4);
    phi   = atan2(2*(w.*x + y.*z), 1 - 2*(x.^2 + y.^2));
    theta = asin(max(min(2*(w.*y - z.*x), 1), -1));
    psi   = atan2(2*(w.*z + x.*y), 1 - 2*(y.^2 + z.^2));
    eul = [phi, theta, psi];
end
