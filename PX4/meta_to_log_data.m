function log_data = meta_to_log_data(meta, dt_grid)
    % META_TO_LOG_DATA  MATLAB 비행 기록(meta.log)만으로 finalize_and_save_results 용 log_data 를 만든다
    %   착륙 전(ULog 가 아직 닫히지 않음)에 바로 결과를 보기 위한 용도. ④ 구간만, dt_grid 간격으로 다시 샘플링
    %   - state_hist  : 루프에서 받은 LOCAL_POSITION_NED + ATTITUDE (경로 좌표계, 50 Hz 수신값)
    %   - u_hist(1)   : PositionCtrl 추력 명령 U1 [N] (PX4 실제 추력 아님), u_hist(2:4) = NaN
    %   ULog 기반 결과(ulog_to_log_data)와 같은 필드 구성
    if nargin < 2, dt_grid = 0.02; end
    cfg = meta.cfg;
    C   = meta.log;
    col = @(name) find(strcmp(meta.log_cols, name), 1);
    C   = C(C(:, col('phase')) == 4, :);
    if isempty(C), error('meta.log 에 ④ 구간 기록이 없습니다.'); end
    tl  = C(:, col('t_boot'));
    [tl, iu] = unique(tl); C = C(iu, :);

    tg = (max(meta.t4_start, tl(1)) : dt_grid : min(meta.t4_end, tl(end)))';
    N  = numel(tg);
    pick = @(names, method) interp1(tl, C(:, cellfun(col, names)), tg, method, 'extrap')';

    st = pick({'x','y','z','vx','vy','vz','phi','theta','psi','p','q','r'}, 'linear');
    log_data.t_hist       = (tg - meta.t4_start)';
    log_data.state_hist   = st;
    log_data.pos_des_hist = pick({'x_des', 'y_des', 'z_des'}, 'linear');
    log_data.att_des_hist = pick({'phi_cmd', 'theta_cmd', 'psi_cmd', 'U1_cmd'}, 'linear');
    log_data.u_hist       = [log_data.att_des_hist(4, :); nan(3, N)];
    log_data.PID_Pos_hist = pick({'kP_z', 'kI_z', 'kD_z'}, 'previous');
    log_data.kappa_hist   = pick({'kappa'}, 'linear');
    if any(strcmp(meta.log_cols, 'kappa_raw'))
        log_data.kappa_raw_hist = pick({'kappa_raw'}, 'linear');
    else
        log_data.kappa_raw_hist = log_data.kappa_hist;
    end
    if any(strcmp(meta.log_cols, 'a_P_xy'))
        log_data.action_hist = pick({'a_P_xy', 'a_D_xy', 'a_P_z', 'a_D_z'}, 'previous');
    else
        log_data.action_hist = nan(4, N);
    end
    if any(strcmp(meta.log_cols, 'thr_norm'))
        log_data.thr_norm_hist = pick({'thr_norm'}, 'nearest');
        log_data.thr_limits    = cfg.px4.thr_limits;
    end
end
