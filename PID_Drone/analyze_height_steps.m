function [S, lines] = analyze_height_steps(log_data, cfg)
    % ANALYZE_HEIGHT_STEPS  고도 계단 실험의 계단별 응답 지표
    %   CircleTrajectory 고도 계단(cfg.height_step, cfg.height_order)과 같은 규칙으로 계단 시각을 계산하고,
    %   계단마다 아래 지표를 구한다. (t_hist 는 경로 추종 시작 = 0 기준)
    %
    %   [고도 응답]   rise_time  : 계단의 10% -> 90% 도달 시간 [s]
    %                 overshoot  : (최대 초과량 / |dh|) x 100 [%]
    %                 settle_time: 오차가 +-band 안에 들어와 계속 머무는 데 걸린 시간 [s] (band = cfg.height_settle_band x |dh|)
    %                 ss_err     : 다음 계단 직전 마지막 3 s 의 평균 고도 오차 [m] (기준 - 실제, 양수 = 낮음)
    %                 IAE_norm   : 계단 후 10 s 동안 int|e_h| dt / |dh| [s]
    %   [결합·부작용] cross_pk / along_pk : 계단 후 5 s 동안 횡방향·진행방향 오차 절댓값 최대 [m]
    %                 cross_pre / along_pre: 계단 전 3 s 평균 (비교 기준) [m]
    %                 dU1_pk     : 계단 후 5 s 동안 추력 명령 최대 변화 (계단 전 3 s 평균 대비) [N]
    %   [유효성]      sat_frac   : 계단 구간에서 정규화 추력이 thr_limits 에 걸린 비율 (log_data.thr_norm_hist 가 있을 때)
    %
    %   S     : 계단별 지표 구조체 배열
    %   lines : 콘솔·보고서 출력용 문자열 셀

    S = struct([]); lines = {};
    if ~isfield(cfg, 'height_order') || isempty(cfg.height_order), return; end

    dirs = CircleTrajectory.parse_height_order(cfg.height_order);
    dh_abs = cfg.height_step;
    if isfield(cfg, 'height_settle_band'), band_rel = cfg.height_settle_band; else, band_rel = 0.10; end

    % 계단 시각 (CircleTrajectory 와 같은 식)
    if isfield(cfg, 'cruise_time'), cruise_t = cfg.cruise_time; else, cruise_t = 0; end
    t_c0    = 2 * cfg.accel_dist_per_speed + cruise_t;
    T_level = circle_level_length(cfg) / cfg.target_speed;   % 원: n_laps 바퀴, 직선: straight_distance
    t_steps = t_c0 + (1:numel(dirs)) * T_level;

    t   = log_data.t_hist(:)';
    h   = -log_data.state_hist(3, :);           % 실제 고도 (NED z -> 위쪽 양수)
    h_r = -log_data.pos_des_hist(3, :);         % 기준 고도
    e_h = h_r - h;

    % 진행방향·횡방향 오차 (기준 경로 접선 기준)
    dx = gradient(log_data.pos_des_hist(1, :), t);
    dy = gradient(log_data.pos_des_hist(2, :), t);
    sp = max(hypot(dx, dy), 1e-6);
    Tx = dx ./ sp; Ty = dy ./ sp;
    ex = log_data.state_hist(1, :) - log_data.pos_des_hist(1, :);
    ey = log_data.state_hist(2, :) - log_data.pos_des_hist(2, :);
    e_along = Tx .* ex + Ty .* ey;
    e_cross = -Ty .* ex + Tx .* ey;

    U1 = log_data.att_des_hist(4, :);
    has_thr = isfield(log_data, 'thr_norm_hist') && isfield(log_data, 'thr_limits');
    if has_thr
        thr = log_data.thr_norm_hist(:)';
        lim = log_data.thr_limits;
        is_sat = thr <= lim(1) + 1e-4 | thr >= lim(2) - 1e-4;
    end

    h0 = -cfg.drone1_init_states(3);
    h_levels = h0 + [0, cumsum(dirs)] * dh_abs;

    for j = 1:numel(dirs)
        ts = t_steps(j);
        if j < numel(dirs), te = t_steps(j + 1); else, te = t(end); end
        if ts >= t(end), break; end
        win  = t >= ts & t < te;
        pre  = t >= ts - 3 & t < ts;
        tw   = t(win) - ts;
        dh   = h_levels(j + 1) - h_levels(j);
        y    = (h(win) - h_levels(j)) / dh;   % 정규화 응답 (0 -> 1)

        s.index     = j;
        s.dir       = dirs(j);
        s.t_step    = ts;
        s.h_from    = h_levels(j);
        s.h_to      = h_levels(j + 1);
        s.rise_time = local_cross(tw, y, 0.9) - local_cross(tw, y, 0.1);
        s.overshoot = max(0, max(y) - 1) * 100;

        out = abs(y - 1) > band_rel;
        if ~any(out)
            s.settle_time = 0;
        elseif out(end)
            s.settle_time = NaN;                % 구간 끝까지 band 밖
        else
            s.settle_time = tw(find(out, 1, 'last') + 1);
        end

        last3 = win & t >= te - 3;
        s.ss_err = mean(e_h(last3));

        w10 = win & t < ts + 10;
        s.IAE_norm = trapz(t(w10), abs(e_h(w10))) / abs(dh);

        w5 = win & t < ts + 5;
        s.cross_pk  = max(abs(e_cross(w5)));
        s.along_pk  = max(abs(e_along(w5)));
        s.cross_pre = mean(abs(e_cross(pre)));
        s.along_pre = mean(abs(e_along(pre)));
        s.dU1_pk    = max(abs(U1(w5) - mean(U1(pre), 'omitnan')));
        if has_thr, s.sat_frac = mean(is_sat(win)); else, s.sat_frac = NaN; end

        if isempty(S), S = s; else, S(end + 1) = s; end %#ok<AGROW>
    end

    %% 출력
    if isempty(S), return; end
    lines{end+1, 1} = sprintf('[고도 계단] step %.2f m, 단계마다 %.1f m (%.1f s), 정착 band +-%.0f%%', dh_abs, circle_level_length(cfg), T_level, band_rel * 100);
    lines{end+1, 1} = ' #  방향  t_step   h [m]          t_r[s]  M_p[%]  t_s[s]  ss_err[m]  IAE/dh  cross_pk(pre)  along_pk(pre)  dU1_pk[N]  sat[%]';
    for j = 1:numel(S)
        s = S(j);
        if s.dir > 0, d = 'up  '; else, d = 'down'; end
        lines{end+1, 1} = sprintf('%2d  %s  %6.1f  %5.2f -> %5.2f  %6.2f  %6.1f  %6.2f  %+9.4f  %6.2f  %5.3f(%5.3f)  %5.3f(%5.3f)  %8.2f  %6.1f', ...
            s.index, d, s.t_step, s.h_from, s.h_to, s.rise_time, s.overshoot, s.settle_time, s.ss_err, s.IAE_norm, ...
            s.cross_pk, s.cross_pre, s.along_pk, s.along_pre, s.dU1_pk, s.sat_frac * 100);
    end
    for dsel = [1, -1]
        k = [S.dir] == dsel;
        if ~any(k), continue; end
        if dsel > 0, d = '상승'; else, d = '하강'; end
        lines{end+1, 1} = sprintf('%s 평균(n=%d): t_r %.2f +- %.2f s, M_p %.1f +- %.1f %%, t_s %.2f +- %.2f s', d, nnz(k), ...
            mean([S(k).rise_time], 'omitnan'), std([S(k).rise_time], 'omitnan'), ...
            mean([S(k).overshoot], 'omitnan'), std([S(k).overshoot], 'omitnan'), ...
            mean([S(k).settle_time], 'omitnan'), std([S(k).settle_time], 'omitnan'));
    end
    if has_thr && any([S.sat_frac] > 0)
        lines{end+1, 1} = '※ 추력 포화가 발생한 계단이 있습니다. 선형 범위 비교가 아니므로 height_step 을 줄이세요.';
    end
end

function tc = local_cross(t, y, level)
    % y 가 처음으로 level 이상이 되는 시각 (선형 보간). 없으면 NaN
    i = find(y >= level, 1);
    if isempty(i), tc = NaN; return; end
    if i == 1, tc = t(1); return; end
    tc = t(i-1) + (level - y(i-1)) * (t(i) - t(i-1)) / (y(i) - y(i-1));
end
