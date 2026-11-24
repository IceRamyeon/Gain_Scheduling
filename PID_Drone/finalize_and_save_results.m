function finalize_and_save_results(log_data, cfg)
    % FINALIZE_AND_SAVE_RESULTS 최종 결과 그래프 출력 및 자동 저장 함수
    
    
    R2D = 180/pi;
    
    % 데이터 언패킹 
    t_hist       = log_data.t_hist;
    state_hist   = log_data.state_hist;
    pos_des_hist = log_data.pos_des_hist;
    att_des_hist = log_data.att_des_hist;
    u_hist       = log_data.u_hist;
    pid_hist     = log_data.PID_Pos_hist;
    kappa_hist   = log_data.kappa_hist;

    %% 0. 평가 지표 계산
    % 평가 구간: 원 궤적(path_type = 'circle')이면 가속 직선·탈출 직선을 빼고 원 구간만 평가
    %  -> CircleTrajectory 와 같은 식: 원 진입 t_acc + cruise_time (t_acc = 2 * accel_dist_per_speed), 원 탈출 = 진입 + n 바퀴 길이 / v
    %     (t_hist 는 0 에서 시작)
    is_circle = isfield(cfg, 'path_type') && strcmpi(cfg.path_type, 'circle');
    if is_circle
        if isfield(cfg, 'cruise_time'), cruise_t = cfg.cruise_time; else, cruise_t = 0; end
        t_eval0  = 2 * cfg.accel_dist_per_speed + cruise_t;
        [~, L_seg] = circle_level_length(cfg);                   % 원(직선) 구간 전체 길이 (고도 계단 단계 수 반영)
        t_eval1  = t_eval0 + L_seg / cfg.target_speed;
        eval_idx = t_hist >= t_eval0 & t_hist <= t_eval1;
        if ~any(eval_idx)
            warning('원 구간(t = %.2f ~ %.2f s) 데이터가 없어 전체 구간으로 평가합니다.', t_eval0, t_eval1);
            eval_idx = true(size(t_hist)); t_eval0 = t_hist(1); t_eval1 = t_hist(end);
        end
    else
        t_eval0  = t_hist(1);
        t_eval1  = t_hist(end);
        eval_idx = true(size(t_hist));
    end

    % 3차원 유클리디안 거리 오차 계산 블록
    dist_error = sqrt(sum((state_hist(1:3, :) - pos_des_hist(1:3, :)).^2, 1));
    
    % 평가 지표 계산 (평가 구간만)
    val_rmse = sqrt(mean(dist_error(eval_idx).^2));
    val_mae  = mean(dist_error(eval_idx));
    val_max  = max(dist_error(eval_idx));
    
    % Z축(고도)만의 오차
    z_error = abs(state_hist(3, :) - pos_des_hist(3, :));
    val_rmse_z = sqrt(mean(z_error(eval_idx).^2));
    val_mae_z  = mean(z_error(eval_idx));
    val_max_z  = max(z_error(eval_idx));
    
    % 평형점(Trim): PositionCtrl 과 같은 식, v = cfg.target_speed, kappa = Feedforward 입력 곡률(kappa_hist)
    g_trim = 9.81;
    v_trim = cfg.target_speed;
    a_cen  = v_trim^2 .* kappa_hist;                      % 구심 가속도 v^2 kappa
    phi_eq_hist = atan(a_cen ./ g_trim);                  % PositionCtrl: phi_eq = atan(+(v^2 kappa)/g) (NED, kappa > 0 -> +phi)
    U1_eq_hist  = cfg.m .* sqrt(g_trim^2 + a_cen.^2);     % PositionCtrl: U1_eq  = m sqrt(g^2 + (v^2 kappa)^2)
    turn_idx = abs(kappa_hist) > 1e-4;                    % 곡선(선회) 구간

    %% 결과 요약 (콘솔 출력 + 자동 저장 시 .txt 로 저장)
    % 헤더: 경로·모드 정보 (콘솔에는 이미 앞에서 출력되므로 .txt 에만 기록)
    hdr = {};
    if is_circle
        R_c   = 1 / abs(cfg.curvature);                    % 직선 모드(curvature = 0)면 Inf
        L_acc = cfg.accel_dist_per_speed * cfg.target_speed;
        a_acc = cfg.target_speed^2 / (2 * L_acc);
        t_acc = 2 * cfg.accel_dist_per_speed;
        [~, L_cir] = circle_level_length(cfg);
        if isfield(cfg, 'cruise_time'), cruise_t = cfg.cruise_time; else, cruise_t = 0; end
        L_crs = cfg.target_speed * cruise_t;                % 등속 직선
        L_ext = L_acc + L_crs;                              % 탈출 직선 (초반 직선과 같은 길이)
        t_c0  = t_acc + cruise_t;
        t_c1  = t_c0 + L_cir / cfg.target_speed;
        tf_c  = t_c1 + L_ext / cfg.target_speed;
        if abs(cfg.curvature) < 1e-6
            hdr{end+1, 1} = sprintf('[CircleTrajectory] 직선 모드: 가속 %.1f m (%.2f m/s^2, %.1f s), 등속 직선 %.1f m (%.1f s), 계단 구간 직진 %.1f m (%.1f ~ %.1f s), 탈출 직선 %.1f m, tf = %.1f s', ...
                L_acc, a_acc, t_acc, L_crs, cruise_t, L_cir, t_c0, t_c1, L_ext, tf_c);
        else
            hdr{end+1, 1} = sprintf('[CircleTrajectory] R = %.2f m, 가속 %.1f m (%.2f m/s^2, %.1f s), 등속 직선 %.1f m (%.1f s), 원 %d바퀴 %.1f m (%.1f ~ %.1f s), 탈출 직선 %.1f m, tf = %.1f s', ...
                R_c, L_acc, a_acc, t_acc, L_crs, cruise_t, circle_total_laps(cfg), L_cir, t_c0, t_c1, L_ext, tf_c);
        end
        hdr{end+1, 1} = sprintf('경로 길이 %.1f m, 추종 시간 %.1f s', L_acc + L_crs + L_cir + L_ext, tf_c);
    else
        hdr{end+1, 1} = sprintf('경로: spline (pathGen), 추종 시간 %.1f s', t_hist(end) - t_hist(1));
    end
    if isfield(cfg, 'mode')
        hdr{end+1, 1} = sprintf('제어 모드: %s', cfg.mode);
    else
        hdr{end+1, 1} = '제어 모드: MATLAB 시뮬레이션 (PID_Drone)';
    end
    % Feedforward / Z 게인 스케줄링 적용 여부 (PositionCtrl 설정, PX4 모드에서는 PositionCtrl 을 쓰지 않음)
    onoff = @(f) ternary_onoff(cfg, f);
    if isfield(cfg, 'mode') && strcmpi(cfg.mode, 'PX4')
        note = ' (PX4 모드: PX4 위치제어기 사용, 이 설정은 미적용)';
    elseif isfield(cfg, 'mode') && strcmpi(cfg.mode, 'PPO')
        note = ' (PPO 모드: PositionCtrl_PX4 + PPO 잔차 사용, 이 설정은 미적용)';
        hdr{end+1, 1} = sprintf('PPO 에이전트: %s, 잔차 적용: %s', cfg.ppo.agent_file, cfg.ppo.on_mode);
    else
        note = '';
    end
    hdr{end+1, 1} = sprintf('Feedforward: %s, Z_Gain_Scheduling: %s%s', onoff('Feedforward'), onoff('Z_Gain_Scheduling'), note);
    gf = @(f, d) local_getfield(cfg, f, d);
    hdr{end+1, 1} = sprintf('max_angle_deg: %g, Err_Yaw: %s, Eq_Delay_Comp: %d (K_att %g), Z_D_on_measurement: %d', ...
        gf('max_angle_deg', 30), char(string(gf('Err_Yaw', 'measured'))), gf('Eq_Delay_Comp', false), gf('K_att', 6.5), gf('Z_D_on_measurement', true));

    rpt = {};
    rpt{end+1, 1} = '[최종 결과].';
    rpt{end+1, 1} = '----------------------------------';
    if is_circle
        rpt{end+1, 1} = sprintf('[평가 구간] 원 구간만: t = %.2f ~ %.2f s (가속·등속 직선 0 ~ %.2f s, 탈출 직선 %.2f s ~ 제외)', t_eval0, t_eval1, t_eval0, t_eval1);
    else
        rpt{end+1, 1} = sprintf('[평가 구간] 전체: t = %.2f ~ %.2f s', t_hist(1), t_hist(end));
    end
    rpt{end+1, 1} = '[3D 위치 오차]';
    rpt{end+1, 1} = sprintf('RMSE: %.4f m', val_rmse);
    rpt{end+1, 1} = sprintf('MAE:  %.4f m', val_mae);
    rpt{end+1, 1} = sprintf('Max:  %.4f m', val_max);
    rpt{end+1, 1} = '[Z축 오차]';
    rpt{end+1, 1} = sprintf('RMSE: %.4f m', val_rmse_z);
    rpt{end+1, 1} = sprintf('MAE:  %.4f m', val_mae_z);
    rpt{end+1, 1} = sprintf('Max:  %.4f m', val_max_z);
    rpt{end+1, 1} = '[평형점 (Trim)]';
    rpt{end+1, 1} = sprintf('v = %.2f m/s, m = %.4f kg', v_trim, cfg.m);
    if is_circle
        % 원 궤적: 평가 구간(원 구간) 평균
        rpt{end+1, 1} = sprintf('phi_eq 원 구간 평균: %.3f deg', mean(phi_eq_hist(eval_idx)) * R2D);
        rpt{end+1, 1} = sprintf('U1_eq  원 구간 평균: %.4f N',   mean(U1_eq_hist(eval_idx)));
    else
        ln = sprintf('phi_eq 평균: %.3f deg', mean(phi_eq_hist) * R2D);
        if any(turn_idx), ln = [ln, sprintf('  (선회 구간 평균: %.3f deg)', mean(phi_eq_hist(turn_idx)) * R2D)]; end
        rpt{end+1, 1} = ln;
        ln = sprintf('U1_eq  평균: %.4f N', mean(U1_eq_hist));
        if any(turn_idx), ln = [ln, sprintf('  (선회 구간 평균: %.4f N)', mean(U1_eq_hist(turn_idx)))]; end
        rpt{end+1, 1} = ln;
    end
    rpt{end+1, 1} = '----------------------------------';
    % 고도 계단 실험: 계단별 응답 지표 (analyze_height_steps)
    if is_circle && isfield(cfg, 'height_order') && ~isempty(cfg.height_order)
        [~, step_lines] = analyze_height_steps(log_data, cfg);
        rpt = [rpt; step_lines(:); {'----------------------------------'}];
    end
    fprintf('%s\n', rpt{:});
    
    %% 1. [그래프 1] 위치 추종 (Position Tracking)
    h_fig_pos = figure('Name', 'Position Tracking', 'Theme', 'light', 'Position', [10 100 600 600]);

    subplot(3,1,1);
    plot(t_hist, pos_des_hist(1,:), 'r--', 'LineWidth', 1.5); hold on;
    plot(t_hist, state_hist(1,:), 'b', 'LineWidth', 1.2);
    ylabel('X [m]'); title('Position X: Desired vs Actual'); grid on; legend('Desired','Actual');

    subplot(3,1,2);
    plot(t_hist, pos_des_hist(2,:), 'r--', 'LineWidth', 1.5); hold on;
    plot(t_hist, state_hist(2,:), 'b', 'LineWidth', 1.2);
    ylabel('Y [m]'); title('Position Y: Desired vs Actual'); grid on;

    subplot(3,1,3);
    plot(t_hist, pos_des_hist(3,:), 'r--', 'LineWidth', 1.5); hold on;
    plot(t_hist, state_hist(3,:), 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Z [m]'); title('Position Z: Desired vs Actual'); grid on;

    %% 2. [그래프 2] 자세 추종 (Attitude Tracking)
    h_fig_att = figure('Name', 'Attitude Tracking', 'Theme', 'light', 'Position', [610 100 600 600]);

    subplot(3,1,1);
    plot(t_hist, att_des_hist(1,:)*R2D, 'r--', 'LineWidth', 1.5); hold on;
    plot(t_hist, state_hist(7,:)*R2D, 'b', 'LineWidth', 1.2);
    ylabel('Roll (\phi) [deg]'); title('Roll: Desired vs Actual'); grid on; legend('Command','Response');

    subplot(3,1,2);
    plot(t_hist, att_des_hist(2,:)*R2D, 'r--', 'LineWidth', 1.5); hold on;
    plot(t_hist, state_hist(8,:)*R2D, 'b', 'LineWidth', 1.2);
    ylabel('Pitch (\theta) [deg]'); title('Pitch: Desired vs Actual'); grid on;

    subplot(3,1,3);
    plot(t_hist, att_des_hist(3,:)*R2D, 'r--', 'LineWidth', 1.5); hold on; 
    plot(t_hist, state_hist(9,:)*R2D, 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Yaw (\psi) [deg]'); title('Yaw: Desired vs Actual'); grid on;

    %% 3. [그래프 3] 제어 입력 (Control Inputs)
    h_fig_in = figure('Name', 'Control Inputs', 'Theme', 'light', 'Position', [1010 100 600 600]);

    subplot(4,1,1);
    plot(t_hist, u_hist(1,:), 'g', 'LineWidth', 1.2); 
    ylabel('Thrust [N]'); title('Total Thrust Input (U1)'); grid on;

    subplot(4,1,2);
    plot(t_hist, u_hist(2,:), 'r', 'LineWidth', 1.2);
    ylabel('Mx [N\cdotm]'); title('Rolling Moment (U2)'); grid on;

    subplot(4,1,3);
    plot(t_hist, u_hist(3,:), 'g', 'LineWidth', 1.2); 
    ylabel('My [N\cdotm]'); title('Pitching Moment (U3)'); grid on;

    subplot(4,1,4);
    plot(t_hist, u_hist(4,:), 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Mz [N\cdotm]'); title('Yawing Moment (U4)'); grid on;

    %% 4. [그래프 4] 총 속력 추종 (Total Speed Tracking)
    if isfield(cfg, 'target_speed')
        des_speed = cfg.target_speed * ones(1, length(t_hist));
    else
        des_speed = zeros(1, length(t_hist)); 
    end
    
    act_speed = sqrt(sum(state_hist(4:6, :).^2, 1));

    h_fig_spd = figure('Name', 'Total Speed Tracking', 'Theme', 'light', 'Position', [1410 100 600 300]);
    plot(t_hist, des_speed, 'r--', 'LineWidth', 1.5); hold on;
    plot(t_hist, act_speed, 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Speed [m/s]'); 
    title('Total Speed: Desired vs Actual'); grid on; legend('Desired', 'Actual');

    %% 5. [그래프 5] Z축 스케줄링 PID 게인 로깅
    h_fig_pid = figure('Name', 'Scheduled PID Gains (Z-axis)', 'Theme', 'light', 'Position', [1410 450 600 250]);
    plot(t_hist, pid_hist(1,:), 'r', 'LineWidth', 1.5); hold on;
    plot(t_hist, pid_hist(2,:), 'g', 'LineWidth', 1.5);
    plot(t_hist, pid_hist(3,:), 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Gain Value'); 
    if isfield(cfg, 'mode') && strcmpi(cfg.mode, 'PX4')
        title('PX4 Equivalent Z-axis PID Gains (fixed, x m)'); grid on;   % PX4 모드는 스케줄링 없음 (고정 게인)
    elseif isfield(cfg, 'mode') && strcmpi(cfg.mode, 'PPO')
        title('PositionCtrl\_PX4 Z-axis Gains + PPO Residual (x m)'); grid on;
    else
        title('Scheduled PID Gains for Z-axis (Kp, Ki, Kd)'); grid on;
    end
    if all(isnan(pid_hist(:)))
        text(0.5, 0.5, 'Z 게인 기록 없음 (NaN)', 'Units', 'normalized', 'HorizontalAlignment', 'center');
    end
    legend('K_{P,z}', 'K_{I,z}', 'K_{D,z}', 'Location', 'best');

    %% 6. [그래프 6] 곡률 추종 (Curvature Tracking)
    h_fig_kappa = figure('Name', 'Curvature Tracking', 'Theme', 'light', 'Position', [1410 750 600 250]);
    use_lk = isfield(cfg, 'kappa_lookahead') && ~isempty(cfg.kappa_lookahead);
    if use_lk && isfield(log_data, 'kappa_raw_hist')
        % 전방 곡률 모드: 원값(댐핑 전)과 Feedforward 입력(댐핑 후)을 같이 기록
        plot(t_hist, log_data.kappa_raw_hist, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0); hold on;
        plot(t_hist, kappa_hist, 'k', 'LineWidth', 1.4);
        tau_txt = '';
        if isfield(cfg, 'kappa_tau'), tau_txt = sprintf(', \\tau = %.2g s', cfg.kappa_tau); end
        title(sprintf('Feedforward Curvature (lookahead %.2g m%s)', cfg.kappa_lookahead, tau_txt));
        legend('\kappa raw (lookahead)', '\kappa_{ff} (damped)', 'Location', 'best');
    else
        plot(t_hist, kappa_hist, 'k', 'LineWidth', 1.2);
        title('Trajectory Average Curvature (\kappa_{avg})');
    end
    xlabel('Time [s]'); ylabel('Curvature \kappa [1/m]'); grid on;
    
    %% 6-1 [그래프] 평형점 (phi_eq, U1_eq)
    h_fig_trim = figure('Name', 'Trim (phi_eq, U1_eq)', 'Theme', 'light', 'Position', [1010 450 600 450]);
    subplot(2,1,1);
    plot(t_hist, phi_eq_hist * R2D, 'b', 'LineWidth', 1.2); hold on;
    % yline(mean(phi_eq_hist) * R2D, 'r--', sprintf('mean %.2f deg', mean(phi_eq_hist) * R2D));
    ylabel('\phi_{eq} [deg]'); title(sprintf('Equilibrium Roll \\phi_{eq} (v = %.2g m/s)', v_trim)); grid on;
    subplot(2,1,2);
    plot(t_hist, U1_eq_hist, 'b', 'LineWidth', 1.2); hold on;
    yline(cfg.m * g_trim, 'k:', 'mg');
    % yline(mean(U1_eq_hist), 'r--', sprintf('mean %.3f N', mean(U1_eq_hist)));
    xlabel('Time [s]'); ylabel('U_{1,eq} [N]'); title('Equilibrium Thrust U_{1,eq}'); grid on;

    %% 7 [그래프 7, 8] 3D & 2D Trajectory
    if ~cfg.play_animation
        h_fig_traj_3d = figure('Name', '3D Trajectory', 'Theme', 'light', 'Position', [10 150 600 600]);
        view(3); axis equal; grid on; hold on;
        set(gca, 'ZDir', 'Reverse', 'YDir', 'Reverse');
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Height [m]');
        title('3D Flight Trajectory');

        plot3(pos_des_hist(1,:), pos_des_hist(2,:), pos_des_hist(3,:), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Desired Trajectory');
        plot3(state_hist(1,:), state_hist(2,:), state_hist(3,:), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Simulated Trajectory');
        legend('show', 'Location', 'best');

        h_fig_traj_2d = figure('Name', '2D XY Trajectory', 'Theme', 'light', 'Position', [620 150 600 600]);
        axis equal; grid on; hold on;
        set(gca, 'YDir', 'Reverse');
        xlabel('X [m]'); ylabel('Y [m]');
        title('XY Flight Trajectory (Top View)');
        
        plot(pos_des_hist(1,:), pos_des_hist(2,:), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Desired Trajectory');
        plot(state_hist(1,:), state_hist(2,:), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Simulated Trajectory');
        legend('show', 'Location', 'best');
    end

    %% 6. 자동 저장 로직 (데이터 + 이미지)
    if isfield(cfg, 'auto_save') && cfg.auto_save
        if ~exist(cfg.save_dir, 'dir')
            mkdir(cfg.save_dir);
        end
        
        currentTimeString = datestr(now, 'yyyymmdd_HHMMSS');
        baseFileName = fullfile(cfg.save_dir, sprintf('DroneSimResult_%s', currentTimeString));
        
        % 1) 데이터 저장 (.mat)
        matFileName = [baseFileName, '.mat'];
        save(matFileName, 'log_data', 'cfg');
        
        % 2) 이미지 저장 (.png)
        exportgraphics(h_fig_pos, [baseFileName, '_Position.png'], 'Resolution', 300);
        exportgraphics(h_fig_att, [baseFileName, '_Attitude.png'], 'Resolution', 300);
        exportgraphics(h_fig_in, [baseFileName, '_ControlInput.png'], 'Resolution', 300);
        exportgraphics(h_fig_spd, [baseFileName, '_Speed.png'], 'Resolution', 300);
        exportgraphics(h_fig_pid, [baseFileName, '_PID_Gains.png'], 'Resolution', 300);
        exportgraphics(h_fig_kappa, [baseFileName, '_Curvature.png'], 'Resolution', 300);
        exportgraphics(h_fig_trim, [baseFileName, '_Trim.png'], 'Resolution', 300);

        % 3) 결과 요약 저장 (.txt): 경로·모드 정보 + 평가 지표 + 평형점
        txtFileName = [baseFileName, '_Summary.txt'];
        writecell([hdr; rpt], txtFileName, 'FileType', 'text', 'Delimiter', 'tab', 'Encoding', 'UTF-8');
        
        if ~cfg.play_animation
            exportgraphics(h_fig_traj_3d, [baseFileName, '_Trajectory_3D.png'], 'Resolution', 300);
            exportgraphics(h_fig_traj_2d, [baseFileName, '_Trajectory_XY.png'], 'Resolution', 300);
        end
        
        disp(['으헤~ 데이터랑 그래프 이미지들 전부 저장했어: ', cfg.save_dir]);
    else
        disp('자동 저장은 꺼져 있어서 아무것도 안 남겼어.');
    end
end

%% ---- local functions ----
function str = ternary_onoff(cfg, field)
    % cfg 의 논리 설정값 -> 'ON' / 'OFF' (필드가 없으면 '미설정')
    if ~isfield(cfg, field) || isempty(cfg.(field))
        str = '미설정';
    elseif cfg.(field)
        str = 'ON';
    else
        str = 'OFF';
    end
end

function v = local_getfield(cfg, f, d)
    if isfield(cfg, f) && ~isempty(cfg.(f)), v = cfg.(f); else, v = d; end
end
