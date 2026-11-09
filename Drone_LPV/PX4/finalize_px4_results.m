function finalize_px4_results(log_data, cfg)
    % FINALIZE_PX4_RESULTS 기존 finalize_and_save_results.m 규격과 100% 호환되면서
    % Gazebo SITL 등가 모델(비대칭 모터, 믹서 포화, 통신 지연, 이중 루프 자세 응답, EKF 노이즈)
    % 검증 플롯을 추가로 생성 및 저장하는 함수

    R2D = 180 / pi;

    % 1. 기존 표준 finalize_and_save_results 호출 (표준 8개 그래프 및 기본 .mat 저장)
    finalize_and_save_results(log_data, cfg);

    % 2. PX4 Gazebo-Equivalent 전용 상세 검증 플롯 생성
    t_hist         = log_data.t_hist;
    state_hist     = log_data.state_hist;
    pos_des_hist   = log_data.pos_des_hist;
    att_des_hist   = log_data.att_des_hist;     % 위치 제어기 직후 명령
    att_delay_hist = log_data.att_delayed_hist; % 0.04s(10스텝) 링버퍼 지연 후 자세 제어기 입력 명령
    f_cmd_hist     = log_data.f_cmd_hist;       % 4-로터 목표 추력 (클리핑 후)
    f_act_hist     = log_data.f_act_hist;       % 4-로터 실제 추력 (비대칭 모터 지연 후)
    p_est_hist     = log_data.p_est_hist;       % EKF2 추정 위치
    thr_cmd_hist   = log_data.thr_cmd_hist;     % 정규화 추력 명령 [0.1, 0.9]

    dist_err = sqrt(sum((state_hist(1:3, :) - pos_des_hist(1:3, :)).^2, 1));
    xy_err   = sqrt(sum((state_hist(1:2, :) - pos_des_hist(1:2, :)).^2, 1));
    z_err    = abs(state_hist(3, :) - pos_des_hist(3, :));

    fprintf('\n====================================================================\n');
    fprintf('  [PX4 Gazebo-Equivalent SITL 시뮬레이션 정밀 요약]\n');
    fprintf('  - 자세 제어기 모드 : Mode %d (%s)\n', cfg.att_mode, ...
         ternary_str(cfg.att_mode == 1, sprintf('SysID 등가 모델 (%d차)', cfg.sysid_order), 'PX4 전체 이중 루프 (mc_att + mc_rate)'));
    fprintf('  - 통신/EKF 순수 지연: %.3f s (%d 스텝 @ 250Hz)\n', cfg.delay_steps * cfg.dt_sim, cfg.delay_steps);
    fprintf('  - 센서 노이즈 (M10) : %s\n', ternary_str(cfg.use_sensor_noise, 'ON (10Hz RW + 8Hz EKF2 LPF)', 'OFF (Ground Truth)'));
    fprintf('  - 3D 위치 RMSE      : %.4f m (XY RMSE: %.4f m, Z RMSE: %.4f m)\n', ...
        sqrt(mean(dist_err.^2)), sqrt(mean(xy_err.^2)), sqrt(mean(z_err.^2)));
    fprintf('  - 3D 최대 오차 (MAX): %.4f m\n', max(dist_err));
    fprintf('  - 로터 추력 포화율  : %.2f %% (8.6 N 클리핑 도달 비율)\n', 100 * mean(log_data.sat_rot_hist));
    fprintf('====================================================================\n');

    h_fig_px4 = figure('Name', 'PX4 Gazebo-Equivalent Verification', ...
        'Theme', 'light', 'Position', [80, 60, 1250, 820]);

    % (1) XY 궤적 비교 (목표 vs 실제 vs EKF 추정)
    subplot(2, 3, 1);
    plot(pos_des_hist(1,:), pos_des_hist(2,:), 'k--', 'LineWidth', 1.6, 'DisplayName', 'Desired'); hold on;
    if cfg.use_sensor_noise
        plot(p_est_hist(1,:), p_est_hist(2,:), 'Color', [0.85 0.33 0.10 0.5], 'LineWidth', 1.0, 'DisplayName', 'EKF2 Est (50Hz)');
    end
    plot(state_hist(1,:), state_hist(2,:), 'b-', 'LineWidth', 1.4, 'DisplayName', 'Actual (True)');
    set(gca, 'YDir', 'Reverse'); axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]');
    title('XY Trajectory: Desired vs Actual vs EKF2');
    legend('Location', 'best');

    % (2) 위치 추종 오차 (X, Y, Z 및 3D 오차)
    subplot(2, 3, 2);
    plot(t_hist, pos_des_hist(1,:) - state_hist(1,:), 'r-', 'LineWidth', 1.1, 'DisplayName', 'e_X'); hold on;
    plot(t_hist, pos_des_hist(2,:) - state_hist(2,:), 'g-', 'LineWidth', 1.1, 'DisplayName', 'e_Y');
    plot(t_hist, pos_des_hist(3,:) - state_hist(3,:), 'b-', 'LineWidth', 1.1, 'DisplayName', 'e_Z');
    plot(t_hist, dist_err, 'k-', 'LineWidth', 1.5, 'DisplayName', '||e_{3D}||');
    grid on; xlabel('Time [s]'); ylabel('Error [m]');
    title(sprintf('Position Tracking Error (RMSE = %.3f m)', sqrt(mean(dist_err.^2))));
    legend('Location', 'best');

    % (3) 롤(Roll) 명령 vs 지연된 명령 vs 실제 응답 + ±30도 클리핑 한계
    subplot(2, 3, 3);
    plot(t_hist, att_des_hist(1,:)*R2D, 'r:', 'LineWidth', 1.1, 'DisplayName', '\phi_{cmd} (50Hz ZOH)'); hold on;
    plot(t_hist, att_delay_hist(1,:)*R2D, 'm--', 'LineWidth', 1.2, 'DisplayName', '\phi_{delayed} (FIFO 40ms)');
    plot(t_hist, state_hist(7,:)*R2D, 'b-', 'LineWidth', 1.4, 'DisplayName', '\phi_{actual} (250Hz)');
    yline(cfg.max_angle_deg, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(-cfg.max_angle_deg, 'k--', 'LineWidth', 1.0, 'DisplayName', '\pm30^\circ Limit');
    grid on; xlabel('Time [s]'); ylabel('Roll [deg]');
    title('Roll: 50Hz ZOH & 40ms Delay vs Actual Response');
    legend('Location', 'best');

    % (4) 피치(Pitch) 명령 vs 지연된 명령 vs 실제 응답 + ±30도 클리핑 한계
    subplot(2, 3, 4);
    plot(t_hist, att_des_hist(2,:)*R2D, 'r:', 'LineWidth', 1.1, 'DisplayName', '\theta_{cmd} (50Hz ZOH)'); hold on;
    plot(t_hist, att_delay_hist(2,:)*R2D, 'm--', 'LineWidth', 1.2, 'DisplayName', '\theta_{delayed} (FIFO 40ms)');
    plot(t_hist, state_hist(8,:)*R2D, 'b-', 'LineWidth', 1.4, 'DisplayName', '\theta_{actual} (250Hz)');
    yline(cfg.max_angle_deg, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(-cfg.max_angle_deg, 'k--', 'LineWidth', 1.0, 'DisplayName', '\pm30^\circ Limit');
    grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');
    title('Pitch: 50Hz ZOH & 40ms Delay vs Actual Response');
    legend('Location', 'best');

    % (5) 4-로터 개별 추력 (f_act) 및 8.6 N 하드 클리핑 한계
    subplot(2, 3, 5);
    plot(t_hist, f_act_hist(1,:), 'r-', 'LineWidth', 1.1, 'DisplayName', 'Rotor 1 (FL)'); hold on;
    plot(t_hist, f_act_hist(2,:), 'g-', 'LineWidth', 1.1, 'DisplayName', 'Rotor 2 (BL)');
    plot(t_hist, f_act_hist(3,:), 'b-', 'LineWidth', 1.1, 'DisplayName', 'Rotor 3 (BR)');
    plot(t_hist, f_act_hist(4,:), 'c-', 'LineWidth', 1.1, 'DisplayName', 'Rotor 4 (FR)');
    yline(cfg.f_max_per_rotor, 'k--', 'LineWidth', 1.3, 'DisplayName', 'Max Limit (8.6 N)');
    yline(0.0, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    grid on; xlabel('Time [s]'); ylabel('Rotor Thrust [N]');
    title('4-Rotor Actual Thrusts (\tau_{up}=12.5ms, \tau_{down}=25ms)');
    legend('Location', 'best');

    % (6) 정규화 추력 명령 (thr_cmd) 및 각속도(p, q) Saturation 확인
    subplot(2, 3, 6);
    yyaxis left;
    plot(t_hist, thr_cmd_hist, 'b-', 'LineWidth', 1.3, 'DisplayName', 'thr\_cmd'); hold on;
    yline(0.9, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Thr Max (0.9)');
    yline(cfg.MPC_THR_HOVER, 'b:', 'LineWidth', 1.0, 'DisplayName', 'Hover (0.6)');
    ylabel('Normalized Thrust [0.1 ~ 0.9]');
    ylim([0.0, 1.05]);

    yyaxis right;
    plot(t_hist, state_hist(10,:)*R2D, 'r-', 'LineWidth', 1.0, 'DisplayName', 'Roll Rate p [deg/s]'); hold on;
    plot(t_hist, state_hist(11,:)*R2D, 'g-', 'LineWidth', 1.0, 'DisplayName', 'Pitch Rate q [deg/s]');
    yline(220, 'r--', 'LineWidth', 0.9, 'DisplayName', '\pm220 deg/s Limit');
    yline(-220, 'r--', 'LineWidth', 0.9, 'HandleVisibility', 'off');
    ylabel('Body Angular Rate [deg/s]');
    ylim([-260, 260]);
    grid on; xlabel('Time [s]');
    title('Normalized Thrust & Body Rate Saturation');
    legend('Location', 'best');

    % 자동 저장 활성화 시 검증 플롯 추가 저장
    if isfield(cfg, 'auto_save') && cfg.auto_save
        if ~exist(cfg.save_dir, 'dir')
            mkdir(cfg.save_dir);
        end
        if isfield(cfg, 'file_prefix') && ~isempty(cfg.file_prefix)
            baseFileName = fullfile(cfg.save_dir, cfg.file_prefix);
        else
            baseFileName = fullfile(cfg.save_dir, 'PX4_SimResult');
        end
        exportgraphics(h_fig_px4, [baseFileName, '_PX4_Verification.png'], 'Resolution', 300);
        if isfield(cfg, 'close_figures') && cfg.close_figures
            close(h_fig_px4);
        end
    end
end

function out = ternary_str(cond, str_true, str_false)
    if cond
        out = str_true;
    else
        out = str_false;
    end
end

