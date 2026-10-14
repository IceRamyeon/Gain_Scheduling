% generate_combined_plots.m
% 4가지 제어 모델(Hovering PID, Feedforward Only, Applied Gain Scheduling, PPO Residual)의
% 1) Position XYZ 비교 플롯 (Subplot 3,1)
% 2) Velocity 비교 플롯 (Subplot 3,1: Total Speed, V_xy, V_z)
% 3) PID Gain 변화 비교 플롯 (Subplot 3,1: K_p, K_i, K_d)
% 을 각 속도별(v = 3.5, 5.5, 7.5 m/s)로 하나의 통합 그래프(Light Theme)로 생성 및 저장

close all; clc; clear;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
end

base_results_dir = fullfile(script_dir, 'simulation_results');
speeds = [3.5, 5.5, 7.5];

% 색상 및 선 스타일 정의
c_des = [0.1 0.1 0.1];
c_m1  = [0.85 0.1 0.1];       % Hovering PID: Red
c_m2  = [0.85 0.2 0.85];      % Feedforward Only: Magenta
c_m3  = [0.1 0.35 0.85];      % Applied Gain Scheduling: Blue
c_m4  = [0.05 0.65 0.25];     % PPO Residual: Forest Green

fprintf('========================================================================\n');
fprintf('  Combined Plots (Position XYZ, Velocity, PID Gains) 생성 시작\n');
fprintf('========================================================================\n');

for s_idx = 1:length(speeds)
    v_tar = speeds(s_idx);
    speed_dir = fullfile(base_results_dir, sprintf('v=%.1f_ms', v_tar));
    
    mat1 = fullfile(speed_dir, 'Model1_Hovering_PID', 'Model1_Hovering_PID.mat');
    mat2 = fullfile(speed_dir, 'Model2_Feedforward_NoZ', 'Model2_Feedforward_NoZ.mat');
    mat3 = fullfile(speed_dir, 'Model3_Drone_LPV', 'Model3_Drone_LPV.mat');
    mat4 = fullfile(speed_dir, 'Model4_PPO', 'Model4_PPO.mat');
    
    if ~isfile(mat1) || ~isfile(mat2) || ~isfile(mat3) || ~isfile(mat4)
        fprintf('  [경고] v=%.1f m/s .mat 파일을 찾을 수 없습니다.\n', v_tar);
        continue;
    end
    
    d1 = load(mat1, 'log_data'); log1 = d1.log_data;
    d2 = load(mat2, 'log_data'); log2 = d2.log_data;
    d3 = load(mat3, 'log_data'); log3 = d3.log_data;
    d4 = load(mat4, 'log_data'); log4 = d4.log_data;
    
    t_hist = log1.t_hist;
    des_speed = v_tar * ones(1, length(t_hist));
    
    % 속도 계산
    spd1_tot = sqrt(sum(log1.state_hist(4:6, :).^2, 1));
    spd2_tot = sqrt(sum(log2.state_hist(4:6, :).^2, 1));
    spd3_tot = sqrt(sum(log3.state_hist(4:6, :).^2, 1));
    spd4_tot = sqrt(sum(log4.state_hist(4:6, :).^2, 1));
    
    spd1_xy = sqrt(sum(log1.state_hist(4:5, :).^2, 1));
    spd2_xy = sqrt(sum(log2.state_hist(4:5, :).^2, 1));
    spd3_xy = sqrt(sum(log3.state_hist(4:5, :).^2, 1));
    spd4_xy = sqrt(sum(log4.state_hist(4:5, :).^2, 1));

    %% =========================================================================
    %% 1. Combined Position XYZ Plot (Subplot 3,1)
    %% =========================================================================
    h_pos = figure('Name', sprintf('Combined Position Tracking (v=%.1f m/s)', v_tar), ...
                   'Theme', 'light', 'Color', [1 1 1], ...
                   'Position', [100, 100, 1100, 850], 'Visible', 'off');
    
    % Subplot (3,1,1): Position X
    subplot(3, 1, 1); ax = gca;
    plot(t_hist, log1.pos_des_hist(1,:), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired Trajectory'); hold on;
    plot(t_hist, log1.state_hist(1,:), '-', 'Color', c_m1, 'LineWidth', 1.4, 'DisplayName', '1. Hovering PID');
    plot(t_hist, log2.state_hist(1,:), '-.', 'Color', c_m2, 'LineWidth', 1.4, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, log3.state_hist(1,:), '--', 'Color', c_m3, 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(t_hist, log4.state_hist(1,:), '-', 'Color', c_m4, 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual (Best)');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    ylabel('X Position [m]', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title(sprintf('Combined Position Tracking Comparison (v = %.1f m/s) - X, Y, Z', v_tar), ...
          'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    legend('Location', 'best', 'FontSize', 9, 'TextColor', [0 0 0], 'EdgeColor', [0.75 0.75 0.75]);
    
    % Subplot (3,1,2): Position Y
    subplot(3, 1, 2); ax = gca;
    plot(t_hist, log1.pos_des_hist(2,:), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired Trajectory'); hold on;
    plot(t_hist, log1.state_hist(2,:), '-', 'Color', c_m1, 'LineWidth', 1.4, 'DisplayName', '1. Hovering PID');
    plot(t_hist, log2.state_hist(2,:), '-.', 'Color', c_m2, 'LineWidth', 1.4, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, log3.state_hist(2,:), '--', 'Color', c_m3, 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(t_hist, log4.state_hist(2,:), '-', 'Color', c_m4, 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual (Best)');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    ylabel('Y Position [m]', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    % Subplot (3,1,3): Position Z
    subplot(3, 1, 3); ax = gca;
    plot(t_hist, log1.pos_des_hist(3,:), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired Trajectory'); hold on;
    plot(t_hist, log1.state_hist(3,:), '-', 'Color', c_m1, 'LineWidth', 1.4, 'DisplayName', '1. Hovering PID');
    plot(t_hist, log2.state_hist(3,:), '-.', 'Color', c_m2, 'LineWidth', 1.4, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, log3.state_hist(3,:), '--', 'Color', c_m3, 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(t_hist, log4.state_hist(3,:), '-', 'Color', c_m4, 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual (Best)');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    xlabel('Time [s]', 'FontSize', 10.5, 'Color', [0 0 0]);
    ylabel('Z Position [m]', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    pos_fig_path = fullfile(speed_dir, sprintf('Combined_Position_XYZ_v=%.1f.png', v_tar));
    exportgraphics(h_pos, pos_fig_path, 'Resolution', 300);
    close(h_pos);
    fprintf('  -> [완료] Position XYZ 플롯 저장: %s\n', pos_fig_path);

    %% =========================================================================
    %% 2. Combined Velocity Plot (Subplot 3,1: Total Speed, V_xy, V_z)
    %% =========================================================================
    h_vel = figure('Name', sprintf('Combined Velocity Tracking (v=%.1f m/s)', v_tar), ...
                   'Theme', 'light', 'Color', [1 1 1], ...
                   'Position', [100, 100, 1100, 850], 'Visible', 'off');
    
    % Subplot (3,1,1): Total Speed
    subplot(3, 1, 1); ax = gca;
    plot(t_hist, des_speed, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Target Speed'); hold on;
    plot(t_hist, spd1_tot, '-', 'Color', c_m1, 'LineWidth', 1.4, 'DisplayName', '1. Hovering PID');
    plot(t_hist, spd2_tot, '-.', 'Color', c_m2, 'LineWidth', 1.4, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, spd3_tot, '--', 'Color', c_m3, 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(t_hist, spd4_tot, '-', 'Color', c_m4, 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual (Best)');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    ylabel('Total Speed ||V|| [m/s]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title(sprintf('Combined Velocity Tracking Comparison (v = %.1f m/s)', v_tar), ...
          'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    legend('Location', 'best', 'FontSize', 9, 'TextColor', [0 0 0], 'EdgeColor', [0.75 0.75 0.75]);
    
    % Subplot (3,1,2): Horizontal Speed V_xy
    subplot(3, 1, 2); ax = gca;
    plot(t_hist, des_speed, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Target Speed (XY)'); hold on;
    plot(t_hist, spd1_xy, '-', 'Color', c_m1, 'LineWidth', 1.4, 'DisplayName', '1. Hovering PID');
    plot(t_hist, spd2_xy, '-.', 'Color', c_m2, 'LineWidth', 1.4, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, spd3_xy, '--', 'Color', c_m3, 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(t_hist, spd4_xy, '-', 'Color', c_m4, 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual (Best)');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    ylabel('Horizontal Speed V_{xy} [m/s]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    % Subplot (3,1,3): Vertical Velocity V_z
    subplot(3, 1, 3); ax = gca;
    plot(t_hist, zeros(1, length(t_hist)), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired V_z = 0'); hold on;
    plot(t_hist, log1.state_hist(6,:), '-', 'Color', c_m1, 'LineWidth', 1.4, 'DisplayName', '1. Hovering PID');
    plot(t_hist, log2.state_hist(6,:), '-.', 'Color', c_m2, 'LineWidth', 1.4, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, log3.state_hist(6,:), '--', 'Color', c_m3, 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(t_hist, log4.state_hist(6,:), '-', 'Color', c_m4, 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual (Best)');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    xlabel('Time [s]', 'FontSize', 10.5, 'Color', [0 0 0]);
    ylabel('Vertical Speed V_z [m/s]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    vel_fig_path = fullfile(speed_dir, sprintf('Combined_Velocity_v=%.1f.png', v_tar));
    exportgraphics(h_vel, vel_fig_path, 'Resolution', 300);
    close(h_vel);
    fprintf('  -> [완료] Velocity 플롯 저장: %s\n', vel_fig_path);

    %% =========================================================================
    %% 3. Combined PID Gain Plot (Subplot 3,1: K_p, K_i, K_d)
    %% =========================================================================
    h_pid = figure('Name', sprintf('Combined PID Gain Variation (v=%.1f m/s)', v_tar), ...
                   'Theme', 'light', 'Color', [1 1 1], ...
                   'Position', [100, 100, 1100, 850], 'Visible', 'off');
    
    % Subplot (3,1,1): K_p
    subplot(3, 1, 1); ax = gca;
    plot(t_hist, log1.PID_Pos_hist(1,:), '-', 'Color', c_m1, 'LineWidth', 1.5, 'DisplayName', '1. Hovering PID'); hold on;
    plot(t_hist, log2.PID_Pos_hist(1,:), '-.', 'Color', c_m2, 'LineWidth', 1.5, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, log4.PID_Pos_hist(1,:), '-', 'Color', c_m4, 'LineWidth', 2.2, 'DisplayName', '4. PPO Residual');
    plot(t_hist, log3.PID_Pos_hist(1,:), '--', 'Color', c_m3, 'LineWidth', 2.0, 'DisplayName', '3. Applied Gain Scheduling');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    ylabel('Proportional Gain K_p', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title(sprintf('Z-axis Scheduled PID Gain Comparison (v = %.1f m/s)', v_tar), ...
          'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    legend('Location', 'best', 'FontSize', 9.5, 'TextColor', [0 0 0], 'EdgeColor', [0.75 0.75 0.75]);
    
    % Subplot (3,1,2): K_i
    subplot(3, 1, 2); ax = gca;
    plot(t_hist, log1.PID_Pos_hist(2,:), '-', 'Color', c_m1, 'LineWidth', 1.5, 'DisplayName', '1. Hovering PID'); hold on;
    plot(t_hist, log2.PID_Pos_hist(2,:), '-.', 'Color', c_m2, 'LineWidth', 1.5, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, log4.PID_Pos_hist(2,:), '-', 'Color', c_m4, 'LineWidth', 2.4, 'DisplayName', '4. PPO Residual');
    plot(t_hist, log3.PID_Pos_hist(2,:), '--', 'Color', c_m3, 'LineWidth', 2.0, 'DisplayName', '3. Applied Gain Scheduling');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    ylabel('Integral Gain K_i', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    % Subplot (3,1,3): K_d
    subplot(3, 1, 3); ax = gca;
    plot(t_hist, log1.PID_Pos_hist(3,:), '-', 'Color', c_m1, 'LineWidth', 1.5, 'DisplayName', '1. Hovering PID'); hold on;
    plot(t_hist, log2.PID_Pos_hist(3,:), '-.', 'Color', c_m2, 'LineWidth', 1.5, 'DisplayName', '2. Feedforward Only');
    plot(t_hist, log4.PID_Pos_hist(3,:), '-', 'Color', c_m4, 'LineWidth', 2.2, 'DisplayName', '4. PPO Residual');
    plot(t_hist, log3.PID_Pos_hist(3,:), '--', 'Color', c_m3, 'LineWidth', 2.0, 'DisplayName', '3. Applied Gain Scheduling');
    set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on;
    set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    xlabel('Time [s]', 'FontSize', 10.5, 'Color', [0 0 0]);
    ylabel('Derivative Gain K_d', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    pid_fig_path = fullfile(speed_dir, sprintf('Combined_PID_Gains_v=%.1f.png', v_tar));
    exportgraphics(h_pid, pid_fig_path, 'Resolution', 300);
    close(h_pid);
    fprintf('  -> [완료] PID Gain 플롯 저장: %s\n', pid_fig_path);
end

fprintf('\n모든 Combined Plot(Position XYZ, Velocity, PID Gains)이 성공적으로 저장되었습니다.\n');
