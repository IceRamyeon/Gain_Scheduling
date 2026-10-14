% regenerate_combined_comparison_light.m
% 기존 시뮬레이션 결과 MAT 파일들을 로드하여 Combined_Comparison 그래프를
% 완전한 'Theme', 'light' (흰색 배경, 검은색 텍스트 및 축)로 덮어쓰기 재생성

close all; clc; clear;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
end

base_results_dir = fullfile(script_dir, 'simulation_results');
speeds = [3.5, 5.5, 7.5];

fprintf('========================================================================\n');
fprintf('  Combined Comparison 그래프를 Light Theme으로 재생성 및 덮어쓰기 시작\n');
fprintf('========================================================================\n');

for s_idx = 1:length(speeds)
    v_tar = speeds(s_idx);
    speed_dir = fullfile(base_results_dir, sprintf('v=%.1f_ms', v_tar));
    
    mat1 = fullfile(speed_dir, 'Model1_Hovering_PID', 'Model1_Hovering_PID.mat');
    mat2 = fullfile(speed_dir, 'Model2_Feedforward_NoZ', 'Model2_Feedforward_NoZ.mat');
    mat3 = fullfile(speed_dir, 'Model3_Drone_LPV', 'Model3_Drone_LPV.mat');
    mat4 = fullfile(speed_dir, 'Model4_PPO', 'Model4_PPO.mat');
    
    if ~isfile(mat1) || ~isfile(mat2) || ~isfile(mat3) || ~isfile(mat4)
        fprintf('  [경고] v=%.1f m/s 관련 .mat 파일을 찾을 수 없습니다.\n', v_tar);
        continue;
    end
    
    d1 = load(mat1, 'log_data'); log1 = d1.log_data;
    d2 = load(mat2, 'log_data'); log2 = d2.log_data;
    d3 = load(mat3, 'log_data'); log3 = d3.log_data;
    d4 = load(mat4, 'log_data'); log4 = d4.log_data;
    
    err1 = sqrt(sum((log1.state_hist(1:3, :) - log1.pos_des_hist(1:3, :)).^2, 1));
    z_err1 = abs(log1.state_hist(3, :) - log1.pos_des_hist(3, :));
    
    err2 = sqrt(sum((log2.state_hist(1:3, :) - log2.pos_des_hist(1:3, :)).^2, 1));
    z_err2 = abs(log2.state_hist(3, :) - log2.pos_des_hist(3, :));
    
    err3 = sqrt(sum((log3.state_hist(1:3, :) - log3.pos_des_hist(1:3, :)).^2, 1));
    z_err3 = abs(log3.state_hist(3, :) - log3.pos_des_hist(3, :));
    
    err4 = sqrt(sum((log4.state_hist(1:3, :) - log4.pos_des_hist(1:3, :)).^2, 1));
    z_err4 = abs(log4.state_hist(3, :) - log4.pos_des_hist(3, :));
    
    % Light Theme Figure 생성
    h_cmp = figure('Name', sprintf('Model Comparison at v=%.1f m/s', v_tar), ...
                   'Theme', 'light', 'Color', [1 1 1], ...
                   'Position', [100, 100, 1250, 720], 'Visible', 'off');
    
    % Subplot 1: 2D Trajectory
    subplot(2, 2, [1, 3]);
    ax1 = gca;
    plot(log1.pos_des_hist(1,:), log1.pos_des_hist(2,:), 'k--', 'LineWidth', 2.2, 'DisplayName', 'Desired Trajectory'); hold on;
    plot(log1.state_hist(1,:), log1.state_hist(2,:), 'r-', 'LineWidth', 1.4, 'DisplayName', '1. Hovering PID');
    plot(log2.state_hist(1,:), log2.state_hist(2,:), 'm-.', 'LineWidth', 1.4, 'DisplayName', '2. Feedforward Only');
    plot(log3.state_hist(1,:), log3.state_hist(2,:), 'b--', 'LineWidth', 1.6, 'DisplayName', '3. Applied Gain Scheduling');
    plot(log4.state_hist(1,:), log4.state_hist(2,:), 'Color', [0.05 0.65 0.25], 'LineWidth', 2.0, 'DisplayName', '4. PPO Residual (Best)');
    set(ax1, 'YDir', 'Reverse', 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]);
    axis equal; grid on;
    set(ax1, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    title(sprintf('2D Flight Trajectory Comparison  (v = %.1f m/s)', v_tar), 'FontSize', 12.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    xlabel('X [m]', 'FontSize', 10.5, 'Color', [0 0 0]);
    ylabel('Y [m]', 'FontSize', 10.5, 'Color', [0 0 0]);
    legend('Location', 'best', 'FontSize', 9.5, 'TextColor', [0 0 0], 'EdgeColor', [0.7 0.7 0.7]);
    
    % Subplot 2: 3D Tracking Error vs Time
    subplot(2, 2, 2);
    ax2 = gca;
    plot(log1.t_hist, err1, 'r-', 'LineWidth', 1.3, 'DisplayName', '1. Hovering PID'); hold on;
    plot(log2.t_hist, err2, 'm-.', 'LineWidth', 1.3, 'DisplayName', '2. Feedforward Only');
    plot(log3.t_hist, err3, 'b--', 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(log4.t_hist, err4, 'Color', [0.05 0.65 0.25], 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual');
    set(ax2, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]);
    grid on; set(ax2, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    title('3D Tracking Error ||e_{pos}|| [m]', 'FontSize', 11.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    xlabel('Time [s]', 'FontSize', 10, 'Color', [0 0 0]);
    ylabel('Error [m]', 'FontSize', 10, 'Color', [0 0 0]);
    legend('Location', 'northeast', 'FontSize', 9, 'TextColor', [0 0 0], 'EdgeColor', [0.7 0.7 0.7]);
    
    % Subplot 3: Z-axis Error vs Time
    subplot(2, 2, 4);
    ax3 = gca;
    plot(log1.t_hist, z_err1, 'r-', 'LineWidth', 1.3, 'DisplayName', '1. Hovering PID'); hold on;
    plot(log2.t_hist, z_err2, 'm-.', 'LineWidth', 1.3, 'DisplayName', '2. Feedforward Only');
    plot(log3.t_hist, z_err3, 'b--', 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(log4.t_hist, z_err4, 'Color', [0.05 0.65 0.25], 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual');
    set(ax3, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]);
    grid on; set(ax3, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    title('Z-axis Error |z - z_{des}| [m]', 'FontSize', 11.5, 'FontWeight', 'bold', 'Color', [0 0 0]);
    xlabel('Time [s]', 'FontSize', 10, 'Color', [0 0 0]);
    ylabel('Z Error [m]', 'FontSize', 10, 'Color', [0 0 0]);
    legend('Location', 'northeast', 'FontSize', 9, 'TextColor', [0 0 0], 'EdgeColor', [0.7 0.7 0.7]);
    
    % 이미지 저장 (기존 파일 덮어쓰기)
    cmp_fig_path = fullfile(speed_dir, sprintf('Combined_Comparison_v=%.1f.png', v_tar));
    exportgraphics(h_cmp, cmp_fig_path, 'Resolution', 300);
    close(h_cmp);
    fprintf('  -> [완료] v=%.1f m/s Light Theme 그래프 저장: %s\n', v_tar, cmp_fig_path);
end

fprintf('\n모든 Combined Comparison 그래프가 Light Theme으로 덮어쓰기 완료되었습니다.\n');
