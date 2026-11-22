% evaluate_px4_ppo.m
% [PX4 + Gazebo x500] Pure PX4 Baseline (Ep 0) vs Trained PPO (Ep 3000) 종합 평가
% - 1) 원 궤적 (R=5.0m, v=5.0m/s, phi_eq=27deg) @ SensorNoise = 0 / 1
% - 2) 339.6m 헤어핀 벤치마크 코스 (v=4.0m/s, v=5.5m/s) @ SensorNoise = 0 / 1

clear; clc; close all;
[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
parent_dir = fileparts(px4_dir);
cd(px4_dir);
addpath(px4_dir);
addpath(parent_dir);

chk_dir = fullfile(px4_dir, 'savedAgents_px4');
s0 = load(fullfile(chk_dir, 'Agent0.mat'));
if isfield(s0, 'saved_agent'), ag0 = s0.saved_agent; else, ag0 = s0.agent; end
s3 = load(fullfile(chk_dir, 'Agent3000.mat'));
if isfield(s3, 'saved_agent'), ag3000 = s3.saved_agent; else, ag3000 = s3.agent; end

% 1. 원 궤적 평가 (직선 20m + R=5m 2바퀴 + 직선 20m, v = 5.0 m/s)
wp_circle = [0, 5.0, 0; 20.0, 4*pi, 20.0; -6, -6, -6];
for ns = [false, true]
    fprintf('\n=================================================================\n');
    fprintf('  [1] Circle Trajectory (R=5m, v=5.0m/s) | SensorNoise = %d\n', double(ns));
    fprintf('=================================================================\n');
    for m = 1:2
        env = RL_DroneEnv_PX4(ns); env.randomize_speed = false; env.randomize_path = false;
        env.cfg.target_speed = 5.0; env.cfg.drone_init_states(4) = 5.0;
        env.drone_px4 = Drone_PX4(env.cfg, wp_circle);
        env.drone_px4.x(9) = 0; env.drone_px4.euler(3) = 0;
        if m == 1, ag = ag0; lbl = 'Ep 0 (Pure PX4)'; else, ag = ag3000; lbl = 'Ep 3000 (PX4+PPO)'; end
        rng(42);
        [lg, ~] = run_px4_simulation_loop(env.drone_px4, env.cfg, ag, env);
        e  = sqrt(sum((lg.state_hist(1:3,:) - lg.pos_des_hist).^2, 1));
        sp = sqrt(sum(lg.state_hist(4:6,:).^2, 1));
        ez = abs(lg.state_hist(3,:) - lg.pos_des_hist(3,:));
        fprintf('%-18s: Pos RMSE=%.4fm | MAE=%.4fm | Max=%.4fm | Z_RMSE=%.4fm | Spd_RMSE=%.4fm/s | Sat=%.1f%%\n', ...
            lbl, sqrt(mean(e.^2)), mean(e), max(e), sqrt(mean(ez.^2)), sqrt(mean((sp-5.0).^2)), mean(lg.sat_rot_hist)*100);
        if ~ns && m == 1, log_c0 = lg; end
        if ~ns && m == 2, log_c3 = lg; end
    end
end

% 2. 339.6m 헤어핀 벤치마크 코스 평가 (v = 4.0 m/s & v = 5.5 m/s)
for v_ev = [4.0, 5.5]
    for ns = [false, true]
        fprintf('\n=================================================================\n');
        fprintf('  [2] 339.6m Hairpin Benchmark (v=%.1f m/s) | SensorNoise = %d\n', v_ev, double(ns));
        fprintf('=================================================================\n');
        for m = 1:2
            env = RL_DroneEnv_PX4(ns); env.randomize_speed = false; env.randomize_path = false;
            env.cfg.target_speed = v_ev; env.cfg.drone_init_states(4) = v_ev;
            reset(env);
            if m == 1, ag = ag0; lbl = 'Ep 0 (Pure PX4)'; else, ag = ag3000; lbl = 'Ep 3000 (PX4+PPO)'; end
            rng(42);
            [lg, ~] = run_px4_simulation_loop(env.drone_px4, env.cfg, ag, env);
            e  = sqrt(sum((lg.state_hist(1:3,:) - lg.pos_des_hist).^2, 1));
            sp = sqrt(sum(lg.state_hist(4:6,:).^2, 1));
            ez = abs(lg.state_hist(3,:) - lg.pos_des_hist(3,:));
            fprintf('%-18s: Pos RMSE=%.4fm | MAE=%.4fm | Max=%.4fm | Z_RMSE=%.4fm | Spd_RMSE=%.4fm/s | Sat=%.1f%%\n', ...
                lbl, sqrt(mean(e.^2)), mean(e), max(e), sqrt(mean(ez.^2)), sqrt(mean((sp-v_ev).^2)), mean(lg.sat_rot_hist)*100);
            if ~ns && v_ev == 4.0 && m == 1, log_h0 = lg; end
            if ~ns && v_ev == 4.0 && m == 2, log_h3 = lg; end
        end
    end
end

% 비교 그래프 저장 (Light Theme, 흰색 배경 + 검은색 글씨)
h_fig = figure('Color', [1 1 1], 'Position', [100, 100, 1400, 850], 'Visible', 'off');

subplot(2, 2, 1);
plot(log_c0.pos_des_hist(1,:), log_c0.pos_des_hist(2,:), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Desired'); hold on;
plot(log_c0.state_hist(1,:), log_c0.state_hist(2,:), 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Ep 0 (Pure PX4)');
plot(log_c3.state_hist(1,:), log_c3.state_hist(2,:), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Ep 3000 (PX4+PPO)');
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], 'YDir', 'reverse'); axis equal; grid on; box on;
title('(A) Circle Trajectory (R=5m, v=5.0m/s)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('X [m]', 'Color', [0 0 0]); ylabel('Y [m]', 'Color', [0 0 0]);
legend('Location', 'best', 'TextColor', [0 0 0], 'Color', [1 1 1]);

subplot(2, 2, 2);
ec0 = sqrt(sum((log_c0.state_hist(1:3,:) - log_c0.pos_des_hist).^2, 1));
ec3 = sqrt(sum((log_c3.state_hist(1:3,:) - log_c3.pos_des_hist).^2, 1));
plot(log_c0.t_hist, ec0, 'r-', 'LineWidth', 1.4, 'DisplayName', sprintf('Ep 0 (RMSE: %.3fm)', sqrt(mean(ec0.^2)))); hold on;
plot(log_c3.t_hist, ec3, 'b-', 'LineWidth', 1.6, 'DisplayName', sprintf('Ep 3000 (RMSE: %.3fm)', sqrt(mean(ec3.^2))));
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
ylim([0, max(max(ec0), max(ec3))*1.3]);
title('(B) Circle 3D Tracking Error (R=5m, v=5.0m/s)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', [0 0 0]); ylabel('Error [m]', 'Color', [0 0 0]);
legend('Location', 'northwest', 'TextColor', [0 0 0], 'Color', [1 1 1]);

subplot(2, 2, 3);
eh0 = sqrt(sum((log_h0.state_hist(1:3,:) - log_h0.pos_des_hist).^2, 1));
eh3 = sqrt(sum((log_h3.state_hist(1:3,:) - log_h3.pos_des_hist).^2, 1));
plot(log_h0.t_hist, eh0, 'r-', 'LineWidth', 1.3, 'DisplayName', sprintf('Ep 0 (RMSE: %.3fm)', sqrt(mean(eh0.^2)))); hold on;
plot(log_h3.t_hist, eh3, 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('Ep 3000 (RMSE: %.3fm)', sqrt(mean(eh3.^2))));
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
ylim([0, max(max(eh0), max(eh3))*1.25]);
title('(C) 339.6m Hairpin 3D Error (v=4.0m/s)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', [0 0 0]); ylabel('Error [m]', 'Color', [0 0 0]);
legend('Location', 'northwest', 'TextColor', [0 0 0], 'Color', [1 1 1]);

subplot(2, 2, 4);
base_Kp_xy = 0.215 * 9.81;
base_Kd_xy = 0.203 * 9.81;
tot_Kp_xy_3 = max(0.1, base_Kp_xy + 0.8 * log_c3.action_hist(1,:));
tot_Kd_xy_3 = max(0.1, base_Kd_xy + max(0, 1.0 * log_c3.action_hist(2,:)));
tot_Kp_z_3  = log_c3.PID_Pos_hist(1,:);
tot_Kd_z_3  = log_c3.PID_Pos_hist(3,:);

plot(log_c3.t_hist, tot_Kp_xy_3, 'LineWidth', 1.5, 'DisplayName', 'Total K_{P,xy}'); hold on;
plot(log_c3.t_hist, tot_Kd_xy_3, 'LineWidth', 1.5, 'DisplayName', 'Total K_{D,xy}');
plot(log_c3.t_hist, tot_Kp_z_3,  'LineWidth', 1.5, 'DisplayName', 'Total K_{P,z}');
plot(log_c3.t_hist, tot_Kd_z_3,  'LineWidth', 1.5, 'DisplayName', 'Total K_{D,z}');
yline(base_Kp_xy, 'k:', 'LineWidth', 1.1, 'DisplayName', 'Base K_{P,xy}');
yline(6.0, 'k--', 'LineWidth', 1.1, 'DisplayName', 'Base K_{P,z}');
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
ylim([0, max([tot_Kp_z_3, 7.5]) * 1.25]);
title('(D) Ep 3000 Total Effective PID Gains (Circle R=5m, v=5.0m/s)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', [0 0 0]); ylabel('Total Gain Value', 'Color', [0 0 0]);
legend('Location', 'north', 'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', 8.5, 'TextColor', [0 0 0], 'Color', [1 1 1]);

sgtitle('PX4 + Gazebo x500 (Hover=0.73, MaxTilt=30deg): Pure PX4 vs Trained PPO', 'Color', [0 0 0], 'FontWeight', 'bold', 'FontSize', 14);
exportgraphics(h_fig, fullfile(chk_dir, 'PX4_PPO_Comparison.png'), 'Resolution', 300);
close(h_fig);

% 3. 전체 PID 계수 상세 비교 그래프 (Ep 0 vs Ep 3000: XY 및 Z축 전체 P, I, D 게인)
h_pid = figure('Color', [1 1 1], 'Position', [100, 100, 1400, 850], 'Visible', 'off');

tot_Kp_xy_0 = base_Kp_xy * ones(size(log_h0.t_hist));
tot_Kd_xy_0 = base_Kd_xy * ones(size(log_h0.t_hist));
tot_Kp_xy_h3 = max(0.1, base_Kp_xy + 0.8 * log_h3.action_hist(1,:));
tot_Kd_xy_h3 = max(0.1, base_Kd_xy + max(0, 1.0 * log_h3.action_hist(2,:)));

subplot(2, 2, 1);
plot(log_h0.t_hist, tot_Kp_xy_0,  'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ep 0 Base (%.2f)', base_Kp_xy)); hold on;
plot(log_h3.t_hist, tot_Kp_xy_h3, 'b-',  'LineWidth', 1.6, 'DisplayName', 'Ep 3000 Total K_{P,xy}');
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
ylim([1.0, 3.3]);
title('(A) Horizontal Proportional Gain K_{P,xy} (v=4.0 m/s)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', [0 0 0]); ylabel('Gain [(m/s^2)/m]', 'Color', [0 0 0]);
legend('Location', 'northwest', 'TextColor', [0 0 0], 'Color', [1 1 1]);

subplot(2, 2, 2);
plot(log_h0.t_hist, tot_Kd_xy_0,  'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Ep 0 Base (%.2f)', base_Kd_xy)); hold on;
plot(log_h3.t_hist, tot_Kd_xy_h3, 'b-',  'LineWidth', 1.6, 'DisplayName', 'Ep 3000 Total K_{D,xy}');
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
ylim([1.5, 3.3]);
title('(B) Horizontal Derivative Gain K_{D,xy} (v=4.0 m/s)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', [0 0 0]); ylabel('Gain [(m/s^2)/(m/s)]', 'Color', [0 0 0]);
legend('Location', 'northwest', 'TextColor', [0 0 0], 'Color', [1 1 1]);

subplot(2, 2, 3);
plot(log_h0.t_hist, log_h0.PID_Pos_hist(1,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ep 0 K_{P,z} (LPV Only)'); hold on;
plot(log_h3.t_hist, log_h3.PID_Pos_hist(1,:), 'b-',  'LineWidth', 1.6, 'DisplayName', 'Ep 3000 Total K_{P,z}');
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
ylim([4.5, 8.5]);
title('(C) Vertical Proportional Gain K_{P,z} (with Tilt Scheduling)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', [0 0 0]); ylabel('Gain [(m/s^2)/m]', 'Color', [0 0 0]);
legend('Location', 'northwest', 'TextColor', [0 0 0], 'Color', [1 1 1]);

subplot(2, 2, 4);
plot(log_h0.t_hist, log_h0.PID_Pos_hist(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ep 0 K_{D,z} (LPV Only)'); hold on;
plot(log_h3.t_hist, log_h3.PID_Pos_hist(3,:), 'b-',  'LineWidth', 1.6, 'DisplayName', 'Ep 3000 Total K_{D,z}');
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
ylim([3.5, 6.0]);
title('(D) Vertical Derivative Gain K_{D,z} (with Tilt Scheduling)', 'Color', [0 0 0], 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', [0 0 0]); ylabel('Gain [(m/s^2)/(m/s)]', 'Color', [0 0 0]);
legend('Location', 'northwest', 'TextColor', [0 0 0], 'Color', [1 1 1]);

sgtitle('Total Effective PID Gains: Pure PX4 (Ep 0) vs PX4+PPO (Ep 3000) on 339.6m Hairpins', 'Color', [0 0 0], 'FontWeight', 'bold', 'FontSize', 14);
exportgraphics(h_pid, fullfile(chk_dir, 'PX4_Total_PID_Gains.png'), 'Resolution', 300);
close(h_pid);
