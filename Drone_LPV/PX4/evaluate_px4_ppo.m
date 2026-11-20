% evaluate_px4_ppo.m
% [PX4 + Gazebo x500] Pure PX4 Baseline (Ep 0) vs Trained PPO (Ep 3000) 비교 평가
% - 339.6m 헤어핀 벤치마크 코스 @ v = 4.0 m/s 또는 5.5 m/s
% - 250Hz Gazebo x500 물리/자세 루프 + 50Hz 위치 루프 + 10Hz PPO + 0.04s 지연

clear; clc; close all;
[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
parent_dir = fileparts(px4_dir);
cd(px4_dir);
addpath(px4_dir);
addpath(parent_dir);

v_eval           = 5.5;   % 평가 비행 속력 [m/s]
use_sensor_noise = false; % true 설정 시 M10 GPS(1.5m) + 8Hz EKF2 LPF 활성화

chk_dir = fullfile(px4_dir, 'savedAgents_px4');
s0 = load(fullfile(chk_dir, 'Agent0.mat'));
if isfield(s0, 'saved_agent'), ag0 = s0.saved_agent; else, ag0 = s0.agent; end
s3 = load(fullfile(chk_dir, 'Agent3000.mat'));
if isfield(s3, 'saved_agent'), ag3000 = s3.saved_agent; else, ag3000 = s3.agent; end

% 1) Pure PX4 Baseline (Ep 0)
env0 = RL_DroneEnv_PX4(use_sensor_noise);
env0.randomize_speed = false; env0.randomize_path = false;
env0.cfg.target_speed = v_eval; env0.cfg.drone_init_states(4) = v_eval;
reset(env0);
[log0, ~] = run_px4_simulation_loop(env0.drone_px4, env0.cfg, ag0, env0);

% 2) Trained PX4 + PPO (Ep 3000)
env3 = RL_DroneEnv_PX4(use_sensor_noise);
env3.randomize_speed = false; env3.randomize_path = false;
env3.cfg.target_speed = v_eval; env3.cfg.drone_init_states(4) = v_eval;
reset(env3);
[log3, ~] = run_px4_simulation_loop(env3.drone_px4, env3.cfg, ag3000, env3);

% 오차 및 속력 유지 비교
err0 = sqrt(sum((log0.state_hist(1:3,:) - log0.pos_des_hist).^2, 1));
err3 = sqrt(sum((log3.state_hist(1:3,:) - log3.pos_des_hist).^2, 1));
spd0 = sqrt(sum(log0.state_hist(4:6,:).^2, 1));
spd3 = sqrt(sum(log3.state_hist(4:6,:).^2, 1));

fprintf('\n=================================================================\n');
fprintf('  [PX4 + Gazebo x500 @ v = %.1f m/s, SensorNoise = %d]\n', v_eval, double(use_sensor_noise));
fprintf('=================================================================\n');
fprintf('Ep 0 (Pure PX4)   : Pos RMSE=%.4fm | Max=%.4fm | Spd RMSE=%.4fm/s | Sat=%.1f%%\n', ...
    sqrt(mean(err0.^2)), max(err0), sqrt(mean((spd0 - v_eval).^2)), mean(log0.sat_rot_hist)*100);
fprintf('Ep 3000 (PX4+PPO) : Pos RMSE=%.4fm | Max=%.4fm | Spd RMSE=%.4fm/s | Sat=%.1f%%\n', ...
    sqrt(mean(err3.^2)), max(err3), sqrt(mean((spd3 - v_eval).^2)), mean(log3.sat_rot_hist)*100);

% 비교 그래프 저장 (Light Theme)
h_fig = figure('Color', 'w', 'Position', [100, 100, 1300, 800], 'Visible', 'off');
subplot(2, 2, 1);
plot(log0.pos_des_hist(1,:), log0.pos_des_hist(2,:), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Desired'); hold on;
plot(log0.state_hist(1,:), log0.state_hist(2,:), 'r-.', 'LineWidth', 1.4, 'DisplayName', 'Ep 0 (Pure PX4)');
plot(log3.state_hist(1,:), log3.state_hist(2,:), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Ep 3000 (PX4+PPO)');
set(gca, 'YDir', 'reverse'); axis equal; grid on; box on;
title('2D Trajectory (PX4 + Gazebo x500)'); xlabel('X [m]'); ylabel('Y [m]'); legend('Location', 'best');

subplot(2, 2, 2);
plot(log0.t_hist, err0, 'r-', 'LineWidth', 1.3, 'DisplayName', sprintf('Ep 0 (RMSE: %.3fm)', sqrt(mean(err0.^2)))); hold on;
plot(log3.t_hist, err3, 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('Ep 3000 (RMSE: %.3fm)', sqrt(mean(err3.^2))));
grid on; box on; title('3D Tracking Error'); xlabel('Time [s]'); ylabel('Error [m]'); legend('Location', 'northwest');

subplot(2, 2, 3);
plot(log0.t_hist, spd0, 'r-', 'LineWidth', 1.3, 'DisplayName', 'Ep 0 Speed'); hold on;
plot(log3.t_hist, spd3, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ep 3000 Speed');
yline(v_eval, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target Speed');
grid on; box on; title('Speed Maintenance'); xlabel('Time [s]'); ylabel('Speed [m/s]'); legend('Location', 'northwest');

subplot(2, 2, 4);
plot(log3.t_hist, log3.action_hist(1,:), 'LineWidth', 1.3, 'DisplayName', '\Delta P_{xy}'); hold on;
plot(log3.t_hist, log3.action_hist(2,:), 'LineWidth', 1.3, 'DisplayName', '\Delta D_{xy}');
plot(log3.t_hist, log3.action_hist(3,:), 'LineWidth', 1.3, 'DisplayName', '\Delta P_z');
plot(log3.t_hist, log3.action_hist(4,:), 'LineWidth', 1.3, 'DisplayName', '\Delta D_z');
grid on; box on; title('Ep 3000 PPO Actions'); xlabel('Time [s]'); ylabel('Action'); legend('Location', 'best');

exportgraphics(h_fig, fullfile(chk_dir, 'PX4_PPO_Comparison.png'), 'Resolution', 300);
close(h_fig);
