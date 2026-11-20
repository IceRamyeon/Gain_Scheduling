% compare_lookahead_options.m
% 339.6m 헤어핀 벤치마크 코스 (v = 5.5 m/s) 에서 4가지 모델 정밀 비교:
%   1) Ep 0 (Pure LPV Baseline)
%   2) Baseline PPO (11D, 고정 시간 예견 dt = 0.5s)
%   3) Option 1 PPO (11D, 속력 반비례 예견 dt = L / v_des, L = 2.0m)
%   4) Option 2 PPO (12D, 속력 반비례 dt1 = L / v_des + 고정 시간 dt2 = 0.5s 동시 입력)

clear; clc; close all;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
cd(script_dir); addpath(genpath(script_dir));

v_eval = 5.5;
[r_arr, th_arr, z_arr] = pathGen(false);
p = TrajectoryPlanner([0;0;-6], [r_arr; th_arr; z_arr], v_eval);
cum_L = p.cum_lengths;

seg_arcs  = [2, 4, 6, 8, 10, 12];
seg_radii = [15.0, 10.0, 7.5, 5.0, 2.5, 1.25];
seg_diam  = [30.0, 20.0, 15.0, 10.0, 5.0, 2.5];

% 에이전트 로드 헬퍼
load_ag = @(p) load(p);
get_ag  = @(s) subsref(s, substruct('.', ifelse_field(s)));

% 1) Ep 0 (Pure LPV)
s0 = load(fullfile(script_dir, 'savedAgents_lpv', 'Agent0.mat'));
if isfield(s0, 'saved_agent'), ag0 = s0.saved_agent; else, ag0 = s0.agent; end
env0 = RL_DroneEnv_LPV('fixed', 2.0);
env0.randomize_speed = false; env0.randomize_path = false;
env0.cfg.target_speed = v_eval; env0.cfg.drone1_init_states(4) = v_eval; reset(env0);
cfg0 = env0.cfg; cfg0.target_speed = v_eval;
[log0, ~] = run_simulation_loop(env0.drone1, cfg0, ag0, env0);

% 2) Baseline PPO (11D Fixed dt = 0.5s)
sb = load(fullfile(script_dir, 'savedAgents_lpv', 'Agent3000.mat'));
if isfield(sb, 'saved_agent'), ag_base = sb.saved_agent; else, ag_base = sb.agent; end
env_b = RL_DroneEnv_LPV('fixed', 2.0);
env_b.randomize_speed = false; env_b.randomize_path = false;
env_b.cfg.target_speed = v_eval; env_b.cfg.drone1_init_states(4) = v_eval; reset(env_b);
cfg_b = env_b.cfg; cfg_b.target_speed = v_eval;
[log_base, ~] = run_simulation_loop(env_b.drone1, cfg_b, ag_base, env_b);

% 3) Option 1 PPO (11D Spatial dt = L / v_des)
s1 = load(fullfile(script_dir, 'savedAgents_opt1_spatial', 'Agent3000.mat'));
if isfield(s1, 'saved_agent'), ag_opt1 = s1.saved_agent; else, ag_opt1 = s1.agent; end
env_1 = RL_DroneEnv_LPV('spatial', 2.0);
env_1.randomize_speed = false; env_1.randomize_path = false;
env_1.cfg.target_speed = v_eval; env_1.cfg.drone1_init_states(4) = v_eval; reset(env_1);
cfg_1 = env_1.cfg; cfg_1.target_speed = v_eval;
[log_opt1, ~] = run_simulation_loop(env_1.drone1, cfg_1, ag_opt1, env_1);

% 4) Option 2 PPO (12D Dual dt1 = L / v_des + dt2 = 0.5s)
s2 = load(fullfile(script_dir, 'savedAgents_opt2_dual', 'Agent3000.mat'));
if isfield(s2, 'saved_agent'), ag_opt2 = s2.saved_agent; else, ag_opt2 = s2.agent; end
env_2 = RL_DroneEnv_LPV('dual', 2.0);
env_2.randomize_speed = false; env_2.randomize_path = false;
env_2.cfg.target_speed = v_eval; env_2.cfg.drone1_init_states(4) = v_eval; reset(env_2);
cfg_2 = env_2.cfg; cfg_2.target_speed = v_eval;
[log_opt2, ~] = run_simulation_loop(env_2.drone1, cfg_2, ag_opt2, env_2);

logs = {log0, log_base, log_opt1, log_opt2};
names = {'Ep 0 (Pure LPV)', 'Fixed Lookahead (0.5s, 11D)', 'Option 1: Spatial (L/v, 11D)', 'Option 2: Dual (L/v + 0.5s, 12D)'};
n_models = length(logs);

fprintf('\n=================================================================================\n');
fprintf('  [Overall 339.6m Course Comparison @ v = %.1f m/s]\n', v_eval);
fprintf('=================================================================================\n');
fprintf('%-34s | %9s | %9s | %9s | %10s\n', 'Model', 'Pos RMSE', 'Pos MAE', 'Max Err', 'Speed RMSE');
fprintf('---------------------------------------------------------------------------------\n');

overall_stats = zeros(n_models, 4);
rad_rmse      = zeros(length(seg_arcs), n_models);
rad_spd_rmse  = zeros(length(seg_arcs), n_models);

for m = 1:n_models
    lg = logs{m};
    e_pos = sqrt(sum((lg.state_hist(1:3,:) - lg.pos_des_hist(1:3,:)).^2, 1));
    spd   = sqrt(sum(lg.state_hist(4:6,:).^2, 1));
    e_spd = abs(spd - v_eval);
    s_h   = v_eval * lg.t_hist;

    overall_stats(m, 1) = sqrt(mean(e_pos.^2));
    overall_stats(m, 2) = mean(e_pos);
    overall_stats(m, 3) = max(e_pos);
    overall_stats(m, 4) = sqrt(mean(e_spd.^2));

    fprintf('%-34s | %8.4fm | %8.4fm | %8.4fm | %8.4fm/s\n', ...
        names{m}, overall_stats(m,1), overall_stats(m,2), overall_stats(m,3), overall_stats(m,4));

    for i = 1:length(seg_arcs)
        seg_idx = seg_arcs(i);
        mask = (s_h >= cum_L(seg_idx)) & (s_h <= cum_L(seg_idx + 1));
        rad_rmse(i, m)     = sqrt(mean(e_pos(mask).^2));
        rad_spd_rmse(i, m) = sqrt(mean(e_spd(mask).^2));
    end
end

fprintf('\n=================================================================================\n');
fprintf('  [Per-Hairpin Position RMSE [m] Comparison @ v = %.1f m/s]\n', v_eval);
fprintf('=================================================================================\n');
fprintf('%-10s | %12s | %16s | %16s | %16s\n', 'Radius', 'Pure LPV', 'Fixed (0.5s)', 'Opt 1 (L/v)', 'Opt 2 (Dual)');
fprintf('---------------------------------------------------------------------------------\n');
for i = 1:length(seg_arcs)
    fprintf('R = %5.2fm | %11.4fm | %8.4fm (%+5.1f%%) | %8.4fm (%+5.1f%%) | %8.4fm (%+5.1f%%)\n', ...
        seg_radii(i), rad_rmse(i,1), ...
        rad_rmse(i,2), (rad_rmse(i,2)-rad_rmse(i,1))/rad_rmse(i,1)*100, ...
        rad_rmse(i,3), (rad_rmse(i,3)-rad_rmse(i,1))/rad_rmse(i,1)*100, ...
        rad_rmse(i,4), (rad_rmse(i,4)-rad_rmse(i,1))/rad_rmse(i,1)*100);
end

% 시각화 (Light Theme, 흰색 배경 + 검은색 글씨)
h_fig = figure('Name', 'Lookahead Options Comparison', ...
    'Units', 'pixels', 'Position', [80, 80, 1450, 950], 'Color', [1 1 1]);

c_des  = [0.15 0.15 0.15];
c_m0   = [0.85 0.28 0.15]; % Ep 0: Coral Red
c_mb   = [0.55 0.55 0.55]; % Fixed 0.5s: Gray/Slate
c_m1   = [0.00 0.45 0.74]; % Option 1 (L/v): Deep Blue
c_m2   = [0.05 0.62 0.28]; % Option 2 (Dual): Forest Green
c_grid = [0.88 0.88 0.88];

% (A) 고곡률 2D 궤적 비교
subplot(2, 2, 1);
s_thresh = 168.54;
m_high = (v_eval * log0.t_hist >= s_thresh);
plot(log0.pos_des_hist(1, m_high), log0.pos_des_hist(2, m_high), '--', 'Color', c_des, 'LineWidth', 2.0, 'DisplayName', 'Desired Path'); hold on;
plot(log0.state_hist(1, m_high), log0.state_hist(2, m_high), ':', 'Color', c_m0, 'LineWidth', 1.6, 'DisplayName', 'Pure LPV (Ep 0)');
plot(log_base.state_hist(1, m_high), log_base.state_hist(2, m_high), '-.', 'Color', c_mb, 'LineWidth', 1.6, 'DisplayName', 'Fixed Lookahead (0.5s)');
plot(log_opt1.state_hist(1, m_high), log_opt1.state_hist(2, m_high), '-', 'Color', c_m1, 'LineWidth', 2.0, 'DisplayName', 'Opt 1: Spatial (L/v, 11D)');
plot(log_opt2.state_hist(1, m_high), log_opt2.state_hist(2, m_high), '-', 'Color', c_m2, 'LineWidth', 2.0, 'DisplayName', 'Opt 2: Dual (L/v+0.5s, 12D)');
grid on; box on; axis equal;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], 'GridColor', c_grid, 'GridAlpha', 0.8, 'YDir', 'reverse');
xlabel('X Position [m]', 'FontWeight', 'bold', 'Color', [0 0 0]);
ylabel('Y Position [m]', 'FontWeight', 'bold', 'Color', [0 0 0]);
title('(A) High-Curvature Trajectory Comparison (R \leq 7.5m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
legend('Location', 'northwest', 'FontSize', 8.5, 'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);

% (B) 반경별 위치 RMSE 비교 바 차트
subplot(2, 2, 2);
radius_labels = {'R=15m', 'R=10m', 'R=7.5m', 'R=5.0m', 'R=2.5m', 'R=1.25m'};
b = bar(rad_rmse, 0.8, 'EdgeColor', 'none');
b(1).FaceColor = c_m0;
b(2).FaceColor = c_mb;
b(3).FaceColor = c_m1;
b(4).FaceColor = c_m2;
grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], 'GridColor', c_grid, 'GridAlpha', 0.8, 'XTickLabel', radius_labels);
xlabel('Hairpin Curvature Radius [m]', 'FontWeight', 'bold', 'Color', [0 0 0]);
ylabel('Position RMSE [m]', 'FontWeight', 'bold', 'Color', [0 0 0]);
title('(B) Position Tracking RMSE by Hairpin Radius', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
legend(names, 'Location', 'northwest', 'FontSize', 8.5, 'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);
ylim([0, max(rad_rmse(:)) * 1.25]);

% (C) 제어 가능 고곡률 구간 (R = 15m ~ 5.0m) 확대 바 차트
subplot(2, 2, 3);
b_ctrl = bar(rad_rmse(1:4, :), 0.8, 'EdgeColor', 'none');
b_ctrl(1).FaceColor = c_m0;
b_ctrl(2).FaceColor = c_mb;
b_ctrl(3).FaceColor = c_m1;
b_ctrl(4).FaceColor = c_m2;
grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], 'GridColor', c_grid, 'GridAlpha', 0.8, 'XTickLabel', radius_labels(1:4));
xlabel('Controllable Hairpin Radius (R \geq 5.0m)', 'FontWeight', 'bold', 'Color', [0 0 0]);
ylabel('Position RMSE [m]', 'FontWeight', 'bold', 'Color', [0 0 0]);
title('(C) Zoomed Position RMSE in Controllable Turns (R = 15m ~ 5m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
legend(names, 'Location', 'northwest', 'FontSize', 8.5, 'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);
ylim([0, max(max(rad_rmse(1:4, :))) * 1.25]);

% (D) 반경별 속력 오차 RMSE 비교 바 차트
subplot(2, 2, 4);
b_spd = bar(rad_spd_rmse, 0.8, 'EdgeColor', 'none');
b_spd(1).FaceColor = c_m0;
b_spd(2).FaceColor = c_mb;
b_spd(3).FaceColor = c_m1;
b_spd(4).FaceColor = c_m2;
grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], 'GridColor', c_grid, 'GridAlpha', 0.8, 'XTickLabel', radius_labels);
xlabel('Hairpin Curvature Radius [m]', 'FontWeight', 'bold', 'Color', [0 0 0]);
ylabel('Speed Error RMSE [m/s]', 'FontWeight', 'bold', 'Color', [0 0 0]);
title('(D) Speed Maintenance RMSE by Hairpin Radius', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
legend(names, 'Location', 'northwest', 'FontSize', 8.5, 'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);
ylim([0, max(rad_spd_rmse(:)) * 1.25]);

sgtitle('Lookahead Strategy Comparison: Fixed (0.5s) vs Option 1 (Spatial L/v) vs Option 2 (Dual L/v + 0.5s)', ...
    'FontSize', 14.5, 'FontWeight', 'bold', 'Color', [0 0 0]);

out_img = fullfile(script_dir, 'savedAgents_lpv', 'Lookahead_Options_Comparison.png');
exportgraphics(h_fig, out_img, 'Resolution', 300);
close(h_fig);
save(fullfile(script_dir, 'savedAgents_lpv', 'lookahead_comparison_results.mat'), ...
    'overall_stats', 'rad_rmse', 'rad_spd_rmse', 'names', 'seg_radii');
fprintf('\n>>> 비교 그래프 및 결과 저장 완료: %s <<<\n', out_img);
