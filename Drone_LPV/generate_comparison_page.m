% generate_comparison_page.m
% Ep 0 (학습 0 / Baseline LPV) vs Ep 3000 (학습 완료 모델) 1:1 심층 비교 평가 및 시각화

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
cd(script_dir);
addpath(genpath(script_dir));

chk_dir = fullfile(script_dir, 'savedAgents_lpv');
v_eval = 5.5;

fprintf('=====================================================\n');
fprintf('Ep 0 vs Ep 3000 비교 시뮬레이션 시작 (v = %.1f m/s)\n', v_eval);
fprintf('=====================================================\n');

% 1. 에이전트 로드
load0 = load(fullfile(chk_dir, 'Agent0.mat'));
if isfield(load0, 'saved_agent'), ag0 = load0.saved_agent; else, ag0 = load0.agent; end
load3000 = load(fullfile(chk_dir, 'Agent3000.mat'));
if isfield(load3000, 'saved_agent'), ag3000 = load3000.saved_agent; else, ag3000 = load3000.agent; end

% 2. 시뮬레이션 환경 구성
[r_arr, th_arr, z_arr] = pathGen(false);
p = TrajectoryPlanner([0;0;-6], [r_arr; th_arr; z_arr], v_eval);
cum_L = p.cum_lengths;

seg_arcs  = [2, 4, 6, 8, 10, 12];
seg_radii = [15.0, 10.0, 7.5, 5.0, 2.5, 1.25];
seg_diam  = [30.0, 20.0, 15.0, 10.0, 5.0, 2.5];

% (1) Ep 0 시뮬레이션
env0 = RL_DroneEnv_LPV();
env0.randomize_speed = false; env0.randomize_path = false;
env0.cfg.target_speed = v_eval;
env0.cfg.drone1_init_states(4) = v_eval;
reset(env0);
cfg0 = env0.cfg; cfg0.target_speed = v_eval;
[log0, ~] = run_simulation_loop(env0.drone1, cfg0, ag0, env0);

% (2) Ep 3000 시뮬레이션
env3 = RL_DroneEnv_LPV();
env3.randomize_speed = false; env3.randomize_path = false;
env3.cfg.target_speed = v_eval;
env3.cfg.drone1_init_states(4) = v_eval;
reset(env3);
cfg3 = env3.cfg; cfg3.target_speed = v_eval;
[log3, ~] = run_simulation_loop(env3.drone1, cfg3, ag3000, env3);

% 3. 위치 오차 및 속력 유지 오차 계산
err0_3d = sqrt(sum((log0.state_hist(1:3,:) - log0.pos_des_hist(1:3,:)).^2, 1));
err3_3d = sqrt(sum((log3.state_hist(1:3,:) - log3.pos_des_hist(1:3,:)).^2, 1));

err0_z  = abs(log0.state_hist(3,:) - log0.pos_des_hist(3,:));
err3_z  = abs(log3.state_hist(3,:) - log3.pos_des_hist(3,:));

spd0_3d = sqrt(sum(log0.state_hist(4:6,:).^2, 1));
spd3_3d = sqrt(sum(log3.state_hist(4:6,:).^2, 1));
err0_spd = abs(spd0_3d - v_eval);
err3_spd = abs(spd3_3d - v_eval);

% 전체 통계
rmse0_all = sqrt(mean(err0_3d.^2));
mae0_all  = mean(err0_3d);
max0_all  = max(err0_3d);
z_rmse0_all = sqrt(mean(err0_z.^2));
spd_rmse0_all = sqrt(mean(err0_spd.^2));
spd_mae0_all  = mean(err0_spd);

rmse3_all = sqrt(mean(err3_3d.^2));
mae3_all  = mean(err3_3d);
max3_all  = max(err3_3d);
z_rmse3_all = sqrt(mean(err3_z.^2));
spd_rmse3_all = sqrt(mean(err3_spd.^2));
spd_mae3_all  = mean(err3_spd);

fprintf('\n[전체 코스 성능 비교]\n');
fprintf('Ep 0    : Pos_RMSE = %.4fm | Pos_MAE = %.4fm | Max = %.4fm | Spd_RMSE = %.4fm/s | Spd_MAE = %.4fm/s\n', ...
    rmse0_all, mae0_all, max0_all, spd_rmse0_all, spd_mae0_all);
fprintf('Ep 3000 : Pos_RMSE = %.4fm | Pos_MAE = %.4fm | Max = %.4fm | Spd_RMSE = %.4fm/s | Spd_MAE = %.4fm/s\n', ...
    rmse3_all, mae3_all, max3_all, spd_rmse3_all, spd_mae3_all);
fprintf('개선율   : Pos_RMSE %.2f%% | Pos_MAE %.2f%% | Max %.2f%% | Spd_MAE %.2f%%\n', ...
    (rmse0_all - rmse3_all)/rmse0_all*100, (mae0_all - mae3_all)/mae0_all*100, ...
    (max0_all - max3_all)/max0_all*100, (spd_mae0_all - spd_mae3_all)/spd_mae0_all*100);

% 4. 구간별(반경별) 오차 계산
s_h0 = v_eval * log0.t_hist;
s_h3 = v_eval * log3.t_hist;

num_arcs = length(seg_arcs);
radius_metrics = zeros(num_arcs, 10); % [D, R, rmse0, rmse3, mae0, mae3, max0, max3, spd_rmse0, spd_rmse3]

for i = 1:num_arcs
    seg_idx = seg_arcs(i);
    s_start = cum_L(seg_idx);
    s_end   = cum_L(seg_idx + 1);
    
    mask0 = (s_h0 >= s_start) & (s_h0 <= s_end);
    mask3 = (s_h3 >= s_start) & (s_h3 <= s_end);
    
    e0_sub = err0_3d(mask0);
    e3_sub = err3_3d(mask3);
    s0_sub = err0_spd(mask0);
    s3_sub = err3_spd(mask3);
    
    radius_metrics(i, 1) = seg_diam(i);
    radius_metrics(i, 2) = seg_radii(i);
    radius_metrics(i, 3) = sqrt(mean(e0_sub.^2));
    radius_metrics(i, 4) = sqrt(mean(e3_sub.^2));
    radius_metrics(i, 5) = mean(e0_sub);
    radius_metrics(i, 6) = mean(e3_sub);
    radius_metrics(i, 7) = max(e0_sub);
    radius_metrics(i, 8) = max(e3_sub);
    radius_metrics(i, 9) = sqrt(mean(s0_sub.^2));
    radius_metrics(i, 10) = sqrt(mean(s3_sub.^2));
end

% 5. 비교 시각화 (Light Theme 3x2 통합 대시보드: 흰색 배경에 검은색 글씨)
h_fig = figure('Name', 'Ep0 vs Ep3000 Comparison Dashboard (Light Theme)', ...
    'Units', 'pixels', 'Position', [80, 50, 1450, 1250], 'Color', [1, 1, 1]);

% 세련된 Light Theme 컬러 팔레트
c_des   = [0.15, 0.15, 0.15];     % 목표 궤적 (진회색 대시)
c_ep0   = [0.85, 0.28, 0.15];     % Ep 0 Untrained (선명한 코랄 레드)
c_ep3k  = [0.00, 0.42, 0.75];     % Ep 3000 Trained (신뢰감 있는 딥 블루)
c_grid  = [0.88, 0.88, 0.88];

% =========================================================================
% (Subplot 1) 고곡률(High-Curvature) 구간 2D 궤적 집중 비교
% =========================================================================
subplot(3, 2, 1);
s_thresh = 168.54; 
mask0_high = (s_h0 >= s_thresh);
mask3_high = (s_h3 >= s_thresh);

plot(log0.pos_des_hist(1, mask0_high), log0.pos_des_hist(2, mask0_high), '--', ...
    'Color', c_des, 'LineWidth', 2.2, 'DisplayName', 'Desired Path');
hold on;
plot(log0.state_hist(1, mask0_high), log0.state_hist(2, mask0_high), '-.', ...
    'Color', c_ep0, 'LineWidth', 1.8, 'DisplayName', 'Ep 0: Untrained (Pure LPV)');
plot(log3.state_hist(1, mask3_high), log3.state_hist(2, mask3_high), '-', ...
    'Color', c_ep3k, 'LineWidth', 2.2, 'DisplayName', 'Ep 3000: Trained (Residual PPO)');

text(22.0, 42.0, ' Hairpin 3 (R=7.5m) ', 'FontSize', 9, 'FontWeight', 'bold', ...
    'Color', [0 0 0], 'BackgroundColor', [1 1 1], 'EdgeColor', [0.4 0.4 0.4], 'Margin', 2);
text(-7.0, 52.0, ' Hairpin 4 (R=5.0m) ', 'FontSize', 9, 'FontWeight', 'bold', ...
    'Color', [0 0 0], 'BackgroundColor', [1 1 1], 'EdgeColor', [0.4 0.4 0.4], 'Margin', 2);
text(10.5, 65.0, ' Hairpin 5 (R=2.5m) ', 'FontSize', 9, 'FontWeight', 'bold', ...
    'Color', [0 0 0], 'BackgroundColor', [1 1 1], 'EdgeColor', [0.4 0.4 0.4], 'Margin', 2);
text(16.5, 78.0, ' Hairpin 6 (R=1.25m) ', 'FontSize', 9, 'FontWeight', 'bold', ...
    'Color', [0 0 0], 'BackgroundColor', [1 1 1], 'EdgeColor', [0.4 0.4 0.4], 'Margin', 2);

grid on; box on; axis equal;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], ...
    'GridColor', c_grid, 'GridAlpha', 0.8, 'LineWidth', 1.0);
set(gca, 'YDir', 'reverse');
xlabel('X Position [m]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
ylabel('Y Position [m]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
title('(A) High-Curvature Trajectory Focus (R \leq 7.5m, v = 5.5 m/s)', ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on', ...
    'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);

% =========================================================================
% (Subplot 2) 시간에 따른 3D 추종 오차 프로파일
% =========================================================================
subplot(3, 2, 2);
plot(log0.t_hist, err0_3d, 'Color', c_ep0, 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Ep 0 (Mean: %.3fm, Max: %.3fm)', mae0_all, max0_all));
hold on;
plot(log3.t_hist, err3_3d, 'Color', c_ep3k, 'LineWidth', 1.7, ...
    'DisplayName', sprintf('Ep 3000 (Mean: %.3fm, Max: %.3fm)', mae3_all, max3_all));

grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], ...
    'GridColor', c_grid, 'GridAlpha', 0.8, 'LineWidth', 1.0);
xlabel('Flight Time [s]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
ylabel('3D Tracking Error ||e_{pos}|| [m]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
title('(B) 3D Position Tracking Error Profile', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
xlim([0, max(log0.t_hist)]);
ylim([0, max(max0_all, max3_all) * 1.25]);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on', ...
    'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);

% =========================================================================
% (Subplot 3) 반경별 위치 RMSE 비교 바 차트 (\n 제거 완료)
% =========================================================================
subplot(3, 2, 3);
radius_labels = {'R=15m', 'R=10m', 'R=7.5m', 'R=5.0m', 'R=2.5m', 'R=1.25m'};
bar_data_rmse = [radius_metrics(:, 3), radius_metrics(:, 4)];
b = bar(bar_data_rmse, 0.75, 'EdgeColor', 'none');
b(1).FaceColor = c_ep0;
b(2).FaceColor = c_ep3k;
grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], ...
    'GridColor', c_grid, 'GridAlpha', 0.8, 'LineWidth', 1.0);
set(gca, 'XTickLabel', radius_labels, 'FontSize', 10);
xlabel('Hairpin Curvature Radius [m]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
ylabel('Position RMSE [m]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
title('(C) Position Tracking RMSE by Hairpin Radius', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
legend({'Ep 0 (Untrained)', 'Ep 3000 (Trained)'}, 'Location', 'northwest', ...
    'FontSize', 9, 'Box', 'on', 'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);
ylim([0, max(radius_metrics(:, 3)) * 1.2]);

% =========================================================================
% (Subplot 4) PPO 적응형 잔차 제어 액션
% =========================================================================
subplot(3, 2, 4);
t_act = log3.t_hist;
c_act1 = [0.00, 0.45, 0.74];
c_act2 = [0.85, 0.33, 0.10];
c_act3 = [0.20, 0.65, 0.20];
c_act4 = [0.55, 0.20, 0.65];

p1 = plot(t_act, log3.action_hist(1, :), 'Color', c_act1, 'LineWidth', 1.4, 'DisplayName', 'Action 1: \Delta P_{xy}'); hold on;
p2 = plot(t_act, log3.action_hist(2, :), 'Color', c_act2, 'LineWidth', 1.4, 'DisplayName', 'Action 2: \Delta D_{xy}');
p3 = plot(t_act, log3.action_hist(3, :), 'Color', c_act3, 'LineWidth', 1.4, 'DisplayName', 'Action 3: \Delta P_z');
p4 = plot(t_act, log3.action_hist(4, :), 'Color', c_act4, 'LineWidth', 1.4, 'DisplayName', 'Action 4: \Delta D_z');
p0 = yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Ep 0 Baseline (Action=0)');

grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], ...
    'GridColor', c_grid, 'GridAlpha', 0.8, 'LineWidth', 1.0);
xlabel('Flight Time [s]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
ylabel('Normalized Action [-1, 1]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
title('(D) Ep 3000 Adaptive PPO Residual Actions', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
xlim([0, max(t_act)]);
ylim([-1.35, 1.65]);
legend([p1, p2, p3, p4, p0], 'Location', 'north', 'Orientation', 'horizontal', ...
    'NumColumns', 3, 'FontSize', 8.5, 'Box', 'on', 'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);

% =========================================================================
% (Subplot 5) 시간에 따른 비행 속력 유지 프로파일 (v = 5.5 m/s 기준)
% =========================================================================
subplot(3, 2, 5);
plot(log0.t_hist, spd0_3d, 'Color', c_ep0, 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Ep 0 (Mean: %.2f m/s, MAE: %.3f m/s)', mean(spd0_3d), spd_mae0_all));
hold on;
plot(log3.t_hist, spd3_3d, 'Color', c_ep3k, 'LineWidth', 1.7, ...
    'DisplayName', sprintf('Ep 3000 (Mean: %.2f m/s, MAE: %.3f m/s)', mean(spd3_3d), spd_mae3_all));
yline(v_eval, '--', 'Color', c_des, 'LineWidth', 1.6, 'DisplayName', sprintf('Target Speed (%.1f m/s)', v_eval));

grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], ...
    'GridColor', c_grid, 'GridAlpha', 0.8, 'LineWidth', 1.0);
xlabel('Flight Time [s]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
ylabel('Flight Speed ||v|| [m/s]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
title('(E) Flight Speed Maintenance Profile (Target = 5.5 m/s)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
xlim([0, max(log0.t_hist)]);
ylim([0, max(max(spd0_3d), max(spd3_3d)) * 1.25]);
legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on', ...
    'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);

% =========================================================================
% (Subplot 6) 반경별 속력 오차 RMSE 비교 바 차트
% =========================================================================
subplot(3, 2, 6);
bar_data_spd = [radius_metrics(:, 9), radius_metrics(:, 10)];
b_spd = bar(bar_data_spd, 0.75, 'EdgeColor', 'none');
b_spd(1).FaceColor = c_ep0;
b_spd(2).FaceColor = c_ep3k;
grid on; box on;
set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], ...
    'GridColor', c_grid, 'GridAlpha', 0.8, 'LineWidth', 1.0);
set(gca, 'XTickLabel', radius_labels, 'FontSize', 10);
xlabel('Hairpin Curvature Radius [m]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
ylabel('Speed Error RMSE [m/s]', 'FontWeight', 'bold', 'FontSize', 10.5, 'Color', [0 0 0]);
title('(F) Speed Tracking RMSE by Hairpin Radius', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
legend({'Ep 0 (Untrained)', 'Ep 3000 (Trained)'}, 'Location', 'northwest', ...
    'FontSize', 9, 'Box', 'on', 'TextColor', [0 0 0], 'Color', [1 1 1], 'EdgeColor', [0.4 0.4 0.4]);
ylim([0, max(radius_metrics(:, 9)) * 1.25]);

% 전체 제목 (검은색 글씨)
sgtitle(sprintf('Model Comparison: Untrained (Ep 0) vs Trained (Ep 3000) at v = %.1f m/s', v_eval), ...
    'FontSize', 15, 'FontWeight', 'bold', 'Color', [0 0 0]);

% 이미지 저장
out_fig_path = fullfile(chk_dir, 'Ep0_vs_Ep3000_Comparison_Page.png');
exportgraphics(h_fig, out_fig_path, 'Resolution', 300);
close(h_fig);
fprintf('>>> [저장 완료] 종합 비교 이미지: %s <<<\n', out_fig_path);

% 6. CSV 저장
summary_file = fullfile(chk_dir, 'Ep0_vs_Ep3000_Comparison_Table.csv');
fid = fopen(summary_file, 'w');
fprintf(fid, 'Section,Diameter_m,Radius_m,Ep0_RMSE_m,Ep3000_RMSE_m,RMSE_Imprv_Pct,Ep0_MAE_m,Ep3000_MAE_m,MAE_Imprv_Pct,Ep0_Max_m,Ep3000_Max_m,Max_Imprv_Pct\n');
for i = 1:num_arcs
    r_imprv = (radius_metrics(i,3) - radius_metrics(i,4)) / radius_metrics(i,3) * 100;
    m_imprv = (radius_metrics(i,5) - radius_metrics(i,6)) / radius_metrics(i,5) * 100;
    x_imprv = (radius_metrics(i,7) - radius_metrics(i,8)) / radius_metrics(i,7) * 100;
    fprintf(fid, 'Hairpin_%d,%.1f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f\n', ...
        i, radius_metrics(i,1), radius_metrics(i,2), ...
        radius_metrics(i,3), radius_metrics(i,4), r_imprv, ...
        radius_metrics(i,5), radius_metrics(i,6), m_imprv, ...
        radius_metrics(i,7), radius_metrics(i,8), x_imprv);
end
fprintf(fid, 'Overall,339.6m,-,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f\n', ...
    rmse0_all, rmse3_all, (rmse0_all - rmse3_all)/rmse0_all*100, ...
    mae0_all, mae3_all, (mae0_all - mae3_all)/mae0_all*100, ...
    max0_all, max3_all, (max0_all - max3_all)/max0_all*100);
fclose(fid);
fprintf('>>> [저장 완료] 비교 데이터 CSV: %s <<<\n', summary_file);
