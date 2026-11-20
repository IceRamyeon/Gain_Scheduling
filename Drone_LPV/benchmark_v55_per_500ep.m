% benchmark_v55_per_500ep.m
% 비행 속력 v = 5.5 m/s 기준 339.6m 고난도 벤치마크 코스 시뮬레이션
% Ep 0, 500, 1000, 1500, 2000, 2500, 3000 체크포인트 비교 평가

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
cd(script_dir);
addpath(genpath(script_dir));

chk_dir = fullfile(script_dir, 'savedAgents_lpv');
[r_arr, th_arr, z_arr] = pathGen(false);
v_eval = 5.5;

p = TrajectoryPlanner([0;0;-6], [r_arr; th_arr; z_arr], v_eval);
cum_L = p.cum_lengths;

% 6개 반원 코스
seg_arcs  = [2, 4, 6, 8, 10, 12];
seg_radii = [15.0, 10.0, 7.5, 5.0, 2.5, 1.25];
seg_diam  = [30.0, 20.0, 15.0, 10.0, 5.0, 2.5];
target_eps = [0, 500, 1000, 1500, 2000, 2500, 3000];

% 1. 시뮬레이션 실행 및 로그 수집
sim_logs = cell(1, length(target_eps));
overall_metrics = cell(length(target_eps), 6);
res_table = cell(length(target_eps) * length(seg_radii), 8);
row_idx = 1;

for e_idx = 1:length(target_eps)
    ep = target_eps(e_idx);
    mat_path = fullfile(chk_dir, sprintf('Agent%d.mat', ep));
    loaded = load(mat_path);
    if isfield(loaded, 'saved_agent')
        agent = loaded.saved_agent;
    else
        agent = loaded.agent;
    end
    
    env = RL_DroneEnv_LPV();
    env.randomize_speed = false;
    env.randomize_path  = false;
    env.cfg.target_speed = v_eval;
    env.cfg.drone1_init_states(4) = v_eval;
    reset(env);
    
    cfg = env.cfg;
    cfg.target_speed = v_eval;
    
    [log_d, ~] = run_simulation_loop(env.drone1, cfg, agent, env);
    sim_logs{e_idx} = log_d;
    
    t_h = log_d.t_hist;
    s_h = v_eval * t_h;
    
    e_pos = sqrt(sum((log_d.state_hist(1:3, :) - log_d.pos_des_hist(1:3, :)).^2, 1));
    e_z   = abs(log_d.state_hist(3, :) - log_d.pos_des_hist(3, :));
    
    % 전체 코스 메트릭
    overall_metrics{e_idx, 1} = ep;
    overall_metrics{e_idx, 2} = sqrt(mean(e_pos.^2));
    overall_metrics{e_idx, 3} = mean(e_pos);
    overall_metrics{e_idx, 4} = max(e_pos);
    overall_metrics{e_idx, 5} = sqrt(mean(e_z.^2));
    overall_metrics{e_idx, 6} = max(e_z);
    
    % 반경별 메트릭
    for a_idx = 1:length(seg_arcs)
        s_idx_arc = seg_arcs(a_idx);
        r_val = seg_radii(a_idx);
        d_val = seg_diam(a_idx);
        
        s_start = cum_L(s_idx_arc);
        s_end   = cum_L(s_idx_arc + 1);
        
        mask = (s_h >= s_start) & (s_h <= s_end);
        sub_err = e_pos(mask);
        sub_z   = e_z(mask);
        
        res_table{row_idx, 1} = ep;
        res_table{row_idx, 2} = d_val;
        res_table{row_idx, 3} = r_val;
        res_table{row_idx, 4} = sqrt(mean(sub_err.^2));
        res_table{row_idx, 5} = mean(sub_err);
        res_table{row_idx, 6} = max(sub_err);
        res_table{row_idx, 7} = sqrt(mean(sub_z.^2));
        
        row_idx = row_idx + 1;
    end
end

T_radius = cell2table(res_table(1:row_idx-1, 1:7), 'VariableNames', ...
    {'Episode', 'Diameter_m', 'Radius_m', 'RMSE_m', 'MAE_m', 'Max_m', 'Z_RMSE_m'});

T_overall = cell2table(overall_metrics, 'VariableNames', ...
    {'Episode', 'RMSE_m', 'MAE_m', 'Max_m', 'Z_RMSE_m', 'Z_Max_m'});

disp('========================================================================');
disp('   [전체 코스 요약 결과 (v = 5.5 m/s)]');
disp('========================================================================');
disp(T_overall);

disp('========================================================================');
disp('   [반경별 상세 결과 (v = 5.5 m/s)]');
disp('========================================================================');
disp(T_radius);

% CSV 저장
writetable(T_overall, fullfile(chk_dir, 'benchmark_v55_overall_summary.csv'));
writetable(T_radius,  fullfile(chk_dir, 'benchmark_v55_metrics_by_radius.csv'));

%% =========================================================================
%% 2. 시각화 그래프 생성 (v = 5.5 m/s 비교 플롯)
%% =========================================================================
h_fig = figure('Name', 'Benchmark Progression at v=5.5m/s', ...
    'Theme', 'light', 'Color', 'w', 'Position', [80, 60, 1300, 800], 'Visible', 'off');

colors = parula(length(target_eps));

% (1) 2D 궤적 비교
subplot(2, 2, [1, 3]);
ax1 = gca;
plot(sim_logs{1}.pos_des_hist(1,:), sim_logs{1}.pos_des_hist(2,:), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired'); hold on;

for k = 1:length(target_eps)
    ep = target_eps(k);
    lg = sim_logs{k};
    if ep == 0
        l_name = 'Ep 0 (Pure LPV)';
        l_style = ':'; l_w = 1.8;
    elseif ep == 3000
        l_name = 'Ep 3000 (Final PPO)';
        l_style = '-'; l_w = 2.2;
    else
        l_name = sprintf('Ep %d', ep);
        l_style = '-.'; l_w = 1.2;
    end
    plot(lg.state_hist(1,:), lg.state_hist(2,:), 'LineStyle', l_style, ...
        'Color', colors(k,:), 'LineWidth', l_w, 'DisplayName', l_name);
end
set(ax1, 'YDir', 'Reverse'); axis equal; grid on;
xlabel('X Position [m]', 'FontWeight', 'bold'); ylabel('Y Position [m]', 'FontWeight', 'bold');
title('2D Trajectory Progression (v = 5.5 m/s)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 8.5);

% (2) 3D 추종 오차 시계열
subplot(2, 2, 2);
for k = 1:length(target_eps)
    ep = target_eps(k);
    lg = sim_logs{k};
    err = sqrt(sum((lg.state_hist(1:3, :) - lg.pos_des_hist(1:3, :)).^2, 1));
    plot(lg.t_hist, err, 'Color', colors(k,:), 'LineWidth', 1.3, ...
        'DisplayName', sprintf('Ep %d (RMSE: %.3fm)', ep, overall_metrics{k, 2}));
    hold on;
end
grid on; xlabel('Time [s]'); ylabel('3D Error [m]');
title('3D Tracking Error vs Time (v = 5.5 m/s)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 8);

% (3) RMSE & Max 개선 바 차트
subplot(2, 2, 4);
all_eps  = cell2mat(overall_metrics(:, 1));
all_rmse = cell2mat(overall_metrics(:, 2));
all_max  = cell2mat(overall_metrics(:, 4));

yyaxis left;
bar(categorical(all_eps), all_rmse, 0.5, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.85);
ylabel('RMSE [m]', 'FontWeight', 'bold');
ylim([0, max(all_rmse) * 1.3]);

yyaxis right;
plot(categorical(all_eps), all_max, '-ro', 'LineWidth', 1.8, 'MarkerFaceColor', 'r');
ylabel('Max Error [m]', 'FontWeight', 'bold');
ylim([0, max(all_max) * 1.3]);

grid on; xlabel('Episode', 'FontWeight', 'bold');
title('Overall Error Convergence (v = 5.5 m/s)', 'FontSize', 11, 'FontWeight', 'bold');

img_save_path = fullfile(chk_dir, 'Benchmark_Progression_v55.png');
exportgraphics(h_fig, img_save_path, 'Resolution', 300);
close(h_fig);
fprintf('>>> [저장 완료] v=5.5 m/s 비교 그래프: %s <<<\n', img_save_path);

