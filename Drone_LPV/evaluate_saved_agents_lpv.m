% evaluate_saved_agents_lpv.m
% savedAgents_lpv 폴더 내의 체크포인트(Agent0, Agent100, Agent200, ...)들을 순회 평가하고,
% 에피소드 진행에 따른 성능 발전 과정(2D 궤적, 3D 추종 오차, PPO 잔차 액션, 학습 발전 추이)을 한눈에 비교 시각화

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end

clearvars -except script_dir; clc; close all;

fprintf('========================================================================\n');
fprintf('  savedAgents_lpv 체크포인트별 시뮬레이션 평가 및 학습 발전 과정 시각화  \n');
fprintf('========================================================================\n');

chk_dir = fullfile(script_dir, 'savedAgents_lpv');
if ~isfolder(chk_dir)
    error('savedAgents_lpv 폴더가 없습니다.');
end

agent_files = dir(fullfile(chk_dir, 'Agent*.mat'));
if isempty(agent_files)
    disp('savedAgents_lpv 폴더에 저장된 체크포인트 파일이 없습니다.');
    return;
end

% 파일명 내 에피소드 번호 기준 정렬 (Agent0 -> Agent100 -> Agent200 ...)
ep_numbers = zeros(1, length(agent_files));
for i = 1:length(agent_files)
    num_str = regexp(agent_files(i).name, '\d+', 'match');
    if ~isempty(num_str)
        ep_numbers(i) = str2double(num_str{1});
    else
        ep_numbers(i) = -1;
    end
end
[ep_numbers, sort_idx] = sort(ep_numbers);
agent_files = agent_files(sort_idx);

% 결과 요약 저장용 컨테이너
results = struct();
num_checkpoints = length(agent_files);

% 색상 팔레트 설정 (초기 -> 후기 점진적 변화)
colors = parula(max(num_checkpoints, 3));

for f_idx = 1:num_checkpoints
    curr_file = agent_files(f_idx).name;
    ep_num    = ep_numbers(f_idx);
    ep_tag    = sprintf('Ep_%d', ep_num);
    fprintf('\n>>> [%d/%d] 체크포인트 평가 중: %s (Episode %d) <<<\n', ...
        f_idx, num_checkpoints, curr_file, ep_num);

    loaded = load(fullfile(chk_dir, curr_file));
    if isfield(loaded, 'saved_agent')
        agent = loaded.saved_agent;
    else
        agent = loaded.agent;
    end

    % 평가 환경 초기화 (고정 벤치마크 경로, 기준 속도 4.0 m/s)
    env = RL_DroneEnv_LPV();
    env.randomize_speed = false;
    env.randomize_path  = false;
    env.cfg.target_speed = 4.0;
    env.cfg.drone1_init_states(4) = 4.0;
    reset(env);

    cfg = env.cfg;
    cfg.play_animation = false;
    cfg.auto_save      = true;
    cfg.close_figures  = true;
    cfg.target_speed   = 4.0;
    cfg.save_dir       = fullfile(chk_dir, sprintf('Result_Agent%d', ep_num));
    cfg.file_prefix    = sprintf('Agent%d', ep_num);

    [log_data, ~] = run_simulation_loop(env.drone1, cfg, agent, env);
    finalize_and_save_results(log_data, cfg);

    % 지표 계산
    err_pos = sqrt(sum((log_data.state_hist(1:3, :) - log_data.pos_des_hist(1:3, :)).^2, 1));
    z_err   = abs(log_data.state_hist(3, :) - log_data.pos_des_hist(3, :));

    results.(ep_tag).ep_num   = ep_num;
    results.(ep_tag).log_data = log_data;
    results.(ep_tag).rmse     = sqrt(mean(err_pos.^2));
    results.(ep_tag).mae      = mean(err_pos);
    results.(ep_tag).max_err  = max(err_pos);
    results.(ep_tag).z_rmse   = sqrt(mean(z_err.^2));

    fprintf('  -> Ep %d: 3D RMSE = %.4f m, Max = %.4f m, Z RMSE = %.4f m\n', ...
        ep_num, results.(ep_tag).rmse, results.(ep_tag).max_err, results.(ep_tag).z_rmse);
end

%% =========================================================================
%% 1. 학습 진행에 따른 종합 비교 시각화 (Learning Progression Comparison)
%% =========================================================================
fprintf('\n>>> 학습 진행 종합 비교 플롯 생성 중... <<<\n');

h_prog = figure('Name', 'PPO Learning Progression by Checkpoint', ...
    'Theme', 'light', 'Color', 'w', 'Position', [100, 80, 1300, 800], 'Visible', 'off');

% [Subplot 1] 2D 비행 궤적 비교
subplot(2, 2, [1, 3]);
ax1 = gca;
ref_log = results.(sprintf('Ep_%d', ep_numbers(1))).log_data;
plot(ref_log.pos_des_hist(1,:), ref_log.pos_des_hist(2,:), 'k--', 'LineWidth', 2.2, 'DisplayName', 'Desired Path'); hold on;

for k = 1:num_checkpoints
    ep_n = ep_numbers(k);
    lg   = results.(sprintf('Ep_%d', ep_n)).log_data;
    if ep_n == 0
        disp_name = 'Ep 0 (Pure LPV Baseline)';
        line_style = ':';
        l_width = 1.8;
    elseif k == num_checkpoints
        disp_name = sprintf('Ep %d (Final Trained)', ep_n);
        line_style = '-';
        l_width = 2.2;
    else
        disp_name = sprintf('Ep %d', ep_n);
        line_style = '-.';
        l_width = 1.2;
    end
    plot(lg.state_hist(1,:), lg.state_hist(2,:), 'LineStyle', line_style, ...
        'Color', colors(k, :), 'LineWidth', l_width, 'DisplayName', disp_name);
end
set(ax1, 'YDir', 'Reverse'); axis equal; grid on;
xlabel('X Position [m]', 'FontWeight', 'bold'); ylabel('Y Position [m]', 'FontWeight', 'bold');
title('2D Trajectory Progression by Checkpoint', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

% [Subplot 2] 3D 추종 오차 시간 이력 비교
subplot(2, 2, 2);
for k = 1:num_checkpoints
    ep_n = ep_numbers(k);
    lg   = results.(sprintf('Ep_%d', ep_n)).log_data;
    err  = sqrt(sum((lg.state_hist(1:3, :) - lg.pos_des_hist(1:3, :)).^2, 1));
    plot(lg.t_hist, err, 'Color', colors(k, :), 'LineWidth', 1.4, ...
        'DisplayName', sprintf('Ep %d (RMSE: %.3fm)', ep_n, results.(sprintf('Ep_%d', ep_n)).rmse));
    hold on;
end
grid on; xlabel('Time [s]'); ylabel('3D Error ||e_{pos}|| [m]');
title('3D Tracking Error Profiles Over Time', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 8.5);

% [Subplot 3] 에피소드별 RMSE 및 Max 오차 개선 추이 바/라인 차트
subplot(2, 2, 4);
all_eps  = ep_numbers;
all_rmse = zeros(1, num_checkpoints);
all_max  = zeros(1, num_checkpoints);
for k = 1:num_checkpoints
    all_rmse(k) = results.(sprintf('Ep_%d', all_eps(k))).rmse;
    all_max(k)  = results.(sprintf('Ep_%d', all_eps(k))).max_err;
end

yyaxis left;
b = bar(categorical(all_eps), all_rmse, 0.5, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.85);
ylabel('RMSE [m]', 'FontWeight', 'bold');
ylim([0, max(all_rmse) * 1.35]);

yyaxis right;
plot(categorical(all_eps), all_max, '-ro', 'LineWidth', 1.8, 'MarkerFaceColor', 'r');
ylabel('Max Error [m]', 'FontWeight', 'bold');
ylim([0, max(all_max) * 1.35]);

grid on; xlabel('Trained Episodes', 'FontWeight', 'bold');
title('Error Convergence by Checkpoint (Ep 0 \rightarrow End)', 'FontSize', 11, 'FontWeight', 'bold');

% 종합 비교 그래프 저장
prog_fig_path = fullfile(chk_dir, 'Learning_Progression_Comparison.png');
exportgraphics(h_prog, prog_fig_path, 'Resolution', 300);
close(h_prog);
fprintf('>>> [저장 완료] 종합 학습 발전 비교 그래프: %s <<<\n', prog_fig_path);

%% =========================================================================
%% 2. 체크포인트별 PPO 잔차 액션(오프셋) 비교 시각화
%% =========================================================================
h_act = figure('Name', 'PPO Residual Action Evolution', ...
    'Theme', 'light', 'Color', 'w', 'Position', [150, 100, 1100, 750], 'Visible', 'off');

action_names = {'\Delta P_{xy} (Lateral P)', '\Delta D_{xy} (Lateral Damping)', ...
                '\Delta P_z (Vertical P)', '\Delta D_z (Vertical Damping)'};

for act_idx = 1:4
    subplot(2, 2, act_idx);
    for k = 1:num_checkpoints
        ep_n = ep_numbers(k);
        lg   = results.(sprintf('Ep_%d', ep_n)).log_data;
        plot(lg.t_hist, lg.action_hist(act_idx, :), 'Color', colors(k, :), 'LineWidth', 1.3, ...
            'DisplayName', sprintf('Ep %d', ep_n));
        hold on;
    end
    grid on; xlabel('Time [s]'); ylabel('Action Output [-1, 1]');
    title(action_names{act_idx}, 'FontWeight', 'bold');
    if act_idx == 1, legend('Location', 'best', 'FontSize', 8); end
end

act_fig_path = fullfile(chk_dir, 'PPO_Residual_Action_Evolution.png');
exportgraphics(h_act, act_fig_path, 'Resolution', 300);
close(h_act);
fprintf('>>> [저장 완료] PPO 잔차 액션 발전 비교 그래프: %s <<<\n', act_fig_path);

% 요약 데이터 MAT 파일 저장
save(fullfile(chk_dir, 'checkpoint_evaluation_summary.mat'), 'results', 'ep_numbers');
fprintf('\n========================================================================\n');
fprintf('  모든 체크포인트 평가 및 비교 그래프 생성/저장 완료!\n');
fprintf('========================================================================\n');
