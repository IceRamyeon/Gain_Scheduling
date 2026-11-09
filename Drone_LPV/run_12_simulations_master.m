% run_12_simulations_master.m
% 제어 모델(Feedforward Only, Applied Gain Scheduling, PPO Residual) x 3가지 속도(3.5, 5.5, 7.5 m/s)
% 일괄 시뮬레이션 수행 및 비교 그래프 저장 스크립트 (공용 run_simulation_loop 사용)

close all; clc; clear;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end

set(groot, 'DefaultFigureVisible', 'off');

speeds = [3.5, 5.5, 7.5];
base_results_dir = fullfile(script_dir, 'simulation_results');
if ~exist(base_results_dir, 'dir'), mkdir(base_results_dir); end

agent_file = 'saved_drone_lpv_ppo_agent.mat';
if ~isfile(agent_file)
    error('PPO 에이전트 가중치 파일(%s)이 존재하지 않습니다.', agent_file);
end
loaded_agent = load(agent_file, 'agent');
agent = loaded_agent.agent;

summary_metrics = struct();
calc_metrics = @(log_d, name_str) struct(...
    'Name', name_str, ...
    'RMSE', sqrt(mean(sum((log_d.state_hist(1:3,:) - log_d.pos_des_hist).^2, 1))), ...
    'MAE',  mean(sqrt(sum((log_d.state_hist(1:3,:) - log_d.pos_des_hist).^2, 1))), ...
    'MAX',  max(sqrt(sum((log_d.state_hist(1:3,:) - log_d.pos_des_hist).^2, 1))), ...
    'Z_RMSE', sqrt(mean((log_d.state_hist(3,:) - log_d.pos_des_hist(3,:)).^2)));

for s_idx = 1:length(speeds)
    v_tar   = speeds(s_idx);
    v_field = sprintf('v_%s', strrep(sprintf('%.1f', v_tar), '.', '_'));
    speed_dir = fullfile(base_results_dir, sprintf('v=%.1f_ms', v_tar));
    if ~exist(speed_dir, 'dir'), mkdir(speed_dir); end

    fprintf('\n>>> [속도: %.1f m/s] 모델 비교 시뮬레이션 시작 <<<\n', v_tar);

    get_cfg = @(ff, z_sched, model_name) struct(...
        'start_with_yaw', true, 'dt', 0.01, 'target_yaw', [], 'target_speed', v_tar, ...
        'Feedforward', ff, 'Z_Gain_Scheduling', z_sched, ...
        'play_animation', false, 'auto_save', true, 'close_figures', true, ...
        'save_dir', fullfile(speed_dir, model_name), 'file_prefix', model_name, ...
        'drone1_init_states', [0.0, 0.0, -6.0, v_tar, 0, 0, 0, 0, 0, 0, 0, 0]', ...
        'm', 1.25, 'armLength', 0.265, 'Ixx', 0.0232, 'Iyy', 0.0232, 'Izz', 0.0468, ...
        'posGain', containers.Map({'P_x','I_x','D_x','P_y','I_y','D_y','P_z','I_z','D_z'}, ...
                                  {13.5, 5.0, 5.5, 13.5, 5.0, 5.5, 16.88, 6.25, 6.88}), ...
        'attGain', containers.Map({'P_phi','I_phi','D_phi','P_theta','I_theta','D_theta','P_psi','I_psi','D_psi','P_zdot','I_zdot','D_zdot'}, ...
                                  {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0}));

    % [Model 2] Feedforward Only
    cfg2   = get_cfg(true, false, 'Model2_Feedforward_NoZ');
    drone2 = init_drone_and_env(cfg2);
    [log2, ~] = run_simulation_loop(drone2, cfg2);
    finalize_and_save_results(log2, cfg2);
    summary_metrics.(v_field).Model2 = calc_metrics(log2, 'Model 2: Feedforward Only');

    % [Model 3] Applied Gain Scheduling (Drone_LPV)
    cfg3   = get_cfg(true, true, 'Model3_Drone_LPV');
    drone3 = init_drone_and_env(cfg3);
    [log3, ~] = run_simulation_loop(drone3, cfg3);
    finalize_and_save_results(log3, cfg3);
    summary_metrics.(v_field).Model3 = calc_metrics(log3, 'Model 3: Applied Gain Scheduling');

    % [Model 4] PPO Residual (Drone_LPV + Residual PPO)
    env4 = RL_DroneEnv_LPV();
    env4.randomize_speed = false;
    env4.randomize_path  = false;
    env4.cfg.target_speed = v_tar;
    env4.cfg.drone1_init_states(4) = v_tar;
    reset(env4);

    cfg4 = get_cfg(true, true, 'Model4_PPO');
    [log4, ~] = run_simulation_loop(env4.drone1, cfg4, agent, env4);
    finalize_and_save_results(log4, cfg4);
    summary_metrics.(v_field).Model4 = calc_metrics(log4, 'Model 4: PPO Residual');

    % 속도별 3개 모델 통합 비교 그래프 저장
    err2   = sqrt(sum((log2.state_hist(1:3,:) - log2.pos_des_hist).^2, 1));
    err3   = sqrt(sum((log3.state_hist(1:3,:) - log3.pos_des_hist).^2, 1));
    err4   = sqrt(sum((log4.state_hist(1:3,:) - log4.pos_des_hist).^2, 1));
    z_err2 = abs(log2.state_hist(3,:) - log2.pos_des_hist(3,:));
    z_err3 = abs(log3.state_hist(3,:) - log3.pos_des_hist(3,:));
    z_err4 = abs(log4.state_hist(3,:) - log4.pos_des_hist(3,:));

    h_cmp = figure('Name', sprintf('Model Comparison at v=%.1f m/s', v_tar), ...
                   'Theme', 'light', 'Color', 'w', 'Position', [100, 100, 1200, 700], 'Visible', 'off');

    subplot(2, 2, [1, 3]);
    plot(log2.pos_des_hist(1,:), log2.pos_des_hist(2,:), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired Trajectory'); hold on;
    plot(log2.state_hist(1,:), log2.state_hist(2,:), 'm-.', 'LineWidth', 1.4, 'DisplayName', '1. Feedforward Only');
    plot(log3.state_hist(1,:), log3.state_hist(2,:), 'b--', 'LineWidth', 1.6, 'DisplayName', '2. Applied Gain Scheduling');
    plot(log4.state_hist(1,:), log4.state_hist(2,:), 'Color', [0.0 0.6 0.2], 'LineWidth', 2.0, 'DisplayName', '3. Proposed: PPO Residual');
    set(gca, 'YDir', 'Reverse'); axis equal; grid on;
    title(sprintf('Trajectory Comparison (v = %.1f m/s)', v_tar));
    xlabel('X [m]'); ylabel('Y [m]'); legend('Location', 'best');

    subplot(2, 2, 2);
    plot(log2.t_hist, err2, 'm-.', 'LineWidth', 1.3, 'DisplayName', '1. Feedforward Only'); hold on;
    plot(log3.t_hist, err3, 'b--', 'LineWidth', 1.5, 'DisplayName', '2. Applied Gain Scheduling');
    plot(log4.t_hist, err4, 'Color', [0.0 0.6 0.2], 'LineWidth', 1.8, 'DisplayName', '3. Proposed: PPO Residual');
    grid on; title('3D Tracking Error ||e_{pos}|| [m]'); xlabel('Time [s]'); ylabel('Error [m]'); legend('Location', 'northeast');

    subplot(2, 2, 4);
    plot(log2.t_hist, z_err2, 'm-.', 'LineWidth', 1.3, 'DisplayName', '1. Feedforward Only'); hold on;
    plot(log3.t_hist, z_err3, 'b--', 'LineWidth', 1.5, 'DisplayName', '2. Applied Gain Scheduling');
    plot(log4.t_hist, z_err4, 'Color', [0.0 0.6 0.2], 'LineWidth', 1.8, 'DisplayName', '3. Proposed: PPO Residual');
    grid on; title('Z-axis Error |z - z_{des}| [m]'); xlabel('Time [s]'); ylabel('Z Error [m]'); legend('Location', 'northeast');

    exportgraphics(h_cmp, fullfile(speed_dir, sprintf('Combined_Comparison_v=%.1f.png', v_tar)), 'Resolution', 300);
    close(h_cmp);

    v_metrics = summary_metrics.(v_field);
    save(fullfile(speed_dir, sprintf('Summary_Metrics_v=%.1f.mat', v_tar)), 'v_metrics', 'v_tar');
end

save(fullfile(base_results_dir, 'all_12_simulations_summary.mat'), 'summary_metrics', 'speeds');
fprintf('\n모든 비교 시뮬레이션 및 저장 완료!\n');
