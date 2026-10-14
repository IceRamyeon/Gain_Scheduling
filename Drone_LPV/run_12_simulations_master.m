% run_12_simulations_master.m
% 4가지 제어 모델 x 3가지 속도(3.5, 5.5, 7.5 m/s) = 총 12회 시뮬레이션 일괄 수행 및 결과 저장
% 
% 4가지 모델:
% 1. Model 1: Hovering 기준 튜닝 모델 (cfg.Feedforward = false, cfg.Z_Gain_Scheduling = false)
% 2. Model 2: cfg.Feedforward = true, cfg.Z_Gain_Scheduling = false
% 3. Model 3: cfg.Feedforward = true, cfg.Z_Gain_Scheduling = true (Drone_LPV)
% 4. Model 4: PPO 모델 (Drone_LPV + Residual PPO)
%
% 3가지 속도: v = 3.5 m/s, 5.5 m/s, 7.5 m/s

close all; clc; clear;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end

% Set figures to offscreen/invisible for fast and reliable headless generation
set(groot, 'DefaultFigureVisible', 'off');

speeds = [3.5, 5.5, 7.5];
base_results_dir = fullfile(script_dir, 'simulation_results');
if ~exist(base_results_dir, 'dir')
    mkdir(base_results_dir);
end

% 에이전트 로드
agent_file = 'saved_drone_lpv_ppo_agent.mat';
if ~isfile(agent_file)
    error('PPO 에이전트 가중치 파일(%s)이 존재하지 않습니다.', agent_file);
end
loaded_agent = load(agent_file, 'agent');
agent = loaded_agent.agent;

% 결과 요약 구조체
summary_metrics = struct();

fprintf('========================================================================\n');
fprintf('  12회 제어 모델 시뮬레이션 일괄 수행 시작 (3가지 속도 x 4가지 모델)\n');
fprintf('========================================================================\n');

for s_idx = 1:length(speeds)
    v_tar = speeds(s_idx);
    v_str = sprintf('%.1f', v_tar);
    v_field = sprintf('v_%s', strrep(v_str, '.', '_'));
    
    % 속도별 디렉터리 생성 (예: simulation_results/v=3.5_ms)
    speed_dir = fullfile(base_results_dir, sprintf('v=%.1f_ms', v_tar));
    if ~exist(speed_dir, 'dir')
        mkdir(speed_dir);
    end
    
    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n');
    fprintf('  [속도: %.1f m/s] 시뮬레이션 시작 (저장 경로: %s)\n', v_tar, speed_dir);
    fprintf('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n');
    
    %% 공통 기본 설정 헬퍼
    get_cfg = @(ff, z_sched, model_name) struct(...
        'start_with_yaw', true, ...
        'dt', 0.01, ...
        'target_yaw', [], ...
        'target_speed', v_tar, ...
        'Feedforward', ff, ...
        'Z_Gain_Scheduling', z_sched, ...
        'play_animation', false, ...
        'auto_save', true, ...
        'close_figures', true, ...
        'save_dir', fullfile(speed_dir, model_name), ...
        'file_prefix', model_name, ...
        'drone1_init_states', [0.0, 0.0, -6.0, v_tar, 0, 0, 0, 0, 0, 0, 0, 0]', ...
        'm', 1.25, ...
        'armLength', 0.265, ...
        'Ixx', 0.0232, ...
        'Iyy', 0.0232, ...
        'Izz', 0.0468, ...
        'posGain', containers.Map({'P_x','I_x','D_x','P_y','I_y','D_y','P_z','I_z','D_z'}, ...
                                  {3.17, 0.001, 0.81, 3.17, 0.001, 0.81, 4.0, 0.001, 0.0}), ...
        'attGain', containers.Map({'P_phi','I_phi','D_phi','P_theta','I_theta','D_theta','P_psi','I_psi','D_psi','P_zdot','I_zdot','D_zdot'}, ...
                                  {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0}) ...
    );

    %% -------------------------------------------------------------------------
    %% [1] Model 1: Hovering 기준 튜닝 모델 (Fixed PID, No Feedforward, No Z-Sched)
    %% -------------------------------------------------------------------------
    fprintf('\n--- [1/4] Model 1: Hovering 기준 튜닝 모델 실행 (v=%.1f m/s) ---\n', v_tar);
    cfg1 = get_cfg(false, false, 'Model1_Hovering_PID');
    drone1 = init_drone_and_env(cfg1);
    [log1, ~] = run_simulation_loop(drone1, cfg1);
    finalize_and_save_results(log1, cfg1);
    
    err1 = sqrt(sum((log1.state_hist(1:3, :) - log1.pos_des_hist(1:3, :)).^2, 1));
    z_err1 = abs(log1.state_hist(3, :) - log1.pos_des_hist(3, :));
    
    summary_metrics.(v_field).Model1.Name = 'Model 1: Hovering PID';
    summary_metrics.(v_field).Model1.RMSE = sqrt(mean(err1.^2));
    summary_metrics.(v_field).Model1.MAE  = mean(err1);
    summary_metrics.(v_field).Model1.MAX  = max(err1);
    summary_metrics.(v_field).Model1.Z_RMSE = sqrt(mean(z_err1.^2));
    fprintf('  -> Model 1: RMSE = %.4f m, MAE = %.4f m, MAX = %.4f m\n', ...
        summary_metrics.(v_field).Model1.RMSE, summary_metrics.(v_field).Model1.MAE, summary_metrics.(v_field).Model1.MAX);

    %% -------------------------------------------------------------------------
    %% [2] Model 2: cfg.Feedforward = true, cfg.Z_Gain_Scheduling = false
    %% -------------------------------------------------------------------------
    fprintf('\n--- [2/4] Model 2: Feedforward=true, Z_Gain_Sched=false 실행 (v=%.1f m/s) ---\n', v_tar);
    cfg2 = get_cfg(true, false, 'Model2_Feedforward_NoZ');
    drone2 = init_drone_and_env(cfg2);
    [log2, ~] = run_simulation_loop(drone2, cfg2);
    finalize_and_save_results(log2, cfg2);
    
    err2 = sqrt(sum((log2.state_hist(1:3, :) - log2.pos_des_hist(1:3, :)).^2, 1));
    z_err2 = abs(log2.state_hist(3, :) - log2.pos_des_hist(3, :));
    
    summary_metrics.(v_field).Model2.Name = 'Model 2: Feedforward Only';
    summary_metrics.(v_field).Model2.RMSE = sqrt(mean(err2.^2));
    summary_metrics.(v_field).Model2.MAE  = mean(err2);
    summary_metrics.(v_field).Model2.MAX  = max(err2);
    summary_metrics.(v_field).Model2.Z_RMSE = sqrt(mean(z_err2.^2));
    fprintf('  -> Model 2: RMSE = %.4f m, MAE = %.4f m, MAX = %.4f m\n', ...
        summary_metrics.(v_field).Model2.RMSE, summary_metrics.(v_field).Model2.MAE, summary_metrics.(v_field).Model2.MAX);

    %% -------------------------------------------------------------------------
    %% [3] Model 3: cfg.Feedforward = true, cfg.Z_Gain_Scheduling = true (Drone_LPV)
    %% -------------------------------------------------------------------------
    fprintf('\n--- [3/4] Model 3: Feedforward=true, Z_Gain_Sched=true 실행 (v=%.1f m/s) ---\n', v_tar);
    cfg3 = get_cfg(true, true, 'Model3_Drone_LPV');
    drone3 = init_drone_and_env(cfg3);
    [log3, ~] = run_simulation_loop(drone3, cfg3);
    finalize_and_save_results(log3, cfg3);
    
    err3 = sqrt(sum((log3.state_hist(1:3, :) - log3.pos_des_hist(1:3, :)).^2, 1));
    z_err3 = abs(log3.state_hist(3, :) - log3.pos_des_hist(3, :));
    
    summary_metrics.(v_field).Model3.Name = 'Model 3: Drone_LPV Full';
    summary_metrics.(v_field).Model3.RMSE = sqrt(mean(err3.^2));
    summary_metrics.(v_field).Model3.MAE  = mean(err3);
    summary_metrics.(v_field).Model3.MAX  = max(err3);
    summary_metrics.(v_field).Model3.Z_RMSE = sqrt(mean(z_err3.^2));
    fprintf('  -> Model 3: RMSE = %.4f m, MAE = %.4f m, MAX = %.4f m\n', ...
        summary_metrics.(v_field).Model3.RMSE, summary_metrics.(v_field).Model3.MAE, summary_metrics.(v_field).Model3.MAX);

    %% -------------------------------------------------------------------------
    %% [4] Model 4: PPO Residual Model (Drone_LPV + Residual PPO)
    %% -------------------------------------------------------------------------
    fprintf('\n--- [4/4] Model 4: PPO 모델 실행 (v=%.1f m/s) ---\n', v_tar);
    env4 = RL_DroneEnv_LPV();
    env4.randomize_speed = false;
    env4.randomize_path  = false;
    env4.cfg.target_speed = v_tar;
    env4.cfg.drone1_init_states(4) = v_tar;
    obs = reset(env4);
    
    cfg4 = env4.cfg;
    cfg4.play_animation = false;
    cfg4.auto_save      = true;
    cfg4.close_figures  = true;
    cfg4.target_speed   = v_tar;
    cfg4.save_dir       = fullfile(speed_dir, 'Model4_PPO');
    cfg4.file_prefix    = 'Model4_PPO';
    
    drone4 = env4.drone1;
    sim_tf = drone4.tf;
    num_steps = ceil(sim_tf / cfg4.dt);
    
    log4 = struct();
    log4.t_hist       = zeros(1, num_steps);
    log4.state_hist   = zeros(12, num_steps);
    log4.pos_des_hist = zeros(3, num_steps);
    log4.att_des_hist = zeros(4, num_steps);
    log4.u_hist       = zeros(4, num_steps);
    log4.PID_Pos_hist = zeros(3, num_steps);
    log4.kappa_hist   = zeros(1, num_steps);
    log4.action_hist  = zeros(4, num_steps);
    
    N_sub = 10;
    prev_yaw = env4.prev_yaw;
    current_act = [0; 0; 0; 0];
    
    for i = 1:num_steps
        drone4_state = drone4.GetState();
        current_time = drone4.t;
        
        [traj_pos, traj_vel, kappa_avg] = drone4.trajCtrl.get_position(current_time);
        
        % 10스텝(10Hz)마다 PPO 잔차 게인 오프셋 업데이트
        if mod(i-1, N_sub) == 0
            e_x = traj_pos(1) - drone4_state(1);
            e_y = traj_pos(2) - drone4_state(2);
            e_z = traj_pos(3) - drone4_state(3);
            v_x = drone4_state(4);
            v_y = drone4_state(5);
            v_z = drone4_state(6);
            phi = drone4_state(7);
            theta = drone4_state(8);
            
            obs = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
            action = getAction(agent, obs);
            if iscell(action), current_act = action{1}; else, current_act = action; end
            
            dP_xy = env4.range_P_xy * current_act(1);
            dD_xy = max(0, env4.range_D_xy * current_act(2));
            dP_z  = env4.range_P_z  * current_act(3);
            dD_z  = max(0, env4.range_D_z  * current_act(4));
            drone4.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);
        end
        
        speed_xy = norm(traj_vel(1:2));
        if speed_xy > 1e-3
            raw_yaw = atan2(traj_vel(2), traj_vel(1));
            yaw_diff = raw_yaw - prev_yaw;
            while yaw_diff > pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - prev_yaw; end
            while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - prev_yaw; end
            prev_yaw = raw_yaw;
        end
        
        current_cmd = [traj_pos; prev_yaw; cfg4.target_speed; kappa_avg];
        att_cmd = drone4.posCtrl.Update(current_cmd, drone4_state);
        u_motor = drone4.attCtrl.Update(att_cmd, drone4_state);
        drone4.UpdateState(u_motor);
        
        log4.t_hist(i)         = current_time;
        log4.state_hist(:,i)   = drone4_state;
        log4.pos_des_hist(:,i) = traj_pos;
        log4.att_des_hist(:,i) = att_cmd;
        log4.u_hist(:,i)       = u_motor;
        log4.PID_Pos_hist(:,i) = drone4.posCtrl.GetZ_PID();
        log4.kappa_hist(i)     = kappa_avg;
        log4.action_hist(:,i)  = current_act;
    end
    
    finalize_and_save_results(log4, cfg4);
    
    err4 = sqrt(sum((log4.state_hist(1:3, :) - log4.pos_des_hist(1:3, :)).^2, 1));
    z_err4 = abs(log4.state_hist(3, :) - log4.pos_des_hist(3, :));
    
    summary_metrics.(v_field).Model4.Name = 'Model 4: PPO Residual';
    summary_metrics.(v_field).Model4.RMSE = sqrt(mean(err4.^2));
    summary_metrics.(v_field).Model4.MAE  = mean(err4);
    summary_metrics.(v_field).Model4.MAX  = max(err4);
    summary_metrics.(v_field).Model4.Z_RMSE = sqrt(mean(z_err4.^2));
    fprintf('  -> Model 4: RMSE = %.4f m, MAE = %.4f m, MAX = %.4f m\n', ...
        summary_metrics.(v_field).Model4.RMSE, summary_metrics.(v_field).Model4.MAE, summary_metrics.(v_field).Model4.MAX);

    %% -------------------------------------------------------------------------
    %% 속도별 4개 모델 비교 종합 그래프 생성 및 저장
    %% -------------------------------------------------------------------------
    h_cmp = figure('Name', sprintf('Model Comparison at v=%.1f m/s', v_tar), ...
                   'Theme', 'light', 'Color', 'w', ...
                   'Position', [100, 100, 1200, 700], 'Visible', 'off');
    
    % Subplot 1: 2D Trajectory
    subplot(2, 2, [1, 3]);
    plot(log1.pos_des_hist(1,:), log1.pos_des_hist(2,:), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired Trajectory'); hold on;
    plot(log1.state_hist(1,:), log1.state_hist(2,:), 'r-', 'LineWidth', 1.3, 'DisplayName', '1. Hovering PID');
    plot(log2.state_hist(1,:), log2.state_hist(2,:), 'm-.', 'LineWidth', 1.3, 'DisplayName', '2. Feedforward Only');
    plot(log3.state_hist(1,:), log3.state_hist(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', '3. Applied Gain Scheduling');
    plot(log4.state_hist(1,:), log4.state_hist(2,:), 'Color', [0.0 0.6 0.2], 'LineWidth', 1.8, 'DisplayName', '4. PPO Residual');
    set(gca, 'YDir', 'Reverse'); axis equal; grid on;
    title(sprintf('Trajectory Comparison (v = %.1f m/s)', v_tar), 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]'); legend('Location', 'best', 'FontSize', 10);
    
    % Subplot 2: 3D Tracking Error vs Time
    subplot(2, 2, 2);
    plot(log1.t_hist, err1, 'r-', 'LineWidth', 1.2, 'DisplayName', '1. Hovering PID'); hold on;
    plot(log2.t_hist, err2, 'm-.', 'LineWidth', 1.2, 'DisplayName', '2. Feedforward Only');
    plot(log3.t_hist, err3, 'b--', 'LineWidth', 1.4, 'DisplayName', '3. Applied Gain Scheduling');
    plot(log4.t_hist, err4, 'Color', [0.0 0.6 0.2], 'LineWidth', 1.6, 'DisplayName', '4. PPO Residual');
    grid on; title('3D Tracking Error ||e_{pos}|| [m]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time [s]'); ylabel('Error [m]'); legend('Location', 'northeast', 'FontSize', 9);
    
    % Subplot 3: Z-axis Error vs Time
    subplot(2, 2, 4);
    plot(log1.t_hist, z_err1, 'r-', 'LineWidth', 1.2, 'DisplayName', '1. Hovering PID'); hold on;
    plot(log2.t_hist, z_err2, 'm-.', 'LineWidth', 1.2, 'DisplayName', '2. Feedforward Only');
    plot(log3.t_hist, z_err3, 'b--', 'LineWidth', 1.4, 'DisplayName', '3. Applied Gain Scheduling');
    plot(log4.t_hist, z_err4, 'Color', [0.0 0.6 0.2], 'LineWidth', 1.6, 'DisplayName', '4. PPO Residual');
    grid on; title('Z-axis Error |z - z_{des}| [m]', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time [s]'); ylabel('Z Error [m]'); legend('Location', 'northeast', 'FontSize', 9);
    
    cmp_fig_path = fullfile(speed_dir, sprintf('Combined_Comparison_v=%.1f.png', v_tar));
    exportgraphics(h_cmp, cmp_fig_path, 'Resolution', 300);
    close(h_cmp);
    
    % 속도별 개별 요약 MAT 파일 저장
    speed_summary_file = fullfile(speed_dir, sprintf('Summary_Metrics_v=%.1f.mat', v_tar));
    v_metrics = summary_metrics.(v_field);
    save(speed_summary_file, 'v_metrics', 'v_tar');
end

% 전체 요약 MAT 파일 저장
summary_all_file = fullfile(base_results_dir, 'all_12_simulations_summary.mat');
save(summary_all_file, 'summary_metrics', 'speeds');

fprintf('\n========================================================================\n');
fprintf('  12회 시뮬레이션 완료 및 모든 결과/그래프 저장 성공!\n');
fprintf('  결과 저장 디렉터리: %s\n', base_results_dir);
fprintf('========================================================================\n');
