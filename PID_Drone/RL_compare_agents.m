% RL_compare_agents.m
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end
clearvars -except script_dir; clc; close all;

disp('========================================');
disp('   제어 방식별 성능 비교 (3가지 모드)   ');
disp('========================================');

if ~isfile('saved_ppo_agent.mat')
    error('saved_ppo_agent.mat 파일이 없습니다.');
end
load('saved_ppo_agent.mat', 'agent');

modes = {'Hovering_PID', 'Baseline_LPV', 'Residual_PPO'};
display_names = {
    '1. Hovering PID (정지 비행 기준 고정 게인)', 
    '2. Baseline LPV (동작점 기반 2D 선형 보간)', 
    '3. Residual PPO (LPV 보간 + PPO 잔차 제어)'
};
results = struct();

for m_idx = 1:3
    current_mode = modes{m_idx};
    disp(['--- Running Mode: ', display_names{m_idx}, ' ---']);
    
    env = RL_DroneEnv();
    obs = reset(env);
    
    cfg = env.cfg;
    cfg.play_animation = 0;
    cfg.auto_save = false;
    drone1 = env.drone1;
    num_steps = ceil(drone1.tf / cfg.dt);
    
    err_hist = zeros(3, num_steps);
    
    N_sub = 10;
    prev_yaw = 0.0;
    
    for i = 1:num_steps
        drone1_state = drone1.GetState();
        current_time = drone1.t;
        
        [traj_pos, traj_vel, traj_acc] = drone1.trajCtrl.get_position(current_time);
        err_hist(:, i) = traj_pos - drone1_state(1:3);
        
        if mod(i-1, N_sub) == 0
            % (1) PPO Action 결정
            if m_idx == 1 || m_idx == 2
                % PPO 잔차 오프셋 미적용
                act_val = [0; 0; 0; 0];
            else
                % PPO 잔차 오프셋 적용
                e_x = traj_pos(1) - drone1_state(1);
                e_y = traj_pos(2) - drone1_state(2);
                e_z = traj_pos(3) - drone1_state(3);
                obs = [e_x; e_y; e_z; drone1_state(4); drone1_state(5); drone1_state(6); drone1_state(7); drone1_state(8)];
                action = getAction(agent, obs);
                if iscell(action)
                    act_val = action{1};
                else
                    act_val = action;
                end
            end
            
            % (2) 동작점(v, kappa) 결정
            if m_idx == 1
                % Hovering PID: 호버링 상태(속도 0, 곡률 0)로 고정
                v_xy_c = 0;
                kappa_c = 0;
            else
                % LPV: 실시간 속도 및 곡률 반영
                v_xy = norm(drone1_state(4:5));
                v_norm = norm(traj_vel);
                if v_norm > 1e-3
                    vel_vec3 = reshape(traj_vel(1:3), 3, 1);
                    acc_vec3 = reshape(traj_acc(1:3), 3, 1);
                    kappa_raw = norm(cross(vel_vec3, acc_vec3)) / (v_norm^3);
                else
                    kappa_raw = 0;
                end
                
                kappa_c = max(min(kappa_raw, max(env.pid_table.kappa_vec)), min(env.pid_table.kappa_vec));
                v_xy_c = max(min(v_xy, max(env.pid_table.v_vec)), min(env.pid_table.v_vec));
            end
            
            % (3) 2D 보간으로 명목 게인 추출
            base_P_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_x, kappa_c, v_xy_c, 'linear');
            base_D_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_x, kappa_c, v_xy_c, 'linear');
            base_P_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_y, kappa_c, v_xy_c, 'linear'));
            base_D_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_y, kappa_c, v_xy_c, 'linear'));
            
            % (4) 최종 게인 합성 (명목 게인 + PPO Action)
            P_x_new = max(base_P_x + env.range_P * act_val(1), 0.1);
            D_x_new = max(base_D_x + env.range_D * act_val(2), 0.01);
            P_y_new = max(base_P_y + env.range_P * act_val(3), 0.1);
            D_y_new = max(base_D_y + env.range_D * act_val(4), 0.01);
            
            drone1.posCtrl.setGains(P_x_new, D_x_new, P_y_new, D_y_new);
        end
        
        % 방향(Yaw) 제어 로직 (진행 방향 응시)
        speed_xy = norm(traj_vel(1:2));
        if speed_xy > 1e-3
            raw_yaw = atan2(traj_vel(2), traj_vel(1));
            yaw_diff = raw_yaw - prev_yaw;
            while yaw_diff > pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - prev_yaw; end
            while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - prev_yaw; end
            dynamic_target_yaw = raw_yaw;
            prev_yaw = dynamic_target_yaw;
        else
            dynamic_target_yaw = prev_yaw;
        end
        
        att_cmd = drone1.posCtrl.Update([traj_pos; dynamic_target_yaw], drone1_state);
        u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);
        drone1.UpdateState(u_motor);
    end
    
    % 오차 계산
    err_norms = sqrt(sum(err_hist.^2, 1));
    results.(current_mode).RMSE = sqrt(mean(err_norms.^2));
    results.(current_mode).MAE = mean(err_norms);
    results.(current_mode).Max = max(err_norms);
end

disp('========================================');
disp('            비교 결과 요약            ');
disp('========================================');

for m_idx = 1:3
    current_mode = modes{m_idx};
    disp(display_names{m_idx});
    disp(['   - RMSE: ', num2str(results.(current_mode).RMSE, '%.4f'), ' m']);
    disp(['   - MAE:  ', num2str(results.(current_mode).MAE, '%.4f'), ' m']);
    disp(['   - Max:  ', num2str(results.(current_mode).Max, '%.4f'), ' m']);
    disp(' ');
end
disp('========================================');
