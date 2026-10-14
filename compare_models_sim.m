% compare_models_sim.m
% Drone_LPV (Feedforward + Z-scheduling LPV) vs PID_Drone Baseline LPV vs Residual PPO
clear; clc; close all;

speeds = [1.5, 3.5, 4.0, 5.5, 7.5];

drone_lpv_path = '/Users/sehyunpark/Downloads/Gain_Scheduling-main/Gain_Scheduling/Drone_LPV';
pid_drone_path = '/Users/sehyunpark/Downloads/Gain_Scheduling-main/Gain_Scheduling/PID_Drone';

results = struct();

fprintf('========================================================================\n');
fprintf('  Drone_LPV vs PID_Drone (Baseline LPV vs Residual PPO) Simulation Comparison\n');
fprintf('========================================================================\n');

for s_idx = 1:length(speeds)
    v_target = speeds(s_idx);
    spd_tag = sprintf('v_%.1f', v_target);
    spd_tag = strrep(spd_tag, '.', '_');
    
    fprintf('Testing Speed: %.1f m/s ...\n', v_target);
    
    %% 1. Simulate Drone_LPV (Feedforward + Z-scheduling LPV)
    cd(drone_lpv_path);
    addpath(drone_lpv_path);
    rmpath(pid_drone_path);
    
    cfg_lpv = struct();
    cfg_lpv.start_with_yaw = true;
    cfg_lpv.dt = 0.01;
    cfg_lpv.target_yaw = [];
    cfg_lpv.target_speed = v_target;
    cfg_lpv.Feedforward = true;
    cfg_lpv.Z_Gain_Scheduling = true;
    cfg_lpv.play_animation = false;
    cfg_lpv.auto_save = false;
    cfg_lpv.drone1_init_states = [0.0, 0.0, -6.0, cfg_lpv.target_speed, 0, 0, 0, 0, 0, 0, 0, 0]';
    cfg_lpv.m = 1.25; cfg_lpv.armLength = 0.265; cfg_lpv.Ixx = 0.0232; cfg_lpv.Iyy = 0.0232; cfg_lpv.Izz = 0.0468;
    cfg_lpv.posGain = containers.Map({'P_x','I_x','D_x','P_y','I_y','D_y','P_z','I_z','D_z'}, {3.17, 0.001, 0.81, 3.17, 0.001, 0.81, 4.0, 0.001, 0.0});
    cfg_lpv.attGain = containers.Map({'P_phi','I_phi','D_phi','P_theta','I_theta','D_theta','P_psi','I_psi','D_psi','P_zdot','I_zdot','D_zdot'}, {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0});
    
    drone_lpv = init_drone_and_env(cfg_lpv);
    [log_lpv, ~] = run_simulation_loop(drone_lpv, cfg_lpv);
    
    err_lpv = log_lpv.state_hist(1:3, :) - log_lpv.pos_des_hist(1:3, :);
    err_lpv_norm = sqrt(sum(err_lpv.^2, 1));
    z_err_lpv = abs(err_lpv(3, :));
    
    results.(spd_tag).Drone_LPV.RMSE = sqrt(mean(err_lpv_norm.^2));
    results.(spd_tag).Drone_LPV.MAE  = mean(err_lpv_norm);
    results.(spd_tag).Drone_LPV.Max  = max(err_lpv_norm);
    results.(spd_tag).Drone_LPV.Z_RMSE = sqrt(mean(z_err_lpv.^2));
    
    %% 2. Simulate PID_Drone: Baseline LPV & Residual PPO
    cd(pid_drone_path);
    addpath(pid_drone_path);
    rmpath(drone_lpv_path);
    
    load('saved_ppo_agent.mat', 'agent');
    
    for m_idx = 1:2
        % m_idx 1: Baseline LPV (lookup table)
        % m_idx 2: Residual PPO
        env = RL_DroneEnv();
        env.cfg.target_speed = v_target;
        env.cfg.drone1_init_states(4) = v_target;
        obs = reset(env);
        
        cfg = env.cfg;
        cfg.play_animation = 0;
        cfg.auto_save = false;
        drone_pid = env.drone1;
        num_steps = ceil(drone_pid.tf / cfg.dt);
        
        err_hist = zeros(3, num_steps);
        N_sub = 10;
        prev_yaw = 0.0;
        
        for i = 1:num_steps
            state = drone_pid.GetState();
            t_now = drone_pid.t;
            [t_pos, t_vel, t_acc] = drone_pid.trajCtrl.get_position(t_now);
            err_hist(:, i) = t_pos - state(1:3);
            
            if mod(i-1, N_sub) == 0
                if m_idx == 2
                    e_x = t_pos(1) - state(1);
                    e_y = t_pos(2) - state(2);
                    e_z = t_pos(3) - state(3);
                    obs = [e_x; e_y; e_z; state(4); state(5); state(6); state(7); state(8)];
                    action = getAction(agent, obs);
                    if iscell(action), act_val = action{1}; else, act_val = action; end
                else
                    act_val = [0; 0; 0; 0];
                end
                
                v_xy = norm(state(4:5));
                v_norm = norm(t_vel);
                if v_norm > 1e-3
                    vel3 = reshape(t_vel(1:3), 3, 1);
                    acc3 = reshape(t_acc(1:3), 3, 1);
                    kappa_raw = norm(cross(vel3, acc3)) / (v_norm^3);
                else
                    kappa_raw = 0;
                end
                kappa_c = max(min(kappa_raw, max(env.pid_table.kappa_vec)), min(env.pid_table.kappa_vec));
                v_xy_c = max(min(v_xy, max(env.pid_table.v_vec)), min(env.pid_table.v_vec));
                
                base_P_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_x, kappa_c, v_xy_c, 'linear');
                base_D_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_x, kappa_c, v_xy_c, 'linear');
                base_P_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_y, kappa_c, v_xy_c, 'linear'));
                base_D_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_y, kappa_c, v_xy_c, 'linear'));
                
                P_x_new = max(base_P_x + env.range_P * act_val(1), 0.1);
                D_x_new = max(base_D_x + env.range_D * act_val(2), 0.01);
                P_y_new = max(base_P_y + env.range_P * act_val(3), 0.1);
                D_y_new = max(base_D_y + env.range_D * act_val(4), 0.01);
                
                drone_pid.posCtrl.setGains(P_x_new, D_x_new, P_y_new, D_y_new);
            end
            
            s_xy = norm(t_vel(1:2));
            if s_xy > 1e-3
                r_yaw = atan2(t_vel(2), t_vel(1));
                y_diff = r_yaw - prev_yaw;
                while y_diff > pi, r_yaw = r_yaw - 2*pi; y_diff = r_yaw - prev_yaw; end
                while y_diff < -pi, r_yaw = r_yaw + 2*pi; y_diff = r_yaw - prev_yaw; end
                prev_yaw = r_yaw;
            end
            
            att_cmd = drone_pid.posCtrl.Update([t_pos; prev_yaw], state);
            u_motor = drone_pid.attCtrl.Update(att_cmd, state);
            drone_pid.UpdateState(u_motor);
        end
        
        err_norm = sqrt(sum(err_hist.^2, 1));
        z_err = abs(err_hist(3, :));
        
        if m_idx == 1
            results.(spd_tag).Baseline_LPV.RMSE = sqrt(mean(err_norm.^2));
            results.(spd_tag).Baseline_LPV.MAE  = mean(err_norm);
            results.(spd_tag).Baseline_LPV.Max  = max(err_norm);
            results.(spd_tag).Baseline_LPV.Z_RMSE = sqrt(mean(z_err.^2));
        else
            results.(spd_tag).Residual_PPO.RMSE = sqrt(mean(err_norm.^2));
            results.(spd_tag).Residual_PPO.MAE  = mean(err_norm);
            results.(spd_tag).Residual_PPO.Max  = max(err_norm);
            results.(spd_tag).Residual_PPO.Z_RMSE = sqrt(mean(z_err.^2));
        end
    end
end

save('/Users/sehyunpark/Downloads/Gain_Scheduling-main/Gain_Scheduling/three_way_sim_comparison.mat', 'results', 'speeds');
fprintf('\nSimulation Finished and saved to three_way_sim_comparison.mat!\n');
