% run_full_comparison_sim.m
% 4가지 제어 모델 대표 속도(1.5, 3.5, 4.0, 5.5, 7.5 m/s) 수치 시뮬레이션 비교
% 1. Hovering PID (고정 PID)
% 2. PID_Drone Baseline LPV (2D Lookup Table)
% 3. Drone_LPV (Feedforward + Z-scheduling + Direct Thrust U1)
% 4. Proposed: Drone_LPV + Residual PPO (초고속 D-damping 파인튜닝 모델)

clear; clc; close all;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
cd(script_dir);
addpath(genpath(script_dir));

speeds = [1.5, 3.5, 4.0, 5.5, 7.5];
sim_results = struct();

% 모델 로드
load('saved_drone_lpv_ppo_agent.mat', 'agent');

fprintf('========================================================================\n');
fprintf('  비행 제어 모델 전 속도 영역 수치 시뮬레이션 비교 시작\n');
fprintf('========================================================================\n');

for s_idx = 1:length(speeds)
    v_tar = speeds(s_idx);
    spd_tag = sprintf('v_%.1f', v_tar);
    spd_tag = strrep(spd_tag, '.', '_');
    
    fprintf('\n>>> 속도: %.1f m/s 평가 중... <<<\n', v_tar);
    
    %% [Model 3] Drone_LPV 단독 (피드포워드 + Z스케줄링)
    cfg3 = struct();
    cfg3.start_with_yaw = true;
    cfg3.dt = 0.01;
    cfg3.target_yaw = [];
    cfg3.target_speed = v_tar;
    cfg3.Feedforward = true;
    cfg3.Z_Gain_Scheduling = true;
    cfg3.play_animation = false;
    cfg3.auto_save = false;
    cfg3.drone1_init_states = [0.0, 0.0, -6.0, v_tar, 0, 0, 0, 0, 0, 0, 0, 0]';
    cfg3.m = 1.25; cfg3.armLength = 0.265; cfg3.Ixx = 0.0232; cfg3.Iyy = 0.0232; cfg3.Izz = 0.0468;
    cfg3.posGain = containers.Map({'P_x','I_x','D_x','P_y','I_y','D_y','P_z','I_z','D_z'}, {3.17, 0.001, 0.81, 3.17, 0.001, 0.81, 4.0, 0.001, 0.0});
    cfg3.attGain = containers.Map({'P_phi','I_phi','D_phi','P_theta','I_theta','D_theta','P_psi','I_psi','D_psi','P_zdot','I_zdot','D_zdot'}, {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0});
    
    drone3 = init_drone_and_env(cfg3);
    [log3, ~] = run_simulation_loop(drone3, cfg3);
    e_pos3 = log3.state_hist(1:3, :) - log3.pos_des_hist(1:3, :);
    err3_norm = sqrt(sum(e_pos3.^2, 1));
    
    sim_results.(spd_tag).Drone_LPV.RMSE = sqrt(mean(err3_norm.^2));
    sim_results.(spd_tag).Drone_LPV.MAE  = mean(err3_norm);
    sim_results.(spd_tag).Drone_LPV.Max  = max(err3_norm);
    sim_results.(spd_tag).Drone_LPV.Z_RMSE = sqrt(mean(abs(e_pos3(3,:)).^2));
    
    %% [Model 4] 제안 기법: Drone_LPV + Residual PPO (파인튜닝 에이전트)
    env4 = RL_DroneEnv_LPV();
    env4.randomize_speed = false;
    env4.randomize_path  = false;
    env4.cfg.target_speed = v_tar;
    env4.cfg.drone1_init_states(4) = v_tar;
    obs = reset(env4);
    
    cfg4 = env4.cfg;
    drone4 = env4.drone1;
    num_steps = ceil(drone4.tf / cfg4.dt);
    
    e_pos4 = zeros(3, num_steps);
    z_err4 = zeros(1, num_steps);
    N_sub = 10;
    prev_yaw = 0.0;
    
    for i = 1:num_steps
        st = drone4.GetState();
        t_now = drone4.t;
        [t_pos, t_vel, kappa_avg] = drone4.trajCtrl.get_position(t_now);
        e_pos4(:, i) = st(1:3) - t_pos;
        z_err4(i) = abs(st(3) - t_pos(3));
        
        if mod(i-1, N_sub) == 0
            obs_vec = [t_pos(1)-st(1); t_pos(2)-st(2); t_pos(3)-st(3); st(4); st(5); st(6); st(7); st(8)];
            act = getAction(agent, obs_vec);
            if iscell(act), act_val = act{1}; else, act_val = act; end
            
            dP_xy = env4.range_P_xy * act_val(1);
            dD_xy = max(0, env4.range_D_xy * act_val(2));
            dP_z  = env4.range_P_z  * act_val(3);
            dD_z  = max(0, env4.range_D_z  * act_val(4));
            drone4.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);
        end
        
        speed_xy = norm(t_vel(1:2));
        if speed_xy > 1e-3
            raw_yaw = atan2(t_vel(2), t_vel(1));
            yaw_diff = raw_yaw - prev_yaw;
            while yaw_diff > pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - prev_yaw; end
            while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - prev_yaw; end
            prev_yaw = raw_yaw;
        end
        
        cmd_vec = [t_pos; prev_yaw; v_tar; kappa_avg];
        att_c = drone4.posCtrl.Update(cmd_vec, st);
        u_mot = drone4.attCtrl.Update(att_c, st);
        drone4.UpdateState(u_mot);
    end
    
    err4_norm = sqrt(sum(e_pos4.^2, 1));
    sim_results.(spd_tag).Proposed_PPO.RMSE = sqrt(mean(err4_norm.^2));
    sim_results.(spd_tag).Proposed_PPO.MAE  = mean(err4_norm);
    sim_results.(spd_tag).Proposed_PPO.Max  = max(err4_norm);
    sim_results.(spd_tag).Proposed_PPO.Z_RMSE = sqrt(mean(z_err4.^2));
    
    fprintf('  [Drone_LPV 단독]   RMSE: %.4f m, Max: %.4f m\n', sim_results.(spd_tag).Drone_LPV.RMSE, sim_results.(spd_tag).Drone_LPV.Max);
    fprintf('  [LPV + Residual PPO] RMSE: %.4f m, Max: %.4f m\n', sim_results.(spd_tag).Proposed_PPO.RMSE, sim_results.(spd_tag).Proposed_PPO.Max);
end

save('full_comparison_sim_results.mat', 'sim_results', 'speeds');
fprintf('\n모든 시뮬레이션 완료 및 full_comparison_sim_results.mat 저장 완료!\n');
