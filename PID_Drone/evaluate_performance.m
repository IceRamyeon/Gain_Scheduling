clear; clc;
% 1. Run Baseline
disp('Running Baseline...');
cfg = struct();
cfg.start_with_yaw = true;
cfg.dt = 0.01;
cfg.target_yaw = [];
cfg.target_speed = 4;
cfg.Calculate_PID_Table = true;
cfg.play_animation = 0; % No animation
cfg.auto_save = false;
cfg.drone1_init_states = [ 0.0, 0.0, -6.0, cfg.target_speed, 0, 0, 0, 0, 0, 0, 0, 0]';
cfg.posGain = containers.Map({'P_x','I_x','D_x', 'P_y','I_y','D_y', 'P_z','I_z','D_z'}, {3.17, 0.001, 0.81, 3.17, 0.001, 0.81, 4.0, 0.001, 0.0});
cfg.attGain = containers.Map({'P_phi','I_phi','D_phi', 'P_theta','I_theta','D_theta', 'P_psi','I_psi','D_psi', 'P_zdot','I_zdot','D_zdot'}, {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0});
drone_base = init_drone_and_env(cfg);
[log_base, ~] = run_simulation_loop(drone_base, cfg);

err_base_x = log_base.state_hist(1,:) - log_base.pos_des_hist(1,:);
err_base_y = log_base.state_hist(2,:) - log_base.pos_des_hist(2,:);
err_base_z = log_base.state_hist(3,:) - log_base.pos_des_hist(3,:);
rmse_base = sqrt(mean(err_base_x.^2 + err_base_y.^2 + err_base_z.^2));
energy_base = sum(sum(log_base.u_hist.^2)) * cfg.dt;

% 2. Run RL
disp('Running RL Agent...');
load('saved_ppo_agent.mat', 'agent');
env = RL_DroneEnv();
obs = reset(env);
env.cfg.play_animation = 0;
drone_rl = env.drone1;
num_steps = ceil(drone_rl.tf / env.cfg.dt);
log_rl.state_hist = zeros(12, num_steps);
log_rl.pos_des_hist = zeros(3, num_steps);
log_rl.u_hist = zeros(4, num_steps);

N_sub = 10;
prev_yaw = 0.0;
for i = 1:num_steps
    state = drone_rl.GetState();
    [traj_pos, traj_vel] = drone_rl.trajCtrl.get_position(drone_rl.t);
    if mod(i-1, N_sub) == 0
        obs = [traj_pos(1)-state(1); traj_pos(2)-state(2); traj_pos(3)-state(3); state(4); state(5); state(6); state(7); state(8)];
        act = getAction(agent, obs); act = act{1};
        v_xy = norm(state(4:5));
        kappa_c = 0; v_xy_c = max(min(v_xy, max(env.pid_table.v_vec)), min(env.pid_table.v_vec));
        P_x = max(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_x, kappa_c, v_xy_c, 'linear') + env.range_P*act(1), 0.1);
        D_x = max(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_x, kappa_c, v_xy_c, 'linear') + env.range_D*act(2), 0.01);
        P_y = max(abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_y, kappa_c, v_xy_c, 'linear')) + env.range_P*act(3), 0.1);
        D_y = max(abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_y, kappa_c, v_xy_c, 'linear')) + env.range_D*act(4), 0.01);
        drone_rl.posCtrl.setGains(P_x, D_x, P_y, D_y);
    end
    speed_xy = norm(traj_vel(1:2));
    if speed_xy > 1e-3
        raw_yaw = atan2(traj_vel(2), traj_vel(1));
        yaw_diff = raw_yaw - prev_yaw;
        while yaw_diff > pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - prev_yaw; end
        while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - prev_yaw; end
        prev_yaw = raw_yaw;
    end
    att_cmd = drone_rl.posCtrl.Update([traj_pos; prev_yaw], state);
    u_motor = drone_rl.attCtrl.Update(att_cmd, state);
    drone_rl.UpdateState(u_motor);
    log_rl.state_hist(:,i) = state;
    log_rl.pos_des_hist(:,i) = traj_pos;
    log_rl.u_hist(:,i) = u_motor;
end

err_rl_x = log_rl.state_hist(1,:) - log_rl.pos_des_hist(1,:);
err_rl_y = log_rl.state_hist(2,:) - log_rl.pos_des_hist(2,:);
err_rl_z = log_rl.state_hist(3,:) - log_rl.pos_des_hist(3,:);
rmse_rl = sqrt(mean(err_rl_x.^2 + err_rl_y.^2 + err_rl_z.^2));
energy_rl = sum(sum(log_rl.u_hist.^2)) * env.cfg.dt;

fprintf('--- Performance Comparison ---\n');
fprintf('RMSE (Position): Baseline = %.4f m, RL = %.4f m\n', rmse_base, rmse_rl);
fprintf('Energy (Control): Baseline = %.4f, RL = %.4f\n', energy_base, energy_rl);
fprintf('Improvement: RMSE %.2f%%, Energy %.2f%%\n', (rmse_base-rmse_rl)/rmse_base*100, (energy_base-energy_rl)/energy_base*100);
