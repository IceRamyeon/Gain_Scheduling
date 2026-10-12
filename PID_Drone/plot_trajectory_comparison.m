% plot_trajectory_comparison.m
% 대표 속도(1.5, 3.5, 5.5, 7.5 m/s)에서의 실제 2D/3D 비행 궤적 비교 시각화

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end
clearvars -except script_dir; clc; close all;

if ~isfile('saved_ppo_agent.mat')
    error('saved_ppo_agent.mat 파일이 없습니다.');
end
load('saved_ppo_agent.mat', 'agent');

eval_speeds = [1.5, 3.5, 5.5, 7.5];
modes = {'Hovering_PID', 'Baseline_LPV', 'Residual_PPO'};
mode_names = {'Hovering PID', 'Baseline LPV', 'Residual PPO'};
colors = {'r', 'b', [0.1, 0.75, 0.2]};

traj_data = struct();

fprintf('대표 속도 궤적 데이터 시뮬레이션 중...\n');

for s_idx = 1:length(eval_speeds)
    v_target = eval_speeds(s_idx);
    spd_tag = sprintf('v_%.1f', v_target);
    spd_tag = strrep(spd_tag, '.', '_');
    
    for m_idx = 1:3
        c_mode = modes{m_idx};
        
        env = RL_DroneEnv();
        env.cfg.target_speed = v_target;
        env.cfg.drone1_init_states(4) = v_target;
        obs = reset(env);
        
        cfg = env.cfg;
        cfg.play_animation = 0;
        cfg.auto_save = false;
        drone1 = env.drone1;
        num_steps = ceil(drone1.tf / cfg.dt);
        
        pos_hist = zeros(3, num_steps);
        traj_hist = zeros(3, num_steps);
        err_hist = zeros(3, num_steps);
        time_hist = zeros(1, num_steps);
        
        N_sub = 10;
        prev_yaw = 0.0;
        
        for i = 1:num_steps
            state = drone1.GetState();
            t_now = drone1.t;
            [t_pos, t_vel, t_acc] = drone1.trajCtrl.get_position(t_now);
            
            time_hist(i) = t_now;
            pos_hist(:, i) = state(1:3);
            traj_hist(:, i) = t_pos;
            err_hist(:, i) = t_pos - state(1:3);
            
            if mod(i-1, N_sub) == 0
                if m_idx == 3
                    e_x = t_pos(1) - state(1);
                    e_y = t_pos(2) - state(2);
                    e_z = t_pos(3) - state(3);
                    obs = [e_x; e_y; e_z; state(4); state(5); state(6); state(7); state(8)];
                    action = getAction(agent, obs);
                    if iscell(action), act_val = action{1}; else, act_val = action; end
                else
                    act_val = [0; 0; 0; 0];
                end
                
                if m_idx == 1
                    v_xy_c = 0; kappa_c = 0;
                else
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
                end
                
                base_P_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_x, kappa_c, v_xy_c, 'linear');
                base_D_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_x, kappa_c, v_xy_c, 'linear');
                base_P_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_y, kappa_c, v_xy_c, 'linear'));
                base_D_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_y, kappa_c, v_xy_c, 'linear'));
                
                P_x_new = max(base_P_x + env.range_P * act_val(1), 0.1);
                D_x_new = max(base_D_x + env.range_D * act_val(2), 0.01);
                P_y_new = max(base_P_y + env.range_P * act_val(3), 0.1);
                D_y_new = max(base_D_y + env.range_D * act_val(4), 0.01);
                
                drone1.posCtrl.setGains(P_x_new, D_x_new, P_y_new, D_y_new);
            end
            
            s_xy = norm(t_vel(1:2));
            if s_xy > 1e-3
                r_yaw = atan2(t_vel(2), t_vel(1));
                y_diff = r_yaw - prev_yaw;
                while y_diff > pi, r_yaw = r_yaw - 2*pi; y_diff = r_yaw - prev_yaw; end
                while y_diff < -pi, r_yaw = r_yaw + 2*pi; y_diff = r_yaw - prev_yaw; end
                prev_yaw = r_yaw;
            end
            
            att_cmd = drone1.posCtrl.Update([t_pos; prev_yaw], state);
            u_motor = drone1.attCtrl.Update(att_cmd, state);
            drone1.UpdateState(u_motor);
        end
        
        traj_data.(spd_tag).(c_mode).pos = pos_hist;
        traj_data.(spd_tag).(c_mode).traj = traj_hist;
        traj_data.(spd_tag).(c_mode).err = sqrt(sum(err_hist.^2, 1));
        traj_data.(spd_tag).(c_mode).time = time_hist;
    end
end

fprintf('궤적 비교 플롯 생성 중...\n');

%% [Fig 1] 2D XY 평면 궤적 비교 (4개 속도)
fig1 = figure('Position', [50, 50, 1300, 900], 'Visible', 'off');

for s_idx = 1:4
    v_val = eval_speeds(s_idx);
    spd_tag = sprintf('v_%.1f', v_val);
    spd_tag = strrep(spd_tag, '.', '_');
    
    subplot(2, 2, s_idx);
    hold on; grid on; box on;
    
    % 기준 목표 궤적
    ref_pos = traj_data.(spd_tag).Residual_PPO.traj;
    plot(ref_pos(1,:), ref_pos(2,:), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired Path');
    
    % 각 제어기 실제 비행 궤적
    for m_idx = 1:3
        c_mode = modes{m_idx};
        d_pos = traj_data.(spd_tag).(c_mode).pos;
        if m_idx == 3
            lw = 1.8;
        else
            lw = 1.2;
        end
        plot(d_pos(1,:), d_pos(2,:), 'Color', colors{m_idx}, 'LineWidth', lw, 'DisplayName', mode_names{m_idx});
    end
    
    % 웨이포인트 마커 표시
    [wp_x, wp_y, ~] = pathGen(false);
    plot(wp_x, wp_y, 'kp', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'DisplayName', 'Waypoints');
    
    xlabel('X Position (m)', 'FontSize', 11);
    ylabel('Y Position (m)', 'FontSize', 11);
    title(sprintf('2D Flight Trajectory Comparison at %.1f m/s', v_val), 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    axis equal;
end

sgtitle('2D Trajectory Tracking Comparison across Flight Speeds', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, 'trajectory_comparison_2d.png');

%% [Fig 2] 시간별 위치 오차 곡선 (4개 속도)
fig2 = figure('Position', [100, 100, 1300, 700], 'Visible', 'off');

for s_idx = 1:4
    v_val = eval_speeds(s_idx);
    spd_tag = sprintf('v_%.1f', v_val);
    spd_tag = strrep(spd_tag, '.', '_');
    
    subplot(2, 2, s_idx);
    hold on; grid on; box on;
    
    for m_idx = 1:3
        c_mode = modes{m_idx};
        t_vec = traj_data.(spd_tag).(c_mode).time;
        e_vec = traj_data.(spd_tag).(c_mode).err;
        if m_idx == 3
            lw = 1.8;
        else
            lw = 1.2;
        end
        plot(t_vec, e_vec, 'Color', colors{m_idx}, 'LineWidth', lw, 'DisplayName', mode_names{m_idx});
    end
    
    xlabel('Time (s)', 'FontSize', 10);
    ylabel('Position Error Norm (m)', 'FontSize', 10);
    title(sprintf('Position Error vs Time (Speed = %.1f m/s)', v_val), 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 9);
end

sgtitle('Tracking Error Norm Profiles over Flight Time', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, 'trajectory_error_profiles.png');

disp('궤적 비교 플롯 2건 저장 완료: trajectory_comparison_2d.png, trajectory_error_profiles.png');
