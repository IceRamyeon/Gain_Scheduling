function [log_data, valid_len] = run_simulation_loop(drone1, obstacles, cfg)
    % RUN_SIMULATION_LOOP 그래픽 없이 시뮬레이션 연산만 수행
    
    num_steps = round(cfg.simTime / cfg.dt);

    % 메모리 사전 할당 (속도 향상)
    save_time    = zeros(1, num_steps);
    save_state   = zeros(12, num_steps);
    save_pos_des = zeros(3, num_steps);
    save_att_des = zeros(3, num_steps);
    save_u       = zeros(4, num_steps);

    valid_len = num_steps;

    for i = 1:num_steps
        % 1. Get State & Time
        drone1_state = drone1.GetState();
        current_time = drone1.t;

        % 2. Desired Trajectory
        traj_pos = drone1.trajCtrl.get_position(current_time);
        current_cmd = [traj_pos; cfg.target_yaw];

        % 3. Control (Outer & Inner)
        att_cmd = drone1.posCtrl.Update(current_cmd, drone1_state);
        u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);

        % 4. Update Drone Dynamics
        drone1.UpdateState(u_motor);

        % 5. Data Logging
        save_time(i)       = current_time;
        save_state(:, i)   = drone1_state;
        save_pos_des(:, i) = traj_pos;
        save_att_des(:, i) = att_cmd(1:3);
        save_u(:, i)       = u_motor;

        % 6. Collision Check (장애물 및 지면)
        x_curr = drone1_state(1);
        y_curr = drone1_state(2);
        z_curr = drone1_state(3);
        
        is_hit = false;
        
        % 지면 충돌 체크 (z는 아래가 +이므로 z > 0이면 땅에 부딪힌 거)
        if z_curr > 0
            is_hit = true;
            disp(['Ground Crash at t = ', num2str(current_time)]);
        end

        % 장애물 충돌 체크
        if ~is_hit
            for k = 1:length(obstacles)
                obs = obstacles{k};
                if isempty(obs), continue; end
                
                if (x_curr >= obs(1,1) && x_curr <= obs(1,2)) && ...
                   (y_curr >= obs(2,1) && y_curr <= obs(2,2)) && ...
                   (z_curr >= obs(3,1) && z_curr <= obs(3,2))
                    is_hit = true;
                    disp(['Wall Crash at t = ', num2str(current_time)]);
                    break;
                end
            end
        end

        % 충돌 시 루프 종료
        if is_hit
            valid_len = i;
            break;
        end
    end

    % 7. 유효한 데이터만 잘라서 구조체로 반환
    log_data.t_hist       = save_time(1:valid_len);
    log_data.state_hist   = save_state(:, 1:valid_len);
    log_data.pos_des_hist = save_pos_des(:, 1:valid_len);
    log_data.att_des_hist = save_att_des(:, 1:valid_len);
    log_data.u_hist       = save_u(:, 1:valid_len);
end