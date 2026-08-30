function [log_data, valid_len] = run_simulation_loop(drone1, cfg)
    % 시뮬레이션 스텝 수 계산
    num_steps = ceil(cfg.simTime / cfg.dt);
    
    % 로그 데이터 메모리 사전 할당
    log_data.t_hist       = zeros(1, num_steps);
    log_data.state_hist   = zeros(12, num_steps);
    log_data.pos_des_hist = zeros(3, num_steps);
    log_data.att_des_hist = zeros(4, num_steps);
    log_data.u_hist       = zeros(4, num_steps);
    
    valid_len = num_steps;

    %% 0. 초기 Yaw 설정
    % cfg.target_yaw가 비어있으면 출발 시점의 궤적 방향을 바라보게 설정
    if isempty(cfg.target_yaw)
        init_vel = drone1.trajCtrl.get_position(0);
        if norm(init_vel(1:2)) > 1e-3
            prev_yaw = atan2(init_vel(2), init_vel(1));
        else
            prev_yaw = 0.0; 
        end
    else
        prev_yaw = cfg.target_yaw;
    end

    %% Simulation Loop
    for i = 1:num_steps
        % 1. 현재 상태 및 시간 가져오기
        drone1_state = drone1.GetState();
        current_time = drone1.t;

        % 2. 목표 궤적 (위치와 속도만 받아옴 - 피드포워드 가속도 없음)
        [traj_pos, traj_vel] = drone1.trajCtrl.get_position(current_time);

        % 3. 동적 Yaw 계산 로직 (랩어라운드 방지 포함)
        speed_xy = norm(traj_vel(1:2));
        
        if speed_xy > 1e-3
            raw_yaw = atan2(traj_vel(2), traj_vel(1));
            yaw_diff = raw_yaw - prev_yaw;
            
            % 180도 넘어갈 때 갑자기 튀는 현상 방지
            while yaw_diff > pi
                raw_yaw = raw_yaw - 2*pi;
                yaw_diff = raw_yaw - prev_yaw;
            end
            while yaw_diff < -pi
                raw_yaw = raw_yaw + 2*pi;
                yaw_diff = raw_yaw - prev_yaw;
            end
            
            dynamic_target_yaw = raw_yaw;
            prev_yaw = dynamic_target_yaw; 
        else
            dynamic_target_yaw = prev_yaw;
        end

        % 예전처럼 위치와 Yaw 각도를 하나의 명령 벡터로 포장
        current_cmd = [traj_pos; dynamic_target_yaw];

        % 4. 제어기 업데이트 (예전 방식으로 원복)
        att_cmd = drone1.posCtrl.Update(current_cmd, drone1_state);
        u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);
        
        % 5. 상태 업데이트 (동역학)
        drone1.UpdateState(u_motor);
        
        % 6. 데이터 로깅
        log_data.t_hist(i)       = current_time;
        log_data.state_hist(:,i) = drone1_state;
        log_data.pos_des_hist(:,i) = traj_pos;
        log_data.att_des_hist(:,i) = att_cmd;
        log_data.u_hist(:,i)       = u_motor;
        
        % (필요하다면 기존에 있던 충돌 체크 로직은 이 아래에 유지하면 돼)
    end
end