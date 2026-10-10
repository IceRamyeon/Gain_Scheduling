% 0. 스크립트 위치 기준으로 작업 경로 자동 이동 및 등록
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end

clearvars -except script_dir; clc; close all;

disp('========================================');
disp('   RL Agent Performance Verification    ');
disp('========================================');

% 1. 학습된 에이전트 및 환경 로드
if ~isfile('saved_ppo_agent.mat')
    error('saved_ppo_agent.mat 파일이 없습니다. RL_train_rl_agent.m을 먼저 실행하거나 학습 완료 여부를 확인하세요.');
end

load('saved_ppo_agent.mat', 'agent');
env = RL_DroneEnv();
obs = reset(env);

% 2. 시뮬레이션 설정 및 메모리 할당
cfg = env.cfg;
cfg.play_animation = 1;          % 애니메이션 재생
cfg.auto_save = false;
drone1 = env.drone1;
sim_tf = drone1.tf;
num_steps = ceil(sim_tf / cfg.dt);

% 로그 데이터 메모리 사전 할당
log_data.t_hist       = zeros(1, num_steps);
log_data.state_hist   = zeros(12, num_steps);
log_data.pos_des_hist = zeros(3, num_steps);
log_data.att_des_hist = zeros(4, num_steps);
log_data.u_hist       = zeros(4, num_steps);

% 강화학습 제어 주기 설정 (환경과 동일하게 10스텝마다 게인 변경)
N_sub = 10; 
prev_yaw = 0.0;

disp('드론 비행 시뮬레이션을 시작합니다...');

%% 3. 시뮬레이션 루프
for i = 1:num_steps
    drone1_state = drone1.GetState();
    current_time = drone1.t;
    
    % 목표 위치 및 속도 추출
    [traj_pos, traj_vel, traj_acc] = drone1.trajCtrl.get_position(current_time);
    
    % --- [A] RL 에이전트의 실시간 게인 스케줄링 (매 N_sub 스텝마다) ---
    if mod(i-1, N_sub) == 0
        % 현재 상태 계산 (Observation)
        e_x = traj_pos(1) - drone1_state(1);
        e_y = traj_pos(2) - drone1_state(2);
        e_z = traj_pos(3) - drone1_state(3);
        v_x = drone1_state(4);
        v_y = drone1_state(5);
        v_z = drone1_state(6);
        phi = drone1_state(7);
        theta = drone1_state(8);
        obs = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
        
        % 에이전트의 Action (잔차 오프셋) 예측
        action = getAction(agent, obs);
        if iscell(action)
            act_val = action{1};
        else
            act_val = action;
        end
        
        % 기본(Base) 게인 보간 (속도와 곡률에 따른 LPV 베이스라인)
        v_xy = norm(drone1_state(4:5));
        
        % 곡률 계산
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
        
        base_P_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_x, kappa_c, v_xy_c, 'linear');
        base_D_x = interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_x, kappa_c, v_xy_c, 'linear');
        base_P_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kp_y, kappa_c, v_xy_c, 'linear'));
        base_D_y = abs(interp2(env.pid_table.kappa_vec, env.pid_table.v_vec, env.pid_table.Kd_y, kappa_c, v_xy_c, 'linear'));
        
        % 최종 게인 합성 (Base + RL Residual) 및 최솟값 제한
        P_x_new = max(base_P_x + env.range_P * act_val(1), 0.1);
        D_x_new = max(base_D_x + env.range_D * act_val(2), 0.01);
        P_y_new = max(base_P_y + env.range_P * act_val(3), 0.1);
        D_y_new = max(base_D_y + env.range_D * act_val(4), 0.01);
        
        % 드론 제어기에 게인 덮어쓰기
        drone1.posCtrl.setGains(P_x_new, D_x_new, P_y_new, D_y_new);
    end

    % --- [B] 제어기 및 물리 동역학 업데이트 ---
    % 동적 Yaw 적용 (직진 방향 바라보기)
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
    current_cmd = [traj_pos; dynamic_target_yaw];
    
    % PID 제어기 연산
    att_cmd = drone1.posCtrl.Update(current_cmd, drone1_state);
    u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);
    
    % 동역학 전개
    drone1.UpdateState(u_motor);
    
    % --- [C] 결과 로깅 ---
    log_data.t_hist(i)       = current_time;
    log_data.state_hist(:,i) = drone1_state;
    log_data.pos_des_hist(:,i) = traj_pos;
    log_data.att_des_hist(:,i) = att_cmd;
    log_data.u_hist(:,i)       = u_motor;
end

disp('시뮬레이션 종료! 애니메이션을 재생합니다.');

%% 4. 결과 출력 및 3D 애니메이션 재생
try
    play_simulation_graphics(drone1, log_data, cfg);
    finalize_and_save_results(log_data, cfg);
catch ME
    warning('애니메이션 재생 중 오류가 발생했습니다: %s', ME.message);
end

disp('========================================');
disp('성능 확인이 성공적으로 마무리되었습니다.');
