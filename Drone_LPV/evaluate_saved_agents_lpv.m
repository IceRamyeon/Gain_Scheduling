% evaluate_saved_agents_lpv.m
% savedAgents_lpv 폴더 내의 각 체크포인트(Agent500, 1000, 1500 ... 3000)들을 순회하며
% finalize_and_save_results 형식으로 시뮬레이션 데이터를 로깅하고 개별 서브디렉토리에 결과 그래프/데이터 저장

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end

clearvars -except script_dir; clc; close all;

disp('========================================================================');
disp('   savedAgents_lpv 체크포인트별 시뮬레이션 검증 및 결과 자동 저장     ');
disp('========================================================================');

chk_dir = fullfile(script_dir, 'savedAgents_lpv');
if ~isfolder(chk_dir)
    error('savedAgents_lpv 폴더가 없습니다.');
end

agent_files = dir(fullfile(chk_dir, 'Agent*.mat'));
if isempty(agent_files)
    disp('savedAgents_lpv 폴더에 아직 저장된 체크포인트 파일이 없습니다.');
    disp('현재 학습이 진행 중이므로 첫 번째 500 에피소드 저장 후 다시 실행할 수 있습니다.');
    return;
end

% 에피소드 번호 기준 정렬
[~, sort_idx] = sort([agent_files.datenum]);
agent_files = agent_files(sort_idx);

for f_idx = 1:length(agent_files)
    curr_file = agent_files(f_idx).name;
    [~, agent_name, ~] = fileparts(curr_file);
    fprintf('\n>>> [%d/%d] 평가 중인 체크포인트: %s <<<\n', f_idx, length(agent_files), curr_file);
    
    loaded = load(fullfile(chk_dir, curr_file), 'saved_agent');
    if isfield(loaded, 'saved_agent')
        agent = loaded.saved_agent;
    else
        loaded2 = load(fullfile(chk_dir, curr_file), 'agent');
        agent = loaded2.agent;
    end
    
    env = RL_DroneEnv_LPV();
    env.randomize_speed = false;
    env.randomize_path  = false;
    obs = reset(env);
    
    cfg = env.cfg;
    cfg.play_animation = false;
    cfg.auto_save      = true;
    cfg.target_speed   = 4.0;
    cfg.save_dir       = fullfile(chk_dir, ['Result_', agent_name]);
    
    drone1 = env.drone1;
    num_steps = ceil(drone1.tf / cfg.dt);
    
    % finalize_and_save_results 변수 로깅 규격
    log_data = struct();
    log_data.t_hist       = zeros(1, num_steps);
    log_data.state_hist   = zeros(12, num_steps);
    log_data.pos_des_hist = zeros(3, num_steps);
    log_data.att_des_hist = zeros(4, num_steps);
    log_data.u_hist       = zeros(4, num_steps);
    log_data.PID_Pos_hist = zeros(3, num_steps);
    log_data.kappa_hist   = zeros(1, num_steps);
    log_data.action_hist  = zeros(4, num_steps);
    
    N_sub = 10;
    prev_yaw = 0.0;
    current_act = [0; 0; 0; 0];
    
    for i = 1:num_steps
        drone1_state = drone1.GetState();
        current_time = drone1.t;
        [traj_pos, traj_vel, kappa_avg] = drone1.trajCtrl.get_position(current_time);
        
        if mod(i-1, N_sub) == 0
            e_x = traj_pos(1) - drone1_state(1);
            e_y = traj_pos(2) - drone1_state(2);
            e_z = traj_pos(3) - drone1_state(3);
            v_x = drone1_state(4);
            v_y = drone1_state(5);
            v_z = drone1_state(6);
            phi = drone1_state(7);
            theta = drone1_state(8);
            
            obs = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
            action = getAction(agent, obs);
            if iscell(action), current_act = action{1}; else, current_act = action; end
            
            dP_xy = env.range_P_xy * current_act(1);
            dD_xy = env.range_D_xy * current_act(2);
            dP_z  = env.range_P_z  * current_act(3);
            dD_z  = env.range_D_z  * current_act(4);
            drone1.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);
        end
        
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
        
        current_cmd = [traj_pos; dynamic_target_yaw; cfg.target_speed; kappa_avg];
        att_cmd = drone1.posCtrl.Update(current_cmd, drone1_state);
        u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);
        drone1.UpdateState(u_motor);
        
        log_data.t_hist(i)         = current_time;
        log_data.state_hist(:,i)   = drone1_state;
        log_data.pos_des_hist(:,i) = traj_pos;
        log_data.att_des_hist(:,i) = att_cmd;
        log_data.u_hist(:,i)       = u_motor;
        log_data.PID_Pos_hist(:,i) = drone1.posCtrl.GetZ_PID();
        log_data.kappa_hist(i)     = kappa_avg;
        log_data.action_hist(:,i)  = current_act;
    end
    
    % finalize_and_save_results 호출하여 그래프 및 수치 자동 저장
    finalize_and_save_results(log_data, cfg);
    close all;
end

disp('========================================================================');
disp('   모든 체크포인트의 평가 및 결과 저장 완료!                          ');
disp('========================================================================');
