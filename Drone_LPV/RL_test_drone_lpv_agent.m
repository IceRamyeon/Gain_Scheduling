% RL_test_drone_lpv_agent.m
% Drone_LPV 피드포워드 + Z스케줄링 + 직접 추력 U1 베이스라인 + PPO 잔차 제어기 성능 검증 스크립트
% Drone_LPV 폴더의 finalize_and_save_results.m 형식에 맞춰 변수들을 완벽하게 로깅하고 시각화/저장

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end

clearvars -except script_dir eval_speed; clc; close all;

disp('========================================================================');
disp('   Drone_LPV + PPO Residual Control Performance Verification Script     ');
disp('========================================================================');

%% 1. 에이전트 파일 탐색 (최종 가중치 또는 savedAgents_lpv 체크포인트)
agent_file = '';
if isfile('saved_drone_lpv_ppo_agent.mat')
    agent_file = 'saved_drone_lpv_ppo_agent.mat';
elseif isfolder('savedAgents_lpv')
    chk_files = dir(fullfile('savedAgents_lpv', 'Agent*.mat'));
    if ~isempty(chk_files)
        % 가장 최신/높은 번호의 체크포인트 선택
        [~, idx] = sort([chk_files.datenum], 'descend');
        agent_file = fullfile('savedAgents_lpv', chk_files(idx(1)).name);
    end
end

if isempty(agent_file) && isfile('saved_ppo_agent.mat')
    agent_file = 'saved_ppo_agent.mat';
end

if isempty(agent_file)
    error('테스트할 에이전트 가중치(.mat) 파일을 찾을 수 없습니다.');
end

fprintf('선택된 에이전트 가중치: %s\n', agent_file);
load(agent_file, 'agent');

%% 2. 시뮬레이션 환경 및 기체 초기화
if ~exist('eval_speed', 'var') || isempty(eval_speed)
    eval_speed = 4.0; % 기본 평가 속도 (필요시 7.5 등으로 변경 가능)
end

env = RL_DroneEnv_LPV();
env.randomize_speed = false; % 검증 시에는 기준 속도 고정
env.randomize_path  = false; % 기준 경로(Path 2) 사용
env.cfg.target_speed = eval_speed;
env.cfg.drone1_init_states(4) = eval_speed;
obs = reset(env);

cfg = env.cfg;
cfg.play_animation = false;       % 애니메이션 재생 여부 (필요 시 true)
cfg.auto_save      = true;        % finalize_and_save_results 자동 저장 활성화
cfg.target_speed   = eval_speed;
cfg.save_dir       = fullfile(script_dir, sprintf('sim_results_ppo/v=%.1f_LPV_PPO', eval_speed));

drone1 = env.drone1;
sim_tf = drone1.tf;
num_steps = ceil(sim_tf / cfg.dt);

%% 3. finalize_and_save_results.m 규격에 맞춘 로깅 메모리 사전 할당
log_data = struct();
log_data.t_hist       = zeros(1, num_steps);
log_data.state_hist   = zeros(12, num_steps);
log_data.pos_des_hist = zeros(3, num_steps);
log_data.att_des_hist = zeros(4, num_steps);
log_data.u_hist       = zeros(4, num_steps);
log_data.PID_Pos_hist = zeros(3, num_steps); % [kP_z; kI_z; kD_z]
log_data.kappa_hist   = zeros(1, num_steps);
log_data.action_hist  = zeros(4, num_steps); % PPO 잔차 액션 추가 기록

N_sub = 10; % 외부 RL 개입 주기 (100 Hz / 10 = 10 Hz)
prev_yaw = 0.0;
current_act = [0; 0; 0; 0];

disp('시뮬레이션 루프를 실행합니다...');

%% 4. 시뮬레이션 루프
for i = 1:num_steps
    drone1_state = drone1.GetState();
    current_time = drone1.t;
    
    [traj_pos, traj_vel, kappa_avg] = drone1.trajCtrl.get_position(current_time);
    
    % --- [A] RL 에이전트 잔차 게인 오프셋 추론 (10스텝마다) ---
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
        if iscell(action)
            current_act = action{1};
        else
            current_act = action;
        end
        
        % 제어기에 PPO 잔차 게인 오프셋 주입
        dP_xy = env.range_P_xy * current_act(1);
        dD_xy = max(0, env.range_D_xy * current_act(2));
        dP_z  = env.range_P_z  * current_act(3);
        dD_z  = max(0, env.range_D_z  * current_act(4));
        drone1.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);
    end
    
    % --- [B] 동적 Yaw 및 제어 지령 산출 ---
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
    
    % 제어 명령: [traj_pos; yaw; speed; kappa]
    current_cmd = [traj_pos; dynamic_target_yaw; cfg.target_speed; kappa_avg];
    
    % 위치 제어기 (직접 Thrust U1 생성)
    att_cmd = drone1.posCtrl.Update(current_cmd, drone1_state);
    
    % 자세 제어기 (모터 명령 생성)
    u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);
    
    % 드론 동역학 업데이트
    drone1.UpdateState(u_motor);
    
    % --- [C] finalize_and_save_results 형식에 맞춘 완벽한 로깅 ---
    log_data.t_hist(i)         = current_time;
    log_data.state_hist(:,i)   = drone1_state;
    log_data.pos_des_hist(:,i) = traj_pos;
    log_data.att_des_hist(:,i) = att_cmd;
    log_data.u_hist(:,i)       = u_motor;
    log_data.PID_Pos_hist(:,i) = drone1.posCtrl.GetZ_PID();
    log_data.kappa_hist(i)     = kappa_avg;
    log_data.action_hist(:,i)  = current_act;
end

disp('시뮬레이션 완료. finalize_and_save_results를 호출하여 결과를 플롯하고 저장합니다.');

%% 5. 애니메이션 및 결과 출력/저장
if cfg.play_animation
    play_simulation_graphics(drone1, log_data, cfg);
end

% Drone_LPV 폴더 내의 finalize_and_save_results 호출 (동일한 6~8개 그래프 생성 및 자동 저장)
finalize_and_save_results(log_data, cfg);

disp('========================================================================');
disp('   모든 로깅 변수 기록 및 그래프 저장 완료!                           ');
disp('========================================================================');
