% RL_test_drone_lpv_agent.m
% Drone_LPV 베이스라인 + PPO 잔차 제어기 성능 검증 스크립트 (공용 run_simulation_loop 활용)

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(genpath(script_dir));
end

clearvars -except script_dir eval_speed; clc; close all;

disp('========================================================================');
disp('   Drone_LPV + PPO Residual Control Performance Verification Script     ');
disp('========================================================================');

%% 1. 에이전트 가중치 탐색 및 로드
agent_file = '';
if isfile('saved_drone_lpv_ppo_agent.mat')
    agent_file = 'saved_drone_lpv_ppo_agent.mat';
elseif isfolder('savedAgents_lpv')
    chk_files = dir(fullfile('savedAgents_lpv', 'Agent*.mat'));
    if ~isempty(chk_files)
        [~, idx] = sort([chk_files.datenum], 'descend');
        agent_file = fullfile('savedAgents_lpv', chk_files(idx(1)).name);
    end
end

if isempty(agent_file)
    error('테스트할 에이전트 가중치(.mat) 파일을 찾을 수 없습니다.');
end

fprintf('선택된 에이전트 가중치: %s\n', agent_file);
loaded = load(agent_file);
if isfield(loaded, 'agent'), agent = loaded.agent; else, agent = loaded.saved_agent; end

%% 2. 시뮬레이션 환경 및 기체 초기화
if ~exist('eval_speed', 'var') || isempty(eval_speed)
    eval_speed = 4.0;
end

env = RL_DroneEnv_LPV();
env.randomize_speed = false;
env.randomize_path  = false;
env.cfg.target_speed = eval_speed;
env.cfg.drone1_init_states(4) = eval_speed;
reset(env);

cfg = env.cfg;
cfg.play_animation = false;
cfg.auto_save      = true;
cfg.target_speed   = eval_speed;
cfg.save_dir       = fullfile(script_dir, sprintf('sim_results_ppo/v=%.1f_LPV_PPO', eval_speed));

%% 3. 공용 시뮬레이션 루프 실행 및 결과 저장
disp('시뮬레이션 루프를 실행합니다...');
[log_data, ~] = run_simulation_loop(env.drone1, cfg, agent, env);

if cfg.play_animation
    play_simulation_graphics(env.drone1, log_data, cfg);
end

finalize_and_save_results(log_data, cfg);
disp('========================================================================');
