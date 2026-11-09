% RL_train_drone_lpv_ppo.m
% Drone_LPV 베이스라인 위에서 동작하는 PPO 잔차 제어기 학습 스크립트
% - 상태 벡터 (11x1): [e_x, e_y, e_z, v_x, v_y, v_z, phi, theta, v_des, kappa_now, kappa_future]
% - 직선(kappa = 0) + 고정 곡률 원호(Arc: r, theta) 조합 경로에서 학습
% - 학습 초기 Actor 출력을 0에 가깝게 초기화(Near-Zero Initialization)하여 초반에 LPV가 운전을 주도하도록 구성

clear; clc; close all;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(script_dir);
end

fprintf('========================================================================\n');
fprintf('  Drone_LPV 기반 PPO 잔차 제어기 신규 학습 (총 3,000 에피소드 고난도 집중)\n');
fprintf('  - 관측 벡터 (11x1): 위치오차(3), 속도(3), 자세(2), 목표속력(1), 현재/미래 곡률(2)\n');
fprintf('  - 코스 확장: 원호 5~8개 (길이 200~350m), 지름 5m/2.5m 헤어핀 발생비율 40%%\n');
fprintf('  - 100에피소드 주기 체크포인트 자동 저장 (Agent0 ~ Agent3000)\n');
fprintf('========================================================================\n');

%% 1. 환경 생성
env = RL_DroneEnv_LPV();
env.randomize_speed = true;
env.randomize_path  = true;
env.use_curriculum  = true;
env.total_episodes  = 3000;
env.lookahead_dt    = 0.5; % 0.5초 후의 미래 곡률 관측

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

%% 2. PPO Actor-Critic 네트워크 구축 (Near-Zero Residual Initialization)
fprintf('>>> 11차원 입력 및 Near-Zero 초기화가 적용된 Actor-Critic 신경망을 구축합니다. <<<\n');

% Critic 네트워크 (Value Function V(s))
criticNetwork = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(64, 'Name', 'CriticFC1')
    reluLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(64, 'Name', 'CriticFC2')
    reluLayer('Name', 'CriticRelu2')
    fullyConnectedLayer(1, 'Name', 'CriticOutput')
];
criticOpts = rlRepresentationOptions('LearnRate', 1e-3, 'GradientThreshold', 1);
critic = rlValueRepresentation(criticNetwork, obsInfo, 'Observation', {'State'}, criticOpts);

% Actor 네트워크 (초기 평균 출력 ≈ 0, 초기 탐색 표준편차 ≈ 0.10 으로 LPV 운전 주도 보장)
inPath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(64, 'Name', 'ActorFC1')
    reluLayer('Name', 'ActorRelu1')
    fullyConnectedLayer(64, 'Name', 'ActorFC2')
    reluLayer('Name', 'ActorRelu2')
];

% Mean 출력층: 가중치를 1e-3 스케일로 매우 작게, 바이어스를 0으로 초기화 -> 초기 잔차 액션 평균 ≈ 0
meanPath = [
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorMeanFC', ...
        'WeightsInitializer', @(sz) 1e-3 * randn(sz, 'single'), ...
        'BiasInitializer', 'zeros')
    tanhLayer('Name', 'ActorTanh') % Action Limit [-1, 1]
];

% Std 출력층: 초기 바이어스를 -2.2로 설정 -> softplus(-2.2) ≈ 0.105 (과도한 초기 랜덤 외란 방지)
stdPath = [
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorStdFC', ...
        'WeightsInitializer', @(sz) 1e-3 * randn(sz, 'single'), ...
        'BiasInitializer', @(sz) -2.2 * ones(sz, 'single'))
    softplusLayer('Name', 'ActorStd')
];

actorNetwork = layerGraph(inPath);
actorNetwork = addLayers(actorNetwork, meanPath);
actorNetwork = addLayers(actorNetwork, stdPath);
actorNetwork = connectLayers(actorNetwork, 'ActorRelu2', 'ActorMeanFC');
actorNetwork = connectLayers(actorNetwork, 'ActorRelu2', 'ActorStdFC');
actorNetwork = dlnetwork(actorNetwork);

actorOpts = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 1);
actor = rlContinuousGaussianActor(actorNetwork, obsInfo, actInfo, ...
    'ObservationInputNames', 'State', ...
    'ActionMeanOutputNames', 'ActorTanh', ...
    'ActionStandardDeviationOutputNames', 'ActorStd');

% PPO 에이전트 옵션 설정
agentOpts = rlPPOAgentOptions(...
    'SampleTime', 0.1, ...              % 외부 RL 개입 주기와 일치 (10 스텝 * 0.01s = 0.1s)
    'ExperienceHorizon', 512, ...
    'ClipFactor', 0.2, ...
    'EntropyLossWeight', 0.005, ...
    'MiniBatchSize', 64, ...
    'NumEpoch', 3, ...
    'AdvantageEstimateMethod', 'gae', ...
    'GAEFactor', 0.95);

agentOpts.ActorOptimizerOptions.LearnRate  = 1e-4;
agentOpts.CriticOptimizerOptions.LearnRate = 1e-3;

agent = rlPPOAgent(actor, critic, agentOpts);

%% 3. 학습 옵션 설정 및 0번째 에이전트(초기 가중치) 사전 저장
chk_dir = fullfile(script_dir, 'savedAgents_lpv');
if ~isfolder(chk_dir)
    mkdir(chk_dir);
end

% 0번째 에피소드 가중치 (Near-Zero 초기화 상태) 사전 저장
saved_agent = agent;
save(fullfile(chk_dir, 'Agent0.mat'), 'saved_agent');
fprintf('>>> [체크포인트] 0번째(초기화된 가중치) 에이전트 저장 완료: Agent0.mat <<<\n');

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 3000, ...            % 총 3,000 에피소드
    'MaxStepsPerEpisode', 1500, ...
    'ScoreAveragingWindowLength', 50, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 100, ...          % 100 에피소드마다 자동 저장 (Agent0, Agent100, ..., Agent3000)
    'SaveAgentDirectory', chk_dir, ...
    'StopTrainingCriteria', 'None');

fprintf('>>> 3,000 에피소드 신규 학습을 시작합니다... <<<\n');
trainingStats = train(agent, env, trainOpts);

%% 4. 최종 에이전트 저장
save('saved_drone_lpv_ppo_agent.mat', 'agent', 'trainingStats');
fprintf('\n========================================================================\n');
fprintf('  학습 완료! saved_drone_lpv_ppo_agent.mat에 저장되었습니다.\n');
fprintf('========================================================================\n');
