% RL_train_px4_ppo.m
% PX4 + Gazebo x500 전용 PPO 잔차 제어기 학습 스크립트
% - Plant: Drone_PX4 (Gazebo x500, 비대칭 모터 시정수, 4-로터 믹서 클리핑, 공기항력)
% - 자세 제어기: PX4_AttitudeCtrl (250 Hz 이중 루프)
% - 위치 제어기: PositionCtrl_PX4 (50 Hz 2-DOF)
% - 관측 벡터: 11차원 (속력 반비례 공간 예견 dt = L / v_des, L = 2.0m)

clear; clc; close all;
[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
parent_dir = fileparts(px4_dir);
cd(px4_dir);
addpath(px4_dir);
addpath(parent_dir); % pathGen.m, TrajectoryPlanner.m, RPY2Rot.m 공용 유틸리티 참조
rng(42);

fprintf('========================================================================\n');
fprintf('  [PX4 + Gazebo x500] PPO 잔차 제어기 학습 시작 (총 3,000 에피소드)\n');
fprintf('  - Plant: Drone_PX4 (m=2.064kg, 4-로터 믹서, tau_up=12.5ms/down=25ms)\n');
fprintf('  - Controller: PX4_AttitudeCtrl (250Hz) + PositionCtrl_PX4 (50Hz)\n');
fprintf('  - Comm Delay: 0.04s (10 steps @ 250Hz) | Spatial Lookahead: L=2.0m\n');
fprintf('========================================================================\n');

%% 1. 환경 생성 (GPS 랜덤워크 분리: 250Hz 모터 지연·믹서·0.04s 통신 지연 정밀 학습 우선)
env = RL_DroneEnv_PX4(false);
env.randomize_speed = true;
env.randomize_path  = true;
env.use_curriculum  = true;
env.total_episodes  = 3000;

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

%% 2. PPO Actor-Critic 네트워크 구축 (Near-Zero Residual Initialization)
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

inPath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(64, 'Name', 'ActorFC1')
    reluLayer('Name', 'ActorRelu1')
    fullyConnectedLayer(64, 'Name', 'ActorFC2')
    reluLayer('Name', 'ActorRelu2')
];
meanPath = [
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorMeanFC', ...
        'WeightsInitializer', @(sz) 1e-3 * randn(sz, 'single'), ...
        'BiasInitializer', 'zeros')
    tanhLayer('Name', 'ActorTanh')
];
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

actor = rlContinuousGaussianActor(actorNetwork, obsInfo, actInfo, ...
    'ObservationInputNames', 'State', ...
    'ActionMeanOutputNames', 'ActorTanh', ...
    'ActionStandardDeviationOutputNames', 'ActorStd');

agentOpts = rlPPOAgentOptions(...
    'SampleTime', 0.1, ...              % 10 Hz (250 Hz 기준 25 스텝마다 1회)
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

%% 3. 체크포인트 저장 및 학습 실행
chk_dir = fullfile(px4_dir, 'savedAgents_px4');
if ~isfolder(chk_dir), mkdir(chk_dir); end

saved_agent = agent;
save(fullfile(chk_dir, 'Agent0.mat'), 'saved_agent');

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 3000, ...
    'MaxStepsPerEpisode', 1500, ...
    'ScoreAveragingWindowLength', 50, ...
    'Verbose', true, ...
    'Plots', 'none', ...
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 500, ...
    'SaveAgentDirectory', chk_dir, ...
    'StopTrainingCriteria', 'None');

trainingStats = train(agent, env, trainOpts);
saved_agent = agent;
save(fullfile(chk_dir, 'Agent3000.mat'), 'saved_agent', 'trainingStats');
save(fullfile(px4_dir, 'saved_px4_ppo_agent.mat'), 'agent', 'trainingStats');
fprintf('>>> [PX4 + Gazebo x500] PPO 학습 및 저장 완료! <<<\n');
