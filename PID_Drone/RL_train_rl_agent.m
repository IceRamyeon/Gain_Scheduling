% RL_train_rl_agent.m
% 온라인 잔차 강화학습 기반 게인 스케줄링 학습 스크립트 (PPO)

clear; clc;

%% 1. 환경 구성
env = RL_DroneEnv();
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

%% 2. Critic 네트워크 생성 (Value Function V(s))
% PPO의 Critic은 상태(State)만을 입력받아 현재 상태의 가치(Value)를 출력합니다.
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

%% 3. Actor 네트워크 생성 (Gaussian, Two Branches)
inPath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(64, 'Name', 'ActorFC1')
    reluLayer('Name', 'ActorRelu1')
    fullyConnectedLayer(64, 'Name', 'ActorFC2')
    reluLayer('Name', 'ActorRelu2')
];
meanPath = [
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorMeanFC')
    tanhLayer('Name', 'ActorTanh') % Action Limit -1 to 1 (잔차 오프셋 제한)
];
stdPath = [
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorStdFC')
    softplusLayer('Name', 'ActorStd') % 양수 보장
];

actorNetwork = layerGraph(inPath);
actorNetwork = addLayers(actorNetwork, meanPath);
actorNetwork = addLayers(actorNetwork, stdPath);
actorNetwork = connectLayers(actorNetwork, 'ActorRelu2', 'ActorMeanFC');
actorNetwork = connectLayers(actorNetwork, 'ActorRelu2', 'ActorStdFC');

actorNetwork = dlnetwork(actorNetwork); % 호환성 확보

actorOpts = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 1);
actor = rlContinuousGaussianActor(actorNetwork, obsInfo, actInfo, ...
    'ObservationInputNames', 'State', ...
    'ActionMeanOutputNames', 'ActorTanh', ...
    'ActionStandardDeviationOutputNames', 'ActorStd');

%% 4. PPO 에이전트 설정
% PPO 알고리즘 특유의 하이퍼파라미터 설정
agentOpts = rlPPOAgentOptions(...
    'SampleTime', 0.01, ...
    'ExperienceHorizon', 256, ...       % N-step 후 업데이트
    'ClipFactor', 0.2, ...              % 정책 업데이트 제한(Surrogate clipping)
    'EntropyLossWeight', 0.01, ...      % 탐험(Exploration) 장려
    'MiniBatchSize', 64, ...
    'NumEpoch', 3, ...
    'AdvantageEstimateMethod', 'gae', ... % Generalized Advantage Estimation
    'GAEFactor', 0.95);

agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
agentOpts.CriticOptimizerOptions.LearnRate = 1e-3;

agent = rlPPOAgent(actor, critic, agentOpts);

%% 5. 학습 옵션 설정 및 학습 시작
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 3000, ...
    'MaxStepsPerEpisode', env.max_steps, ...
    'ScoreAveragingWindowLength', 50, ...
    'Verbose', false, ...
    'Plots', 'training-progress', ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1500); % 목표 보상 도달 시 학습 종료

disp('Training PPO Agent... (This might take a while)');
trainingStats = train(agent, env, trainOpts);
save('saved_ppo_agent.mat', 'agent');
disp('Training Completed and Agent Saved.');
