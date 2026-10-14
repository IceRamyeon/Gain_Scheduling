% RL_train_drone_lpv_ppo.m
% 깨끗해진 Drone_LPV 베이스라인(D-on-Measurement + 65도 제한 해제) 위에서
% D 게인 하한선 보호 및 모멘트 페널티가 탑재된 PPO 잔차 제어기를 처음부터 500 에피소드 신규 훈련

clear; clc; close all;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if ~isempty(script_dir)
    cd(script_dir);
    addpath(script_dir);
end

fprintf('========================================================================\n');
fprintf('  Drone_LPV 기반 PPO 잔차 제어기 신규 학습 (총 500 에피소드)            \n');
fprintf('  - D-on-Measurement 적용 완료 (미분 킥 채터링 원천 제거)                \n');
fprintf('  - 65도 뱅크각 포화 해제 완료 (7.5 m/s 초고속 선회력 확보)             \n');
fprintf('  - D-gain 하한선 보호 적용 (댐핑 삭감 금지, Damping 보강 전용)         \n');
fprintf('  - 모멘트(U2, U3, U4) 고주파 채터링 억제 페널티 보상 적용              \n');
fprintf('========================================================================\n');

%% 1. 환경 생성
env = RL_DroneEnv_LPV();
env.randomize_speed = true;
env.randomize_path  = true;
env.use_curriculum  = true;
env.total_episodes  = 500;

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

%% 2. 깨끗한 PPO Actor-Critic 네트워크 신규 구축 (From Scratch)
fprintf('>>> 새로운 Actor-Critic 신경망을 구축합니다. <<<\n');

% Critic 네트워크 생성 (Value Function V(s))
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

% Actor 네트워크 생성 (가우시안 정책, 초기 출력은 0 근처)
inPath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(64, 'Name', 'ActorFC1')
    reluLayer('Name', 'ActorRelu1')
    fullyConnectedLayer(64, 'Name', 'ActorFC2')
    reluLayer('Name', 'ActorRelu2')
];
meanPath = [
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorMeanFC')
    tanhLayer('Name', 'ActorTanh') % Action Limit [-1, 1]
];
stdPath = [
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorStdFC')
    softplusLayer('Name', 'ActorStd') % 양수 분산
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

% PPO 에이전트 생성
agentOpts = rlPPOAgentOptions(...
    'SampleTime', 0.01, ...
    'ExperienceHorizon', 256, ...
    'ClipFactor', 0.2, ...
    'EntropyLossWeight', 0.01, ...
    'MiniBatchSize', 64, ...
    'NumEpoch', 3, ...
    'AdvantageEstimateMethod', 'gae', ...
    'GAEFactor', 0.95);

agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
agentOpts.CriticOptimizerOptions.LearnRate = 1e-3;

agent = rlPPOAgent(actor, critic, agentOpts);

%% 3. 학습 옵션 설정 (MATLAB training-progress 시각화 창 표시)
if ~isfolder('savedAgents_lpv')
    mkdir('savedAgents_lpv');
end

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 500, ...             % 총 500 에피소드
    'MaxStepsPerEpisode', 1000, ...
    'ScoreAveragingWindowLength', 30, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...   % ★ MATLAB 실시간 학습 그래프 창 활성화!
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 100, ...          % 100 에피소드마다 중간 가중치 자동 저장 (Agent100, Agent200, ...)
    'SaveAgentDirectory', fullfile(script_dir, 'savedAgents_lpv'), ...
    'StopTrainingCriteria', 'None');

fprintf('>>> MATLAB 학습 진행 그래프 창을 띄우고 500 에피소드 신규 학습을 시작합니다... <<<\n');
trainingStats = train(agent, env, trainOpts);

%% 4. 최종 에이전트 저장
save('saved_drone_lpv_ppo_agent.mat', 'agent', 'trainingStats');
fprintf('\n========================================================================\n');
fprintf('  500 에피소드 학습 완료! saved_drone_lpv_ppo_agent.mat에 저장되었습니다.\n');
fprintf('========================================================================\n');
