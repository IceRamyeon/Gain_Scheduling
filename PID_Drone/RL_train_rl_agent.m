% RL_train_rl_agent.m
% 온라인 잔차 강화학습 기반 게인 스케줄링 학습 스크립트 (PPO)
% 사전 학습된 가중치 기반 전이 학습(Warm-Start / Fine-Tuning) 지원

clear; clc;

%% 1. 환경 구성 (2D 커리큘럼: 속도 & 곡률 단계적 난이도 상향)
env = RL_DroneEnv();
env.randomize_speed = true;        % 속도 무작위화 활성화
env.randomize_path  = true;        % 궤적 곡률 무작위화 활성화
env.use_curriculum  = true;        % 2D 커리큘럼 학습(Stage 1 -> Stage 2 -> Stage 3) 활성화
env.max_tilt_deg    = 75.0;        % 고속 선회 뱅크각 허용 한계 완화 (추락 오인 방지)

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

%% 2. 에이전트 준비: 기존 학습된 가중치 로드(Fine-Tuning) 또는 신규 생성
pretrained_file = '';
if isfile('saved_ppo_agent.mat')
    pretrained_file = 'saved_ppo_agent.mat';
end

if ~isempty(pretrained_file)
    fprintf('========================================================================\n');
    fprintf('  기존에 잘 학습된 에이전트(%s)를 로드하여 Fine-Tuning을 진행합니다.\n', pretrained_file);
    fprintf('========================================================================\n');
    loaded_data = load(pretrained_file, 'agent');
    agent = loaded_data.agent;
    
    % 기존에 잘 학습된 지식을 파괴하지 않으면서 새로운 고속 영역에 적응하도록 미세 조정
    agent.AgentOptions.ActorOptimizerOptions.LearnRate  = 5e-5;  % 미세 조정용 학습률
    agent.AgentOptions.CriticOptimizerOptions.LearnRate = 5e-4;
    agent.AgentOptions.EntropyLossWeight                = 0.005; % 적절한 탐색 유지
else
    fprintf('========================================================================\n');
    fprintf('  사전 학습된 모델이 없어 처음부터(From Scratch) 네트워크를 구축합니다.\n');
    fprintf('========================================================================\n');
    
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

    % Actor 네트워크 생성 (Gaussian, Two Branches)
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
    actorNetwork = dlnetwork(actorNetwork);

    actorOpts = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 1);
    actor = rlContinuousGaussianActor(actorNetwork, obsInfo, actInfo, ...
        'ObservationInputNames', 'State', ...
        'ActionMeanOutputNames', 'ActorTanh', ...
        'ActionStandardDeviationOutputNames', 'ActorStd');

    % PPO 에이전트 설정
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
end

%% 3. 학습 옵션 설정 및 학습 시작 (총 5,000 에피소드 완주)
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 5000, ...            % 총 5000 에피소드 학습
    'MaxStepsPerEpisode', 1000, ...
    'ScoreAveragingWindowLength', 50, ...
    'Verbose', false, ...
    'Plots', 'training-progress', ...
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 500, ...          % 500 에피소드마다 중간 가중치 자동 백업
    'StopTrainingCriteria', 'None');    % 중도 종료 없이 5000회 완주

disp('Fine-Tuning PPO Agent across all speeds (1~10 m/s) and curvatures for 5000 episodes...');
trainingStats = train(agent, env, trainOpts);

% 결과 저장
save('saved_ppo_agent.mat', 'agent');
disp('학습 완료! 업데이트된 가중치가 saved_ppo_agent.mat에 저장되었습니다.');
