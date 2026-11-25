% RL_train_gs_altstep.m
% (B안) ChoeJae PositionCtrl_GS 기반 원 운동 중 고도 계단(+0.3m) 추종 PPO 학습 스크립트
% - 2,000 에피소드 학습, 체크포인트 250 에피소드마다 저장
% - 학습 완료본: saved_gs_altstep_ppo_agent.mat

clear; clc; close all;
[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
parent_dir = fileparts(px4_dir);
cd(px4_dir);
addpath(px4_dir);
addpath(parent_dir);

fprintf('=================================================================\n');
fprintf('  [Option B] ChoeJae PositionCtrl_GS + PPO Altitude Step Training\n');
fprintf('  - Base Controller : PositionCtrl_GS (Z_D_on_meas=true, Err_Yaw=desired)\n');
fprintf('  - Thrust Curve    : u = 0.73*sqrt(U1/mg) + Gazebo T ~ (u/0.73)^[1.75,2.05]\n');
fprintf('  - Observation     : 11D World NED (ChoeJae run_px4_mission.m 동일)\n');
fprintf('  - Mission         : CircleTrajectory + Altitude Step [0.2~0.5 m]\n');
fprintf('=================================================================\n');

rng(42);
env     = RL_DroneEnv_GS_AltStep(false);
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
obsDim  = obsInfo.Dimension(1);
actDim  = actInfo.Dimension(1);

% Critic Network (64x64 ReLU MLP)
criticNet = [
    featureInputLayer(obsDim, 'Name', 'obs')
    fullyConnectedLayer(64, 'Name', 'c_fc1')
    reluLayer('Name', 'c_relu1')
    fullyConnectedLayer(64, 'Name', 'c_fc2')
    reluLayer('Name', 'c_relu2')
    fullyConnectedLayer(1, 'Name', 'c_out')
];
critic = rlValueFunction(dlnetwork(layerGraph(criticNet)), obsInfo);

% Actor Network (64x64 ReLU MLP + Near-Zero Init -> Ep 0 = Pure GS Baseline)
commonPath = [
    featureInputLayer(obsDim, 'Name', 'obs')
    fullyConnectedLayer(64, 'Name', 'a_fc1')
    reluLayer('Name', 'a_relu1')
    fullyConnectedLayer(64, 'Name', 'a_fc2')
    reluLayer('Name', 'a_relu2')
];
meanPath = [
    fullyConnectedLayer(actDim, 'Name', 'mean_fc', ...
        'Weights', randn(actDim, 64) * 1e-3, 'Bias', zeros(actDim, 1))
    tanhLayer('Name', 'mean_out')
];
stdPath = [
    fullyConnectedLayer(actDim, 'Name', 'std_fc', ...
        'Weights', zeros(actDim, 64), 'Bias', -2.2 * ones(actDim, 1))
    softplusLayer('Name', 'std_out')
];

actorLG = layerGraph(commonPath);
actorLG = addLayers(actorLG, meanPath);
actorLG = addLayers(actorLG, stdPath);
actorLG = connectLayers(actorLG, 'a_relu2', 'mean_fc');
actorLG = connectLayers(actorLG, 'a_relu2', 'std_fc');
actor = rlContinuousGaussianActor(dlnetwork(actorLG), obsInfo, actInfo, ...
    'ActionMeanOutputNames', 'mean_out', ...
    'ActionStandardDeviationOutputNames', 'std_out');

% PPO Agent Options
agentOpts = rlPPOAgentOptions( ...
    'ExperienceHorizon', 2048, ...
    'MiniBatchSize', 128, ...
    'ClipFactor', 0.2, ...
    'EntropyLossWeight', 0.005, ...
    'NumEpoch', 4, ...
    'AdvantageEstimateMethod', 'gae', ...
    'GAEFactor', 0.95, ...
    'DiscountFactor', 0.99, ...
    'SampleTime', 0.1);
agentOpts.ActorOptimizerOptions  = rlOptimizerOptions('LearnRate', 2e-4, 'GradientThreshold', 1.0);
agentOpts.CriticOptimizerOptions = rlOptimizerOptions('LearnRate', 5e-4, 'GradientThreshold', 1.0);

agent = rlPPOAgent(actor, critic, agentOpts);

% Save Ep 0 Baseline Agent
save_dir = fullfile(px4_dir, 'savedAgents_gs_altstep');
if ~exist(save_dir, 'dir'), mkdir(save_dir); end
saved_agent = agent;
save(fullfile(save_dir, 'Agent0.mat'), 'saved_agent');

% Training Options (4,000 Episodes, 500 에피소드마다 가중치 저장)
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 4000, ...
    'MaxStepsPerEpisode', 1200, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'Verbose', true, ...
    'Plots', 'none', ...
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 500, ...
    'SaveAgentDirectory', save_dir);

fprintf('\n>>> Training Started (4,000 Episodes, checkpoint every 500 ep)...\n');
trainingStats = train(agent, env, trainOpts);

saved_agent = agent;
save(fullfile(px4_dir, 'saved_gs_altstep_ppo_agent.mat'), 'agent', 'saved_agent', 'trainingStats');
save(fullfile(save_dir, 'Agent4000.mat'), 'saved_agent');
fprintf('\n>>> Training Complete! Saved to saved_gs_altstep_ppo_agent.mat\n');
