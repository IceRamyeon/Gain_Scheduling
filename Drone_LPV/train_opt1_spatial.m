% train_opt1_spatial.m
% Option 1 (11차원 유지): 미래 곡률 예견 시간을 속력 반비례(dt = L / v_des, L = 2.0m)로 적용하여 학습
clear; clc; close all;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
cd(script_dir); addpath(genpath(script_dir));
rng(42);

fprintf('=== [Option 1: 11D Spatial Lookahead (dt = L/v, L=2.0m)] 학습 시작 ===\n');
env = RL_DroneEnv_LPV('spatial', 2.0);
env.randomize_speed = true;
env.randomize_path  = true;
env.use_curriculum  = true;
env.total_episodes  = 3000;

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

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
    'SampleTime', 0.1, ...
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

chk_dir = fullfile(script_dir, 'savedAgents_opt1_spatial');
if ~isfolder(chk_dir), mkdir(chk_dir); end

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
fprintf('=== [Option 1] 학습 및 저장 완료 ===\n');
