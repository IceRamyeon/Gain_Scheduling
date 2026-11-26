% RL_resume_gs_altstep_6000.m
% Ep 4000 가중치(saved_gs_altstep_ppo_agent.mat)에서 이어서 2,000 에피소드 추가 학습 (총 6,000 에피소드)
% - 500 에피소드마다 체크포인트 저장 -> Agent4500, Agent5000, Agent5500, Agent6000 으로 정리

clear; clc; close all;
[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
parent_dir = fileparts(px4_dir);
cd(px4_dir);
addpath(px4_dir);
addpath(parent_dir);

% 1. 기존 Ep 4000 모델 백업 및 로드
copyfile('saved_gs_altstep_ppo_agent.mat', 'saved_gs_altstep_4000ep_backup.mat');
S = load('saved_gs_altstep_ppo_agent.mat');
if isfield(S, 'agent'), agent = S.agent; else, agent = S.saved_agent; end
agent.UseExplorationPolicy = true;

rng(100);
env = RL_DroneEnv_GS_AltStep(false);

resume_dir = fullfile(px4_dir, 'savedAgents_gs_resume');
if exist(resume_dir, 'dir'), rmdir(resume_dir, 's'); end
mkdir(resume_dir);

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 2000, ...
    'MaxStepsPerEpisode', 1200, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'Verbose', true, ...
    'Plots', 'none', ...
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 500, ...
    'SaveAgentDirectory', resume_dir);

fprintf('\n>>> Resuming Training from Ep 4000 -> Ep 6000 (+2,000 Episodes)...\n');
trainingStats = train(agent, env, trainOpts);

% 체크포인트를 savedAgents_gs_altstep/Agent4500~6000.mat 으로 복사
main_ckpt_dir = fullfile(px4_dir, 'savedAgents_gs_altstep');
for ep_add = [500, 1000, 1500, 2000]
    src_f = fullfile(resume_dir, sprintf('Agent%d.mat', ep_add));
    dst_f = fullfile(main_ckpt_dir, sprintf('Agent%d.mat', 4000 + ep_add));
    if exist(src_f, 'file')
        copyfile(src_f, dst_f);
    end
end

saved_agent = agent;
save(fullfile(main_ckpt_dir, 'Agent6000.mat'), 'saved_agent');
save(fullfile(px4_dir, 'saved_gs_altstep_ppo_agent.mat'), 'agent', 'saved_agent', 'trainingStats');
fprintf('\n>>> Resume Training Complete (Ep 6000)! Saved to saved_gs_altstep_ppo_agent.mat\n');
