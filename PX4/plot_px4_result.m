%% PX4 비행 결과 플롯 (ULog + MATLAB 비행 기록)
%  ④ 경로 추종 구간만 잘라서 PID_Drone/finalize_and_save_results 로 시뮬레이션과 같은 그래프를 그린다.

close all; clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'PID_Drone'));

%% 입력 파일 (비워 두면 파일 선택 창이 뜸)
metaFile = '';     % 예: fullfile(here, 'results', 'px4_run_20261031_160000.mat')
ulgFile  = '';     % 예: '~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-31/07_00_00.ulg'

if isempty(metaFile)
    [f, p] = uigetfile(fullfile(here, 'results', '*.mat'), 'px4_main 이 저장한 비행 기록 선택');
    metaFile = fullfile(p, f);
end
if isempty(ulgFile)
    [f, p] = uigetfile('*.ulg', '같은 비행의 ULog 선택');
    ulgFile = fullfile(p, f);
end

S = load(metaFile); meta = S.meta;
if meta.aborted
    warning('중단된 비행입니다: %s', meta.abort_reason);
end

%% ULog -> log_data
log_data = ulog_to_log_data(ulgFile, meta, 0.01);

%% 시뮬레이션과 같은 플롯 (자동 저장 원하면 아래 두 줄 수정)
cfg = meta.cfg;
cfg.play_animation = false;
cfg.auto_save = false;
cfg.save_dir  = fullfile(here, 'results', 'plots');

disp('※ Control Inputs 그래프의 U2~U4 는 PX4 정규화 토크(-1 ~ 1)이고, U1 은 정규화 추력을 N 으로 근사 변환한 값입니다.');
finalize_and_save_results(log_data, cfg);
