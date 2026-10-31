%% PX4 비행 결과 플롯 (ULog + MATLAB 비행 기록)
%  ④ 경로 추종 구간만 잘라서 PID_Drone/finalize_and_save_results 로 시뮬레이션과 같은 그래프를 그린다.
%
%  기본 동작 (파일 경로를 비워 두면 자동)
%   - 비행 기록: PX4/results 에서 가장 최근 px4_run_*.mat
%   - ULog     : ulogRoot 아래(날짜 폴더 포함)에서 그 비행 기록과 저장 시각이 가장 가까운 .ulg

close all; clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'PID_Drone'));

%% ULog 폴더 (PX4-Autopilot 위치에 맞게 한 번만 설정)
ulogRoot = fullfile(getenv('HOME'), 'PX4-Autopilot', 'build', 'px4_sitl_default', 'rootfs', 'fs', 'log');

%% 입력 파일 (특정 비행을 고르고 싶을 때만 채우기)
metaFile = '';     % 예: fullfile(here, 'results', 'px4_run_20261031_162748.mat')
ulgFile  = '';     % 예: fullfile(ulogRoot, '2026-10-31', '07_27_48.ulg')

if isempty(metaFile)
    metaFile = find_latest_meta(fullfile(here, 'results'));
end
S = load(metaFile); meta = S.meta;
if isempty(ulgFile)
    ulgFile = find_matching_ulog(ulogRoot, metaFile);
end
fprintf('비행 기록: %s\nULog     : %s\n', metaFile, ulgFile);

if meta.aborted
    warning('중단된 비행입니다: %s', meta.abort_reason);
end

%% ULog -> log_data
log_data = ulog_to_log_data(ulgFile, meta, 0.01);

%% 시뮬레이션과 같은 플롯 (자동 저장 원하면 아래 두 줄 수정)
cfg = meta.cfg;
cfg.play_animation = false;
cfg.auto_save = true;
cfg.save_dir  = fullfile(here, 'results', 'plots');

disp('※ Control Inputs 그래프의 U2~U4 는 PX4 정규화 토크(-1 ~ 1)이고, U1 은 정규화 추력을 N 으로 근사 변환한 값입니다.');
finalize_and_save_results(log_data, cfg);

%% ---- local functions ----
function f = find_latest_meta(folder)
    d = dir(fullfile(folder, 'px4_run_*.mat'));
    if isempty(d)
        error('%s 에 px4_run_*.mat 이 없습니다. px4_main.m 을 먼저 실행하세요.', folder);
    end
    [~, i] = max([d.datenum]);
    f = fullfile(d(i).folder, d(i).name);
end

function f = find_matching_ulog(root, metaFile)
    % ULog 는 착륙 후 DISARM 될 때 닫히므로, 비행 기록(.mat) 저장 직후에 수정 시각이 찍힌다.
    % -> 비행 기록 저장 시각과 수정 시각이 가장 가까운 .ulg 를 고른다.
    if ~isfolder(root)
        error(['ULog 폴더가 없습니다: %s\n' ...
               'plot_px4_result.m 위쪽의 ulogRoot 를 PX4-Autopilot 위치에 맞게 고쳐 주세요.'], root);
    end
    d = dir(fullfile(root, '**', '*.ulg'));
    if isempty(d)
        error('%s 아래에 .ulg 가 없습니다. ARM 후 비행까지 했는지 확인하세요.', root);
    end
    m = dir(metaFile);
    gap_min = abs([d.datenum] - m.datenum) * 24 * 60;
    [g, i] = min(gap_min);
    f = fullfile(d(i).folder, d(i).name);
    if g > 10
        warning('가장 가까운 ULog 도 비행 기록과 %.0f 분 차이가 납니다. 같은 비행인지 확인하세요.', g);
    end
end
