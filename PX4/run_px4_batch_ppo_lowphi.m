function run_px4_batch_ppo_lowphi(start_at, agent_file, with_baseline)
%RUN_PX4_BATCH_PPO_LOWPHI  저각도(phi_eq 10 / 20 deg) GS+PPO 일괄 실험, 계단 0.5 m
%   run_px4_batch_ppo_lowphi                      % GS+PPO 6회 (에이전트는 px4_main 의 cfg.ppo.agent_file)
%   run_px4_batch_ppo_lowphi([], 'ppo/새_에이전트.mat')   % 다른 에이전트 파일로 (PX4 폴더 기준 상대 경로 또는 절대 경로)
%   run_px4_batch_ppo_lowphi([], [], true)        % 같은 운용점의 FF, GS 도 같이 (18회)
%   run_px4_batch_ppo_lowphi(4)                   % 4번째 실험부터 이어서 ([i/N] 표시 번호와 같음)
%
%  구성: phi_eq = 10 / 20 deg x v = 10 / 9 / 8, 계단 0.5 m, 지연 보상 Eq_Delay_Comp = false, 직진 없음
%  kappa (run_px4_batch_sweep 와 같은 값 -> 그 배치의 FF / GS 결과와 바로 비교 가능)
%            v = 10    v = 9     v = 8
%     10 deg 0.0173    0.0214    0.0270     (n_laps 1)
%     20 deg 0.0357    0.0441    0.0558     (n_laps 2)
%  n_laps: 고도 단계당 약 t_level = 30 s (sweep 와 같은 규칙)
%  계단 순서: {up,down} x 4 (고도 단계 9개)
%  순서: phi_eq 10 -> 20 deg, 각각 v = 10 -> 9 -> 8 (with_baseline 이면 각 운용점에서 FF -> GS -> GS+PPO)
%  예상 시간: GS+PPO 6회 약 40분 (with_baseline 이면 18회 약 2시간)
%
%  동작
%   - 모든 실험은 원래 프리롤 시작점(원점)에서 시작하고, 끝나면 원점으로 복귀 (복귀는 최대 20 m/s, 사인 램프 가감속)
%     원 궤도·직진 모두: 착륙하지 않고 공중에서 바로 원점 복귀 -> 착륙·disarm -> ULog 결과 저장 (end_mode = 'return_land_plot')
%       (착륙 -> disarm -> 재이륙 과정이 없어 재시동 거부로 멈추는 문제를 피함)
%              (Gazebo 바닥이 유한해서 ~1.2 km 밖에서 착륙하면 바닥을 뚫고 떨어짐)
%   - 실행 사이 pause_between 초 대기 후 다음 실험 시작
%   - 비행이 중단(abort)되어도 다음 실험 계속 (continue_on_abort), MATLAB 오류가 나면 나머지 실험은 건너뜀
%   - 끝나면 실험 이름과 시작·종료 시각 표를 출력하고 results/batch_<시각>.txt 로 저장
%     (결과 Summary 파일 이름의 시각으로 어떤 실험인지 대응시키면 됨)
%
%  실험을 바꾸려면 아래 runs 목록만 고치면 된다. 덮어쓰는 필드 이름은 px4_main 의 cfg 필드와 같아야 함.
%  (중첩 필드는 struct 로: 예) ov.px4 = struct('end_mode', 'land_plot'))

here = fileparts(mfilename('fullpath'));
addpath(here);

% 명령창에서 px4_main 을 직접 돌린 적이 있으면 base 작업공간의 link(Px4Link)가 UDP 14540 포트를 계속 잡고 있다
%  -> 여기서 새로 연결하면 "bind: Address already in use". base 의 연결 객체를 먼저 지워 포트를 놓아준다.
evalin('base', 'clear(''link'', ''cleanupObj'')');
pause(1);

pause_between = 1;      % 실험 사이 대기 [s]
continue_on_abort = true;   % 비행이 중단돼도(⑤ 착륙·복귀는 마침) 다음 실험 계속 (MATLAB 오류면 멈춤)

%% 공통 설정
if nargin < 1 || isempty(start_at), start_at = 1; end            % 이어서 시작할 실험 번호
if nargin < 2, agent_file = ''; end                              % '' 이면 px4_main 의 cfg.ppo.agent_file
if nargin < 3 || isempty(with_baseline), with_baseline = false; end
t_level  = 30;           % 고도 단계 하나의 목표 시간 [s] -> n_laps 자동 계산
vs       = [10 9 8];
phis     = [10 20];
K = [0.0173 0.0214 0.0270;      % 10 deg  (열: v = 10, 9, 8)
     0.0357 0.0441 0.0558];     % 20 deg
base = struct();
base.path_type         = 'circle';
base.Feedforward       = true;
base.Z_Gain_Scheduling = true;
base.Eq_Delay_Comp     = false;
base.height_step       = 0.5;
base.height_order      = {"up","down","up","down","up","down","up","down"};
ppo_ov = struct();
if ~isempty(agent_file)
    if ~isfile(agent_file) && isfile(fullfile(here, agent_file)), agent_file = fullfile(here, agent_file); end
    if ~isfile(agent_file), error('에이전트 파일이 없습니다: %s', agent_file); end
    ppo_ov.agent_file = agent_file;
    fprintf('PPO 에이전트 지정: %s\n', agent_file);
end

%% 실험 목록 (순서대로 실행)
runs = cell(0, 2);
for ip = 1:numel(phis)
    for iv = 1:numel(vs)
        v = vs(iv); k = K(ip, iv);
        nl = max(1, round(t_level / (2 * pi / (k * v))));
        tagc = sprintf('s0.5_v%d_p%d_k%.4g_n%d', v, phis(ip), k, nl);
        b = setf(base, 'target_speed', v, 'curvature', k, 'n_laps', nl);
        if with_baseline
            runs(end + 1, :) = {[tagc '_FF'], setf(b, 'mode', 'MATLAB', 'Z_Gain_Scheduling', false)};   %#ok<AGROW>
            runs(end + 1, :) = {[tagc '_GS'], setf(b, 'mode', 'MATLAB')};                                %#ok<AGROW>
        end
        bp = setf(b, 'mode', 'PPO');
        if ~isempty(fieldnames(ppo_ov)), bp.ppo = ppo_ov; end
        runs(end + 1, :) = {[tagc '_GSPPO'], bp};                                                        %#ok<AGROW>
    end
end
fprintf('일괄 실험 %d개 구성 (계단 0.5 m, phi_eq 10/20 deg)\n', size(runs, 1));

nRun = size(runs, 1);
status = repmat({'건너뜀'}, nRun, 1);
t_start = NaT(nRun, 1);
t_end   = NaT(nRun, 1);

% 끝나면(오류·Ctrl+C 포함) 덮어쓰기 값을 반드시 지움 -> 이후 px4_main 단독 실행에 영향 없음
cleanupOv = onCleanup(@() rmappdata_safe());

batch_time = datetime('now');
fprintf('===== 일괄 실험 %d개 시작 (%s) =====\n', nRun, char(batch_time));

if start_at > 1
    fprintf('%d번째 실험부터 이어서 시작 (1 ~ %d 번은 건너뜀)\n', start_at, start_at - 1);
end
for i = start_at:nRun
    name = runs{i, 1};
    ov   = runs{i, 2};
    % 모든 실험: 원래 프리롤 시작점(원점)에서 시작하고, 끝나면 원점으로 복귀
    %  - 원 궤도·직진 모두: 착륙하지 않고 공중에서 원점 복귀 -> 착륙 -> ULog 결과 저장 (return_land_plot)
    if ~isfield(ov, 'px4'), ov.px4 = struct(); end
    ov.px4.start_here = false;
    ov.px4.end_mode = 'return_land_plot';
    fprintf('\n===== [%d/%d] %s =====\n', i, nRun, name);
    setappdata(0, 'px4_batch_label', sprintf('[%d/%d] %s', i, nRun, name));   % px4_main 이 clc 후 표시
    t_start(i) = datetime('now');
    try
        aborted = run_one(ov);
        t_end(i) = datetime('now');
        if aborted
            status{i} = '중단(abort)';
            if continue_on_abort
                fprintf(2, '[%s] 비행이 중단되었습니다. 다음 실험을 계속합니다.\n', name);
            else
                fprintf(2, '[%s] 비행이 중단되어 나머지 실험을 건너뜁니다.\n', name);
                break;
            end
        else
            status{i} = '완료';
        end
    catch err
        t_end(i) = datetime('now');
        status{i} = ['오류: ' err.message];
        fprintf(2, '[%s] 오류: %s\n나머지 실험을 건너뜁니다.\n', name, err.message);
        break;
    end
    if i < nRun
        fprintf('다음 실험까지 %d s 대기...\n', pause_between);
        pause(pause_between);
    end
end

%% 결과 표 출력·저장
lines = cell(nRun + 2, 1);
lines{1} = sprintf('일괄 실험 (%s)', char(batch_time));
lines{2} = sprintf('%-3s %-34s %-20s %-20s %s', '#', '실험', '시작', '종료', '상태');
for i = 1:nRun
    lines{i + 2} = sprintf('%-3d %-34s %-20s %-20s %s', i, runs{i, 1}, ...
        fmt_time(t_start(i)), fmt_time(t_end(i)), status{i});
end
fprintf('\n%s\n', strjoin(lines, newline));

result_dir = fullfile(here, 'results');
if ~exist(result_dir, 'dir'), mkdir(result_dir); end
logFile = fullfile(result_dir, sprintf('batch_ppo_lowphi_%s.txt', char(datetime(batch_time, 'Format', 'yyyyMMdd_HHmmss'))));
fid = fopen(logFile, 'w');
if fid > 0
    fprintf(fid, '%s\n', strjoin(lines, newline));
    fclose(fid);
    fprintf('일괄 실험 기록: %s\n', logFile);
end
end

%% ---- 로컬 함수 ----
function aborted = run_one(ov)
% px4_main 은 스크립트이고 맨 앞에서 clear 를 하므로, 이 함수 안에서 실행해 호출자 변수(루프 등)를 보호한다.
% 덮어쓰기 값은 clear 로 지워지지 않는 appdata 로 넘긴다.
setappdata(0, 'px4_cfg_override', ov);
px4_main;                                  % 이 함수 작업공간에서 실행됨
% meta 는 스크립트가 만든 변수라 파서가 패키지 meta.* 로 오인하지 않도록 eval 로 꺼낸다
aborted = false;
if exist('meta', 'var')
    M = eval('meta');
    aborted = isfield(M, 'aborted') && M.aborted;
end
end

function s = setf(s, varargin)
for k = 1:2:numel(varargin)
    s.(varargin{k}) = varargin{k + 1};
end
end

function rmappdata_safe()
if isappdata(0, 'px4_cfg_override')
    rmappdata(0, 'px4_cfg_override');
end
if isappdata(0, 'px4_batch_label')
    rmappdata(0, 'px4_batch_label');
end
end

function s = fmt_time(t)
if isnat(t), s = '-'; else, s = char(datetime(t, 'Format', 'HH:mm:ss')); end
end
