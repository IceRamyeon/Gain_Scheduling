function run_px4_batch_stepsize(steps)
%RUN_PX4_BATCH_STEPSIZE  고도 계단 크기 비교 일괄 실험
%   run_px4_batch_stepsize            % 0.3 m 세트 -> 0.7 m 세트 (총 96회)
%   run_px4_batch_stepsize(0.3)       % 0.3 m 세트만 (48회)
%   run_px4_batch_stepsize(0.7)       % 0.7 m 세트만 (48회)
%
%  한 세트 = v = 10 / 9 / 8 x [직진 1회 + phi_eq 10/20/30/35/40 deg x {FF, GS, GS+PPO}] = 3 x 16 = 48회
%    지연 보상 Eq_Delay_Comp = false
%  kappa (이전 실험과 같은 값, phi_eq = atan(v^2 kappa / g))
%            v = 10    v = 9     v = 8
%     10 deg 0.0173    0.0214    0.0270
%     20 deg 0.0357    0.0441    0.0558
%     30 deg 0.0566    0.0699    0.0885
%     35 deg 0.069     0.085     0.107
%     40 deg 0.082     0.102     0.129
%  원 궤도: n_laps 를 고도 단계당 약 t_level = 30 s 가 되도록 자동 계산 (10 deg 1 ~ 40 deg v8 5바퀴)
%  직진   : 단계마다 straight_distance = 120 m (이전 직진 기준선과 같음), 끝나면 공중 복귀 후 착륙
%  GS+PPO : cfg.mode = 'PPO' (기반 제어기 = GS 와 같은 법칙, 에이전트·on_mode 등은 px4_main 의 cfg.ppo 그대로)
%  계단 순서: {up,down} x 4 (고도 단계 9개)
%  순서: 세트(계단 크기) -> v = 10, 9, 8 -> [직진, 그다음 phi_eq 10 -> 40 deg 마다 FF -> GS -> GS+PPO]
%  예상 시간: 세트당 경로 약 3시간 50분 + 이착륙·복귀 약 1시간 50분 = 약 5시간 40분, 두 세트 약 11시간 20분
%  주의: 0.7 m 계단은 40 deg 에서 추력 상한(약 29.5 N) 근처 (GS 피크 약 30.4 N, FF 약 29.5 N) -> 포화 가능
%
%  동작
%   - 모든 실험은 원래 프리롤 시작점(원점)에서 시작하고, 끝나면 원점으로 복귀 (복귀는 최대 20 m/s, 사인 램프 가감속)
%     원 궤도: 그 자리 착륙 -> 결과 저장 -> 재이륙 -> 원점 복귀 -> 착륙·disarm (end_mode = 'land_plot_return')
%     직진   : 착륙하지 않고 공중에서 원점 복귀 -> 착륙·disarm -> 결과 저장 (end_mode = 'return_land_plot')
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
if nargin < 1 || isempty(steps), steps = [0.3 0.7]; end   % 세트별 고도 계단 크기 [m]
t_level  = 30;           % 원 궤도: 고도 단계 하나의 목표 시간 [s] -> n_laps 자동 계산
vs       = [10 9 8];
phis     = [10 20 30 35 40];
K = [0.0173 0.0214 0.0270;      % 10 deg  (열: v = 10, 9, 8)
     0.0357 0.0441 0.0558;      % 20 deg
     0.0566 0.0699 0.0885;      % 30 deg
     0.069  0.085  0.107 ;      % 35 deg
     0.082  0.102  0.129 ];     % 40 deg
L_str    = 120;          % 직진: 단계마다 직진 거리 [m]
base = struct();
base.path_type         = 'circle';
base.Feedforward       = true;
base.Eq_Delay_Comp     = false;
base.height_order      = {"up","down","up","down","up","down","up","down"};

%% 실험 목록 (순서대로 실행)
runs = cell(0, 2);
for is = 1:numel(steps)
    for iv = 1:numel(vs)
        v = vs(iv);
        b  = setf(base, 'height_step', steps(is), 'target_speed', v);
        tag = sprintf('s%.1f_v%d', steps(is), v);
        runs(end + 1, :) = {[tag '_straight'], setf(b, 'mode', 'MATLAB', 'curvature', 0.0, ...
            'straight_distance', L_str, 'n_laps', 1, 'Z_Gain_Scheduling', true)};                   %#ok<AGROW>
        for ip = 1:numel(phis)
            k  = K(ip, iv);
            nl = max(1, round(t_level / (2 * pi / (k * v))));
            tagc = sprintf('%s_p%d_k%.4g_n%d', tag, phis(ip), k, nl);
            runs(end + 1, :) = {[tagc '_FF'],    setf(b, 'mode', 'MATLAB', 'curvature', k, 'n_laps', nl, 'Z_Gain_Scheduling', false)};  %#ok<AGROW>
            runs(end + 1, :) = {[tagc '_GS'],    setf(b, 'mode', 'MATLAB', 'curvature', k, 'n_laps', nl, 'Z_Gain_Scheduling', true )};  %#ok<AGROW>
            runs(end + 1, :) = {[tagc '_GSPPO'], setf(b, 'mode', 'PPO',    'curvature', k, 'n_laps', nl, 'Z_Gain_Scheduling', true )};  %#ok<AGROW>
        end
    end
end
fprintf('일괄 실험 %d개 구성 (계단 %s m)\n', size(runs, 1), num2str(steps));

nRun = size(runs, 1);
status = repmat({'건너뜀'}, nRun, 1);
t_start = NaT(nRun, 1);
t_end   = NaT(nRun, 1);

% 끝나면(오류·Ctrl+C 포함) 덮어쓰기 값을 반드시 지움 -> 이후 px4_main 단독 실행에 영향 없음
cleanupOv = onCleanup(@() rmappdata_safe());

batch_time = datetime('now');
fprintf('===== 일괄 실험 %d개 시작 (%s) =====\n', nRun, char(batch_time));

for i = 1:nRun
    name = runs{i, 1};
    ov   = runs{i, 2};
    % 모든 실험: 원래 프리롤 시작점(원점)에서 시작하고, 끝나면 원점으로 복귀
    %  - 원 궤도: 그 자리 착륙 -> 결과 저장 -> 재이륙 -> 원점 복귀 -> 착륙 (land_plot_return)
    %  - 직진   : 착륙하지 않고 공중에서 원점 복귀 -> 착륙 -> 결과 저장 (return_land_plot)
    %             (~1.2 km 밖은 Gazebo 바닥이 없어 착륙하면 바닥을 뚫고 떨어짐)
    if ~isfield(ov, 'px4'), ov.px4 = struct(); end
    ov.make_plots = false;             % 배치 중 그래프 창 없음 (결과 .mat + Summary 만, 그래프는 replot_px4_result 로)
    ov.px4.start_here = false;
    if abs(ov.curvature) < 1e-6
        ov.px4.end_mode = 'return_land_plot';
    else
        ov.px4.end_mode = 'land_plot_return';
    end
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
logFile = fullfile(result_dir, sprintf('batch_stepsize_%s.txt', char(datetime(batch_time, 'Format', 'yyyyMMdd_HHmmss'))));
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
