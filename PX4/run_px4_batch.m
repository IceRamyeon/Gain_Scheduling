function run_px4_batch()
%RUN_PX4_BATCH  px4_main 을 설정만 바꿔 여러 번 연속 실행 (명령창에서 run_px4_batch 한 번)
%
%  현재 구성: phi_eq 30 deg, v = 8 / 9 / 10 x {직진, GS, FF} + v = 10 만 {GS + 지연 보상, FF + 지연 보상} = 11회
%     kappa = g tan(30 deg) / v^2
%     1~3)  v = 8 : kappa = 0.0885 (R 11.30 m, omega 0.71 rad/s)  직진 -> GS -> FF
%     4~6)  v = 9 : kappa = 0.0699 (R 14.31 m, omega 0.63 rad/s)  직진 -> GS -> FF
%     7~11) v = 10: kappa = 0.0566 (R 17.67 m, omega 0.57 rad/s)  직진 -> GS -> FF -> GS+보상 -> FF+보상
%     (지연 보상(Eq_Delay_Comp) 실험은 v = 10 에서만, v = 8 / 9 는 보상 없음)
%  공통: 고도 계단 0.5 m, {up,down} x 4, n_laps 4 (원 36바퀴), 직진은 단계마다 120 m
%  이전 구성(v = 8, phi_eq 35 / 40 deg 9회)은 커밋 19f67a7 참고
%
%  동작
%   - 모든 실험은 원래 프리롤 시작점(원점)에서 시작하고, 끝나면 원점으로 복귀 (복귀는 최대 20 m/s, 사인 램프 가감속)
%     원 궤도: 그 자리 착륙 -> 결과 저장 -> 재이륙 -> 원점 복귀 -> 착륙·disarm (end_mode = 'land_plot_return')
%     직진   : 착륙하지 않고 공중에서 원점 복귀 -> 착륙·disarm -> 결과 저장 (end_mode = 'return_land_plot')
%              (Gazebo 바닥이 유한해서 ~1.2 km 밖에서 착륙하면 바닥을 뚫고 떨어짐)
%   - 실행 사이 pause_between 초 대기 후 다음 실험 시작
%   - 오류가 나거나 비행이 중단(abort)되면 나머지 실험은 건너뜀 (기체 상태를 확인하고 다시 실행할 것)
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

pause_between = 10;     % 실험 사이 대기 [s]

%% 공통 설정
base = struct();
base.mode              = 'MATLAB';
base.path_type         = 'circle';
base.Feedforward       = true;
base.height_step       = 0.5;
base.height_order      = {"up","down","up","down","up","down","up","down"};
base.n_laps            = 4;

%% 실험 목록 (순서대로 실행)
v8  = setf(base, 'target_speed', 8);
v9  = setf(base, 'target_speed', 9);
v10 = setf(base, 'target_speed', 10);
k30_v8  = 0.0885;    % phi_eq = atan(v^2 kappa / g) = 30.0 deg (v = 8)
k30_v9  = 0.0699;    % 30.0 deg (v = 9)
k30_v10 = 0.0566;    % 30.0 deg (v = 10)
runs = { ...
    'v8_straight',           setf(v8,  'curvature', 0.0,     'straight_distance', 120, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v8_k0.0885_GS',         setf(v8,  'curvature', k30_v8,  'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v8_k0.0885_FF',         setf(v8,  'curvature', k30_v8,  'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', false); ...
    'v9_straight',           setf(v9,  'curvature', 0.0,     'straight_distance', 120, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v9_k0.0699_GS',         setf(v9,  'curvature', k30_v9,  'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v9_k0.0699_FF',         setf(v9,  'curvature', k30_v9,  'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', false); ...
    'v10_straight',          setf(v10, 'curvature', 0.0,     'straight_distance', 120, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v10_k0.0566_GS',        setf(v10, 'curvature', k30_v10, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v10_k0.0566_FF',        setf(v10, 'curvature', k30_v10, 'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', false); ...
    'v10_k0.0566_GS_comp',   setf(v10, 'curvature', k30_v10, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', true ); ...
    'v10_k0.0566_FF_comp',   setf(v10, 'curvature', k30_v10, 'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', true ); ...
    };

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
            fprintf(2, '[%s] 비행이 중단되어 나머지 실험을 건너뜁니다.\n', name);
            break;
        end
        status{i} = '완료';
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
lines{2} = sprintf('%-3s %-22s %-20s %-20s %s', '#', '실험', '시작', '종료', '상태');
for i = 1:nRun
    lines{i + 2} = sprintf('%-3d %-22s %-20s %-20s %s', i, runs{i, 1}, ...
        fmt_time(t_start(i)), fmt_time(t_end(i)), status{i});
end
fprintf('\n%s\n', strjoin(lines, newline));

result_dir = fullfile(here, 'results');
if ~exist(result_dir, 'dir'), mkdir(result_dir); end
logFile = fullfile(result_dir, sprintf('batch_%s.txt', char(datetime(batch_time, 'Format', 'yyyyMMdd_HHmmss'))));
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
