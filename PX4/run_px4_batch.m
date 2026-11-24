function run_px4_batch()
%RUN_PX4_BATCH  px4_main 을 설정만 바꿔 여러 번 연속 실행 (명령창에서 run_px4_batch 한 번)
%
%  현재 구성: v = 8, 직진 1회 + phi_eq 35 / 40 deg x {GS, FF, GS + 지연 보상, FF + 지연 보상} = 9회
%     1) 직진 (curvature 0, 단계마다 straight_distance)
%     2~5) phi_eq 35 deg: kappa = 0.107 (R 9.35 m, omega 0.86 rad/s)  GS -> FF -> GS+보상 -> FF+보상
%     6~9) phi_eq 40 deg: kappa = 0.129 (R 7.75 m, omega 1.03 rad/s)  GS -> FF -> GS+보상 -> FF+보상
%  공통: 고도 계단 0.5 m, {up,down} x 3, n_laps 3 (원 21바퀴)
%  (40 deg GS 는 상승 계단에서 추력 포화가 약간 날 수 있음. 깔끔한 비교가 필요하면 height_step 0.4 로)
%
%  동작
%   - 각 실행은 px4_main 과 똑같이 ①~⑤ -> 착륙·disarm -> ULog 결과 저장 -> 재이륙·프리롤 시작점 복귀 -> 착륙·disarm
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
base.height_order      = {"up","down","up","down","up","down"};
base.n_laps            = 3;

%% 실험 목록 (순서대로 실행)
v8 = setf(base, 'target_speed', 8);
k35 = 0.107;     % phi_eq = atan(v^2 kappa / g) = 34.9 deg
k40 = 0.129;     % 40.1 deg
runs = { ...
    'v8_straight',        setf(v8, 'curvature', 0.0, 'straight_distance', 120, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v8_k0.107_GS',       setf(v8, 'curvature', k35, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v8_k0.107_FF',       setf(v8, 'curvature', k35, 'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', false); ...
    'v8_k0.107_GS_comp',  setf(v8, 'curvature', k35, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', true ); ...
    'v8_k0.107_FF_comp',  setf(v8, 'curvature', k35, 'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', true ); ...
    'v8_k0.129_GS',       setf(v8, 'curvature', k40, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', false); ...
    'v8_k0.129_FF',       setf(v8, 'curvature', k40, 'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', false); ...
    'v8_k0.129_GS_comp',  setf(v8, 'curvature', k40, 'Z_Gain_Scheduling', true,  'Eq_Delay_Comp', true ); ...
    'v8_k0.129_FF_comp',  setf(v8, 'curvature', k40, 'Z_Gain_Scheduling', false, 'Eq_Delay_Comp', true ); ...
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
    fprintf('\n===== [%d/%d] %s =====\n', i, nRun, name);
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
end

function s = fmt_time(t)
if isnat(t), s = '-'; else, s = char(datetime(t, 'Format', 'HH:mm:ss')); end
end
