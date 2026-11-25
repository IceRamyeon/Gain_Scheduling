function replot_px4_result(metaFiles, ulgFiles, varargin)
%REPLOT_PX4_RESULT  저장된 비행 기록(px4_run_*.mat)과 ULog(.ulg)로 결과 그래프·Summary 를 다시 만든다.
%
%  입력 파일 두 종류
%   1) 결과 파일 DroneSimResult_*.mat (log_data + cfg): ULog 없이 그대로 다시 그림
%   2) 비행 기록 px4_run_*.mat (meta): ULog 를 찾아 log_data 를 새로 만든 뒤 그림
%  파일 이름만 넣어도 됨 (예: 'DroneSimResult_20261125_155529.mat') -> PX4/results 아래 전체에서 찾음
%
%  사용법
%   replot_px4_result('DroneSimResult_20261125_155529.mat')   % 결과 파일 이름만 복사·붙여넣기
%   replot_px4_result                                  % 파일 선택 창 (여러 개 선택 가능)
%   replot_px4_result('results/px4_run_20261125_123001_MATLAB.mat')
%   replot_px4_result('results/px4_run_20261125_15*.mat')          % 와일드카드: 해당 파일 전부
%   replot_px4_result({'a.mat', 'b.mat'})                         % 여러 개
%   replot_px4_result('a.mat', '~/PX4-Autopilot/.../03_10_20.ulg')  % ULog 직접 지정
%   replot_px4_result(..., 'ulogRoot', '<ULog 폴더>', 'saveRoot', '<저장 폴더>', 'closeFigs', true)
%
%  ULog 자동 매칭 (ulgFiles 를 비우면)
%   - ulogRoot 아래 .ulg 중, 수정 시각이 비행 기록 저장 시각과 가까운 순서로 후보를 보고
%   - ULog 의 시간 범위(StartTime ~ EndTime, PX4 부팅 후 시간)가 비행 기록의 ④ 구간(t4_start ~ t4_end)을
%     포함하는 첫 번째 파일을 고른다 (land_plot_return 의 복귀 비행 ULog 를 잘못 고르지 않도록)
%   - 맞는 ULog 가 없으면 MATLAB 비행 기록(meta.log)만으로 그린다 (U2~U4 없음)
%
%  저장 위치: <saveRoot>/<비행 기록 이름>/DroneSimResult_<비행 기록 시각>_replot_*  (기본 saveRoot = PX4/results/plots/replot)
%   - 같은 비행을 다시 그리면 같은 파일에 덮어씀
%   - 비행 기록 시각은 착륙 전 저장 시각이라, 비행 직후 만들어진 원래 Summary 이름의 시각과 수십 초 다를 수 있음

here = fileparts(mfilename('fullpath'));
addpath(here);

p = inputParser;
p.addParameter('ulogRoot', fullfile(getenv('HOME'), 'PX4-Autopilot', 'build', 'px4_sitl_default', 'rootfs', 'fs', 'log'));
p.addParameter('saveRoot', fullfile(here, 'results', 'plots', 'replot'));
p.addParameter('closeFigs', []);          % [] 이면 2개 이상일 때 그래프 저장 후 창 닫음
p.addParameter('save', true);             % false: 그래프만 띄우고 저장하지 않음
p.parse(varargin{:});
opt = p.Results;

%% 비행 기록 목록
if nargin < 1 || isempty(metaFiles)
    [f, d] = uigetfile({'px4_run_*.mat;DroneSimResult_*.mat', '비행 기록 / 결과 파일'}, '파일 선택', ...
        fullfile(here, 'results'), 'MultiSelect', 'on');
    if isequal(f, 0), disp('선택 취소'); return; end
    metaFiles = fullfile(d, cellstr(f));
end
metaFiles = expand_files(metaFiles, here);
if isempty(metaFiles), error('비행 기록 파일을 찾지 못했습니다.'); end

if nargin < 2 || isempty(ulgFiles)
    ulgFiles = repmat({''}, size(metaFiles));
else
    ulgFiles = cellstr(ulgFiles);
    if numel(ulgFiles) ~= numel(metaFiles)
        error('ulgFiles 개수(%d)가 비행 기록 개수(%d)와 다릅니다.', numel(ulgFiles), numel(metaFiles));
    end
end
closeFigs = opt.closeFigs;
if isempty(closeFigs), closeFigs = numel(metaFiles) > 1; end

%% 하나씩 다시 그리기
nOk = 0;
for i = 1:numel(metaFiles)
    mf = metaFiles{i};
    [~, mname] = fileparts(mf);
    fprintf('\n===== [%d/%d] %s =====\n', i, numel(metaFiles), mname);
    try
        S = load(mf);
        if isfield(S, 'log_data') && isfield(S, 'cfg')
            % 결과 파일(DroneSimResult_*.mat): 저장된 log_data 로 바로 다시 그림
            cfg = S.cfg;
            cfg.auto_save = opt.save;
            cfg.save_dir  = fullfile(opt.saveRoot, mname);
            cfg.result_name = [mname, '_replot'];
            if opt.save && ~exist(cfg.save_dir, 'dir'), mkdir(cfg.save_dir); end
            finalize_and_save_results(S.log_data, cfg);
            if opt.save, fprintf('저장: %s\n', cfg.save_dir); end
            nOk = nOk + 1;
            if closeFigs, close all; end
            continue;
        elseif ~isfield(S, 'meta')
            error('log_data/cfg(결과 파일)도 meta(비행 기록)도 없는 .mat 입니다.');
        end
        meta = S.meta;
        if meta.aborted
            warning('중단된 비행입니다: %s', meta.abort_reason);
        end

        uf = ulgFiles{i};
        if isempty(uf)
            uf = match_ulog(opt.ulogRoot, mf, meta);
        end

        if ~isempty(uf)
            fprintf('ULog: %s\n', uf);
            log_data = ulog_to_log_data(uf, meta, 0.01);
            disp('※ U2~U4 는 PX4 정규화 토크(-1 ~ 1), U1 은 정규화 추력을 N 으로 근사 변환한 값입니다.');
        else
            warning('맞는 ULog 가 없어 MATLAB 비행 기록(meta.log)으로 그립니다 (U1 = 추력 명령, U2~U4 없음).');
            log_data = meta_to_log_data(meta, 0.02);
        end

        cfg = meta.cfg;
        cfg.save_dir  = fullfile(opt.saveRoot, mname);
        cfg.auto_save = opt.save;
        tok = regexp(mname, 'px4_run_(\d{8}_\d{6})', 'tokens', 'once');   % 원래 비행 시각으로 파일 이름
        if ~isempty(tok), cfg.result_name = sprintf('DroneSimResult_%s_replot', tok{1}); end
        if opt.save && ~exist(cfg.save_dir, 'dir'), mkdir(cfg.save_dir); end
        finalize_and_save_results(log_data, cfg);
        if opt.save, fprintf('저장: %s\n', cfg.save_dir); end
        nOk = nOk + 1;
        if closeFigs, close all; end
    catch err
        fprintf(2, '[%s] 실패: %s\n', mname, err.message);
    end
end
fprintf('\n완료: %d / %d 개\n', nOk, numel(metaFiles));
end

%% ---- 로컬 함수 ----
function files = expand_files(in, here)
% 문자열·셀·와일드카드·폴더를 파일 경로 셀로
in = cellstr(in);
files = {};
for k = 1:numel(in)
    s = char(in{k});
    if ~isfile(s) && isfile(fullfile(here, s)), s = fullfile(here, s); end
    if isfolder(s), s = fullfile(s, 'px4_run_*.mat'); end
    if contains(s, '*')
        d = dir(s);
        if isempty(d) && ~startsWith(s, filesep), d = dir(fullfile(here, s)); end
        d = d(~[d.isdir]);
        [~, ord] = sort({d.name});
        d = d(ord);
        files = [files, fullfile({d.folder}, {d.name})];       %#ok<AGROW>
    elseif isfile(s)
        files{end + 1} = s;                                     %#ok<AGROW>
    else
        % 이름만 준 경우: PX4/results 아래 전체(하위 폴더 포함)에서 찾기
        [~, nm, ext] = fileparts(s);
        if isempty(ext), ext = '.mat'; end
        d = dir(fullfile(here, 'results', '**', [nm, ext]));
        d = d(~contains({d.folder}, fullfile('plots', 'replot')));   % 다시 그린 사본은 제외
        if isempty(d)
            warning('파일이 없습니다 (PX4/results 아래에서도 못 찾음): %s', s);
        else
            if numel(d) > 1
                warning('%s 가 %d 개 있어 가장 최근 것을 씁니다.', [nm, ext], numel(d));
                [~, j] = max([d.datenum]); d = d(j);
            end
            files{end + 1} = fullfile(d.folder, d.name);        %#ok<AGROW>
            fprintf('찾음: %s\n', files{end});
        end
    end
end
end

function uf = match_ulog(root, metaFile, meta)
% 수정 시각이 가까운 순서로 ULog 를 보면서, ④ 구간 시간(부팅 후 시간)을 포함하는 파일을 고른다
uf = '';
if ~isfolder(root)
    warning('ULog 폴더가 없습니다: %s (''ulogRoot'' 옵션으로 지정)', root);
    return;
end
d = dir(fullfile(root, '**', '*.ulg'));
if isempty(d), warning('%s 아래에 .ulg 가 없습니다.', root); return; end
m = dir(metaFile);
gap = abs([d.datenum] - m.datenum) * 24 * 60;            % [분]
[gap, ord] = sort(gap);
t4 = [meta.t4_start, meta.t4_end];
for k = 1:min(numel(ord), 8)                             % 가까운 8개까지만 확인
    if gap(k) > 60, break; end                           % 1시간 넘게 떨어진 파일은 보지 않음
    f = fullfile(d(ord(k)).folder, d(ord(k)).name);
    if any(isnan(t4))
        uf = f; return;                                  % ④ 시간이 없으면 가장 가까운 파일
    end
    try
        u = ulogreader(f);
        ts = seconds(u.StartTime); te = seconds(u.EndTime);
        if ts <= t4(1) + 0.5 && te >= t4(2) - 0.5
            uf = f; return;
        end
    catch
        % 읽을 수 없는 파일(기록 중이거나 손상)은 건너뜀
    end
end
end
