function cfg = apply_cfg_override(cfg, ov)
%APPLY_CFG_OVERRIDE  cfg 구조체의 필드를 ov 의 값으로 덮어쓴다 (중첩 구조체는 재귀 병합).
%   cfg = apply_cfg_override(cfg, ov)
%   - ov.px4.end_mode 처럼 중첩 필드는 해당 필드만 바꾸고 나머지 px4.* 는 유지
%   - cfg 에 없는 필드를 덮어쓰면 오타일 수 있으므로 경고 후 그대로 추가
%   run_px4_batch 가 setappdata(0, 'px4_cfg_override', ov) 로 넘긴 값을 px4_main 이 적용할 때 사용

f = fieldnames(ov);
for i = 1:numel(f)
    k = f{i};
    if ~isfield(cfg, k)
        warning('apply_cfg_override: cfg 에 없는 필드 "%s" 를 추가합니다 (오타 확인).', k);
        cfg.(k) = ov.(k);
    elseif isstruct(ov.(k)) && isstruct(cfg.(k))
        cfg.(k) = apply_cfg_override(cfg.(k), ov.(k));
    else
        cfg.(k) = ov.(k);
    end
end
end
