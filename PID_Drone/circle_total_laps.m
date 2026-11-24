function n = circle_total_laps(cfg)
    % CIRCLE_TOTAL_LAPS  원 궤적 전체 바퀴 수
    %   고도 계단(cfg.height_order)이 있으면 고도 단계마다 cfg.n_laps 바퀴씩 돈다
    %   -> n = cfg.n_laps * (numel(cfg.height_order) + 1). 없으면 cfg.n_laps.
    %   CircleTrajectory 와 같은 규칙 (finalize_and_save_results 등에서 평가 구간 계산에 사용)
    n = cfg.n_laps;
    if isfield(cfg, 'height_order') && ~isempty(cfg.height_order)
        n = cfg.n_laps * (numel(CircleTrajectory.parse_height_order(cfg.height_order)) + 1);
    end
end
