function R = circle_ref_radius(cfg)
    % CIRCLE_REF_RADIUS  원 궤적 길이·시각 계산용 반지름 [m]
    %   cfg.curvature ~= 0 : 1 / |cfg.curvature|
    %   cfg.curvature  = 0 : 직선 모드 -> 1 / cfg.straight_ref_curvature (기본 0.1)
    %   CircleTrajectory 와 같은 규칙 (finalize_and_save_results, analyze_height_steps 에서 사용)
    if abs(cfg.curvature) < 1e-6
        if isfield(cfg, 'straight_ref_curvature') && ~isempty(cfg.straight_ref_curvature)
            R = 1 / abs(cfg.straight_ref_curvature);
        else
            R = 1 / 0.1;
        end
    else
        R = 1 / abs(cfg.curvature);
    end
end
