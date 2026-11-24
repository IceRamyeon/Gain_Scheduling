function [L_level, L_total, n_levels] = circle_level_length(cfg)
    % CIRCLE_LEVEL_LENGTH  CircleTrajectory 의 원(또는 직선) 구간 길이 [m]
    %   L_level  : 고도 단계 하나의 길이 (원: n_laps x 2 pi / |kappa|, 직선 모드(curvature = 0): straight_distance, 기본 30 m)
    %   L_total  : 원(직선) 구간 전체 길이 = L_level x n_levels
    %   n_levels : 고도 단계 수 = numel(height_order) + 1 (계단 없으면 1)
    %   CircleTrajectory 와 같은 규칙 (finalize_and_save_results, analyze_height_steps 에서 사용)
    n_levels = 1;
    if isfield(cfg, 'height_order') && ~isempty(cfg.height_order)
        n_levels = numel(CircleTrajectory.parse_height_order(cfg.height_order)) + 1;
    end
    if abs(cfg.curvature) < 1e-6
        if isfield(cfg, 'straight_distance') && ~isempty(cfg.straight_distance)
            L_level = cfg.straight_distance;
        else
            L_level = 30;
        end
    else
        L_level = cfg.n_laps * 2 * pi / abs(cfg.curvature);
    end
    L_total = L_level * n_levels;
end
