function [seg_r, seg_theta, seg_z] = pathGen(is_random, max_kappa)
    % PATHGEN 직선(Straight Line)과 고정 곡률 원호(Circular Arc: r, theta) 조합 경로 생성
    %
    % 출력 규격 (각 열이 하나의 구간(Segment)을 의미):
    %   - seg_r     : 구간의 부호 있는 곡률반경 r [m]
    %                 * 0 (또는 Inf) : 직선 구간 (곡률 kappa = 0)
    %                 * +r (r > 0)   : 좌선회/양(+)의 곡률 원호 구간 (고정 곡률 kappa = +1/r)
    %                 * -r (r < 0)   : 우선회/음(-)의 곡률 원호 구간 (고정 곡률 kappa = -1/|r|)
    %   - seg_theta : 구간 길이 파라미터
    %                 * 직선 구간 (seg_r == 0) : 직선 거리 L [m]
    %                 * 원호 구간 (seg_r ~= 0) : 원호의 중심 회전각 theta [rad] (호 길이 s = |r| * theta)
    %   - seg_z     : 목표 고도 z [m] (기본 -6.0 m)

    if nargin < 1 || isempty(is_random), is_random = false; end
    if nargin < 2 || isempty(max_kappa), max_kappa = 0.50;  end

    if ~is_random
        %% [고정 벤치마크 경로: 점진적 고곡률 헤어핀(반원, theta=pi) 챌린지 코스]
        % 구간 구성:
        %  1. 직선 30m
        %  2. 지름 30m (r = +15.0m, kappa = 0.067) 반원 180도 (pi rad)
        %  3. 직선 30m
        %  4. 지름 20m (r = -10.0m, kappa = -0.100) 반원 180도 (pi rad)
        %  5. 직선 30m
        %  6. 지름 15m (r = +7.5m,  kappa = 0.133) 반원 180도 (pi rad)
        %  7. 직선 30m
        %  8. 지름 10m (r = -5.0m,  kappa = -0.200) 반원 180도 (pi rad)
        %  9. 직선 30m
        % 10. 지름 5m  (r = +2.5m,  kappa = 0.400) 반원 180도 (pi rad)
        % 11. 직선 30m
        % 12. 지름 2.5m(r = -1.25m, kappa = -0.800) 극한 반원 180도 (pi rad)
        % 13. 직선 30m
        
        seg_r = [  0.0,  15.0,   0.0, -10.0,   0.0,   7.5,   0.0,  -5.0,   0.0,   2.5,   0.0,  -1.25,   0.0];
        seg_theta = [30.0,    pi,  30.0,    pi,  30.0,    pi,  30.0,    pi,  30.0,    pi,  30.0,     pi,  30.0];
        seg_z = -6.0 * ones(1, length(seg_r));
        return;
    end

    %% [강화학습 일반화 훈련용 랜덤 경로]
    % 요구사항 반영:
    % 1) 원호 구간 개수를 5~8개로 상향 (총 비행 거리 200m ~ 350m 수준으로 확장)
    % 2) 고난도 지름 5m (r=2.5m) 및 지름 2.5m (r=1.25m) 헤어핀(반원, theta=pi) 발생 확률 대폭 강화 (40% 확률로 직접 등장)
    num_arcs = randi([5, 8]);

    seg_r_list     = [];
    seg_theta_list = [];

    % 1) 초기 가속 및 안정화를 위한 직선 구간 (15 ~ 25 m, kappa = 0)
    L0 = 15.0 + 10.0 * rand();
    seg_r_list(end+1)     = 0.0;
    seg_theta_list(end+1) = L0;

    prev_dir = 2 * randi([0, 1]) - 1;

    for k = 1:num_arcs
        % 회전 방향 결정 (S-커브 반전이 자주 일어나도록 70% 확률로 이전 방향 반전)
        if rand() < 0.70
            turn_dir = -prev_dir;
        else
            turn_dir = prev_dir;
        end
        prev_dir = turn_dir;

        % --- [핵심] 고난도 헤어핀 샘플링 가중치 부여 ---
        rand_type = rand();
        if rand_type < 0.25
            % 고난도 타입 A: 지름 5m 헤어핀 (r = 2.5m, kappa = 0.40, 반원 180도)
            r_mag     = 2.5;
            theta_arc = pi;
        elseif rand_type < 0.40
            % 고난도 타입 B: 극한 지름 2.5m 헤어핀 (r = 1.25m, kappa = 0.80, 반원 180도)
            r_mag     = 1.25;
            theta_arc = pi;
        else
            % 일반 무작위 곡률: 0.08 ~ max_kappa (r = 1 / kappa), 60도 ~ 180도 선회
            kappa_mag = 0.08 + max(0.02, max_kappa - 0.08) * rand();
            r_mag     = 1.0 / kappa_mag;
            theta_arc = deg2rad(60.0 + 120.0 * rand());
        end

        % 원호 구간 추가: 부호 있는 반경 r = turn_dir * r_mag, 회전각 theta_arc
        seg_r_list(end+1)     = turn_dir * r_mag;
        seg_theta_list(end+1) = theta_arc;

        % 원호 사이에 중간 직선 구간 (10 ~ 25 m, kappa = 0) 삽입
        if k < num_arcs
            L_mid = 10.0 + 15.0 * rand();
            seg_r_list(end+1)     = 0.0;
            seg_theta_list(end+1) = L_mid;
        end
    end

    % 마지막 직선 안정화 구간 (15 ~ 25 m, kappa = 0)
    L_end = 15.0 + 10.0 * rand();
    seg_r_list(end+1)     = 0.0;
    seg_theta_list(end+1) = L_end;

    seg_r     = seg_r_list;
    seg_theta = seg_theta_list;
    seg_z     = -6.0 * ones(1, length(seg_r));
end
