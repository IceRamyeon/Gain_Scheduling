function [x, y, z] = pathGen(is_random, max_kappa)
    % PATHGEN 웨이포인트(x, y, z) 궤적 생성
    % is_random == false 또는 인자가 없으면 기존 기준 궤적(Path 2) 반환
    % is_random == true 이면 다양한 곡률(kappa) 및 좌/우 선회가 포함된 랜덤 궤적 생성
    
    if nargin < 1, is_random = false; end
    if nargin < 2, max_kappa = 0.35; end
    
    if ~is_random
        %% Path 2 : custom path with curvature (기존 고정 궤적, 평가/비교용)
        % 총 40m 길이, 각 10m 구간마다 일정한 양의 곡률(좌회전) 적용
        x = [10.00, 30.00, 39.59, 44.77, 39.06, 33.56];
        y = [ 0.00,  0.00,  2.45, 10.52, 17.59, 11.22];
        z = [-6.00, -6.00, -6.00, -6.00, -6.00, -6.00];
        return;
    end
    
    %% 랜덤 곡률 궤적 생성 (강화학습 전 속도/곡률 일반화 훈련용)
    curr_x = 10.0;
    curr_y = 0.0;
    curr_heading = 0.0; % +x축 방향 (라디안)
    
    num_segments = randi([4, 6]);
    x = zeros(1, num_segments + 2);
    y = zeros(1, num_segments + 2);
    z = -6.0 * ones(1, num_segments + 2);
    
    x(1) = curr_x;
    y(1) = curr_y;
    
    % 0번 구간: 초기 가속 및 안정화를 위한 직선 구간 (길이 15~20m)
    seg0_len = 15.0 + 5.0 * rand();
    curr_x = curr_x + seg0_len;
    x(2) = curr_x;
    y(2) = curr_y;
    
    % 1 ~ num_segments 구간: 다양한 곡률(kappa)과 회전 방향의 원호 구간 연결
    for seg = 1:num_segments
        ds = 8.0 + 6.0 * rand();                           % 구간 길이 (8m ~ 14m)
        kappa = 0.05 + (max_kappa - 0.05) * rand();       % 곡률 (0.05 ~ max_kappa)
        turn_dir = (2 * randi([0, 1]) - 1);                % 좌회전(+1) 또는 우회전(-1)
        
        d_theta = turn_dir * kappa * ds;
        % 연속 선회 시 너무 가파르게 꺾이지 않도록 최대 80도 제한
        max_d_theta = deg2rad(80);
        if abs(d_theta) > max_d_theta
            d_theta = sign(d_theta) * max_d_theta;
            kappa = abs(d_theta) / ds;
        end
        
        % 로컬 좌표계 변위 계산 (원호 적분 기하학)
        dx_loc = (1 / kappa) * sin(abs(d_theta));
        dy_loc = turn_dir * (1 / kappa) * (1 - cos(abs(d_theta)));
        
        % 현재 헤딩각(curr_heading) 기준으로 글로벌 2D 회전 변환
        R_mat = [cos(curr_heading), -sin(curr_heading);
                 sin(curr_heading),  cos(curr_heading)];
        d_glob = R_mat * [dx_loc; dy_loc];
        
        curr_x = curr_x + d_glob(1);
        curr_y = curr_y + d_glob(2);
        curr_heading = curr_heading + d_theta;
        
        x(seg + 2) = curr_x;
        y(seg + 2) = curr_y;
    end
end