classdef TrajectoryPlanner_LPV < handle
    % TRAJECTORYPLANNER 직선(Straight Line, kappa = 0)과 고정 곡률 원호(Circular Arc: r, theta, kappa = ±1/r)
    % 조합으로 구성된 G1 연속 등속 궤적 생성기
    % - 각 구간 내부에서 곡률 kappa가 완벽하게 상수로 고정됨 (스플라인 진동 및 평균 왜곡 제거)
    % - 현재 곡률 kappa_now 및 dt_lookahead 이후의 미래 곡률 kappa_future를 정확히 반환

    properties
        waypoints       % 입력받은 세그먼트 정의 행렬 [seg_r; seg_theta; seg_z]
        segments        % 계산된 구간별 기하학 구조체 배열
        num_segments
        cum_lengths     % 구간 누적 호 길이 [0, s1, s1+s2, ...]
        total_length    % 전체 경로 총 길이 [m]
        speed           % 목표 등속 비행 속력 [m/s]
        tf              % 총 필요 비행 시간 [s]

        lookahead_dt    % 미래 곡률 조회용 기본 예견 시간 [s] (기본 0.5 s)
        last_kappa
    end

    methods
        function obj = TrajectoryPlanner_LPV(init_pos, seg_spec_in, target_speed, lookahead_dt)
            if nargin < 4 || isempty(lookahead_dt)
                lookahead_dt = 0.5; % 기본 0.5초 후의 곡률 예견
            end
            obj.lookahead_dt = lookahead_dt;
            obj.waypoints    = seg_spec_in;
            obj.speed        = target_speed;

            % 시작 위치 및 초기 헤딩 (+x축 방향: psi = 0)
            curr_x   = init_pos(1);
            curr_y   = init_pos(2);
            curr_z   = init_pos(3);
            curr_psi = 0.0;

            num_segs = size(seg_spec_in, 2);
            obj.num_segments = num_segs;
            obj.cum_lengths  = zeros(1, num_segs + 1);

            segs = struct('type', {}, 'x0', {}, 'y0', {}, 'z0', {}, 'psi0', {}, ...
                          'r', {}, 'theta', {}, 'dir', {}, 'L', {}, 'kappa', {});

            for k = 1:num_segs
                r_val  = seg_spec_in(1, k);
                th_val = seg_spec_in(2, k);
                if size(seg_spec_in, 1) >= 3
                    z_val = seg_spec_in(3, k);
                else
                    z_val = curr_z;
                end

                seg = struct();
                seg.x0   = curr_x;
                seg.y0   = curr_y;
                seg.z0   = z_val;
                seg.psi0 = curr_psi;

                if abs(r_val) < 1e-6 || isinf(r_val)
                    % [직선 구간] kappa = 0, 길이 L = th_val
                    seg.type  = 'line';
                    seg.r     = Inf;
                    seg.theta = 0.0;
                    seg.dir   = 0;
                    seg.L     = max(0.0, th_val);
                    seg.kappa = 0.0;

                    % 구간 끝점 갱신
                    curr_x = curr_x + seg.L * cos(curr_psi);
                    curr_y = curr_y + seg.L * sin(curr_psi);
                else
                    % [고정 곡률 원호 구간] 반경 r = |r_val|, 회전각 theta = th_val, 고정 곡률 kappa = 1 / r_val
                    seg.type  = 'arc';
                    seg.r     = abs(r_val);
                    seg.theta = abs(th_val);
                    seg.dir   = sign(r_val);        % +1: 양(+)의 곡률 선회, -1: 음(-)의 곡률 선회
                    seg.L     = seg.r * seg.theta;  % 호의 길이 s = r * theta
                    seg.kappa = 1.0 / r_val;        % 구간 내 완전 고정된 곡률 kappa

                    % 로컬 원호 끝점 변위 -> 글로벌 회전 변환
                    dx_loc = seg.r * sin(seg.theta);
                    dy_loc = seg.dir * seg.r * (1.0 - cos(seg.theta));

                    curr_x   = curr_x + cos(curr_psi) * dx_loc - sin(curr_psi) * dy_loc;
                    curr_y   = curr_y + sin(curr_psi) * dx_loc + cos(curr_psi) * dy_loc;
                    curr_psi = curr_psi + seg.dir * seg.theta;
                end
                curr_z = z_val;

                segs(k) = seg;
                obj.cum_lengths(k + 1) = obj.cum_lengths(k) + seg.L;
            end

            obj.segments     = segs;
            obj.total_length = obj.cum_lengths(end);
            obj.tf           = obj.total_length / max(obj.speed, 1e-3);
            obj.last_kappa   = 0.0;

            disp(['직선+원호(Arc) 궤적 총 길이: ', num2str(obj.total_length, '%.2f'), 'm']);
            disp(['비행 속력: ', num2str(obj.speed, '%.2f'), 'm/s, 필요 비행 시간(tf): ', num2str(obj.tf, '%.2f'), '초']);
        end

        function [pos_des, vel_des, kappa_now, kappa_future, acc_des] = get_position(obj, t_in, dt_lookahead)
            if nargin < 3 || isempty(dt_lookahead)
                dt_lookahead = obj.lookahead_dt;
            end

            is_finished = (t_in >= obj.tf);
            t = max(0.0, min(t_in, obj.tf));
            s_now = obj.speed * t;

            % 현재 거리 s_now에서의 위치, 접선 방향, 고정 곡률 산출
            [pos_des, psi_now, kappa_now] = obj.eval_at_arclength(s_now);

            if is_finished
                vel_des      = [0; 0; 0];
                acc_des      = [0; 0; 0];
                kappa_now    = 0.0;
                kappa_future = 0.0;
            else
                vel_des = [obj.speed * cos(psi_now); obj.speed * sin(psi_now); 0.0];
                % 구심 가속도 벡터: a_c = v^2 * kappa * [-sin(psi); cos(psi); 0]
                acc_des = (obj.speed^2 * kappa_now) * [-sin(psi_now); cos(psi_now); 0.0];

                % t + dt_lookahead 이후의 미래 곡률 조회
                t_fut = min(t_in + dt_lookahead, obj.tf);
                s_fut = obj.speed * max(0.0, t_fut);
                [~, ~, kappa_future] = obj.eval_at_arclength(s_fut);
            end

            obj.last_kappa = kappa_now;
        end

        function kappa_val = get_curvature_at_time(obj, t_query)
            if t_query >= obj.tf
                kappa_val = 0.0;
                return;
            end
            s_q = obj.speed * max(0.0, min(t_query, obj.tf));
            [~, ~, kappa_val] = obj.eval_at_arclength(s_q);
        end

        function [pos, psi, kappa] = eval_at_arclength(obj, s_in)
            s = max(0.0, min(s_in, obj.total_length));

            % 해당 호 길이 s가 속한 구간 탐색
            seg_idx = find(s <= obj.cum_lengths(2:end), 1, 'first');
            if isempty(seg_idx)
                seg_idx = obj.num_segments;
            end

            seg = obj.segments(seg_idx);
            ds  = max(0.0, s - obj.cum_lengths(seg_idx));

            if strcmp(seg.type, 'line')
                px    = seg.x0 + ds * cos(seg.psi0);
                py    = seg.y0 + ds * sin(seg.psi0);
                psi   = seg.psi0;
                kappa = 0.0;
            else
                alpha_loc = ds / seg.r; % 원호 상의 진행 각도 [rad]
                dx_loc    = seg.r * sin(alpha_loc);
                dy_loc    = seg.dir * seg.r * (1.0 - cos(alpha_loc));

                px    = seg.x0 + cos(seg.psi0) * dx_loc - sin(seg.psi0) * dy_loc;
                py    = seg.y0 + sin(seg.psi0) * dx_loc + cos(seg.psi0) * dy_loc;
                psi   = seg.psi0 + seg.dir * alpha_loc;
                kappa = seg.kappa; % 원호 구간 내 완벽히 고정된 상수 곡률 (±1/r)
            end

            pos = [px; py; seg.z0];
        end

        function [X, Y, Z] = get_full_path(obj, dt_sample)
            time_vec = 0:dt_sample:obj.tf;
            pts = zeros(3, length(time_vec));
            for i = 1:length(time_vec)
                [pts(:, i), ~, ~] = obj.get_position(time_vec(i));
            end
            X = pts(1, :);
            Y = pts(2, :);
            Z = pts(3, :);
        end
    end
end
