classdef CircleTrajectory < handle
    % CIRCLETRAJECTORY  정지 -> 직선 가속 -> 직선 등속 -> 원 궤적 (n 바퀴) -> 직선 탈출
    %   - 시작점에서 +x 방향으로 정지 상태부터 등가속, 가속 구간 길이 L_acc = c * v
    %   - 가속 후 목표 속력 v 로 cruise_time [s] 동안 직선 등속 (L_cruise = v * cruise_time)
    %   - 초반 직선(L_str = L_acc + L_cruise) 끝에서 원에 접하도록 진입 (kappa > 0 : +y 쪽으로 선회, kappa < 0 : -y 쪽)
    %   - 원을 n_laps 바퀴 돌면 진입점으로 돌아오고, 같은 접선(+x) 방향으로 탈출
    %   - 탈출 직선: 초반 직선과 같은 길이 L_exit = L_str 를 목표 속력 v 로 등속 주행 후 종료 (tf)
    %   - 구간 시각: [0, t_acc] 가속, [t_acc, t_circ0] 등속 직선, [t_circ0, t_circ1] 원, [t_circ1, tf] 탈출 직선
    %   TrajectoryPlanner 와 같은 인터페이스:
    %     [pos, vel, kappa_ff, kappa_raw] = get_position(t), get_full_path(dt), tf, total_length
    %   kappa_ff 는 TrajectoryPlanner 와 같은 방식(전방 lookahead_dist 지점 곡률 + 1차 댐핑 kappa_tau)
    %
    %   고도 계단 (height_step, height_order, 선택)
    %   - height_order = {'up', 'up', 'down'} 처럼 주면, 고도 단계마다 n_laps 바퀴씩 돈다
    %     -> 원 전체 바퀴 수 = n_laps x (numel(height_order) + 1)
    %   - 기준 고도 h0 = -init_pos(3) (예: 초기 z = -6 -> 6 m) 에서 시작
    %   - n_laps 바퀴를 돌 때마다 (바퀴 경계 = 원 진입점) 'up' 이면 +height_step, 'down' 이면 -height_step 으로 계단 변화
    %     예: h0 = 6, step = 0.3, {'up','up','down'} -> 6.0 > 6.3 > 6.6 > 6.3 m
    %   - 마지막 고도를 유지한 채 탈출 직선을 날고 끝난다 (h0 로 돌아가지 않음)
    %   - 수평 경로·시간·곡률은 고도와 무관 (경로 길이 s 는 수평 길이 기준)
    %
    %   직선 모드 (curvature = 0)
    %   - 원 구간 자리에 직선을 난다. 고도 단계 하나당 straight_distance [m] (기본 30 m) 를 직진하고 다음 계단으로
    %     -> 직선 구간 전체 길이 = straight_distance x (numel(height_order) + 1), n_laps 는 쓰지 않음
    %   - 직진(phi_eq = 0) 기준 응답 측정용

    properties
        init_pos            % 3x1 시작점
        v                   % 목표 속력 [m/s]
        kappa               % 원 곡률 [1/m] (부호 = 선회 방향)
        R                   % 반지름 [m]
        n_laps              % 고도 단계 하나당 바퀴 수 (고도 계단이 없으면 전체 바퀴 수)
        n_laps_total        % 원 전체 바퀴 수 = n_laps x 고도 단계 수
        is_straight         % true: 직선 모드 (curvature = 0)
        straight_distance   % 직선 모드에서 고도 단계 하나당 직진 거리 [m]
        L_level             % 고도 단계 하나의 경로 길이 [m] (원: n_laps x 2 pi R, 직선: straight_distance)
        L_acc               % 가속 구간 길이 [m] = accel_dist_per_speed * v
        a_acc               % 가속도 [m/s^2] = v^2 / (2 L_acc)
        t_acc               % 가속 시간 [s] = 2 L_acc / v
        L_circle            % 원 구간 총 길이 [m] (직선 모드: 직선 구간 총 길이)
        cruise_time         % 가속 후 원 진입 전 등속 직선 시간 [s]
        L_cruise            % 등속 직선 길이 [m] = v * cruise_time
        L_str               % 초반 직선 전체 길이 [m] = L_acc + L_cruise (원 진입 지점)
        L_exit              % 탈출 직선 길이 [m] (= L_str)
        t_circ0             % 원 진입 시각 [s] (= t_acc + cruise_time)
        t_circ1             % 원 탈출 시각 [s]
        total_length        % 전체 경로 길이 [m]
        tf                  % 종료 시각 [s]

        lookahead_dist      % 전방 곡률 예견 거리 [m] ([] 이면 현재 지점 곡률)
        kappa_tau           % 전방 곡률 댐핑 시정수 [s]
        kappa_raw
        kappa_ff
        t_kappa_prev
        last_kappa

        h0                  % 기준(시작) 고도 [m] = -init_pos(3)
        height_step         % 계단 크기 [m]
        height_order        % 계단 방향 ({'up', 'down', ...}, 비어 있으면 고도 고정)
        h_levels            % 단계별 고도 [m] (1 x numel(height_order)+1)
        s_steps             % 계단이 바뀌는 경로 길이 [m]
        t_steps             % 계단이 바뀌는 시각 [s]
    end

    methods
        function obj = CircleTrajectory(init_pos, target_speed, curvature, accel_dist_per_speed, n_laps, lookahead_dist, kappa_tau, cruise_time, height_step, height_order, straight_distance)
            if nargin < 6, lookahead_dist = []; end
            if nargin < 7 || isempty(kappa_tau), kappa_tau = 0; end
            if nargin < 8 || isempty(cruise_time), cruise_time = 0; end
            if nargin < 9 || isempty(height_step), height_step = 0; end
            if nargin < 10, height_order = {}; end
            if nargin < 11 || isempty(straight_distance), straight_distance = 30; end
            obj.is_straight       = abs(curvature) < 1e-6;
            obj.straight_distance = straight_distance;
            if obj.is_straight && ~(straight_distance > 0)
                error('CircleTrajectory: 직선 모드(curvature = 0)에는 straight_distance > 0 [m] 이 필요합니다.');
            end

            obj.init_pos = init_pos(:);
            obj.v        = target_speed;
            if obj.is_straight
                obj.kappa = 0;
                obj.R     = Inf;
            else
                obj.kappa = curvature;
                obj.R     = 1 / abs(curvature);
            end
            obj.n_laps   = n_laps;

            % 고도 계단 단계
            dirs = CircleTrajectory.parse_height_order(height_order);
            if ~isempty(dirs) && ~(height_step > 0)
                error('CircleTrajectory: height_order 를 쓰려면 height_step > 0 [m] 이어야 합니다.');
            end
            obj.height_step  = height_step;
            obj.height_order = height_order;
            obj.n_laps_total = n_laps * (numel(dirs) + 1);
            obj.h0           = -obj.init_pos(3);
            obj.h_levels     = obj.h0 + [0, cumsum(dirs)] * height_step;

            obj.L_acc = accel_dist_per_speed * target_speed;          % 속도에 비례하는 가속 구간
            obj.a_acc = target_speed^2 / (2 * obj.L_acc);
            obj.t_acc = 2 * obj.L_acc / target_speed;

            if obj.is_straight
                obj.L_level = straight_distance;                        % 직선: 단계마다 straight_distance
            else
                obj.L_level = n_laps * 2 * pi * obj.R;                  % 원: 단계마다 n_laps 바퀴
            end
            obj.L_circle     = obj.L_level * (numel(dirs) + 1);
            obj.cruise_time  = cruise_time;
            obj.L_cruise     = target_speed * cruise_time;              % 목표 속력 도달 후 등속 직선
            obj.L_str        = obj.L_acc + obj.L_cruise;                % 원 진입 지점까지의 직선 길이
            obj.L_exit       = obj.L_str;                               % 초반 직선과 같은 거리만큼 탈출 직진
            obj.total_length = obj.L_str + obj.L_circle + obj.L_exit;
            obj.t_circ0      = obj.t_acc + cruise_time;
            obj.t_circ1      = obj.t_circ0 + obj.L_circle / target_speed;   % 원 진입 시각 + 원 구간 시간
            obj.tf           = obj.t_circ1 + obj.L_exit / target_speed;

            obj.lookahead_dist = lookahead_dist;
            obj.kappa_tau      = kappa_tau;
            obj.kappa_raw = 0; obj.kappa_ff = []; obj.t_kappa_prev = []; obj.last_kappa = 0;

            % 계단 위치·시각: 고도 단계마다 L_level (원 n_laps 바퀴 또는 직선 straight_distance)
            obj.s_steps = obj.L_str + (1:numel(dirs)) * obj.L_level;
            obj.t_steps = obj.t_circ0 + (1:numel(dirs)) * obj.L_level / target_speed;

            if obj.is_straight
                fprintf('[CircleTrajectory] 직선 모드: 가속 %.1f m, 등속 직선 %.1f m, 계단 구간 %.1f m (단계마다 %.1f m, %.1f ~ %.1f s), 탈출 직선 %.1f m, tf = %.1f s\n', ...
                    obj.L_acc, obj.L_cruise, obj.L_circle, obj.L_level, obj.t_circ0, obj.t_circ1, obj.L_exit, obj.tf);
            else
            fprintf('[CircleTrajectory] R = %.2f m, 가속 %.1f m (%.2f m/s^2, %.1f s), 등속 직선 %.1f m (%.1f s), 원 %d바퀴 %.1f m (%.1f ~ %.1f s), 탈출 직선 %.1f m, tf = %.1f s\n', ...
                obj.R, obj.L_acc, obj.a_acc, obj.t_acc, obj.L_cruise, cruise_time, obj.n_laps_total, obj.L_circle, obj.t_circ0, obj.t_circ1, obj.L_exit, obj.tf);
            end
            if ~isempty(dirs)
                fprintf('[CircleTrajectory] 고도 계단: %s m (단계마다 %.1f m, 계단 시각 %s s)\n', ...
                    strjoin(arrayfun(@(h) sprintf('%.2f', h), obj.h_levels, 'UniformOutput', false), ' > '), obj.L_level, ...
                    strjoin(arrayfun(@(x) sprintf('%.1f', x), obj.t_steps, 'UniformOutput', false), ', '));
            end
        end

        %% 경로 길이 s -> 기준 고도 [m] (양수 = 위)
        function h = height_at(obj, s)
            h = obj.h_levels(1 + sum(s >= obj.s_steps));
        end

        %% 시간 t -> 경로 길이 s, 속력
        function [s, spd] = arc_at_time(obj, t)
            t = max(0, min(t, obj.tf));
            if t < obj.t_acc
                s   = 0.5 * obj.a_acc * t^2;
                spd = obj.a_acc * t;
            else
                s   = obj.L_acc + obj.v * (t - obj.t_acc);
                spd = obj.v;
            end
        end

        %% 경로 길이 s -> 위치, 단위 접선, 곡률
        function [p, T, k] = pose_at(obj, s)
            s = max(0, min(s, obj.total_length));
            s_exit = obj.L_str + obj.L_circle;                          % 원 탈출 지점까지의 거리
            if obj.is_straight                                          % 직선 모드: 전 구간 +x 직진
                p = obj.init_pos + [s; 0; 0];
                T = [1; 0; 0];
                k = 0;
            elseif s <= obj.L_str                                       % 가속 + 등속 직선
                p = obj.init_pos + [s; 0; 0];
                T = [1; 0; 0];
                k = 0;
            elseif s >= s_exit
                % 탈출 직선: n 바퀴 후 진입점으로 돌아와 같은 +x 방향으로 직진
                p = obj.init_pos + [obj.L_str + (s - s_exit); 0; 0];
                T = [1; 0; 0];
                k = 0;
            else
                ang   = obj.kappa * (s - obj.L_str);                % 진입 후 회전각 (부호 포함)
                entry = obj.init_pos + [obj.L_str; 0; 0];
                ctr   = entry + [0; 1/obj.kappa; 0];                 % 원 중심 (진입점에서 접선에 수직)
                r0    = [0; -1/obj.kappa];                           % 중심 -> 진입점
                Rot   = [cos(ang) -sin(ang); sin(ang) cos(ang)];
                p = ctr + [Rot * r0; 0];
                T = [cos(ang); sin(ang); 0];
                k = obj.kappa;
            end
            p(3) = -obj.height_at(s);                                   % NED: z = -고도
        end

        function k = curvature_at(obj, s)
            [~, ~, k] = obj.pose_at(s);
        end

        %% TrajectoryPlanner 와 같은 출력
        function [pos_des, vel_des, kappa_out, kappa_raw] = get_position(obj, t_in)
            [s, spd] = obj.arc_at_time(t_in);
            [pos_des, T] = obj.pose_at(s);
            vel_des = spd * T;

            if t_in >= obj.tf
                kappa_out = obj.last_kappa; kappa_raw = obj.kappa_raw;
                return;
            end

            % 전방 곡률 (lookahead) + 1차 댐핑
            if isempty(obj.lookahead_dist)
                kappa_raw = obj.curvature_at(s);
            else
                kappa_raw = obj.curvature_at(min(s + obj.lookahead_dist, obj.total_length));
            end
            obj.kappa_raw = kappa_raw;
            if isempty(obj.kappa_ff) || obj.kappa_tau <= 0
                obj.kappa_ff = kappa_raw;
            else
                dt_k = t_in - obj.t_kappa_prev;
                if dt_k > 0
                    alpha = dt_k / (obj.kappa_tau + dt_k);
                    obj.kappa_ff = obj.kappa_ff + alpha * (kappa_raw - obj.kappa_ff);
                end
            end
            obj.t_kappa_prev = t_in;
            kappa_out = obj.kappa_ff;
            obj.last_kappa = kappa_out;
        end

        %% 경로 그리기용 (필터 상태는 건드리지 않음)
        function [X, Y, Z] = get_full_path(obj, dt_sample)
            tv = 0:dt_sample:obj.tf;
            P = zeros(3, numel(tv));
            for i = 1:numel(tv)
                [s, ~] = obj.arc_at_time(tv(i));
                P(:, i) = obj.pose_at(s);
            end
            X = P(1, :); Y = P(2, :); Z = P(3, :);
        end
    end

    methods (Static)
        function dirs = parse_height_order(order)
            % {'up','down'} 또는 ["up","down"] -> [+1, -1]
            if isempty(order), dirs = zeros(1, 0); return; end
            if ischar(order)
                error(['CircleTrajectory: height_order 는 셀 배열 {''up'', ''down''} 이나 문자열 배열 ["up", "down"] 으로 주세요. ' ...
                       '([''up'', ''down''] 처럼 대괄호에 문자를 넣으면 ''updown'' 하나로 붙어 버립니다.)']);
            end
            order = cellstr(order);
            dirs = zeros(1, numel(order));
            for i = 1:numel(order)
                switch lower(strtrim(order{i}))
                    case 'up',   dirs(i) = +1;
                    case 'down', dirs(i) = -1;
                    otherwise
                        error('CircleTrajectory: height_order 의 ''%s'' 는 ''up'' 또는 ''down'' 이어야 합니다.', order{i});
                end
            end
        end
    end
end
