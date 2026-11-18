classdef CircleTrajectory < handle
    % CIRCLETRAJECTORY  정지 -> 직선 가속 -> 원 궤적 (n 바퀴)
    %   - 시작점에서 +x 방향으로 정지 상태부터 등가속, 가속 구간 길이 L_acc = c * v
    %   - 가속이 끝나는 점에서 원에 접하도록 진입 (kappa > 0 : +y 쪽으로 선회, kappa < 0 : -y 쪽)
    %   - 원을 n_laps 바퀴 돌면 종료 (tf)
    %   TrajectoryPlanner 와 같은 인터페이스:
    %     [pos, vel, kappa_ff, kappa_raw] = get_position(t), get_full_path(dt), tf, total_length
    %   kappa_ff 는 TrajectoryPlanner 와 같은 방식(전방 lookahead_dist 지점 곡률 + 1차 댐핑 kappa_tau)

    properties
        init_pos            % 3x1 시작점
        v                   % 목표 속력 [m/s]
        kappa               % 원 곡률 [1/m] (부호 = 선회 방향)
        R                   % 반지름 [m]
        n_laps              % 바퀴 수
        L_acc               % 가속 구간 길이 [m] = accel_dist_per_speed * v
        a_acc               % 가속도 [m/s^2] = v^2 / (2 L_acc)
        t_acc               % 가속 시간 [s] = 2 L_acc / v
        L_circle            % 원 구간 총 길이 [m]
        total_length        % 전체 경로 길이 [m]
        tf                  % 종료 시각 [s]

        lookahead_dist      % 전방 곡률 예견 거리 [m] ([] 이면 현재 지점 곡률)
        kappa_tau           % 전방 곡률 댐핑 시정수 [s]
        kappa_raw
        kappa_ff
        t_kappa_prev
        last_kappa
    end

    methods
        function obj = CircleTrajectory(init_pos, target_speed, curvature, accel_dist_per_speed, n_laps, lookahead_dist, kappa_tau)
            if nargin < 6, lookahead_dist = []; end
            if nargin < 7 || isempty(kappa_tau), kappa_tau = 0; end
            if abs(curvature) < 1e-6, error('CircleTrajectory: curvature 는 0 이 아니어야 합니다.'); end

            obj.init_pos = init_pos(:);
            obj.v        = target_speed;
            obj.kappa    = curvature;
            obj.R        = 1 / abs(curvature);
            obj.n_laps   = n_laps;

            obj.L_acc = accel_dist_per_speed * target_speed;          % 속도에 비례하는 가속 구간
            obj.a_acc = target_speed^2 / (2 * obj.L_acc);
            obj.t_acc = 2 * obj.L_acc / target_speed;

            obj.L_circle     = n_laps * 2 * pi * obj.R;
            obj.total_length = obj.L_acc + obj.L_circle;
            obj.tf           = obj.t_acc + obj.L_circle / target_speed;

            obj.lookahead_dist = lookahead_dist;
            obj.kappa_tau      = kappa_tau;
            obj.kappa_raw = 0; obj.kappa_ff = []; obj.t_kappa_prev = []; obj.last_kappa = 0;

            fprintf('[CircleTrajectory] R = %.2f m, 가속 %.1f m (%.2f m/s^2, %.1f s), 원 %d바퀴 %.1f m, tf = %.1f s\n', ...
                obj.R, obj.L_acc, obj.a_acc, obj.t_acc, n_laps, obj.L_circle, obj.tf);
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
            if s <= obj.L_acc
                p = obj.init_pos + [s; 0; 0];
                T = [1; 0; 0];
                k = 0;
            else
                ang   = obj.kappa * (s - obj.L_acc);                % 진입 후 회전각 (부호 포함)
                entry = obj.init_pos + [obj.L_acc; 0; 0];
                ctr   = entry + [0; 1/obj.kappa; 0];                 % 원 중심 (진입점에서 접선에 수직)
                r0    = [0; -1/obj.kappa];                           % 중심 -> 진입점
                Rot   = [cos(ang) -sin(ang); sin(ang) cos(ang)];
                p = ctr + [Rot * r0; 0];
                T = [cos(ang); sin(ang); 0];
                k = obj.kappa;
            end
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
end
