classdef TrajectoryCtrl < handle
%% MEMBERS
    properties
        waypoints
        num_segments
        dt_segment

        tf

        % 5차 다항식 경로 생성 계수
        coeffs_x
        coeffs_y
        coeffs_z

    end
%% METHODS
    methods
        %% Constructor
        function obj = TrajectoryCtrl(init_pos, waypoints_in, duration)

            obj.waypoints = [init_pos(:), waypoints_in];

            obj.num_segments = size(obj.waypoints, 2) - 1; % 점이 3개면 구간은 2개
            obj.tf = duration;
            obj.dt_segment = obj.tf / obj.num_segments;

            % precalculation of Coefficients
            obj.coeffs_x = zeros(6, obj.num_segments);
            obj.coeffs_y = zeros(6, obj.num_segments);
            obj.coeffs_z = zeros(6, obj.num_segments);

            for i = 1:obj.num_segments
                % i번째 구간의 시작점(p_start)과 끝점(p_end)
                p_start = obj.waypoints(:, i);
                p_end   = obj.waypoints(:, i+1);

                obj.coeffs_x(:, i) = obj.calc_coeff(p_start(1), p_end(1), obj.dt_segment);
                obj.coeffs_y(:, i) = obj.calc_coeff(p_start(2), p_end(2), obj.dt_segment);
                obj.coeffs_z(:, i) = obj.calc_coeff(p_start(3), p_end(3), obj.dt_segment);
            end

        end

        function a = calc_coeff(~, start_val, end_val, tf)
            del_p = end_val - start_val;
        
            a = zeros(6, 1);
            a(1) = start_val;
            a(2) = 0;
            a(3) = 0;
            a(4) = 10 * del_p / (tf^3);
            a(5) = -15 * del_p / (tf^4);
            a(6) = 6 * del_p / (tf^5);
        end

        function pos_des = get_position(obj, t)
            % 시간이 끝나면 최종 위치 반환
            if t >= obj.tf
                pos_des = obj.waypoints(:, end);
                return;
            end
            
            % 1. 현재 몇 번째 구간인지 찾기 (Segment Index)
            % 예: 5초 비행, 구간 2개 -> 구간당 2.5초
            % t=1.0 -> 1구간, t=3.0 -> 2구간
            seg_idx = floor(t / obj.dt_segment) + 1;
            
            % (혹시 모를 인덱스 초과 방지)
            if seg_idx > obj.num_segments
                seg_idx = obj.num_segments;
            end
            
            % 2. 해당 구간 내에서의 '로컬 시간' 계산
            % 2구간(2.5초~5.0초)에 있다면, t=3.0일 때 로컬시간은 0.5초
            t_local = t - (seg_idx - 1) * obj.dt_segment;
            
            % 3. 해당 구간의 계수 꺼내기
            cx = obj.coeffs_x(:, seg_idx);
            cy = obj.coeffs_y(:, seg_idx);
            cz = obj.coeffs_z(:, seg_idx);
            
            % 4. 5차 다항식 계산
            time_vec = [1; t_local; t_local^2; t_local^3; t_local^4; t_local^5];
            
            x_d = cx' * time_vec;
            y_d = cy' * time_vec;
            z_d = cz' * time_vec;
            
            pos_des = [x_d; y_d; z_d];
        end
        % TrajectoryCtrl.m 안의 methods 부분 끝에 추가
        function [X, Y, Z] = get_full_path(obj, dt_sample)
            % 0초부터 tf까지 dt_sample 간격으로 시간 벡터 생성
            time_vec = 0:dt_sample:obj.tf;
            pts = zeros(3, length(time_vec));
            
            for i = 1:length(time_vec)
                pts(:, i) = obj.get_position(time_vec(i));
            end
            
            X = pts(1, :);
            Y = pts(2, :);
            Z = pts(3, :);
        end
    end
end