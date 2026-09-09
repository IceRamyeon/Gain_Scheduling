classdef TrajectoryCtrl < handle
    %% MEMBERS
    properties
        waypoints
        tf
        total_length
        speed
        
        % 스플라인 구조체 (x, y, z 각각)
        sp_x
        sp_y
        sp_z
        
        arc_lengths % 누적 곡선 길이
        u_samples   % 샘플링된 파라미터 u
    end
    
    %% METHODS
    methods
        %% Constructor
        function obj = TrajectoryCtrl(init_pos, waypoints_in, duration)
            % 시작점과 웨이포인트 합치기 (총 5개 점)
            pts = [init_pos(:), waypoints_in];
            obj.tf = duration;
            
            % 1. 점들 사이의 직선 거리(현의 길이)를 기준으로 파라미터 u 생성
            num_pts = size(pts, 2);
            u = zeros(1, num_pts);
            for i = 2:num_pts
                u(i) = u(i-1) + norm(pts(:,i) - pts(:,i-1));
            end
            u = u / u(end); % 0 ~ 1 사이로 정규화
            
            % 2. 3차 스플라인 곡선 생성
            obj.sp_x = spline(u, pts(1,:));
            obj.sp_y = spline(u, pts(2,:));
            obj.sp_z = spline(u, pts(3,:));
            
            % 3. 곡선의 실제 '호의 길이(Arc length)' 계산 (잘게 쪼개서 더하기)
            N_sample = 1000;
            obj.u_samples = linspace(0, 1, N_sample);
            
            px = ppval(obj.sp_x, obj.u_samples);
            py = ppval(obj.sp_y, obj.u_samples);
            pz = ppval(obj.sp_z, obj.u_samples);
            
            % 점과 점 사이 거리 계산
            dx = diff(px); dy = diff(py); dz = diff(pz);
            segment_lengths = sqrt(dx.^2 + dy.^2 + dz.^2);
            
            % 누적 거리 저장 및 총 길이 계산
            obj.arc_lengths = [0, cumsum(segment_lengths)];
            obj.total_length = obj.arc_lengths(end);
            
            % 4. 등속 비행을 위한 속력 계산 (전체 거리 / 비행 시간)
            obj.speed = obj.total_length / obj.tf;
            
            disp(['으헤~ 스플라인 궤적 총 길이: ', num2str(obj.total_length), 'm, 비행 속력: ', num2str(obj.speed), 'm/s']);
        end

        %% Get Position & Velocity
        function [pos_des, vel_des] = get_position(obj, t)
            % 시간 제한 (0 ~ tf)
            t = max(0, min(t, obj.tf));
            
            % 1. 현재 시간에 가야 할 목표 이동 거리 (d = v * t)
            d_target = obj.speed * t;
            
            % 2. 목표 거리에 해당하는 파라미터 u 찾기 (보간법)
            u_target = interp1(obj.arc_lengths, obj.u_samples, d_target, 'linear', 'extrap');
            u_target = max(0, min(u_target, 1));
            
            % 3. 목표 위치 계산
            x_d = ppval(obj.sp_x, u_target);
            y_d = ppval(obj.sp_y, u_target);
            z_d = ppval(obj.sp_z, u_target);
            pos_des = [x_d; y_d; z_d];
            
            % 4. 목표 속도 벡터(접선 방향) 계산 (수치 미분 활용)
            du = 1e-5;
            u_eval = min(u_target + du, 1);
            u_prev = max(u_target - du, 0);
            
            dp_x = ppval(obj.sp_x, u_eval) - ppval(obj.sp_x, u_prev);
            dp_y = ppval(obj.sp_y, u_eval) - ppval(obj.sp_y, u_prev);
            dp_z = ppval(obj.sp_z, u_eval) - ppval(obj.sp_z, u_prev);
            
            tangent = [dp_x; dp_y; dp_z];
            tangent_norm = norm(tangent);
            
            if tangent_norm > 1e-6
                tangent = tangent / tangent_norm;
            else
                tangent = [1; 0; 0]; % 혹시 정지해 있을 경우 대비
            end
            
            % 최종 목표 속도 = 방향 벡터 * 지정된 속력
            vel_des = obj.speed * tangent;
        end
        
        %% Get Full Path (애니메이션에서 점선 그리기용)
        function [X, Y, Z] = get_full_path(obj, dt_sample)
            time_vec = 0:dt_sample:obj.tf;
            pts = zeros(3, length(time_vec));
            
            for i = 1:length(time_vec)
                [pts(:, i), ~] = obj.get_position(time_vec(i));
            end
            
            X = pts(1, :);
            Y = pts(2, :);
            Z = pts(3, :);
        end
    end
end