classdef PositionCtrl < handle
    %% MEMBERS
    properties
        g
        dt
        state
        cmd
        
        x_des, y_des, z_des, psi_des
        
        x_err, x_err_prev, x_err_sum
        y_err, y_err_prev, y_err_sum
        z_err, z_err_prev, z_err_sum
        
        kP_x, kI_x, kD_x
        kP_y, kI_y, kD_y
        kP_z, kI_z, kD_z
        
        is_first_run

        max_angle
        
        obs_center1         % 장애물1 중심위치
        obs_bounds1         % 장애물 범위
        obs_radius1         % 안전 반경

        obs_pos2            % 장애물2 위치
        obs_radius2         % 안전 반경

        k_rep               % 충돌 회피 게인

        obs_list
        has_obstacle
    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = PositionCtrl(gains, obs_list, dt)
            obj.g = 9.81;
            obj.dt = dt;
            
            obj.kP_x = gains('P_x'); obj.kI_x = gains('I_x'); obj.kD_x = gains('D_x');
            obj.kP_y = gains('P_y'); obj.kI_y = gains('I_y'); obj.kD_y = gains('D_y');
            obj.kP_z = gains('P_z'); obj.kI_z = gains('I_z'); obj.kD_z = gains('D_z');
            
            % R2D = 180/pi;
            obj.max_angle = 20 * (pi/180); 
            
            obj.Reset();

            if ~isempty(obs_list)
                obj.has_obstacle = true;

                obj.obs_bounds1 = obs_list{1};

                obj.obs_center1 = mean(obj.obs_bounds1, 2);

                % 회피 반경 계산
                dims = obj.obs_bounds1(:,2) - obj.obs_bounds1(:,1);
                obj.obs_radius1 = norm(dims) / 2 + 1.5;
            else
                % 장애물이 없으면 멀리 치워두기
                obj.has_obstacle = false;
                obj.obs_center1 = [99900;99900;99900];
                obj.obs_radius1 = 0;
            end
        
            obj.k_rep = 0.0;
        end
        
        %% Reset
        function Reset(obj)
            obj.x_err = 0; obj.x_err_prev = 0; obj.x_err_sum = 0;
            obj.y_err = 0; obj.y_err_prev = 0; obj.y_err_sum = 0;
            obj.z_err = 0; obj.z_err_prev = 0; obj.z_err_sum = 0;

            obj.is_first_run = true;
        end

        %% GetState
        function state = GetState(obj)
            state = obj.state;
        end

        %% Get cmd
        function cmd = commandSig(obj)
            cmd = obj.cmd;
        end
        
        %% Update
        % 입력 cmd: [x_des, y_des, z_des, psi_des]
        % 입력 state: [x, y, z, u, v, w, phi, theta, psi, ...]
        % 출력 att_cmd: [phi_des, theta_des, psi_des, zdot_des]
        function att_cmd = Update(obj, cmd, state)

            % 1. 목표 및 현재 상태 해석
            obj.x_des = cmd(1);
            obj.y_des = cmd(2);
            obj.z_des = cmd(3);
            obj.psi_des = cmd(4); % 헤딩(Yaw)은 위치 제어기가 아니라 사용자가 정함
            
            x = state(1);
            y = state(2);
            z = state(3);
            psi = state(9); % 현재 드론의 Yaw 각도 (매우 중요!)
            
            % 2. 오차 계산
            obj.x_err = obj.x_des - x;
            obj.y_err = obj.y_des - y;
            obj.z_err = obj.z_des - z;

            if obj.is_first_run
                obj.x_err_prev = obj.x_err;
                obj.y_err_prev = obj.y_err;
                obj.z_err_prev = obj.z_err;
                obj.is_first_run = false;
            end
            
            % 3. X, Y PID 제어 -> 가상 가속도(Virtual Acceleration) 생성
            % (Global Frame 기준)
            accel_x = obj.kP_x * obj.x_err + ...
                      obj.kI_x * obj.x_err_sum + ...
                      obj.kD_x * (obj.x_err - obj.x_err_prev)/obj.dt;
                      
            accel_y = obj.kP_y * obj.y_err + ...
                      obj.kI_y * obj.y_err_sum + ...
                      obj.kD_y * (obj.y_err - obj.y_err_prev)/obj.dt;

            % 3.5 Obstacle Avoid
            if obj.has_obstacle

                vec_to_obs = [x; y; z] - obj.obs_center1;
                dist = norm(vec_to_obs);

                if dist < obj.obs_radius1

                    unit_vec_away = vec_to_obs / dist;

                    rep_mag = obj.k_rep * (1/dist - 1/obj.obs_radius1);

                    repulsive_accel = rep_mag * unit_vec_away;

                    accel_x = accel_x + repulsive_accel(1);
                    accel_y = accel_y + repulsive_accel(2);
                end
            end
            
            % 4. 좌표계 변환 (Global -> Body)
            % 우리가 구한 가속도는 '지구 기준' 동서남북이야.
            % 하지만 드론은 자기가 보는 방향(Body Frame) 기준으로 기울임.
            % Rotation Matrix (Yaw only):
            % [ ax_body ] = [  cos(psi)   sin(psi) ] [ accel_x ]
            % [ ay_body ] = [ -sin(psi)   cos(psi) ] [ accel_y ]
            
            accel_fwd =  cos(psi) * accel_x + sin(psi) * accel_y;
            accel_rgt = -sin(psi) * accel_x + cos(psi) * accel_y;
            
            % 5. 가속도를 목표 각도(Angle)로 변환            
            theta_des = -accel_fwd / obj.g;  % (가속도 / 중력) = 기울기
            phi_des   =  accel_rgt / obj.g;  % 오른쪽 가속 = 오른쪽 기울기(Roll)
            
            % 6. 각도 제한 (Safety)
            theta_des = max(min(theta_des, obj.max_angle), -obj.max_angle);
            phi_des   = max(min(phi_des,   obj.max_angle), -obj.max_angle);
            
            % 7. Z축 제어 (위치 오차 -> 목표 수직 속도)
            % AttitudeCtrl이 zdot_des를 받는다고 했으므로,
            % 여기서 Z 위치 오차를 PID 돌려서 "목표 속도"를 만들어 줌.
            zdot_des = obj.kP_z * obj.z_err + ...
                       obj.kI_z * obj.z_err_sum + ...
                       obj.kD_z * (obj.z_err - obj.z_err_prev)/obj.dt;
            
            % 8. 오차 적분 및 과거 값 업데이트
            obj.x_err_sum = obj.x_err_sum + obj.x_err * obj.dt;
            obj.x_err_prev = obj.x_err;
            
            obj.y_err_sum = obj.y_err_sum + obj.y_err * obj.dt;
            obj.y_err_prev = obj.y_err;
            
            obj.z_err_sum = obj.z_err_sum + obj.z_err * obj.dt;
            obj.z_err_prev = obj.z_err;
            
            % 9. 최종 출력 포장 [Roll_d, Pitch_d, Yaw_d, Zdot_d]
            att_cmd = [phi_des; theta_des; obj.psi_des; zdot_des];
        end
    end
end