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
    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = PositionCtrl(gains, dt)
            obj.g = 9.81;
            obj.dt = dt;
            
            obj.kP_x = gains('P_x'); obj.kI_x = gains('I_x'); obj.kD_x = gains('D_x');
            obj.kP_y = gains('P_y'); obj.kI_y = gains('I_y'); obj.kD_y = gains('D_y');
            obj.kP_z = gains('P_z'); obj.kI_z = gains('I_z'); obj.kD_z = gains('D_z');
            
            obj.max_angle = 20 * (pi/180); 
            
            obj.Reset();
        end
        
        %% Reset, GetState, commandSig (이전과 동일하므로 생략 가능)
        function Reset(obj)
            obj.x_err = 0; obj.x_err_prev = 0; obj.x_err_sum = 0;
            obj.y_err = 0; obj.y_err_prev = 0; obj.y_err_sum = 0;
            obj.z_err = 0; obj.z_err_prev = 0; obj.z_err_sum = 0;
            obj.is_first_run = true;
        end

        function state = GetState(obj)
            state = obj.state;
        end

        function cmd = commandSig(obj)
            cmd = obj.cmd;
        end
        
        %% Update
        function att_cmd = Update(obj, cmd, state)
            % 1. 목표 및 현재 상태 해석
            obj.x_des = cmd(1); obj.y_des = cmd(2); obj.z_des = cmd(3); obj.psi_des = cmd(4);
            x = state(1); y = state(2); z = state(3); psi = state(9); 
            
            % 2. 오차 계산
            obj.x_err = obj.x_des - x; obj.y_err = obj.y_des - y; obj.z_err = obj.z_des - z;

            if obj.is_first_run
                obj.x_err_prev = obj.x_err; obj.y_err_prev = obj.y_err; obj.z_err_prev = obj.z_err;
                obj.is_first_run = false;
            end
            
            % 3. X, Y PID 제어 -> 가상 가속도 생성
            accel_x = obj.kP_x * obj.x_err + obj.kI_x * obj.x_err_sum + obj.kD_x * (obj.x_err - obj.x_err_prev)/obj.dt;
            accel_y = obj.kP_y * obj.y_err + obj.kI_y * obj.y_err_sum + obj.kD_y * (obj.y_err - obj.y_err_prev)/obj.dt;
            
            % 4. 좌표계 변환 (Global -> Body)
            accel_fwd =  cos(psi) * accel_x + sin(psi) * accel_y;
            accel_rgt = -sin(psi) * accel_x + cos(psi) * accel_y;
            
            % 5. 가속도를 목표 각도(Angle)로 변환            
            theta_des = -accel_fwd / obj.g;  
            phi_des   =  accel_rgt / obj.g;  
            
            % 6. 각도 제한 (Safety)
            theta_des = max(min(theta_des, obj.max_angle), -obj.max_angle);
            phi_des   = max(min(phi_des,   obj.max_angle), -obj.max_angle);
            
            % 7. Z축 제어
            zdot_des = obj.kP_z * obj.z_err + obj.kI_z * obj.z_err_sum + obj.kD_z * (obj.z_err - obj.z_err_prev)/obj.dt;
            
            % 8. 오차 적분 및 과거 값 업데이트
            obj.x_err_sum = obj.x_err_sum + obj.x_err * obj.dt; obj.x_err_prev = obj.x_err;
            obj.y_err_sum = obj.y_err_sum + obj.y_err * obj.dt; obj.y_err_prev = obj.y_err;
            obj.z_err_sum = obj.z_err_sum + obj.z_err * obj.dt; obj.z_err_prev = obj.z_err;
            
            % 9. 최종 출력
            att_cmd = [phi_des; theta_des; obj.psi_des; zdot_des];
        end
    end
end