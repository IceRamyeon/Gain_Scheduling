classdef PositionCtrl < handle
    %% MEMBERS
    properties
        g
        m
        dt
        state
        cmd
        
        use_lpv
        
        x_des, y_des, z_des, psi_des
        v_des, kappa_des
        
        x_err, x_err_prev, x_err_sum
        y_err, y_err_prev, y_err_sum
        z_err, z_err_prev, z_err_sum
        
        % Fixed Gains (LPV 꺼졌을 때 사용)
        kP_x_fix, kI_x_fix, kD_x_fix
        kP_y_fix, kI_y_fix, kD_y_fix
        kP_z_fix, kI_z_fix, kD_z_fix
        
        % LPV Desired Pole Coefficients (s^3 + a2*s^2 + a1*s + a0 = 0)
        % Poles: [-1.5+1.2i, -1.5-1.2i, -5] => a2=8, a1=18.69, a0=18.45
        lpv_a2, lpv_a1, lpv_a0
        
        is_first_run
        max_angle
    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = PositionCtrl(gains, params, cfg)
            obj.g = 9.81;
            obj.dt = cfg.dt;
            obj.m = params('mass');
            
            % LPV 사용 여부
            if isfield(cfg, 'LPV_Gain_Scheduling')
                obj.use_lpv = cfg.LPV_Gain_Scheduling;
            else
                obj.use_lpv = false;
            end
            
            % 기본 고정 게인 저장
            obj.kP_x_fix = gains('P_x'); obj.kI_x_fix = gains('I_x'); obj.kD_x_fix = gains('D_x');
            obj.kP_y_fix = gains('P_y'); obj.kI_y_fix = gains('I_y'); obj.kD_y_fix = gains('D_y');
            obj.kP_z_fix = gains('P_z'); obj.kI_z_fix = gains('I_z'); obj.kD_z_fix = gains('D_z');
            
            % 극점 역산 계수 (Overshoot <= 4%, t_ss <= 3s, Ess = 0)
            obj.lpv_a2 = 8.0;
            obj.lpv_a1 = 18.69;
            obj.lpv_a0 = 18.45;
            
            obj.max_angle = 30 * (pi/180); 
            obj.Reset();
        end
        
        %% Reset
        function Reset(obj)
            obj.x_err = 0; obj.x_err_prev = 0; obj.x_err_sum = 0;
            obj.y_err = 0; obj.y_err_prev = 0; obj.y_err_sum = 0;
            obj.z_err = 0; obj.z_err_prev = 0; obj.z_err_sum = 0;
            obj.is_first_run = true;
        end
        
        %% Update
        function att_cmd = Update(obj, cmd, state)
            % 1. 목표 수신: cmd = [x_des, y_des, z_des, psi_des, v, kappa]
            obj.x_des = cmd(1); obj.y_des = cmd(2); obj.z_des = cmd(3); obj.psi_des = cmd(4);
            if length(cmd) >= 6
                obj.v_des = cmd(5); obj.kappa_des = cmd(6);
            else
                obj.v_des = 0; obj.kappa_des = 0;
            end
            
            x = state(1); y = state(2); z = state(3); psi = state(9); 
            
            % 2. 글로벌 오차 계산
            obj.x_err = obj.x_des - x; 
            obj.y_err = obj.y_des - y; 
            obj.z_err = obj.z_des - z;

            if obj.is_first_run
                obj.x_err_prev = obj.x_err; obj.y_err_prev = obj.y_err; obj.z_err_prev = obj.z_err;
                obj.is_first_run = false;
            end
            
            % 오차 적분 및 미분 계산
            obj.x_err_sum = obj.x_err_sum + obj.x_err * obj.dt; 
            obj.y_err_sum = obj.y_err_sum + obj.y_err * obj.dt; 
            obj.z_err_sum = obj.z_err_sum + obj.z_err * obj.dt; 
            
            dx_err = (obj.x_err - obj.x_err_prev)/obj.dt;
            dy_err = (obj.y_err - obj.y_err_prev)/obj.dt;
            dz_err = (obj.z_err - obj.z_err_prev)/obj.dt;
            
            % 3. Body 좌표계 오차 회전 (Heading 기준)
            ex_body =  cos(psi) * obj.x_err + sin(psi) * obj.y_err;
            ey_body = -sin(psi) * obj.x_err + cos(psi) * obj.y_err;
            
            ex_body_sum =  cos(psi) * obj.x_err_sum + sin(psi) * obj.y_err_sum;
            ey_body_sum = -sin(psi) * obj.x_err_sum + cos(psi) * obj.y_err_sum;
            
            dex_body =  cos(psi) * dx_err + sin(psi) * dy_err;
            dey_body = -sin(psi) * dx_err + cos(psi) * dy_err;
            
            if obj.use_lpv
                % [LPV 자동 튜닝 기반 평형점 및 제어 산출]
                phi_eq = atan(obj.v_des^2 * obj.kappa_des / obj.g);
                U1_eq = obj.m * sqrt(obj.g^2 + (obj.v_des^2 * obj.kappa_des)^2);
                
                % Zero-Pole matching에 의한 PID 게인 자동 산출
                % X, Y축은 시스템 상수(b)가 'g'로 동일
                kP_xy = obj.lpv_a1 / obj.g;
                kI_xy = obj.lpv_a0 / obj.g;
                kD_xy = obj.lpv_a2 / obj.g;
                
                % Z축은 시스템 상수(b)가 cos(phi_eq)/m
                b_z = cos(phi_eq) / obj.m;
                kP_z = obj.lpv_a1 / b_z;
                kI_z = obj.lpv_a0 / b_z;
                kD_z = obj.lpv_a2 / b_z;
                
                % 오차 피드백 제어량 (delta) 계산
                % x축 전진을 위해서는 theta를 숙여야 하므로 마이너스 부호
                delta_theta = -(kP_xy * ex_body + kI_xy * ex_body_sum + kD_xy * dex_body);
                % y축 우측 이동을 위해서는 phi를 기울여야 하므로 플러스 부호
                delta_phi = (kP_xy * ey_body + kI_xy * ey_body_sum + kD_xy * dey_body);
                
                delta_U1 = -(kP_z * obj.z_err + kI_z * obj.z_err_sum + kD_z * dz_err);
                
                % 최종 지령 = 평형점 + 보상값
                theta_des = 0 + delta_theta;
                phi_des   = phi_eq + delta_phi;
                U1_des    = U1_eq + delta_U1;
                
            else
                % [기존 비선형 역산 방식 (NDI)]
                accel_x = obj.kP_x_fix * obj.x_err + obj.kI_x_fix * obj.x_err_sum + obj.kD_x_fix * dx_err;
                accel_y = obj.kP_y_fix * obj.y_err + obj.kI_y_fix * obj.y_err_sum + obj.kD_y_fix * dy_err;
                
                accel_fwd =  cos(psi) * accel_x + sin(psi) * accel_y;
                accel_rgt = -sin(psi) * accel_x + cos(psi) * accel_y;
                accel_up = obj.g;
                
                thrust_sp = sqrt(accel_fwd^2 + accel_rgt^2 + accel_up^2);
                theta_des = -atan2(accel_fwd, accel_up); 
                sin_phi = max(min(accel_rgt / thrust_sp, 1.0), -1.0);
                phi_des = asin(sin_phi);
                
                % Z축 출력을 추력(U1)으로 맞춰서 변환 (NDI 방식에서의 추력)
                % 기존 zdot 제어를 유지하되, 출력을 U1 단위로 변환해 내보냄
                zdot_des = obj.kP_z_fix * obj.z_err + obj.kI_z_fix * obj.z_err_sum + obj.kD_z_fix * dz_err;
                U1_des = (obj.m * obj.g) - zdot_des; % 기존 AttitudeCtrl에 있던 공식
            end
            
            % 각도 제한 (Safety)
            theta_des = max(min(theta_des, obj.max_angle), -obj.max_angle);
            phi_des   = max(min(phi_des,   obj.max_angle), -obj.max_angle);
            
            obj.x_err_prev = obj.x_err;
            obj.y_err_prev = obj.y_err;
            obj.z_err_prev = obj.z_err;
            
            % 9. 최종 출력 (이제 Z축 출력이 zdot_des가 아니라 U1_des임!)
            att_cmd = [phi_des; theta_des; obj.psi_des; U1_des];
        end
    end
end