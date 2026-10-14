classdef RL_PositionCtrl_LPV < handle
    %% MEMBERS
    properties
        g
        m
        dt
        state
        cmd
        
        x_des, y_des, z_des, psi_des
        v_des, kappa_des
        
        x_err, x_err_prev, x_err_sum
        y_err, y_err_prev, y_err_sum
        z_err, z_err_prev, z_err_sum
        
        % LPV Desired Pole Coefficients (s^3 + a2*s^2 + a1*s + a0 = 0)
        % Poles: [-1.5+1.2i, -1.5-1.2i, -5] => a2=8, a1=18.69, a0=18.45
        lpv_a2, lpv_a1, lpv_a0
        
        is_first_run
        max_angle

        use_feedforward  % Feedforward 사용 여부
        use_z_scheduling % Z축 게인 스케줄링 여부

        % PPO Residual Gain Offsets
        offset_P_xy
        offset_D_xy
        offset_P_z
        offset_D_z

        % Logging용 변수
        current_kP_z
        current_kI_z
        current_kD_z
    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = RL_PositionCtrl_LPV(gains, params, cfg)
            obj.g = 9.81;
            obj.dt = cfg.dt;
            obj.m = params('mass');
            
            if isfield(cfg, 'Feedforward')
                obj.use_feedforward = cfg.Feedforward;
            else
                obj.use_feedforward = true;
            end
            
            if isfield(cfg, 'Z_Gain_Scheduling')
                obj.use_z_scheduling = cfg.Z_Gain_Scheduling;
            else
                obj.use_z_scheduling = true;
            end
            
            % 극점 역산 계수 (Overshoot <= 4%, t_ss <= 3s, Ess = 0)
            obj.lpv_a2 = 8.0;
            obj.lpv_a1 = 18.69;
            obj.lpv_a0 = 18.45;
            
            % 최대 각도 제한 상향 (고속 선회 뱅크각 확보: 30도 -> 65도)
            obj.max_angle = 65 * (pi/180); 
            
            % Residual Offsets 초기화
            obj.offset_P_xy = 0.0;
            obj.offset_D_xy = 0.0;
            obj.offset_P_z  = 0.0;
            obj.offset_D_z  = 0.0;
            
            obj.Reset();
        end
        
        %% Reset
        function Reset(obj)
            obj.x_err = 0; obj.x_err_prev = 0; obj.x_err_sum = 0;
            obj.y_err = 0; obj.y_err_prev = 0; obj.y_err_sum = 0;
            obj.z_err = 0; obj.z_err_prev = 0; obj.z_err_sum = 0;
            obj.is_first_run = true;
        end

        function setResidualOffsets(obj, dP_xy, dD_xy, dP_z, dD_z)
            obj.offset_P_xy = dP_xy;
            obj.offset_D_xy = dD_xy;
            obj.offset_P_z  = dP_z;
            obj.offset_D_z  = dD_z;
        end

        function pid_gains = GetZ_PID(obj)
            pid_gains = [obj.current_kP_z; obj.current_kI_z; obj.current_kD_z];
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
            
            % 4. 피드포워드 계산 (선회 롤각 및 평형 추력)
            phi_eq = atan(-(obj.v_des^2 * obj.kappa_des) / obj.g);
            U1_eq  = obj.m * sqrt(obj.g^2 + (obj.v_des^2 * obj.kappa_des)^2);
            
            % 5. LPV 기본 게인 산출
            base_kP_xy = obj.lpv_a1 / obj.g;
            kI_xy      = obj.lpv_a0 / obj.g;
            base_kD_xy = obj.lpv_a2 / obj.g;
            
            % Z축 게인 산출 (기울기 phi_eq에 따른 LPV 스케줄링)
            if obj.use_z_scheduling
                b_z = cos(phi_eq) / obj.m;
            else
                b_z = 1.0 / obj.m;
            end
            base_kP_z = obj.lpv_a1 / b_z;
            kI_z      = obj.lpv_a0 / b_z;
            base_kD_z = obj.lpv_a2 / b_z;
            
            % 6. PPO 잔차 게인 합성 (RL Damping & Proportional Correction)
            kP_xy = max(base_kP_xy + obj.offset_P_xy, 0.1);
            kD_xy = max(base_kD_xy + obj.offset_D_xy, 0.05);
            kP_z  = max(base_kP_z  + obj.offset_P_z,  0.5);
            kD_z  = max(base_kD_z  + obj.offset_D_z,  0.1);
            
            % 로깅용 저장
            obj.current_kP_z = kP_z; 
            obj.current_kI_z = kI_z; 
            obj.current_kD_z = kD_z;
            
            % 7. 오차 피드백 제어량 계산
            delta_theta = -(kP_xy * ex_body + kI_xy * ex_body_sum + kD_xy * dex_body);
            delta_phi   =  (kP_xy * ey_body + kI_xy * ey_body_sum + kD_xy * dey_body);
            delta_U1    = -(kP_z  * obj.z_err + kI_z  * obj.z_err_sum + kD_z  * dz_err);
            
            % 8. 최종 지령 = 평형점(피드포워드) + 보상값 (Thrust U1 직접 산출)
            theta_des = 0 + delta_theta;
            phi_des   = phi_eq + delta_phi;
            U1_des    = U1_eq + delta_U1;
            
            % 각도 제한 (Safety)
            theta_des = max(min(theta_des, obj.max_angle), -obj.max_angle);
            phi_des   = max(min(phi_des,   obj.max_angle), -obj.max_angle);
            
            obj.x_err_prev = obj.x_err;
            obj.y_err_prev = obj.y_err;
            obj.z_err_prev = obj.z_err;
            
            % 9. 최종 출력 (Roll, Pitch, Yaw, Direct Thrust U1)
            att_cmd = [phi_des; theta_des; obj.psi_des; U1_des];
        end
    end
end
