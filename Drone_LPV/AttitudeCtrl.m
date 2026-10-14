classdef AttitudeCtrl < handle
    %% MEMBERS
    properties
        m
        g
        dt
        x
        cmd

        phi_des
        theta_des
        psi_des
        U1_des % zdot_des에서 변경
        
        % 지령 평활화 필터 (외란/수치 이산화 톱니파 노이즈 제거 LPF)
        use_cmd_filter = true;
        filter_alpha = 0.25; % 100Hz 기준 시상수 약 0.03초
        phi_des_filt = 0;
        theta_des_filt = 0;
        psi_des_filt = 0;
        is_first_run = true;
        
        phi_err, phi_err_prev, phi_err_sum
        theta_err, theta_err_prev, theta_err_sum
        psi_err, psi_err_prev, psi_err_sum
    
        kP_phi, kD_phi, kI_phi
        kP_theta, kD_theta, kI_theta
        kP_psi, kD_psi, kI_psi
    end
    
    %% METHODS
    methods
       %% CONSTRUCTOR
       function obj = AttitudeCtrl(params, gains, dt)
            obj.g = 9.81;
            obj.dt = dt;
            obj.m = params('mass');
            
            obj.kP_phi = gains('P_phi'); obj.kI_phi = gains('I_phi'); obj.kD_phi = gains('D_phi');
            obj.kP_theta = gains('P_theta'); obj.kI_theta = gains('I_theta'); obj.kD_theta = gains('D_theta');
            obj.kP_psi = gains('P_psi'); obj.kI_psi = gains('I_psi'); obj.kD_psi = gains('D_psi');
            
            % zdot 게인은 더 이상 사용하지 않으므로 삭제
            obj.Reset();
       end
       
        %% Reset
        function Reset(obj)
            obj.phi_err = 0; obj.phi_err_prev = 0; obj.phi_err_sum = 0;
            obj.theta_err = 0; obj.theta_err_prev = 0; obj.theta_err_sum = 0;
            obj.psi_err = 0; obj.psi_err_prev = 0; obj.psi_err_sum = 0;
            obj.is_first_run = true;
        end

        function state = GetState(obj)
            state = obj.x;
        end

        function cmd = commandSig(obj)
            cmd = obj.cmd;
        end

        %% Update
        function u = Update(obj, cmd, state)
            
            raw_phi_des   = cmd(1); 
            raw_theta_des = cmd(2); 
            raw_psi_des   = cmd(3); 
            obj.U1_des    = cmd(4); % 추력 다이렉트 수신
            
            if obj.is_first_run
                obj.phi_des_filt   = raw_phi_des;
                obj.theta_des_filt = raw_theta_des;
                obj.psi_des_filt   = raw_psi_des;
                obj.is_first_run   = false;
            end
            
            % 지령 저역통과 필터 (Command Shaping LPF): 외루프 40Hz 공진 억제
            if obj.use_cmd_filter
                a = obj.filter_alpha;
                obj.phi_des_filt   = (1 - a) * obj.phi_des_filt   + a * raw_phi_des;
                obj.theta_des_filt = (1 - a) * obj.theta_des_filt + a * raw_theta_des;
                
                % Yaw는 각도 래핑(wrap) 고려
                yaw_diff = raw_psi_des - obj.psi_des_filt;
                while yaw_diff > pi, raw_psi_des = raw_psi_des - 2*pi; yaw_diff = raw_psi_des - obj.psi_des_filt; end
                while yaw_diff < -pi, raw_psi_des = raw_psi_des + 2*pi; yaw_diff = raw_psi_des - obj.psi_des_filt; end
                obj.psi_des_filt   = (1 - a) * obj.psi_des_filt   + a * raw_psi_des;
                
                obj.phi_des   = obj.phi_des_filt;
                obj.theta_des = obj.theta_des_filt;
                obj.psi_des   = obj.psi_des_filt;
            else
                obj.phi_des   = raw_phi_des;
                obj.theta_des = raw_theta_des;
                obj.psi_des   = raw_psi_des;
            end
       
            phi = state(7); 
            theta = state(8); 
            psi = state(9); 
            p = state(10);
            q = state(11);
            r = state(12);
            
            obj.phi_err = obj.phi_des - phi;
            obj.theta_err = obj.theta_des - theta;
            obj.psi_err = obj.psi_des - psi;
            
            % D-on-Measurement 적용: 수치 오차 차분 대신 실제 기체 각속도(p, q, r)를 직접 피드백
            % -> 목표각 스텝 변화에 의한 미분 킥(Derivative Kick) 및 고주파 채터링 완벽 제거
            % Roll
            u_phi = obj.kP_phi * obj.phi_err + ...
                    obj.kI_phi * obj.phi_err_sum - ...
                    obj.kD_phi * p;
            
            % Pitch
            u_theta = obj.kP_theta * obj.theta_err + ...
                      obj.kI_theta * obj.theta_err_sum - ...
                      obj.kD_theta * q;
            
            % Yaw
            u_psi = obj.kP_psi * obj.psi_err + ...
                    obj.kI_psi * obj.psi_err_sum - ...
                    obj.kD_psi * r;
            
            % Thrust (다이렉트 패스스루)
            u_thrust = obj.U1_des;

            % 과거값 업데이트
            obj.phi_err_sum = obj.phi_err_sum + obj.phi_err * obj.dt;
            obj.phi_err_prev = obj.phi_err;
            
            obj.theta_err_sum = obj.theta_err_sum + obj.theta_err * obj.dt;
            obj.theta_err_prev = obj.theta_err;
            
            obj.psi_err_sum = obj.psi_err_sum + obj.psi_err * obj.dt;
            obj.psi_err_prev = obj.psi_err;
            
            thrust_max = 500;
            phi_max = 8.0; % 라디안인지 디그리인지 단위 주의
            theta_max = 8.0; 
            
            % 제한 적용
            u_thrust = max(0, min(u_thrust, thrust_max)); % 추력은 보통 0 이상
            u_phi = max(-phi_max, min(u_phi, phi_max));
            u_theta = max(-theta_max, min(u_theta, theta_max));

            u = [u_thrust; u_phi; u_theta; u_psi];
        end
    end
end