classdef PositionCtrl_LPV < handle
    % POSITIONCTRL LPV 피드포워드 + Z축 스케줄링 + PPO 잔차 게인 오프셋 통합 위치 제어기
    % - 기본(offset = 0): 순수 LPV 제어기 (또는 NDI 고정 게인 제어기)로 동작
    % - setResidualOffsets() 호출 시: PPO 잔차 제어기(RL_PositionCtrl_LPV)로 동작

    properties
        g = 9.81
        m
        dt

        x_des, y_des, z_des, psi_des
        v_des, kappa_des

        x_err, x_err_prev, x_err_sum
        y_err, y_err_prev, y_err_sum
        z_err, z_err_prev, z_err_sum

        % Fixed Gains (Feedforward 꺼졌을 때 사용)
        kP_x_fix, kI_x_fix, kD_x_fix
        kP_y_fix, kI_y_fix, kD_y_fix
        kP_z_fix, kI_z_fix, kD_z_fix

        % LPV Desired Pole Coefficients (s^3 + a2*s^2 + a1*s + a0 = 0)
        lpv_a2, lpv_a1, lpv_a0

        is_first_run
        max_angle

        use_feedforward
        use_z_scheduling

        % PPO Residual Gain Offsets (기본값 0)
        offset_P_xy = 0.0
        offset_D_xy = 0.0
        offset_P_z  = 0.0
        offset_D_z  = 0.0

        % Logging용 변수
        current_kP_z
        current_kI_z
        current_kD_z
    end

    methods
        function obj = PositionCtrl_LPV(gains, params, cfg)
            obj.dt = cfg.dt;
            obj.m  = params('mass');

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

            % 기본 고정 게인 저장
            obj.kP_x_fix = gains('P_x'); obj.kI_x_fix = gains('I_x'); obj.kD_x_fix = gains('D_x');
            obj.kP_y_fix = gains('P_y'); obj.kI_y_fix = gains('I_y'); obj.kD_y_fix = gains('D_y');
            obj.kP_z_fix = gains('P_z'); obj.kI_z_fix = gains('I_z'); obj.kD_z_fix = gains('D_z');

            % 극점 역산 계수
            obj.lpv_a2 = gains('D_x');
            obj.lpv_a1 = gains('P_x');
            obj.lpv_a0 = gains('I_x');

            % 최대 뱅크각 제한 (45도)
            obj.max_angle = 45 * (pi / 180);
            obj.Reset();
        end

        function Reset(obj)
            obj.x_err = 0; obj.x_err_prev = 0; obj.x_err_sum = 0;
            obj.y_err = 0; obj.y_err_prev = 0; obj.y_err_sum = 0;
            obj.z_err = 0; obj.z_err_prev = 0; obj.z_err_sum = 0;
            obj.offset_P_xy = 0.0; obj.offset_D_xy = 0.0;
            obj.offset_P_z  = 0.0; obj.offset_D_z  = 0.0;
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

        function att_cmd = Update(obj, cmd, state)
            % 1. 목표 수신: cmd = [x_des; y_des; z_des; psi_des; v; kappa]
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
                obj.x_err_prev = obj.x_err;
                obj.y_err_prev = obj.y_err;
                obj.z_err_prev = obj.z_err;
                obj.is_first_run = false;
            end

            obj.x_err_sum = obj.x_err_sum + obj.x_err * obj.dt;
            obj.y_err_sum = obj.y_err_sum + obj.y_err * obj.dt;
            obj.z_err_sum = obj.z_err_sum + obj.z_err * obj.dt;

            dx_err = (obj.x_err - obj.x_err_prev) / obj.dt;
            dy_err = (obj.y_err - obj.y_err_prev) / obj.dt;
            dz_err = (obj.z_err - obj.z_err_prev) / obj.dt;

            % 3. Body 좌표계 오차 회전 (Heading 기준)
            ex_body     =  cos(psi) * obj.x_err     + sin(psi) * obj.y_err;
            ey_body     = -sin(psi) * obj.x_err     + cos(psi) * obj.y_err;
            ex_body_sum =  cos(psi) * obj.x_err_sum + sin(psi) * obj.y_err_sum;
            ey_body_sum = -sin(psi) * obj.x_err_sum + cos(psi) * obj.y_err_sum;
            dex_body    =  cos(psi) * dx_err        + sin(psi) * dy_err;
            dey_body    = -sin(psi) * dx_err        + cos(psi) * dy_err;

            if obj.use_feedforward
                % 피드포워드 평형점 계산
                phi_eq = atan(-(obj.v_des^2 * obj.kappa_des) / obj.g);
                U1_eq  = obj.m * sqrt(obj.g^2 + (obj.v_des^2 * obj.kappa_des)^2);

                % LPV 기본 게인 및 Z축 스케줄링
                base_kP_xy = obj.lpv_a1 / obj.g;
                kI_xy      = obj.lpv_a0 / obj.g;
                base_kD_xy = obj.lpv_a2 / obj.g;

                if obj.use_z_scheduling
                    b_z = cos(phi_eq) / obj.m;
                else
                    b_z = 1.0 / obj.m;
                end

                base_kP_z = obj.lpv_a1 / b_z;
                kI_z      = obj.lpv_a0 / b_z;
                base_kD_z = obj.lpv_a2 / b_z;

                % PPO 잔차 오프셋 합성 (기본값은 offset == 0 이므로 순수 LPV와 동일)
                kP_xy = max(base_kP_xy + obj.offset_P_xy, 0.1);
                kD_xy = max(base_kD_xy + obj.offset_D_xy, 0.05);
                kP_z  = max(base_kP_z  + obj.offset_P_z,  0.5);
                kD_z  = max(base_kD_z  + obj.offset_D_z,  0.1);

                obj.current_kP_z = kP_z;
                obj.current_kI_z = kI_z;
                obj.current_kD_z = kD_z;

                delta_theta = -(kP_xy * ex_body + kI_xy * ex_body_sum + kD_xy * dex_body);
                delta_phi   =  (kP_xy * ey_body + kI_xy * ey_body_sum + kD_xy * dey_body);
                delta_U1    = -(kP_z  * obj.z_err + kI_z  * obj.z_err_sum + kD_z  * dz_err);

                theta_des = delta_theta;
                phi_des   = phi_eq + delta_phi;
                U1_des    = U1_eq  + delta_U1;
            else
                % 비선형 역산 방식 (NDI)
                accel_x = obj.kP_x_fix * obj.x_err + obj.kI_x_fix * obj.x_err_sum + obj.kD_x_fix * dx_err;
                accel_y = obj.kP_y_fix * obj.y_err + obj.kI_y_fix * obj.y_err_sum + obj.kD_y_fix * dy_err;

                accel_fwd =  cos(psi) * accel_x + sin(psi) * accel_y;
                accel_rgt = -sin(psi) * accel_x + cos(psi) * accel_y;
                accel_up  = obj.g;

                thrust_sp = sqrt(accel_fwd^2 + accel_rgt^2 + accel_up^2);
                theta_des = -atan2(accel_fwd, accel_up);
                phi_des   = asin(max(min(accel_rgt / thrust_sp, 1.0), -1.0));

                zdot_des = obj.kP_z_fix * obj.z_err + obj.kI_z_fix * obj.z_err_sum + obj.kD_z_fix * dz_err;
                U1_des   = (obj.m * obj.g) - zdot_des;

                obj.current_kP_z = obj.kP_z_fix;
                obj.current_kI_z = obj.kI_z_fix;
                obj.current_kD_z = obj.kD_z_fix;
            end

            % 각도 제한
            theta_des = max(min(theta_des, obj.max_angle), -obj.max_angle);
            phi_des   = max(min(phi_des,   obj.max_angle), -obj.max_angle);

            obj.x_err_prev = obj.x_err;
            obj.y_err_prev = obj.y_err;
            obj.z_err_prev = obj.z_err;

            att_cmd = [phi_des; theta_des; obj.psi_des; U1_des];
        end
    end
end
