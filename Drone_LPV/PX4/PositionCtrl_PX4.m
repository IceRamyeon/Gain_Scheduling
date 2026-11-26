classdef PositionCtrl_PX4 < handle
    % POSITIONCTRL_PX4 ChoeJae PX4/PositionCtrl.m (GS 제어기)와 100% 동일한 기반 제어 법칙 + PPO 잔차 게인
    % - ChoeJae px4_main.m 및 run_px4_mission.m 의 PositionCtrl_PX4 인터페이스를 그대로 유지하면서,
    %   내부 제어 법칙·게인·오차 회전(Err_Yaw='desired')·Z 미분항(Z_D_on_measurement=true)·
    %   피드포워드(phi_eq, U1_eq)·Z축 스케줄링(b_z = cos(phi_eq)/m)을 ChoeJae PX4/PositionCtrl.m 과
    %   100% 일치시킨 통합 제어기.
    % - 잔차 오프셋이 0일 때 ChoeJae 'MATLAB' (GS) 모드와 완전히 동일한 출력(U1, phi, theta, Z 게인)을 생성함.
    % - Z 적분 안티와인드업은 실제 전송 추력(제곱근 역보정 thr_raw = thr_hover * sqrt(U1 / mg)) 기준으로 판정하여
    %   35°/40° 선회에서도 적분이 멈추지 않고 정상 동작함.

    properties
        g = 9.81
        m
        dt_pos              % 위치 제어기 주기: 0.02 s (50 Hz)
        mpc_thr_hover       % 호버링 정규화 추력: 0.73

        % LPV Desired Pole Coefficients (ChoeJae PositionCtrl.m 동일: lpv_a = gain_ang * g)
        lpv_a1              % P_x * g = 0.215  * 9.81 = 2.1092
        lpv_a0              % I_x * g = 0.0387 * 9.81 = 0.3796
        lpv_a2              % D_x * g = 0.203  * 9.81 = 1.9914

        % 제어 옵션 (ChoeJae px4_main.m 기본값과 100% 동일)
        use_feedforward  = true
        use_z_scheduling = true
        z_D_on_meas      = true
        err_yaw_desired  = true
        eq_delay_comp    = false
        K_att            = 6.5

        % 오차 적분 및 이전 오차 상태 (ChoeJae PositionCtrl.m 동일)
        x_err = 0; x_err_prev = 0; x_err_sum = 0;
        y_err = 0; y_err_prev = 0; y_err_sum = 0;
        z_err = 0; z_err_prev = 0; z_err_sum = 0;
        is_first_run = true
        max_angle           % 최대 롤/피치 각도 제한 (기본 50 deg)

        % PPO 잔차 게인 오프셋 (run_px4_mission.m 의 cfg.ppo.range_* 가속도 단위 입력 수용)
        offset_P_xy = 0.0
        offset_D_xy = 0.0
        offset_P_z  = 0.0
        offset_D_z  = 0.0

        % 로깅용 현재 Z축 PID 게인 [m/s^2 단위 -> run_px4_mission 에서 * cfg.m 하여 N/m 단위로 기록]
        current_kP_z
        current_kI_z
        current_kD_z
        is_angle_saturated = false
    end

    methods
        function obj = PositionCtrl_PX4(cfg)
            obj.g             = 9.81;
            obj.m             = cfg.m;
            obj.dt_pos        = cfg.dt_pos;
            obj.mpc_thr_hover = cfg.MPC_THR_HOVER;

            if isfield(cfg, 'pos_Kp_ang')
                p_x = cfg.pos_Kp_ang;
                i_x = cfg.pos_Ki_ang;
                d_x = cfg.pos_Kd_ang;
            else
                p_x = 0.215;
                i_x = 0.0387;
                d_x = 0.203;
            end

            % ChoeJae PositionCtrl.m 과 동일한 극점 역산 계수
            obj.lpv_a1 = p_x * obj.g; % 2.1092
            obj.lpv_a0 = i_x * obj.g; % 0.3796
            obj.lpv_a2 = d_x * obj.g; % 1.9914

            if isfield(cfg, 'Z_Gain_Scheduling')
                obj.use_z_scheduling = cfg.Z_Gain_Scheduling;
            else
                obj.use_z_scheduling = true;
            end

            if isfield(cfg, 'max_angle_deg') && ~isempty(cfg.max_angle_deg)
                obj.max_angle = deg2rad(cfg.max_angle_deg);
            else
                obj.max_angle = deg2rad(50);
            end

            if isfield(cfg, 'K_att') && ~isempty(cfg.K_att)
                obj.K_att = cfg.K_att;
            end

            % ChoeJae px4_main 최상위 제어 옵션 호환
            if isfield(cfg, 'Feedforward') && ~isempty(cfg.Feedforward)
                obj.use_feedforward = logical(cfg.Feedforward);
            end
            if isfield(cfg, 'Err_Yaw') && ~isempty(cfg.Err_Yaw)
                obj.err_yaw_desired = strcmpi(cfg.Err_Yaw, 'desired');
            end
            if isfield(cfg, 'Z_D_on_measurement') && ~isempty(cfg.Z_D_on_measurement)
                obj.z_D_on_meas = logical(cfg.Z_D_on_measurement);
            end
            if isfield(cfg, 'Eq_Delay_Comp') && ~isempty(cfg.Eq_Delay_Comp)
                obj.eq_delay_comp = logical(cfg.Eq_Delay_Comp);
            end

            obj.Reset();
        end

        function Reset(obj)
            obj.x_err = 0; obj.x_err_prev = 0; obj.x_err_sum = 0;
            obj.y_err = 0; obj.y_err_prev = 0; obj.y_err_sum = 0;
            obj.z_err = 0; obj.z_err_prev = 0; obj.z_err_sum = 0;
            obj.offset_P_xy = 0.0;
            obj.offset_D_xy = 0.0;
            obj.offset_P_z  = 0.0;
            obj.offset_D_z  = 0.0;
            obj.current_kP_z = obj.lpv_a1;
            obj.current_kI_z = obj.lpv_a0;
            obj.current_kD_z = obj.lpv_a2;
            obj.is_first_run = true;
            obj.is_angle_saturated = false;
        end

        function setResidualOffsets(obj, dP_xy, dD_xy, dP_z, dD_z)
            % run_px4_mission.m 호환 (10 Hz 호출):
            %   - 모든 입력이 0(PPO 비활성 구간)이면 즉시 0으로 리셋
            %   - XY 잔차는 저역통과(alpha=0.08 @ 10Hz)로 원 선회 중 World-NED 방위각 회전 리플 억제
            %   - Z 비례 잔차(dP_z)는 계단 Soft-Start 하향 감쇠(<=0)만 허용해 정상상태 리플 차단, 미분 잔차(dD_z)는 감쇠 보강 유지
            if dP_xy == 0 && dD_xy == 0 && dP_z == 0 && dD_z == 0
                obj.offset_P_xy = 0.0;
                obj.offset_D_xy = 0.0;
                obj.offset_P_z  = 0.0;
                obj.offset_D_z  = 0.0;
                return;
            end
            obj.offset_P_xy = 0.92 * obj.offset_P_xy + 0.08 * dP_xy;
            obj.offset_D_xy = 0.92 * obj.offset_D_xy + 0.08 * dD_xy;
            obj.offset_P_z  = 0.70 * obj.offset_P_z  + 0.30 * min(0.0, dP_z);
            obj.offset_D_z  = 0.80 * obj.offset_D_z  + 0.20 * dD_z;
        end

        function pid_gains = GetZ_PID(obj)
            % run_px4_mission.m 253행에서 posCtrl.GetZ_PID()' * cfg.m 을 수행하므로
            % [m/s^2] 단위로 반환하면 힘 단위 [N/m] = lpv_a * m / cos(phi_eq) (GS와 100% 동일!)가 됨
            pid_gains = [obj.current_kP_z; obj.current_kI_z; obj.current_kD_z];
        end

        function [att_cmd, U1_phys] = Update(obj, p_des, v_des, a_des, psi_des, p_est, v_est, euler_est)
            % 입력:
            %   p_des, v_des, a_des: 3x1 목표 위치, 속도, 가속도 (NED)
            %   psi_des: 목표 Yaw 각도 [rad]
            %   p_est, v_est: 3x1 추정 위치 및 속도 (NED)
            %   euler_est: [phi; theta; psi] 현재 자세 각도 [rad]
            % 출력:
            %   att_cmd: [phi_cmd; theta_cmd; psi_cmd; thr_cmd]
            %   U1_phys: 물리적 목표 총 추력 [N]

            dt = obj.dt_pos;

            % 1. 목표 속력 v_scalar 및 곡률 kappa_des 복원 (v_des, a_des 외적)
            v_xy = norm(v_des(1:2));
            if v_xy > 0.5
                kappa_des = (v_des(1) * a_des(2) - v_des(2) * a_des(1)) / (v_xy^3);
            else
                kappa_des = 0.0;
            end

            % 2. 오차 회전용 Yaw 선택 (ChoeJae PositionCtrl.m: Err_Yaw = 'desired')
            if obj.err_yaw_desired
                psi = psi_des;
            else
                psi = euler_est(3);
            end

            % 3. 글로벌 위치 오차 및 적분/미분 (ChoeJae PositionCtrl.m 과 100% 동일)
            obj.x_err = p_des(1) - p_est(1);
            obj.y_err = p_des(2) - p_est(2);
            obj.z_err = p_des(3) - p_est(3);

            if obj.is_first_run
                obj.x_err_prev = obj.x_err;
                obj.y_err_prev = obj.y_err;
                obj.z_err_prev = obj.z_err;
                obj.is_first_run = false;
            end

            obj.x_err_sum = obj.x_err_sum + obj.x_err * dt;
            obj.y_err_sum = obj.y_err_sum + obj.y_err * dt;

            dx_err = (obj.x_err - obj.x_err_prev) / dt;
            dy_err = (obj.y_err - obj.y_err_prev) / dt;
            if obj.z_D_on_meas
                dz_err = -v_est(3); % 측정 속도 기반 (-vz): 계단 입력 시 D-kick 원천 차단
            else
                dz_err = (obj.z_err - obj.z_err_prev) / dt;
            end

            % 4. Body 좌표계 오차 회전 (ChoeJae PositionCtrl.m 동일)
            c_psi = cos(psi);
            s_psi = sin(psi);
            ex_body     =  c_psi * obj.x_err     + s_psi * obj.y_err;
            ey_body     = -s_psi * obj.x_err     + c_psi * obj.y_err;
            ex_body_sum =  c_psi * obj.x_err_sum + s_psi * obj.y_err_sum;
            ey_body_sum = -s_psi * obj.x_err_sum + c_psi * obj.y_err_sum;
            dex_body    =  c_psi * dx_err        + s_psi * dy_err;
            dey_body    = -s_psi * dx_err        + c_psi * dy_err;

            % 5. 평형점 피드포워드 (ChoeJae PositionCtrl.m 동일)
            if obj.use_feedforward
                phi_eq = atan((v_xy^2 * kappa_des) / obj.g);
                U1_eq  = obj.m * sqrt(obj.g^2 + (v_xy^2 * kappa_des)^2);
            else
                phi_eq = 0.0;
                U1_eq  = obj.m * obj.g;
            end

            % 6. 게인 산출 (ChoeJae PositionCtrl.m 기저 게인 + 대각도 위상여유 보호 PPO 잔차 합성)
            %    - 대각도 선회(phi_eq -> 40 deg)에서는 횡방향 유효 플랜트 이득 d(g*tan(phi))/d(phi) = g*sec^2(phi_eq) 가
            %      1.70배 증폭되고 선회 각속도(omega = v*|kappa|) 교차 결합이 커져 위상 여유가 감소함.
            %    - 복합 유효 부하 지수 Lambda = sec^2(phi_eq) * (1 + 0.4 * v * |kappa|) 에 따라
            %      phi_eq <= 33 deg 에서는 XY 잔차를 100% 유지하고, 40 deg 고각속도 구간에서만 부드럽게 감쇠시켜
            %      롤-고도 한계사이클(Limit Cycle) 진동을 원천 차단함.
            %    - Z 잔차는 기저 Z 스케줄링(1/cos(phi_eq))과 동일하게 1/cos(phi_eq) 스케일을 곱해 대각도 감쇠력 보존.
            cos_phi_eq = max(cos(phi_eq), 0.5);
            omega_turn = v_xy * abs(kappa_des);
            lambda_lat = (1.0 / (cos_phi_eq^2)) * (1.0 + 0.4 * omega_turn);
            s_xy       = max(0.15, min(1.0, (2.50 - lambda_lat) / 0.60));
            inv_cos_z  = 1.0 / cos_phi_eq;

            dP_xy_rad = 0.05 * (obj.offset_P_xy / 0.8) * s_xy;
            dD_xy_rad = 0.16 * (max(0.0, obj.offset_D_xy) / 1.0) * s_xy;
            dP_z_N    = 1.50 * (min(0.0, obj.offset_P_z) / 1.5) * inv_cos_z;
            dD_z_N    = 1.50 * (max(0.0, obj.offset_D_z) / 1.0) * inv_cos_z;

            kP_xy = max(obj.lpv_a1 / obj.g + dP_xy_rad, 0.05);
            kI_xy = obj.lpv_a0 / obj.g;
            kD_xy = max(obj.lpv_a2 / obj.g + dD_xy_rad, 0.05);

            if obj.use_z_scheduling && obj.use_feedforward
                cos_phi = max(cos(phi_eq), 0.5);
            else
                cos_phi = 1.0;
            end

            % 물리 힘 단위 [N/m] Z 게인 (= ChoeJae PositionCtrl.m 의 kP_z = lpv_a1 / b_z + offset 과 100% 동일!)
            b_z  = cos_phi / obj.m;
            kP_z = max(obj.lpv_a1 / b_z + dP_z_N, 0.5);
            kI_z = obj.lpv_a0 / b_z;
            kD_z = max(obj.lpv_a2 / b_z + dD_z_N, 0.2);

            % run_px4_mission.m 253행에서 GetZ_PID() * cfg.m 으로 기록하므로 m 으로 나누어 저장
            obj.current_kP_z = kP_z / obj.m;
            obj.current_kI_z = kI_z / obj.m;
            obj.current_kD_z = kD_z / obj.m;

            % 7. 오차 피드백 및 최종 명령 산출 (ChoeJae PositionCtrl.m 동일)
            delta_theta = -(kP_xy * ex_body + kI_xy * ex_body_sum + kD_xy * dex_body);
            delta_phi   =  (kP_xy * ey_body + kI_xy * ey_body_sum + kD_xy * dey_body);
            delta_U1    = -(kP_z  * obj.z_err + kI_z * obj.z_err_sum + kD_z * dz_err);

            theta_eq = 0.0;
            if obj.eq_delay_comp && obj.use_feedforward
                q_eq     = abs(v_xy * kappa_des) * sin(abs(phi_eq));
                theta_eq = q_eq / obj.K_att;
            end

            theta_raw = theta_eq + delta_theta;
            phi_raw   = phi_eq   + delta_phi;
            U1_raw    = U1_eq    + delta_U1;

            theta_cmd = max(-obj.max_angle, min(theta_raw, obj.max_angle));
            phi_cmd   = max(-obj.max_angle, min(phi_raw,   obj.max_angle));
            obj.is_angle_saturated = (abs(theta_raw) > obj.max_angle) || (abs(phi_raw) > obj.max_angle);

            U1_phys = max(0.0, U1_raw);

            % 8. 제곱근 추력 정규화 (ChoeJae run_px4_mission.m 154행과 100% 동일!) 및 Z 적분 갱신
            %    - 35°/40°에서도 실제 전송 추력(sqrt)이 [0.1, 0.9] 안에 있으면 정상 적분하여 E_ss ≈ 0 보장
            thr_raw = obj.mpc_thr_hover * sqrt(max(U1_phys / (obj.m * obj.g), 0.0));
            thr_cmd = max(0.1, min(thr_raw, 0.9));

            if thr_raw >= 0.1 && thr_raw <= 0.95
                obj.z_err_sum = obj.z_err_sum + obj.z_err * dt;
            end

            obj.x_err_prev = obj.x_err;
            obj.y_err_prev = obj.y_err;
            obj.z_err_prev = obj.z_err;

            att_cmd = [phi_cmd; theta_cmd; psi_des; thr_cmd];
        end
    end
end
