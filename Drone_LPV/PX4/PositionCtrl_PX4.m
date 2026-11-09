classdef PositionCtrl_PX4 < handle
    % POSITIONCTRL_PX4 50 Hz 2-DOF (D-on-Measurement + 속도/가속도 Feedforward) 위치 제어기
    % - PX4의 느린 자세 루프(a ≈ 4.0 rad/s)와 결합 시 발생하는 D-on-Error 유한 영점 오버슈트 및
    %   2.4초 주기 발산 문제를 근본 해결한 2-DOF 구조
    % - 제어 법칙:
    %     a_fb  = Kp .* (p_des - p_est) + Ki .* int_e + Kd .* (k_ff_v .* v_des - v_est)
    %     a_cmd = a_fb + k_ff_a .* a_des
    % - 각도 클리핑: max_angle = deg2rad(30)
    % - Anti-windup: 각도/추력 포화 시 조건부 적분 정지 및 적분항 클리핑
    % - 추력 정규화: thr_cmd = clip(MPC_THR_HOVER * (U1 / (m * g)), 0.1, 0.9)

    properties
        g
        m
        dt_pos              % 위치 제어기 고정 주기: 0.02 s (50 Hz)
        mpc_thr_hover       % 호버링 정규화 추력: 0.6

        % 2-DOF 게인 (가속도 단위 [m/s^2] 기준: 각도 게인 * g)
        Kp                  % [Kp_x; Kp_y; Kp_z]
        Ki                  % [Ki_x; Ki_y; Ki_z]
        Kd                  % [Kd_x; Kd_y; Kd_z]

        % 피드포워드 가중치 스위치 (0.0 또는 1.0 등 파라미터화)
        k_ff_v              % 목표 속도 피드포워드 계수 (기본 1.0)
        k_ff_a              % 목표 가속도 피드포워드 계수 (기본 1.0)
        use_z_scheduling    % 기울기(cos(phi)*cos(theta)) 기반 Z축 게인/추력 스케줄링 여부

        % 적분 상태 및 Anti-windup 제한
        int_e               % 오차 적분 누적값 [int_ex; int_ey; int_ez]
        int_max             % 적분항 최대 한계 [m*s]
        max_angle           % 최대 롤/피치 각도 제한: deg2rad(30)

        % PPO 잔차 게인 오프셋 호환용
        offset_P_xy = 0.0
        offset_D_xy = 0.0
        offset_P_z  = 0.0
        offset_D_z  = 0.0

        % 로깅용 현재 Z축 PID 게인 및 포화 플래그
        current_kP_z
        current_kI_z
        current_kD_z
        is_angle_saturated = false
    end

    methods
        function obj = PositionCtrl_PX4(cfg)
            obj.g             = 9.81;
            obj.m             = cfg.m;             % 2.064 kg
            obj.dt_pos        = cfg.dt_pos;        % 0.02 s (50 Hz)
            obj.mpc_thr_hover = cfg.MPC_THR_HOVER; % 0.6

            % 기본 안정 게인 세팅:
            % a = 4.0 rad/s 기준 위상 여유 >= 40 deg, 오버슈트 Mp <= 4% 만족 보수적 게인
            % 각도 환산 기준: Kp_ang = 0.10, Ki_ang = 0.001, Kd_ang = 0.17
            % 가속도 단위 환산: Kp = Kp_ang * g, Ki = Ki_ang * g, Kd = Kd_ang * g
            if isfield(cfg, 'pos_Kp_ang')
                kp_xy = cfg.pos_Kp_ang * obj.g;
                ki_xy = cfg.pos_Ki_ang * obj.g;
                kd_xy = cfg.pos_Kd_ang * obj.g;
            else
                kp_xy = 0.10  * obj.g; % 0.9810 (m/s^2)/m
                ki_xy = 0.001 * obj.g; % 0.0098 (m/s^2)/(m*s)
                kd_xy = 0.17  * obj.g; % 1.6677 (m/s^2)/(m/s)
            end

            if isfield(cfg, 'pos_Kp_z')
                kp_z = cfg.pos_Kp_z;
                ki_z = cfg.pos_Ki_z;
                kd_z = cfg.pos_Kd_z;
            else
                kp_z = 4.0;
                ki_z = 0.2;
                kd_z = 3.5;
            end

            obj.Kp = [kp_xy; kp_xy; kp_z];
            obj.Ki = [ki_xy; ki_xy; ki_z];
            obj.Kd = [kd_xy; kd_xy; kd_z];

            if isfield(cfg, 'k_ff_v'), obj.k_ff_v = cfg.k_ff_v; else, obj.k_ff_v = 1.0; end
            if isfield(cfg, 'k_ff_a'), obj.k_ff_a = cfg.k_ff_a; else, obj.k_ff_a = 1.0; end
            if isfield(cfg, 'Z_Gain_Scheduling')
                obj.use_z_scheduling = cfg.Z_Gain_Scheduling;
            else
                obj.use_z_scheduling = true;
            end

            if isfield(cfg, 'max_angle_deg')
                obj.max_angle = deg2rad(cfg.max_angle_deg);
            else
                obj.max_angle = deg2rad(30); % 요구사항 4: 30도 클리핑
            end

            obj.int_max = [5.0; 5.0; 5.0];
            obj.Reset();
        end

        function Reset(obj)
            obj.int_e              = zeros(3, 1);
            obj.current_kP_z       = obj.Kp(3);
            obj.current_kI_z       = obj.Ki(3);
            obj.current_kD_z       = obj.Kd(3);
            obj.is_angle_saturated = false;
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

        function [att_cmd, U1_phys] = Update(obj, p_des, v_des, a_des, psi_des, p_est, v_est, euler_est)
            % 입력:
            %   p_des, v_des, a_des: 3x1 목표 위치, 속도, 가속도 (NED)
            %   psi_des: 목표 Yaw 각도 [rad]
            %   p_est, v_est: 3x1 추정(또는 참값) 위치 및 속도 (NED)
            %   euler_est: [phi; theta; psi] 현재 자세 각도 [rad]
            % 출력:
            %   att_cmd: [phi_cmd; theta_cmd; psi_cmd; thr_cmd] (thr_cmd는 [0.1, 0.9] 정규화 추력)
            %   U1_phys: 물리적 목표 총 추력 [N]

            psi = euler_est(3);

            % 1. 위치 오차 계산
            e_p = p_des - p_est;

            % 2. 유효 게인 산출 (PPO 잔차 오프셋 및 Z축 기울기 스케줄링 반영)
            eff_Kp = [
                max(0.1, obj.Kp(1) + obj.offset_P_xy);
                max(0.1, obj.Kp(2) + obj.offset_P_xy);
                max(0.5, obj.Kp(3) + obj.offset_P_z)
            ];
            eff_Ki = obj.Ki;
            eff_Kd = [
                max(0.1, obj.Kd(1) + obj.offset_D_xy);
                max(0.1, obj.Kd(2) + obj.offset_D_xy);
                max(0.2, obj.Kd(3) + obj.offset_D_z)
            ];

            % 3. 2-DOF 제어 법칙 (D-on-Measurement + 속도 피드포워드 + 가속도 피드포워드)
            a_fb  = eff_Kp .* e_p + eff_Ki .* obj.int_e + eff_Kd .* (obj.k_ff_v .* v_des - v_est);
            a_cmd = a_fb + obj.k_ff_a .* a_des;

            % 4. 수평 가속도 명령 -> Body 프레임(전방/우측) 회전 -> 롤/피치 각도 변환
            a_fwd =  cos(psi) * a_cmd(1) + sin(psi) * a_cmd(2);
            a_rgt = -sin(psi) * a_cmd(1) + cos(psi) * a_cmd(2);

            theta_raw = atan2(-a_fwd, obj.g);
            phi_raw   = atan2( a_rgt * cos(theta_raw), obj.g);

            % 5. 최대 기울기 각도 제한 (max_angle = deg2rad(30))
            theta_cmd = max(-obj.max_angle, min(theta_raw, obj.max_angle));
            phi_cmd   = max(-obj.max_angle, min(phi_raw,   obj.max_angle));

            % 6. Anti-windup (조건부 적분: 각도 포화 시 해당 축 적분 정지)
            sat_xy = (abs(theta_raw) > obj.max_angle) || (abs(phi_raw) > obj.max_angle);
            obj.is_angle_saturated = sat_xy;
            if ~sat_xy
                obj.int_e(1:2) = obj.int_e(1:2) + e_p(1:2) * obj.dt_pos;
                obj.int_e(1:2) = max(-obj.int_max(1:2), min(obj.int_e(1:2), obj.int_max(1:2)));
            end

            % 7. Z축 추력 산출 및 기울기 보상 (NED: +Z가 아래쪽이므로 위쪽 추력은 g - a_z_cmd)
            if obj.use_z_scheduling
                tilt_factor = max(cos(phi_cmd) * cos(theta_cmd), 0.5);
            else
                tilt_factor = 1.0;
            end

            obj.current_kP_z = eff_Kp(3) / tilt_factor;
            obj.current_kI_z = eff_Ki(3) / tilt_factor;
            obj.current_kD_z = eff_Kd(3) / tilt_factor;

            U1_raw = (obj.m * (obj.g - a_cmd(3))) / tilt_factor;
            U1_phys = max(0.0, U1_raw);

            % 8. 정규화 추력 변환 및 [0.1, 0.9] 클리핑 + Z축 Anti-windup
            thr_raw = obj.mpc_thr_hover * (U1_phys / (obj.m * obj.g));
            thr_cmd = max(0.1, min(thr_raw, 0.9));

            if thr_raw >= 0.1 && thr_raw <= 0.9
                obj.int_e(3) = obj.int_e(3) + e_p(3) * obj.dt_pos;
                obj.int_e(3) = max(-obj.int_max(3), min(obj.int_e(3), obj.int_max(3)));
            end

            att_cmd = [phi_cmd; theta_cmd; psi_des; thr_cmd];
        end
    end
end

