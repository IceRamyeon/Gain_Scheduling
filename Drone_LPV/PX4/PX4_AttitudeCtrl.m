classdef PX4_AttitudeCtrl < handle
    % PX4_ATTITUDECTRL PX4 mc_att_control & mc_rate_control 이중 루프 자세 제어기 모사
    %
    % 지원 모드 (cfg.att_mode):
    %   - Mode 1 (시스템 식별 등가 모델):
    %       * 1차 지연 모델 (cfg.sysid_order == 1): G(s) = a / (s + a), 기본 a = 4.0 rad/s
    %       * 2차 캐스케이드 모델 (cfg.sysid_order == 2): G(s) = K_att / (tau_rate*s^2 + s + K_att),
    %         기본 K_att = 4.0, tau_rate = 0.04 s
    %       * 내부 목표 각속도는 반드시 MC_ROLLRATE_MAX = deg2rad(220) (3.84 rad/s)로 Saturation
    %   - Mode 2 (전체 이중 루프 모델, 기본값):
    %       * 외부 각도 루프: MC_ROLL_P = 4.0, MC_PITCH_P = 4.0, MC_YAW_P = 2.8 -> ±220 deg/s 클리핑
    %       * 내부 각속도 루프 (250 Hz): 각속도 오차 PID (P = 0.15, I = 0.2, D = 0.003, 관성모멘트 스케일링 반영)

    properties
        dt                  % 250 Hz (0.004 s)
        att_mode            % 1: 시스템 식별 등가 모델, 2: 전체 이중 루프 모델
        sysid_order         % Mode 1 세부 옵션 (1: 1차 지연, 2: 2차 캐스케이드)

        % 기체 관성모멘트 및 토크 스케일링
        Ixx, Iyy, Izz
        torque_scale        % 정규화 각속도 PID 출력 -> 물리 토크 [N*m] 변환 스케일 (관성모멘트 반영)

        % Mode 1 파라미터
        sysid_a             % 1차 모델 대역폭 (기본 4.0 rad/s)
        K_att               % 2차 캐스케이드 외부 게인 (기본 4.0)
        tau_rate            % 2차 캐스케이드 내부 각속도 시정수 (기본 0.04 s)
        ref_euler           % Mode 1 1차 참조 모델 내부 상태 [phi_ref; theta_ref; psi_ref]

        % Mode 2 외부 각도 P 게인
        MC_ROLL_P
        MC_PITCH_P
        MC_YAW_P

        % 각속도 제한 (Saturation)
        MC_ROLLRATE_MAX     % deg2rad(220) = 3.8397 rad/s
        MC_PITCHRATE_MAX    % deg2rad(220) = 3.8397 rad/s
        MC_YAWRATE_MAX      % deg2rad(200) = 3.4907 rad/s

        % Mode 2 내부 각속도 PID 게인 (PX4 표준 정규화 게인)
        rate_P              % [0.15; 0.15; 0.20]
        rate_I              % [0.20; 0.20; 0.10]
        rate_D              % [0.003; 0.003; 0.0]

        % 내부 각속도 루프 상태 변수
        rate_sp             % 목표 각속도 [p_sp; q_sp; r_sp] (rad/s)
        rate_int            % 각속도 오차 적분항 (3x1)
        prev_rates          % 이전 스텝 기체 각속도 [p; q; r] (D-on-Measurement용)
        d_rate_filt         % 각가속도 LPF 상태 (3x1)
        is_first_run
    end

    methods
        function obj = PX4_AttitudeCtrl(cfg)
            obj.dt = cfg.dt_sim; % 0.004 s (250 Hz)

            if isfield(cfg, 'att_mode'), obj.att_mode = cfg.att_mode; else, obj.att_mode = 2; end
            if isfield(cfg, 'sysid_order'), obj.sysid_order = cfg.sysid_order; else, obj.sysid_order = 2; end

            obj.Ixx = cfg.Ixx;
            obj.Iyy = cfg.Iyy;
            obj.Izz = cfg.Izz;

            % Mode 1 파라미터
            if isfield(cfg, 'sysid_a'),  obj.sysid_a  = cfg.sysid_a;  else, obj.sysid_a  = 4.0;  end
            if isfield(cfg, 'K_att'),    obj.K_att    = cfg.K_att;    else, obj.K_att    = 4.0;  end
            if isfield(cfg, 'tau_rate'), obj.tau_rate = cfg.tau_rate; else, obj.tau_rate = 0.04; end

            % Mode 2 외부 루프 P 게인
            obj.MC_ROLL_P  = 4.0;
            obj.MC_PITCH_P = 4.0;
            obj.MC_YAW_P   = 2.8;

            % 각속도 Saturation 제한: ±220 deg/s (3.84 rad/s)
            obj.MC_ROLLRATE_MAX  = deg2rad(220);
            obj.MC_PITCHRATE_MAX = deg2rad(220);
            obj.MC_YAWRATE_MAX   = deg2rad(200);

            % Mode 2 내부 각속도 루프 PID 게인 (PX4 표준값)
            obj.rate_P = [0.15;  0.15;  0.20];
            obj.rate_I = [0.20;  0.20;  0.10];
            obj.rate_D = [0.003; 0.003; 0.00];

            % 관성모멘트 스케일링:
            % P_rate = 0.15 일 때 내부 각속도 폐루프 시정수가 tau_rate = 0.04 s (대역폭 25 rad/s)와
            % 물리적으로 일치하도록 관성모멘트 기반 토크 스케일링 팩터 산출:
            % k_torque = I / (P_rate * tau_rate) -> x500 기준 Roll/Pitch 약 3.61 N*m
            obj.torque_scale = [
                obj.Ixx / (obj.rate_P(1) * obj.tau_rate);
                obj.Iyy / (obj.rate_P(2) * obj.tau_rate);
                obj.Izz / (obj.rate_P(3) * 0.08)
            ];

            obj.Reset();
        end

        function Reset(obj)
            obj.ref_euler    = zeros(3, 1);
            obj.rate_sp      = zeros(3, 1);
            obj.rate_int     = zeros(3, 1);
            obj.prev_rates   = zeros(3, 1);
            obj.d_rate_filt  = zeros(3, 1);
            obj.is_first_run = true;
        end

        function ctrl_out = Update(obj, att_cmd, state)
            % 입력:
            %   att_cmd: [phi_cmd; theta_cmd; psi_cmd; thr_cmd]
            %   state:   12x1 드론 상태 벡터
            % 출력:
            %   ctrl_out: [thr_cmd; Mx_cmd; My_cmd; Mz_cmd]

            phi_cmd   = att_cmd(1);
            theta_cmd = att_cmd(2);
            psi_cmd   = att_cmd(3);
            thr_cmd   = att_cmd(4);

            phi   = state(7);
            theta = state(8);
            psi   = state(9);
            p     = state(10);
            q     = state(11);
            r     = state(12);
            rates = [p; q; r];

            if obj.is_first_run
                obj.ref_euler    = [phi; theta; psi];
                obj.prev_rates   = rates;
                obj.is_first_run = false;
            end

            % Yaw 오차 래핑 (-pi ~ pi)
            e_psi = psi_cmd - psi;
            e_psi = atan2(sin(e_psi), cos(e_psi));

            if obj.att_mode == 1
                %% [Mode 1] 시스템 식별 등가 모델 (1차 지연 또는 2차 캐스케이드)
                if obj.sysid_order == 1
                    % 1차 지연 모델 G(s) = a / (s + a)
                    % 참조 각도 상태(ref_euler)의 미분(=목표 각속도)을 ±220 deg/s로 클리핑
                    e_ref_psi = atan2(sin(psi_cmd - obj.ref_euler(3)), cos(psi_cmd - obj.ref_euler(3)));
                    p_ref = max(-obj.MC_ROLLRATE_MAX,  min(obj.sysid_a * (phi_cmd - obj.ref_euler(1)),   obj.MC_ROLLRATE_MAX));
                    q_ref = max(-obj.MC_PITCHRATE_MAX, min(obj.sysid_a * (theta_cmd - obj.ref_euler(2)), obj.MC_PITCHRATE_MAX));
                    r_ref = max(-obj.MC_YAWRATE_MAX,   min(obj.MC_YAW_P * e_ref_psi,                     obj.MC_YAWRATE_MAX));

                    obj.ref_euler = obj.ref_euler + [p_ref; q_ref; r_ref] * obj.dt;
                    obj.rate_sp   = [p_ref; q_ref; r_ref];

                    % 플랜트가 1차 참조 모델 궤적(ref_euler, rate_sp)을 추종하도록 토크 생성
                    tau_track = obj.tau_rate;
                    e_track_psi = atan2(sin(obj.ref_euler(3) - psi), cos(obj.ref_euler(3) - psi));
                    alpha_des = [
                        (p_ref - p) / tau_track + (obj.ref_euler(1) - phi)   / (tau_track * 0.25);
                        (q_ref - q) / tau_track + (obj.ref_euler(2) - theta) / (tau_track * 0.25);
                        (r_ref - r) / 0.08      + e_track_psi                / (0.08 * 0.25)
                    ];
                else
                    % 2차 캐스케이드 등가 모델 G(s) = K_att / (tau_rate * s^2 + s + K_att)
                    p_sp = max(-obj.MC_ROLLRATE_MAX,  min(obj.K_att * (phi_cmd - phi),     obj.MC_ROLLRATE_MAX));
                    q_sp = max(-obj.MC_PITCHRATE_MAX, min(obj.K_att * (theta_cmd - theta), obj.MC_PITCHRATE_MAX));
                    r_sp = max(-obj.MC_YAWRATE_MAX,   min(obj.MC_YAW_P * e_psi,            obj.MC_YAWRATE_MAX));
                    obj.rate_sp = [p_sp; q_sp; r_sp];

                    % 내부 각속도 1차 시정수 tau_rate 응답: dot(omega) = (omega_sp - omega) / tau_rate
                    alpha_des = [
                        (p_sp - p) / obj.tau_rate;
                        (q_sp - q) / obj.tau_rate;
                        (r_sp - r) / 0.08
                    ];
                end

                I_mat = diag([obj.Ixx, obj.Iyy, obj.Izz]);
                M_cmd = I_mat * alpha_des + cross(rates, I_mat * rates);

            else
                %% [Mode 2] 전체 이중 루프 모델 (mc_att_control + mc_rate_control)
                % 1) 외부 각도 P 루프 + 목표 각속도 ±220 deg/s 클리핑
                p_sp = max(-obj.MC_ROLLRATE_MAX,  min(obj.MC_ROLL_P  * (phi_cmd - phi),     obj.MC_ROLLRATE_MAX));
                q_sp = max(-obj.MC_PITCHRATE_MAX, min(obj.MC_PITCH_P * (theta_cmd - theta), obj.MC_PITCHRATE_MAX));
                r_sp = max(-obj.MC_YAWRATE_MAX,   min(obj.MC_YAW_P   * e_psi,               obj.MC_YAWRATE_MAX));
                obj.rate_sp = [p_sp; q_sp; r_sp];

                % 2) 내부 각속도 PID 루프 (250 Hz)
                rate_err = obj.rate_sp - rates;

                % 각가속도 수치 미분 (D-on-Measurement + 80 Hz LPF)
                raw_d_rate = (rates - obj.prev_rates) / obj.dt;
                alpha_d = obj.dt / (0.002 + obj.dt);
                obj.d_rate_filt = (1 - alpha_d) * obj.d_rate_filt + alpha_d * raw_d_rate;
                obj.prev_rates  = rates;

                % 적분항 업데이트 및 Anti-windup 클리핑
                obj.rate_int = obj.rate_int + (obj.rate_I .* rate_err) * obj.dt;
                obj.rate_int = max(-0.3, min(obj.rate_int, 0.3));

                % 정규화 토크 출력 [-1, 1]
                u_norm = obj.rate_P .* rate_err + obj.rate_int - obj.rate_D .* obj.d_rate_filt;
                u_norm = max(-1.0, min(u_norm, 1.0));

                % 관성모멘트 스케일링 반영하여 물리 토크 [N*m] 산출
                M_cmd = obj.torque_scale .* u_norm;
            end

            ctrl_out = [thr_cmd; M_cmd(1); M_cmd(2); M_cmd(3)];
        end
    end
end

