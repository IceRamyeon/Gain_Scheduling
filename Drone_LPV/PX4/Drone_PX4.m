classdef Drone_PX4 < handle
    % DRONE_PX4 Gazebo SITL (x500) 등가 6-DOF 멀티로터 플랜트 모델
    % - Gazebo x500 물리 파라미터 (m = 2.064 kg, L = 0.246 m, MPC_THR_HOVER = 0.6)
    % - 4-로터 X-Configuration 믹서 할당 및 하드 클리핑 (로터당 최대 8.6 N, 합산 34.4 N)
    % - 비대칭 1차 지연 모터 동역학 (tau_up = 0.0125 s, tau_down = 0.0250 s)
    % - 선형 공기항력 (Rotor Drag: -diag([dx, dy, dz]) * v_ned)

    properties
        g
        t
        dt
        tf

        % 기체 물리량 (Gazebo x500 기준)
        m
        l               % 암 길이 L = 0.246 m
        l_xy            % 로터 X, Y 모멘트 팔 길이 = L / sqrt(2) = 0.174 m
        I               % 관성모멘트 행렬 (3x3)
        mpc_thr_hover   % 호버링 정규화 추력 (0.6)

        % 모터 및 믹서 파라미터
        f_max_per_rotor % 로터당 최대 추력 (8.6 N, 4개 합산 34.4 N)
        km_yaw          % 로터 추력 대비 반토크 계수 [m]
        tau_up          % 모터 가속 시정수 (0.0125 s)
        tau_down        % 모터 감속 시정수 (0.0250 s)

        % 모터 추력 상태 (4x1)
        f_cmd           % 믹서 클리핑 후 목표 로터 추력 [N]
        f_act           % 비대칭 지연 필터 통과 후 실제 로터 추력 [N]
        is_saturated    % 로터 포화 발생 여부 플래그 (0 또는 1)

        % 선형 공기항력 계수 (Rotor Drag)
        drag_coeff      % [dx; dy; dz] 기본값 [0.1; 0.1; 0.2]

        % 12-상태 벡터: [X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r]'
        x
        r               % [X; Y; Z] (NED)
        dr              % [dX; dY; dZ] (NED)
        euler           % [phi; theta; psi]
        w               % [p; q; r] (Body)
        dx              % 상태 미분 벡터 (12x1)

        % 실제 플랜트에 인가된 제어력 및 모멘트
        u               % [U1_act; M1_act; M2_act; M3_act]
        T               % 실제 총 추력 U1_act [N]
        M               % 실제 모멘트 [M1; M2; M3] [N*m]

        % 하위 모듈 참조
        trajCtrl
        posCtrl
        attCtrl
        sensorEKF
    end

    methods
        function obj = Drone_PX4(cfg, waypoints)
            obj.g  = 9.81;
            obj.t  = 0.0;
            obj.dt = cfg.dt_sim; % 기본 250 Hz (0.004 s)

            % 1. Gazebo x500 물리 파라미터 초기화
            obj.m    = cfg.m;             % 2.064 kg
            obj.l    = cfg.armLength;     % 0.246 m
            obj.l_xy = obj.l / sqrt(2);   % 0.174 m (±0.174, ±0.174)
            obj.I    = diag([cfg.Ixx, cfg.Iyy, cfg.Izz]);
            obj.mpc_thr_hover = cfg.MPC_THR_HOVER; % 0.6

            % 2. 믹서 및 비대칭 모터 시정수 설정
            obj.f_max_per_rotor = cfg.f_max_per_rotor; % 8.6 N
            obj.km_yaw          = cfg.km_yaw;          % 0.05 m
            obj.tau_up          = cfg.tau_up;          % 0.0125 s
            obj.tau_down        = cfg.tau_down;        % 0.0250 s

            % 3. 선형 공기항력(Rotor Drag) 설정
            obj.drag_coeff = cfg.drag_coeff(:); % [0.1; 0.1; 0.2]

            % 4. 초기 상태 및 초기 호버링 로터 추력 세팅
            obj.x     = cfg.drone_init_states(:);
            obj.r     = obj.x(1:3);
            obj.dr    = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w     = obj.x(10:12);
            obj.dx    = zeros(12, 1);

            f_hover = (obj.m * obj.g) / 4.0; % 로터당 호버링 추력 (~5.062 N)
            obj.f_cmd = f_hover * ones(4, 1);
            obj.f_act = f_hover * ones(4, 1);
            obj.is_saturated = false;

            obj.T = obj.m * obj.g;
            obj.M = zeros(3, 1);
            obj.u = [obj.T; obj.M];

            % 5. 궤적 생성기, 위치 제어기, 자세 제어기, 센서 모듈 초기화
            obj.trajCtrl  = TrajectoryPlanner(obj.r, waypoints, cfg.target_speed);
            obj.tf        = obj.trajCtrl.tf;
            obj.posCtrl   = PositionCtrl_PX4(cfg);
            obj.attCtrl   = PX4_AttitudeCtrl(cfg);
            obj.sensorEKF = PX4_Sensor_EKF(cfg, obj.r, obj.dr);
        end

        function state = GetState(obj)
            state = obj.x;
        end

        function [u_actual, f_cmd_out, f_act_out] = MixerAndMotorDynamics(obj, ctrl_in)
            % ctrl_in: [thr_cmd; Mx_cmd; My_cmd; Mz_cmd]
            %   - thr_cmd: 정규화 추력 명령 [0.1, 0.9]
            %   - Mx_cmd, My_cmd, Mz_cmd: 자세 제어기에서 출력된 목표 모멘트 [N*m]

            thr_cmd = max(0.1, min(ctrl_in(1), 0.9));
            Mx_cmd  = ctrl_in(2);
            My_cmd  = ctrl_in(3);
            Mz_cmd  = ctrl_in(4);

            % 정규화 추력 명령을 목표 총 추력 [N]으로 변환
            % thr_cmd == MPC_THR_HOVER(0.6) 일 때 정확히 m*g (20.248 N) 발생
            T_cmd = (thr_cmd / obj.mpc_thr_hover) * (obj.m * obj.g);

            % 4-로터 X-Configuration 역믹서 할당 (Inverse Control Allocation)
            % Rotor 1: Front-Left  (+x, -y), CCW (+yaw torque)
            % Rotor 2: Back-Left   (-x, -y), CW  (-yaw torque)
            % Rotor 3: Back-Right  (-x, +y), CCW (+yaw torque)
            % Rotor 4: Front-Right (+x, +y), CW  (-yaw torque)
            f_base = T_cmd / 4.0;
            df_roll  = Mx_cmd / (4.0 * obj.l_xy);
            df_pitch = My_cmd / (4.0 * obj.l_xy);
            df_yaw   = Mz_cmd / (4.0 * obj.km_yaw);

            f_raw = [
                f_base + df_roll + df_pitch + df_yaw;
                f_base + df_roll - df_pitch - df_yaw;
                f_base - df_roll - df_pitch + df_yaw;
                f_base - df_roll + df_pitch - df_yaw
            ];

            % 로터당 [0, 8.6 N] 하드 클리핑 (합산 최대 34.4 N)
            obj.is_saturated = any(f_raw < 0.0 | f_raw > obj.f_max_per_rotor);
            obj.f_cmd = max(0.0, min(f_raw, obj.f_max_per_rotor));

            % 비대칭 1차 지연 모터 동역학 (gazebo_motor_model 모사)
            for idx = 1:4
                if obj.f_cmd(idx) >= obj.f_act(idx)
                    tau = obj.tau_up;   % 가속 시정수 (0.0125 s)
                else
                    tau = obj.tau_down; % 감속 시정수 (0.0250 s)
                end
                alpha_m = obj.dt / (tau + obj.dt);
                obj.f_act(idx) = obj.f_act(idx) + alpha_m * (obj.f_cmd(idx) - obj.f_act(idx));
            end

            % 지연 필터를 거친 실제 로터 추력 f_act(1:4)로부터 실제 U1 및 [M1, M2, M3] 재합산
            U1_act = sum(obj.f_act);
            M1_act = obj.l_xy * ( obj.f_act(1) + obj.f_act(2) - obj.f_act(3) - obj.f_act(4));
            M2_act = obj.l_xy * ( obj.f_act(1) - obj.f_act(2) - obj.f_act(3) + obj.f_act(4));
            M3_act = obj.km_yaw * (obj.f_act(1) - obj.f_act(2) + obj.f_act(3) - obj.f_act(4));

            u_actual  = [U1_act; M1_act; M2_act; M3_act];
            f_cmd_out = obj.f_cmd;
            f_act_out = obj.f_act;
        end

        function EvalEOM(obj)
            bRi = RPY2Rot(obj.euler);
            R = bRi'; % Body to NED 회전행렬

            % 1. 위치 미분
            obj.dx(1:3) = obj.dr;

            % 2. 선형 공기항력 (Rotor Drag: -diag([dx, dy, dz]) * v_ned)
            f_drag_ned = -obj.drag_coeff .* obj.dr;

            % 3. 병진 운동방정식 (중력 + 로터 추력 + 선형 항력)
            obj.dx(4:6) = (1.0 / obj.m) * ([0; 0; obj.m * obj.g] + R * [0; 0; -obj.T] + f_drag_ned);

            % 4. 오일러 각 미분
            phi   = obj.euler(1);
            theta = obj.euler(2);
            % 수치 특이점 보호 (cos(theta) 근처)
            cos_th = sign(cos(theta)) * max(abs(cos(theta)), 1e-3);
            tan_th = sin(theta) / cos_th;
            sec_th = 1.0 / cos_th;

            W_mat = [1, sin(phi)*tan_th, cos(phi)*tan_th;
                     0, cos(phi),        -sin(phi);
                     0, sin(phi)*sec_th, cos(phi)*sec_th];
            obj.dx(7:9) = W_mat * obj.w;

            % 5. 회전 운동방정식
            obj.dx(10:12) = obj.I \ (obj.M - cross(obj.w, obj.I * obj.w));
        end

        function [u_actual, f_cmd_out, f_act_out] = UpdateState(obj, ctrl_in)
            % 1. 믹서 할당 및 비대칭 모터 동역학 계산
            [u_actual, f_cmd_out, f_act_out] = obj.MixerAndMotorDynamics(ctrl_in);

            obj.u = u_actual;
            obj.T = u_actual(1);
            obj.M = u_actual(2:4);

            % 2. 250 Hz 물리 동역학 적분
            obj.t = obj.t + obj.dt;
            obj.EvalEOM();
            obj.x = obj.x + obj.dx .* obj.dt;

            obj.r     = obj.x(1:3);
            obj.dr    = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w     = obj.x(10:12);
        end
    end
end

