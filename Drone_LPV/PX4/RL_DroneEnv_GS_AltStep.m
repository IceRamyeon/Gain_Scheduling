classdef RL_DroneEnv_GS_AltStep < rl.env.MATLABEnvironment
    % RL_DRONEENV_GS_ALTSTEP (B안) ChoeJae PositionCtrl_GS 기반 원 운동 중 고도 계단 추종 PPO 환경
    % - 기반 제어기: ChoeJae PX4/PositionCtrl (Z_D_on_measurement=true, Err_Yaw='desired',
    %               Feedforward=true, Z_Gain_Scheduling=true, max_angle_deg=50)
    % - 추력 정규화: ChoeJae f8ccf1f 동일 u = clip(0.73 * sqrt(U1 / mg), 0.1, 0.9)
    % - 플랜트:      Drone_PX4 (Gazebo x500, 제곱 추력 T = mg*(u/0.73)^alpha, 250 Hz PX4_AttitudeCtrl)
    % - 경로:        CircleTrajectory (정지 출발 -> 가속 -> 등속 직선 -> 원 운동 중 고도 계단)
    % - 관측 (11x1, ChoeJae run_px4_mission.m 203행과 동일한 World NED):
    %     [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta; v_des; kappa_now; kappa_spatial]
    % - 행동 (4x1): [a_P_xy; a_D_xy; a_P_z; a_D_z] in [-1, 1]^4

    properties
        cfg
        drone_px4
        posCtrl
        traj
        current_step
        max_steps

        % 속력 반비례 공간 예견 거리 (ChoeJae cfg.ppo.lookahead_L = 2.0)
        lookahead_L = 2.0;

        % PositionCtrl_GS 단위(XY: rad/m, Z: N/m)에 맞춘 잔차 게인 스케일
        % (PositionCtrl_PX4 에서도 run_px4_mission 의 range_[0.8, 1.0, 1.5, 1.0] 입력이 이 값과 1:1 동일하게 매핑됨)
        range_P_xy = 0.05;
        range_D_xy = 0.16;
        range_P_z  = 1.5;
        range_D_z  = 1.5;

        % 다중 주기 & FIFO 지연 버퍼 상태
        prev_yaw
        prev_action
        prev_u_moment
        prev_U1_des
        zoh_att_cmd
        fifo_buffer
        buf_idx
        delay_steps
        last_p_est
        last_v_est

        % 도메인 무작위화 옵션
        randomize_mission    = true;
        randomize_delay      = true;
        delay_steps_range    = [8, 20];     % 32~80 ms @ 250 Hz
        randomize_thrust_exp = true;
        thrust_exp_range     = [1.75, 2.05];
        randomize_att_loop   = true;
        mc_att_p_range       = [5.5, 7.5];  % ChoeJae SITL 확인값 K_att = MC_ROLL/PITCH_P = 6.5 중심 무작위화
        tau_rate_range       = [0.03, 0.05];
        max_tilt_deg         = 60.0;        % ChoeJae abort_tilt_deg = 60

        episode_count = 0;
    end

    methods
        function this = RL_DroneEnv_GS_AltStep(use_sensor_noise_in)
            obsInfo = rlNumericSpec([11 1]);
            obsInfo.Name = 'PX4 GS Altitude Step States (World NED)';

            actInfo = rlNumericSpec([4 1], 'LowerLimit', -1, 'UpperLimit', 1);
            actInfo.Name = 'PositionCtrl_GS Residual Gain Offsets';

            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);

            this.max_steps   = 1200;
            this.prev_action = zeros(4, 1);

            % ChoeJae px4_main.m (커밋 f02c317) 기준 표준 파라미터
            cfg = struct();
            cfg.dt_sim         = 0.004; % 250 Hz
            cfg.dt_pos         = 0.020; % 50 Hz
            cfg.dt             = cfg.dt_pos; % PositionCtrl_GS 적분/미분 주기 = 50 Hz
            cfg.use_comm_delay = true;
            cfg.delay_steps    = 10;    % 0.04 s @ 250 Hz

            % Gazebo x500 물리량
            cfg.m               = 2.064;
            cfg.armLength       = 0.246;
            cfg.Ixx             = 0.021667;
            cfg.Iyy             = 0.021667;
            cfg.Izz             = 0.040000;
            cfg.MPC_THR_HOVER   = 0.73;
            cfg.thrust_exp      = 2.0;
            cfg.f_max_per_rotor = 8.60;
            cfg.km_yaw          = 0.05;
            cfg.tau_up          = 0.0125;
            cfg.tau_down        = 0.0250;
            cfg.drag_coeff      = [0.10; 0.10; 0.20];

            % PX4 자세 제어기 (250 Hz 이중 루프, ChoeJae SITL 확인값 K_att = MC_ROLL/PITCH_P = 6.5 일치)
            cfg.att_mode    = 2;
            cfg.sysid_order = 2;
            cfg.sysid_a     = 6.5;
            cfg.K_att       = 6.5;
            cfg.MC_ROLL_P   = 6.5;
            cfg.MC_PITCH_P  = 6.5;
            cfg.tau_rate    = 0.04;

            % ChoeJae PositionCtrl_GS 설정 (px4_main.m 과 100% 동일)
            cfg.Feedforward        = true;
            cfg.Z_Gain_Scheduling  = true;
            cfg.Z_D_on_measurement = true;
            cfg.Use_NDI            = false;
            cfg.max_angle_deg      = 50.0;
            cfg.Err_Yaw            = 'desired';
            cfg.Eq_Delay_Comp      = false;
            cfg.posGain = containers.Map( ...
                {'P_x','I_x','D_x','P_y','I_y','D_y','P_z','I_z','D_z'}, ...
                {0.215, 0.0387, 0.203, ...
                 0.215, 0.0387, 0.203, ...
                 6.0*cfg.m, 2.0*cfg.m, 4.0*cfg.m});

            % 기본 미션 설정 (원 운동 중 +0.3m 고도 계단)
            cfg.target_speed         = 5.0;
            cfg.curvature            = 0.15;
            cfg.n_laps               = 3;
            cfg.accel_dist_per_speed = 3.0;
            cfg.cruise_time          = 3.0;
            cfg.height_step          = 0.3;
            cfg.height_order         = {'up'};
            cfg.target_yaw           = [];
            cfg.start_with_yaw       = true;
            cfg.drone_init_states    = [0.0; 0.0; -6.0; 0.0; 0; 0; 0; 0; 0; 0; 0; 0];

            if nargin >= 1 && ~isempty(use_sensor_noise_in)
                cfg.use_sensor_noise = logical(use_sensor_noise_in);
            else
                cfg.use_sensor_noise = false;
            end
            cfg.gps_drift_bound = 1.5;

            this.cfg = cfg;
        end

        function [Observation, Reward, IsDone, LoggedSignals] = step(this, Action)
            % 1. PPO Action -> PositionCtrl_GS 잔차 게인 오프셋 주입
            %    - dP_z: 계단 입력 시 순간 추력(U1) 임펄스 스파이크를 억제할 수 있도록
            %      하향 감쇠(-range_P_z = -2.0 N/m)는 충분히 허용하고 상향 증폭은 +0.25 N/m로 제한
            dP_xy = this.range_P_xy * Action(1);
            dD_xy = max(0, this.range_D_xy * Action(2));
            if Action(3) >= 0
                dP_z = 0.25 * Action(3);
            else
                dP_z = this.range_P_z * Action(3); % 최대 -2.0 N/m 감쇠 (Soft-Start)
            end
            dD_z  = max(0, this.range_D_z * Action(4));
            this.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);

            % 2. 다중 주기 시뮬레이션 (RL 10 Hz = 0.1 s = 250 Hz 25 스텝)
            N_sub        = 25;
            pos_ratio    = round(this.cfg.dt_pos / this.cfg.dt_sim); % 5 스텝마다 50 Hz 위치 루프
            sat_count    = 0;
            max_dU1_norm = 0.0; % 구간 내 평형 추력 대비 최대 순간 추력 편차 |U1_des - U1_eq| / mg
            max_jmp_U1   = 0.0; % 구간 내 50Hz 스텝 간 추력 점프(임펄스) |U1_des - prev_U1_des| / mg
            mg_val       = this.cfg.m * 9.81;

            for k = 1:N_sub
                state_true   = this.drone_px4.GetState();
                t_now        = this.drone_px4.t;
                is_50hz_tick = (mod(k - 1, pos_ratio) == 0);

                [traj_pos, traj_vel, kappa_ff, ~] = this.traj.get_position(t_now);
                [p_est, v_est] = this.drone_px4.sensorEKF.Update(state_true(1:3), state_true(4:6), is_50hz_tick);
                this.last_p_est = p_est;
                this.last_v_est = v_est;

                if is_50hz_tick
                    speed_xy = norm(traj_vel(1:2));
                    if speed_xy > 1e-3
                        raw_yaw  = atan2(traj_vel(2), traj_vel(1));
                        yaw_diff = raw_yaw - this.prev_yaw;
                        while yaw_diff >  pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                        while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                        this.prev_yaw = raw_yaw;
                    end

                    % ChoeJae PositionCtrl_GS 입력 형식: cmd = [x; y; z; psi; v_des; kappa]
                    cmd_gs = [traj_pos; this.prev_yaw; this.cfg.target_speed; kappa_ff];
                    state_est = state_true;
                    state_est(1:3) = p_est;
                    state_est(4:6) = v_est;

                    att_u1 = this.posCtrl.Update(cmd_gs, state_est); % [phi; theta; psi; U1_des]
                    U1_des_now = att_u1(4);
                    U1_eq_now  = this.cfg.m * sqrt(9.81^2 + (this.cfg.target_speed^2 * kappa_ff)^2);
                    max_dU1_norm = max(max_dU1_norm, abs(U1_des_now - U1_eq_now) / mg_val);
                    max_jmp_U1   = max(max_jmp_U1,   abs(U1_des_now - this.prev_U1_des) / mg_val);
                    this.prev_U1_des = U1_des_now;

                    % ChoeJae f8ccf1f 동일: 제곱근 추력 정규화 + [0.1, 0.9] 클리핑
                    thr_raw = this.cfg.MPC_THR_HOVER * sqrt(max(U1_des_now / mg_val, 0.0));
                    thr_cmd = max(0.1, min(thr_raw, 0.9));
                    if thr_raw < 0.1 || thr_raw > 0.9
                        sat_count = sat_count + 1;
                    end
                    this.zoh_att_cmd = [att_u1(1); att_u1(2); att_u1(3); thr_cmd];
                end

                % FIFO 통신 지연 버퍼
                if this.cfg.use_comm_delay && this.delay_steps > 0
                    delayed_att_cmd = this.fifo_buffer(:, this.buf_idx);
                    this.fifo_buffer(:, this.buf_idx) = this.zoh_att_cmd;
                    this.buf_idx = mod(this.buf_idx, this.delay_steps) + 1;
                else
                    delayed_att_cmd = this.zoh_att_cmd;
                end

                % 250 Hz PX4 자세 제어기 + Gazebo x500 플랜트 적분
                ctrl_in = this.drone_px4.attCtrl.Update(delayed_att_cmd, state_true);
                [u_actual, ~, ~] = this.drone_px4.UpdateState(ctrl_in);
                if this.drone_px4.is_saturated
                    sat_count = sat_count + 1;
                end
            end

            this.current_step = this.current_step + 1;

            % 3. 관측 벡터 수집 (11x1 World NED, ChoeJae run_px4_mission.m 203행과 100% 동일)
            state = this.drone_px4.GetState();
            t_now = this.drone_px4.t;
            [traj_pos, traj_vel, ~, ~] = this.traj.get_position(t_now);
            [s_now, ~] = this.traj.arc_at_time(t_now);
            kappa_now     = this.traj.curvature_at(s_now);
            kappa_spatial = this.traj.curvature_at(min(s_now + this.lookahead_L, this.traj.total_length));

            e_est = traj_pos - this.last_p_est;
            Observation = [
                e_est(1); e_est(2); e_est(3);
                this.last_v_est(1); this.last_v_est(2); this.last_v_est(3);
                state(7); state(8);
                this.cfg.target_speed;
                kappa_now;
                kappa_spatial
            ];

            % 4. 고도 계단 정밀 추종 + 원 궤적 유지 + 평형점 기준 자세 진동 억제 보상함수
            e_true = traj_pos - state(1:3);
            err_xy = norm(e_true(1:2));
            err_z  = abs(e_true(3));
            err_3d = norm(e_true);
            vz     = state(6);
            phi    = state(7);
            theta  = state(8);
            p      = state(10);
            q      = state(11);

            % (1) XY 원 궤적 정밀 추종 보상
            r_xy = 8.0 * exp(-(err_xy^2) / (2 * 0.15^2)) - 6.0 * err_xy - 2.0 * (err_xy^2);

            % (2) Z축 고도 계단 응답 보상 (가중치 2 -> 5 상향 + 목표 근접 적응형 댐핑으로 맞바꿈 방지)
            r_z  = 5.0 * exp(-(err_z^2) / (2 * 0.04^2)) - 8.0 * err_z ...
                 - 1.0 * (vz^2) * exp(-((err_z / 0.06)^2)) - 0.15 * (vz^2);

            % (3) 평형점(phi_eq, theta_eq=0, p_eq=0, q_eq=v*kappa*sin(phi_eq)) 기준 자세 진동 페널티
            v_now  = norm(traj_vel(1:2));
            phi_eq = atan((v_now^2 * kappa_now) / 9.81);
            q_eq   = (v_now * kappa_now) * sin(phi_eq);
            r_att  = max(-2.5 * ((phi - phi_eq)^2 + theta^2) - 0.15 * (p^2 + (q - q_eq)^2), -30);

            % (4) 순간 과도 추력(U1 임펄스 스파이크) 억제 보상 + 액션 평활화 + 포화 방지
            %     - 계단 입력 순간 U1이 임펄스 모양으로 튀는 현상(|U1 - U1_eq| 및 스텝 간 점프 |dU1|)을 직접 감점
            r_u1_imp = -3.0 * (max_dU1_norm^2) - 5.0 * (max_jmp_U1^2);
            r_smooth = -0.01 * sum(Action.^2) - 0.05 * sum((Action - this.prev_action).^2);
            r_sat    = -5.0 * (sat_count / N_sub);
            this.prev_action = Action;

            Reward = r_xy + r_z + r_att + r_u1_imp + r_smooth + r_sat + 0.1;

            % 5. 에피소드 종료 판정
            IsDone = false;
            if this.drone_px4.t >= this.traj.tf || this.current_step >= this.max_steps
                IsDone = true;
                Reward = Reward + 30.0;
            end

            tilt_limit = deg2rad(this.max_tilt_deg);
            if err_3d > 8.0 || abs(phi) > tilt_limit || abs(theta) > tilt_limit
                IsDone = true;
                remaining_steps = max(0, this.max_steps - this.current_step);
                Reward = Reward - (500 + 0.5 * remaining_steps);
            end

            LoggedSignals = [];
        end

        function InitialObservation = reset(this)
            this.episode_count = this.episode_count + 1;

            if this.randomize_mission
                % 속력 [3.0, 10.0] m/s, 곡률 [0.05, 2.5] m^-1 (물리적 선회 한계 phi_eq<=45deg 클리핑),
                % 고도 상승 계단 [0.1, 1.0] m ('up' 전용) 무작위화
                v_rand = 3.0 + 7.0 * rand();
                k_phys_max = (9.81 * tand(45.0)) / max(v_rand^2, 1.0);
                k_upper    = min(2.5, max(0.05, k_phys_max));
                k_mag      = 0.05 + (k_upper - 0.05) * rand();
                dir_c = 1; if rand() < 0.5, dir_c = -1; end

                this.cfg.target_speed = v_rand;
                this.cfg.curvature    = dir_c * k_mag;
                this.cfg.n_laps       = 2; % 학습 효율을 위해 단계당 2바퀴
                this.cfg.height_step  = 0.10 + 0.90 * rand(); % 0.1 ~ 1.0 m 랜덤 계단 크기

                % 상승('up')과 하강('down') 모두 균등하게 학습 (단일 상승, 단일 하강, 상승->하강, 하강->상승)
                r_ord = rand();
                if r_ord < 0.25
                    this.cfg.height_order = {'up'};
                elseif r_ord < 0.50
                    this.cfg.height_order = {'down'};
                elseif r_ord < 0.75
                    this.cfg.height_order = {'up', 'down'};
                else
                    this.cfg.height_order = {'down', 'up'};
                end
            end

            if this.randomize_delay
                this.cfg.delay_steps = randi(this.delay_steps_range);
            else
                this.cfg.delay_steps = 10;
            end
            if this.randomize_thrust_exp
                this.cfg.thrust_exp = this.thrust_exp_range(1) + ...
                    (this.thrust_exp_range(2) - this.thrust_exp_range(1)) * rand();
            else
                this.cfg.thrust_exp = 2.0;
            end
            if this.randomize_att_loop
                mc_p = this.mc_att_p_range(1) + (this.mc_att_p_range(2) - this.mc_att_p_range(1)) * rand();
                this.cfg.MC_ROLL_P  = mc_p;
                this.cfg.MC_PITCH_P = mc_p;
                this.cfg.K_att      = mc_p;
                this.cfg.tau_rate   = this.tau_rate_range(1) + ...
                    (this.tau_rate_range(2) - this.tau_rate_range(1)) * rand();
            else
                this.cfg.MC_ROLL_P  = 6.5;
                this.cfg.MC_PITCH_P = 6.5;
                this.cfg.K_att      = 6.5;
                this.cfg.tau_rate   = 0.04;
            end

            % 정지 출발 (v0 = 0, CircleTrajectory 와 동일)
            this.cfg.drone_init_states = [0.0; 0.0; -6.0; 0.0; 0; 0; 0; 0; 0; 0; 0; 0];
            this.prev_yaw = 0.0;

            % CircleTrajectory 생성 (고도 계단 포함)
            this.traj = CircleTrajectory( ...
                this.cfg.drone_init_states(1:3), this.cfg.target_speed, this.cfg.curvature, ...
                this.cfg.accel_dist_per_speed, this.cfg.n_laps, 0.0, 0.0, this.cfg.cruise_time, ...
                this.cfg.height_step, this.cfg.height_order);

            % Drone_PX4 플랜트 생성 (더미 waypoints 전달, 궤적은 this.traj 사용)
            dummy_wp = [0; 10.0; -6.0];
            cfg_plant = this.cfg;
            cfg_plant.dt = this.cfg.dt_sim;
            this.drone_px4 = Drone_PX4(cfg_plant, dummy_wp);

            % ChoeJae PositionCtrl_GS 생성
            params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'}, ...
                {this.cfg.m, this.cfg.armLength, this.cfg.Ixx, this.cfg.Iyy, this.cfg.Izz});
            this.posCtrl = PositionCtrl_GS(this.cfg.posGain, params, this.cfg);

            % FIFO 지연 버퍼 초기화
            init_att_cmd      = [0.0; 0.0; 0.0; this.cfg.MPC_THR_HOVER];
            this.zoh_att_cmd  = init_att_cmd;
            this.delay_steps  = max(1, round(this.cfg.delay_steps));
            this.fifo_buffer  = repmat(init_att_cmd, 1, this.delay_steps);
            this.buf_idx      = 1;

            this.max_steps     = ceil(this.traj.tf / 0.1) + 20;
            this.current_step  = 0;
            this.prev_action   = zeros(4, 1);
            this.prev_u_moment = zeros(3, 1);
            this.prev_U1_des   = this.cfg.m * 9.81;
            this.last_p_est    = this.drone_px4.r;
            this.last_v_est    = this.drone_px4.dr;

            % 초기 관측치 (11x1 World NED)
            state = this.drone_px4.GetState();
            [traj_pos, ~, ~, ~] = this.traj.get_position(0);
            kappa_now     = this.traj.curvature_at(0);
            kappa_spatial = this.traj.curvature_at(min(this.lookahead_L, this.traj.total_length));
            e_init = traj_pos - state(1:3);
            InitialObservation = [
                e_init(1); e_init(2); e_init(3);
                state(4); state(5); state(6);
                state(7); state(8);
                this.cfg.target_speed;
                kappa_now;
                kappa_spatial
            ];
        end
    end
end
