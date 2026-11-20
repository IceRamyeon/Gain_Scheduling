classdef RL_DroneEnv_PX4 < rl.env.MATLABEnvironment
    % RL_DRONEENV_PX4 PX4 + Gazebo x500 전용 잔차 PPO 강화학습 환경
    % - Plant: Drone_PX4 (m=2.064kg, L=0.246m, 4-로터 믹서 클리핑 [0, 8.6N],
    %                     비대칭 모터 시정수 tau_up=0.0125s / tau_down=0.0250s, 공기항력)
    % - 자세 제어기: PX4_AttitudeCtrl (250 Hz 이중 루프 Angle P + ±220 deg/s + Rate PID)
    % - 위치 제어기: PositionCtrl_PX4 (50 Hz 2-DOF + Z축 스케줄링 + 잔차 게인)
    % - 지연 및 센서: 0.04 s FIFO 통신 지연 + PX4_Sensor_EKF (선택적 M10 GPS 노이즈)
    %
    % Observation (11x1, Speed-Inverse Spatial Lookahead):
    %   [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta; v_des; kappa_now; kappa_spatial]
    % Action (4x1):
    %   [a_P_xy; a_D_xy; a_P_z; a_D_z] in [-1, 1]^4

    properties
        cfg
        drone_px4
        current_step
        max_steps

        % 속력 반비례 공간 예견 거리 (Option 1: dt = lookahead_L / v_des)
        lookahead_L  = 2.0;
        lookahead_dt = 0.5;

        % PositionCtrl_PX4 기준 잔차 게인 스케일링 범위
        % (기본 Kp_xy = 0.981, Kd_xy = 1.668, Kp_z = 4.0, Kd_z = 3.5)
        range_P_xy = 0.8;
        range_D_xy = 1.0;
        range_P_z  = 1.5;
        range_D_z  = 1.0;

        % 다중 주기 & FIFO 지연 버퍼 상태
        prev_yaw
        prev_action
        prev_u_moment
        zoh_att_cmd
        fifo_buffer
        buf_idx
        delay_steps
        last_p_est
        last_v_est

        % 도메인 무작위화 및 커리큘럼 옵션
        randomize_speed = true;
        speed_range     = [2.5, 5.5];
        randomize_path  = true;
        max_kappa       = 0.25;
        max_tilt_deg    = 40.0; % Gazebo x500 최대 30 deg + 과도 마진 10 deg

        use_curriculum = true;
        episode_count  = 0;
        total_episodes = 3000;
    end

    methods
        function this = RL_DroneEnv_PX4(use_sensor_noise_in)
            obsInfo = rlNumericSpec([11 1]);
            obsInfo.Name = 'PX4 Drone States with Speed-Inverse Curvature Lookahead';

            actInfo = rlNumericSpec([4 1], 'LowerLimit', -1, 'UpperLimit', 1);
            actInfo.Name = 'PX4 Residual Gain Offsets';

            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);

            this.max_steps   = 1500;
            this.prev_action = zeros(4, 1);

            % Gazebo x500 + PX4 표준 파라미터 구성
            cfg = struct();
            cfg.dt_sim         = 0.004; % 250 Hz 물리/자세 루프
            cfg.dt_pos         = 0.020; % 50 Hz 위치/EKF 루프
            cfg.dt             = cfg.dt_sim;
            cfg.use_comm_delay = true;
            cfg.delay_steps    = 10;    % 0.04 s 순수 지연 @ 250 Hz

            % Gazebo x500 물리량
            cfg.m               = 2.064;
            cfg.armLength       = 0.246;
            cfg.Ixx             = 0.021667;
            cfg.Iyy             = 0.021667;
            cfg.Izz             = 0.040000;
            cfg.MPC_THR_HOVER   = 0.60;
            cfg.f_max_per_rotor = 8.60;
            cfg.km_yaw          = 0.05;
            cfg.tau_up          = 0.0125;
            cfg.tau_down        = 0.0250;
            cfg.drag_coeff      = [0.10; 0.10; 0.20];

            % PX4 이중 루프 자세 제어기 설정 (Mode 2)
            cfg.att_mode    = 2;
            cfg.sysid_order = 2;
            cfg.sysid_a     = 4.0;
            cfg.K_att       = 4.0;
            cfg.tau_rate    = 0.04;

            % PX4 2-DOF 위치 제어기 설정
            cfg.pos_Kp_ang        = 0.10;
            cfg.pos_Ki_ang        = 0.001;
            cfg.pos_Kd_ang        = 0.17;
            cfg.pos_Kp_z          = 4.0;
            cfg.pos_Ki_z          = 0.2;
            cfg.pos_Kd_z          = 3.5;
            cfg.k_ff_v            = 1.0;
            cfg.k_ff_a            = 1.0;
            cfg.Z_Gain_Scheduling = true;
            cfg.max_angle_deg     = 30.0;

            % 센서 노이즈 / EKF2 설정 (Gazebo/실기체 현실 모사: 기본 true)
            if nargin >= 1 && ~isempty(use_sensor_noise_in)
                cfg.use_sensor_noise = logical(use_sensor_noise_in);
            else
                cfg.use_sensor_noise = true;
            end
            cfg.gps_drift_bound = 1.5;

            cfg.target_speed      = 4.0;
            cfg.target_yaw        = [];
            cfg.start_with_yaw    = true;
            cfg.drone_init_states = [0.0; 0.0; -6.0; cfg.target_speed; 0; 0; 0; 0; 0; 0; 0; 0];

            this.cfg = cfg;
        end

        function [Observation, Reward, IsDone, LoggedSignals] = step(this, Action)
            % 1. PPO Action -> PositionCtrl_PX4 잔차 게인 오프셋 주입
            dP_xy = this.range_P_xy * Action(1);
            dD_xy = max(0, this.range_D_xy * Action(2));
            dP_z  = this.range_P_z  * Action(3);
            dD_z  = max(0, this.range_D_z  * Action(4));
            this.drone_px4.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);

            % 2. 다중 주기 시뮬레이션 (RL 10 Hz = 0.1 s = 250 Hz 25 스텝)
            N_sub     = 25;
            pos_ratio = round(this.cfg.dt_pos / this.cfg.dt_sim); % 5 스텝마다 50 Hz 위치 루프
            sat_count = 0;

            for k = 1:N_sub
                state_true   = this.drone_px4.GetState();
                t_now        = this.drone_px4.t;
                is_50hz_tick = (mod(k - 1, pos_ratio) == 0);

                [traj_pos, traj_vel, ~, ~, traj_acc] = this.drone_px4.trajCtrl.get_position(t_now);
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
                    [this.zoh_att_cmd, ~] = this.drone_px4.posCtrl.Update(...
                        traj_pos, traj_vel, traj_acc, this.prev_yaw, ...
                        p_est, v_est, state_true(7:9));
                end

                % 0.04 s FIFO 통신 지연 버퍼
                if this.cfg.use_comm_delay && this.delay_steps > 0
                    delayed_att_cmd = this.fifo_buffer(:, this.buf_idx);
                    this.fifo_buffer(:, this.buf_idx) = this.zoh_att_cmd;
                    this.buf_idx = mod(this.buf_idx, this.delay_steps) + 1;
                else
                    delayed_att_cmd = this.zoh_att_cmd;
                end

                % 250 Hz PX4 자세 제어기 + Gazebo x500 믹서/모터/6-DOF 적분
                ctrl_in = this.drone_px4.attCtrl.Update(delayed_att_cmd, state_true);
                [u_actual, ~, ~] = this.drone_px4.UpdateState(ctrl_in);
                if this.drone_px4.is_saturated
                    sat_count = sat_count + 1;
                end
            end

            this.current_step = this.current_step + 1;

            % 3. 새로운 관측치 수집: Actor는 실제 Gazebo와 동일하게 EKF2 추정치(p_est, v_est)만 관측
            state      = this.drone_px4.GetState();
            v_des      = this.cfg.target_speed;
            dt_spatial = this.lookahead_L / max(v_des, 0.5);
            [traj_pos, traj_vel, kappa_now, kappa_spatial] = ...
                this.drone_px4.trajCtrl.get_position(this.drone_px4.t, dt_spatial);

            e_x_est = traj_pos(1) - this.last_p_est(1);
            e_y_est = traj_pos(2) - this.last_p_est(2);
            e_z_est = traj_pos(3) - this.last_p_est(3);
            v_x_est = this.last_v_est(1);
            v_y_est = this.last_v_est(2);
            v_z_est = this.last_v_est(3);
            phi     = state(7);
            theta   = state(8);

            Observation = [e_x_est; e_y_est; e_z_est; v_x_est; v_y_est; v_z_est; phi; theta; v_des; kappa_now; kappa_spatial];

            % 4. 다목적 보상 계산: Critic/보상 평가는 참값(Ground Truth) 기반 (Privileged Reward)
            e_true    = traj_pos - state(1:3);
            err_pos   = norm(e_true);
            err_vel   = norm(traj_vel - state(4:6));
            act_speed = norm(state(4:5));
            err_spd   = abs(v_des - act_speed);
            rates     = state(10:12);

            r_pos    = 6.0 * exp(-(err_pos^2) / (2 * 0.08^2)) - 2.5 * err_pos;
            r_vel    = max(-0.15 * (err_vel^2) - 0.25 * (err_spd^2), -40);
            r_att    = max(-0.10 * (phi^2 + theta^2) - 0.03 * norm(rates)^2, -40);
            r_smooth = -0.01 * sum(Action.^2) - 0.03 * sum((Action - this.prev_action).^2);

            curr_u_moment = u_actual(2:4);
            r_moment = max(-0.02 * sum(curr_u_moment.^2) - 0.05 * sum((curr_u_moment - this.prev_u_moment).^2), -30);
            this.prev_u_moment = curr_u_moment;

            r_alive = 0.1;

            Reward = r_pos + r_vel + r_att + r_smooth + r_moment + r_alive;
            this.prev_action = Action;

            % 5. 에피소드 종료 판정
            IsDone = false;
            if this.drone_px4.t >= this.drone_px4.tf || this.current_step >= this.max_steps
                IsDone = true;
                Reward = Reward + 35.0;
            end

            tilt_limit = deg2rad(this.max_tilt_deg);
            if err_pos > 5.0 || abs(phi) > tilt_limit || abs(theta) > tilt_limit
                IsDone = true;
                remaining_steps = max(0, this.max_steps - this.current_step);
                Reward = Reward - (500 + 0.5 * remaining_steps);
            end

            LoggedSignals = [];
        end

        function InitialObservation = reset(this)
            this.episode_count = this.episode_count + 1;

            % 3,000 에피소드 3단계 커리큘럼 (Gazebo x500 30도 뱅크각 한계 고려)
            if this.use_curriculum
                ep = this.episode_count;
                if ep <= 600
                    cur_v_min = 2.5; cur_v_max = 3.8;
                    cur_k_max = 0.20;
                elseif ep <= 1800
                    cur_v_min = 3.0; cur_v_max = 4.5;
                    cur_k_max = 0.40;
                else
                    cur_v_min = 3.5; cur_v_max = 5.5;
                    cur_k_max = 0.60;
                end
                this.speed_range = [cur_v_min, cur_v_max];
                this.max_kappa   = cur_k_max;
            end

            if this.randomize_speed
                v_rand = this.speed_range(1) + (this.speed_range(2) - this.speed_range(1)) * rand();
                this.cfg.target_speed = v_rand;
                this.cfg.drone_init_states(4) = v_rand;
            end

            v_now = this.cfg.target_speed;
            phys_kappa_limit = (9.81 * tand(this.cfg.max_angle_deg)) / max(v_now^2, 1.0);
            eff_max_kappa    = max(this.max_kappa, phys_kappa_limit);

            if this.randomize_path
                [seg_r, seg_theta, seg_z] = pathGen(true, eff_max_kappa);
            else
                [seg_r, seg_theta, seg_z] = pathGen(false);
            end
            waypoints = [seg_r; seg_theta; seg_z];

            % 초기 Yaw 정렬
            temp_planner = TrajectoryPlanner(this.cfg.drone_init_states(1:3), waypoints, this.cfg.target_speed, this.lookahead_dt);
            [~, init_vel, ~] = temp_planner.get_position(0);
            if norm(init_vel(1:2)) > 1e-3
                init_yaw = atan2(init_vel(2), init_vel(1));
            else
                init_yaw = 0.0;
            end
            this.cfg.drone_init_states(9) = init_yaw;
            this.prev_yaw = init_yaw;

            % Drone_PX4 (Gazebo x500 + PX4_AttitudeCtrl + PositionCtrl_PX4 + PX4_Sensor_EKF) 생성
            this.drone_px4 = Drone_PX4(this.cfg, waypoints);

            % FIFO 지연 버퍼 초기화
            init_att_cmd      = [0.0; 0.0; init_yaw; this.cfg.MPC_THR_HOVER];
            this.zoh_att_cmd  = init_att_cmd;
            this.delay_steps  = max(1, round(this.cfg.delay_steps));
            this.fifo_buffer  = repmat(init_att_cmd, 1, this.delay_steps);
            this.buf_idx      = 1;

            this.max_steps     = ceil(this.drone_px4.tf / 0.1) + 30;
            this.current_step  = 0;
            this.prev_action   = zeros(4, 1);
            this.prev_u_moment = zeros(3, 1);
            this.last_p_est    = this.drone_px4.r;
            this.last_v_est    = this.drone_px4.dr;

            % 초기 관측치 계산 (11x1)
            state      = this.drone_px4.GetState();
            v_des      = this.cfg.target_speed;
            dt_spatial = this.lookahead_L / max(v_des, 0.5);
            [traj_pos, ~, kappa_now, kappa_spatial] = ...
                this.drone_px4.trajCtrl.get_position(0, dt_spatial);

            e_x   = traj_pos(1) - state(1);
            e_y   = traj_pos(2) - state(2);
            e_z   = traj_pos(3) - state(3);
            v_x   = state(4);
            v_y   = state(5);
            v_z   = state(6);
            phi   = state(7);
            theta = state(8);

            InitialObservation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta; v_des; kappa_now; kappa_spatial];
        end
    end
end
