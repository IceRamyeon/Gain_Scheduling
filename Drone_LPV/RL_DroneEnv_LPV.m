classdef RL_DroneEnv_LPV < rl.env.MATLABEnvironment
    % RL_DroneEnv_LPV: Drone_LPV 피드포워드 + Z축 스케줄링 기반 잔차 강화학습 환경
    % Observation (11x1):
    %   [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta; v_des; kappa_now; kappa_future]
    % Action (4x1):
    %   [a_P_xy; a_D_xy; a_P_z; a_D_z] in [-1, 1]^4 (LPV 베이스라인 위 잔차 게인 보정)

    properties
        cfg
        drone1
        current_step
        max_steps

        % 미래 곡률 예견 시간 (t 이후의 곡률 조회용, 기본 0.5초)
        lookahead_dt = 0.5;

        % Residual Action Scaling Ranges
        range_P_xy = 1.0;
        range_D_xy = 1.2;
        range_P_z  = 1.5;
        range_D_z  = 1.0;

        prev_yaw
        prev_action
        prev_u_moment

        % Randomization options
        randomize_speed = true;
        speed_range     = [2.5, 6.0];
        randomize_path  = true;
        max_kappa       = 0.30;
        max_tilt_deg    = 50.0; % 45도 뱅크각 제한 + 과도 마진 5도

        % Curriculum options
        use_curriculum = true;
        episode_count  = 0;
        total_episodes = 3000;
    end

    methods
        function this = RL_DroneEnv_LPV()
            % 11차원 상태 벡터: [e_x, e_y, e_z, v_x, v_y, v_z, phi, theta, v_des, kappa_now, kappa_future]
            obsInfo = rlNumericSpec([11 1]);
            obsInfo.Name = 'Drone States with Target Speed and Curvature Lookahead';

            actInfo = rlNumericSpec([4 1], 'LowerLimit', -1, 'UpperLimit', 1);
            actInfo.Name = 'Residual Gain Offsets';

            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);

            this.max_steps   = 1500; % 긴 코스(5~8개 원호)에 맞춰 최대 스텝 상향
            this.prev_action = zeros(4, 1);

            % 기본 Drone_LPV 파라미터 초기화
            cfg = struct();
            cfg.dt                = 0.01;
            cfg.target_yaw        = [];
            cfg.target_speed      = 4.0;
            cfg.Feedforward       = true;
            cfg.Z_Gain_Scheduling = true;
            cfg.play_animation    = false;
            cfg.auto_save         = false;
            cfg.start_with_yaw    = true;
            cfg.drone1_init_states = [0.0, 0.0, -6.0, cfg.target_speed, 0, 0, 0, 0, 0, 0, 0, 0]';
            cfg.m   = 1.25;   cfg.armLength = 0.265;
            cfg.Ixx = 0.0232; cfg.Iyy = 0.0232; cfg.Izz = 0.0468;
            cfg.posGain = containers.Map({'P_x','I_x','D_x','P_y','I_y','D_y','P_z','I_z','D_z'}, ...
                {13.5, 5.0, 5.5, 13.5, 5.0, 5.5, 16.88, 6.25, 6.88});
            cfg.attGain = containers.Map({'P_phi','I_phi','D_phi','P_theta','I_theta','D_theta','P_psi','I_psi','D_psi','P_zdot','I_zdot','D_zdot'}, ...
                {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0});
            this.cfg = cfg;
        end

        function [Observation, Reward, IsDone, LoggedSignals] = step(this, Action)
            % 1. PPO Action -> 잔차 게인 오프셋으로 변환하여 주입
            dP_xy = this.range_P_xy * Action(1);
            dD_xy = max(0, this.range_D_xy * Action(2)); % D_xy 하한선 보호 (댐핑 보강 전용)
            dP_z  = this.range_P_z  * Action(3);
            dD_z  = max(0, this.range_D_z  * Action(4)); % D_z 하한선 보호

            this.drone1.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);

            % 2. 서브스텝 시뮬레이션 (10스텝 = 0.1초 동안 제어 유지)
            N_sub = 10;

            for k = 1:N_sub
                drone_state = this.drone1.GetState();
                t_now = this.drone1.t;
                [traj_pos, traj_vel, kappa_now] = this.drone1.trajCtrl.get_position(t_now, this.lookahead_dt);

                % 동적 Yaw 계산
                speed_xy = norm(traj_vel(1:2));
                if speed_xy > 1e-3
                    raw_yaw  = atan2(traj_vel(2), traj_vel(1));
                    yaw_diff = raw_yaw - this.prev_yaw;
                    while yaw_diff >  pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                    while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                    this.prev_yaw = raw_yaw;
                end

                % 위치 제어기 (직접 Thrust U1 지령 생성)
                current_cmd = [traj_pos; this.prev_yaw; this.cfg.target_speed; kappa_now];
                att_cmd = this.drone1.posCtrl.Update(current_cmd, drone_state);

                % 자세 제어기
                u_motor = this.drone1.attCtrl.Update(att_cmd, drone_state);

                % 동역학 전파
                this.drone1.UpdateState(u_motor);
            end

            this.current_step = this.current_step + 1;

            % 3. 새로운 관측치 수집 (목표 속력, 현재 곡률, t 이후 곡률 포함 11x1)
            state = this.drone1.GetState();
            [traj_pos, traj_vel, kappa_now, kappa_future] = ...
                this.drone1.trajCtrl.get_position(this.drone1.t, this.lookahead_dt);

            e_x   = traj_pos(1) - state(1);
            e_y   = traj_pos(2) - state(2);
            e_z   = traj_pos(3) - state(3);
            v_x   = state(4);
            v_y   = state(5);
            v_z   = state(6);
            phi   = state(7);
            theta = state(8);
            v_des = this.cfg.target_speed;

            Observation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta; v_des; kappa_now; kappa_future];

            % 4. 보상 계산
            err_pos   = norm([e_x, e_y, e_z]);
            err_vel   = norm(traj_vel - state(4:6));
            act_speed = norm(state(4:5));
            err_spd   = abs(v_des - act_speed);
            rates     = state(10:12);

            % 정밀 위치 추종 + 등속 유지 패널티 + 자세/모멘트 안정화 + 잔차 액션 최소화
            r_pos    = 6.0 * exp(-(err_pos^2) / (2 * 0.08^2)) - 2.5 * err_pos;
            r_vel    = max(-0.15 * (err_vel^2) - 0.25 * (err_spd^2), -40);
            r_att    = max(-0.10 * (phi^2 + theta^2) - 0.03 * norm(rates)^2, -40);
            r_smooth = -0.01 * sum(Action.^2) - 0.03 * sum((Action - this.prev_action).^2);

            curr_u_moment = u_motor(2:4);
            r_moment = max(-0.02 * sum(curr_u_moment.^2) - 0.05 * sum((curr_u_moment - this.prev_u_moment).^2), -30);
            this.prev_u_moment = curr_u_moment;

            r_alive = 0.1;

            Reward = r_pos + r_vel + r_att + r_smooth + r_moment + r_alive;
            this.prev_action = Action;

            % 5. 에피소드 종료 판정
            IsDone = false;
            if this.drone1.t >= this.drone1.tf || this.current_step >= this.max_steps
                IsDone = true;
                Reward = Reward + 35.0; % 완주 보너스
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

            % 3000 에피소드 맞춤형 점진적 커리큘럼
            if this.use_curriculum
                ep = this.episode_count;
                if ep <= 600
                    % Stage 1 (Ep 1~600): 기초 안정화 및 중속 적응
                    cur_v_min = 2.5; cur_v_max = 4.0;
                    cur_k_max = 0.20;
                elseif ep <= 1800
                    % Stage 2 (Ep 601~1800): 지름 5m 헤어핀 적응 및 고속 선회
                    cur_v_min = 3.0; cur_v_max = 5.0;
                    cur_k_max = 0.40;
                else
                    % Stage 3 (Ep 1801~3000): 극한 지름 2.5m 헤어핀 및 초고속 댐핑 마스터
                    cur_v_min = 3.5; cur_v_max = 5.5;
                    cur_k_max = 0.60;
                end
                this.speed_range = [cur_v_min, cur_v_max];
                this.max_kappa   = cur_k_max;
            end

            % 속도 무작위화
            if this.randomize_speed
                v_rand = this.speed_range(1) + (this.speed_range(2) - this.speed_range(1)) * rand();
                this.cfg.target_speed = v_rand;
                this.cfg.drone1_init_states(4) = v_rand;
            end

            % 뱅크각 45도 한계(안전 마진 45도: a_c <= g*tan(45deg) = 9.81 m/s^2)를 고려한 기본 상한
            v_now = this.cfg.target_speed;
            phys_kappa_limit = 9.81 / max(v_now^2, 1.0);
            eff_max_kappa    = max(this.max_kappa, phys_kappa_limit);

            % 직선 + 고정 곡률 원호(r, theta) 경로 생성
            if this.randomize_path
                [seg_r, seg_theta, seg_z] = pathGen(true, eff_max_kappa);
            else
                [seg_r, seg_theta, seg_z] = pathGen(false);
            end
            waypoints = [seg_r; seg_theta; seg_z];

            drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'}, ...
                {this.cfg.m, this.cfg.armLength, this.cfg.Ixx, this.cfg.Iyy, this.cfg.Izz});

            % 초기 Yaw 정렬
            temp_planner = TrajectoryPlanner(this.cfg.drone1_init_states(1:3), waypoints, this.cfg.target_speed, this.lookahead_dt);
            [~, init_vel, ~] = temp_planner.get_position(0);
            if norm(init_vel(1:2)) > 1e-3
                this.cfg.drone1_init_states(9) = atan2(init_vel(2), init_vel(1));
                this.prev_yaw = atan2(init_vel(2), init_vel(1));
            else
                this.cfg.drone1_init_states(9) = 0.0;
                this.prev_yaw = 0.0;
            end

            % Drone 인스턴스화 (PositionCtrl에 PPO 잔차 합성 기능 내장)
            this.drone1 = Drone(drone1_params, this.cfg.drone1_init_states, [0;0;0;0], ...
                this.cfg.posGain, this.cfg.attGain, this.cfg, waypoints);
            this.drone1.trajCtrl.lookahead_dt = this.lookahead_dt;

            this.max_steps     = ceil(this.drone1.tf / (this.cfg.dt * 10)) + 30;
            this.current_step  = 0;
            this.prev_action   = zeros(4, 1);
            this.prev_u_moment = zeros(3, 1);

            % 초기 관측치 계산 (11x1)
            state = this.drone1.GetState();
            [traj_pos, ~, kappa_now, kappa_future] = ...
                this.drone1.trajCtrl.get_position(0, this.lookahead_dt);

            e_x   = traj_pos(1) - state(1);
            e_y   = traj_pos(2) - state(2);
            e_z   = traj_pos(3) - state(3);
            v_x   = state(4);
            v_y   = state(5);
            v_z   = state(6);
            phi   = state(7);
            theta = state(8);
            v_des = this.cfg.target_speed;

            InitialObservation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta; v_des; kappa_now; kappa_future];
        end
    end
end
