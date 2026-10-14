classdef RL_DroneEnv_LPV < rl.env.MATLABEnvironment
    % RL_DroneEnv_LPV: Drone_LPV 피드포워드 + Z축 스케줄링 기반 잔차 강화학습 환경
    % Observation: [e_x, e_y, e_z, v_x, v_y, v_z, phi, theta] (8x1)
    % Action: [a_P_xy, a_D_xy, a_P_z, a_D_z] in [-1, 1]^4 (D 게인 중심 잔차 보정)
    
    properties
        cfg
        drone1
        current_step
        max_steps
        
        % Residual Action Scaling Ranges
        range_P_xy = 1.0;
        range_D_xy = 1.2; % 초고속 댐핑 억제를 위해 D 게인 범위 확장
        range_P_z  = 1.5;
        range_D_z  = 1.0;
        
        prev_yaw
        prev_action
        prev_u_moment
        
        % Randomization options
        randomize_speed = true;
        speed_range = [2.0, 7.5];
        randomize_path = true;
        max_kappa = 0.25;
        max_tilt_deg = 75.0;
        
        % Curriculum options
        use_curriculum = true;
        episode_count = 0;
        total_episodes = 500;
    end
    
    methods
        function this = RL_DroneEnv_LPV()
            obsInfo = rlNumericSpec([8 1]);
            obsInfo.Name = 'Drone States';
            
            actInfo = rlNumericSpec([4 1], 'LowerLimit', -1, 'UpperLimit', 1);
            actInfo.Name = 'Residual Gain Offsets';
            
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
            
            this.max_steps = 1000;
            this.prev_action = zeros(4,1);
            
            % 기본 Drone_LPV 파라미터 초기화
            cfg = struct();
            cfg.dt = 0.01;
            cfg.target_yaw = [];
            cfg.target_speed = 4.0;
            cfg.Feedforward = true;
            cfg.Z_Gain_Scheduling = true;
            cfg.play_animation = false;
            cfg.auto_save = false;
            cfg.start_with_yaw = true;
            cfg.drone1_init_states = [0.0, 0.0, -6.0, cfg.target_speed, 0, 0, 0, 0, 0, 0, 0, 0]';
            cfg.m = 1.25; cfg.armLength = 0.265; 
            cfg.Ixx = 0.0232; cfg.Iyy = 0.0232; cfg.Izz = 0.0468;
            cfg.posGain = containers.Map({'P_x','I_x','D_x','P_y','I_y','D_y','P_z','I_z','D_z'}, ...
                {3.17, 0.001, 0.81, 3.17, 0.001, 0.81, 4.0, 0.001, 0.0});
            cfg.attGain = containers.Map({'P_phi','I_phi','D_phi','P_theta','I_theta','D_theta','P_psi','I_psi','D_psi','P_zdot','I_zdot','D_zdot'}, ...
                {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0});
            this.cfg = cfg;
        end
        
        function [Observation, Reward, IsDone, LoggedSignals] = step(this, Action)
            % 1. PPO Action -> 잔차 게인 오프셋으로 변환하여 주입
            % D 게인 하한선 보호: 댐핑을 삭감하는 음수 오프셋을 원천 차단하고 오직 댐핑 보강(+0 ~ +range)만 허용
            dP_xy = this.range_P_xy * Action(1);
            dD_xy = max(0, this.range_D_xy * Action(2));  % D_xy 하한선 보호
            dP_z  = this.range_P_z  * Action(3);
            dD_z  = max(0, this.range_D_z  * Action(4));   % D_z 하한선 보호
            
            this.drone1.posCtrl.setResidualOffsets(dP_xy, dD_xy, dP_z, dD_z);
            
            % 2. 서브스텝 시뮬레이션 (10스텝 = 0.1초 동안 제어 유지)
            N_sub = 10;
            
            for k = 1:N_sub
                drone_state = this.drone1.GetState();
                t_now = this.drone1.t;
                [traj_pos, traj_vel, kappa_avg] = this.drone1.trajCtrl.get_position(t_now);
                
                % 동적 Yaw 계산
                speed_xy = norm(traj_vel(1:2));
                if speed_xy > 1e-3
                    raw_yaw = atan2(traj_vel(2), traj_vel(1));
                    yaw_diff = raw_yaw - this.prev_yaw;
                    while yaw_diff > pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                    while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                    this.prev_yaw = raw_yaw;
                end
                
                % 위치 제어기 (직접 Thrust U1 지령 생성)
                current_cmd = [traj_pos; this.prev_yaw; this.cfg.target_speed; kappa_avg];
                att_cmd = this.drone1.posCtrl.Update(current_cmd, drone_state);
                
                % 자세 제어기 (추력 패스스루)
                u_motor = this.drone1.attCtrl.Update(att_cmd, drone_state);
                
                % 동역학 전파
                this.drone1.UpdateState(u_motor);
            end
            
            this.current_step = this.current_step + 1;
            
            % 3. 새로운 관측치 수집
            state = this.drone1.GetState();
            [traj_pos, traj_vel, ~] = this.drone1.trajCtrl.get_position(this.drone1.t);
            
            e_x = traj_pos(1) - state(1);
            e_y = traj_pos(2) - state(2);
            e_z = traj_pos(3) - state(3);
            v_x = state(4);
            v_y = state(5);
            v_z = state(6);
            phi = state(7);
            theta = state(8);
            
            Observation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
            
            % 4. 보상 계산
            err_pos = norm([e_x, e_y, e_z]);
            err_vel = norm(traj_vel - state(4:6));
            rates   = state(10:12);
            
            % 정밀 추종 커널 + 속도/자세 패널티 + 액션 평활화
            r_pos    = 6.0 * exp(- (err_pos^2) / (2 * 0.08^2)) - 2.5 * err_pos;
            r_vel    = max(- 0.15 * (err_vel^2), -40);
            r_att    = max(- 0.1 * (phi^2 + theta^2) - 0.03 * norm(rates)^2, -40);
            r_smooth = - 0.005 * sum(Action.^2) - 0.02 * sum((Action - this.prev_action).^2);
            
            % 모멘트(U2, U3, U4) 고주파 채터링 및 과대 진동 억제 페널티
            curr_u_moment = u_motor(2:4);
            r_moment = max(- 0.02 * sum(curr_u_moment.^2) - 0.05 * sum((curr_u_moment - this.prev_u_moment).^2), -30);
            this.prev_u_moment = curr_u_moment;
            
            r_alive  = 0.1;
            
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
            
            % 500 에피소드 맞춤형 커리큘럼 (3단계 점진적 난이도 상향)
            if this.use_curriculum
                ep = this.episode_count;
                if ep <= 150
                    % Stage 1: 저~중속 기본 안정화 (2.0 ~ 5.0 m/s, kappa <= 0.15)
                    cur_v_min = 2.0; cur_v_max = 5.0;
                    cur_k_max = 0.15;
                elseif ep <= 350
                    % Stage 2: 중~고속 과도 적응 (3.0 ~ 6.5 m/s, kappa <= 0.22)
                    cur_v_min = 3.0; cur_v_max = 6.5;
                    cur_k_max = 0.22;
                else
                    % Stage 3: 초고속 및 급선회 댐핑 마스터 (4.0 ~ 7.5 m/s, kappa <= 0.30)
                    cur_v_min = 4.0; cur_v_max = 7.5;
                    cur_k_max = 0.30;
                end
                
                this.speed_range = [cur_v_min, cur_v_max];
                this.max_kappa = cur_k_max;
            end
            
            % 속도 무작위화
            if this.randomize_speed
                v_rand = this.speed_range(1) + (this.speed_range(2) - this.speed_range(1)) * rand();
                this.cfg.target_speed = v_rand;
                this.cfg.drone1_init_states(4) = v_rand;
            end
            
            % 궤적 생성 (랜덤 또는 고정)
            if this.randomize_path
                % PID_Drone의 검증된 다양한 곡률 궤적 생성
                [wp_x, wp_y, wp_z] = pathGen(true, this.max_kappa);
            else
                [wp_x, wp_y, wp_z] = pathGen(false);
            end
            waypoints = [wp_x; wp_y; wp_z];
            
            % 드론 객체 생성 (RL_PositionCtrl_LPV 탑재)
            drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'},...
                {this.cfg.m, this.cfg.armLength, this.cfg.Ixx, this.cfg.Iyy, this.cfg.Izz});
            
            % 초기 Yaw 정렬
            temp_planner = TrajectoryPlanner(this.cfg.drone1_init_states(1:3), waypoints, this.cfg.target_speed);
            [~, init_vel, ~] = temp_planner.get_position(0);
            if norm(init_vel(1:2)) > 1e-3
                this.cfg.drone1_init_states(9) = atan2(init_vel(2), init_vel(1));
                this.prev_yaw = atan2(init_vel(2), init_vel(1));
            else
                this.cfg.drone1_init_states(9) = 0.0;
                this.prev_yaw = 0.0;
            end
            
            % Drone 인스턴스화
            this.drone1 = Drone(drone1_params, this.cfg.drone1_init_states, [0;0;0;0], ...
                this.cfg.posGain, this.cfg.attGain, this.cfg, waypoints);
            
            % 제어기를 잔차 합성 가능한 RL_PositionCtrl_LPV로 교체
            this.drone1.posCtrl = RL_PositionCtrl_LPV(this.cfg.posGain, drone1_params, this.cfg);
            
            this.max_steps = ceil(this.drone1.tf / (this.cfg.dt * 10)) + 30;
            this.current_step = 0;
            this.prev_action = zeros(4,1);
            this.prev_u_moment = zeros(3,1);
            
            % 초기 관측치 계산
            state = this.drone1.GetState();
            [traj_pos, ~] = this.drone1.trajCtrl.get_position(0);
            
            e_x = traj_pos(1) - state(1);
            e_y = traj_pos(2) - state(2);
            e_z = traj_pos(3) - state(3);
            v_x = state(4);
            v_y = state(5);
            v_z = state(6);
            phi = state(7);
            theta = state(8);
            
            InitialObservation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
        end
    end
end
