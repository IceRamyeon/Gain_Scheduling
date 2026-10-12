classdef RL_DroneEnv < rl.env.MATLABEnvironment
    % DRONEENV: RL Environment for Drone Gain Scheduling
    % Observation: [e_x, e_y, e_z, v_x, v_y, v_z, phi, theta] (8x1)
    % Action: [P_x, D_x, P_y, D_y] (Normalized -1 to 1)
    
    properties
        % Simulation variables
        cfg
        drone1
        current_step
        max_steps
        
        % Base/Nominal gains and ranges
        base_P_x = 3.17; range_P = 2.0;
        base_D_x = 0.81; range_D = 0.5;
        base_P_y = 3.17; 
        base_D_y = 0.81; 
        
        % Reward weights
        w_pos = 1.0;
        w_ctrl = 0.05;
        
        prev_yaw
        prev_action
        
        % Loaded PID Table Data
        pid_table
        
        % Randomization options for robust training (Domain Randomization)
        randomize_speed = false;
        speed_range = [2.0, 10.0];
        randomize_path = false;
        max_kappa = 0.35;
        max_tilt_deg = 75.0;
        
        % 2D Curriculum Learning options (Speed & Curvature)
        use_curriculum = false;
        episode_count = 0;
        curriculum_total_episodes = 1200;
    end
    
    methods
        function this = RL_DroneEnv()
            % Define Observation Space (8-dim)
            obsInfo = rlNumericSpec([8 1]);
            obsInfo.Name = 'Drone States';
            
            % Define Action Space (4-dim)
            actInfo = rlNumericSpec([4 1], 'LowerLimit', -1, 'UpperLimit', 1);
            actInfo.Name = 'Gain Offsets';
            
            % Initialize Environment
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
            
            % Env state
            this.max_steps = 1000;
            this.prev_action = zeros(4,1);
            
            % Initialize default configuration for the drone
            cfg = struct();
            cfg.dt = 0.01;
            cfg.target_speed = 4;
            cfg.drone1_init_states = [ 0.0, 0.0, -6.0, cfg.target_speed, 0, 0, 0, 0, 0, 0, 0, 0]';
            cfg.posGain = containers.Map(...
                 {'P_x','I_x','D_x', 'P_y','I_y','D_y', 'P_z','I_z','D_z'},...
                 {3.17, 0.001, 0.81, 3.17, 0.001, 0.81, 4.0, 0.001, 0.0});
            cfg.attGain = containers.Map(...
                 {'P_phi','I_phi','D_phi', 'P_theta','I_theta','D_theta', 'P_psi','I_psi','D_psi', 'P_zdot','I_zdot','D_zdot'},...
                 {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0});
            this.cfg = cfg;
            
            % Load PID Table for Gain Scheduling
            try
                this.pid_table = load('PID_Table.mat');
            catch
                warning('PID_Table.mat not found! Please run Build_PID_Table.m first.');
                this.pid_table = [];
            end
        end
        
        function [Observation, Reward, IsDone, LoggedSignals] = step(this, Action)
            % --- 1. Baseline Gain Scheduling (Lookup Table) ---
            % 동작점(Operating Point): 드론의 수평 속도 (XY 평면) 및 궤적 곡률
            drone1_state = this.drone1.GetState();
            v_xy = norm(drone1_state(4:5)); % 현재 수평 속도
            
            % 현재 궤적 곡률(kappa) 계산
            [~, traj_vel, traj_acc] = this.drone1.trajCtrl.get_position(this.drone1.t);
            v_norm = norm(traj_vel);
            if v_norm > 1e-3
                vel_vec3 = reshape(traj_vel(1:3), 3, 1);
                acc_vec3 = reshape(traj_acc(1:3), 3, 1);
                kappa = norm(cross(vel_vec3, acc_vec3)) / (v_norm^3);
            else
                kappa = 0;
            end
            
            % PID_Table.mat의 2D Interpolation (속도 v, 곡률 kappa)
            if ~isempty(this.pid_table)
                % 클램핑 (extrapolation 방지)
                v_xy_c = max(min(v_xy, max(this.pid_table.v_vec)), min(this.pid_table.v_vec));
                kappa_c = max(min(kappa, max(this.pid_table.kappa_vec)), min(this.pid_table.kappa_vec));
                
                base_P_current_x = interp2(this.pid_table.kappa_vec, this.pid_table.v_vec, this.pid_table.Kp_x, kappa_c, v_xy_c, 'linear');
                base_D_current_x = interp2(this.pid_table.kappa_vec, this.pid_table.v_vec, this.pid_table.Kd_x, kappa_c, v_xy_c, 'linear');
                base_P_current_y = abs(interp2(this.pid_table.kappa_vec, this.pid_table.v_vec, this.pid_table.Kp_y, kappa_c, v_xy_c, 'linear'));
                base_D_current_y = abs(interp2(this.pid_table.kappa_vec, this.pid_table.v_vec, this.pid_table.Kd_y, kappa_c, v_xy_c, 'linear'));
            else
                % 테이블이 없으면 하드코딩된 기본값 사용
                base_P_current_x = this.base_P_x; base_D_current_x = this.base_D_x;
                base_P_current_y = abs(this.base_P_y); base_D_current_y = abs(this.base_D_y);
            end
            
            % --- 2. RL Agent's Residual Action ---
            % 보간된 미지의 기본 계수값에 RL 에이전트의 Action을 잔차(Residual/Offset)로 더함
            P_x_new = base_P_current_x + this.range_P * Action(1);
            D_x_new = base_D_current_x + this.range_D * Action(2);
            P_y_new = base_P_current_y + this.range_P * Action(3);
            D_y_new = base_D_current_y + this.range_D * Action(4);
            
            % 안정성을 위한 최소 게인 제한
            P_x_new = max(P_x_new, 0.1); D_x_new = max(D_x_new, 0.01);
            P_y_new = max(P_y_new, 0.1); D_y_new = max(D_y_new, 0.01);
            
            % 드론 제어기에 새로운 게인 설정
            this.drone1.posCtrl.setGains(P_x_new, D_x_new, P_y_new, D_y_new);
            
            % --- 3. Step Simulation ---
            % 서브스텝(예: 10스텝)만큼 시뮬레이션 진행 (RL 에이전트의 1 step)
            N_sub = 10;
            err_pos_sum = 0;
            
            for k = 1:N_sub
                % Trajectory에서 현재 목표 위치 가져오기
                [traj_pos, ~] = this.drone1.trajCtrl.get_position(this.drone1.t);
                traj_cmd = [traj_pos; this.prev_yaw];
                
                % Drone Update (제어기 + 동역학)
                current_state = this.drone1.GetState();
                att_cmd = this.drone1.posCtrl.Update(traj_cmd, current_state);
                u_motor = this.drone1.attCtrl.Update(att_cmd, current_state);
                this.drone1.UpdateState(u_motor);
                
                % 위치 오차 누적 (보상 계산용)
                current_pos = this.drone1.GetState();
                err_pos_sum = err_pos_sum + norm(traj_pos - current_pos(1:3))^2;
            end
            
            this.current_step = this.current_step + 1;
            
            % --- 4. Get New Observation ---
            drone_state = this.drone1.GetState();
            [traj_pos, traj_vel] = this.drone1.trajCtrl.get_position(this.drone1.t);
            
            e_x = traj_pos(1) - drone_state(1);
            e_y = traj_pos(2) - drone_state(2);
            e_z = traj_pos(3) - drone_state(3);
            v_x = drone_state(4);
            v_y = drone_state(5);
            v_z = drone_state(6);
            phi = drone_state(7);
            theta = drone_state(8);
            
            Observation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
            
            % --- 5. Calculate Multiobjective Reward ---
            err_pos = norm([e_x, e_y, e_z]);
            err_vel = norm(traj_vel - drone_state(4:6));
            rates   = drone_state(10:12); % [p, q, r]
            
            r_pos    = 5.0 * exp(- (err_pos^2) / (2 * 0.1^2)) - 2.0 * err_pos;
            r_vel    = max(- 0.2 * (err_vel^2), -50);
            r_att    = max(- 0.1 * (phi^2 + theta^2) - 0.05 * norm(rates)^2, -50);
            r_smooth = - 0.005 * sum(Action.^2) - 0.02 * sum((Action - this.prev_action).^2);
            r_alive  = 0.1;

            Reward = r_pos + r_vel + r_att + r_smooth + r_alive;
            this.prev_action = Action; % Update previous action
            
            % --- 6. Check Termination ---
            IsDone = false;
            
            % Terminate if trajectory completed or max steps reached
            if this.drone1.t >= this.drone1.tf || this.current_step >= this.max_steps
                IsDone = true;
                Reward = Reward + 30.0; % 완주 성공 보너스
            end
            
            % Terminate and penalize heavily if out of bounds (Crash)
            % 고속 선회 시 뱅크각(Roll/Pitch)이 60도를 넘길 수 있으므로 max_tilt_deg(기본 75도)로 안정성 확보
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
            
            % 2D 커리큘럼 스케줄링 (속력 및 곡률 난이도 단계적 상향, 총 5000회 기준)
            if this.use_curriculum
                ep = this.episode_count;
                if ep <= 1000
                    % Stage 1: 입문 단계 (저속 + 완만한 곡률, 1~1000회)
                    cur_v_min = 2.0; cur_v_max = 5.0;
                    cur_k_max = 0.10;
                    stage_str = 'Stage 1 [입문]: 저속(2~5 m/s) 및 완만한 곡률(k<=0.10)';
                elseif ep <= 2500
                    % Stage 2: 중급 단계 (중속 + 중간 곡률, 1001~2500회)
                    cur_v_min = 2.0; cur_v_max = 7.5;
                    cur_k_max = 0.22;
                    stage_str = 'Stage 2 [중급]: 중속(2~7.5 m/s) 및 일반 곡률(k<=0.22)';
                else
                    % Stage 3: 고급 단계 (전 속도 및 급격한 곡률 마스터, 2501~5000회)
                    cur_v_min = 2.0; cur_v_max = 10.0;
                    cur_k_max = 0.35;
                    stage_str = 'Stage 3 [고급]: 고속(2~10 m/s) 및 급격한 곡률(k<=0.35)';
                end
                
                if ismember(ep, [1, 1001, 2501])
                    fprintf('\n>>> [Curriculum Update] %s 시작 (Episode %d / 5000) <<<\n', stage_str, ep);
                end
                
                this.speed_range = [cur_v_min, cur_v_max];
                this.max_kappa = cur_k_max;
            end
            
            % 1. 속도 무작위화 (Domain Randomization)
            if this.randomize_speed
                v_rand = this.speed_range(1) + (this.speed_range(2) - this.speed_range(1)) * rand();
                this.cfg.target_speed = v_rand;
                this.cfg.drone1_init_states(4) = v_rand;
            end
            
            % 2. 궤적 곡률 무작위화 (Path Randomization)
            if this.randomize_path
                this.cfg.random_path = true;
                this.cfg.max_kappa = this.max_kappa;
            else
                this.cfg.random_path = false;
            end
            
            % Initialize Drone and Environment
            this.drone1 = init_drone_and_env(this.cfg);
            this.max_steps = ceil(this.drone1.tf / (this.cfg.dt * 10)) + 30;
            this.current_step = 0;
            this.prev_action = zeros(4,1);
            
            [~, init_vel] = this.drone1.trajCtrl.get_position(0);
            if norm(init_vel(1:2)) > 1e-3
                this.prev_yaw = atan2(init_vel(2), init_vel(1));
            else
                this.prev_yaw = 0.0; 
            end
            
            % Calculate Initial Observation
            drone_state = this.drone1.GetState();
            [traj_pos, ~] = this.drone1.trajCtrl.get_position(0);
            
            e_x = traj_pos(1) - drone_state(1);
            e_y = traj_pos(2) - drone_state(2);
            e_z = traj_pos(3) - drone_state(3);
            v_x = drone_state(4);
            v_y = drone_state(5);
            v_z = drone_state(6);
            phi = drone_state(7);
            theta = drone_state(8);
            
            InitialObservation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
        end
    end
end
