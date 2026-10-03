classdef DroneEnv < rl.env.MATLABEnvironment
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
    end
    
    methods
        function this = DroneEnv()
            % Define Observation Space (8-dim)
            obsInfo = rlNumericSpec([8 1]);
            obsInfo.Name = 'Drone States';
            obsInfo.Description = 'e_x, e_y, e_z, v_x, v_y, v_z, phi, theta';
            
            % Define Action Space (4-dim, continuous between -1 and 1)
            actInfo = rlNumericSpec([4 1], 'LowerLimit', -1, 'UpperLimit', 1);
            actInfo.Name = 'Gain Tuning';
            actInfo.Description = 'Delta P_x, D_x, P_y, D_y';
            
            % Call superclass constructor
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
            
            % Initialize simulation config
            this.cfg = struct();
            this.cfg.drone1_init_states = [0; 0; -6.0; 0;0;0; 0;0;0; 0;0;0];
            this.cfg.simTime = 10; % Shorter episode for training
            this.cfg.dt = 0.01;
            this.cfg.target_yaw = [];
            
            this.cfg.posGain = containers.Map(...
                {'P_x','I_x','D_x', 'P_y','I_y','D_y', 'P_z','I_z','D_z'},...
                {3.17, 0.001, 0.81, 3.17, 0.001, 0.81, 4.0, 0.001, 0.0});
            
            this.cfg.attGain = containers.Map(...
                {'P_phi','I_phi','D_phi', 'P_theta','I_theta','D_theta', 'P_psi','I_psi','D_psi', 'P_zdot','I_zdot','D_zdot'},...
                {18.10, 0.001, 0.93, 18.10, 0.001, 0.93, 36.51, 0.001, 1.87, 25.0, 0.001, 0.0});
            
            this.max_steps = ceil(this.cfg.simTime / this.cfg.dt);
        end
        
        function [Observation, Reward, IsDone, LoggedSignals] = step(this, Action)
            % --- 1. Baseline Gain Scheduling (Lookup Table) ---
            % 동작점(Operating Point): 드론의 수평 속도 (XY 평면)
            drone1_state = this.drone1.GetState();
            v_xy = norm(drone1_state(4:5)); % 현재 수평 속도
            
            % 사전에 계산된 동작점별 PID 계수 (Look-up Table)
            op_speeds = [0.0, 2.0, 4.0, 6.0, 8.0]; % 동작점 (m/s)
            op_Px     = [3.17, 3.50, 4.00, 4.20, 4.50]; % 사전에 튜닝된 P 게인
            op_Dx     = [0.81, 0.90, 1.00, 1.10, 1.20]; % 사전에 튜닝된 D 게인
            
            % 동작점 사이의 기본 게인값을 선형 보간(Linear Interpolation)으로 구함
            base_P_current = interp1(op_speeds, op_Px, v_xy, 'linear', 'extrap');
            base_D_current = interp1(op_speeds, op_Dx, v_xy, 'linear', 'extrap');
            
            % --- 2. RL Agent's Residual Action ---
            % 보간된 미지의 기본 계수값에 RL 에이전트의 Action을 잔차(Residual/Offset)로 더함
            % RL 에이전트는 이 오프셋을 조절하여 보간법의 한계를 극복하는 최적의 미지 계수를 찾아냄
            P_x_new = base_P_current + this.range_P * Action(1);
            D_x_new = base_D_current + this.range_D * Action(2);
            P_y_new = base_P_current + this.range_P * Action(3);  % Y축도 대칭적으로 같은 베이스 적용
            D_y_new = base_D_current + this.range_D * Action(4);
            
            % Clamp to be safe (must be > 0.1 to avoid instability)
            P_x_new = max(0.1, P_x_new);
            D_x_new = max(0.01, D_x_new);
            P_y_new = max(0.1, P_y_new);
            D_y_new = max(0.01, D_y_new);
            
            % Apply gains to PositionCtrl
            this.drone1.posCtrl.setGains(P_x_new, D_x_new, P_y_new, D_y_new);
            
            % 2. Run simulation for 1 step (10ms)
            % Or alternatively, run for N steps per RL step to speed up learning. Let's do 5 steps (50ms).
            N_substeps = 5;
            penalty_action = sum(Action.^2);
            pos_error_sum = 0;
            
            for k = 1:N_substeps
                drone1_state = this.drone1.GetState();
                current_time = this.drone1.t;
                
                [traj_pos, traj_vel] = this.drone1.trajCtrl.get_position(current_time);
                
                speed_xy = norm(traj_vel(1:2));
                if speed_xy > 1e-3
                    raw_yaw = atan2(traj_vel(2), traj_vel(1));
                    yaw_diff = raw_yaw - this.prev_yaw;
                    while yaw_diff > pi, raw_yaw = raw_yaw - 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                    while yaw_diff < -pi, raw_yaw = raw_yaw + 2*pi; yaw_diff = raw_yaw - this.prev_yaw; end
                    this.prev_yaw = raw_yaw; 
                end
                dynamic_target_yaw = this.prev_yaw;
                
                current_cmd = [traj_pos; dynamic_target_yaw];
                
                att_cmd = this.drone1.posCtrl.Update(current_cmd, drone1_state);
                u_motor = this.drone1.attCtrl.Update(att_cmd, drone1_state);
                this.drone1.UpdateState(u_motor);
                
                this.current_step = this.current_step + 1;
                
                err_pos = norm(traj_pos - drone1_state(1:3));
                pos_error_sum = pos_error_sum + err_pos^2;
            end
            
            % 3. Calculate Observation
            drone_state = this.drone1.GetState();
            [traj_pos, ~] = this.drone1.trajCtrl.get_position(this.drone1.t);
            
            e_x = traj_pos(1) - drone_state(1);
            e_y = traj_pos(2) - drone_state(2);
            e_z = traj_pos(3) - drone_state(3);
            v_x = drone_state(4);
            v_y = drone_state(5);
            v_z = drone_state(6);
            phi = drone_state(7);
            theta = drone_state(8);
            
            Observation = [e_x; e_y; e_z; v_x; v_y; v_z; phi; theta];
            
            % 4. Calculate Reward
            % Reward is negative of error and effort
            Reward = - (this.w_pos * (pos_error_sum / N_substeps) + this.w_ctrl * penalty_action);
            
            % 5. Check Termination
            IsDone = false;
            
            % Terminate if episode is over
            if this.current_step >= this.max_steps
                IsDone = true;
            end
            
            % Terminate and penalize heavily if out of bounds (Crash)
            if norm([e_x, e_y, e_z]) > 5.0 || abs(phi) > deg2rad(60) || abs(theta) > deg2rad(60)
                IsDone = true;
                Reward = Reward - 1000; % Heavy penalty for crashing
            end
            
            LoggedSignals = [];
        end
        
        function InitialObservation = reset(this)
            % Initialize Drone and Environment
            this.drone1 = init_drone_and_env(this.cfg);
            this.current_step = 0;
            
            init_vel = this.drone1.trajCtrl.get_position(0);
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
