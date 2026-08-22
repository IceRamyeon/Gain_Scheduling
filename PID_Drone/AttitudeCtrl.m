classdef AttitudeCtrl < handle
    %% MEMBERS (설계도: 여기에 있는 이름만 obj.이름 으로 쓸 수 있음!)
    properties
        m
        g
        dt
        x
        cmd

        phi_des
        theta_des
        psi_des
        zdot_des
        
        phi_err
        phi_err_prev
        phi_err_sum
        
        theta_err
        theta_err_prev
        theta_err_sum
        
        psi_err
        psi_err_prev
        psi_err_sum
        
        zdot_err
        zdot_err_prev
        zdot_err_sum
    
        kP_phi, kD_phi, kI_phi
        kP_theta, kD_theta, kI_theta
        kP_psi, kD_psi, kI_psi
        kP_zdot, kD_zdot, kI_zdot
    end
    
    %% METHODS
    methods
       %% CONSTRUCTOR
       function obj = AttitudeCtrl(params, gains, dt)
            obj.g = 9.81;
            obj.dt = dt;
            obj.m = params('mass');
            
            obj.kP_phi = gains('P_phi'); obj.kI_phi = gains('I_phi'); obj.kD_phi = gains('D_phi');
            obj.kP_theta = gains('P_theta'); obj.kI_theta = gains('I_theta'); obj.kD_theta = gains('D_theta');
            obj.kP_psi = gains('P_psi'); obj.kI_psi = gains('I_psi'); obj.kD_psi = gains('D_psi');
            obj.kP_zdot = gains('P_zdot'); obj.kI_zdot = gains('I_zdot'); obj.kD_zdot = gains('D_zdot');
            
            obj.Reset();
       end
       
        %% Reset
        function Reset(obj)
            obj.phi_err = 0; obj.phi_err_prev = 0; obj.phi_err_sum = 0;
            obj.theta_err = 0; obj.theta_err_prev = 0; obj.theta_err_sum = 0;
            obj.psi_err = 0; obj.psi_err_prev = 0; obj.psi_err_sum = 0;
            obj.zdot_err = 0; obj.zdot_err_prev = 0; obj.zdot_err_sum = 0;
        end

        %% GetState
        function state = GetState(obj)
            state = obj.x;
        end

        %% Get cmd
        function cmd = commandSig(obj)
            cmd = obj.cmd;
        end

        %% Update
        function u = Update(obj, cmd, state)
            
            obj.phi_des = cmd(1); 
            obj.theta_des = cmd(2); 
            obj.psi_des = cmd(3); 
            obj.zdot_des = cmd(4);
       
            phi = state(7); 
            theta = state(8); 
            psi = state(9); 
            zdot = state(6);
            
            obj.phi_err = obj.phi_des - phi;
            obj.theta_err = obj.theta_des - theta;
            obj.psi_err = obj.psi_des - psi;
            obj.zdot_err = obj.zdot_des - zdot;
            
            % Roll
            u_phi = obj.kP_phi * obj.phi_err + ...
                    obj.kI_phi * obj.phi_err_sum + ...
                    obj.kD_phi * (obj.phi_err - obj.phi_err_prev)/obj.dt;
            
            % Pitch
            u_theta = obj.kP_theta * obj.theta_err + ...
                      obj.kI_theta * obj.theta_err_sum + ...
                      obj.kD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt;
            
            % Yaw
            u_psi = obj.kP_psi * obj.psi_err + ...
                    obj.kI_psi * obj.psi_err_sum + ...
                    obj.kD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt;
            
            % zdot (Thrust)
            u_zdot_pid = obj.kP_zdot * obj.zdot_err + ...
                         obj.kI_zdot * obj.zdot_err_sum + ...
                         obj.kD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt;
                     
            u_thrust = (obj.m * obj.g) - u_zdot_pid; % (좌표계 부호 확인 필요!)

            obj.phi_err_sum = obj.phi_err_sum + obj.phi_err * obj.dt;
            obj.phi_err_prev = obj.phi_err;
            
            obj.theta_err_sum = obj.theta_err_sum + obj.theta_err * obj.dt;
            obj.theta_err_prev = obj.theta_err;
            
            obj.psi_err_sum = obj.psi_err_sum + obj.psi_err * obj.dt;
            obj.psi_err_prev = obj.psi_err;
            
            obj.zdot_err_sum = obj.zdot_err_sum + obj.zdot_err * obj.dt;
            obj.zdot_err_prev = obj.zdot_err;

            % D2R = 180/pi; % Conversion factor from degrees to radians

            thrust_max = 500;
            phi_max = 8.0;
            theta_max = 8.0; 
            
            % thrust 제한
            u_thrust = max(-thrust_max, min(u_thrust, thrust_max));

            % phi (Roll) 제한
            u_phi = max(-phi_max, min(u_phi, phi_max));

            % theta (Pitch) 제한
            u_theta = max(-theta_max, min(u_theta, theta_max));

            u = [u_thrust; u_phi; u_theta; u_psi];
        end
    end
end