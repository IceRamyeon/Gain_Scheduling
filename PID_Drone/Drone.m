    classdef Drone < handle
    %% MEMBERS
    properties
        g
        t
        dt
        tf

        m
        l
        I

        x                   % [X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r]
        r                   % [X, Y, Z]
        dr                  % [dX, dY, dZ]
        euler               % [phi, theta, psi]
        w                   % [p, q, r]
        dx
        

        u                   % [T_sum, M1, M2, M3]'
        T                   % T_sum
        M                   % [M1, M2, M3]'

        trajCtrl
        posCtrl
        attCtrl

        obs
    end

    %% METHODS
    methods
       %% CONSTRUCTOR
       function obj = Drone(params, initStates, initInputs, posGains, ...
               attGains, simTime, waypoints)
           obj.g = 9.81;
           obj.t = 0.0;
           obj.dt = 0.01;
           obj.tf = simTime;

           obj.m = params('mass');
           obj.l = params('armLength');
           obj.I = [params('Ixx'), 0, 0; ... 
                    0, params('Iyy'), 0; ...
                    0, 0, params('Izz')];

            obj.x = initStates;
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
            obj.dx = zeros(12,1);

            obj.u = initInputs;
            
            obj.trajCtrl = TrajectoryCtrl(initStates(1:3), waypoints, simTime);
            obj.posCtrl = PositionCtrl(posGains, obj.dt);
            obj.attCtrl = AttitudeCtrl(params, attGains, obj.dt);
        end

        %% GetState
            function state = GetState(obj)
                state = obj.x;
            end

            %% EvalEOM
            function obj = EvalEOM(obj)

                bRi = RPY2Rot(obj.euler);
                R = bRi';

                obj.dx(1:3) = obj.dr; % x(4:6)
                
                obj.dx(4:6) = 1 / obj.m * ([0 ; 0; obj.m * obj.g] ...
                    + R * obj.T * [0 ; 0; -1]);

                phi = obj.euler(1); 
                theta = obj.euler(2);

                W_mat =   [1    sin(phi)*tan(theta) cos(phi)*tan(theta);
                           0    cos(phi)            -sin(phi);
                           0    sin(phi)*sec(theta) cos(phi)*sec(theta)];
            
                obj.dx(7:9) = W_mat * obj.w;

                obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));
            end

            %% UpdateState
            function obj = UpdateState(obj, motor_inputs)
                obj.u = motor_inputs;
                obj.T = obj.u(1);
                obj.M = obj.u(2:4);

                obj.t = obj.t + obj.dt;
                obj.EvalEOM();
                obj.x = obj.x + obj.dx .* obj.dt;

                obj.r = obj.x(1:3);
                obj.dr = obj.x(4:6);
                obj.euler = obj.x(7:9);
                obj.w = obj.x(10:12);
            end

    end
    end