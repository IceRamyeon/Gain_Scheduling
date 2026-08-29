function [drone1, obstacles] = init_drone_and_env(cfg)
    % INIT_DRONE_AND_ENV 드론 객체 생성 및 환경(장애물, 웨이포인트) 초기화 함수
    
    %% 1. 드론 초기 파라미터 셋업 (물리량 및 초기 상태)
    drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'},...
        {1.25, 0.265, 0.0232, 0.0232, 0.0468});

    % 초기 상태: [x, y, z, dx, dy, dz, phi, theta, psi, p, q, r]'
    drone1_initStates = [ 0.0, -4.5, -0.0, ...                                      
                          0, 0, 0, ...                                                                
                          0, 0, 0, ...                                                                
                          0, 0, 0]';                                                                  

    % 초기 입력: [ThrottleCMD, R, P, Y CMD]'
    drone1_initInputs = [0, ...                                                     
                         0, 0, 0]';                                                                  

    %% 2. Command Signals (웨이포인트)
    % desired [x; y; z] 궤적
    waypoints = [ 0.0,  0.5,  0.0, -0.5,  0.0,  0.0,  1.0,  0.0,  0.0;  
                 -3.8, -2.0, -2.8, -2.8,  1.8,  1.8,  1.8,  4.6,  5.0;  
                 -6.0, -5.0,  0.0, -5.0, -5.0, -0.0, -5.0, -5.0, -0.0]; 

    %% 3. 장애물 세팅 (Obstacles)
    % info: [x_min x_max; y_min y_max; z_min z_max]
    limbo1 = [-5.0,  5.0;
              -3.0, -2.5;
              -5.0,  0.0];

    limbo2 = [-5.0,  5.0;
               0.0,  0.5;
              -4.0,  0.0];

    limbo3 = [-2.0,  2.0;
               3.0,  3.5;
              -4.0,  0.0];

    obstacles = {limbo1, limbo2, limbo3};
    % obstacles = {0; 0; 0}; % 장애물 없을 때 쓸 수 있게 냅둔 주석

    %% 4. 드론 객체 생성 (BIRTH OF A DRONE)
    % main에서 넘어온 cfg를 사용해서 게인 값과 simTime을 전달
    drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, ...
        cfg.posGain, cfg.attGain, cfg.simTime, ...
        waypoints, obstacles);
        
end