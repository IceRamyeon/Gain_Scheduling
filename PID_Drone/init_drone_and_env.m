function [drone1] = init_drone_and_env(cfg)
    % INIT_DRONE_AND_ENV 드론 객체 생성 및 환경(장애물, 웨이포인트) 초기화 함수
    
    %% 1. 드론 초기 파라미터 셋업 (물리량 및 초기 상태)
    drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'},...
        {1.25, 0.265, 0.0232, 0.0232, 0.0468}); %[cite: 1]

    % 초기 상태: [x, y, z, dx, dy, dz, phi, theta, psi, p, q, r]'
    drone1_initStates = cfg.drone1_init_states;

    % 초기 입력: [ThrottleCMD, R, P, Y CMD]'
    drone1_initInputs = [0, ...                                                     
                         0, 0, 0]'; %[cite: 1]                                                                 

    %% 2. Command Signals (웨이포인트)
    % pathGen 함수에서 x, y, z를 받아와서 행렬로 합치기
    [wp_x, wp_y, wp_z] = pathGen();
    waypoints = [wp_x; wp_y; wp_z];

    %% 4. 드론 객체 생성 (BIRTH OF A DRONE)
    % main에서 넘어온 cfg를 사용해서 게인 값과 simTime을 전달
    drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, ...
        cfg.posGain, cfg.attGain, cfg.simTime, ...
        waypoints); %[cite: 1]
        
end