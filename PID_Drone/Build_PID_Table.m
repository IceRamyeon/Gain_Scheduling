function [Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z] = Build_PID_Table()
    % 드론 물리 파라미터 (실제 기체 제원에 맞게 수정 필요)
    m = 1.25; 
    g = 9.81;

    % 1. 스케줄링 격자(Grid) 정의
    v_min = 0; 
    v_max = 10; 
    v_step = 1.0; 
    
    kappa_min = 0; 
    kappa_max = 0.5; 
    kappa_step = 0.05;

    v_vec = v_min:v_step:v_max;
    kappa_vec = kappa_min:kappa_step:kappa_max;

    n_v = length(v_vec);
    n_kappa = length(kappa_vec);

    % 출력용 테이블 메모리 사전 할당 (v_length x kappa_length 크기)
    Kp_x = zeros(n_v, n_kappa); Ki_x = zeros(n_v, n_kappa); Kd_x = zeros(n_v, n_kappa);
    Kp_y = zeros(n_v, n_kappa); Ki_y = zeros(n_v, n_kappa); Kd_y = zeros(n_v, n_kappa);
    Kp_z = zeros(n_v, n_kappa); Ki_z = zeros(n_v, n_kappa); Kd_z = zeros(n_v, n_kappa);

    % 2. 동작점(Grid) 순회하며 LPV 모델링 수행
    for i = 1:n_v
        for j = 1:n_kappa
            v = v_vec(i);
            kappa = kappa_vec(j);

            % [1] 파라미터 종속 평형점 (Trim Condition)
            phi_eq = atan(v^2 * kappa / g);
            U1_eq = m * sqrt(g^2 + (v^2 * kappa)^2);

            % [2] 2차원 LPV 상태공간 모델 행렬 A, B 구성 (psi = 0 기준)
            A = [zeros(3,3), eye(3);
                 zeros(3,3), zeros(3,3)];

            B_acc = [ 0,                           (U1_eq/m)*cos(phi_eq),  0;
                     -(U1_eq/m)*cos(phi_eq), 0,                           -(1/m)*sin(phi_eq);
                     -(U1_eq/m)*sin(phi_eq), 0,                            (1/m)*cos(phi_eq)];
            
            B = [zeros(3,3); 
                 B_acc];

            % [3] 적분(I) 제어를 위한 상태 확장 (State Augmentation)
            % 원래 상태 변수: [x, y, z, dx, dy, dz]^T (6x1)
            % 추가할 적분 변수: [int_x, int_y, int_z]^T (3x1)
            
            % 위치 오차(x, y, z)만 추출하는 3x6 행렬 생성
            C_I = [eye(3), zeros(3,3)]; 
            
            % 9x9 크기의 확장된 시스템 행렬 A_aug 구성
            % [ A (6x6)   , 0 (6x3) ]
            % [ C_I (3x6) , 0 (3x3) ]
            A_aug = [A, zeros(6,3);
                     C_I, zeros(3,3)];

            % 9x3 크기의 확장된 입력 행렬 B_aug 구성
            % [ B (6x3) ]
            % [ 0 (3x3) ]
            B_aug = [B;
                     zeros(3,3)];

            % [4] Pole Placement를 통한 PID 계수 산출
            % 요구사항: Overshoot <= 4%, t_ss <= 3s, Ess = 0
            % 1. Ess = 0 : 적분(I) 상태가 포함되어 자동 만족
            % 2. t_ss <= 3s : 실수부(sigma)가 -1.33 이하가 되도록 -1.5 이하로 설정
            % 3. Overshoot <= 4% : 감쇠비(zeta)가 0.715 이상이 되도록 허수부를 설정
            
            % 지배적 2차 극점(켤레 복소수) + 빠른 1차 극점(적분기용, 실수부 -5 부근)
            p_x = [-1.5 + 1.2i, -1.5 - 1.2i, -5.0]; 
            p_y = [-1.6 + 1.3i, -1.6 - 1.3i, -5.1]; 
            p_z = [-1.7 + 1.4i, -1.7 - 1.4i, -5.2]; 
            
            desired_poles = [p_x, p_y, p_z];
            
            % 3x9 크기의 상태 피드백 게인 행렬 K_aug 산출
            K_aug = place(A_aug, B_aug, desired_poles);
            
            % X축 제어는 Pitch(theta, 입력 2번)가 담당
            Kp_x(i,j) = K_aug(2, 1);
            Kd_x(i,j) = K_aug(2, 4);
            Ki_x(i,j) = K_aug(2, 7);
            
            % Y축 제어는 Roll(phi, 입력 1번)이 담당
            Kp_y(i,j) = K_aug(1, 2);
            Kd_y(i,j) = K_aug(1, 5);
            Ki_y(i,j) = K_aug(1, 8);
            
            % Z축 제어는 추력(U1, 입력 3번)이 담당
            Kp_z(i,j) = K_aug(3, 3);
            Kd_z(i,j) = K_aug(3, 6);
            Ki_z(i,j) = K_aug(3, 9);
        end
    end

    % 3. 산출된 PID 테이블을 .mat 파일로 저장
    save('PID_Table.mat', 'v_vec', 'kappa_vec', ...
         'Kp_x', 'Ki_x', 'Kd_x', ...
         'Kp_y', 'Ki_y', 'Kd_y', ...
         'Kp_z', 'Ki_z', 'Kd_z');
    
    disp('PID_Table.mat 생성 및 저장 완료!');
end