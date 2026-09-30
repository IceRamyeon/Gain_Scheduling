%% =========================================================================
%  Neural Network-Based Gain Scheduling PID Control Demo
%  - Plant: \dot{x} = sin(x) + u
%  - Framework:
%    1) 작동점별 선형화 (Local Linearization)
%    2) 선형 모델 기반 PID 게인 사전 튜닝 (Gain Dictionary 구축)
%    3) Neural Network 학습 (스케줄링 변수 x_eq -> [Kp, Ki, Kd] 보간)
%    4) 비선형 시스템에 실시간 NN 게인 스케줄링 제어 적용 시뮬레이션
% =========================================================================
clear; clc; close all;

%% Step 1 & 2: 작동 조건 샘플링 및 PID Gain Dictionary 생성
fprintf('1. 작동 조건별 선형화 및 PID 게인 튜닝 중...\n');

% 스케줄링 변수 (작동점 평형 상태 x_eq: -pi/2 ~ pi/2)
x_eq_samples = linspace(-pi/2, pi/2, 41); % 41개 작동점 샘플링
num_samples  = length(x_eq_samples);

Kp_dict = zeros(1, num_samples);
Ki_dict = zeros(1, num_samples);
Kd_dict = zeros(1, num_samples);

for i = 1:num_samples
    x0 = x_eq_samples(i);
    
    % dot_x = sin(x) + u 에서 x0 주위 섭동 해석:
    % \Delta\dot{x} = a * \Delta x + b * \Delta u
    % a = d(sin(x))/dx |_{x0} = cos(x0), b = 1
    a = cos(x0);
    b = 1;
    sys_linear = tf(b, [1, -a]); % P(s) = 1 / (s - cos(x0))
    
    % PID 튜닝 (원하는 닫힌루프 대역폭 지정: 3 rad/s)
    target_bw = 3.0;
    try
        [C_pid, ~] = pidtune(sys_linear, 'PID', target_bw);
        Kp_dict(i) = C_pid.Kp;
        Ki_dict(i) = C_pid.Ki;
        Kd_dict(i) = C_pid.Kd;
    catch
        % 불안정점 등 pidtune 예외 시 기본값 할당
        Kp_dict(i) = 5.0;
        Ki_dict(i) = 2.0;
        Kd_dict(i) = 0.5;
    end
end

% 학습 데이터 행렬 구성
% 입력 X: [1 x N] (현재 동작 상태/평형점)
% 타깃 T: [3 x N] (튜닝된 Kp, Ki, Kd)
X_train = x_eq_samples;
T_train = [Kp_dict; Ki_dict; Kd_dict];

%% Step 3: Neural Network 학습 (Function Approximation)
fprintf('2. Neural Network 모델 생성 및 학습 중...\n');

% 은닉층 1개(뉴런 10개)로 구성된 피드포워드 신경망
hiddenLayerSize = 10;
net = fitnet(hiddenLayerSize, 'trainlm'); % Levenberg-Marquardt 최적화

% 연속 미분 가능한 tansig 활성화 함수 지정 (부드러운 게인 전이 목적)
net.layers{1}.transferFcn = 'tansig';
net.layers{2}.transferFcn = 'purelin';

% 데이터 분할 및 학습 제어
net.divideParam.trainRatio = 0.8;
net.divideParam.valRatio   = 0.1;
net.divideParam.testRatio  = 0.1;
net.trainParam.showWindow  = false; % GUI 팝업 생략

[net, ~] = train(net, X_train, T_train);

%% Step 4: 비선형 폐루프 시뮬레이션
fprintf('3. 비선형 시스템에 실시간 NN 게인 스케줄링 적용 시뮬레이션 중...\n');

dt = 0.005;          % 제어 주기 (5 ms)
t_final = 20.0;
t_span = 0:dt:t_final;
N_steps = length(t_span);

% 목표 궤적 설정 (여러 비선형 구간을 통과하는 계단형 신호)
r_traj = zeros(1, N_steps);
for k = 1:N_steps
    t = t_span(k);
    if t < 5
        r_traj(k) = 0.4;
    elseif t < 10
        r_traj(k) = 1.2;
    elseif t < 15
        r_traj(k) = -0.8;
    else
        r_traj(k) = 0.0;
    end
end

% 시뮬레이션 변수 초기화
x_nn  = 0.0; % NN 스케줄링 적용 시스템 상태
x_fix = 0.0; % 고정 PID 게인 적용 시스템 상태 (원점 x=0 기준 튜닝 게인)

% 고정 게인 기준 (x=0, sys = 1/(s-1) 기준 PID)
C_fix = pidtune(tf(1, [1, -1]), 'PID', 3.0);
Kp_fixed = C_fix.Kp;
Ki_fixed = C_fix.Ki;
Kd_fixed = C_fix.Kd;

% 로깅용 배열
x_nn_history   = zeros(1, N_steps);
x_fix_history  = zeros(1, N_steps);
u_nn_history   = zeros(1, N_steps);
gains_history  = zeros(3, N_steps);

% PID 적분항 및 이전 오차 초기화
int_e_nn  = 0.0; prev_e_nn  = 0.0;
int_e_fix = 0.0; prev_e_fix = 0.0;

for k = 1:N_steps
    r = r_traj(k);
    
    % ----------------------------------------------------
    % [Case A] NN 기반 Gain Scheduled PID 제어
    % ----------------------------------------------------
    e_nn = r - x_nn;
    int_e_nn = int_e_nn + e_nn * dt;
    der_e_nn = (e_nn - prev_e_nn) / dt;
    prev_e_nn = e_nn;
    
    % Step 5: 현재 상태(x)를 스케줄링 변수로 NN에 전달 -> 게인 추출
    current_gains = net(x_nn); 
    Kp = current_gains(1);
    Ki = current_gains(2);
    Kd = current_gains(3);
    
    % PID 제어 입력 계산
    u_nn = Kp * e_nn + Ki * int_e_nn + Kd * der_e_nn;
    
    % 비선형 플랜트 적분 (Euler-Forward 또는 RK4)
    x_nn_dot = sin(x_nn) + u_nn;
    x_nn = x_nn + x_nn_dot * dt;
    
    % ----------------------------------------------------
    % [Case B] 고정(Fixed) 게인 PID 제어 (비교군)
    % ----------------------------------------------------
    e_fix = r - x_fix;
    int_e_fix = int_e_fix + e_fix * dt;
    der_e_fix = (e_fix - prev_e_fix) / dt;
    prev_e_fix = e_fix;
    
    u_fix = Kp_fixed * e_fix + Ki_fixed * int_e_fix + Kd_fixed * der_e_fix;
    x_fix_dot = sin(x_fix) + u_fix;
    x_fix = x_fix + x_fix_dot * dt;
    
    % 로깅
    x_nn_history(k)  = x_nn;
    x_fix_history(k) = x_fix;
    u_nn_history(k)  = u_nn;
    gains_history(:, k) = [Kp; Ki; Kd];
end

%% Step 5: 결과 시각화
fprintf('4. 결과 플롯 생성 중...\n');
figure('Color', [1 1 1], 'Position', [150 150 900 650], 'theme', 'light');

% 1) 트래킹 성능 비교
subplot(3, 1, 1);
plot(t_span, r_traj, 'k--', 'LineWidth', 1.5); hold on;
plot(t_span, x_nn_history, 'b-', 'LineWidth', 1.3);
plot(t_span, x_fix_history, 'r-.', 'LineWidth', 1.1);
grid on;
ylabel('State x(t)');
legend('Reference r(t)', 'NN-Scheduled PID', 'Fixed PID', 'Location', 'SouthEast');
title('Tracking Performance: NN-Scheduled PID vs Fixed Gain PID');

% 2) 실시간 게인 스케줄링 거동 (Kp, Ki, Kd)
subplot(3, 1, 2);
plot(t_span, gains_history(1, :), 'LineWidth', 1.2); hold on;
plot(t_span, gains_history(2, :), 'LineWidth', 1.2);
plot(t_span, gains_history(3, :), 'LineWidth', 1.2);
grid on;
ylabel('Gain Values');
legend('K_p', 'K_i', 'K_d');
title('Real-time PID Gains Output from Neural Network');

% 3) 제어 입력 u(t)
subplot(3, 1, 3);
plot(t_span, u_nn_history, 'b', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Control Input u(t)');
title('Control Effort u(t)');