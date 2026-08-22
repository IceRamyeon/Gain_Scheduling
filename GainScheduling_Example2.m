% Example 2 & 8 Gain Scheduling Simulation
clear; clc; close all;

% Simulation Parameters
dt = 0.01;
t = 0:dt:10;
N = length(t);

% Input signal (Reference r)
% 논문에 따르면 r이 0.8보다 커지면 불안정(instability)해짐
r_value = 0.5; % 0.8 이상의 값(예: 0.9)을 넣어서 불안정해지는지 테스트 가능
r = r_value * ones(1, N); 

% 변수 초기화
x = zeros(1, N);     % Plant state
xc = zeros(1, N);    % Controller state
y = zeros(1, N);     % Output
u = zeros(1, N);     % Control input

% 시뮬레이션 루프 (Euler Integration)
for k = 1:N-1
    %% 1. System Output
    y(k) = tanh(x(k));
    
    %% 2. 매개변수 sigma
    % Gain Scheduling을 위한 스케줄링 변수 (현재 출력 y를 사용)
    sigma = y(k); 
    
    %% 3. Gain Scheduling (Example 8 참고)
    % 제어기 입력 방정식: u = (3*xc + r - y) / (3*(1 - y^2))
    % 분모가 0이 되는 것을 방지하기 위해 sigma^2 가 1에 너무 가깝지 않도록 주의
    if abs(sigma) >= 0.99
        sigma = sign(sigma) * 0.99;
    end
    
    u(k) = (3 * xc(k) + r(k) - y(k)) / (3 * (1 - sigma^2));
    
    % 상태 변화율 계산 (플랜트 및 제어기)
    dot_x = -x(k) + u(k);
    dot_xc = r(k) - y(k);
    
    % 상태 업데이트
    x(k+1) = x(k) + dot_x * dt;
    xc(k+1) = xc(k) + dot_xc * dt;
end

% 마지막 출력 계산
y(N) = tanh(x(N));

% 결과 그래프 출력
figure("Theme", "light");
plot(t, r, '--k', 'LineWidth', 1.5); hold on;
plot(t, y, 'b', 'LineWidth', 1.5);
legend('Reference (r)', 'Output (y)');
xlabel('Time (s)');
ylabel('Response');
title('Gain Scheduled PI Control (Example 8)');
grid on;