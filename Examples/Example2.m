clear; clc; close all;

dt = 0.01;
t = 0:dt:10;
N = length(t);

% input signal (Reference r)
r_value = 0.5;
r = r_value * ones(1, N); % step input

% Initialize variables
x_ex2 = zeros(1, N);  xc_ex2 = zeros(1, N);  y_ex2 = zeros(1, N);  u_ex2 = zeros(1, N);

for k = 1:N-1
    y_ex2(k) = 1/(1+exp(-x_ex2(k)));
    sigma_2 = y_ex2(k);
    if abs(sigma_2) >= 0.99, sigma_2 = sign(sigma_2) * 0.99; end
    
    kp_gs = 1 / (3 * (1 - sigma_2^2));
    ki_gs = 1 / (1 - sigma_2^2);
    u_ex2(k) = xc_ex2(k) + kp_gs * (r(k) - y_ex2(k));
    
    x_ex2(k+1) = x_ex2(k) + (-x_ex2(k) + u_ex2(k)) * dt;
    xc_ex2(k+1) = xc_ex2(k) + (ki_gs * (r(k) - y_ex2(k))) * dt;
end

% 마지막 출력 계산
y_ex2(N) = 1/(1+exp(-x_ex2(N)));