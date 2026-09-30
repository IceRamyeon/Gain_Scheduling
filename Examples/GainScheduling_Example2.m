% Example 8 vs Example 11 vs Nominal PI Simulation (Metrics 추가)
clear; clc; close all;

% Simulation Parameters
dt = 0.01;
t = 0:dt:10;
N = length(t);

% Input signal (Reference r)
r_value = 0.5; 
r = r_value * ones(1, N); 

% 변수 초기화
x_ex8 = zeros(1, N);  xc_ex8 = zeros(1, N);  y_ex8 = zeros(1, N);  u_ex8 = zeros(1, N);     
x_ex11 = zeros(1, N); xc_ex11 = zeros(1, N); y_ex11 = zeros(1, N); u_ex11 = zeros(1, N);     
x_nom = zeros(1, N);  xc_nom = zeros(1, N);  y_nom = zeros(1, N);  u_nom = zeros(1, N);

kp_nom = 1/3; ki_nom = 1;

% 시뮬레이션 루프
for k = 1:N-1
    %% 1. Example 8 (Before Improvement)
    y_ex8(k) = tanh(x_ex8(k));
    sigma_8 = y_ex8(k); 
    if abs(sigma_8) >= 0.99, sigma_8 = sign(sigma_8) * 0.99; end
    u_ex8(k) = (3 * xc_ex8(k) + r(k) - y_ex8(k)) / (3 * (1 - sigma_8^2));
    
    x_ex8(k+1) = x_ex8(k) + (-x_ex8(k) + u_ex8(k)) * dt;
    xc_ex8(k+1) = xc_ex8(k) + (r(k) - y_ex8(k)) * dt;
    
    %% 2. Example 11 (After Improvement)
    y_ex11(k) = tanh(x_ex11(k));
    sigma_11 = y_ex11(k); 
    if abs(sigma_11) >= 0.99, sigma_11 = sign(sigma_11) * 0.99; end
    
    kp_gs = 1 / (3 * (1 - sigma_11^2));
    ki_gs = 1 / (1 - sigma_11^2);
    u_ex11(k) = xc_ex11(k) + kp_gs * (r(k) - y_ex11(k));
    
    x_ex11(k+1) = x_ex11(k) + (-x_ex11(k) + u_ex11(k)) * dt;
    xc_ex11(k+1) = xc_ex11(k) + (ki_gs * (r(k) - y_ex11(k))) * dt;
    
    %% 3. Nominal PI Control
    y_nom(k) = tanh(x_nom(k));
    u_nom(k) = kp_nom * (r(k) - y_nom(k)) + ki_nom * xc_nom(k);
    
    x_nom(k+1) = x_nom(k) + (-x_nom(k) + u_nom(k)) * dt;
    xc_nom(k+1) = xc_nom(k) + (r(k) - y_nom(k)) * dt;
end

% 마지막 출력 계산
y_ex8(N) = tanh(x_ex8(N));
y_ex11(N) = tanh(x_ex11(N));
y_nom(N) = tanh(x_nom(N));

% 응답 특성(Metrics) 계산 함수 정의
calc_metrics = @(y) get_metrics(y, t, r_value);

[os_8, ts_8, tr_8, tp_8, ymax_8] = calc_metrics(y_ex8);
[os_11, ts_11, tr_11, tp_11, ymax_11] = calc_metrics(y_ex11);
[os_nom, ts_nom, tr_nom, tp_nom, ymax_nom] = calc_metrics(y_nom);

% 결과 그래프 출력
figure("Theme", "light", "Position", [100, 100, 800, 900]);

% 1. Example 8 (Improve 이전)
subplot(3, 1, 1);
plot(t, r, '--k', 'LineWidth', 1.5); hold on;
plot(t, y_ex8, 'b', 'LineWidth', 1.5);
plot(tp_8, ymax_8, 'v', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'b');
text(tp_8, ymax_8, sprintf('  OS: %.1f%%', os_8), 'VerticalAlignment', 'bottom');
plot(ts_8, r_value*0.98, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y');
text(ts_8, r_value*0.98, sprintf('  Ts: %.1fs', ts_8), 'VerticalAlignment', 'top');
legend('Reference', 'Output', 'Peak', 'Settled (2%)', 'Location', 'southeast');
xlabel('Time (s)'); ylabel('Response');
title('1. Before Improvement: Gain Scheduled PI (Example 8)');
grid on;

% 2. Example 11 (Improve 이후)
subplot(3, 1, 2);
plot(t, r, '--k', 'LineWidth', 1.5); hold on;
plot(t, y_ex11, 'g', 'LineWidth', 1.5);
plot(tp_11, ymax_11, 'v', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');
text(tp_11, ymax_11, sprintf('  OS: %.1f%%', os_11), 'VerticalAlignment', 'bottom');
plot(ts_11, r_value*0.98, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y');
text(ts_11, r_value*0.98, sprintf('  Ts: %.1fs', ts_11), 'VerticalAlignment', 'top');
legend('Reference', 'Output', 'Peak', 'Settled (2%)', 'Location', 'southeast');
xlabel('Time (s)'); ylabel('Response');
title('2. After Improvement: Gain Scheduled PI (Example 11)');
grid on;

% 3. Nominal PI
subplot(3, 1, 3);
plot(t, r, '--k', 'LineWidth', 1.5); hold on;
plot(t, y_nom, 'r', 'LineWidth', 1.5);
plot(tp_nom, ymax_nom, 'v', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');
text(tp_nom, ymax_nom, sprintf('  OS: %.1f%%', os_nom), 'VerticalAlignment', 'bottom');
plot(ts_nom, r_value*0.98, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y');
text(ts_nom, r_value*0.98, sprintf('  Ts: %.1fs', ts_nom), 'VerticalAlignment', 'top');
legend('Reference', 'Output', 'Peak', 'Settled (2%)', 'Location', 'southeast');
xlabel('Time (s)'); ylabel('Response');
title('3. Nominal PI Control (\sigma = 0 Fixed Gain)');
grid on;

% ====================================================
% 콘솔 출력 (fprintf)
% ====================================================
fprintf('======================================\n');
fprintf('== 1. Overshoot ==\n');
fprintf('  Ex.8   : %.1f %%  Ex.11  : %.1f %%  No     : %.1f %%\n', os_8, os_11, os_nom);
fprintf('== 2. Settling Time (2%% criterion) ==\n');
fprintf('  Ex.8   : %.1f s  Ex.11  : %.1f s  No     : %.1f s\n', ts_8, ts_11, ts_nom);
fprintf('== 3. Rising Time (10%% -> 90%%) ==\n');
fprintf('  Ex.8   : %.2f s  Ex.11  : %.2f s  No     : %.2f s\n', tr_8, tr_11, tr_nom);
fprintf('== 4. Peak Time ==\n');
fprintf('  Ex.8   : %.2f s  Ex.11  : %.2f s  No     : %.2f s\n', tp_8, tp_11, tp_nom);
fprintf('======================================\n');

% ====================================================
% 보조 함수 (스크립트 맨 끝에 위치해야 함)
% ====================================================
function [OS, Ts, Tr, Tp, ymax] = get_metrics(y, t, r_val)
    [ymax, imax] = max(y);
    OS = max(0, (ymax - r_val)/r_val * 100);
    Tp = t(imax);
    
    % Settling time (2% band)
    err = abs(y - r_val);
    idx = find(err > 0.02 * r_val, 1, 'last');
    if isempty(idx)
        Ts = 0;
    elseif idx < length(t)
        Ts = t(idx+1);
    else
        Ts = NaN;
    end
    
    % Rise time (10% to 90%)
    i10 = find(y >= 0.1 * r_val, 1, 'first');
    i90 = find(y >= 0.9 * r_val, 1, 'first');
    if ~isempty(i10) && ~isempty(i90)
        Tr = t(i90) - t(i10);
    else
        Tr = NaN;
    end
end