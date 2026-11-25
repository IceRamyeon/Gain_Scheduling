function px4_smooth_transit(link, p_from, p_goal, px)
%PX4_SMOOTH_TRANSIT  PX4 오프보드 위치제어로 p_from -> p_goal 을 부드러운 속도 프로파일로 이동 (호버 -> 이동 -> 정지)
%   px4_smooth_transit(link, p_from, p_goal, px)
%   - 속도 프로파일: 가속·감속을 사인 램프로 (v = v_pk (1 - cos(pi t / T_a)) / 2) -> 가속도가 0 에서 시작해 0 으로 끝남
%       최대 가속도 a_max = px.return_acc, 최대 속도 v_max = px.return_speed
%       거리가 짧으면 v_pk = sqrt(2 a_max D / pi) 로 낮춰 등속 구간 없이 가속 -> 감속
%   - 위치·속도·가속도 셋포인트를 같이 보냄 (진행 방향을 보면서)
%   - PX4 MPC_XY_VEL_MAX 가 v_max 보다 작으면 이동하는 동안만 올리고 끝나면 원래 값으로 되돌림
%   - 고도는 p_from(3) -> p_goal(3) 를 같은 비율로 보간 (보통 같은 고도)
%   도착 후 정지 확인(hold)은 호출한 쪽에서 한다.

Ts = 1 / px.rate_hz;
if isfield(px, 'return_speed') && ~isempty(px.return_speed), v_max = px.return_speed; else, v_max = 10; end
if isfield(px, 'return_acc')   && ~isempty(px.return_acc),   a_max = px.return_acc;   else, a_max = 2.0; end

p_from = p_from(:); p_goal = p_goal(:);
dp = p_goal - p_from; D = norm(dp);
if D < 0.5, return; end
u = dp / D;

% 속도 프로파일
v_pk = min(v_max, sqrt(2 * a_max * D / pi));
T_a  = pi * v_pk / (2 * a_max);                 % 사인 램프 시간 (최대 가속도 = a_max)
d_a  = v_pk * T_a / 2;                          % 램프 구간 거리
T_c  = max(0, (D - 2 * d_a) / v_pk);            % 등속 시간
T    = 2 * T_a + T_c;
fprintf('부드러운 이동: %.0f m, 최대 %.1f m/s, 가속 %.1f s + 등속 %.1f s + 감속 %.1f s = %.1f s\n', ...
    D, v_pk, T_a, T_c, T_a, T);

% PX4 수평 속도 제한
vmax_old = NaN;
try
    vmax_old = link.readParam('MPC_XY_VEL_MAX', 3);
    if ~isnan(vmax_old) && vmax_old < v_pk + 1
        link.setParam('MPC_XY_VEL_MAX', v_pk + 2);
    else
        vmax_old = NaN;                         % 바꾸지 않았으면 되돌릴 필요 없음
    end
catch err
    warning('MPC_XY_VEL_MAX 설정 실패 (%s). PX4 속도 제한에 걸려 느리게 이동할 수 있습니다.', err.message);
    vmax_old = NaN;
end
restoreObj = onCleanup(@() restore_vmax(link, vmax_old));

% 진행 방향 yaw
yaw_m = atan2(u(2), u(1));
if hypot(u(1), u(2)) < 0.3
    [xs, ~, okk] = link.getState();
    if okk, yaw_m = xs(9); else, yaw_m = 0; end
end
% 제자리에서 진행 방향으로 회전 (최대 10 s)
tr = tic;
while toc(tr) < 10
    link.sendPosition(p_from, yaw_m);
    [xs, ~, okk] = link.getState();
    if okk && abs(atan2(sin(xs(9) - yaw_m), cos(xs(9) - yaw_m))) < deg2rad(5), break; end
    pause(Ts);
end

% 이동
tr = tic;
while true
    t = toc(tr);
    if t >= T, break; end
    [s, v, a] = profile(t);
    link.sendPosition(p_from + s * u, yaw_m, v * u, a * u);
    pause(Ts);
end
link.sendPosition(p_goal, yaw_m, [0; 0; 0], [0; 0; 0]);

    function [s, v, a] = profile(t)
        if t < T_a                                  % 가속 램프
            w = pi * t / T_a;
            s = v_pk / 2 * (t - T_a / pi * sin(w));
            v = v_pk / 2 * (1 - cos(w));
            a = v_pk * pi / (2 * T_a) * sin(w);
        elseif t < T_a + T_c                        % 등속
            s = d_a + v_pk * (t - T_a);
            v = v_pk; a = 0;
        else                                        % 감속 램프 (가속의 시간 반전)
            tb = T - t; w = pi * tb / T_a;
            s = D - v_pk / 2 * (tb - T_a / pi * sin(w));
            v = v_pk / 2 * (1 - cos(w));
            a = -v_pk * pi / (2 * T_a) * sin(w);
        end
    end
end

function restore_vmax(link, vmax_old)
if ~isnan(vmax_old)
    try
        link.setParam('MPC_XY_VEL_MAX', vmax_old);
    catch
        warning('MPC_XY_VEL_MAX 를 원래 값(%.1f)으로 되돌리지 못했습니다.', vmax_old);
    end
end
end
