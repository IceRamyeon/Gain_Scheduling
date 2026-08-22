close all;
clc; clear;

addpath('./lib');
%% DEFINE
R2D = 180/pi;
D2R = pi/180;

%% INIT. PARAMS.
drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz'},...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468});

drone1_initStates = [ 0.0, -4.5, -0.0, ...                                      % x, y, z
    0, 0, 0, ...                                                                % dx, dy, dz
    0, 0, 0, ...                                                                % phi, theta, psi
    0, 0, 0]';                                                                  % p, q, r

drone1_initInputs = [0, ...                                                     % ThrottleCMD
    0, 0, 0]';                                                                  % R, P, Y CMD

drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
			
drone1_PositionCtrl_gains = containers.Map(...
	{'P_x','I_x','D_x',...
	'P_y','I_y','D_y',...
	'P_z','I_z','D_z'},...
    {1.2, 0.001, 1.8,...
	1.2, 0.001, 1.8,...
	2.0, 0.003, 0.9});

drone1_AttitudeCtrl_gains = containers.Map(...
	{'P_phi','I_phi','D_phi',...
	'P_theta','I_theta','D_theta',...
	'P_psi','I_psi','D_psi',...
	'P_zdot','I_zdot','D_zdot'},...
    {7.0, 0.006, 1.0,...
	7.0, 0.006, 1.0,...
	7.0, 0.006, 1.0,...
	8.0, 0.01, 1.0});

simulationTime = 30;                                                          % [sec]

%% Command Signals
waypoints = [ 0.0,  0.5,  0.0, -0.5,  0.0,  0.0,  1.0,  0.0,  0.0;  % desired x1, x2, x3, ...
             -3.8, -2.0, -2.8, -2.8,  1.8,  1.8,  1.8,  4.6,  5.0;  % desired y1, y2, y3, ...
             -6.0, -5.0,  0.0, -5.0, -5.0, -0.0, -5.0, -5.0, -0.0]; % desired z1, z2, z3, ...
target_yaw = 0.0;
current_time = 0;

% Obstacles
% info [x_min x_max; y_min y_max; z_min z_max]
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
    % obstacles = {0; 0; 0};

    % limbo2 = [ x, x;
    %            y, y;
    %            z, z];

%% BIRTH OF A DRONE
drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, ...
    drone1_PositionCtrl_gains, drone1_AttitudeCtrl_gains, simulationTime, ...
    waypoints, obstacles);


%% Init. 3D Fig.
fig1 = figure('pos',[0 100 800 700]);
h = gca;
view(3);
fig1.CurrentAxes.ZDir = 'Reverse';
fig1.CurrentAxes.YDir = 'Reverse';

axis equal;
grid on;
xlim([-6 6]);
ylim([-6 6]);
zlim([-8 0]);
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Height[m]');

hold(gca, 'on');

%% Obstacles
    % 1. 장애물 목록이 비어있는지 확인 (Safety Check)
    if ~exist('obstacles', 'var') || isempty(obstacles)
        disp('장애물이 없습니다.');
    else
        % 2. 장애물 개수만큼 반복 (Loop)
        % obstacles{1}, obstacles{2}... 하나씩 꺼내서 그린다.
        for k = 1:length(obstacles)
            
            curr_obs = obstacles{k}; % k번째 장애물 꺼내기
            
            % 혹시 빈 데이터가 들어있으면 건너뛰기
            if isempty(curr_obs)
                continue; 
            end

            % 좌표 추출
            obs_x_min = curr_obs(1, 1); obs_x_max = curr_obs(1, 2);
            obs_y_min = curr_obs(2, 1); obs_y_max = curr_obs(2, 2);
            obs_z_min = curr_obs(3, 1); obs_z_max = curr_obs(3, 2);

            % 꼭짓점 8개 정의 (직육면체)
            obs_verts = [
                obs_x_min obs_y_min obs_z_min; % 1. 앞-왼-아래
                obs_x_max obs_y_min obs_z_min; % 2. 앞-오-아래
                obs_x_max obs_y_max obs_z_min; % 3. 뒤-오-아래
                obs_x_min obs_y_max obs_z_min; % 4. 뒤-왼-아래
                obs_x_min obs_y_min obs_z_max; % 5. 앞-왼-위
                obs_x_max obs_y_min obs_z_max; % 6. 앞-오-위
                obs_x_max obs_y_max obs_z_max; % 7. 뒤-오-위
                obs_x_min obs_y_max obs_z_max  % 8. 뒤-왼-위
            ];

            % 면 정의
            obs_faces = [
                1 2 6 5;
                2 3 7 6;
                3 4 8 7;
                4 1 5 8;
                1 2 3 4;
                5 6 7 8
            ];

            % 그리기 (Patch)
            patch('Vertices', obs_verts, 'Faces', obs_faces, ...
                'FaceColor', [0.8 0.2 0.2], ... 
                'FaceAlpha', 0.5, ...           
                'EdgeColor', 'g');
        end
    end


drone1_state = drone1.GetState();
wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];

% [Rot(also contains shear, reflection, local sacling), displacement; perspective ,global scaling]
drone1_world = wHb * drone1_body; % [4x4][4x6]
drone1_atti = drone1_world(1:3, :); 
    
fig1_ARM13 = plot3(gca, drone1_atti(1,[1 3]), drone1_atti(2,[1 3]), drone1_atti(3,[1 3]), ...
        '-ro', 'MarkerSize', 5);
fig1_ARM24 = plot3(gca, drone1_atti(1,[2 4]), drone1_atti(2,[2 4]), drone1_atti(3,[2 4]), ...
        '-bo', 'MarkerSize', 5);
fig1_payload = plot3(gca, drone1_atti(1,[5 6]), drone1_atti(2,[5 6]), drone1_atti(3,[5 6]), ...
        '-y', 'Linewidth', 3);
fig1_shadow = plot3(gca,0,0,0,'xk','Linewidth',3);

h_traj = animatedline('Color', 'r', 'LineStyle', ':', 'LineWidth', 1.5, 'Parent', gca);

hold(gca, 'off');

%% Init. 3D Fig.
fig2 = figure('pos',[800 250 800 550]);

subplot(2,3,1); h_phi = animatedline('Color', 'b', 'LineWidth', 1.5); title('phi[deg]'); grid on;
subplot(2,3,2); h_theta = animatedline('Color', 'r', 'LineWidth', 1.5); title('theta[deg]'); grid on;
subplot(2,3,3); h_psi = animatedline('Color', 'g', 'LineWidth', 1.5); title('psi[deg]'); grid on;
subplot(2,3,4); h_x = animatedline('Color', 'b', 'LineWidth', 1.5); title('x[m]'); grid on;
subplot(2,3,5); h_y = animatedline('Color', 'r', 'LineWidth', 1.5); title('y[m]'); grid on;
subplot(2,3,6); h_z = animatedline('Color', 'g', 'LineWidth', 1.5); title('z[m]'); grid on;

figure(fig1);

%% ------------------------------------------------------------------------ 마지막 3D PLOT

% 0. 데이터 저장
num_steps = simulationTime/0.01;
save_time  = zeros(1, num_steps);      % 시간
save_state = zeros(12, num_steps);     % 실제 상태 [x,y,z, ... ]

% desired pos과 제어 입력 inputs 저장 공간
save_pos_des = zeros(3, num_steps);    % 목표 위치 [x_d; y_d; z_d] (Trajectory Output)
save_att_des = zeros(3, num_steps);    % 목표 자세 [phi_d; theta_d; psi_d] (PositionCtrl Output)
save_u       = zeros(4, num_steps);    % 제어 입력 [Thrust; Mx; My; Mz] (AttitudeCtrl Output)

%% ------------------------------------------------------------------------
% [추가] 시뮬레이션 Start Delay
% ------------------------------------------------------------------------
if exist('fig1', 'var'), figure(fig1); end
if exist('fig2', 'var'), figure(fig2); end
if exist('fig3', 'var'), figure(fig3); end 

drawnow; % 밀린 그림 다 그리고 창 띄우기
fprintf('1초 뒤에 시작....\n');
pause(4); % 1초 대기 (이 숫자를 늘리면 더 오래 기다림)
% ------------------------------------------------------------------------

%% Simulation Loop
for i = 1:num_steps
    % 1. Get State
    drone1_state = drone1.GetState();

    % Trajectory Control을 위한 t 불러오기
    current_time = drone1.t;
    traj_pos = drone1.trajCtrl.get_position(current_time);
    current_cmd = [traj_pos; target_yaw];

    % 2. Outer Loop Control (Position Control)
    att_cmd = drone1.posCtrl.Update(current_cmd, drone1_state);

    % 3. Inner Loop Control (Attitude Control)
    u_motor = drone1.attCtrl.Update(att_cmd, drone1_state);

    % 4. Update Drone
    drone1.UpdateState(u_motor);

    % 현재 상태 기록
    save_time(i) = current_time;
    save_state(:, i) = drone1_state;

    % 입력에 대한 출력 저장
    save_pos_des(:, i) = traj_pos;      % Trajectory의 desired Position
    save_att_des(:, i) = att_cmd(1:3);  % PositionCtrl의 desired attitude
    save_u(:, i)       = u_motor;       % AttitudeCtrl의 desired Thrust, Moments

    % 충돌 체크 변수 준비
    x_curr = drone1_state(1);
    y_curr = drone1_state(2);
    z_curr = drone1_state(3);
    
    % 1. 드론 3D 궤적 업데이트 (Figure 1)
    addpoints(h_traj, x_curr, y_curr, z_curr);

    % 2. 드론 몸체 움직임 업데이트 (Figure 1)
    % (몸체는 addpoints가 아니라 위치 데이터를 set으로 바꿔줘야 해)
    wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
    drone1_world = wHb * drone1_body; 
    drone1_atti = drone1_world(1:3, :); 

    set(fig1_ARM13, 'xData', drone1_atti(1,[1 3]), 'yData', drone1_atti(2,[1 3]), 'zData', drone1_atti(3,[1 3]));
    set(fig1_ARM24, 'xData', drone1_atti(1,[2 4]), 'yData', drone1_atti(2,[2 4]), 'zData', drone1_atti(3,[2 4]));
    set(fig1_payload, 'xData', drone1_atti(1,[5 6]), 'yData', drone1_atti(2,[5 6]), 'zData', drone1_atti(3,[5 6]));
    set(fig1_shadow, 'xData', drone1_state(1), 'yData', drone1_state(2), 'zData', 0);

    % 3. 상태 그래프 업데이트 (Figure 2)
    addpoints(h_phi,   current_time, drone1_state(7)*R2D);
    addpoints(h_theta, current_time, drone1_state(8)*R2D);
    addpoints(h_psi,   current_time, drone1_state(9)*R2D);
    addpoints(h_x, current_time, x_curr);
    addpoints(h_y, current_time, y_curr);
    addpoints(h_z, current_time, z_curr);

    % 4. 화면 갱신 (drawnow)
    drawnow limitrate; % 너무 빠르면 limitrate 제거하고 drawnow만 쓰거나 pause(0.01) 추가
    
    % =================================================

    % 충돌 체크 로직
    is_hit_obstacle = false;

    if exist('limbo1', 'var')
        if (x_curr >= limbo1(1,1) && x_curr <= limbo1(1,2)) && ...
           (y_curr >= limbo1(2,1) && y_curr <= limbo1(2,2)) && ...
           (z_curr >= limbo1(3,1) && z_curr <= limbo1(3,2))
            is_hit_obstacle = true;
        end
    end

    if exist('limbo2', 'var')
        if (x_curr >= limbo2(1,1) && x_curr <= limbo2(1,2)) && ...
           (y_curr >= limbo2(2,1) && y_curr <= limbo2(2,2)) && ...
           (z_curr >= limbo2(3,1) && z_curr <= limbo2(3,2))
            is_hit_obstacle = true;
        end
    end

    if exist('limbo3', 'var')
        if (x_curr >= limbo3(1,1) && x_curr <= limbo3(1,2)) && ...
           (y_curr >= limbo3(2,1) && y_curr <= limbo3(2,2)) && ...
           (z_curr >= limbo3(3,1) && z_curr <= limbo3(3,2))
            is_hit_obstacle = true;
        end
    end

    is_hit_ground = (z_curr > 0);

    if is_hit_ground || is_hit_obstacle
        crash_time = num2str(save_time(i));
        disp(['드론 충돌. 시간: t=' crash_time]);

        if is_hit_obstacle
            msgbox(['Wall Crash at t=' crash_time], 'Game Over', 'error');
        else
            msgbox(['Ground Crash at t=' crash_time], 'Game Over', 'error');
        end
        break;
    end
end

%% 3D Plot (루프 끝나고 딱 한 번만 그리기)

%% Final Plot

% 1. 데이터 슬라이싱 (충돌 전까지만 유효한 데이터)
valid_len = i; 
t_hist = save_time(1:valid_len);

pos_des_hist = save_pos_des(:, 1:valid_len);
att_des_hist = save_att_des(:, 1:valid_len);
state_hist   = save_state(:, 1:valid_len);
u_hist       = save_u(:, 1:valid_len);

% 2. [그래프 1] 위치 추종 (Position Tracking)
% 변수명을 h_fig_pos로 바꿔서 'unused' 경고를 피하거나 명확하게 함
h_fig_pos = figure('Name', 'Position Tracking', 'Theme', 'light', 'Position', [10 100 600 600]);

subplot(3,1,1);
plot(t_hist, pos_des_hist(1,:), 'r--', 'LineWidth', 1.5); hold on;
plot(t_hist, state_hist(1,:), 'b', 'LineWidth', 1.2);
ylabel('X [m]'); title('Position X: Desired vs Actual'); grid on; legend('Desired','Actual');

subplot(3,1,2);
plot(t_hist, pos_des_hist(2,:), 'r--', 'LineWidth', 1.5); hold on;
plot(t_hist, state_hist(2,:), 'b', 'LineWidth', 1.2);
ylabel('Y [m]'); title('Position Y: Desired vs Actual'); grid on;

subplot(3,1,3);
plot(t_hist, pos_des_hist(3,:), 'r--', 'LineWidth', 1.5); hold on;
plot(t_hist, state_hist(3,:), 'b', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Z [m]'); title('Position Z: Desired vs Actual'); grid on;

% 3. [그래프 2] 자세 추종 (Attitude Tracking)
h_fig_att = figure('Name', 'Attitude Tracking', 'Theme', 'light', 'Position', [610 100 600 600]);

subplot(3,1,1);
plot(t_hist, att_des_hist(1,:)*R2D, 'r--', 'LineWidth', 1.5); hold on;
plot(t_hist, state_hist(7,:)*R2D, 'b', 'LineWidth', 1.2);
ylabel('Roll (\phi) [deg]'); title('Roll: Desired vs Actual'); grid on; legend('Command','Response');

subplot(3,1,2);
plot(t_hist, att_des_hist(2,:)*R2D, 'r--', 'LineWidth', 1.5); hold on;
plot(t_hist, state_hist(8,:)*R2D, 'b', 'LineWidth', 1.2);
ylabel('Pitch (\theta) [deg]'); title('Pitch: Desired vs Actual'); grid on;

subplot(3,1,3);
% Yaw 목표값 (Trajectory에서 왔거나 고정값)
plot(t_hist, ones(size(t_hist))*target_yaw*R2D, 'r--', 'LineWidth', 1.5); hold on; 
plot(t_hist, state_hist(9,:)*R2D, 'b', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Yaw (\psi) [deg]'); title('Yaw: Desired vs Actual'); grid on;

% 4. [그래프 3] 제어 입력 (Control Inputs)
h_fig_in = figure('Name', 'Control Inputs', 'Theme', 'light', 'Position', [1010 100 600 600]);

subplot(4,1,1);
plot(t_hist, u_hist(1,:), 'g', 'LineWidth', 1.2); 
ylabel('Thrust [N]'); title('Total Thrust Input (U1)'); grid on;

subplot(4,1,2);
plot(t_hist, u_hist(2,:), 'r', 'LineWidth', 1.2);
ylabel('Mx [N\cdotm]'); title('Rolling Moment (U2)'); grid on;

subplot(4,1,3);
plot(t_hist, u_hist(3,:), 'g', 'LineWidth', 1.2); 
ylabel('My [N\cdotm]'); title('Pitching Moment (U3)'); grid on;

subplot(4,1,4);
plot(t_hist, u_hist(4,:), 'b', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Mz [N\cdotm]'); title('Yawing Moment (U4)'); grid on;

% 결과 메시지
msgbox('Simulation & Plotting Complete!', 'Success');