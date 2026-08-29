function play_simulation_graphics(log_data, obstacles, cfg)
    % PLAY_SIMULATION_GRAPHICS 계산된 데이터를 바탕으로 애니메이션 재생
    
    %% 드론 형상 정의
    drone1_body = [ 0.265,      0,     0, 1; ...
                        0, -0.265,     0, 1; ...
                   -0.265,      0,     0, 1; ...
                        0,  0.265,     0, 1; ...
                        0,      0,     0, 1; ...
                        0,      0, -0.15, 1]';

    %% Figure 1 (3D 뷰) 초기화
    fig1 = figure('Name','Drone 3D Simulation','pos',[0 100 800 700], "Theme", "light");
    view(3); axis equal; grid on; hold on;
    fig1.CurrentAxes.ZDir = 'Reverse';
    fig1.CurrentAxes.YDir = 'Reverse';
    xlim([-6 6]); ylim([-6 6]); zlim([-8 0]);
    xlabel('X[m]'); ylabel('Y[m]'); zlabel('Height[m]');

    % 장애물 그리기
    for k = 1:length(obstacles)
        curr_obs = obstacles{k};
        if isempty(curr_obs), continue; end

        obs_x_min = curr_obs(1, 1); obs_x_max = curr_obs(1, 2);
        obs_y_min = curr_obs(2, 1); obs_y_max = curr_obs(2, 2);
        obs_z_min = curr_obs(3, 1); obs_z_max = curr_obs(3, 2);

        obs_verts = [
            obs_x_min obs_y_min obs_z_min; obs_x_max obs_y_min obs_z_min;
            obs_x_max obs_y_max obs_z_min; obs_x_min obs_y_max obs_z_min;
            obs_x_min obs_y_min obs_z_max; obs_x_max obs_y_min obs_z_max;
            obs_x_max obs_y_max obs_z_max; obs_x_min obs_y_max obs_z_max];
        obs_faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

        patch('Vertices', obs_verts, 'Faces', obs_faces, ...
            'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.5, 'EdgeColor', 'g');
    end

    % 드론 핸들 초기화
    fig1_ARM13 = plot3(gca, 0,0,0, '-ro', 'MarkerSize', 5);
    fig1_ARM24 = plot3(gca, 0,0,0, '-bo', 'MarkerSize', 5);
    fig1_payload = plot3(gca, 0,0,0, '-y', 'Linewidth', 3);
    fig1_shadow = plot3(gca, 0,0,0, 'xk', 'Linewidth', 3);
    h_traj = animatedline('Color', 'r', 'LineStyle', ':', 'LineWidth', 1.5);

    %% Figure 2 (실시간 그래프) 초기화
    fig2 = figure('Name','State Tracking','pos',[800 250 800 550], "Theme", "light");
    subplot(2,3,1); h_phi = animatedline('Color', 'b', 'LineWidth', 1.5); title('phi[deg]'); grid on;
    subplot(2,3,2); h_theta = animatedline('Color', 'r', 'LineWidth', 1.5); title('theta[deg]'); grid on;
    subplot(2,3,3); h_psi = animatedline('Color', 'g', 'LineWidth', 1.5); title('psi[deg]'); grid on;
    subplot(2,3,4); h_x = animatedline('Color', 'b', 'LineWidth', 1.5); title('x[m]'); grid on;
    subplot(2,3,5); h_y = animatedline('Color', 'r', 'LineWidth', 1.5); title('y[m]'); grid on;
    subplot(2,3,6); h_z = animatedline('Color', 'g', 'LineWidth', 1.5); title('z[m]'); grid on;

    %% 애니메이션 루프 (Playback)
    disp('0.1 Animation Delay 0.1 sec');
    pause(0.1);
    
    R2D = 180/pi;
    valid_len = length(log_data.t_hist);

    for i = 1:2:valid_len % 렌더링 속도 조절을 위해 2스텝씩 건너뛰어도 돼 (필요시 1로 변경)
        t_curr = log_data.t_hist(i);
        state_curr = log_data.state_hist(:, i);
        
        x_curr = state_curr(1); y_curr = state_curr(2); z_curr = state_curr(3);
        phi = state_curr(7); theta = state_curr(8); psi = state_curr(9);

        % 드론 회전 행렬 계산 (RPY2Rot 함수가 따로 있다고 가정)
        wHb = [RPY2Rot([phi, theta, psi])' [x_curr; y_curr; z_curr]; 0 0 0 1];
        drone1_world = wHb * drone1_body; 
        drone1_atti = drone1_world(1:3, :); 

        % 3D 플롯 업데이트
        addpoints(h_traj, x_curr, y_curr, z_curr);
        set(fig1_ARM13, 'xData', drone1_atti(1,[1 3]), 'yData', drone1_atti(2,[1 3]), 'zData', drone1_atti(3,[1 3]));
        set(fig1_ARM24, 'xData', drone1_atti(1,[2 4]), 'yData', drone1_atti(2,[2 4]), 'zData', drone1_atti(3,[2 4]));
        set(fig1_payload, 'xData', drone1_atti(1,[5 6]), 'yData', drone1_atti(2,[5 6]), 'zData', drone1_atti(3,[5 6]));
        set(fig1_shadow, 'xData', x_curr, 'yData', y_curr, 'zData', 0);

        % 상태 그래프 업데이트
        addpoints(h_phi, t_curr, phi*R2D);
        addpoints(h_theta, t_curr, theta*R2D);
        addpoints(h_psi, t_curr, psi*R2D);
        addpoints(h_x, t_curr, x_curr);
        addpoints(h_y, t_curr, y_curr);
        addpoints(h_z, t_curr, z_curr);

        drawnow limitrate;
    end
end