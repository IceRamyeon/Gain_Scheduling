function finalize_and_save_results(log_data, cfg)
    % FINALIZE_AND_SAVE_RESULTS 최종 결과 그래프 출력 및 자동 저장 함수
    
    disp('[최종 결과].');
    
    R2D = 180/pi;
    
    % 데이터 언패킹 (코드를 짧고 보기 좋게 쓰기 위해)
    t_hist       = log_data.t_hist;
    state_hist   = log_data.state_hist;
    pos_des_hist = log_data.pos_des_hist;
    att_des_hist = log_data.att_des_hist;
    u_hist       = log_data.u_hist;

    %% 0. 평가 지표 계산
    % 3차원 유클리디안 거리 오차 계산 블록
    dist_error = sqrt(sum((state_hist(1:3, :) - pos_des_hist(1:3, :)).^2, 1));
    
    % 평가 지표 계산
    val_rmse = sqrt(mean(dist_error.^2));
    val_mae  = mean(dist_error);
    val_max  = max(dist_error);
    
    disp('----------------------------------');
    fprintf('RMSE: %.4f m\n', val_rmse);
    fprintf('MAE:  %.4f m\n', val_mae);
    fprintf('Max:  %.4f m\n', val_max);
    disp('----------------------------------');
    
    %% 1. [그래프 1] 위치 추종 (Position Tracking)
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

    %% 2. [그래프 2] 자세 추종 (Attitude Tracking)
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
    % Yaw 목표값
    plot(t_hist, att_des_hist(3,:)*R2D, 'r--', 'LineWidth', 1.5); hold on; 
    plot(t_hist, state_hist(9,:)*R2D, 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Yaw (\psi) [deg]'); title('Yaw: Desired vs Actual'); grid on;

    %% 3. [그래프 3] 제어 입력 (Control Inputs)
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

    %% 4. [그래프 4] 총 속력 추종 (Total Speed Tracking)
    % 목표 속력은 cfg.speed 사용
    if isfield(cfg, 'speed')
        des_speed = cfg.speed * ones(1, length(t_hist));
    else
        des_speed = zeros(1, length(t_hist)); 
    end
    
    % 실제 속력은 상태 변수 4~6번째 행(Vx, Vy, Vz)의 유클리디안 노름
    act_speed = sqrt(sum(state_hist(4:6, :).^2, 1));

    h_fig_spd = figure('Name', 'Total Speed Tracking', 'Theme', 'light', 'Position', [1410 100 600 300]);
    plot(t_hist, des_speed, 'r--', 'LineWidth', 1.5); hold on;
    plot(t_hist, act_speed, 'b', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Speed [m/s]'); 
    title('Total Speed: Desired vs Actual'); grid on; legend('Desired', 'Actual');

    %% 5. 자동 저장 로직 (데이터 + 이미지)
    if isfield(cfg, 'auto_save') && cfg.auto_save
        if ~exist(cfg.save_dir, 'dir')
            mkdir(cfg.save_dir);
        end
        
        currentTimeString = datestr(now, 'yyyymmdd_HHMMSS');
        baseFileName = fullfile(cfg.save_dir, sprintf('DroneSimResult_%s', currentTimeString));
        
        % 1) 데이터 저장 (.mat)
        matFileName = [baseFileName, '.mat'];
        save(matFileName, 'log_data', 'cfg');
        
        % 2) 이미지 저장 (.png) - Resolution 300으로 선명하게
        exportgraphics(h_fig_pos, [baseFileName, '_Position.png'], 'Resolution', 300);
        exportgraphics(h_fig_att, [baseFileName, '_Attitude.png'], 'Resolution', 300);
        exportgraphics(h_fig_in, [baseFileName, '_ControlInput.png'], 'Resolution', 300);
        exportgraphics(h_fig_spd, [baseFileName, '_Speed.png'], 'Resolution', 300); % 속력 그래프 저장
        
        disp(['으헤~ 데이터랑 그래프 이미지들 전부 저장했어: ', cfg.save_dir]);
    else
        disp('자동 저장은 꺼져 있어서 아무것도 안 남겼어.');
    end
end