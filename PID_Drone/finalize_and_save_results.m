function finalize_and_save_results(log_data, cfg)
    % FINALIZE_AND_SAVE_RESULTS 최종 결과 그래프 출력 및 자동 저장 함수
    
    disp('으헤~ 이제 최종 결과를 정리해서 보여줄게.');
    
    R2D = 180/pi;
    
    % 데이터 언패킹 (코드를 짧고 보기 좋게 쓰기 위해)
    t_hist       = log_data.t_hist;
    state_hist   = log_data.state_hist;
    pos_des_hist = log_data.pos_des_hist;
    att_des_hist = log_data.att_des_hist;
    u_hist       = log_data.u_hist;
    
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
    % Yaw 목표값 (cfg.target_yaw 사용)
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

    %% 4. 자동 저장 로직 (데이터 + 이미지)
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
        
        disp(['으헤~ 데이터랑 그래프 이미지들 전부 저장했어: ', cfg.save_dir]);
    else
        disp('자동 저장은 꺼져 있어서 아무것도 안 남겼어.');
    end
end