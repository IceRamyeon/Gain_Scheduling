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
    plot(t_hist, ones(size(t_hist))*cfg.target_yaw*R2D, 'r--', 'LineWidth', 1.5); hold on; 
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

    %% 4. 자동 저장 로직 (Auto Save)
    if isfield(cfg, 'auto_save') && cfg.auto_save
        % 디렉토리가 없으면 폴더를 새로 만들기
        if ~exist(cfg.save_dir, 'dir')
            mkdir(cfg.save_dir);
        end
        
        % 파일명 생성 (예: DroneSimResult_20260829_214718.mat)
        currentTimeString = datestr(now, 'yyyymmdd_HHMMSS');
        fileName = fullfile(cfg.save_dir, sprintf('DroneSimResult_%s.mat', currentTimeString));
        
        % log_data 구조체랑 cfg 통째로 저장해버리기
        save(fileName, 'log_data', 'cfg');
        
        disp(['으헤~ 결과 데이터가 여기 저장됐어: ', fileName]);
    else
        disp('자동 저장은 꺼져 있어서 파일로 남기지는 않았어.');
    end
    
    % 완료 메시지 창 띄우기
    msgbox('Simulation, Plotting & Auto-Saving Complete!', 'Success');
end