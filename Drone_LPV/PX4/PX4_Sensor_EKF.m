classdef PX4_Sensor_EKF < handle
    % PX4_SENSOR_EKF M10 GPS 노이즈(10 Hz 랜덤 워크 + 백색 노이즈) 및 EKF2 위상 지연(8 Hz LPF) 모사 모듈
    % - use_sensor_noise == false: Ground Truth 위치/속도 즉시 반환
    % - use_sensor_noise == true:
    %     * 10 Hz (0.1 s) 주기로 GPS 랜덤 워크(드리프트 1.5 m 수준) + 백색 노이즈 갱신
    %     * 차단주파수 8 Hz (시정수 tau ≈ 0.020 s, 위상 지연 약 20~30 ms) 1차 저역통과 필터 통과
    %     * 50 Hz LOCAL_POSITION_NED 형태(p_est, v_est)로 출력

    properties
        use_sensor_noise    % On/Off 스위치
        dt_sim              % 250 Hz (0.004 s)
        dt_gps              % 10 Hz (0.100 s)
        gps_step_ratio      % 25 스텝 (0.1 / 0.004)
        step_counter

        % 랜덤 워크 및 백색 노이즈 파라미터
        drift_bound         % 최대 드리프트 바운드 (1.5 m)
        rw_sigma_pos        % 10 Hz 랜덤 워크 스텝 표준편차 [m]
        white_sigma_pos     % GPS 위치 백색 노이즈 표준편차 [m]
        white_sigma_vel     % GPS/EKF 속도 백색 노이즈 표준편차 [m/s]

        % 내부 상태
        gps_drift           % 현재 누적된 랜덤 워크 드리프트 (3x1) [m]
        gps_noise_pos       % 10 Hz 샘플앤홀드된 위치 노이즈 (3x1)
        gps_noise_vel       % 10 Hz 샘플앤홀드된 속도 노이즈 (3x1)

        % EKF2 8 Hz 1차 LPF 상태 (지연 ~ 20-30 ms)
        fc_ekf              % 차단주파수 8 Hz
        tau_ekf             % 1 / (2*pi*8) ≈ 0.0199 s
        p_est               % EKF2 필터링된 위치 추정치 (3x1)
        v_est               % EKF2 필터링된 속도 추정치 (3x1)

        % 50 Hz LOCAL_POSITION_NED 출력 홀드 상태
        p_local_ned
        v_local_ned
    end

    methods
        function obj = PX4_Sensor_EKF(cfg, init_pos, init_vel)
            if isfield(cfg, 'use_sensor_noise')
                obj.use_sensor_noise = cfg.use_sensor_noise;
            else
                obj.use_sensor_noise = false;
            end

            obj.dt_sim         = cfg.dt_sim; % 0.004 s
            obj.dt_gps         = 0.10;       % 10 Hz M10 GPS
            obj.gps_step_ratio = round(obj.dt_gps / obj.dt_sim); % 25 스텝
            obj.step_counter   = 0;

            % 드리프트 및 노이즈 세팅
            if isfield(cfg, 'gps_drift_bound')
                obj.drift_bound = cfg.gps_drift_bound;
            else
                obj.drift_bound = 1.5; % 1.5 m 수준 드리프트
            end
            obj.rw_sigma_pos    = 0.06;
            obj.white_sigma_pos = 0.05;
            obj.white_sigma_vel = 0.04;

            % EKF2 8 Hz LPF 세팅
            obj.fc_ekf  = 8.0;
            obj.tau_ekf = 1.0 / (2.0 * pi * obj.fc_ekf); % ~0.0199 s (20 ms)

            obj.gps_drift     = zeros(3, 1);
            obj.gps_noise_pos = zeros(3, 1);
            obj.gps_noise_vel = zeros(3, 1);

            obj.p_est       = init_pos(:);
            obj.v_est       = init_vel(:);
            obj.p_local_ned = init_pos(:);
            obj.v_local_ned = init_vel(:);
        end

        function [p_out, v_out] = Update(obj, true_pos, true_vel, is_50hz_tick)
            % 매 250 Hz 스텝마다 EKF 연속 필터 갱신, 50 Hz 틱에서 LOCAL_POSITION_NED 샘플링 출력
            obj.step_counter = obj.step_counter + 1;

            if ~obj.use_sensor_noise
                obj.p_est       = true_pos(:);
                obj.v_est       = true_vel(:);
                obj.p_local_ned = obj.p_est;
                obj.v_local_ned = obj.v_est;
                p_out = obj.p_local_ned;
                v_out = obj.v_local_ned;
                return;
            end

            % 1. 10 Hz (25스텝마다) M10 GPS 랜덤 워크 + 백색 노이즈 갱신
            if mod(obj.step_counter - 1, obj.gps_step_ratio) == 0
                % 평균 회귀(Ornstein-Uhlenbeck) 랜덤 워크 + 1.5m 바운드 클리핑 (수직 채널은 기압계 융합으로 0.3배)
                scale_vec = [1.0; 1.0; 0.3];
                rw_step = obj.rw_sigma_pos * scale_vec .* randn(3, 1);
                obj.gps_drift = 0.98 * obj.gps_drift + rw_step;
                obj.gps_drift = max(-obj.drift_bound, min(obj.gps_drift, obj.drift_bound));

                obj.gps_noise_pos = obj.gps_drift + obj.white_sigma_pos * scale_vec .* randn(3, 1);
                obj.gps_noise_vel = obj.white_sigma_vel * scale_vec .* randn(3, 1);
            end

            % 2. EKF2 8 Hz 1차 저역통과 필터 (평활화 및 ~20-30 ms 위상 지연 모사)
            alpha_ekf = obj.dt_sim / (obj.tau_ekf + obj.dt_sim);
            meas_pos  = true_pos(:) + obj.gps_noise_pos;
            meas_vel  = true_vel(:) + obj.gps_noise_vel;

            obj.p_est = (1.0 - alpha_ekf) * obj.p_est + alpha_ekf * meas_pos;
            obj.v_est = (1.0 - alpha_ekf) * obj.v_est + alpha_ekf * meas_vel;

            % 3. 50 Hz LOCAL_POSITION_NED 출력 갱신
            if is_50hz_tick
                obj.p_local_ned = obj.p_est;
                obj.v_local_ned = obj.v_est;
            end

            p_out = obj.p_local_ned;
            v_out = obj.v_local_ned;
        end
    end
end

