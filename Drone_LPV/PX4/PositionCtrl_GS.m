classdef PositionCtrl_GS < handle
    %% MEMBERS
    properties
        g
        m
        dt
        state
        cmd
        
        use_lpv
        
        x_des, y_des, z_des, psi_des
        v_des, kappa_des
        
        x_err, x_err_prev, x_err_sum
        y_err, y_err_prev, y_err_sum
        z_err, z_err_prev, z_err_sum
        
        % Fixed Gains (LPV 꺼졌을 때 사용)
        kP_x_fix, kI_x_fix, kD_x_fix
        kP_y_fix, kI_y_fix, kD_y_fix
        kP_z_fix, kI_z_fix, kD_z_fix
        
        % LPV Desired Pole Coefficients (s^3 + a2*s^2 + a1*s + a0 = 0)
        % Poles: [-1.5+1.2i, -1.5-1.2i, -5] => a2=8, a1=18.69, a0=18.45
        lpv_a2, lpv_a1, lpv_a0
        
        is_first_run
        max_angle

        use_feedforward  % Feedforward 사용 여부 플래그 추가
        use_z_scheduling  % Z축 게인 스케줄링 여부 플래그 추가
        z_D_on_meas       % true: z 미분항을 측정 속도(-vz)로 계산 (기준 고도 계단 시 미분 킥 제거)
        use_ndi           % true: 기존 비선형 역산(NDI) 분기 사용 (cfg.Use_NDI)
        err_yaw_desired   % true: 오차를 목표 yaw(psi_des)로 회전, false: 측정 yaw (cfg.Err_Yaw)
        eq_delay_comp     % true: 평형점에 자세 루프 지연 보정 피치 추가 (cfg.Eq_Delay_Comp)
        K_att             % 자세 루프 각도 P 게인 [1/s] (PX4 MC_PITCH_P, 지연 보정용)

        % Logging용 변수
        current_kP_z
        current_kI_z
        current_kD_z

        % PPO 잔차 게인 오프셋 (기본값 0 -> ChoeJae PositionCtrl과 100% 동일)
        offset_P_xy = 0.0
        offset_D_xy = 0.0
        offset_P_z  = 0.0
        offset_D_z  = 0.0
    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = PositionCtrl_GS(gains, params, cfg)
            obj.g = 9.81;
            obj.dt = cfg.dt;
            obj.m = params('mass');
            
            % LPV 사용 여부
            if isfield(cfg, 'Feedforward')
                obj.use_feedforward = cfg.Feedforward;
            else
                obj.use_feedforward = false;
            end
            
            % Z축 스케줄링 사용 여부
            if isfield(cfg, 'Z_Gain_Scheduling')
                obj.use_z_scheduling = cfg.Z_Gain_Scheduling;
            else
                obj.use_z_scheduling = true; % 기본값은 3번 모델
            end

            % 제어기 구조
            %   Use_NDI = false (기본): 평형점 + 선형 보정 구조
            %       Feedforward = true  -> 평형점 phi_eq(v,kappa), U1_eq(v,kappa) 스케줄링
            %       Feedforward = false -> 호버 평형점 고정 (phi_eq = 0, U1_eq = m g, b_z = 1/m) = 피드포워드 없는 고정 PID
            %   Use_NDI = true: 기존 비선형 역산 분기 (고정 게인 P_x.. 를 가속도 단위로 사용 -> posGain 단위 주의)
            if isfield(cfg, 'Use_NDI')
                obj.use_ndi = cfg.Use_NDI;
            else
                obj.use_ndi = false;
            end

            % z 미분항 구성: true(기본) = 측정 속도 기반 K_D*(0 - vz),  false = 기존 오차 미분 K_D*d(z_err)/dt
            %  -> 두 방식은 K_D*vz_des 항만 다르다. 등고도(vz_des = 0)에서는 같은 값을 내고,
            %     기준 고도가 계단으로 바뀌는 순간에만 기존 방식이 K_D*dh/dt 의 킥을 낸다.
            %     폐루프 특성방정식(극점)은 같고 기준 입력 -> 고도 전달함수의 영점만 다르다.
            if isfield(cfg, 'Z_D_on_measurement')
                obj.z_D_on_meas = cfg.Z_D_on_measurement;
            else
                obj.z_D_on_meas = true;
            end

            % 기본 고정 게인 저장
            obj.kP_x_fix = gains('P_x'); obj.kI_x_fix = gains('I_x'); obj.kD_x_fix = gains('D_x');
            obj.kP_y_fix = gains('P_y'); obj.kI_y_fix = gains('I_y'); obj.kD_y_fix = gains('D_y');
            obj.kP_z_fix = gains('P_z'); obj.kI_z_fix = gains('I_z'); obj.kD_z_fix = gains('D_z');
            
            % 극점 역산 계수 (Overshoot ~4.3%, t_ss ~4s)
            obj.lpv_a1 = gains('P_x') * obj.g;
            obj.lpv_a0 = gains('I_x') * obj.g;
            obj.lpv_a2 = gains('D_x') * obj.g;
            
            % 각도 제한: cfg.max_angle_deg (기본 30 deg). phi_eq 가 이 값을 넘으면 평형점 자체가 잘린다
            if isfield(cfg, 'max_angle_deg') && ~isempty(cfg.max_angle_deg)
                obj.max_angle = cfg.max_angle_deg * (pi/180);
            else
                obj.max_angle = 30 * (pi/180);
            end

            % 오차 회전 yaw: 'measured'(기본, 측정 yaw) / 'desired'(목표 yaw)
            %  PX4 는 자세 명령(psi_des 포함 쿼터니언)의 추력 방향을 먼저 맞추고 yaw 는 늦게 따라간다.
            %  선회 중 yaw 가 늦으면 측정 yaw 로 돌린 body 오차가 psi_des 기준 명령과 어긋나므로 PX4 에서는 'desired' 권장
            obj.err_yaw_desired = isfield(cfg, 'Err_Yaw') && strcmpi(cfg.Err_Yaw, 'desired');

            % 평형점 지연 보정: 기울어진 채 수직축으로 회전하려면 피치 각속도 q_eq = |v kappa| sin|phi_eq| 가 필요하고,
            %  각도 P 자세 루프(K_att)는 이를 만들려고 정상상태 피치 오차 q_eq / K_att 를 남긴다 -> 그만큼 명령 피치에 미리 더함
            obj.eq_delay_comp = isfield(cfg, 'Eq_Delay_Comp') && cfg.Eq_Delay_Comp;
            if isfield(cfg, 'K_att') && ~isempty(cfg.K_att)
                obj.K_att = cfg.K_att;
            else
                obj.K_att = 6.5;
            end
            obj.Reset();
        end
        
        %% Reset
        function Reset(obj)
            obj.x_err = 0; obj.x_err_prev = 0; obj.x_err_sum = 0;
            obj.y_err = 0; obj.y_err_prev = 0; obj.y_err_sum = 0;
            obj.z_err = 0; obj.z_err_prev = 0; obj.z_err_sum = 0;
            obj.offset_P_xy = 0.0; obj.offset_D_xy = 0.0;
            obj.offset_P_z  = 0.0; obj.offset_D_z  = 0.0;
            obj.is_first_run = true;
        end

        function setResidualOffsets(obj, dP_xy, dD_xy, dP_z, dD_z)
            obj.offset_P_xy = dP_xy;
            obj.offset_D_xy = dD_xy;
            obj.offset_P_z  = dP_z;
            obj.offset_D_z  = dD_z;
        end

        function pid_gains = GetZ_PID(obj)
            pid_gains = [obj.current_kP_z; obj.current_kI_z; obj.current_kD_z];
        end
        
        %% Update
        function att_cmd = Update(obj, cmd, state)
            % 1. 목표 수신: cmd = [x_des, y_des, z_des, psi_des, v, kappa]
            obj.x_des = cmd(1); obj.y_des = cmd(2); obj.z_des = cmd(3); obj.psi_des = cmd(4);
            if length(cmd) >= 6
                obj.v_des = cmd(5); obj.kappa_des = cmd(6);
            else
                obj.v_des = 0; obj.kappa_des = 0;
            end
            
            x = state(1); y = state(2); z = state(3); psi = state(9); 
            if obj.err_yaw_desired
                psi = obj.psi_des;      % 오차 회전을 명령 좌표계(목표 yaw)와 맞춤
            end
            
            % 2. 글로벌 오차 계산
            obj.x_err = obj.x_des - x; 
            obj.y_err = obj.y_des - y; 
            obj.z_err = obj.z_des - z;

            if obj.is_first_run
                obj.x_err_prev = obj.x_err; obj.y_err_prev = obj.y_err; obj.z_err_prev = obj.z_err;
                obj.is_first_run = false;
            end
            
            % 오차 적분 및 미분 계산
            obj.x_err_sum = obj.x_err_sum + obj.x_err * obj.dt; 
            obj.y_err_sum = obj.y_err_sum + obj.y_err * obj.dt; 
            obj.z_err_sum = obj.z_err_sum + obj.z_err * obj.dt; 
            
            dx_err = (obj.x_err - obj.x_err_prev)/obj.dt;
            dy_err = (obj.y_err - obj.y_err_prev)/obj.dt;
            dz_err = (obj.z_err - obj.z_err_prev)/obj.dt;
            if obj.z_D_on_meas
                dz_err = -state(6);     % 측정 속도 기반: d(z_err)/dt = vz_des - vz 에서 vz_des(기준 고도 변화율) 항 제외
            end
            
            % 3. Body 좌표계 오차 회전 (Heading 기준)
            ex_body =  cos(psi) * obj.x_err + sin(psi) * obj.y_err;
            ey_body = -sin(psi) * obj.x_err + cos(psi) * obj.y_err;
            
            ex_body_sum =  cos(psi) * obj.x_err_sum + sin(psi) * obj.y_err_sum;
            ey_body_sum = -sin(psi) * obj.x_err_sum + cos(psi) * obj.y_err_sum;
            
            dex_body =  cos(psi) * dx_err + sin(psi) * dy_err;
            dey_body = -sin(psi) * dx_err + cos(psi) * dy_err;
            
            if ~obj.use_ndi
                if obj.use_feedforward
                    % 피드포워드 계산 (NED: +phi -> +y(오른쪽) 가속, kappa > 0 -> +y 쪽 선회)
                    %  -> 구심 가속도 v^2 kappa 를 롤로 만들려면 tan(phi_eq) = +v^2 kappa / g
                    phi_eq = atan((obj.v_des^2 * obj.kappa_des) / obj.g);
                    U1_eq = obj.m * sqrt(obj.g^2 + (obj.v_des^2 * obj.kappa_des)^2);
                else
                    % 피드포워드 없음: 호버 평형점 고정 (선회에 필요한 롤·추력을 전부 PID 피드백이 만들어야 함)
                    phi_eq = 0;
                    U1_eq  = obj.m * obj.g;
                end
                
                % X, Y축 게인 자동 산출 (+ PPO 잔차 오프셋 합성, 기본값 0)
                kP_xy = max(obj.lpv_a1 / obj.g + obj.offset_P_xy, 0.02);
                kI_xy = obj.lpv_a0 / obj.g;
                kD_xy = max(obj.lpv_a2 / obj.g + obj.offset_D_xy, 0.02);
                
                % Z축 게인 산출 로직 분리
                if obj.use_z_scheduling && obj.use_feedforward
                    % 3번 모델: 기울기(phi_eq)에 따른 스케줄링 적용
                    b_z = cos(phi_eq) / obj.m;
                else
                    % 2번 모델: 호버링 기준 고정 게인 (cos(0) = 1)
                    b_z = 1.0 / obj.m;
                end
                
                kP_z = max(obj.lpv_a1 / b_z + obj.offset_P_z, 0.5);
                kI_z = obj.lpv_a0 / b_z;
                kD_z = max(obj.lpv_a2 / b_z + obj.offset_D_z, 0.2);
                
                % 로깅을 위해 객체 속성에 저장
                obj.current_kP_z = kP_z; 
                obj.current_kI_z = kI_z; 
                obj.current_kD_z = kD_z;
                
                % 오차 피드백 제어량 계산 (부호 주의)
                delta_theta = -(kP_xy * ex_body + kI_xy * ex_body_sum + kD_xy * dex_body);
                delta_phi = (kP_xy * ey_body + kI_xy * ey_body_sum + kD_xy * dey_body);
                delta_U1 = -(kP_z * obj.z_err + kI_z * obj.z_err_sum + kD_z * dz_err);
                
                % 최종 지령 = 평형점(피드포워드) + 보상값
                % 평형점 피치: 기본 0, 지연 보정 시 자세 루프 정상상태 오차만큼 기수를 미리 든다 (NED: +theta = 기수 들림)
                theta_eq = 0;
                if obj.eq_delay_comp && obj.use_feedforward
                    q_eq     = abs(obj.v_des * obj.kappa_des) * sin(abs(phi_eq));
                    theta_eq = q_eq / obj.K_att;
                end
                theta_des = theta_eq + delta_theta;
                phi_des   = phi_eq + delta_phi;
                U1_des    = U1_eq + delta_U1;
                
            else
                % [기존 비선형 역산 방식 (NDI)]
                accel_x = obj.kP_x_fix * obj.x_err + obj.kI_x_fix * obj.x_err_sum + obj.kD_x_fix * dx_err;
                accel_y = obj.kP_y_fix * obj.y_err + obj.kI_y_fix * obj.y_err_sum + obj.kD_y_fix * dy_err;
                
                accel_fwd =  cos(psi) * accel_x + sin(psi) * accel_y;
                accel_rgt = -sin(psi) * accel_x + cos(psi) * accel_y;
                accel_up = obj.g;
                
                thrust_sp = sqrt(accel_fwd^2 + accel_rgt^2 + accel_up^2);
                theta_des = -atan2(accel_fwd, accel_up); 
                sin_phi = max(min(accel_rgt / thrust_sp, 1.0), -1.0);
                phi_des = asin(sin_phi);
                
                % Z축 출력을 추력(U1)으로 맞춰서 변환 (NDI 방식에서의 추력)
                % 기존 zdot 제어를 유지하되, 출력을 U1 단위로 변환해 내보냄
                zdot_des = obj.kP_z_fix * obj.z_err + obj.kI_z_fix * obj.z_err_sum + obj.kD_z_fix * dz_err;
                U1_des = (obj.m * obj.g) - zdot_des; % 기존 AttitudeCtrl에 있던 공식

                % LPV 꺼졌을 땐 고정 게인 반환
                obj.current_kP_z = obj.kP_z_fix; 
                obj.current_kI_z = obj.kI_z_fix; 
                obj.current_kD_z = obj.kD_z_fix;
            end
            
            % 각도 제한 (Safety)
            theta_des = max(min(theta_des, obj.max_angle), -obj.max_angle);
            phi_des   = max(min(phi_des,   obj.max_angle), -obj.max_angle);
            
            obj.x_err_prev = obj.x_err;
            obj.y_err_prev = obj.y_err;
            obj.z_err_prev = obj.z_err;
            
            % 9. 최종 출력 (이제 Z축 출력이 zdot_des가 아니라 U1_des임!)
            att_cmd = [phi_des; theta_des; obj.psi_des; U1_des];
        end
    end
end