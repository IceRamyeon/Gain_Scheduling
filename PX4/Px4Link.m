classdef Px4Link < handle
    % PX4LINK  MATLAB(UAV Toolbox mavlinkio) <-> PX4 MAVLink 연결
    %   SITL(Gazebo)과 실기체 모두 같은 클래스를 사용하고, 연결 포트만 바꾼다.
    %   - 상태 수신 : LOCAL_POSITION_NED + ATTITUDE  -> [x y z vx vy vz phi theta psi p q r]' (NED)
    %   - 위치 명령 : SET_POSITION_TARGET_LOCAL_NED  (PX4 기본 위치제어기 사용 구간: 이륙, 이동, 종료)
    %   - 자세 명령 : SET_ATTITUDE_TARGET            (MATLAB PositionCtrl 사용 구간: 프리롤, 경로 추종)

    properties
        dialect
        mav
        client
        subPos
        subAtt
        target_sys  = 1
        target_comp = 1
        msgPosT
        msgAttT
        msgCmd
    end

    methods
        %% CONSTRUCTOR
        function obj = Px4Link(localPort, timeout_s)
            if nargin < 1, localPort = 14540; end
            if nargin < 2, timeout_s = 15; end

            obj.dialect = mavlinkdialect("common.xml");
            obj.mav = mavlinkio(obj.dialect, "SystemID", 255, "ComponentID", 190);
            connect(obj.mav, "UDP", "LocalPort", localPort);

            % PX4 하트비트 대기
            t0 = tic;
            while true
                c = listClients(obj.mav);
                if ~isempty(c) && any(c.SystemID == obj.target_sys), break; end
                if toc(t0) > timeout_s
                    disconnect(obj.mav);
                    error("Px4Link:noHeartbeat", ...
                        "PX4 하트비트 없음 (UDP %d). SITL 실행 여부와 포트를 확인하세요.", localPort);
                end
                pause(0.2);
            end

            obj.client = mavlinkclient(obj.mav, obj.target_sys, obj.target_comp);
            obj.subPos = mavlinksub(obj.mav, obj.client, "LOCAL_POSITION_NED", "BufferSize", 1);
            obj.subAtt = mavlinksub(obj.mav, obj.client, "ATTITUDE", "BufferSize", 1);

            obj.msgPosT = createmsg(obj.dialect, "SET_POSITION_TARGET_LOCAL_NED");
            obj.msgAttT = createmsg(obj.dialect, "SET_ATTITUDE_TARGET");
            obj.msgCmd  = createmsg(obj.dialect, "COMMAND_LONG");
        end

        function delete(obj)
            try
                disconnect(obj.mav);
            catch
            end
        end

        %% 상태 수신
        function [x, t_boot_s, ok] = getState(obj)
            % x        : [x y z vx vy vz phi theta psi p q r]' (NED, rad, rad/s)
            % t_boot_s : 위치 샘플(LOCAL_POSITION_NED)의 PX4 부팅 후 시간 [s] (ULog 타임스탬프와 같은 기준)
            %            -> 위치가 새로 들어왔을 때만 값이 바뀐다. PositionCtrl 의 dt·미분은 이 시간으로 계산해야
            %               위치가 갱신되지 않은 스텝에서 미분이 0/큰 값으로 번갈아 튀지 않는다.
            x = nan(12, 1); t_boot_s = NaN; ok = false;
            mp = latestmsgs(obj.subPos, 1);
            ma = latestmsgs(obj.subAtt, 1);
            if isempty(mp) || isempty(ma), return; end
            P = mp.Payload; A = ma.Payload;
            x = double([P.x; P.y; P.z; P.vx; P.vy; P.vz; ...
                        A.roll; A.pitch; A.yaw; A.rollspeed; A.pitchspeed; A.yawspeed]);
            t_boot_s = double(P.time_boot_ms) * 1e-3;
            ok = true;
        end

        %% 위치 명령 (PX4 기본 위치제어기)
        function sendPosition(obj, p_ned, yaw, v_ned, a_ned)
            m = obj.msgPosT;
            m.Payload.target_system(:)    = obj.target_sys;
            m.Payload.target_component(:) = obj.target_comp;
            m.Payload.coordinate_frame(:) = 1;                 % MAV_FRAME_LOCAL_NED
            if nargin < 5 || isempty(a_ned)
                a_ned = [0; 0; 0];
                if nargin < 4 || isempty(v_ned)
                    m.Payload.type_mask(:) = 56 + 448 + 2048;  % 위치 + yaw
                    v_ned = [0; 0; 0];
                else
                    m.Payload.type_mask(:) = 448 + 2048;       % 위치 + 속도 + yaw
                end
            else
                m.Payload.type_mask(:) = 2048;                 % 위치 + 속도 + 가속도(feedforward) + yaw
            end
            m.Payload.afx(:) = a_ned(1); m.Payload.afy(:) = a_ned(2); m.Payload.afz(:) = a_ned(3);
            m.Payload.x(:)  = p_ned(1); m.Payload.y(:)  = p_ned(2); m.Payload.z(:)  = p_ned(3);
            m.Payload.vx(:) = v_ned(1); m.Payload.vy(:) = v_ned(2); m.Payload.vz(:) = v_ned(3);
            m.Payload.yaw(:) = yaw;
            sendmsg(obj.mav, m, obj.client);
        end

        %% 자세 + 추력 명령 (MATLAB PositionCtrl 출력)
        function sendAttitude(obj, phi, theta, psi, thrust_norm)
            q = Px4Link.eul2quat_zyx(phi, theta, psi);
            m = obj.msgAttT;
            m.Payload.target_system(:)    = obj.target_sys;
            m.Payload.target_component(:) = obj.target_comp;
            m.Payload.type_mask(:) = 7;                        % body rate 무시, 자세 + 추력 사용
            m.Payload.q(:) = q;
            m.Payload.thrust(:) = thrust_norm;                 % 0 ~ 1
            sendmsg(obj.mav, m, obj.client);
        end

        %% COMMAND_LONG
        function command(obj, id, p)
            m = obj.msgCmd;
            m.Payload.target_system(:)    = obj.target_sys;
            m.Payload.target_component(:) = obj.target_comp;
            m.Payload.command(:) = id;
            m.Payload.param1(:) = p(1); m.Payload.param2(:) = p(2); m.Payload.param3(:) = p(3);
            m.Payload.param4(:) = p(4); m.Payload.param5(:) = p(5); m.Payload.param6(:) = p(6);
            m.Payload.param7(:) = p(7);
            sendmsg(obj.mav, m, obj.client);
        end

        % MAV_CMD_DO_SET_MODE(176): base_mode=1(custom), PX4 custom main/sub mode
        function setOffboard(obj), obj.command(176, [1 6 0 0 0 0 0]); end   % OFFBOARD
        function setHold(obj),     obj.command(176, [1 4 3 0 0 0 0]); end   % AUTO.LOITER
        function setLand(obj),     obj.command(176, [1 4 6 0 0 0 0]); end   % AUTO.LAND
        % MAV_CMD_COMPONENT_ARM_DISARM(400)
        function arm(obj),         obj.command(400, [1 0 0 0 0 0 0]); end
        % MAV_CMD_SET_MESSAGE_INTERVAL(511): 메시지 송신 주기 요청 (예: LOCAL_POSITION_NED=32, ATTITUDE=30)
        function setMessageRate(obj, msgId, hz), obj.command(511, [msgId, 1e6 / hz, 0, 0, 0, 0, 0]); end

        %% 파라미터 읽기 (예: MPC_THR_HOVER)
        function val = readParam(obj, name, timeout_s)
            if nargin < 3, timeout_s = 3; end
            val = NaN;
            sub = mavlinksub(obj.mav, obj.client, "PARAM_VALUE", "BufferSize", 50);
            m = createmsg(obj.dialect, "PARAM_REQUEST_READ");
            m.Payload.target_system(:)    = obj.target_sys;
            m.Payload.target_component(:) = obj.target_comp;
            m.Payload.param_id(:) = 0;
            m.Payload.param_id(1:numel(name)) = double(name);
            m.Payload.param_index(:) = -1;
            sendmsg(obj.mav, m, obj.client);

            t0 = tic;
            while toc(t0) < timeout_s && isnan(val)
                msgs = latestmsgs(sub, 50);
                for k = 1:numel(msgs)
                    pid = char(double(msgs(k).Payload.param_id(:)'));
                    pid = pid(pid ~= char(0));
                    if strcmp(strtrim(pid), name)
                        val = double(msgs(k).Payload.param_value);
                        break;
                    end
                end
                pause(0.1);
            end
            try
                delete(sub);
            catch
            end
        end
    end

    methods (Static)
        function q = eul2quat_zyx(phi, theta, psi)
            % ZYX(yaw-pitch-roll) 오일러각 -> 쿼터니언 [w x y z] (Hamilton, NED/FRD)
            cr = cos(phi/2);   sr = sin(phi/2);
            cp = cos(theta/2); sp = sin(theta/2);
            cy = cos(psi/2);   sy = sin(psi/2);
            q = [cr*cp*cy + sr*sp*sy, ...
                 sr*cp*cy - cr*sp*sy, ...
                 cr*sp*cy + sr*cp*sy, ...
                 cr*cp*sy - sr*sp*cy];
        end
    end
end
