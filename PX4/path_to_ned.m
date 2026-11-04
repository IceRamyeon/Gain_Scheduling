function out = path_to_ned(v, cfg, isPoint)
% PATH_TO_NED  경로 좌표계(시뮬레이션 좌표) -> PX4 Local NED
%   경로 좌표계 = PID_Drone 시뮬레이션의 좌표계 (x, y, z, z는 아래 방향).
%   cfg.px4.path_origin  : 경로 좌표계 원점의 NED 위치 [m]
%   cfg.px4.path_rot_deg : 경로 좌표계 x축의 NED 방위각 [deg] (0 = 북쪽, 90 = 동쪽)
%   isPoint = true 이면 위치(원점 이동 포함), false 이면 벡터(회전만)
    if nargin < 3, isPoint = true; end
    a = deg2rad(cfg.px4.path_rot_deg);
    Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
    out = Rz * v(:);
    if isPoint
        out = out + cfg.px4.path_origin(:);
    end
end
