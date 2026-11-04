function x_path = ned_to_path(x_ned, cfg)
% NED_TO_PATH  PX4 상태 [x y z vx vy vz phi theta psi p q r]'(NED) -> 경로 좌표계 상태
%   위치는 원점 이동 + 회전, 속도는 회전, yaw는 경로 좌표계 방위각만큼 뺀다.
%   roll, pitch, 각속도(p, q, r)는 기체 좌표계 값이라 그대로 둔다.
    a = deg2rad(cfg.px4.path_rot_deg);
    Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
    x_path = x_ned(:);
    x_path(1:3) = Rz' * (x_ned(1:3) - cfg.px4.path_origin(:));
    x_path(4:6) = Rz' * x_ned(4:6);
    x_path(9)   = x_ned(9) - a;
end
