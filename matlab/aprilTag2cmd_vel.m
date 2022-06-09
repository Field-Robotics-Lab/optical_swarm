function [v_c,r_c] = aprilTag2cmd_vel(trans)
% aprilTag2cmd_vel receives tf translation data from the apriltag_ros
% wrapper and calculates surge and yaw rate commands

X = trans.X;
Y = trans.Y;
Z = trans.Z;

dist = norm([X Y Z]);
% angle = quat2eul([w x y z]);
% head = rad2deg(angle(1))
psi=atan2(Y,Z);

% Linear and Angular gains and command velocities
k_v=1;
k_h=5;
v_c=k_v*dist;
r_c=k_h*psi;
end