function [left,right] = aprilTag_thrustcmd(trans,rot)
% aprilTag2cmd_vel receives tf translation data from the apriltag_ros
% wrapper and calculates left and right thrust commands

% Position
X = trans.X;
Y = trans.Y;
Z = trans.Z;

% Quaternions
w = rot.W;
x = rot.X;
y = rot.Y;
z = rot.Z;

eul = rad2deg(quat2eul([w x y z]))
psi = eul(1)
dist = norm([X Y Z]);
% angle = quat2eul([w x y z]);
% head = rad2deg(angle(1))
psi=atan2d(Y,Z);

% Linear and Angular gains and command velocities
k_v=0.01;
k_h=0.1;
turn = k_h*psi;

left = (k_v*dist)-turn;
right = (k_v*dist)+turn;
end