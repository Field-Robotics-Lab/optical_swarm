function [left,right] = aprilTag_thrustcmd(dist,head)
% aprilTag2cmd_vel receives tf translation data from the apriltag_ros
% wrapper and calculates left and right thrust commands

% Position
% X = trans.X;
% Y = trans.Y;
% Z = trans.Z;

% Quaternions
% w = rot.W;
% x = rot.X;
% y = rot.Y;
% z = rot.Z;


dist = mean(dist);
% angle = quat2eul([w x y z]);
% head = rad2deg(angle(1))
psi=mean(head);

% Linear and Angular gains and command velocities
% k_v=0.1;
% k_h=0.1;
% turn = k_h*psi;
% 
% left = (k_v*dist)-turn
% right = (k_v*dist)+turn
if dist > 50
    fwd = 1.5;
elseif dist <= 50 && dist > 5
    fwd = (1.5/45)*dist - (1/6);
else
    fwd = 0;
end

val=abs(psi);

if val > 30
    turn = 1;
elseif val <=30 && val>5
    turn = 0.04*val - 0.2;
else
    turn = 0;
end
k_v = 0.3;
k_r = 1.5;
left = k_v*fwd -k_r*(sign(psi)*turn);
right = k_v*fwd + k_r*(sign(psi)*turn);
    


end