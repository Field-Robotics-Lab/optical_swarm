function [speed,rate,dist_target,head_target] = rabbit_follow(p3d,rabbit)
% 
follow_dist = 20;

boatX = p3d.Pose.Pose.Position.X;
boatY = p3d.Pose.Pose.Position.Y;

W=p3d.Pose.Pose.Orientation.W;
X=p3d.Pose.Pose.Orientation.X;
Y=p3d.Pose.Pose.Orientation.Y;
Z=p3d.Pose.Pose.Orientation.Z;

q = [W,X,Y,Z];
e = quat2eul(q);
psi=rad2deg(e(1));


rabbitX = rabbit.Point.X;
rabbitY = rabbit.Point.Y;

dist_target = ((rabbitX - boatX).^2 + (rabbitY - boatY).^2).^0.5;
head_target = wrapTo180(atan2d((rabbitY - boatY),(rabbitX - boatX))-psi);

if dist_target <= follow_dist
    dist_err = 0;
else
    dist_err = dist_target - follow_dist;
end


% Max speed and turn rate
max_speed = 10;
max_rate = 30; % 22

% Speed and turn rate gains
Ku = 1; % 0.3
Kr = 0.3; % 2


speed_cmd = Ku*dist_err;

% Saturation 
speed = min(max_speed,max(-max_speed,speed_cmd));
rate = min(max_rate,max(-max_rate,Kr*head_target));
    
end
