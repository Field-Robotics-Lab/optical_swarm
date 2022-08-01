function [left,right] = aprilTag_tagfollow(ID,dist,head)
% aprilTag2cmd_vel receives tf translation data from the apriltag_ros
% wrapper and calculates left and right thrust commands


% AprilCube Tag numbers are 1-4
ind = find(ID >=20);
dist = dist(ind);
head = head(ind);


dist = mean(dist);
psi=mean(head);


if dist > 50
    fwd = 1.25;
elseif dist <= 50 && dist > 10
    fwd = (1.25/40)*dist - (12.5/40);
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