function [left,right] = aprilTag_tagfollow(ID,dist,head)
% aprilTag_tagfollow receives tag IDs, distances,and headings and computes
% left and right turn commands to follow AprilCube rabbit model2


% AprilCube Tag numbers are 90-95
ind = find(ID >=90);
debug = ID(ind)
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
k_v = 0.1; % default 0.3
k_r = 1.0; % default 1.5
left = k_v*fwd -k_r*(sign(psi)*turn);
right = k_v*fwd + k_r*(sign(psi)*turn);
    


end