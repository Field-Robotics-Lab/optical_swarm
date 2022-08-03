function [debug,left,right] = aprilTag_tagfollow(ID,dist,head,range)
% aprilTag_tagfollow receives tag IDs, distances,headings, and the range
% of tag IDs and computes left and right turn 
% commands to follow AprilCube rabbit model


% AprilCube Tag numbers are 90-95
indmin = find(ID >= min(range));
indmax = find(ID <= max(range));
ind=intersect(indmin,indmax);
debug = ID(ind);
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
k_v = 0.3; % default 0.3
k_r = 1.7; % default 1.5
left = k_v*fwd -k_r*(sign(psi)*turn);
right = k_v*fwd + k_r*(sign(psi)*turn);
    


end