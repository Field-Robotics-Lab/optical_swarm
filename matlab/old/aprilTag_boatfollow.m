function [debug,left,right] = aprilTag_boatfollow(ID,dist,head,range)
% aprilTag_boatfollow receives tag IDs, distances, heading angles, 
% and the range of desired tag values and computes left and right 
% turn commands to follow another sandwich boat

% Tag numbers for the boat to be followed must be known and specified

% Sandwich 0 tag numbers are 00-03
indmin = find(ID >= min(range));
indmax = find(ID <= max(range));
ind=intersect(indmin,indmax);
debug = ID(ind);
dist = dist(ind);
head = head(ind);


dist = mean(dist);
psi=mean(head);

if dist > 50
    fwd = 2;
elseif dist <= 50 && dist > 5
    fwd = (2/40)*dist - (10/45);
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
k_v = 0.4; % 0.3
k_r = 2; % 2
left = k_v*fwd -k_r*(sign(psi)*turn);
right = k_v*fwd + k_r*(sign(psi)*turn);
    


end