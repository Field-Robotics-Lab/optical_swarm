function [debug,left,right] = aprilTag_boatfollow(ID,dist,head)
% aprilTag_boatfollow receives tag IDs, distances, and heading angles and
% computes left and right turn commands to follow another sandwich boat

% Tag numbers for the boat to be followed must be known and specified

% Sandwich 0 tag numbers are 00-03
ind = find(ID <=4);
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
k_v = 0.3;
k_r = 2;
left = k_v*fwd -k_r*(sign(psi)*turn);
right = k_v*fwd + k_r*(sign(psi)*turn);
    


end