function [debug,target_ID,left,right] = aprilTag_vbap(ID,dist,head,target_range,boat_range)
% aprilTag_vbap implements a vehiche-body artificial potential field
% to maintain inter-vehicle spacing while tracking an aprilCube target.
% 
% aprilTag_vbap receives:
%   - tag IDs (1 x k)
%   - distances (1 x k)
%   - heading angles (1 x k)
%   - range of target IDs (1 x l)
%   - range of other boat IDs (m x n)
% where:
%   - k = number of currently detected tags
%   - l = number of tags on target (aprilCube, lead boat, etc.)
%   - m = number of swarm partner vessels
%   - n = number of tags on partner vessels
% 
% aprilTag_vbap returns:
%   - left and right thrust commands


debug = ID;
target_indmin = find(ID >= min(target_range));
target_indmax = find(ID <= max(target_range));
target_ind=intersect(target_indmin,target_indmax);
target_ID = ID(target_ind);
% ****** ADD LOGIC HERE TO SELECT PARTNER BOAT AS TARGET TO FOLLOW IF 
% ****** APRILCUBE IS LOST
if isempty(target_ind) == 1
   dist_target = 10;
   head_target = 0;
else
   dist_target = dist(target_ind);
   head_target = head(target_ind);
 end

dist_target = mean(dist_target);
psi_target=mean(head_target);

% Forward component of thrust commands: depends only on distance from
% target
if dist_target > 50
    fwd = 1.25;
elseif dist_target <= 50 && dist_target > 10
    fwd = (1.25/40)*dist_target - (12.5/40);
else
    fwd = 0;
end

% Distance and heading to partner vessels

% Spring dmin, dmax, ko
dmin = 15;
dmax = 30;
ko = -0.075;

m = height(boat_range);
n = length(boat_range);
dist_boat = zeros(1,m);
head_boat = zeros(1,m);


for ii = 1:m
    boat_indmin = find(ID >= min(boat_range(ii,:)));
    boat_indmax = find(ID <= max(boat_range(ii,:)));
    boat_ind=intersect(boat_indmin,boat_indmax);
    if isempty(boat_ind) == 0
        boat_ID(ii,1:numel(boat_ind)) = ID(boat_ind);
        dist_boat(ii) = mean(dist(boat_ind));
        head_boat(ii) = mean(head(boat_ind));
        if dist_boat(ii) < dmax
         turn_boat(ii) = ko*(dist_boat(ii)-dmin)*sign(head_boat(ii));

        else
        turn_boat(ii) = 0;
        end
    else
        turn_boat(ii) = 0;
    end

end

turn_boat = sum(turn_boat);
% 
% dist_target
% psi_target
% boat_ID'
% dist_boat
% head_boat
% turn_boat




val=abs(psi_target);

if val > 30
    turn_target = 1;
elseif val <=30 && val>5
    turn_target = 0.04*val - 0.2;
else
    turn_target = 0;
end

turn_target = sign(psi_target)*turn_target;

k_v = 0.4; % 0.3
k_r = 1.5; % 2
left = k_v*fwd -k_r*(turn_target - turn_boat);
right = k_v*fwd + k_r*(turn_target - turn_boat);
    
end
