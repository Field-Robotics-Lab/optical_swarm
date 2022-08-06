function [debug,target_ID,speed,rate,dist_target] = vbap_test(ID,dist,head,target_range,boat_range,dist_old,head_offset)
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
follow_dist = 20;


% ****** ADD LOGIC HERE TO SELECT PARTNER BOAT AS TARGET TO FOLLOW IF 
% ****** APRILCUBE IS LOST
if isempty(target_ind) == 1
   dist_target = follow_dist;
   head_target = 0;
else
   dist_target = dist(target_ind);
   head_target = head(target_ind);
 end
dist_target = mean(dist_target);
dist_net = dist_target-follow_dist;


% Forward component of thrust commands: depends only on distance from
% target

% Distance and heading to partner vessels

% Spring dmin, dmax, ko
d0 = 25;
dmin = 20;
dmax = 50;
ko = 0.1;

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
        
        if dist_boat(ii) <= dmax @@ dist_boat(ii) >= d0
         turn_boat(ii) = ko*(dist_boat(ii)-d0)*sign(head_boat(ii));
         elseif dist_boat(ii) < dmin
         turn_boat(ii) = -ko*sign(head_boat(ii))*(dist_boat(ii) - dmin)^2;
        else
        turn_boat(ii) = 0;
        end
    else
        turn_boat(ii) = 0;
    end

end
head_offset = head_offset*sign(sum(head_boat));

turn_target=mean(head_target)-head_offset;
turn_boat = sum(turn_boat);


% Max speed and turn rate
max_speed = 20;
max_rate = 22; % 22

% Speed and turn rate gains
Ku = 0.5; % 0.3
Kd = 0.1; % Derivative gain to avoid collision 0.02

Kr = 0.4; % 2

rel_vel = (dist_target - dist_old)/0.1;

speed_cmd = Ku*dist_net + Kd*rel_vel;
% Saturation 
speed = min(max_speed,max(-max_speed,Ku*speed_cmd));
rate = min(max_rate,max(-max_rate,Kr*turn_target + turn_boat));
    
end
