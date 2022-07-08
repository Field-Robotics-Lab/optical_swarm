function [v_c,r_c] = aprilTag2cmd_vel(dist,head)
% aprilTag2cmd_vel receives tf transform information from the april tag
% detections and calculates a command surge and yaw rate


dist = mean(dist);
head = mean(head);

% Linear and Angular gains and command velocities
k_v=0.2;
k_h=0.5;
v_c=k_v*dist;
r_c=k_h*head;
end