close all
clear

try
    rosinit
catch
    rosshutdown
    rosinit
end


% ROS Calculation Rate
desiredRate = 10; %10 hertz 
rate = rateControl(desiredRate);


sandwich_1_left_pub = rospublisher('/sandwich_1/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_1_right_pub = rospublisher('/sandwich_1/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_1_left_msg = rosmessage(sandwich_1_left_pub);
sandwich_1_right_msg = rosmessage(sandwich_1_right_pub);
f


%%

while true
tftree = rostf;
frames = tftree.AvailableFrames;

boat_index = find(not(cellfun('isempty',strfind(frames,'boat0_tag'))));


% -------------------- Boat Following -----------------------------
if isempty(boat_index) == 1
    boat_tags = {};
else
    clear boat_tag_tf
    clear boat_tags
   for i=1:numel(boat_index)
        boat_tags{i} = frames{boat_index(i)};
   end
    for j=1:numel(boat_tags)
         boat_tag_tf(j) = getTransform(tftree,'sandwich_1/base_link',boat_tags{j},'Timeout',inf);
    end
[boattagID,boatdist,boathead] = aprilTag_xform(boat_tag_tf)
[boatleft,boatright] = aprilTag_boatfollow(boatdist,boathead);
 sandwich_1_left_msg.Data = boatleft;
 sandwich_1_right_msg.Data = boatright;
send(sandwich_1_left_pub, sandwich_1_left_msg);
send(sandwich_1_right_pub, sandwich_1_right_msg);
 end

% waitfor(rate);
end
