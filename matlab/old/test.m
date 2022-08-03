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




% Setup Publisher
% cmd_pub = rospublisher('/robot0/sandwich_0/cmd_vel','geometry_msgs/Twist');
% cmd_msg = rosmessage(cmd_pub);
sandwich_0_left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_0_right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_0_left_msg = rosmessage(sandwich_0_left_pub);
sandwich_0_right_msg = rosmessage(sandwich_0_right_pub);

% sandwich_1_left_pub = rospublisher('/sandwich_1/thrusters/left_thrust_cmd','std_msgs/Float32');
% sandwich_1_right_pub = rospublisher('/sandwich_1/thrusters/right_thrust_cmd','std_msgs/Float32');
% sandwich_1_left_msg = rosmessage(sandwich_1_left_pub);
% sandwich_1_right_msg = rosmessage(sandwich_1_right_pub);

%%

while true
tftree = rostf;
frames = tftree.AvailableFrames;
tag_index = find(not(cellfun('isempty',strfind(frames,'april_tag'))));

% boat_index = find(not(cellfun('isempty',strfind(frames,'boat_tag'))));

% --------------------- AprilCube Following ----------------------------
if isempty(tag_index) == 1
    tags = {};
else
    clear tag_tf
    clear tags
   for i=1:numel(tag_index)
        tags{i} = frames{tag_index(i)};
   end
    for j=1:numel(tags)
         tag_tf(j) = getTransform(tftree,'sandwich_0/base_link',tags{j},'Timeout',inf);
    end
[apriltagID,aprildist,aprilhead] = aprilTag_xform(tag_tf)
[aprilleft,aprilright] = aprilTag_tagfollow(aprildist,aprilhead);
 sandwich_0_left_msg.Data = aprilleft;
 sandwich_0_right_msg.Data = aprilright;
send(sandwich_0_left_pub, sandwich_0_left_msg);
send(sandwich_0_right_pub, sandwich_0_right_msg);
end

% -------------------- Boat Following -----------------------------
% if isempty(boat_index) == 1
%     boat_tags = {};
% else
%     clear boat_tag_tf
%    for i=1:numel(boat_index)
%         boat_tags{i} = frames{boat_index(i)};
%    end
%     for j=1:numel(boat_tags)
%          boat_tag_tf(j) = getTransform(tftree,'sandwich_1/base_link',boat_tags{j},'Timeout',inf);
%     end
% [boattagID,boatdist,boathead] = aprilTag_xform(boat_tag_tf)
% [boatleft,boatright] = aprilTag_boatfollow(boatdist,boathead);
%  sandwich_1_left_msg.Data = boatleft;
%  sandwich_1_right_msg.Data = boatright;
% send(sandwich_1_left_pub, sandwich_1_left_msg);
% send(sandwich_1_right_pub, sandwich_1_right_msg);
% 
% 
% end

% waitfor(rate);
end