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
cmd_pub = rospublisher('/robot0/sandwich_0/cmd_vel','geometry_msgs/Twist');
cmd_msg = rosmessage(cmd_pub);
left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
left_msg = rosmessage(left_pub);
right_msg = rosmessage(right_pub);

%%

while true
tftree = rostf;
frames = tftree.AvailableFrames;
tag_index = find(not(cellfun('isempty',strfind(frames,'april_tag'))));
if isempty(tag_index) == 1
    tags = {};
else
    clear tag_tf
   for i=1:numel(tag_index)
        tags{i} = frames{tag_index(i)};
   end
    for j=1:numel(tags)
         tag_tf(j) = getTransform(tftree,'sandwich_0/base_link',tags{j},'Timeout',inf);
    end
[tagID,dist,head] = aprilTag_xform(tag_tf)
[left,right] = aprilTag_thrustcmd(dist,head);
 left_msg.Data = left;
 right_msg.Data = right;
send(left_pub, left_msg);
send(right_pub, right_msg);
end



waitfor(rate);
end