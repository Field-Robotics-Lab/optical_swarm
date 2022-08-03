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


tag_tf = rossubscriber('/tf','DataFormat','struct');


% Setup Publisher
% cmd_pub = rospublisher('/sandwich_0/cmd_vel','geometry_msgs/Twist');
% cmd_msg = rosmessage(cmd_pub);

left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
left_msg = rosmessage(left_pub);
right_msg = rosmessage(right_pub);

while true
tag = receive(tag_tf);
trans = tag.Transforms.Transform.Translation;
rot = tag.Transforms.Transform.Rotation;

[left,right] = aprilTag_thrustcmd(trans,rot);
 left_msg.Data = left;
 right_msg.Data = right;
send(left_pub, left_msg);
send(right_pub, right_msg);

% [v_c,r_c] = aprilTag2cmd_vel(trans);
% cmd_msg.Linear.X = v_c
% cmd_msg.Angular.Z = r_c
% send(cmd_pub, cmd_msg);

waitfor(rate);
end

