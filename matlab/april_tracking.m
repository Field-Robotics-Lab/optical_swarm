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
cmd_pub = rospublisher('/sandwich_0/cmd_vel','geometry_msgs/Twist');
cmd_msg = rosmessage(cmd_pub);


while true
tag = receive(tag_tf,3);
trans = tag.Transforms.Transform.Translation;
rot = tag.Transforms.Transform.Rotation;
[v_c,r_c] = aprilTag2cmd_vel(trans);
cmd_msg.Linear.X = v_c
cmd_msg.Angular.Z = r_c
send(cmd_pub, cmd_msg);
waitfor(rate);
end

