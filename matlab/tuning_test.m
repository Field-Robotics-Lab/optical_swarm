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


% -------------------- Boat 1 ---------------------------------------

sand1_nav_sub = rossubscriber('/robot1/sandwich_1/sensors/p3d','nav_msgs/Odometry','dataformat','struct');
rabbit_sub = rossubscriber('/april_cube/rabbit','DataFormat','struct');
sand1_imu_sub = rossubscriber('/robot1/sandwich_1/sensors/imu/imu/data');



% ---------- Thrust Command Publishers ----------------------------

sandwich_1_cmd_pub = rospublisher('/sandwich_1/cmd_vel','geometry_msgs/Twist');
sandwich_1_cmd_msg = rosmessage(sandwich_1_cmd_pub);








while true


% -------------------- Boat 1 -------------------------

sand1 = receive(sand1_nav_sub,inf);
rabbit = receive(rabbit_sub,inf);

%----------- Calculate Speed and Rate Commands-----------------
[speed1, rate1, dist_target1,head1] = rabbit_follow(sand1,rabbit)

    
    sandwich_1_cmd_msg.Linear.X = speed1;
    sandwich_1_cmd_msg.Angular.Z = rate1;
    send(sandwich_1_cmd_pub, sandwich_1_cmd_msg);
  

waitfor(rate);
end
