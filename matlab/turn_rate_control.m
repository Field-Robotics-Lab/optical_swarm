% Heading Rate Controller Test

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

%---------------------- Boat 0 ----------------------------


sand0_nav_sub = rossubscriber('/robot0/sandwich_0/sensors/p3d','nav_msgs/Odometry','dataformat','struct');
sand0_imu_sub = rossubscriber('/robot0/sandwich_0/sensors/imu/imu/data');

% ---------- Thrust Command Publishers ----------------------------

sandwich_0_left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_0_right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_0_left_msg = rosmessage(sandwich_0_left_pub);
sandwich_0_right_msg = rosmessage(sandwich_0_right_pub);


figure(3)
grid on
hold on
axis([0 200 -15 15])
title('Turn Rate Control')
ylabel('Turn Rate [deg/s]')



iter = 0;

% ----------------Initialize and set PI gains-----------------
rate_err_cum = 0;
rate_err_old = 0;
rate_err1 = 0;
rate_cmd = 12;
thrust = 0;
max_thrust = 1;
% Gains
K_p = 1;
K_i = 0.1;


while true


    
%---------------------- Turn Rate Controller ----------------------------
imu = receive(sand0_imu_sub);
turn_rate = rad2deg(imu.AngularVelocity.Z);
rate_err1 = (rate_cmd - turn_rate);

rate_err_cum = rate_err_cum + 0.5*(rate_err1 + rate_err_old)*0.1;

thrust = K_p*rate_err1 + K_i*rate_err_cum;
left_thrust = -min(max_thrust, max(-max_thrust,thrust))
right_thrust = min(max_thrust, max(-max_thrust,thrust))

% Update old error
 rate_err_old = rate_err1;

% Publish thrust commands

 sandwich_0_left_msg.Data = fwd+left_thrust;
 sandwich_0_right_msg.Data = fwd+right_thrust;
 send(sandwich_0_left_pub, sandwich_0_left_msg);
 send(sandwich_0_right_pub, sandwich_0_right_msg);





waitfor(rate);
end


