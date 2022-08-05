% Speed and Turn Rate data collection

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


figure(1)
grid on
hold on
axis([-1100 -700 200 600])
title('Position')

left_thrust = 1;
right_thrust = -1; 

% figure(2)
% grid on
% hold on
% axis([0 800 0 25])
% title(sprintf('Thrust %0g',thrust))
% ylabel('Speed')

figure(3)
grid on
hold on
axis([0 800 -30 30])
title(sprintf('Fwd Turn, delta thrust = %0g',abs(left_thrust - right_thrust)))
ylabel('Turn Rate [deg/s]')

iter = 0;
while true

%---------------------- Boat 0 ----------------------------
 sand0_nav = receive(sand0_nav_sub,inf);
 sand0_X = sand0_nav.Pose.Pose.Position.X;
 sand0_Y = sand0_nav.Pose.Pose.Position.Y;
 figure(1)
 plot(sand0_X,sand0_Y,'b*','MarkerSize',4)
    
%  sand0_U = sand0_nav.Twist.Twist.Linear.X;
%  sand0_V = sand0_nav.Twist.Twist.Linear.Y;
% %  sand0_speed = norm([sand0_U sand0_V])
% figure(2)
%     plot(iter,sand0_speed,'r*','MarkerSize',4)

imu = receive(sand0_imu_sub);
turn_rate = rad2deg(imu.AngularVelocity.Z);
figure(3)
plot(iter,turn_rate,'r*','MarkerSize',4)


 sandwich_0_left_msg.Data = left_thrust;
 sandwich_0_right_msg.Data = right_thrust;
 send(sandwich_0_left_pub, sandwich_0_left_msg);
 send(sandwich_0_right_pub, sandwich_0_right_msg);




iter = iter+1;
waitfor(rate);
end


figure(3)
yline(turn_rate)
legend(sprintf('Turn Rate = %0.2f deg/s',turn_rate),'Location','best')

