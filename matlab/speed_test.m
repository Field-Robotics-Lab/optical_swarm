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


sand2_nav_sub = rossubscriber('/robot2/sandwich_2/sensors/p3d','nav_msgs/Odometry','dataformat','struct');
sand2_imu_sub = rossubscriber('/robot2/sandwich_2/sensors/imu/imu/data');

% ---------- Thrust Command Publishers ----------------------------

sandwich_2_left_pub = rospublisher('/sandwich_2/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_2_right_pub = rospublisher('/sandwich_2/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_2_left_msg = rosmessage(sandwich_2_left_pub);
sandwich_2_right_msg = rosmessage(sandwich_2_right_pub);


% figure(1)
% grid on
% hold on
% axis([-1000 -600 -500 -300])
% title('Position')

left_thrust = 1;
% right_thrust = left_thrust; 
right_thrust = 0; 

% figure(2)
% grid on
% hold on
% axis([0 25 0 15])
% title(sprintf('Thrust %0g',left_thrust))
% ylabel('Speed')
% xlabel('Time [sec]')

figure(3)
grid on
hold on
axis([0 25 -25 25])
title(sprintf('Fwd Turn, delta thrust = %0g',abs(left_thrust - right_thrust)))
ylabel('Turn Rate [deg/s]')
xlabel('Time [sec]')

iter = 0;
while true

%---------------------- Boat 0 ----------------------------
 sand2_nav = receive(sand2_nav_sub,inf);
 sand2_X = sand2_nav.Pose.Pose.Position.X;
 sand2_Y = sand2_nav.Pose.Pose.Position.Y;
%  figure(1)
%  plot(sand2_X,sand2_Y,'b*','MarkerSize',4)
    
%  sand2_U = sand2_nav.Twist.Twist.Linear.X;
%  sand2_V = sand2_nav.Twist.Twist.Linear.Y;
%  sand2_speed = norm([sand2_U sand2_V])
% figure(2)
%     plot(iter/10,sand2_speed,'r*','MarkerSize',4)

imu = receive(sand2_imu_sub);
turn_rate = rad2deg(imu.AngularVelocity.Z)
figure(3)
plot(iter/10,turn_rate,'r*','MarkerSize',4)

if iter < 150
 sandwich_2_left_msg.Data = left_thrust;
 sandwich_2_right_msg.Data = right_thrust;
 send(sandwich_2_left_pub, sandwich_2_left_msg);
 send(sandwich_2_right_pub, sandwich_2_right_msg);
else
sandwich_2_left_msg.Data = 0;
 sandwich_2_right_msg.Data = 0;
 send(sandwich_2_left_pub, sandwich_2_left_msg);
 send(sandwich_2_right_pub, sandwich_2_right_msg);
end

iter = iter+1;
waitfor(rate);
end

% 
% figure(3)
% yline(turn_rate)
% legend(sprintf('Turn Rate = %0.2f deg/s',turn_rate),'Location','best')

