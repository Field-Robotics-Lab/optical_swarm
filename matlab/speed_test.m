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

sandwich_2_cmd_pub = rospublisher('/sandwich_2/cmd_vel','geometry_msgs/Twist');
sandwich_2_cmd_msg = rosmessage(sandwich_2_cmd_pub);



% figure(1)
% grid on
% hold on
% axis([-1000 -600 -500 -300])
% title('Position')

speed2 = 10;
rate2 = 0;

figure(2)
grid on
hold on
axis([0 25 0 15])
title(sprintf('Speed %0g',speed2))
ylabel('Speed')
xlabel('Time [sec]')

figure(3)
grid on
hold on
axis([0 25 -40 40])
title(sprintf('Fwd Turn, rate = %0g',rate2))
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
    
 sand2_U = sand2_nav.Twist.Twist.Linear.X;
 sand2_V = sand2_nav.Twist.Twist.Linear.Y;
 sand2_speed = norm([sand2_U sand2_V])
figure(2)
    plot(iter/10,sand2_speed,'r*','MarkerSize',4)

imu = receive(sand2_imu_sub);
turn_rate = rad2deg(imu.AngularVelocity.Z);
figure(3)
plot(iter/10,turn_rate,'r*','MarkerSize',4)

if iter < 100
    sandwich_2_cmd_msg.Linear.X = speed2;
    sandwich_2_cmd_msg.Angular.Z = deg2rad(rate2);
    send(sandwich_2_cmd_pub, sandwich_2_cmd_msg);
else
sandwich_2_cmd_msg.Linear.X = speed2;
    sandwich_2_cmd_msg.Angular.Z = 0;
    send(sandwich_2_cmd_pub, sandwich_2_cmd_msg);
end

iter = iter+1;
waitfor(rate);
end

% 
% figure(3)
% yline(turn_rate)
% legend(sprintf('Turn Rate = %0.2f deg/s',turn_rate),'Location','best')

