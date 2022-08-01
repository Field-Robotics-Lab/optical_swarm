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

% --------------- Tag Detection Subscribers -----------------------------

%---------------------- Boat 0 ----------------------------

% sand0_front_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand0_front_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand0_side_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand0_side_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand0_rear_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand0_rear_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');

% -------------------- Boat 1 ---------------------------------------

sand1_front_right_sub = rossubscriber('/robot1/sandwich_1/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_front_left_sub =  rossubscriber('/robot1/sandwich_1/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_side_right_sub =  rossubscriber('/robot1/sandwich_1/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_side_left_sub =   rossubscriber('/robot1/sandwich_1/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_rear_right_sub =  rossubscriber('/robot1/sandwich_1/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_rear_left_sub =   rossubscriber('/robot1/sandwich_1/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');

% ------------------- Boat 2 ---------------------------------------------

% sand2_front_right_sub = rossubscriber('/robot2/sandwich_2/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand2_front_left_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand2_side_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand2_side_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand2_rear_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% sand2_rear_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
% 




% ---------- Thrust Command Publishers ----------------------------

% sandwich_0_left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
% sandwich_0_right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
% sandwich_0_left_msg = rosmessage(sandwich_0_left_pub);
% sandwich_0_right_msg = rosmessage(sandwich_0_right_pub);
% 
% sandwich_1_left_pub = rospublisher('/sandwich_1/thrusters/left_thrust_cmd','std_msgs/Float32');
% sandwich_1_right_pub = rospublisher('/sandwich_1/thrusters/right_thrust_cmd','std_msgs/Float32');
% sandwich_1_left_msg = rosmessage(sandwich_1_left_pub);
% sandwich_1_right_msg = rosmessage(sandwich_1_right_pub);
% 
% sandwich_2_left_pub = rospublisher('/sandwich_2/thrusters/left_thrust_cmd','std_msgs/Float32');
% sandwich_2_right_pub = rospublisher('/sandwich_2/thrusters/right_thrust_cmd','std_msgs/Float32');
% sandwich_2_left_msg = rosmessage(sandwich_2_left_pub);
% sandwich_2_right_msg = rosmessage(sandwich_2_right_pub);


%%

while true
    tftree = rostf;

sand1_side_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/side_left_camera_link_optical','Timeout',inf);

sand1_side_left = receive(sand1_side_left_sub,inf);
[ID,pose] = tag_detect(sand1_side_left);
[tagID,dist,head] = tag_disthead(ID,pose,sand1_side_xform)

waitfor(rate);
end
