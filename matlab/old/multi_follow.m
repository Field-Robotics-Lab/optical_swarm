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

sand0_front_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand0_front_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand0_side_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand0_side_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand0_rear_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand0_rear_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');

% -------------------- Boat 1 ---------------------------------------

sand1_front_right_sub = rossubscriber('/robot1/sandwich_1/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_front_left_sub =  rossubscriber('/robot1/sandwich_1/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_side_right_sub =  rossubscriber('/robot1/sandwich_1/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_side_left_sub =   rossubscriber('/robot1/sandwich_1/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_rear_right_sub =  rossubscriber('/robot1/sandwich_1/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand1_rear_left_sub =   rossubscriber('/robot1/sandwich_1/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');

% % ------------------- Boat 2 ---------------------------------------------

sand2_front_right_sub = rossubscriber('/robot2/sandwich_2/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_front_left_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_side_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_side_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_rear_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_rear_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');

% ----------------P3D Subscribers-----------------------

% sand0_nav_sub = rossubscriber('/robot0/sandwich_0/sensors/p3d','nav_msgs/Odometry','dataformat','struct');
% sand1_nav_sub = rossubscriber('/robot1/sandwich_1/sensors/p3d','nav_msgs/Odometry','dataformat','struct');
% sand2_nav_sub = rossubscriber('/robot2/sandwich_2/sensors/p3d','nav_msgs/Odometry','dataformat','struct');

% ---------- Thrust Command Publishers ----------------------------

sandwich_0_left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_0_right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_0_left_msg = rosmessage(sandwich_0_left_pub);
sandwich_0_right_msg = rosmessage(sandwich_0_right_pub);

sandwich_1_left_pub = rospublisher('/sandwich_1/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_1_right_pub = rospublisher('/sandwich_1/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_1_left_msg = rosmessage(sandwich_1_left_pub);
sandwich_1_right_msg = rosmessage(sandwich_1_right_pub);

sandwich_2_left_pub = rospublisher('/sandwich_2/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_2_right_pub = rospublisher('/sandwich_2/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_2_left_msg = rosmessage(sandwich_2_left_pub);
sandwich_2_right_msg = rosmessage(sandwich_2_right_pub);




% -------------------- Camera to Base Link Transforms ---------------------
tftree = rostf;

%---------------------- Boat 0 ----------------------------
sand0_front_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/front_right_camera_link_optical','Timeout',inf);  
sand0_front_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/front_left_camera_link_optical','Timeout',inf); 
sand0_side_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/side_right_camera_link_optical','Timeout',inf); 
sand0_side_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/side_left_camera_link_optical','Timeout',inf); 
sand0_rear_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/rear_right_camera_link_optical','Timeout',inf); 
sand0_rear_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/rear_left_camera_link_optical','Timeout',inf); 

% -------------------- Boat 1 -------------------------
sand1_front_right_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/front_right_camera_link_optical','Timeout',inf);  
sand1_front_left_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/front_left_camera_link_optical','Timeout',inf); 
sand1_side_right_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/side_right_camera_link_optical','Timeout',inf); 
sand1_side_left_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/side_left_camera_link_optical','Timeout',inf); 
sand1_rear_right_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/rear_right_camera_link_optical','Timeout',inf); 
sand1_rear_left_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/rear_left_camera_link_optical','Timeout',inf);    


% ------------------------ Boat 2 -------------------------
sand2_front_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/front_right_camera_link_optical','Timeout',inf);  
sand2_front_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/front_left_camera_link_optical','Timeout',inf); 
sand2_side_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/side_right_camera_link_optical','Timeout',inf); 
sand2_side_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/side_left_camera_link_optical','Timeout',inf); 
sand2_rear_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/rear_right_camera_link_optical','Timeout',inf); 
sand2_rear_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/rear_left_camera_link_optical','Timeout',inf);    


% figure(1)
% grid on
% hold on
% axis([-700 -300 150 400])

while true

%---------------------- Boat 0 ----------------------------
% sand0_nav = receive(sand0_nav_sub,inf);
% sand0_X = sand0_nav.Pose.Pose.Position.X;
% sand0_Y = sand0_nav.Pose.Pose.Position.Y;
% % figure(1)
% plot(sand0_X,sand0_Y,'b*','MarkerSize',2)

%--------------Receive Tag Detection Message -------------
sand0_front_right = receive(sand0_front_right_sub,inf);
sand0_front_left = receive(sand0_front_left_sub,inf);
sand0_side_right = receive(sand0_side_right_sub,inf);
sand0_side_left = receive(sand0_side_left_sub,inf);
sand0_rear_right = receive(sand0_rear_right_sub,inf);
sand0_rear_left = receive(sand0_rear_left_sub,inf);

%------------- Extract  all ID and XYZ (Camera Frame) ------------
[sand0_ID_fr,sand0_pose_fr] = tag_detect(sand0_front_right);
[sand0_ID_fl,sand0_pose_fl] = tag_detect(sand0_front_left);
[sand0_ID_sr,sand0_pose_sr] = tag_detect(sand0_side_right);
[sand0_ID_sl,sand0_pose_sl] = tag_detect(sand0_side_left);
[sand0_ID_rr,sand0_pose_rr] = tag_detect(sand0_rear_right);
[sand0_ID_rl,sand0_pose_rl] = tag_detect(sand0_rear_left);

%------------- Convert to Distance/Heading (Body Frame) ------------------
[sand0_tagID_fr,sand0_dist_fr,sand0_head_fr] = tag_disthead(sand0_ID_fr,sand0_pose_fr,sand0_front_right_xform);
[sand0_tagID_fl,sand0_dist_fl,sand0_head_fl] = tag_disthead(sand0_ID_fl,sand0_pose_fl,sand0_front_left_xform);
[sand0_tagID_sr,sand0_dist_sr,sand0_head_sr] = tag_disthead(sand0_ID_sr,sand0_pose_sr,sand0_side_right_xform);
[sand0_tagID_sl,sand0_dist_sl,sand0_head_sl] = tag_disthead(sand0_ID_sl,sand0_pose_sl,sand0_side_left_xform);
[sand0_tagID_rr,sand0_dist_rr,sand0_head_rr] = tag_disthead(sand0_ID_rr,sand0_pose_rr,sand0_rear_right_xform);
[sand0_tagID_rl,sand0_dist_rl,sand0_head_rl] = tag_disthead(sand0_ID_rl,sand0_pose_rl,sand0_rear_left_xform);

%-------------- Consolidate Camera Outputs -----------------------
sand0_tagID = [sand0_tagID_fr, sand0_tagID_fl, sand0_tagID_sr, ...
         sand0_tagID_sl, sand0_tagID_rr, sand0_tagID_rl];
sand0_dist = [sand0_dist_fr, sand0_dist_fl, sand0_dist_sr,...
        sand0_dist_sl, sand0_dist_rr, sand0_dist_rl];
sand0_head = [sand0_head_fr, sand0_head_fl, sand0_head_sr,...
        sand0_head_sl, sand0_head_rr, sand0_head_rl];

% Specify Range of Target and Boat IDs
% target_range0 = [90,91,92,93,94,95];
% boat_range0 = [10,11,12,13;
%                20,21,22,23];
target_range0 = [10,11,12,13];
boat_range0 = [20,21,22,23];
%----------- Calculate and Publish Thrust Commands-----------------
[debug0,target_ID0, sand0_left, sand0_right] = aprilTag_vbap(sand0_tagID, sand0_dist, sand0_head,target_range0,boat_range0)

if isempty(debug0) == 0
    sandwich_0_left_msg.Data = sand0_left;
    sandwich_0_right_msg.Data = sand0_right;
    send(sandwich_0_left_pub, sandwich_0_left_msg);
    send(sandwich_0_right_pub, sandwich_0_right_msg);
else
%     sandwich_0_left_msg.Data = 0;
%     sandwich_0_right_msg.Data = 0;
%     send(sandwich_0_left_pub, sandwich_0_left_msg);
%     send(sandwich_0_right_pub, sandwich_0_right_msg);
end

% -------------------- Boat 1 -------------------------

% sand1_nav = receive(sand1_nav_sub,inf);
% sand1_X = sand1_nav.Pose.Pose.Position.X;
% sand1_Y = sand1_nav.Pose.Pose.Position.Y;
% % figure(1)
% plot(sand1_X,sand1_Y,'r*','MarkerSize',2)

%--------------Receive Tag Detection Message -------------
sand1_front_right = receive(sand1_front_right_sub,inf);
sand1_front_left = receive(sand1_front_left_sub,inf);
sand1_side_right = receive(sand1_side_right_sub,inf);
sand1_side_left = receive(sand1_side_left_sub,inf);
sand1_rear_right = receive(sand1_rear_right_sub,inf);
sand1_rear_left = receive(sand1_rear_left_sub,inf);

%------------- Extract  all ID and XYZ (Camera Frame) ------------
[sand1_ID_fr,sand1_pose_fr] = tag_detect(sand1_front_right);
[sand1_ID_fl,sand1_pose_fl] = tag_detect(sand1_front_left);
[sand1_ID_sr,sand1_pose_sr] = tag_detect(sand1_side_right);
[sand1_ID_sl,sand1_pose_sl] = tag_detect(sand1_side_left);
[sand1_ID_rr,sand1_pose_rr] = tag_detect(sand1_rear_right);
[sand1_ID_rl,sand1_pose_rl] = tag_detect(sand1_rear_left);

%------------- Convert to Distance/Heading (Body Frame) ------------------
[sand1_tagID_fr,sand1_dist_fr,sand1_head_fr] = tag_disthead(sand1_ID_fr,sand1_pose_fr,sand1_front_right_xform);
[sand1_tagID_fl,sand1_dist_fl,sand1_head_fl] = tag_disthead(sand1_ID_fl,sand1_pose_fl,sand1_front_left_xform);
[sand1_tagID_sr,sand1_dist_sr,sand1_head_sr] = tag_disthead(sand1_ID_sr,sand1_pose_sr,sand1_side_right_xform);
[sand1_tagID_sl,sand1_dist_sl,sand1_head_sl] = tag_disthead(sand1_ID_sl,sand1_pose_sl,sand1_side_left_xform);
[sand1_tagID_rr,sand1_dist_rr,sand1_head_rr] = tag_disthead(sand1_ID_rr,sand1_pose_rr,sand1_rear_right_xform);
[sand1_tagID_rl,sand1_dist_rl,sand1_head_rl] = tag_disthead(sand1_ID_rl,sand1_pose_rl,sand1_rear_left_xform);

%-------------- Consolidate Camera Outputs -----------------------
sand1_tagID = [sand1_tagID_fr, sand1_tagID_fl, sand1_tagID_sr, ...
         sand1_tagID_sl, sand1_tagID_rr, sand1_tagID_rl];
sand1_dist = [sand1_dist_fr, sand1_dist_fl, sand1_dist_sr,...
        sand1_dist_sl, sand1_dist_rr, sand1_dist_rl];
sand1_head = [sand1_head_fr, sand1_head_fl, sand1_head_sr,...
        sand1_head_sl, sand1_head_rr, sand1_head_rl];

% Specify Range of Target and Boat IDs
target_range1 = [90,91,92,93,94,95];
% boat_range1 = [ 0, 1, 2, 3;
%                20,21,22,23];
boat_range1 = [999999];
%----------- Calculate and Publish Thrust Commands-----------------

[debug1,target_ID1, sand1_left, sand1_right] = aprilTag_vbap(sand1_tagID, sand1_dist, sand1_head, target_range1,boat_range1)


if isempty(debug1) == 0

    sandwich_1_left_msg.Data = sand1_left;
    sandwich_1_right_msg.Data = sand1_right;
    send(sandwich_1_left_pub, sandwich_1_left_msg);
    send(sandwich_1_right_pub, sandwich_1_right_msg);
else
%     sandwich_1_left_msg.Data = 0;
%     sandwich_1_right_msg.Data = 0;
%     send(sandwich_1_left_pub, sandwich_1_left_msg);
%     send(sandwich_1_right_pub, sandwich_1_right_msg);
end

% ------------------------ Boat 2 -------------------------

% sand2_nav = receive(sand2_nav_sub,inf);
% sand2_X = sand2_nav.Pose.Pose.Position.X;
% sand2_Y = sand2_nav.Pose.Pose.Position.Y;
% % figure(1)
% plot(sand2_X,sand2_Y,'g*','MarkerSize',2)
%--------------Receive Tag Detection Message -------------
sand2_front_right = receive(sand2_front_right_sub,inf);
sand2_front_left = receive(sand2_front_left_sub,inf);
sand2_side_right = receive(sand2_side_right_sub,inf);
sand2_side_left = receive(sand2_side_left_sub,inf);
sand2_rear_right = receive(sand2_rear_right_sub,inf);
sand2_rear_left = receive(sand2_rear_left_sub,inf);

%------------- Extract  all ID and XYZ (Camera Frame) ------------
[sand2_ID_fr,sand2_pose_fr] = tag_detect(sand2_front_right);
[sand2_ID_fl,sand2_pose_fl] = tag_detect(sand2_front_left);
[sand2_ID_sr,sand2_pose_sr] = tag_detect(sand2_side_right);
[sand2_ID_sl,sand2_pose_sl] = tag_detect(sand2_side_left);
[sand2_ID_rr,sand2_pose_rr] = tag_detect(sand2_rear_right);
[sand2_ID_rl,sand2_pose_rl] = tag_detect(sand2_rear_left);

%------------- Convert to Distance/Heading (Body Frame) ------------------
[sand2_tagID_fr,sand2_dist_fr,sand2_head_fr] = tag_disthead(sand2_ID_fr,sand2_pose_fr,sand2_front_right_xform);
[sand2_tagID_fl,sand2_dist_fl,sand2_head_fl] = tag_disthead(sand2_ID_fl,sand2_pose_fl,sand2_front_left_xform);
[sand2_tagID_sr,sand2_dist_sr,sand2_head_sr] = tag_disthead(sand2_ID_sr,sand2_pose_sr,sand2_side_right_xform);
[sand2_tagID_sl,sand2_dist_sl,sand2_head_sl] = tag_disthead(sand2_ID_sl,sand2_pose_sl,sand2_side_left_xform);
[sand2_tagID_rr,sand2_dist_rr,sand2_head_rr] = tag_disthead(sand2_ID_rr,sand2_pose_rr,sand2_rear_right_xform);
[sand2_tagID_rl,sand2_dist_rl,sand2_head_rl] = tag_disthead(sand2_ID_rl,sand2_pose_rl,sand2_rear_left_xform);

%-------------- Consolidate Camera Outputs -----------------------
sand2_tagID = [sand2_tagID_fr, sand2_tagID_fl, sand2_tagID_sr, ...
         sand2_tagID_sl, sand2_tagID_rr, sand2_tagID_rl];
sand2_dist = [sand2_dist_fr, sand2_dist_fl, sand2_dist_sr,...
        sand2_dist_sl, sand2_dist_rr, sand2_dist_rl];
sand2_head = [sand2_head_fr, sand2_head_fl, sand2_head_sr,...
        sand2_head_sl, sand2_head_rr, sand2_head_rl];


% Specify Range of Target and Boat IDs
% target_range2 = [90,91,92,93,94,95];
% boat_range2 = [ 0, 1, 2, 3;
%                10,11,12,13];


target_range2 = [10,11,12,13];
boat_range2 = [ 0, 1, 2, 3];
%----------- Calculate and Publish Thrust Commands-----------------
[debug2,target_ID2, sand2_left, sand2_right] = aprilTag_vbap(sand2_tagID, sand2_dist, sand2_head,target_range2,boat_range2)

if isempty(target_ID2) == 0

    sandwich_2_left_msg.Data = sand2_left;
    sandwich_2_right_msg.Data = sand2_right;
    send(sandwich_2_left_pub, sandwich_2_left_msg);
    send(sandwich_2_right_pub, sandwich_2_right_msg);
else
% %     sandwich_2_left_msg.Data = 0;
% %     sandwich_2_right_msg.Data = 0;
% %     send(sandwich_2_left_pub, sandwich_2_left_msg);
% %     send(sandwich_2_right_pub, sandwich_2_right_msg);
end

waitfor(rate);
end