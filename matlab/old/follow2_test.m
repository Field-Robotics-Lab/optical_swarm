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

sand1_nav_sub = rossubscriber('/robot1/sandwich_1/sensors/p3d','nav_msgs/Odometry','dataformat','struct');
rabbit_sub = rossubscriber('/april_cube/rabbit','DataFormat','struct');

% ------------------- Boat 2 ---------------------------------------------

sand2_front_right_sub = rossubscriber('/robot2/sandwich_2/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_front_left_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_side_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_side_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_rear_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_rear_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');


% ---------- Thrust Command Publishers ----------------------------

sandwich_0_cmd_pub = rospublisher('/sandwich_0/cmd_vel','geometry_msgs/Twist');
sandwich_0_cmd_msg = rosmessage(sandwich_0_cmd_pub);

sandwich_1_cmd_pub = rospublisher('/sandwich_1/cmd_vel','geometry_msgs/Twist');
sandwich_1_cmd_msg = rosmessage(sandwich_1_cmd_pub);

sandwich_2_cmd_pub = rospublisher('/sandwich_2/cmd_vel','geometry_msgs/Twist');
sandwich_2_cmd_msg = rosmessage(sandwich_2_cmd_pub);


% -------------------- Camera to Base Link Transforms ---------------------
tftree = rostf;

%---------------------- Boat 0 ----------------------------
sand0_front_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/front_right_camera_link_optical','Timeout',inf);  
sand0_front_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/front_left_camera_link_optical','Timeout',inf); 
sand0_side_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/side_right_camera_link_optical','Timeout',inf); 
sand0_side_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/side_left_camera_link_optical','Timeout',inf); 
sand0_rear_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/rear_right_camera_link_optical','Timeout',inf); 
sand0_rear_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/rear_left_camera_link_optical','Timeout',inf); 



% ------------------------ Boat 2 -------------------------
sand2_front_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/front_right_camera_link_optical','Timeout',inf);  
sand2_front_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/front_left_camera_link_optical','Timeout',inf); 
sand2_side_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/side_right_camera_link_optical','Timeout',inf); 
sand2_side_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/side_left_camera_link_optical','Timeout',inf); 
sand2_rear_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/rear_right_camera_link_optical','Timeout',inf); 
sand2_rear_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/rear_left_camera_link_optical','Timeout',inf);    

% ----------------- IMU Subscriber -----------------------------------

% ----------------Initialize and set PI gains-----------------

head_offset0 = 0;
head_offset1 = 0;
head_offset2 = head_offset0;

iter = 0;
while true

%---------------------- Boat 0 ----------------------------
%--------------Receive Tag Detection Message -------------
iter
tic
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
% boat_range0 = [10,11,12,13];
               
target_range0 = [10,11,12,13];
boat_range0 = [20,21,22,23];

%----------- Calculate Speed and Rate Commands-----------------
[debug0,target_ID0, speed0, rate0, dist_target0] = vbap_test(sand0_tagID, sand0_dist, sand0_head,target_range0,boat_range0,head_offset0);

% speed0
% rate0


if isempty(debug0) == 0
     %---------------------- Turn Rate Controller ----------------------------
        sandwich_0_cmd_msg.Linear.X = speed0;
    sandwich_0_cmd_msg.Angular.Z = deg2rad(rate0);
%     send(sandwich_0_cmd_pub, sandwich_0_cmd_msg);
else
%     sandwich_0_left_msg.Data = 0;
%     sandwich_0_right_msg.Data = 0;
%     send(sandwich_0_left_pub, sandwich_0_left_msg);
%     send(sandwich_0_right_pub, sandwich_0_right_msg);
end


% -------------------- Boat 1 -------------------------

%--------------Receive Tag Detection Message -------------

sand1 = receive(sand1_nav_sub,inf);
rabbit = receive(rabbit_sub,inf);

%----------- Calculate Speed and Rate Commands-----------------
[speed1, rate1, dist_target1,head1] = rabbit_follow(sand1,rabbit);

    
    sandwich_1_cmd_msg.Linear.X = speed1;
    sandwich_1_cmd_msg.Angular.Z = deg2rad(rate1);
    send(sandwich_1_cmd_pub, sandwich_1_cmd_msg);
  

% -------------------- Boat 2 -------------------------

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
% boat_range2 = [ 0, 1, 2, 3];

target_range2 = [10,11,12,13];
boat_range2 = [0, 1, 2, 3];
                
%----------- Calculate Speed and Rate Commands-----------------
[debug2,target_ID2, speed2, rate2, dist_target2] = vbap_test(sand2_tagID, sand2_dist, sand2_head,target_range2,boat_range2,head_offset2);
speed2
rate2


if isempty(debug2) == 0
%---------------------- Turn Rate Controller ----------------------------
        sandwich_2_cmd_msg.Linear.X = speed2;
    sandwich_2_cmd_msg.Angular.Z = deg2rad(rate2);
    send(sandwich_2_cmd_pub, sandwich_2_cmd_msg);
else
%     sandwich_2_left_msg.Data = 0;
%     sandwich_2_right_msg.Data = 0;
%     send(sandwich_2_left_pub, sandwich_2_left_msg);
%     send(sandwich_2_right_pub, sandwich_2_right_msg);
end

iter = iter +1;
toc
% waitfor(rate);
end
