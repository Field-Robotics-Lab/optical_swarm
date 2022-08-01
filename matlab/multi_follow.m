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

% ----------------------- Boat 0 -------------------------
sand0_front_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/front_right_camera_link_optical','Timeout',inf);  
sand0_front_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/front_left_camera_link_optical','Timeout',inf); 
sand0_side_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/side_right_camera_link_optical','Timeout',inf); 
sand0_side_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/side_left_camera_link_optical','Timeout',inf); 
sand0_rear_right_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/rear_right_camera_link_optical','Timeout',inf); 
sand0_rear_left_xform = getTransform(tftree,'sandwich_0/base_link','sandwich_0/rear_left_camera_link_optical','Timeout',inf);    

sand0_front_right = receive(sand0_front_right_sub,inf);
sand0_front_left = receive(sand0_front_left_sub,inf);
sand0_side_right = receive(sand0_side_right_sub,inf);
sand0_side_left = receive(sand0_side_left_sub,inf);
sand0_rear_right = receive(sand0_rear_right_sub,inf);
sand0_rear_left = receive(sand0_rear_left_sub,inf);

[sand0_ID_fr,sand0_pose_fr] = tag_detect(sand0_front_right);
[sand0_ID_fl,sand0_pose_fl] = tag_detect(sand0_front_left);
[sand0_ID_sr,sand0_pose_sr] = tag_detect(sand0_side_right);
[sand0_ID_sl,sand0_pose_sl] = tag_detect(sand0_side_left);
[sand0_ID_rr,sand0_pose_rr] = tag_detect(sand0_rear_right);
[sand0_ID_rl,sand0_pose_rl] = tag_detect(sand0_rear_left);

[sand0_tagID_fr,sand0_dist_fr,sand0_head_fr] = tag_disthead(sand0_ID_fr,sand0_pose_fr,sand0_front_right_xform);
[sand0_tagID_fl,sand0_dist_fl,sand0_head_fl] = tag_disthead(sand0_ID_fl,sand0_pose_fl,sand0_front_left_xform);
[sand0_tagID_sr,sand0_dist_sr,sand0_head_sr] = tag_disthead(sand0_ID_sr,sand0_pose_sr,sand0_side_right_xform);
[sand0_tagID_sl,sand0_dist_sl,sand0_head_sl] = tag_disthead(sand0_ID_sl,sand0_pose_sl,sand0_side_left_xform);
[sand0_tagID_rr,sand0_dist_rr,sand0_head_rr] = tag_disthead(sand0_ID_rr,sand0_pose_rr,sand0_rear_right_xform);
[sand0_tagID_rl,sand0_dist_rl,sand0_head_rl] = tag_disthead(sand0_ID_rl,sand0_pose_rl,sand0_rear_left_xform);

sand0_tagID = [sand0_tagID_fr, sand0_tagID_fl, sand0_tagID_sr, ...
         sand0_tagID_sl, sand0_tagID_rr, sand0_tagID_rl]
sand0_dist = [sand0_dist_fr, sand0_dist_fl, sand0_dist_sr,...
        sand0_dist_sl, sand0_dist_rr, sand0_dist_rl]
sand0_head = [sand0_head_fr, sand0_head_fl, sand0_head_sr,...
        sand0_head_sl, sand0_head_rr, sand0_head_rl]


[sand0_left sand0_right] = aprilTag_tagfollow(sand0_tagID, sand0_dist, sand0_head)
% -------------------- Boat 1 -------------------------
% sand1_front_right_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/front_right_camera_link_optical','Timeout',inf);  
% sand1_front_left_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/front_left_camera_link_optical','Timeout',inf); 
% sand1_side_right_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/side_right_camera_link_optical','Timeout',inf); 
% sand1_side_left_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/side_left_camera_link_optical','Timeout',inf); 
% sand1_rear_right_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/rear_right_camera_link_optical','Timeout',inf); 
% sand1_rear_left_xform = getTransform(tftree,'sandwich_1/base_link','sandwich_1/rear_left_camera_link_optical','Timeout',inf);    
% 
% sand1_front_right = receive(sand1_front_right_sub,inf);
% sand1_front_left = receive(sand1_front_left_sub,inf);
% sand1_side_right = receive(sand1_side_right_sub,inf);
% sand1_side_left = receive(sand1_side_left_sub,inf);
% sand1_rear_right = receive(sand1_rear_right_sub,inf);
% sand1_rear_left = receive(sand1_rear_left_sub,inf);
% 
% [sand1_ID_fr,sand1_pose_fr] = tag_detect(sand1_front_right);
% [sand1_ID_fl,sand1_pose_fl] = tag_detect(sand1_front_left);
% [sand1_ID_sr,sand1_pose_sr] = tag_detect(sand1_side_right);
% [sand1_ID_sl,sand1_pose_sl] = tag_detect(sand1_side_left);
% [sand1_ID_rr,sand1_pose_rr] = tag_detect(sand1_rear_right);
% [sand1_ID_rl,sand1_pose_rl] = tag_detect(sand1_rear_left);
% 
% [sand1_tagID_fr,sand1_dist_fr,sand1_head_fr] = tag_disthead(sand1_ID_fr,sand1_pose_fr,sand1_front_right_xform);
% [sand1_tagID_fl,sand1_dist_fl,sand1_head_fl] = tag_disthead(sand1_ID_fl,sand1_pose_fl,sand1_front_left_xform);
% [sand1_tagID_sr,sand1_dist_sr,sand1_head_sr] = tag_disthead(sand1_ID_sr,sand1_pose_sr,sand1_side_right_xform);
% [sand1_tagID_sl,sand1_dist_sl,sand1_head_sl] = tag_disthead(sand1_ID_sl,sand1_pose_sl,sand1_side_left_xform);
% [sand1_tagID_rr,sand1_dist_rr,sand1_head_rr] = tag_disthead(sand1_ID_rr,sand1_pose_rr,sand1_rear_right_xform);
% [sand1_tagID_rl,sand1_dist_rl,sand1_head_rl] = tag_disthead(sand1_ID_rl,sand1_pose_rl,sand1_rear_left_xform);
% 
% sand1_tagID = [sand1_tagID_fr, sand1_tagID_fl, sand1_tagID_sr, ...
%          sand1_tagID_sl, sand1_tagID_rr, sand1_tagID_rl]
% sand1_dist = [sand1_dist_fr, sand1_dist_fl, sand1_dist_sr,...
%         sand1_dist_sl, sand1_dist_rr, sand1_dist_rl]
% sand1_head = [sand1_head_fr, sand1_head_fl, sand1_head_sr,...
%         sand1_head_sl, sand1_head_rr, sand1_head_rl]

% ------------------------ Boat 2 -------------------------
% sand2_front_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/front_right_camera_link_optical','Timeout',inf);  
% sand2_front_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/front_left_camera_link_optical','Timeout',inf); 
% sand2_side_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/side_right_camera_link_optical','Timeout',inf); 
% sand2_side_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/side_left_camera_link_optical','Timeout',inf); 
% sand2_rear_right_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/rear_right_camera_link_optical','Timeout',inf); 
% sand2_rear_left_xform = getTransform(tftree,'sandwich_2/base_link','sandwich_2/rear_left_camera_link_optical','Timeout',inf);    
% 
% sand2_front_right = receive(sand2_front_right_sub,inf);
% sand2_front_left = receive(sand2_front_left_sub,inf);
% sand2_side_right = receive(sand2_side_right_sub,inf);
% sand2_side_left = receive(sand2_side_left_sub,inf);
% sand2_rear_right = receive(sand2_rear_right_sub,inf);
% sand2_rear_left = receive(sand2_rear_left_sub,inf);
% 
% [sand2_ID_fr,sand2_pose_fr] = tag_detect(sand2_front_right);
% [sand2_ID_fl,sand2_pose_fl] = tag_detect(sand2_front_left);
% [sand2_ID_sr,sand2_pose_sr] = tag_detect(sand2_side_right);
% [sand2_ID_sl,sand2_pose_sl] = tag_detect(sand2_side_left);
% [sand2_ID_rr,sand2_pose_rr] = tag_detect(sand2_rear_right);
% [sand2_ID_rl,sand2_pose_rl] = tag_detect(sand2_rear_left);
% 
% 
% 
% [sand2_tagID_fr,sand2_dist_fr,sand2_head_fr] = tag_disthead(sand2_ID_fr,sand2_pose_fr,sand2_front_right_xform);
% [sand2_tagID_fl,sand2_dist_fl,sand2_head_fl] = tag_disthead(sand2_ID_fl,sand2_pose_fl,sand2_front_left_xform);
% [sand2_tagID_sr,sand2_dist_sr,sand2_head_sr] = tag_disthead(sand2_ID_sr,sand2_pose_sr,sand2_side_right_xform);
% [sand2_tagID_sl,sand2_dist_sl,sand2_head_sl] = tag_disthead(sand2_ID_sl,sand2_pose_sl,sand2_side_left_xform);
% [sand2_tagID_rr,sand2_dist_rr,sand2_head_rr] = tag_disthead(sand2_ID_rr,sand2_pose_rr,sand2_rear_right_xform);
% [sand2_tagID_rl,sand2_dist_rl,sand2_head_rl] = tag_disthead(sand2_ID_rl,sand2_pose_rl,sand2_rear_left_xform);
% 
% sand2_tagID = [sand2_tagID_fr, sand2_tagID_fl, sand2_tagID_sr, ...
%          sand2_tagID_sl, sand2_tagID_rr, sand2_tagID_rl]
% sand2_dist = [sand2_dist_fr, sand2_dist_fl, sand2_dist_sr,...
%         sand2_dist_sl, sand2_dist_rr, sand2_dist_rl]
% sand2_head = [sand2_head_fr, sand2_head_fl, sand2_head_sr,...
%         sand2_head_sl, sand2_head_rr, sand2_head_rl]


waitfor(rate);
end
