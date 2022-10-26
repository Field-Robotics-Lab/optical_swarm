close all
clear

addpath('callbacks')
% Declare global variables for tag detection
global sand0_front_right; 
global sand0_front_left; 
global sand0_side_right; 
global sand0_side_left; 
global sand0_rear_right;
global sand0_rear_left; 

global sand2_front_right; 
global sand2_front_left; 
global sand2_side_right; 
global sand2_side_left; 
global sand2_rear_right; 
global sand2_rear_left;



try
    rosinit
catch
    rosshutdown
    rosinit
end



% --------------- Tag Detection Subscribers -----------------------------

%---------------------- Boat 0 ----------------------------

sand0_front_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand0_front_right_callback,'DataFormat','struct');
sand0_front_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand0_front_left_callback,'DataFormat','struct');
sand0_side_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand0_side_right_callback,'DataFormat','struct');
sand0_side_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand0_side_left_callback,'DataFormat','struct');
sand0_rear_right_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand0_rear_right_callback,'DataFormat','struct');
sand0_rear_left_sub = rossubscriber('/robot0/sandwich_0/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand0_rear_left_callback,'DataFormat','struct');


sand0_front_right = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand0_front_left = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand0_side_right = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand0_side_left = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand0_rear_right = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand0_rear_left = rosmessage('apriltag_ros/AprilTagDetectionArray');


% -------------------- Boat 1 ---------------------------------------

sand1_nav_sub = rossubscriber('/robot1/sandwich_1/sensors/p3d','nav_msgs/Odometry','dataformat','struct');
rabbit_sub = rossubscriber('/april_cube/rabbit','DataFormat','struct');

% ------------------- Boat 2 ---------------------------------------------

sand2_front_right_sub = rossubscriber('/robot2/sandwich_2/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand2_front_right_callback,'DataFormat','struct');
sand2_front_left_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand2_front_left_callback,'DataFormat','struct');
sand2_side_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand2_side_right_callback,'DataFormat','struct');
sand2_side_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand2_side_left_callback,'DataFormat','struct');
sand2_rear_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand2_rear_right_callback,'DataFormat','struct');
sand2_rear_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray',@sand2_rear_left_callback,'DataFormat','struct');


sand2_front_right = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand2_front_left = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand2_side_right = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand2_side_left = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand2_rear_right = rosmessage('apriltag_ros/AprilTagDetectionArray');
sand2_rear_left = rosmessage('apriltag_ros/AprilTagDetectionArray');

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


% Initialize integral terms for speed control

dist_err_cum0 = 0;
dist_err_cum1 = 0;
dist_err_cum2 = 0;

then = 0;

iter = 0;
while true

iter
now = rostime('now');
now = now.Sec + now.Nsec*1e-09;
dt = now - then;

%---------------------- Boat 0 ----------------------------

[sand0_tagID_fr,sand0_dist_fr,sand0_head_fr] = tag_detect_2(sand0_front_right,sand0_front_right_xform);
[sand0_tagID_fl,sand0_dist_fl,sand0_head_fl] = tag_detect_2(sand0_front_left,sand0_front_left_xform);
[sand0_tagID_sr,sand0_dist_sr,sand0_head_sr] = tag_detect_2(sand0_side_right,sand0_side_right_xform);
[sand0_tagID_sl,sand0_dist_sl,sand0_head_sl] = tag_detect_2(sand0_side_left,sand0_side_left_xform);
[sand0_tagID_rr,sand0_dist_rr,sand0_head_rr] = tag_detect_2(sand0_rear_right,sand0_rear_right_xform);
[sand0_tagID_rl,sand0_dist_rl,sand0_head_rl] = tag_detect_2(sand0_rear_left,sand0_rear_left_xform);

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
[debug0,target_ID0, speed0, rate0, dist_target0,err0] = apriltag_vbap(sand0_tagID, sand0_dist, sand0_head,target_range0,boat_range0,dist_err_cum0,dt)
dist_err_cum0 = err0;
% speed0
% rate0


if isempty(debug0) == 0
     %---------------------- Turn Rate Controller ----------------------------
        sandwich_0_cmd_msg.Linear.X = speed0;
    sandwich_0_cmd_msg.Angular.Z = deg2rad(rate0);
    send(sandwich_0_cmd_pub, sandwich_0_cmd_msg);
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

[sand2_tagID_fr,sand2_dist_fr,sand2_head_fr] = tag_detect_2(sand2_front_right,sand2_front_right_xform);
[sand2_tagID_fl,sand2_dist_fl,sand2_head_fl] = tag_detect_2(sand2_front_left,sand2_front_left_xform);
[sand2_tagID_sr,sand2_dist_sr,sand2_head_sr] = tag_detect_2(sand2_side_right,sand2_side_right_xform);
[sand2_tagID_sl,sand2_dist_sl,sand2_head_sl] = tag_detect_2(sand2_side_left,sand2_side_left_xform);
[sand2_tagID_rr,sand2_dist_rr,sand2_head_rr] = tag_detect_2(sand2_rear_right,sand2_rear_right_xform);
[sand2_tagID_rl,sand2_dist_rl,sand2_head_rl] = tag_detect_2(sand2_rear_left,sand2_rear_left_xform);

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
[debug2,target_ID2, speed2, rate2, dist_target2,err2] = apriltag_vbap(sand2_tagID, sand2_dist, sand2_head,target_range2,boat_range2,dist_err_cum2,dt)
dist_err_cum2 = err2;
speed2;
rate2;

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
then = now;

end
