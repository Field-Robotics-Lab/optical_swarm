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


% ---------- Thrust Command Publishers ----------------------------

sandwich_0_left_pub = rospublisher('/sandwich_0/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_0_right_pub = rospublisher('/sandwich_0/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_0_left_msg = rosmessage(sandwich_0_left_pub);
sandwich_0_right_msg = rosmessage(sandwich_0_right_pub);

sandwich_1_left_pub = rospublisher('/sandwich_1/thrusters/left_thrust_cmd','std_msgs/Float32');
sandwich_1_right_pub = rospublisher('/sandwich_1/thrusters/right_thrust_cmd','std_msgs/Float32');
sandwich_1_left_msg = rosmessage(sandwich_1_left_pub);
sandwich_1_right_msg = rosmessage(sandwich_1_right_pub);

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

% ----------------- IMU Subscriber -----------------------------------
sand0_imu_sub = rossubscriber('/robot0/sandwich_0/sensors/imu/imu/data');
sand1_imu_sub = rossubscriber('/robot1/sandwich_1/sensors/imu/imu/data');

% ----------------Initialize and set PI gains-----------------
rate_err_old0 = 0;
rate_err0 = 0;
rate_err_cum0 = 0;

rate_err_old1 = 0;
rate_err1 = 0;
rate_err_cum1=0;

dist_old0 = 0;
dist_old1 = 0;

head_offset0 = 10;
head_offset1 = -head_offset0;

% Gains
K_p = 0.1;
K_i = 0.01;

% Max thrust value
max_thrust = 1.0;

while true

%---------------------- Boat 0 ----------------------------
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
% boat_range0 = [10,11,12,13];
               
target_range0 = [90,91,92,93,94,95];
boat_range0 = [10,11,12,13];

%----------- Calculate Speed and Rate Commands-----------------
[debug0,target_ID0, speed0, rate0, dist_target0] = vbap_test(sand0_tagID, sand0_dist, sand0_head,target_range0,boat_range0,dist_old0,head_offset0)
dist_old0 = dist_target0;
if isempty(debug0) == 0
     %---------------------- Turn Rate Controller ----------------------------
        imu0 = receive(sand0_imu_sub);
        imu0_rate = rad2deg(imu0.AngularVelocity.Z);
        rate_err0 = (rate0 - imu0_rate);
        rate_err_cum0 = rate_err_cum0+ 0.5*(rate_err0 + rate_err_old0)*0.1;
        turn0 = K_p*rate_err0 + K_i*rate_err_cum0;
         
        left_thrust0 = -turn0;
        right_thrust0 = turn0;

        % Update old error
        rate_err_old0 = rate_err0;

    fwd0 = sandwich_speed(speed0);
    sand0_left = min(max_thrust, max(-max_thrust,fwd0 + left_thrust0));
    sand0_right = min(max_thrust, max(-max_thrust,fwd0 + right_thrust0));
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
boat_range1 = [ 0, 1, 2, 3];

% target_range1 = [ 0, 1, 2, 3];
% boat_range1 = [9999999];
                
%----------- Calculate Speed and Rate Commands-----------------
[debug1,target_ID1, speed1, rate1, dist_target1] = vbap_test(sand1_tagID, sand1_dist, sand1_head,target_range1,boat_range1,dist_old1,head_offset1)
dist_old1 = dist_target1;

if isempty(debug1) == 0
%---------------------- Turn Rate Controller ----------------------------
        imu1 = receive(sand1_imu_sub);
        imu1_rate = rad2deg(imu1.AngularVelocity.Z);
        rate_err1 = (rate1 - imu1_rate);
        rate_err_cum1 = rate_err_cum1+ 0.5*(rate_err1 + rate_err_old1)*0.1;
        turn1 = K_p*rate_err1 + K_i*rate_err_cum1;
        left_thrust1 = -turn1;
        right_thrust1 = turn1;

        % Update old error
        rate_err_old1 = rate_err1;

    fwd1 = sandwich_speed(speed1);
    sand1_left = min(max_thrust, max(-max_thrust,fwd1 + left_thrust1));
    sand1_right = min(max_thrust, max(-max_thrust,fwd1 + right_thrust1));
    sandwich_1_left_msg.Data = sand1_left;
    sandwich_1_right_msg.Data = sand1_right;
    send(sandwich_1_left_pub, sandwich_1_left_msg);
    send(sandwich_1_right_pub, sandwich_1_right_msg);
else
    sandwich_1_left_msg.Data = 0;
    sandwich_1_right_msg.Data = 0;
    send(sandwich_1_left_pub, sandwich_1_left_msg);
    send(sandwich_1_right_pub, sandwich_1_right_msg);
end

waitfor(rate);
end
