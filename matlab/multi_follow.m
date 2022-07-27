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

sand2_front_right_sub = rossubscriber('/robot2/sandwich_2/sensors/cameras/front_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_front_left_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/front_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_side_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/side_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_side_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/side_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_rear_right_sub =  rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_right_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');
sand2_rear_left_sub =   rossubscriber('/robot2/sandwich_2/sensors/cameras/rear_left_camera/tag_detections','apriltag_ros/AprilTagDetectionArray','DataFormat','struct');





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


%%

while true
tftree = rostf;
frames = tftree.AvailableFrames;

% boat0_index = find(not(cellfun('isempty',strfind(frames,'boat0_tag'))));
boat1_index = find(not(cellfun('isempty',strfind(frames,'boat1_tag'))));

% % -------------------- Boat 1 Following -----------------------------
% if isempty(boat0_index) == 1
%     boat_tags = {};
% else
%     clear boat_tag_tf
%     clear boat_tags
%    for i=1:numel(boat0_index)
%         boat_tags{i} = frames{boat0_index(i)};
%    end
%     for j=1:numel(boat_tags)
%          boat_tag_tf(j) = getTransform(tftree,'sandwich_1/base_link',boat_tags{j},'Timeout',inf);
%     end
% [boattagID,boatdist,boathead] = aprilTag_xform(boat_tag_tf)
% [boatleft,boatright] = aprilTag_boatfollow(boatdist,boathead);
%  sandwich_1_left_msg.Data = boatleft;
%  sandwich_1_right_msg.Data = boatright;
% send(sandwich_1_left_pub, sandwich_1_left_msg);
% send(sandwich_1_right_pub, sandwich_1_right_msg);
%  end
% -------------------- Boat 2 Following -----------------------------
if isempty(boat1_index) == 1
    boat1_tags = {};
else
    clear boat1_tag_tf
    clear boat1_tags
   for i=1:numel(boat1_index)
        boat1_tags{i} = frames{boat1_index(i)};
   end
    for j=1:numel(boat1_tags)
         boat1_tag_tf(j) = getTransform(tftree,'sandwich_2/base_link',boat1_tags{j},'Timeout',inf);
    end
[boat1tagID,boat1dist,boat1head] = aprilTag_xform(boat1_tag_tf)
[boat1left,boat1right] = aprilTag_boatfollow(boat1dist,boat1head);
 sandwich_2_left_msg.Data = boat1left;
 sandwich_2_right_msg.Data = boat1right;
send(sandwich_2_left_pub, sandwich_2_left_msg);
send(sandwich_2_right_pub, sandwich_2_right_msg);
 end


% waitfor(rate);
end
