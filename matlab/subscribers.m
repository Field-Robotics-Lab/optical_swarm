% Tag Detection Subscribers

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

% Tag Detection Receive Messages


% ----------------- Boat 0 ---------------------------------

sand0_front_right_msg = receive(sand0_front_right_sub,inf); 
sand0_front_left_msg = receive(sand0_front_left_sub ,inf);
sand0_side_right_msg = receive(sand0_side_right_sub ,inf);
sand0_side_left_msg = receive(sand0_side_left_sub ,inf);
sand0_rear_right_msg = receive(sand0_rear_right_sub,inf);
sand0_rear_left_msg = receive(sand0_rear_left_sub ,inf);

% -------------------- Boat 1 ---------------------------------------
  
sand1_front_right_msg = receive(sand1_front_right_sub,inf);
sand1_front_left_msg = receive(sand1_front_left_sub ,inf);
sand1_side_right_msg = receive(sand1_side_right_sub ,inf);
sand1_side_left_msg = receive(sand1_side_left_sub ,inf);
sand1_rear_right_msg = receive(sand1_rear_right_sub ,inf);
sand1_rear_left_msg = receive(sand1_rear_left_sub ,inf);

% ------------------- Boat 2 ---------------------------------------------

sand2_front_right_msg = receive(sand2_front_right_sub,inf);
sand2_front_left_msg = receive(sand2_front_left_sub ,inf);
sand2_side_right_msg = receive(sand2_side_right_sub ,inf);
sand2_side_left_msg = receive(sand2_side_left_sub ,inf);
sand2_rear_right_msg = receive(sand2_rear_right_sub ,inf);
sand2_rear_left_msg = receive(sand2_rear_left_sub ,inf);