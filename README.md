# optical_swarm

Dependencies:

geonav_transform
git clone git@github.com:bsb808/geonav_transform.git

mrc_examples
git clone git@github.com:MultiRobotControl/mrc_examples.git

tams_apriltags
git clone git@github.com:TAMS-Group/tams_apriltags.git

usv_control
git clone git@github.com:bsb808/usv_control.git

vorc
git clone git@github.com:osrf/vorc.git

vrx
git clone git@github.com:osrf/vrx.git

# this spawns the vorc model with lidar and camera, rabbit node with buoy visual, and aprilcube robots from tams_apriltags
# in a separate terminal window

$ roslaunch optical_swarm trial.launch

# this spawns rviz with both lidar and camera displayed. 

$ roslaunch optical_swarm rviz_cora1.launch

