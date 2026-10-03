#!/bin/bash
# TODO: verify this is run a student user

source ~/yumift_ws/devel/setup.bash
# TODO: do something to verify connection to the robot
export ROS_MASTER_URI=http://192.168.131.1:11311
export ROS_IP=192.168.131.70

# TODO: in an ideal universe you manage to read the ip from arguments here in bash
roslaunch yumi_controller bringup.launch robot_ip:=192.168.131.65 rviz:=False &

# TODO: this might fail and you might have to do something
# see what this is and whether it can be automated.
# otherwise you'll have to open a terminal in which you run the docker
#rosrun yumi_controller start_egm.py &

# TODO: ideally check if you are to use f/t sensors from arguments
# NOTE: this does gravity compensation
#roslaunch yumi_controller sensors.launch sensor_ip_right:=<right_ip> sensor_ip_left:=<left_ip>

# run the translator
#python3 slumi_translator.py "$@"

