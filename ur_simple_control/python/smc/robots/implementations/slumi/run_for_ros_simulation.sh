#!/bin/bash
roscore &
roslaunch yumi_controller bringup.launch rviz:=False &
python3 slumi_translator.py "$@"
