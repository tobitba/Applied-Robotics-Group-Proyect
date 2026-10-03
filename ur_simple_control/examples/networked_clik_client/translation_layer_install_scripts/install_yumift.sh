#!/bin/bash
cd ~
mkdir -p ~/yumift_ws/src && cd ~/yumift_ws && catkin init
cd ./src
git clone https://github.com/ros-industrial/abb_robot_driver.git
vcs import . --input abb_robot_driver/pkgs.repos
git clone https://github.com/UTNuclearRoboticsPublic/netft_utils.git -b master
git clone https://github.com/ilVecc/yumift.git
cd ..
rosdep update --rosdistro=noetic
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
catkin build # fails
echo "source ~/yumift_ws/devel/setup.bash" >> ~/.bashrc
