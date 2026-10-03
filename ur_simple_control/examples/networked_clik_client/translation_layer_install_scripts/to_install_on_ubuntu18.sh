#/bin/bash
cd ~
source /opt/ros/noetic/setup.bash
apt update
apt install --yes python3.8
#update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6.9
#update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8.1
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2
apt install python3-pip
pip3 install protobuf

apt install --yes git
sudo apt install --yes python3-catkin-tools
mkdir -p ~/yumift_ws/src && cd ~/yumift_ws/src && catkin init
git clone https://github.com/ros-industrial/abb_robot_driver.git
gpg --keyserver hkps://keyserver.ubuntu.com --recv-key 0xAB17C654
sudo apt update
sudo apt install --yes python3-vcstool
vcs import . --input abb_robot_driver/pkgs.repos
git clone https://github.com/UTNuclearRoboticsPublic/netft_utils.git -b master
git clone https://github.com/ilVecc/yumift.git
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro noetic
catkin build
echo "source ~/yumift_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
