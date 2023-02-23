#!/usr/bin/env bash
set -eu

# Copyright 2023 Anis Koubaa
# Licensed under the Apache License, Version 2.0


CHOOSE_ROS_DISTRO=humble
INSTALL_PACKAGE=desktop
TARGET_OS=jammy
ROS2_WORKSPACE=cs460_ws

# Check OS version
if ! which lsb_release > /dev/null ; then
	sudo apt-get update
	sudo apt-get install -y curl lsb-release
fi

if [[ "$(lsb_release -sc)" == "$TARGET_OS" ]]; then
	echo "OS Check Passed"
else
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This OS (version: $(lsb_release -sc)) is not supported"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

if ! dpkg --print-architecture | grep -q 64; then
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This architecture ($(dpkg --print-architecture)) is not supported"
	printf '\033[33m%s\033[m\n' "See https://www.ros.org/reps/rep-2000.html"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

#sudo apt-get update && sudo apt upgrade -y

# Install
#sudo apt-get update

sudo apt-get install ros-$CHOOSE_ROS_DISTRO-gazebo-*

sudo apt install -y ros-$CHOOSE_ROS_DISTRO-cartographer
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-cartographer-ros

sudo apt install -y ros-$CHOOSE_ROS_DISTRO-navigation2
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-nav2-*
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-tf2-*
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-tf-*
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-nav2-bringup

source ~/.bashrc
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-dynamixel-sdk
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-turtlebot3-msgs
sudo apt install -y ros-$CHOOSE_ROS_DISTRO-turtlebot3

sudo apt install -y ros-$CHOOSE_ROS_DISTRO-desktop

source ~/.bashrc

cd ~/$ROS2_WORKSPACE/src/
#git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
#git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
cd ~/$ROS2_WORKSPACE/
colcon build --symlink-install

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export svga_vgpu10=0'
source ~/.bashrc
echo "success installing Turtlebot3 on ROS2 $CHOOSE_ROS_DISTRO"
echo "Turtlebot3 Burger is activated. You can change it from .bashrc file"

