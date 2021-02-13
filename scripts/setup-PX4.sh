#!/bin/bash
​
# script to download px4 and upload to board
​
# To install ROS/Gazebo "Melodic" and PX4, execute the following:
# wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
# bash ubuntu_sim_ros_melodic.sh
​
# install/build PX4
sudo apt-get install git
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
# install Gazebo, JMAVSim, and NuttX
# bash ./Tools/setup/ubuntu.sh # may need password
# upload to board
# make px4_sitl gazebo upload