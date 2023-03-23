#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /px4_sitl/PX4-Autopilot

DONT_RUN=1 make px4_sitl_default gazebo
source /mission9-2022/software_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

echo ""
echo "SET UP ENVIRONMENT VARIABLES "
echo ""
echo "run the following to start gazebo:"
echo "	roslaunch px4 posix_sitl.launch world:=/mission9-2022/mast.world"
