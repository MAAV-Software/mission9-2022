#!/bin/bash

# Source
source /opt/ros/melodic/setup.bash
source /maav-mission9/software/devel/setup.bash

source /usr/share/gazebo/setup.sh


EXPORT SITL_GAZEBO_PATH=/px4_sitl/sitl_gazebo

cd software/
source software/devel/setup.bash
source /opt/ros/melodic/setup.bash
catkin build
