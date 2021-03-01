#!/bin/bash

cd ../../px4_sitl/PX4-Autopilot/
source /opt/ros/melodic/setup.bash
DONT_RUN=1 make px4_sitl_default gazebo
#source /mission9-2021/software/devel/setup.bash
#source Tools/setup_gazebo.bash $(pwd)/build/px4_sitl_default

#cd ../../mission9-2021/software
#roslaunch px4 mavros_posix_sitl.launch
