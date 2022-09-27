cd /px4_sitl/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source /mission9-2022/software_ws/devel/setup.bash
source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo/sitl_gazebo
roslaunch px4 posix_sitl.launch
