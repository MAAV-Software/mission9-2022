FROM ros:melodic

RUN apt-get update

# Install ROS GUI extensions
RUN apt-get install -y \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins

# Other tools
# vim preferrably

# Setup Catkin Workspace
# TODO

# Install Gazebo
# TODO

# Install PX4 Stuff
# TODO
