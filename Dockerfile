FROM ros:melodic

RUN apt-get update

# Install ROS GUI extensions
RUN apt-get install -y \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins

# TODO Other ROS tools?

# Setup Catkin Workspace
# TODO

# Install Gazebo
# TODO

# Install PX4 Stuff
# TODO
