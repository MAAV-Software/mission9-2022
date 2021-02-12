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
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive \
    cd PX4-Autopilot \
    make px4_sitl gazebo
