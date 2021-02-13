FROM ros:melodic

WORKDIR /

RUN apt-get update

# Install ROS GUI extensions
RUN apt-get install -y \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins \
    curl

# Other tools
# vim preferrably

# Setup Catkin Workspace
# TODO

# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh
#ADD scripts/setup-gazebo.sh /setup-gazebo.sh
#RUN /setup-gazebo.sh

# Install PX4 Stuff
# TODO
