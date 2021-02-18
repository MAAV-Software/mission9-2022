FROM ros:melodic

WORKDIR /

RUN apt-get update

# Install ROS GUI extensions
RUN apt-get install -y \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins


# Install Gazebo and PX4 Firmware
ADD scripts/setup-gazebo.sh /setup-gazebo.sh
RUN chmod +x /setup-gazebo.sh
RUN /setup-gazebo.sh
ADD scripts/setup-px4.sh /setup-px4.sh
RUN chmod +x /setup-px4.sh
RUN /setup-px4.sh
ADD scripts/setup-sim.sh /setup-sim.sh
RUN chmod +x /setup-sim.sh

