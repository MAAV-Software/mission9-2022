#!/bin/bash

# install required dependencies for px4_sitl
apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
apt-get install ros-melodic-mavros ros-melodic-mavros-extras -y
apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python3-jinja2 python3-numpy -y
pip3 install numpy toml empy pyyaml

# Clone and build sitl
cd px4_sitl
git clone --recursive https://github.com/PX4/sitl_gazebo.git
cd sitl_gazebo/
mkdir build && cd build
#CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/bin/gazebo
#cmake ..
#make -j
#make install

#cd /px4_sitl/PX4-Autopilot
#DONT_RUN=1 make px4_sitl_default gazebo
