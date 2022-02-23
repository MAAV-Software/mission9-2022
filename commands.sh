#!/bin/bash
#Install Gazebo
echo "INSTALLING GAZEBO"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libgazebo11
curl -sSL http://get.gazebosim.org | sh

#Install PX4
echo "INSTALLING PX4 Autopilot"
mkdir -p /px4_sitl
cd /px4_sitl
git clone https://github.com/PX4/PX4-Autopilot.git 
cd /px4_sitl/PX4-Autopilot
git checkout a6274bc
git submodule update --init --recursive

apt-get remove modemmanager -y

echo "PX4: Installing dependencies"
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -P /bin && \
    chmod a+x /bin/QGroundControl.AppImage

#Install mavros
apt-get install -y \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins \
    ros-melodic-mavros \
    ros-melodic-mavros-extras

# Clone sitl
cd /px4_sitl
git clone --recursive https://github.com/PX4/sitl_gazebo.git
mkdir /px4_sitl/sitl_gazebo/build
cd /px4_sitl/sitl_gazebo/build/

# Build Sitl
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/bin/gazebo
cmake .. && make -j3 && make install

#Set some environment variables to get PX4 to build
export LANG=C.UTF-8
export LC_ALL=C.UTF-8