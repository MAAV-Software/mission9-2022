#!/bin/bash

echo "Installing PX4 firmware"
cd /
mkdir -p px4_sitl && cd px4_sitl
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

cd PX4-Autopilot/
git submodule update --init --recursive

echo "Configuring modemanager"
sudo usermod -a -G dialout $USER
apt-get remove modemmanager -y

apt-get install git zip qtcreator cmake build-essential genromfs ninja-build libopencv-dev wget -y
apt-get install python-argparse python-empy python-toml python-numpy python-dev python3 python3-pip python-yaml -y
python3 -m pip install pandas jinja2 pyserial

# optional python tools
python3 -m pip install pyulog
python3 -m pip install pyyaml

echo "Installing dependencies"
if [ ! -f exe/QGroundControl.AppImage ]; then
	wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -P /bin
	chmod a+x /bin/QGroundControl.AppImage
fi
