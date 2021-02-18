#!/bin/bash

# Clone and build sitl
cd /
mkdir -p /px4_sitl && cd px4_sitl
git clone --recursive https://github.com/PX4/sitl_gazebo.git
#mkdir build && cd build
