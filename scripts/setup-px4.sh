#!/bin/bash

echo "Installing PX4 firmware"
cd /
mkdir -p px4_sitl && cd px4_sitl
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
