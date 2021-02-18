#!/bin/bash

echo "Installing Gazebo"
sleep 3

sudo apt-get install curl
curl -sSL http://get.gazebosim.org | sh # installs ROS as well
