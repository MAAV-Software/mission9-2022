echo "MAAV 2022 Installation Script"

#Get env 
cd /

#installing ROS http://wiki.ros.org/noetic/Installation/Ubuntu
echo "Step 1: Installing ROS"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full

/bin/bash -c

echo "Step 2: Installing python ros dependencies"
echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
source /root/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

apt-get update

#Install Deps
echo "Step 3: Installing dependencies"
apt-get install -y \
    curl \
    git \
    zip \
    qtcreator \
    cmake \
    build-essential \
    genromfs \
    ninja-build \
    libopencv-dev \
    wget \
    python-argparse \
    python3-empy \
    python3-toml \
    python-numpy \
    python-dev \
    python3 \
    python3-pip \
    python-yaml \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libprotobuf-dev \
    libprotoc-dev \
    protobuf-compiler \
    libeigen3-dev \
    libxml2-utils \
    python3-rospkg \
    python3-jinja2 \
    python3-numpy

pip3 install -U future 

# Install some Python tools
python3 -m pip install pandas jinja2 pyserial pyulog pyyaml numpy toml empy packaging jsonschema future 

pip install --upgrade numpy

#Install Mavlink
echo "Step 4: Install MAVLINK"
cd /usr/include
git clone https://github.com/mavlink/c_library_v2.git --recursive
rm -rf /usr/include/c_library_v2/.git
mv /usr/include/c_library_v2/* /usr/include
rmdir /usr/include/c_library_v2

# Set the PWD to root for convenience
cd /

#Install Gazebo
echo "Step 5: INSTALLING GAZEBO"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libgazebo11
curl -sSL http://get.gazebosim.org | sh

#Install PX4
echo "Step 6: INSTALLING PX4 Autopilot"
mkdir -p /px4_sitl
cd /px4_sitl
git clone https://github.com/PX4/PX4-Autopilot.git 
cd /px4_sitl/PX4-Autopilot
git checkout a6274bc
git submodule update --init --recursive

apt-get remove modemmanager -y

echo "Step 7: PX4 Installing dependencies"
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -P /bin && \
    chmod a+x /bin/QGroundControl.AppImage

#Install mavros
echo "Step 8: Installing mavros"
apt-get install -y \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-mavros \
    ros-noetic-mavros-extras

# Clone sitl
echo "Step 9: Cloning PX4 sitl"
cd /px4_sitl
git clone --recursive https://github.com/PX4/sitl_gazebo.git
mkdir /px4_sitl/sitl_gazebo/build
cd /px4_sitl/sitl_gazebo/build/

# Build Sitl
echo "Step 10: Building PX4 sitl"
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/bin/gazebo
cmake .. && make -j2 && make install

#Set some environment variables to get PX4 to build
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# Install GeographicLib
echo "Step 11: Installing GeographicLib"
cd /
git clone https://github.com/OSGeo/GeographicLib
mkdir /GeographicLib/build
cd /GeographicLib/build
cmake ..
make
make install

# Set GeographicLib Environment Variables
echo 'export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/share/GeographicLib/CMake' >> /root/.bashrc

source /root/.bashrc
