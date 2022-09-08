FROM dorowu/ubuntu-desktop-lxde-vnc:focal-lxqt

EXPOSE 5900

WORKDIR /

RUN apt-get update

# Fix dumb dirmngr
RUN sudo apt purge dirmngr -y && sudo apt update && sudo apt install dirmngr -y

#installing ROS http://wiki.ros.org/noetic/Installation/Ubuntu
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt update
RUN sudo apt install -y ros-noetic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN source /root/.bashrc
RUN sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN sudo rosdep init
RUN rosdep update

RUN apt-get update

# # Install other Linux apps
RUN apt-get install -y \
    git \
    zip \
    qtcreator \
    cmake \
    g++ \
    unzip \
    build-essential \
    genromfs \
    ninja-build \
    libopencv-dev \
    wget \
    at-spi2-core \
    python-argparse \
    python3-empy \
    python3-toml \
    python3-numpy \
    python3-dev \
    python3 \
    python3-pip \
    python3-yaml \
    # From PX4 SITL script
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

# Install some Python tools
RUN python3 -m pip install pandas jinja2 pyserial pyulog pyyaml numpy toml empy packaging jsonschema future 

#Install Gazebo
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get -y install libgazebo11
RUN curl -sSL http://get.gazebosim.org | sh

#Install PX4
RUN mkdir -p /px4_sitl
WORKDIR /px4_sitl
RUN git clone https://github.com/PX4/PX4-Autopilot.git 
WORKDIR /px4_sitl/PX4-Autopilot
RUN git checkout a6274bc
RUN git submodule update --init --recursive

RUN apt-get remove modemmanager -y

RUN echo "PX4: Installing dependencies"
RUN wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -P /bin && \
    chmod a+x /bin/QGroundControl.AppImage

#Install mavros
RUN apt-get install -y \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-mavros \
    ros-noetic-mavros-extras

# Install Mavlink
WORKDIR /usr
RUN git clone https://github.com/mavlink/c_library_v2.git --recursive
RUN rm -rf /usr/c_library_v2/.git
RUN mv /usr/c_library_v2/* /usr
RUN rmdir /usr/c_library_v2

# Clone sitl
WORKDIR /px4_sitl
RUN git clone --recursive https://github.com/PX4/sitl_gazebo.git
RUN mkdir /px4_sitl/sitl_gazebo/build
WORKDIR /px4_sitl/sitl_gazebo/build/

# Build Sitl
RUN CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/bin/gazebo
RUN cmake .. && make -j3 && make install

# Fix stuff
RUN export LANG=C.UTF-8
RUN export LC_ALL=C.UTF-8

# Install OpenCV
RUN sudo apt install -y libopencv-dev

# Install Realsense
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN sudo apt-get install -y librealsense2-dkms
RUN sudo apt-get install -y librealsense2-utils

# Install ROS packages
RUN apt-get install -y \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description \ 
    ros-noetic-vision-visp \ 
    ros-noetic-find-object-2d

# Set the PWD to root for convenience
WORKDIR /
