FROM ros:melodic

WORKDIR /

RUN apt-get update

# Install ROS GUI extensions
RUN apt-get install -y \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins \
    ros-melodic-mavros \
    ros-melodic-mavros-extras

# Install other Linux apps
RUN apt-get install -y \
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
    python-empy \
    python-toml \
    python-numpy \
    python-dev \
    python3 \
    python3-pip \
    python-yaml \
    # From PX4 SITL script
    libgstreamer1.0-0 \
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
    python-rospkg \
    python3-jinja2 \
    python3-numpy

# Install some Python tools
RUN python3 -m pip install pandas jinja2 pyserial pyulog pyyaml numpy toml empy

# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh
# TODO Installs ROS as well...???

# Install PX4 Firmware and AutoPilot
# TODO Rename px4_sitl to px4?
RUN mkdir -p /px4_sitl && cd /px4_sitl && \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN cd PX4-Autopilot/ && git submodule update --init --recursive
RUN echo "PX4: Configuring modemanager"
RUN usermod -a -G dialout $USER && apt-get remove modemmanager -y
RUN echo "PX4: Installing dependencies"
RUN if [ ! -f exe/QGroundControl.AppImage ]; then \
        wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -P /bin \
        chmod a+x /bin/QGroundControl.AppImage \
    fi

# Clone and build sitl
# TODO Should this be re-using the px4-sitl directory?
RUN cd /px4_sitl && git clone --recursive https://github.com/PX4/sitl_gazebo.git
RUN cd sitl_gazebo/ && mkdir build && cd build
# RUN CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/bin/gazebo
# TODO Implement num cores thing from last semester
# RUN cmake .. && make -j && make install

# RUN cd /px4_sitl/PX4-Autopilot
# RUN DONT_RUN=1 make px4_sitl_default gazebo
