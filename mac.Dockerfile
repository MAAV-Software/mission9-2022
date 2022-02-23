FROM dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt

#FROM ros:melodic
EXPOSE 5900

WORKDIR /

RUN apt-get update

# Fix dumb dirmngr
RUN sudo apt purge dirmngr -y && sudo apt update && sudo apt install dirmngr -y

#installing ROS http://wiki.ros.org/melodic/Installation/Ubuntu
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt update
RUN sudo apt install -y ros-melodic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN source /root/.bashrc
RUN sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN sudo rosdep init
RUN rosdep update

RUN apt-get update

# RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# RUN /bin/bash -c "source ~/.bashrc"

# # Install other Linux apps
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
    python-rospkg \
    python3-jinja2 \
    python3-numpy


RUN pip3 install -U future 

# Install some Python tools
RUN python3 -m pip install pandas jinja2 pyserial pyulog pyyaml numpy toml empy packaging jsonschema future 

#Install Mavlink
WORKDIR /usr/include
RUN git clone https://github.com/mavlink/c_library_v2.git --recursive
RUN rm -rf /usr/include/c_library_v2/.git
RUN mv /usr/include/c_library_v2/* /usr/include
RUN rmdir /usr/include/c_library_v2

COPY ./commands.sh /scripts/commands.sh
RUN chmod +x /scripts/commands.sh

# Set the PWD to root for convenience
WORKDIR /
















