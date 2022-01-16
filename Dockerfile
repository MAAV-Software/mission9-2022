FROM dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt

#FROM ros:melodic
EXPOSE 5900

WORKDIR /

RUN apt-get update

# Fix dumb dirmngr
RUN sudo apt purge dirmngr -y && sudo apt update && sudo apt install dirmngr -y

# Adding keys for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

    
# Make it so we dont have to source devel every freaking time
# RUN /bin/bash -c "echo 'source /mission9-2021/software/devel/setup.bash' >> ~/.bashrc && \
#                   echo 'source /mission9-2021/software/devel/setup.bash' >> /root/.bashrc "

RUN apt-get update



# # Install ROS GUI extensions
# RUN apt-get install -y \
#     ros-melodic-rqt \
#     ros-melodic-rqt-common-plugins \
#     ros-melodic-mavros \
#     ros-melodic-mavros-extras



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


RUN wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
RUN bash ubuntu_sim_ros_melodic.sh

# Install Gazebo
# TODO This installs ROS as well. Is this a problem?
# RUN curl -sSL http://get.gazebosim.org | sh

# RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
#     wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
#     apt-get update && \
#     apt-get install -y gazebo9 libgazebo9-dev && \
#     apt-get install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control




# Set the PWD to root for convenience
WORKDIR /
