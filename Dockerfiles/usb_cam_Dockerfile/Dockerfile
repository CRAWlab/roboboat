# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-robot

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-robot=1.4.1-0* \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Taken form Dr. Vaughan. https://github.com/CRAWlab/CRAWLAB-Docker/blob/master/Raspberry%20Pi%20ROS%20Tutor$
RUN sudo apt-get update && sudo apt-get install -y \
    ros-melodic-ros-tutorials \
    ros-melodic-common-tutorials \
    ros-melodic-usb-cam \
    ros-melodic-roscpp \
    ros-melodic-catkin \
    ros-melodic-compressed-image-transport \
    ros-melodic-roscpp \
    ros-melodic-std-msgs \
    ros-melodic-std-srvs \
    ros-melodic-sensor-msgs \
    ros-melodic-camera-info-manager \
    ros-melodic-dynamic-reconfigure \
    ros-melodic-diagnostic-updater \
    ros-melodic-message-generation \
    ros-melodic-message-runtime

# install necessary build tools
RUN apt-get -y install build-essential \
    wget \
    cmake \
    pkg-config \
    apt-transport-https \
    dirmngr \
    gnupg2 \
    lsb-release \
    automake \
    autopoint \
    libboost-all-dev \
    libtool \
    libzip-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavdevice-dev \
    libwebp-dev \
    gettext \
    libmicrohttpd-dev \
    && rm -rf /var/lib/apt/lists/*


# Makes catkin_ws in root direcory...
RUN mkdir -p ~/catkin_ws/src

# Set up the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd ~/catkin_ws/ && \
                  catkin_make &&\
                  echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc"
