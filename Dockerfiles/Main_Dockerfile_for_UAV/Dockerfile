# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em 

#This is from the ROS Docker Hub https://hub.docker.com/_/ros
FROM ros:melodic-robot

# install ros packages from osrf https://github.com/osrf/docker_images
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils \
    ros-melodic-robot=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

ENV OPENCV_VERSION 3.2.0

RUN sudo apt-get update && sudo apt-get install -y build-essential \
    pkg-config \
    unzip \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    nano \
    htop \
    tmux \
    ros-melodic-usb-cam \
    ros-melodic-compressed-image-transport \
    ros-melodic-camera-info-manager \ 
    ros-melodic-realsense2-camera \
    ros-melodic-realsense2-description \
    ros-melodic-rtabmap-ros \
    ros-melodic-rtabmap \ 
    ros-melodic-multirobot-map-merge \
    ros-melodic-multi-map-server \
    ros-melodic-robot-localization \
    ros-melodic-cv-bridge 

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && sudo ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

RUN git clone https://github.com/ros-drivers/video_stream_opencv.git 

# ----- Install OpenCV -----
ENV OPENCV_VERSION 3.4.5

# install necessary libraries
RUN apt-get -y install \
	libjpeg-dev \
	libtiff5-dev \
#	libjasper-dev \
#	libpng12-dev \
	libavcodec-dev \
	libavformat-dev \
	libswscale-dev \
	libv4l-dev \
	libxvidcore-dev \
	libx264-dev \
	libgtk2.0-dev \
	libatlas-base-dev \
	gfortran \
	python2.7-dev \
	python3-dev \
	python-pip \
	python-numpy

# download latest source & contrib
RUN cd /tmp && \
	wget -O opencv.zip https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip && \
	unzip opencv.zip
	
RUN cd /tmp && \
	wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip && \
	unzip opencv_contrib.zip


RUN cd /tmp/opencv-$OPENCV_VERSION && \
	mkdir build && \
	cd build && \
	cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_C_EXAMPLES=ON \
	-D BUILD_PYTHON_SUPPORT=ON \
	-D BUILD_NEW_PYTHON_SUPPORT=ON \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-$OPENCV_VERSION/modules \
	-D BUILD_EXAMPLES=ON .. && \
	make -j4 && \
	make && \
    make install && \
	# ldconfig && \
	make clean
	
# cleanup source
RUN cd /tmp && rm -rf opencv-$OPENCV_VERSION


# cleanup source
RUN cd /tmp && rm -rf opencv-$OPENCV_VERSION
RUN rm -rf /var/lib/apt/lists/* 
