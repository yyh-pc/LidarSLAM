# Base Image
# ROS Noetic is based on Ubuntu 20.04
FROM ros:noetic-perception

WORKDIR /usr/src/app

# needed to avoid interaction during dependencies installation
ARG DEBIAN_FRONTEND=noninteractive

# Install binaries dependencies
RUN apt-get update \
  && apt-get install -y \
    build-essential \
    git \
    cmake \
    # LidarSlam lib
    libeigen3-dev \
    libpcl-dev \
    libceres-dev \
    # ROS wrapping
    ros-noetic-velodyne-pcl \
    ros-noetic-geodesy \
    ros-noetic-gps-common \
  && rm -rf /var/lib/apt/lists/*

# Install nanoflann
RUN git clone --branch v1.3.2 https://github.com/jlblancoc/nanoflann.git nanoflann
RUN mkdir nanoflann_build
RUN cd nanoflann_build && cmake \
  -DBUILD_TESTS:BOOL=OFF \
  -DBUILD_EXAMPLES:BOOL=OFF \
  -DCMAKE_INSTALL_PREFIX:FILEPATH=/usr/local/ \
  ../nanoflann
RUN cd nanoflann_build && make install -j 16
RUN rm -rf nanoflann nanoflann_build

# Install g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git g2o
RUN mkdir g2o_build
RUN cd g2o_build && cmake \
  -DG2O_USE_OPENGL:BOOL=OFF \
  -DG2O_BUILD_APPS:BOOL=OFF \
  -DG2O_BUILD_EXAMPLES:BOOL=OFF \
  -DCMAKE_INSTALL_PREFIX:FILEPATH=/usr/local \
  ../g2o
RUN cd g2o_build && make install -j 16
RUN rm -rf g2o g2o_build
