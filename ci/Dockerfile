# Base Image
# ROS Noetic is based on Ubuntu 20.04
FROM ros:noetic-perception

WORKDIR /usr/src/app

# needed to avoid interaction during dependencies installation
ARG DEBIAN_FRONTEND=noninteractive

# Check the update and change key if needed
RUN apt-get update \
    || (rm -rf /etc/apt/trusted.gpg \
        && apt-get install -y curl && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -)

# Install binaries dependencies
RUN apt-get update \
  && apt-get install -y \
    build-essential \
    git \
    cmake \
    # CERES
    libgoogle-glog-dev \
    # LidarSlam lib
    libeigen3-dev \
    libpcl-dev \
    # ROS wrapping
    ros-noetic-rviz \
    ros-noetic-velodyne-pcl \
    ros-noetic-velodyne-pointcloud \
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

#Install Ceres
RUN git clone https://github.com/ceres-solver/ceres-solver.git ceres
RUN mkdir ceres_build
RUN cd ceres_build && cmake \
  -DCMAKE_INSTALL_PREFIX:FILEPATH=/usr/local \
  -DBUILD_EXAMPLES:BOOL=OFF \
  -DBUILD_TESTING:BOOL=OFF \
  ../ceres
RUN cd ceres_build && make install -j 16
RUN rm -rf ceres ceres_build

# Install g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git g2o
RUN mkdir g2o_build
RUN cd g2o_build && cmake \
  -DG2O_USE_OPENGL:BOOL=OFF \
  -DG2O_USE_VENDORED_CERES:BOOL=OFF \
  -DG2O_BUILD_APPS:BOOL=OFF \
  -DCeres_DIR:FILEPATH=/usr/local/lib/cmake/Ceres \
  -DG2O_BUILD_EXAMPLES:BOOL=OFF \
  -DCMAKE_INSTALL_PREFIX:FILEPATH=/usr/local \
  ../g2o
RUN cd g2o_build && make install -j 16
RUN rm -rf g2o g2o_build

# Testing data
# Dockerfile must be run in a folder containing all data folders (e.g test1, test2...)
RUN mkdir -p ./data/test1
RUN mkdir -p ./data/test2
COPY ./test1/test1.bag ./data/test1
COPY ./test2/test2.bag ./data/test2