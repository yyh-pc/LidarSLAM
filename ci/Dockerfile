# Base Image
FROM ubuntu:18.04

WORKDIR /usr/src/app

# needed to avoid interaction during dependancies installation
ARG DEBIAN_FRONTEND=noninteractive 
# Install dependancies
# freeglut install OpenGL which is needed by g2o event when disabling OpenGL support
# libgoogle-glog-dev is needed by ceres
RUN apt-get update \
    && apt-get install -y \
        build-essential \
        git \
        cmake \
	libeigen3-dev \
        libpcl-dev \
	freeglut3-dev \
	libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ceres
RUN git clone https://github.com/ceres-solver/ceres-solver.git ceres
RUN mkdir ceres_build
RUN cd ceres_build && cmake \
        -DBUILD_EXAMPLES:BOOL=ON \
        -DBUILD_TESTING:BOOL=ON \
        -DEXPORT_BUILD_DIR:BOOL=OFF \
        -DCMAKE_INSTALL_PREFIX:FILEPATH=/usr/local/ \
		../ceres
RUN cd ceres_build && make install -j 16
RUN rm -rf ceres ceres_build

# Install nanoflann
RUN git clone https://github.com/jlblancoc/nanoflann.git nanoflann
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
