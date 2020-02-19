cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(LidarSlam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(nanoflann REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

find_package(g2o REQUIRED)

find_package(PCL REQUIRED COMPONENTS common kdtree registration io)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Build core SLAM lib
add_subdirectory(slam_lib)

# Build optionnal ParaView wrapping
option(PARAVIEW_WRAPPING "Build a Paraview plugin to wrap SLAM" OFF)
if (PARAVIEW_WRAPPING)
    add_subdirectory(paraview_wrapping)
endif(PARAVIEW_WRAPPING)

# To build/use ROS wrapping, just put this entire directory in the catkin src
# workspace and run catkin_make.