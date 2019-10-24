cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(LidarSlam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(nanoflann REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

find_package(PCL REQUIRED COMPONENTS common kdtree)
include(cmake/create_PCL_target.cmake)

add_subdirectory(slam_lib)

option(PARAVIEW_WRAPPING "Create a Paraview plugin (not implemented yet)" OFF)
if (PARAVIEW_WRAPPING)
    add_subdirectory(paraview_wrapping)
endif(PARAVIEW_WRAPPING)
