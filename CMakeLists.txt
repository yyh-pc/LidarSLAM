cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

project(LidarSlam LANGUAGES CXX)

#-------------------------
#  CMake parameters
#-------------------------

set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ standard")
message(STATUS "Building LidarSlam with C++${CMAKE_CXX_STANDARD} standard")

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "MinSizeRel" "RelWithDebInfo" "Release")
endif()

# Install parameters
set(SLAM_INSTALL_LIBRARY_DIR "lib"     CACHE STRING "Directory where to install LidarSlam libraries")
set(SLAM_INSTALL_INCLUDE_DIR "include" CACHE STRING "Directory where to install LidarSlam header files")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${SLAM_INSTALL_LIBRARY_DIR}")

#-------------------------
#  Find dependencies
#-------------------------

find_package(nanoflann REQUIRED)

# Find Eigen3. If it defines the target, this is used. If not,
# fall back to the using the module form.
# See https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html for details
find_package(Eigen3 REQUIRED)
if (TARGET Eigen3::Eigen)
  message(STATUS "Lidar SLAM : using Eigen3::Eigen target (version ${Eigen3_VERSION})")
  set(Eigen3_target Eigen3::Eigen)
else()
  message(STATUS "Lidar SLAM : using Eigen3 header files (version ${EIGEN3_VERSION})")
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

find_package(Ceres REQUIRED)

find_package(PCL REQUIRED COMPONENTS common kdtree registration io)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find optional g2o (only used for pose graph optimization)
option(ENABLE_g2o "Use G2O, necessary for pose graph optimization." ON)
if (ENABLE_g2o)
  find_package(g2o QUIET)
  if (NOT g2o_FOUND)
    message("Lidar SLAM : G2O requested but not found, pose graph optimization will be ignored.")
  endif(NOT g2o_FOUND)
endif (ENABLE_g2o)

# Find optional OpenMP
find_package(OpenMP)
if(TARGET OpenMP::OpenMP_CXX)
  message(STATUS "Lidar SLAM : using OpenMP ${OpenMP_CXX_VERSION} target")
  set(OpenMP_target OpenMP::OpenMP_CXX)
elseif(OpenMP_FOUND)
  message(STATUS "Lidar SLAM : using OpenMP ${OpenMP_VERSION} flags")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
  message("Lidar SLAM : OpenMP not found")
endif()

#-------------------------
#  Build and install
#-------------------------

# Build core SLAM lib
add_subdirectory(slam_lib)

# Build optional ParaView plugin
option(SLAM_PARAVIEW_PLUGIN "Build a Paraview plugin to wrap SLAM" OFF)
if (SLAM_PARAVIEW_PLUGIN)
  set(SLAM_INSTALL_PARAVIEW_PLUGIN_DIR ${SLAM_INSTALL_LIBRARY_DIR} CACHE STRING "Directory where to install Paraview LidarSlamPlugin")
  add_subdirectory(paraview_wrapping)
endif(SLAM_PARAVIEW_PLUGIN)

# To build/use ROS wrapping, just put this entire directory in the catkin src
# workspace and run `catkin_make` or `catkin build`.