cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

project(LidarSlam
        LANGUAGES CXX
        VERSION 2.0)

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

include(GNUInstallDirs) # Required for CMAKE_INSTALL_VARIABLES

# Runtime parameters
# /!\ warning : must be done before target declaration
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

# MSVC enforce MT
if(WIN32 AND MSVC)
	set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

# Boost do not use STATIC libs (Superbuild like), coherent with BUILD_SHARED_LIBS ON
set(Boost_USE_STATIC_LIBS OFF)

#-------------------------
#  Find dependencies
#-------------------------

find_package(Boost REQUIRED)

# If the Paraview plugin needs to be compiled, we must find Paraview before PCL.
# Otherwise, if PCL is built with pcl_visualization module and uses another VTK
# than the one provided by Paraview, it may override the correct include
# directories and libraries.
option(SLAM_PARAVIEW_PLUGIN "Build a Paraview plugin to wrap SLAM" OFF)
if (SLAM_PARAVIEW_PLUGIN)
  # Find ParaView
  find_package(ParaView REQUIRED)
  # As the PV plugin is a shared library, we also need to ensure that the core
  # SLAM lib is compatible with position independent code.
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

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
endif()

find_package(Ceres REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io octree geometry)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
#We do not want VTK for now, FIX WIN32 command too long issue
#PCL is linked finds VTK in paraview/src rather than paraview/build
list(FILTER PCL_LIBRARIES EXCLUDE REGEX "VTK::.*")

# Find optional g2o (only used for pose graph optimization)
option(ENABLE_g2o "Use G2O, necessary for pose graph optimization." ON)
if (ENABLE_g2o)
  find_package(g2o QUIET)
  if (NOT g2o_FOUND)
    message("Lidar SLAM : G2O requested but not found, pose graph optimization will be ignored.")
  endif()
endif()

# Find optional gtsam (only used for pose IMU raw data integration)
option(ENABLE_GTSAM "Use GTSAM, necessary for raw IMU data integration." ON)
if (ENABLE_GTSAM)
  find_package(GTSAM QUIET)
  if (NOT GTSAM_FOUND)
    message("Lidar SLAM : GTSAM requested but not found, IMU raw data will be ignored.")
  endif()
endif()

option(ENABLE_OpenCV "Use OpenCV, necessary for camera integration." ON)
if (ENABLE_OpenCV)
  find_package(OpenCV QUIET COMPONENTS core)
  if (NOT OpenCV_FOUND)
    message("Lidar SLAM : OpenCV requested but not found, camera will be ignored.")
  endif()
endif()

# Find optional teaserpp (only used for automatic detection of loop closure)
option(ENABLE_teaserpp "Use TEASER++, necessary for automatic detection of loop closure." ON)
if (ENABLE_teaserpp)
  find_package(teaserpp QUIET)
  if (NOT teaserpp_FOUND)
    message("Lidar SLAM : TEASER++ requested but not found, automatic loop closure detection will be ignored.")
  endif()
endif()

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
if (SLAM_PARAVIEW_PLUGIN)
  add_subdirectory(paraview_wrapping)
endif()

# To build/use ROS wrapping, just put this entire directory in the catkin src
# workspace and run `catkin_make` or `catkin build`.

#-------------------------
#  Install export files
#-------------------------

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/LidarSlamConfigVersion.cmake"
  VERSION "${CMAKE_PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${CMAKE_BINARY_DIR}/LidarSlamConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LidarSlam"
)

# Is automatically exported in paraview_wrapping with LidarSlamPlugin
if (NOT SLAM_PARAVIEW_PLUGIN)
  install(
    EXPORT LidarSlamLib
    FILE "LidarSlamLib-targets.cmake"
    NAMESPACE LidarSlam::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LidarSlam"
  )
endif()

install(
  FILES "${CMAKE_BINARY_DIR}/LidarSlamConfig.cmake"
        "${CMAKE_BINARY_DIR}/LidarSlamConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LidarSlam"
)
