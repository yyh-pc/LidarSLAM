cmake_minimum_required(VERSION 3.9)
project(lidar_slam_test VERSION 1.6)

# Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  lidar_slam
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp lidar_slam nav_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

# Add LiDAR SLAM ROS node
add_executable(lidar_slam_test_node
  src/LidarSlamTestNode.cxx
)

target_link_libraries(lidar_slam_test_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS lidar_slam_test_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY params
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
