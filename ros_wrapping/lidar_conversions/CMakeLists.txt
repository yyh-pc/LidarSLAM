cmake_minimum_required(VERSION 3.9)
project(lidar_conversions VERSION 2.0)

# Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

##################
## Dependencies ##
##################

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    pcl_conversions
    sensor_msgs
)

###########
## Build ##
###########

# This package needs LidarPoint definition in LidarSlam lib.
# Sadly, adding lidar_slam dependency will create a circular dependency:
# - lidar_conversions needs a header file from LidarSlam, exported with lidar_slam: build dependency
# - lidar_slam uses lidar_conversions in its launchfiles: exec dependency
# These "soft" circular dependencies (build/exec) are not supported by catkin tools.
# To avoid this, we directly include the correct headers.
include_directories(
  "../../slam_lib/include/"
  "./external_include"
  ${catkin_INCLUDE_DIRS}
)

# Velodyne Lidar
add_executable(velodyne_conversion_node src/VelodyneToLidarNode.cxx)
target_link_libraries(velodyne_conversion_node ${catkin_LIBRARIES})

# Ouster Lidar
add_executable(ouster_conversion_node src/OusterToLidarNode.cxx)
target_link_libraries(ouster_conversion_node ${catkin_LIBRARIES})

# Robosense RSLidar
add_executable(robosense_conversion_node src/RobosenseToLidarNode.cxx)
target_link_libraries(robosense_conversion_node ${catkin_LIBRARIES})

# Livox mid100 Lidar
add_executable(livox_conversion_node src/LivoxToLidarNode.cxx)
target_link_libraries(livox_conversion_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS velodyne_conversion_node robosense_conversion_node ouster_conversion_node livox_conversion_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
