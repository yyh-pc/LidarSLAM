cmake_minimum_required(VERSION 3.9)
project(gps_conversions VERSION 1.4)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

##################
## Dependencies ##
##################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geodesy
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  gps_common
)

catkin_package(
  CATKIN_DEPENDS roscpp geodesy tf2_ros tf2_geometry_msgs nav_msgs gps_common
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gps_to_utm src/GpsToUtmNode.cxx)
target_link_libraries(gps_to_utm ${catkin_LIBRARIES})

add_executable(odom_to_gps src/OdomToGpsNode.cxx)
target_link_libraries(odom_to_gps ${catkin_LIBRARIES})

add_executable(navsatfix_to_gpsfix src/NavSatFixToGpsFixNode.cxx)
target_link_libraries(navsatfix_to_gpsfix ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS gps_to_utm odom_to_gps navsatfix_to_gpsfix
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
