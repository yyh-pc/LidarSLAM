cmake_minimum_required(VERSION 2.8.3)
project(gps_conversions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geodesy
  tf2_ros
  nav_msgs
  gps_common
  )

catkin_package(
  )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gps_to_utm src/GpsToUtmNode.cxx)
target_link_libraries(gps_to_utm ${catkin_LIBRARIES})

add_executable(odom_to_gps src/OdomToGpsNode.cxx)
target_link_libraries(odom_to_gps ${catkin_LIBRARIES})