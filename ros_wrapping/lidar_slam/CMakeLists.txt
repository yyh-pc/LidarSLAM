cmake_minimum_required(VERSION 3.9)
project(lidar_slam VERSION 2.0)

# If you're not familiar with catkin, please have a look at these useful links.
# Catkin documentation:
#  https://docs.ros.org/en/api/catkin/html/howto/format2/index.html
# Catkin CMakeLists.txt ROS tutorial:
#  http://wiki.ros.org/catkin/CMakeLists.txt
# CMakeLists.txt template created by catkin_create_pkg:
#  https://github.com/ros-infrastructure/catkin_pkg/blob/master/src/catkin_pkg/templates/CMakeLists.txt.in

# Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "ROS wrapping : No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(BUILD_SLAM_LIB "Build the SLAM library before the ROS package" ON)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  pcl_ros
  pcl_conversions
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
  apriltag_ros
)

# Find catkin macros and libraries
find_package(cv_bridge)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  SlamCommand.msg
  Confidence.msg
)

# Generate service in the 'srv' folder
add_service_files(
  FILES
  save_pc.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
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

set(catkin_dependencies
  roscpp
  tf2_ros
  pcl_ros
  pcl_conversions
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_runtime
  apriltag_ros
)

if (BUILD_SLAM_LIB)
  # Export LidarSlam lib if built at the same time than ROS package
  catkin_package (
    LIBRARIES LidarSlam
    CATKIN_DEPENDS ${catkin_dependencies}
  )
else()
  catkin_package (
    CATKIN_DEPENDS ${catkin_dependencies}
  )
endif()

###############################################
##        Set superbuild dependencies        ##
###############################################

# If lib_name has been installed with the superbuild, set lib_name_DIR to install_path
function(SET_SB_LIB lib_name lib_install_path)
  if (EXISTS ${lib_install_path})
    set("${lib_name}_DIR" ${lib_install_path} CACHE PATH "PATH to ${lib_name} directory")
    message("${lib_name}_DIR set to ${lib_install_path}")
  else()
    message("${lib_name} not found in SuperBuild")
  endif()
endfunction()

# Add path to use superbuild
if (SUPERBUILD_INSTALL_DIR)
  # Set dependencies installed with the superbuild
  SET_SB_LIB(glog "${SUPERBUILD_INSTALL_DIR}/lib/cmake/glog")
  SET_SB_LIB(Eigen3 "${SUPERBUILD_INSTALL_DIR}/share/eigen3/cmake")
  SET_SB_LIB(Ceres "${SUPERBUILD_INSTALL_DIR}/lib/cmake/Ceres")
  SET_SB_LIB(nanoflann "${SUPERBUILD_INSTALL_DIR}/share/nanoflann/cmake")
  SET_SB_LIB(Boost "${SUPERBUILD_INSTALL_DIR}/lib/cmake/Boost-1.76.0")
  if (EXISTS "${SUPERBUILD_INSTALL_DIR}/install/include/boost")
    set(BOOST_ROOT  "${SUPERBUILD_INSTALL_DIR}/install")
    message("BOOST_ROOT set to ${SUPERBUILD_INSTALL_DIR}/install")
  else()
    message("BOOST_ROOT not found in SuperBuild")
  endif()
  SET_SB_LIB(GTSAM "${SUPERBUILD_INSTALL_DIR}/lib/cmake/GTSAM")
  SET_SB_LIB(g2o "${SUPERBUILD_INSTALL_DIR}/lib/cmake/g2o")
  if (EXISTS "${SUPERBUILD_INSTALL_DIR}/include/pcl-1.10")
    message(FATAL_ERROR "PCL was built within the superbuild : this would cause runtime issues so the process is aborted.
    Rebuild the superbuild from scratch without PCL (`-DINSTALL_PCL=OFF`) and rebuild the ROS package afterwards.")
  endif()
  if (NOT BUILD_SLAM_LIB)
    SET_SB_LIB(LidarSlam "${SUPERBUILD_INSTALL_DIR}/lib/cmake/LidarSlam")
  endif()
endif()

###########
## Build ##
###########

# Build the LidarSlam library or use an external one
if (BUILD_SLAM_LIB)
  add_subdirectory(../.. ${CMAKE_BINARY_DIR}/slam)
  set(LidarSlam_target LidarSlam)
else()
  find_package(LidarSlam REQUIRED)
  set(LidarSlam_target LidarSlam::LidarSlam)
endif()

# Add LiDAR SLAM ROS node
add_executable(lidar_slam_node
  src/LidarSlamNode.cxx
  src/LidarSlamNode_main.cxx
)

# See here : https://answers.ros.org/question/173342/compile-twice-to-see-custom-message/
add_dependencies(lidar_slam_node lidar_slam_generate_messages_cpp)

if(cv_bridge_FOUND)
  message("ROS wrapping : cv_bridge was found, camera interface is built")
  list(APPEND optional_INCLUDE_DIRS ${cv_bridge_INCLUDE_DIRS})
  list(APPEND optional_LIBRARIES ${cv_bridge_LIBRARIES})
  # Anything else you need to do to enable use of the optional dep, like add definitions
  target_compile_definitions(lidar_slam_node PUBLIC "-DUSE_CV_BRIDGE")
else()
  message("ROS wrapping : cv_bridge was not found, camera interface cannot be used")
endif()

target_link_libraries(lidar_slam_node
  ${LidarSlam_target}
  ${catkin_LIBRARIES}
  ${optional_LIBRARIES}
)

target_include_directories(lidar_slam_node PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${optional_INCLUDE_DIRS}
)

# Add aggregation node
add_executable(aggregation_node
  src/AggregationNode.cxx
)

# Compile the lidar_slam interfaces before the node
add_dependencies(aggregation_node lidar_slam_generate_messages_cpp)

target_link_libraries(aggregation_node
  ${LidarSlam_target}
  ${catkin_LIBRARIES}
)

target_include_directories(aggregation_node PUBLIC
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(TARGETS lidar_slam_node aggregation_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY params
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
