# If G2O is available, compile pose graph optimization stuff
if (g2o_FOUND)
  set(SLAM_g2o_sources src/PoseGraphOptimizer.cxx)
  set(g2o_targets g2o::core g2o::types_slam3d g2o::types_slam3d_addons)
  add_definitions(-DUSE_G2O=1)
endif()

# Generate export symbols on Windows to use this lib
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(LidarSlam
  src/ConfidenceEstimators.cxx
  src/KeypointsMatcher.cxx
  src/LocalOptimizer.cxx
  src/MotionModel.cxx
  src/RollingGrid.cxx
  src/ExternalSensorManagers.cxx
  src/Slam.cxx
  src/SpinningSensorKeypointExtractor.cxx
  src/Utilities.cxx
  src/VoxelGrid.cxx
  ${SLAM_g2o_sources}
)

target_link_libraries(LidarSlam
  PUBLIC
    nanoflann::nanoflann
    ceres
    ${PCL_LIBRARIES}
    ${g2o_targets}
  PRIVATE
    ${Eigen3_target}
    ${OpenMP_target}
)

target_include_directories(LidarSlam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Prefer rpath over runpath to bypass LD_LIBRARY_PATH at runtime (which can be set by ROS notably)
# See https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10/47243544#47243544
# and https://stackoverflow.com/questions/52018092/how-to-set-rpath-and-runpath-with-gcc-ld
# for more info
if (UNIX AND NOT APPLE)
  target_link_options(LidarSlam PUBLIC "-Wl,--disable-new-dtags")
endif()

install(TARGETS LidarSlam
        RUNTIME DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        ARCHIVE DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        PUBLIC_HEADER DESTINATION ${SLAM_INSTALL_INCLUDE_DIR}/LidarSlam
        COMPONENT Runtime)
