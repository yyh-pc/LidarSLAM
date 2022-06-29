# If G2O is available, compile pose graph optimization stuff
if (g2o_FOUND)
  set(SLAM_g2o_sources src/PoseGraphOptimizer.cxx)
  set(g2o_targets g2o::core g2o::types_slam3d g2o::types_slam3d_addons)
  message("Lidar SLAM : G2O was found, pose graph API compiled")
endif()

# If GTSAM is available, compile IMU stuff
if (GTSAM_FOUND)
  set(gtsam_targets gtsam)
  message("Lidar SLAM : GTSAM was found, IMU preintegration compiled")
endif()

# If OpenCV is available, compile camera stuff
if (OpenCV_FOUND)
  set(opencv_targets opencv_core opencv_highgui opencv_imgproc opencv_video)
  message("Lidar SLAM : OPENCV was found, camera API compiled")
endif()

# Generate export symbols on Windows to use this lib
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(LidarSlam
  src/ConfidenceEstimators.cxx
  src/KeypointsMatcher.cxx
  src/LocalOptimizer.cxx
  src/RollingGrid.cxx
  src/ExternalSensorManagers.cxx
  src/Slam.cxx
  src/SpinningSensorKeypointExtractor.cxx
  src/Utilities.cxx
  src/VoxelGrid.cxx
  src/InterpolationModels.cxx
  ${SLAM_g2o_sources}
)

target_link_libraries(LidarSlam
  PUBLIC
    nanoflann::nanoflann
    ceres
    ${PCL_LIBRARIES}
    ${g2o_targets}
    ${gtsam_targets}
    ${opencv_targets}
  PRIVATE
    ${Eigen3_target}
    ${OpenMP_target}
)

# If G2O is available, set a precompilation variable
if (g2o_FOUND)
  set(g2o_def USE_G2O=1)
endif()

# If GTSAM is available, set a precompilation variable
if (GTSAM_FOUND)
  set(gtsam_def USE_GTSAM=1)
endif()

# If OpenCV is available, set a precompilation variable
if (OpenCV_FOUND)
  set(opencv_def USE_OPENCV=1)
endif()

target_compile_definitions(LidarSlam PUBLIC ${g2o_def} ${gtsam_def} ${opencv_def})

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
