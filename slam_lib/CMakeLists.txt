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

# If TEASER is available, compile automatic loop closure detection by teaserpp stuff
if (teaserpp_FOUND)
  set(teaserpp_targets teaserpp::teaser_registration)
  message("Lidar SLAM : TEASER++ was found, automatic loop closure detection enabled")
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
    # nanoflann::nanoflann
    ceres
    ${PCL_LIBRARIES} # Add boost targets too
    ${g2o_targets}
    ${gtsam_targets}
    ${opencv_targets}
    ${teaserpp_targets}
  PRIVATE
    ${Eigen3_target}
    ${OpenMP_target}
)

target_include_directories(LidarSlam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>
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

# If TEASERPP is available, set a precompilation variable
if (teaserpp_FOUND)
  set(teaserpp_def USE_TEASERPP=1)
endif()

# NOMINMAX needs to be defined to remove macro file windows.h
# this file causes error when using min max from std
# windows.h is included by some dependencies (e.g. glog)
if (WIN32)
  set(windows_def NOMINMAX)
  message("Warning: NOMINMAX is defined for windows to solve dependencies windows.h inclusion.")
endif()

target_compile_definitions(LidarSlam
  PUBLIC
    ${g2o_def}
    ${gtsam_def}
    ${opencv_def}
    ${teaserpp_def}
    ${windows_def}
)

# Prefer rpath over runpath to bypass LD_LIBRARY_PATH at runtime (which can be set by ROS notably)
# See https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10/47243544#47243544
# and https://stackoverflow.com/questions/52018092/how-to-set-rpath-and-runpath-with-gcc-ld
# for more info
if (UNIX AND NOT APPLE)
  target_link_options(LidarSlam PUBLIC "-Wl,--disable-new-dtags")
endif()

# Install library

set(target_public_header
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/CeresCostFunctions.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/ConfidenceEstimators.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/Enums.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/ExternalSensorManagers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/InterpolationModels.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/KDTreePCLAdaptor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/KeypointsMatcher.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/LidarPoint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/LocalOptimizer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/PointCloudStorage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/PoseGraphOptimizer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/RollingGrid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/Slam.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/SpinningSensorKeypointExtractor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/State.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/Utilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarSlam/VoxelGrid.h
)

set_target_properties(LidarSlam PROPERTIES PUBLIC_HEADER "${target_public_header}")

install(TARGETS LidarSlam
        EXPORT LidarSlamLib
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/LidarSlam
        COMPONENT Runtime
)
