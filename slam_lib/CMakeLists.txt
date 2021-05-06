# If G2O is available, compile pose graph optimization stuff
if (g2o_FOUND)
  set(SLAM_g2o_headers include/LidarSlam/PoseGraphOptimization.h)
  set(SLAM_g2o_sources src/PoseGraphOptimization.cxx)
  set(g2o_targets g2o::core g2o::types_slam3d)
  add_definitions(-DUSE_G2O=1)
endif()

set(LidarSlam_PUBLIC_HEADERS
  include/LidarSlam/CeresCostFunctions.h
  include/LidarSlam/Enums.h
  include/LidarSlam/GlobalTrajectoriesRegistration.h
  include/LidarSlam/KDTreePCLAdaptor.h
  include/LidarSlam/KeypointsMatcher.h
  include/LidarSlam/LidarPoint.h
  include/LidarSlam/LocalOptimizer.h
  include/LidarSlam/MotionModel.h
  include/LidarSlam/PointCloudStorage.h
  include/LidarSlam/RollingGrid.h
  include/LidarSlam/SensorConstraints.h
  include/LidarSlam/Slam.h
  include/LidarSlam/SpinningSensorKeypointExtractor.h
  include/LidarSlam/Transform.h
  include/LidarSlam/Utilities.h
  ${SLAM_g2o_headers}
)

# Generate export symbols on Windows to use this lib
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_library(LidarSlam
  src/GlobalTrajectoriesRegistration.cxx
  src/KeypointsMatcher.cxx
  src/LocalOptimizer.cxx
  src/MotionModel.cxx
  src/RollingGrid.cxx
  src/SensorConstraints.cxx
  src/Slam.cxx
  src/SpinningSensorKeypointExtractor.cxx
  src/Transform.cxx
  src/Utilities.cxx
  ${LidarSlam_PUBLIC_HEADERS}
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

set_target_properties(LidarSlam PROPERTIES PUBLIC_HEADER "${LidarSlam_PUBLIC_HEADERS}")

target_include_directories(LidarSlam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS LidarSlam
        RUNTIME DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        ARCHIVE DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        PUBLIC_HEADER DESTINATION ${SLAM_INSTALL_INCLUDE_DIR}/LidarSlam
        COMPONENT Runtime)