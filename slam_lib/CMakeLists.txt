add_library(slamlib SHARED
  LidarPoint.h
  Transform.cxx
  Transform.h
  RollingGrid.cxx
  RollingGrid.h
  Slam.cxx
  Slam.h
  SpinningSensorKeypointExtractor.cxx
  SpinningSensorKeypointExtractor.h
  MotionModel.cxx
  MotionModel.h
  PointCloudStorage.h
  PoseGraphOptimization.cxx
  PoseGraphOptimization.h
  GlobalTrajectoriesRegistration.cxx
  GlobalTrajectoriesRegistration.h
  )

target_link_libraries(slamlib
  PUBLIC
    ${PCL_LIBRARIES}
    nanoflann::nanoflann
    g2o::core
  PRIVATE
    ceres
    g2o::types_slam3d
    ${Eigen3_target}
    ${OpenMP_target}
  )

set(HEADER_FILES
  LidarPoint.h
  Transform.h
  RollingGrid.h
  Slam.h
  SpinningSensorKeypointExtractor.h
  MotionModel.h
  PointCloudStorage.h
  PoseGraphOptimization.h
  GlobalTrajectoriesRegistration.h
  )

set_target_properties(slamlib PROPERTIES
  PUBLIC_HEADER "${HEADER_FILES}")

target_include_directories(slamlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS slamlib
        RUNTIME DESTINATION ${SLAM_INSTALL_RUNTIME_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        COMPONENT Runtime)