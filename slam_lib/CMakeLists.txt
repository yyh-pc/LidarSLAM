# If G2O is available, compile pose graph optimization stuff
if (g2o_FOUND)
  set(SLAM_g2o_headers PoseGraphOptimization.h)
  set(SLAM_g2o_sources PoseGraphOptimization.cxx ${SLAM_g2o_headers})
  set(g2o_targets g2o::core g2o::types_slam3d)
  add_compile_definitions(USE_G2O=1)
endif(g2o_FOUND)

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
  GlobalTrajectoriesRegistration.cxx
  GlobalTrajectoriesRegistration.h
  ${SLAM_g2o_sources}
  )

target_link_libraries(slamlib
  PUBLIC
    ${PCL_LIBRARIES}
    nanoflann::nanoflann
    ${g2o_targets}
  PRIVATE
    ceres
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
  GlobalTrajectoriesRegistration.h
  ${SLAM_g2o_headers}
  )

set_target_properties(slamlib PROPERTIES
  PUBLIC_HEADER "${HEADER_FILES}")

target_include_directories(slamlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS slamlib
        RUNTIME DESTINATION ${SLAM_INSTALL_RUNTIME_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        COMPONENT Runtime)