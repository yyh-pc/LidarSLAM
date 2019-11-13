add_library(slamlib
  LidarPoint.h
  Transform.cxx
  Transform.h
  Slam.cxx
  Slam.h
  SpinningSensorKeypointExtractor.cxx
  SpinningSensorKeypointExtractor.h
  MotionModel.cxx
  MotionModel.h
  PoseGraphOptimization.cxx
  PoseGraphOptimization.h
  )

target_link_libraries(slamlib
  PUBLIC
    ${PCL_LIBRARIES}
    nanoflann::nanoflann
    g2o::core
  PRIVATE
    ceres
    Eigen3::Eigen
    g2o::types_slam3d
  )

set(HEADER_FILES
  LidarPoint.h
  Transform.h
  Slam.h
  SpinningSensorKeypointExtractor.h
  MotionModel.h
  PoseGraphOptimization.h
  )

set_target_properties(slamlib PROPERTIES
  PUBLIC_HEADER "${HEADER_FILES}")

target_include_directories(slamlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})