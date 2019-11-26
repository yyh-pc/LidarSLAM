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
  GlobalTrajectoriesRegistration.cxx
  GlobalTrajectoriesRegistration.h
  )

target_link_libraries(slamlib
  PUBLIC
    ${PCL_LIBRARIES}
    nanoflann::nanoflann
  PRIVATE
    ceres
    Eigen3::Eigen
  )

set(HEADER_FILES
  LidarPoint.h
  Transform.h
  Slam.h
  SpinningSensorKeypointExtractor.h
  MotionModel.h
  GlobalTrajectoriesRegistration.h
  )

set_target_properties(slamlib PROPERTIES
  PUBLIC_HEADER "${HEADER_FILES}")

target_include_directories(slamlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})