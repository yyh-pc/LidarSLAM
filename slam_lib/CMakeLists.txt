add_library(slamlib SHARED
    LidarPoint.h    
    Slam.cxx
    Slam.h
    SpinningSensorKeypointExtractor.cxx
    SpinningSensorKeypointExtractor.h
    MotionModel.cxx
    MotionModel.h
    )
    
target_link_libraries(slamlib
    PUBLIC
      PCL::PCL
    PRIVATE
      ceres
      Eigen3::Eigen
      nanoflann::nanoflann
    )

set(HEADER_FILES
    LidarPoint.h
    Slam.h
    SpinningSensorKeypointExtractor.h
    MotionModel.h
)
target_include_directories(slamlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(slamlib PROPERTIES
    PUBLIC_HEADER "${HEADER_FILES}")
