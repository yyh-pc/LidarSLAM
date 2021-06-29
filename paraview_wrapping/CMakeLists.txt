set(SLAM_INSTALL_PARAVIEW_PLUGIN_DIR ${SLAM_INSTALL_LIBRARY_DIR} CACHE STRING "Directory where to install Paraview LidarSlamPlugin")

set(ServerManagerSources
  vtkSlam.cxx
  vtkSlam.h
  vtkSlamManager.cxx
  vtkSlamManager.h
  vtkSpinningSensorKeypointExtractor.cxx
  vtkSpinningSensorKeypointExtractor.h)

set(ServerManagerXML
  Slam.xml)

add_paraview_plugin(LidarSlamPlugin ${PROJECT_VERSION}
  SERVER_MANAGER_XML ${ServerManagerXML}
  SERVER_MANAGER_SOURCES ${ServerManagerSources}
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)

target_link_libraries(LidarSlamPlugin PUBLIC LidarSlam)

install(TARGETS LidarSlamPlugin
        RUNTIME DESTINATION ${SLAM_INSTALL_PARAVIEW_PLUGIN_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_PARAVIEW_PLUGIN_DIR}
        COMPONENT Runtime)