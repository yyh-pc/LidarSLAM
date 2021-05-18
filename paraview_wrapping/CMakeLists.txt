# Find ParaView
find_package(ParaView REQUIRED)
message(STATUS "PARAVIEW_USE_FILE: ${PARAVIEW_USE_FILE}")
include(${PARAVIEW_USE_FILE})

# construct the plugin
set(ServerManagerSources
  vtkSlam.cxx
  vtkSlam.h
  vtkSlamManager.cxx
  vtkSlamManager.h
  vtkSpinningSensorKeypointExtractor.cxx
  vtkSpinningSensorKeypointExtractor.h)

set(ServerManagerXML
  Slam.xml)

add_paraview_plugin(LidarSlamPlugin "1.4"
  SERVER_MANAGER_XML ${ServerManagerXML}
  SERVER_MANAGER_SOURCES ${ServerManagerSources}
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)

target_link_libraries(LidarSlamPlugin PUBLIC LidarSlam)

install(TARGETS LidarSlamPlugin
        RUNTIME DESTINATION ${SLAM_INSTALL_PARAVIEW_PLUGIN_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_PARAVIEW_PLUGIN_DIR}
        COMPONENT Runtime)