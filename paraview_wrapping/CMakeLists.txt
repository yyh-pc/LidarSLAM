# Find ParaView
if(NOT ParaView_BINARY_DIR)
  find_package(ParaView REQUIRED)
  message(STATUS "PARAVIEW_USE_FILE: ${PARAVIEW_USE_FILE}")
  include(${PARAVIEW_USE_FILE})
endif()
include(ParaViewPlugins)

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

ADD_PARAVIEW_PLUGIN(LidarSlamPlugin "1.0"
  SERVER_MANAGER_XML ${ServerManagerXML}
  SERVER_MANAGER_SOURCES ${ServerManagerSources}
)

target_link_libraries(LidarSlamPlugin PUBLIC slamlib ${VTK_LIBRARIES})

install(TARGETS LidarSlamPlugin
        RUNTIME DESTINATION ${SLAM_INSTALL_RUNTIME_DIR}
        LIBRARY DESTINATION ${SLAM_INSTALL_LIBRARY_DIR}
        COMPONENT Runtime)