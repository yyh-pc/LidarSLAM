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

set(plugin_name LidarSlam)

ADD_PARAVIEW_PLUGIN(
  ${plugin_name} "1.0"
  SERVER_MANAGER_XML ${ServerManagerXML}
  SERVER_MANAGER_SOURCES ${ServerManagerSources}
)

target_link_libraries(${plugin_name} PUBLIC slamlib ${VTK_LIBRARIES} )
