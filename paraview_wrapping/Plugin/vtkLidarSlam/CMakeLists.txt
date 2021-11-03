set(lidarslam_sources)

set(lidarslam_classes
  vtkSlam
  vtkSlamManager
  vtkSpinningSensorKeypointExtractor
)

set(lidarslam_headers)
set(lidarslam_templates)

set(lidarslam_xml
  Slam.xml
)

# Create Module
vtk_module_add_module(vtkLidarSlam
  FORCE_STATIC # Using FORCE_STATIC build the vtk module statically into the plugin library, to avoid confusion when loading                
  SOURCES ${lidarslam_sources} # Will not trigger a PV wrapping
  CLASSES ${lidarslam_classes} # Those will be PV ClientServer wrapped #shorcut for sources x.cxx x.h  
  HEADERS ${lidarslam_headers} # Those will be PV ClientServer wrapped
  #TEMPLATES ${lidarslam_templates} # Those will be PV ClientServer wrapped
  #PRIVATE_* variants exists and will not be wrapped, CLASSES are always PRIVATE/NOT-WRAPPED
)
paraview_add_server_manager_xmls(
  MODULE vtkLidarSlam
  XMLS  ${lidarslam_xml}
)

target_link_libraries(vtkLidarSlam PUBLIC LidarSlam)
target_include_directories(vtkLidarSlam
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

