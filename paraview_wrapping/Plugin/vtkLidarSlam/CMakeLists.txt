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
  SOURCES ${lidarslam_sources} # Will not trigger a PV wrapping
  CLASSES ${lidarslam_classes} # Those will be PV ClientServer wrapped #shorcut for sources x.cxx x.h
  HEADERS ${lidarslam_headers} # Those will be PV ClientServer wrapped
)
paraview_add_server_manager_xmls(
  MODULE vtkLidarSlam
  XMLS  ${lidarslam_xml}
)

target_link_libraries(vtkLidarSlam PRIVATE LidarSlam)
