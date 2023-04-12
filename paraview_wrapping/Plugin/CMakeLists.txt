# Create Plugin
paraview_add_plugin(LidarSlamPlugin
  VERSION ${CMAKE_PROJECT_VERSION}
  MODULES vtkLidarSlam
  MODULE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/vtkLidarSlam/vtk.module"
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
)
