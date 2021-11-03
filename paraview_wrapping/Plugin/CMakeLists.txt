if(NOT LidarSlam_VERSION)
  message(FATAL_ERROR "LidarSlam_VERSION not defined")
endif()

# Create Plugin
paraview_add_plugin(LidarSlamPlugin 
  VERSION ${LidarSlam_VERSION}
  MODULES vtkLidarSlam
  MODULE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/vtkLidarSlam/vtk.module"
  DOCUMENTATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc/"
  MODULE_ARGS
    LICENSE_DESTINATION "${LV_INSTALL_INCLUDE_DIR}" #wip Temporary fix
    TEST_DIRECTORY_NAME "NONE" # Do not let vtk_module build the default 'Testing' subdir, it complexifies paths
)
