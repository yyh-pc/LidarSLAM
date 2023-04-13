cmake_minimum_required(VERSION 3.16)
project(LidarSlamPlugin C CXX)

paraview_plugin_scan(
  PLUGIN_FILES      "${CMAKE_CURRENT_LIST_DIR}/Plugin/paraview.plugin"
  ENABLE_BY_DEFAULT ON
  PROVIDES_PLUGINS  plugins
  REQUIRES_MODULES  required_modules)

foreach (module IN LISTS required_modules)
  if (NOT TARGET "${module}")
    message(FATAL_ERROR "LidarSlamPlugin missing required module ${module}")
  endif ()
endforeach ()

paraview_plugin_build(
  HEADERS_DESTINATION   "${CMAKE_INSTALL_INCLUDEDIR}"
  RUNTIME_DESTINATION   "${CMAKE_INSTALL_BINDIR}"
  LIBRARY_DESTINATION   "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY_SUBDIRECTORY  "slam/plugins"
  PLUGINS_FILE_NAME     "slam.plugins.xml"
  TARGET                LidarSlam::paraview_wrapping
  INSTALL_EXPORT        LidarSlamLib
  TARGET_COMPONENT      Runtime
  CMAKE_DESTINATION     "${CMAKE_INSTALL_LIBDIR}/cmake/LidarSlam"
  PLUGINS               ${plugins}
  AUTOLOAD              LidarSlamPlugin
)
