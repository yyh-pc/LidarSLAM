cmake_minimum_required(VERSION 3.14)
project(Superbuild)

# -----------------------------------------------------------------------------------------------
# Select Options
option(WITH_EIGEN "Whether to install Eigen3" ON)
option(WITH_NANOFLANN "Whether to install nanoflann" ON)
option(WITH_PCL "Whether to install PCL" ON)
option(WITH_CERES "Whether to install Ceres" ON)
option(WITH_G2O "Whether to install G2O" OFF)

# -----------------------------------------------------------------------------------------------
# Read CMAKE Arguments

# -- Set c++ standard
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "install prefix" FORCE)
    message(INFO " No install directory specified, set it to ${CMAKE_INSTALL_PREFIX} as default")
endif ()

set(GENERIC_PROJECTS_ARGS
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
  -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
  -DCMAKE_PREFIX_PATH:STRING=${CMAKE_PREFIX_PATH}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DBUILD_SHARED_LIBS:BOOL=ON
  -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
  -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
  )


include(ExternalProject)

set(LOG_PREFIX " [Superbuild] -- [Generation] -- ")

# -----------------------------------------------------------------------------------------------
# INSTALL Import files
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/superbuild_import.cmake.in ${CMAKE_INSTALL_PREFIX}/superbuild_import.cmake @ONLY)

# -----------------------------------------------------------------------------------------------
# INSTALL Eigen3
if (WITH_EIGEN)

    message(INFO "${LOG_PREFIX} Generating Eigen3 dependency")
    ExternalProject_Add(
            Eigen3_ext

            GIT_REPOSITORY https://gitlab.com/libeigen/eigen
            GIT_TAG 3.3.7
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DBUILD_TESTING:BOOL=OFF
                ${GENERIC_PROJECTS_ARGS}
            )

endif ()

# -----------------------------------------------------------------------------------------------
# INSTALL PCL
if (WITH_PCL)

    # INSTALL Boost
    message(INFO "${LOG_PREFIX} Generating Boost dependency")
    ExternalProject_Add(
            Boost_ext
            GIT_REPOSITORY https://github.com/boostorg/boost.git
            GIT_TAG boost-1.71.0
            GIT_SHALLOW 1
            BUILD_IN_SOURCE True # needed as Boost is meant to be built in source
            CONFIGURE_COMMAND <SOURCE_DIR>/bootstrap.sh --prefix=${CMAKE_INSTALL_PREFIX}
            BUILD_COMMAND <SOURCE_DIR>/b2 --without-python
            INSTALL_COMMAND <SOURCE_DIR>/b2 install --without-python
    )

    # INSTALL PCL
    message(INFO "${LOG_PREFIX} Generating PCL dependency")
    ExternalProject_Add(
            PCL_ext
            DEPENDS Boost_ext Eigen3_ext

            GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl
            GIT_TAG pcl-1.10.0
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DBUILD_common:BOOL=ON
                -DBUILD_geometry:BOOL=ON
                -DBUILD_io:BOOL=ON
                -DBUILD_octree:BOOL=ON
                -DPCL_ONLY_CORE_POINT_TYPES:BOOL=ON
                -DBUILD_examples:BOOL=OFF
                -DBUILD_apps:BOOL=OFF
                -DBUILD_tools:BOOL=OFF
                -DBUILD_global_tests:BOOL=OFF
                -DBUILD_stereo:BOOL=OFF
                -DBUILD_surface:BOOL=OFF
                -DBUILD_tracking:BOOL=OFF
                -DBUILD_ml:BOOL=OFF
                -DBUILD_keypoints:BOOL=OFF
                -DBUILD_outofcore:BOOL=OFF
                -DBUILD_people:BOOL=OFF
                -DBUILD_2d:BOOL=OFF
                -DBUILD_CUDA:BOOL=OFF
                -DBUILD_GPU:BOOL=OFF
                -DBUILD_kdtree:BOOL=OFF
                -DBUILD_search:BOOL=OFF
                -DBUILD_sample_consensus:BOOL=OFF
                -DBUILD_features:BOOL=OFF
                -DBUILD_filters:BOOL=OFF
                -DBUILD_registration:BOOL=OFF
                -DBUILD_recognition:BOOL=OFF
                -DBUILD_segmentation:BOOL=OFF
                -DBUILD_simulation:BOOL=OFF
                -DBUILD_visualization:BOOL=OFF
                -DWITH_QT:BOOL=OFF
                -DWITH_OPENNI=OFF
                -DWITH_OPENNI2=OFF
                -DWITH_VTK:BOOL=OFF
                -DWITH_LIBUSB:BOOL=OFF
                -DWITH_PNG:BOOL=OFF
                -DWITH_QHULL:BOOL=OFF
                -DWITH_RSSDK:BOOL=OFF
                -DWITH_RSSDK2:BOOL=OFF
                -DWITH_OPENGL:BOOL=OFF
                -DEIGEN_INCLUDE_DIR:PATH=${CMAKE_INSTALL_PREFIX}/include/eigen3/
                ${GENERIC_PROJECTS_ARGS}
            )

endif ()

# -----------------------------------------------------------------------------------------------
# INSTALL nanoflann
if (WITH_NANOFLANN)

    message(INFO "${LOG_PREFIX} Generating nanoflann dependency")
    ExternalProject_Add(
            nanoflann_ext
            DEPENDS Eigen3_ext

            GIT_REPOSITORY https://github.com/jlblancoc/nanoflann
            GIT_TAG v1.4.2
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DNANOFLANN_BUILD_TESTS:BOOL=OFF
                -DNANOFLANN_BUILD_EXAMPLES:BOOL=OFF
                -DEigen3_DIR:FILEPATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake
                -DEIGEN_INCLUDE_DIR:PATH=${CMAKE_INSTALL_PREFIX}/include/eigen3/
                ${GENERIC_PROJECTS_ARGS}
            )

endif ()

# -----------------------------------------------------------------------------------------------
# INSTALL Ceres
if (WITH_CERES)

    # Install glog
    message(INFO "${LOG_PREFIX} Generating Glog dependency")
    ExternalProject_Add(
            glog_ext

            GIT_REPOSITORY https://github.com/google/glog
            GIT_TAG v0.6.0
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DWITH_GTEST:BOOL=OFF
                -DWITH_GFLAGS:BOOL=OFF
                ${GENERIC_PROJECTS_ARGS}
            )

    # Install Ceres
    message(INFO "${LOG_PREFIX} Generating Ceres dependency")
    ExternalProject_Add(
            Ceres_ext
            DEPENDS Eigen3_ext glog_ext

            GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver
            GIT_TAG 2.0.0
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DBUILD_EXAMPLES:BOOL=OFF
                -DBUILD_TESTING:BOOL=OFF
                -DEigen3_DIR:FILEPATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake
                -DEIGEN_INCLUDE_DIR:PATH=${CMAKE_INSTALL_PREFIX}/include/eigen3/
                -Dglog_DIR:FILEPATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/glog
                ${GENERIC_PROJECTS_ARGS}
            )

endif ()

# -----------------------------------------------------------------------------------------------
# INSTALL g2o
if (WITH_G2O)

    message(INFO "${LOG_PREFIX} Generating g2o dependency")
    ExternalProject_Add(
            G2o_ext
            DEPENDS Eigen3_ext Ceres_ext

            GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DG2O_USE_OPENGL:BOOL=OFF
                -DG2O_USE_VENDORED_CERES:BOOL=OFF
                -DG2O_BUILD_APPS:BOOL=OFF
                -DCeres_DIR:FILEPATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres
                -DEigen3_DIR:FILEPATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake
                -DEIGEN_INCLUDE_DIR:PATH=${CMAKE_INSTALL_PREFIX}/include/eigen3/
                -DG2O_BUILD_EXAMPLES:BOOL=OFF
                ${GENERIC_PROJECTS_ARGS}
            )

endif ()