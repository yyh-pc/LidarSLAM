cmake_minimum_required(VERSION 3.14)
project(Superbuild)

# -----------------------------------------------------------------------------------------------
# Select Options
option(WITH_EIGEN "Whether to install Eigen3" ON)
option(WITH_NANOFLANN "Whether to install nanoflann" ON)
option(WITH_PCL "Whether to install PCL" ON)
option(WITH_CERES "Whether to install Ceres" ON)
option(WITH_G2O "Whether to install Ceres" OFF)

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

    # INSTALL FLANN
    message(INFO "${LOG_PREFIX} Generating Flann dependency")
    # Flann official version has a cmake issue (https://stackoverflow.com/questions/50763621/building-flann-with-cmake-fails)
    # We use a patched version
    ExternalProject_Add(
            FLANN_ext

            GIT_REPOSITORY https://github.com/tkircher/flann
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DCMAKE_CXX_STANDARD:STRING=14
                -DBUILD_EXAMPLES:BOOL=OFF
                -DBUILD_DOC:BOOL=OFF
                -DBUILD_PYTHON_BINDINGS:BOOL=OFF
                -DBUILD_MATLAB_BINDINGS:BOOL=OFF
                -DBUILD_TESTS:BOOL=OFF
                ${GENERIC_PROJECTS_ARGS}
            )

    # INSTALL PCL
    message(INFO "${LOG_PREFIX} Generating PCL dependency")
    ExternalProject_Add(
            PCL_ext
            DEPENDS FLANN_ext Boost_ext Eigen3_ext

            GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl
            GIT_TAG pcl-1.10.0
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DBUILD_examples:BOOL=OFF
                -DBUILD_global_tests:BOOL=OFF
                -DBUILD_visualization:BOOL=OFF
                -DBUILD_stereo:BOOL=OFF
                -DBUILD_surface_on_nurbs:BOOL=OFF
                -DBUILD_tracking:BOOL=OFF
                -DBUILD_ml:BOOL=OFF
                -DBUILD_keypoints:BOOL=OFF
                -DBUILD_benchmarks:BOOL=OFF
                -DBUILD_2d:BOOL=ON
                -DBUILD_common:BOOL=ON
                -DBUILD_octree:BOOL=ON
                -DBUILD_kdtree:BOOL=ON
                -DBUILD_search:BOOL=ON
                -DBUILD_sample_consensus:BOOL=ON
                -DBUILD_features:BOOL=ON
                -DBUILD_filters:BOOL=ON
                -DBUILD_registration:BOOL=ON
                -DWITH_QT:BOOL=OFF
                -DWITH_OPENNI=OFF
                -DWITH_OPENNI2=OFF
                -DPCL_ONLY_CORE_POINT_TYPES:BOOL=ON
                -Dflann_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/flann
                -DEIGEN_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake
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
            GIT_TAG 2.1.0
            GIT_SHALLOW 1

            CMAKE_ARGS
                -DBUILD_EXAMPLES:BOOL=OFF
                -DBUILD_TESTING:BOOL=OFF
                -DEigen3_DIR:FILEPATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake
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
                -DG2O_BUILD_EXAMPLES:BOOL=OFF
                ${GENERIC_PROJECTS_ARGS}
            )

endif ()