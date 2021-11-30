# Setup Continuous Integration

- [Register](https://docs.gitlab.com/runner/register/) a new GitLab runner.
- Ensure all the SLAM dependencies (the [core SLAM lib's](../README.md#dependencies), the [ROS wrapping's](../README.md#dependencies-1) and/or the [Paraview wrapping's](../README.md#dependencies-2)) are installed on it.

  _**NOTE**: Be careful with system dependencies. If using them, one must be sure that they do not conflict with other dependencies. For example, installing a binary release of ROS comes with Eigen, PCL and VTK. However, these Eigen or VTK may conflict with Paraview's._

- If all these dependencies are not installed system-wide or not visible by CMake, it is necessary to indicate where to find them on the runner. To do so, the following variables can be optionally defined in the `environment` field of the `[[runners]]` section, defined in the `config.toml` file. These variables are CMake options that will be forwarded for the configuration step. For example, on a Linux runner, we can set:
```
environment = ["slam_cmake_option_Eigen_DIR=-DEIGEN3_INCLUDE_DIR=/home/ci/deps/install/include/eigen3",
               "slam_cmake_option_Ceres_DIR=-DCeres_DIR=/home/ci/deps/install/lib/cmake/Ceres",
               "slam_cmake_option_nanoflann_DIR=-Dnanoflann_DIR=/home/ci/deps/install/lib/cmake/nanoflann",
               "slam_cmake_option_PCL_DIR=-DPCL_DIR=/home/ci/deps/install/share/pcl-1.10",
               "slam_cmake_option_g2o_DIR=-Dg2o_DIR=/home/ci/deps/install/lib/cmake/g2o",
               "slam_cmake_option_ParaView_DIR=-DParaView_DIR=/home/ci/deps/build/paraview",
               "slam_cmake_option_Boost_DIR=-DBOOST_ROOT=/home/ci/deps/install",
               "slam_cmake_option_Qt5_DIR=-DQt5_DIR=/home/ci/deps/install/lib/cmake/Qt5"]
```

## About Windows

As Paraview must currently be built with MSVC 2015, this current CI script imposes to use MSVC 2015 for all jobs, for sake of coherence. However, the Core Lidar SLAM lib should build with recent MSVC versions.

## Using a Docker executor

To avoid encountering some issues linked to system libraries or other local dependencies conflict, using a Docker container may be a good solution.

As an example, the [Dockerfile](Dockerfile) file defines an image than can be used for a Linux runner building the ROS wrapping.

To register a new docker runner:

- Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository)
- Get the image either from the [Package/Container Registery](../../container_registry), build it from the [Dockerfile](Dockerfile) or from an available image on the [Docker Hub](https://hub.docker.com/search?q=&type=image) .
- Change the runner [docker pull policy](https://docs.gitlab.com/runner/executors/docker.html#using-the-if-not-present-pull-policy) to `if-not-present`. This will enable to use the local image you just get. To do so, open your [runner configuration file](https://docs.gitlab.com/runner/configuration/advanced-configuration.html).

## Add a test

Architecture choices :
- The test data are included in the image using the COPY command
- Build data are stored in cache after build job for next test job
- When the pipeline is triggered for master branch, log data are stored in cache
- When the pipeline is triggered on another branch, the log data are computed and compared with the master log data

Therefore, these are the detailed steps to add a test :
1. Add a *testk* (for test #**k**) folder to the gitlab-runner/Testing/SLAM folder containing the new bag file on which to test the SLAM
2. Add a COPY line to the Dockerfile for these new test data
3. Build the new docker image on CI PC
  ```
  docker build -t slam-ros-noetic-perception path/to/new/DockerFile
  ```
4. Add a make_reference job with the *testk* variable and its specific parameters (vlp16, outdoor...)
5. Add a test job with the same config as the one used in reference creation
