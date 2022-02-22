![SLAM in LidarView](doc/paraview_plugin.png)

![SLAM in ROS](doc/ros_node.png)

# LiDAR SLAM

- [Introduction and contents](#introduction-and-contents)
- [Core SLAM lib](#core-slam-lib)
  - [Dependencies](#dependencies)
  - [Installation](#installation)
- [ROS wrapping](#ros-wrapping)
  - [Dependencies](#dependencies-1)
  - [Installation](#installation-1)
  - [Usage](#usage)
- [ParaView wrapping](#paraview-wrapping)
  - [Dependencies](#dependencies-2)
  - [Installation](#installation-2)
  - [Usage](#usage-1)
  - [Use SLAM in LidarView](#use-slam-in-lidarview)

## Introduction and contents

This repo contains LiDAR-only visual SLAM developped by Kitware, as well as ROS and ParaView wrappings for easier use.

It has been successfully tested on data from several common LiDAR sensors:
- Velodyne (VLP-16, VLP-32c, HDL-32, HDL-64, VLS-128)
- Ouster (OS0/1/2-32/64/128)
- RoboSense (RS-LiDAR-16)
- Hesai (Pandar128)

Have a look at our [SLAM demo video](https://vimeo.com/524848891)!

This codebase is under active development. If you're interested by new features, new sensors' support or any project that could be using this SLAM, do not hesitate to contact us at kitware@kitware.com.

Repo contents :
- `slam_lib/` : core *LidarSlam* library containing SLAM algorithm and other utilities.
- `ros_wrapping/` : ROS packages to enable SLAM use on a ROS system.
- `paraview_wrapping/` : ParaView plugin to enable SLAM use with ParaView/LidarView.
- `ci/` : continuous integration files to automatically build and check *LidarSlam* lib.
- `CMakeLists.txt` : *CMakeLists* used to call to build core *LidarSlam* lib and *paraview_wrapping*.

## Core SLAM lib

### Dependencies

Dependencies are listed in the table below along with the version used during development and testing. Minimum required versions have not been determined yet.

| Dependency | Minimum tested Version |
| :--------: | :--------------------: |
| Eigen3     | 3.3.4                  |
| Ceres      | 1.13.0                 |
| PCL        | 1.8                    |
| nanoflann  | 1.3.0                  |
| g2o*       | 1.0.0 (master)         |
| OpenMP*    | 2.0                    |

(*) optional dependencies :

- If G2O is not available (or disabled), *LidarSlam* lib will still be compiled, but without pose graph optimization features.
- If OpenMP is available, it is possible to use multi-threading to run some SLAM steps in parallel and achieve higher processing speed.

**/!\ Warning** Make sure to compile/install G2O with the same Ceres version as the one used in the SLAM compilation. To do so, disable the feature [G2O_USE_VENDORED_CERES](https://github.com/RainerKuemmerle/g2o/blob/master/CMakeLists.txt) during G2O compilation and link against the right version of Ceres.

### Installation

To build only *LidarSlam* lib, just `cd` to this repo root dir and run :

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make
make install
```

The *LidarSlam* lib has been tested on Linux, Windows and OS X.

**NOTE:** You can link the local libraries you are using adding cmake flags. Notably with G2O:
  cmake -DCeres_DIR=/usr/local/lib/cmake/Ceres -Dg2o_DIR=/usr/local/lib/cmake/g2o path/to/slam_sources

## ROS wrapping

### Dependencies

Ensure all *LidarSlam* dependencies are respected. Specific ROS packages dependencies are listed in the table below along with the version used during development and testing.

| Dependency     | Tested Version | Install (`sudo apt-get install <pkg>`)                                           |
|:--------------:|:--------------:|:--------------------------------------------------------------------------------:|
| ROS            | melodic        | `ros-melodic-desktop-full` and [tutorial](http://wiki.ros.org/ROS/Installation)  |
| velodyne_pcl   | 1.6.1          | `ros-noetic-velodyne-pcl` or [git repo](https://github.com/ros-drivers/velodyne) |
| gps_common     | 0.3.0          | `ros-$ROS_DISTRO-gps-common`                                                     |
| geodesy        | 0.5.3          | `ros-$ROS_DISTRO-geodesy`                                                        |

Please note that the ROS Velodyne driver with minimum version 1.6 is needed.
Be careful, this ROS Velodyne driver 1.6 is not backward-compatible with previous versions.
If you're running on Ubuntu 20 / ROS Noetic, you can install the new Velodyne driver using the command `sudo apt install ros-noetic-velodyne ros-noetic-velodyne-pcl`.
If running on previous versions of Ubuntu/ROS (18/Melodic and below), you need to compile this driver from source : just clone the [git repo](https://github.com/ros-drivers/velodyne) in your catkin worskpace sources, it will be automatically built with next  `catkin_make`.

### Installation

Clone this git repo directly into your catkin directory, and run `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo` or `catkin_make -DCMAKE_BUILD_TYPE=Release` (to turn on optimizations, highly recommended when using Eigen). It will automatically build *LidarSlam* lib before ROS packages.

The ROS wrapping has been tested on Linux only.

### Usage

```bash
roslaunch lidar_slam slam.launch
roslaunch lidar_slam slam.launch gps:=true   # if GPS/SLAM calibration has to be run
```

See [ros_wrapping/lidar_slam/README.md](ros_wrapping/lidar_slam/README.md) for more details.

## ParaView wrapping

### Dependencies

Ensure all *LidarSlam* dependencies are respected. Specific dependencies are listed in the table below along with the version used during development and testing.

| Dependency | Tested Version |
| :--------: | :------------: |
| ParaView   | 5.4 and 5.6    |

Be careful to use and link to the same libraries as ParaView/LidarView's (especially with VTK, Eigen, PCL, Ceres, nanoflann, etc.). Otherwise, if different flags or modules were enabled, some troubles may arise at build time, or it could lead to version mismatch and segfault at runtime.

For example, if PCL is built with `pcl_visualization` module, it must link to the same VTK than the one used by ParaView.

### Installation

To build *LidarSlam* lib and this ParaView plugin *LidarSlamPlugin*, just `cd` to this repo root dir and run :

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSLAM_PARAVIEW_PLUGIN:BOOL=ON
make
make install
```

The *LidarSlamPlugin* Paraview wrapping has been tested on Linux, Windows and OS X.

### Usage

- Open ParaView
- **Tools** tab > **Manage Plugins** > **Load New**
- Browse to the `<install>/lib/` dir and select `libLidarSlamPlugin.so` or `LidarSlamPlugin.dll`
- Load LiDAR frames and LiDAR calibration to use
- Select the frames in the Pipeline Browser, instantiate a SLAM filter, and apply it.

Currently, all features are not available in ParaView plugin. Features such as GPS/LiDAR calibration, pose graph optimization or temporal logging are only supported in ROS wrapping. However, ParaView plugin is useful to play with SLAM, interactively try out parameters, visualize and export results.

### Use SLAM in LidarView

This *LidarSlamPlugin* is natively included in [LidarView](https://www.paraview.org/lidarview/). For more detailed information on how to enable and use SLAM filter in LidarView, see [paraview_wrapping/Plugin/doc/How_to_SLAM_with_LidarView.md](paraview_wrapping/Plugin/doc/How_to_SLAM_with_LidarView.md).

Pre-built binaries of LidarView with this SLAM plugin are available for download [here](https://drive.google.com/drive/folders/1ouNd3KD2p62a0XqRu4eJ2Tus6LJ-LBE8?usp=sharing).
