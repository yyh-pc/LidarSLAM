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
  - [Download LidarView pre-built binaries with SLAM](#download-lidarview-pre-built-binaries-with-slam)
  - [Usage](#usage-1)

## Introduction and contents

This repo contains LiDAR-only visual SLAM developped by Kitware, as well as ROS and ParaView wrappings for easier use.

Repo contents :
- `slam_lib/` : core library containing SLAM algorithm and other utilities.
- `ros_wrapping/` : ROS packages to enable SLAM use on a ROS system.
- `paraview_wrapping/` : ParaView plugin to enable SLAM use with ParaView/LidarView.
- `ci/` : continuous integration files to automatically build and check *LidarSlam* lib.
- `CMakeLists.txt` : *CMakeLists* used to call to build core *LidarSlam* lib and *paraview_wrapping*.

## Core SLAM lib

### Dependencies

Dependencies are listed in the table below along with the version used during development and testing. Minimum required versions have not been determined yet.

| Dependency | Minimum tested Version |
| :--------: | :--------------------: |
| nanoflann  | 1.3.0                  |
| Eigen3     | 3.3.4                  |
| Ceres      | 1.13.0                 |
| PCL        | 1.8                    |
| g2o*       | 1.0.0 (master)         |
| OpenMP*    | 4.5                    |

(*) optional dependencies :

- If G2O is not available (or disabled), *LidarSlam* lib will still be compiled, but without pose graph optimization features.
- If OpenMP is available, it is possible to use multi-threading to run some SLAM steps in parallel and achieve higher processing speed.

### Installation

To build only *LidarSlam* lib, just `cd` to this repo root dir and run :

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make
make install
```

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

Be careful to use and link to the same libraries as ParaView/LidarView (especially with VTK, Eigen, PCL, Ceres, nanoflann, etc.) or it could lead to version mismatch and segfault.

### Installation

To build *LidarSlam* lib and this ParaView plugin *LidarSlamPlugin*, just `cd` to this repo root dir and run :

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSLAM_PARAVIEW_PLUGIN:BOOL=ON
make
make install
```

### Download LidarView pre-built binaries with SLAM

Pre-build binaries of LidarView with this SLAM plugin are also available for download [here](https://drive.google.com/drive/folders/1etSkoaR_863MYyRNXgv6PFTiUcggBAcx?usp=sharing).

### Usage

- Open ParaView
- **Tools** tab > **Manage Plugins** > **Load New**
- Browse to the `<install>/lib/` dir and select `libLidarSlamPlugin.so`
- Load LiDAR frames and LiDAR calibration to use
- Select the frames in the Pipeline Browser, instantiate a SLAM filter, and apply it.

Currently, all features are not available in ParaView plugin. Features such as GPS/LiDAR calibration, pose graph optimization or temporal logging are only supported in ROS wrapping. However, ParaView plugin is useful to play with SLAM, load it in LidarView, and interactively try out parameters.

This *LidarSlamPlugin* is natively included in [LidarView](https://www.paraview.org/lidarview/). For more detailed information on how to enable and use SLAM filter in LidarView, see [paraview_wrapping/doc/How_to_SLAM_with_LidarView.md](paraview_wrapping/doc/How_to_SLAM_with_LidarView.md).