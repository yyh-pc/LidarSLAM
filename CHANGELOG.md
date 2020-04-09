# SLAM changes history

## *v1.1 (2020/04/09)*

Add several functionalities to **v1.0**, such as compressed pointclouds logging, latency compensation, multi-threading or ParaView/LidarView plugin.

This release includes lots of bug fixes, code cleaning/refactoring or small and various improvements, but only major changes are reported below. 

### Core lib

New features:

* Add transform extrapolation to compensate latency due to computation duration
* Add OpenMP multi-threading support for faster processing
* Enable keypoints maps saving/loading to/from PCD files
* Enable memory consumption display
* Enable logging keypoints and maps pointclouds as octree or PCD files to reduce memory usage

Major bug fixes or improvements:

* Major clean up, acceleration and fixes of `SpinningSensorKeypointExtractor`
* Fix maps update after PGO
* Fix Eigen alignment issues

General or CMake related changes:

* Rename *`slamlib`* to *`LidarSlam`*
* Add installation step for headers and libs
* Defaults to C++14 standard and RelWithDebInfo build type
* Ceres becomes a public dependency, G2O (and thus PGO) becomes optional
* Use modern CMake to link against Eigen and OpenMP targets if possible
* Move CI to docker

### ROS wrapping

* New SLAM functionalities support
* Add `SpinningSensorKeypointExtractor` parameters initialization from ROS parameter server
* Enable full 6D GPS pose conversions and pitch/heading computation from motion

### ParaView wrapping

* Working version of *`LidarSlamPlugin`*, ParaView plugin to enable using basic SLAM as a `vtkPolyData` algorithm
* This SLAM and ParaView plugin are now included in LidarView superbuild to be used directly in LidarView-based applications.

## *v1.0 (2019/12/18)*

First release of LiDAR SLAM as an independent project.
As this is the first 'official' version, most changes are not reported since **v0.0**, and only a small subset of useful or major changes is listed below.

### Core lib

* Numerous misc bug fixes and improvements
* Major code cleaning and refactoring
* Add CI for core SLAM lib
* Add pose graph optimization (PGO)
* Add optional logging of keypoints and trajectory
* Add verbosity modes to display state, steps durations and results
* Replace 6 DoF state vector by `Eigen::Isometry3d`
* Major acceleration of `RollingGrid`
* Add documentation for dependencies and installation


### ROS wrapping

* First version of ROS package `lidar_slam`, supporting all core SLAM lib functionalities
* Add SLAM parameters setting from ROS parameter server
* Optional SLAM/GPS global calibration from trajectories
* Add ROS package `gps_conversions` to manage conversions to standard `gps_common::GPSFix` message and process UTM/WGS84 transformations
* Compute GPS heading from movement when it is not available.
* Add documentation for usage


## *v0.0 (2019/11/11)*

Raw SLAM extracted from LidarView.