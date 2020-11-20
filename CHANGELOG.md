# SLAM changes history

## *v1.3 (2020/11/20)*

This release brings various minor fixes or features to improve stability, performance or code readibility.

Major changes are reported below.

### Core lib

* Fix compilation on Windows with MSVC 2015 (!39)
* Simplify `KDTreePCLAdaptor` usage and clean its code (!41)
* Misc bug fixes and code refactoring in ICP functions (!43)
* Fix ego motion extrapolation when using Pose Graph Optimization (!44)
* Ignore Ego-Motion extrapolation, approximated undistortion and Localization results if error occurs (!49 and !52)
* Replace max variance error by max position and orientation errors (!47)
* Fix rolling grid and maps update (!50, !55, !58)
* Add missing `std` prefix in `PointCloudStorage` that prevente compilation on some machines (!42)
* Add colors to verbose display (!53)
* Major refactoring of ICP and LM optimization parts into separate classes (!54)
* Reset timers values when resetting SLAM (!63)

### ROS wrapping

* Publish SLAM pose as `gps_common/GPSFix` instead of `sensor_msgs/NavSatFix` (!45)
* Publish SLAM-estimated speed in GPS-like message (!46)

### ParaView wrapping

* Add verbose timings to check VTK <-> PCL conversions and perform less maps updates (!48)
* Fix frame stamp to last point measurement time (!49)

## *v1.2 (2020/06/26)*

This new release brings important improvements (in terms of processing speed as well as precision) such as *undistortion* or *motion extrapolation*. It also greatly improves user interface for easier parameters settings.

Major changes are reported below.

### Core lib

New features:
* Run SLAM in intermediary BASE coordinates system : user can set a rigid transform between BASE and LIDAR coordinates systems, in order to compensate lidar sensor's position and orientation relatively to the moving base plateform. Outputs are expressed in BASE frame, defaulting to LIDAR frame.
* Estimate realtive motion since last frame using previous motion extrapolation, and set Ego-Motion step as optional. This greatly improves processing speed, as well as stability for smooth movements (e.g. vehicles).
* Enable undistortion to correct rolling shutter distortion. 2 modes are available. This greatly improves precision.

Major bug fixes or improvements:
* Major acceleration of Ceres cost functions
* Remove undistortion in Ego-Motion step : this step is only used as an initialization for Localization step, and thus just need to be fast and approximate.
* Simplify `Slam` class by deleting unused attributes or refactoring them.
* Fix compilation warnings

Other noticeable changes changes:
* Updated copyright and Apache 2.0 license
* Rename Mapping step to Localization
* Add `Utilities.h` file to centralize all helper functions

ROS and ParaView wrappings have been adapted to support these previously mentionned changes.

### ROS wrapping

* Use *tf2* for sensors calibrations.
* Refactor output publishers and set TF publication as optional : user can now choose which outputs he wants to be published.
* Enable output of extracted keypoints from current frame.

### ParaView wrapping

* SLAM filters can now be applied on the selected input `vtkPolyData` frame, and not anymore on the `vtkTable` calibration. This is a more intuitive behavior.
* Filters proxies display bugs are fixed (duplicated titles or lines, parameters orders, hide unavailable parameters, ...).
* Update filters/parameters names and documentation.
* Add *How to SLAM with LidarView* tutorial.
* Maps and keypoints become optional outputs to avoid unecessary conversions and save time.
* Orientation is also saved as `AxisAngle` representation.
* Rename *DisplayMode* to *AdvancedReturnMode* and disable it by default.

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