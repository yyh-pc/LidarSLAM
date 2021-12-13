# SLAM changes history

## *v1.5 (2021/12/13)*

This release mainly brings new features (keyframes structure, integration of IMU/wheel odometry constraints, moving objects rejection, confidence estimators integration, CI implementation) and contains some refactoring, notably in the map structure and in the keypoint types management.

The changes are summarized below.

### Core lib

**Major new features:**

* Simplify some parameters for model fitting (matches validation) (!98)
* Auto compute angular resolution for SSKE (!102)
* Add invalidation parameter for oblique surfaces (!115)
* Add 2D optimization mode (!118)
* Add only keyframes points to maps (!117)
* Add wheel odometry and IMU constraints to optimization (!114)
* Add an optional overlap estimator (!127 and !131)
* Add motion limitations to detect failures (!129)
* Add a mapping mode + a sampling mode (!147)
* Add an option to reject moving objects (!147)
* Add an option to add temporal keypoints (!147)

**Refactoring and code architecture changes:**

* Change keypoint invalidation criteria to use max oblique plane (!93)
* Gather all keypoint types relative structures and functions (!103)
* Split Matching and optimization (!101)
* Fix refactoring bugs (!108)
* Update README and LidarView tutorial (!113)
* Clean, correct and refactor keypoints matching (!136, !140)
* Replace the pcl voxel grid by a custom one (!147)

**Performance improvements:**

* Add NbPoints attribute to RollingGrid to accelerate Size method (!128)
* Store rolling grid as unordered_map instead of stacked vectors (!130)
* Accelerate outputs transformations (!132)
* Avoid rebuilding KD-trees (!135)
* Change susbsampling strategy in overlap computation (!138)

**Bug fixes:**

* Fix keypoints extraction (!100)
* Fix warning, laser id mapping and undistortion time range (!105)
* Correct debug array error (!109)
* Fix Keypoints extraction and matching bugs and rename variables (!112)
* Add criterion to add keyframe (!124)
* Add checks to estimated resolution angle (!125)
* Check timestamp before ego motion extrapolation (!142, !151)
* Add a temporal window to estimate velocity (!141 and !143)
* Fix build in debug mode (!148)

**Compilation / CMake related changes / CI:**

* Add missing include to allow compilation on Win10 MSVC 2015 (!107)
* Allow to build LidarSlam lib as STATIC (!119 and !120)
* Add CI build jobs (!123)
* Specify CMake project version (!144)
* Fix build in debug mode (!148)
* Update to fit PV5.9 requirements (!157)
* Add regression tests on CI (!159)

### ROS wrapping

* Fix and clean ROS/catkin dependencies, compilation and installation (!121)
* Set initial maps and pose (!122)
* Include confidence in ROS documentation (!134)
* Allow to change the mapping mode online (!147)
* Add computation time to confidence estimation output (!159)
* Add config file for indoor context (!159)
* Complete documentation (!161)

### ParaView wrapping

* Optional calibration in PV wrapping (!106)
* Remove label field in xml to solve python access to variables (!116)
* Rename ENABLE_slam to LIDARVIEW_BUILD_SLAM (!126)
* Reformat XML proxy (!139)
* Allow to choose the mapping mode + the sampling mode (!147)
* Allow to load an initial map (!149)
* Update to fit PV5.9 requirements (!152, !153, !155, !157, !158, !163, !164)
* Add specific support for Hesai sensor (!165)

## *v1.4 (2021/04/01)*

This release brings important changes in code architecture (change SLAM point definition, add namespaces, reorganize ROS wrapping),
and adds new LiDAR sensors support (Velodyne and Ouster on PV, Velodyne and RoboSense on ROS).
It also focuses on improved performance and eases user experience.

Major changes are summarized below.

### Core lib

**Major new features:**

* Change SLAM point definition to add more point-wise fields (!64 and !65)
* Use real point wise time and compute output pose at frame timestamp (!82)
* Add multi lidar support, currently only available from ROS wrapping (!83 and !84)

**Refactoring and code architecture changes:**

* Standardize convention for pose formatting to XYZRPY (!66)
* Add `LidarSlam` and `Utils` namespaces and reorganize code structure (!72 and !73)
* Laser ID mapping arg becomes optional (!77)
* Move laser ID mapping parameter to wrappings (!79)
* Add `Utilities.cxx` to avoid defining entire helpers functions in header (!74 and !89)
* SLAM steps refactoring (!81)
* Major undistortion refactoring, remove `OPTIMIZED` mode and split `APPROXIMATED` to `ONCE` and `REFINED` (!87)

**Performance improvements:**

* Accelerate PCA computation and use float precision in keypoints extraction and registration (!86)
* Accelerate x3 linear transform interpolation (!88)
* Improve residuals weighting: fix duplicated weighting, use TukeyLoss robustifier, add debug info (!90)
* Simplify auto diff ceres (!95)

**Bug fixes:**

* Correct nb of laser rings detection (!70)
* Fix Ceres residuals conflict with LV's and deprecate unused residuals (!71)
* Fix ICP saturation range (!94)

### ROS wrapping

The ROS wrapping is adapted to the core lib improvements, such as the SLAM point definition, the optional laser ID mapping setting and other modified parameters and options.
It also now supports the multi-lidar mode to SLAM with several LiDAR devices.

- Update ROS wrapping to use ROS Velodyne driver 1.6 (!67)
- Allow inheritance from LidarSlamNode (!69)
- Add independent `lidar_conversions` package to perform pointclouds conversion to the expected SLAM point type (!68 and !75)
- Add Robosense RSLidar conversion node, and improve approximate point-wise timestamp computation (!76)
- Set point-wise `device_id` field in lidar_conversions nodes (!80)
- Update ROS params, doc and rename output frame topic (!85)

### ParaView wrapping

The PV wrapping is adapted to the core lib improvements, such as the SLAM point definition, the optional laser ID mapping setting and other modified parameters and options.

- Adapt PV wrapping to Ouster, and add manual setting for other LiDAR data (!56)

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