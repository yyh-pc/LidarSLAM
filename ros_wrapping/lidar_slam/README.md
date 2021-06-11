# lidar_slam

- [lidar_slam](#lidarslam)
  - [LiDAR SLAM node](#lidar-slam-node)
    - [Description and basic usage](#description-and-basic-usage)
    - [More advanced usage](#more-advanced-usage)
  - [Optional GPS use](#optional-gps-use)
    - [Map (GPS) / Odom (SLAM) calibration](#map-gps--odom-slam-calibration)
    - [SLAM pose graph optimization (PGO) with GPS prior](#slam-pose-graph-optimization-pgo-with-gps-prior)
    - [Running SLAM on same zone at different times (e.g. refining/aggregating map or running localization only on fixed map)](#running-slam-on-same-zone-at-different-times-eg-refiningaggregating-map-or-running-localization-only-on-fixed-map)
      - [Enabling/disabling SLAM map udpate](#enablingdisabling-slam-map-udpate)
      - [Setting SLAM pose from GPS pose guess](#setting-slam-pose-from-gps-pose-guess)
      - [Running SLAM on same zone](#running-slam-on-same-zone)
  - [About the published TF tree](#about-the-published-tf-tree)

Wrapping for Kitware LiDAR-only SLAM. It can also use GPS data to publish SLAM output pose as GPS coordinates, or to correct SLAM trajectory and map.

## LiDAR SLAM node

### Description and basic usage

The SLAM node subscribes to one or several topics of LiDAR pointclouds, and computes current pose of the tracked frame relative to the fixed odometry frame. It can output various data, such as SLAM pose (as Odometry msg or TF), keypoint maps, etc.

SLAM node supports massive configuration from ROS parameter server (even if default values are already provided). An example of configuration file can be found in [`params/slam_config.yaml`](params/slam_config.yaml). All parameters have to be set as private parameters.

To start only raw LiDAR SLAM, just start *lidar_slam_node*:
```bash
rosrun lidar_slam lidar_slam_node
```
If you want to specify parameters, you should consider using a launchfile.

### More advanced usage

SLAM is often used with a multi-sensor system, for navigation or mapping purposes. Therefore, for easier fusion procedure, it can be useful to output SLAM pose in an other frame than LiDAR sensor's or as a world GPS coordinate. In that case, consider using the given launchfile :
- To start SLAM when replaying rosbag file, run :
```bash
roslaunch lidar_slam slam.launch   # in 1st shell
rosbag play --clock <my_bag_file>  # in 2nd shell
```
- When using it in real conditions, use :
```bash
roslaunch lidar_slam slam.launch use_sim_time:=false
```

This launch file will start a *lidar_slam_node*, a pre-configured RViz session, a *lidar_conversion_node* (converts the point type to expected SLAM use, see next paragraph) and, if `gps` arg is enabled, GPS/UTM conversions nodes to publish SLAM pose as a GPS coordinate in WGS84 format, with the prior that full GPS pose and GPS/LiDAR calibration are correctly known and set (see [GPS/SLAM calibration](#gpsslam-calibration) section below).

The SLAM node subscribes to one or several input pointclouds topics ((default single topic is *lidar_points*) as *sensor_msgs/PointCloud2* messages. These pointclouds should have the following fields:
- **x**, **y**, **z** (`float`) : point coordinates
- **time** (`double`) : time offset to add to the pointcloud header timestamp to get approximate point-wise acquisition timestamp
- **intensity** (`float`) : intensity/reflectivity of the point
- **laser_id** (`uint16`) : numeric identifier of the laser ring that shot this point. The lowest/bottom laser ring should be 0, and it should increase upward.
- **device_id** (`uint8`) : numeric identifier of the LiDAR device/sensor. This id should be the same for all points of the cloud acquired by the same sensor.
- **label** (`uint8`) : optional input, not yet used.

If your LiDAR driver does not output such data, you can use the `lidar_conversions` nodes.

Optional input GPS (see [Optional GPS use](#optional-gps-use) section) fix must be a *gps_common/GPSFix* message published on topic '*gps_fix*'.

SLAM outputs can also be configured out to publish :
- current pose as an *nav_msgs/Odometry* message on topic '*slam_odom*' and/or a TF from '*odometry_frame*' to '*tracking_frame*';
- extracted keypoints from current frame as *sensor_msgs/PointCloud2* on topics '*keypoints/{edges,planes,blobs}*';
- keypoints maps as *sensor_msgs/PointCloud2* on topics '*maps/{edges,planes,blobs}*';
- registered and undistorted point cloud from current frame, in odometry frame, as *sensor_msgs/PointCloud2* on topic '*slam_registered_points*';
- confidence estimations on pose output, as *lidar_slam/Confidence* custom message on topic '*slam_confidence*'. It contains the pose covariance, an overlap estimation, the number of matched keypoints and a binary estimator to check motion limitations.

UTM/GPS conversion node can output SLAM pose as a *gps_common/GPSFix* message on topic '*slam_fix*'.

**NOTE** : It is possible to track any *tracking_frame* in *odometry_frame*, using a pointcloud expressed in an *lidar_frame*. However, please ensure that a valid TF tree is beeing published to link *lidar_frame* to *tracking_frame*.

## Optional GPS use

If GPS use is enabled, *LidarSlamNode* subscribes to the GPS odometry on topic '*gps_odom*', and records the most recent GPS positions. To use GPS data, we transform GPS WGS84 fix into cartesian space using UTM projection. This can be used to estimate calibration between GPS and SLAM trajectories, or post-optimize SLAM trajectory with pose graph optimization (PGO).

To use GPS data :
- enable gps use : `gps/use_gps = true`:
- enable logging of previous poses, covariances and keypoints : `slam/logging_timeout != 0`

**NOTE** : If GPS odometry expresses the pose of a *gps_frame* different from *tracking_frame*, please ensure a valid static TF is beeing broadcasted.

### Map (GPS) / Odom (SLAM) calibration

To be able to publish local SLAM odometry as GPS coordinates, it is necessary to link local SLAM odometry frame (often called `odom`) to world fixed frame (often called `map`).

If GPS use is enabled, *LidarSlamNode* can try to estimate the transform that links these frames by aligning SLAM and GPS trajectories with rigid ICP matching. The resulting transform is published as a static transform on TF server.

The calibration process can be triggered at any time by publishing the `lidar_slam/SlamCommand/GPS_SLAM_CALIBRATION` command to '*slam_command*' topic.

1. NOTE: During this auto-calibration process, GPS position and SLAM should be precise enough to guarantee a robust calibration.
2. NOTE: As registration is done via ICP without any other prior, the trajectories need to have some turns in order to fully constrain the problem. If the movement is only following a straight line, 1 rotation remains unconstrained, and could lead to serious artefacts.
3. NOTE: To fix this straight line case, a supplementary prior can be introduced, imposing for example the output calibration to have no roll angle (hypothesis of flat ground plane in front direction). However, if ground is not flat, it could also lead to bad calibration.
4. NOTE: Timestamps are currently not used for calibration, as GPS and SLAM clocks are not synchronized. It could be a nice future improvement.

To enable this GPS/SLAM auto-calibration, you can use option `gps:=true` :
```bash
roslaunch lidar_slam slam.launch gps:=true  # Start SLAM node and enable GPS use.
...
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 0"  # Trigger GPS/SLAM calibration
```

### SLAM pose graph optimization (PGO) with GPS prior

Available GPS positions can also be used to optimize the SLAM trajectory by correcting drift error accumulated over time. The GPS positions and their associated covariances can be used as priors to optimize the SLAM pose graph with g2o framework. SLAM maps will also be corrected. The [map/odom calibration](#map-gps--odom-slam-calibration) will also be computed and published as a static TF (but should be more precise than the global ICP calibration process).

PGO can be triggered at any time by publishing the `lidar_slam/SlamCommand/GPS_SLAM_POSE_GRAPH_OPTIMIZATION` command to '*slam_command*' topic.

NOTE: This PGO is not real-time, and should therefore be run when system is not or slowly moving.

To enable PGO, you can use option `gps:=true` :
```bash
roslaunch lidar_slam slam.launch gps:=true  # Start SLAM node and enable GPS use.
...
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 2"  # Trigger PGO
```

### Running SLAM on same zone at different times (e.g. refining/aggregating map or running localization only on fixed map)

#### Enabling/disabling SLAM map udpate

At any time, command `lidar_slam/SlamCommand/ENABLE_SLAM_MAP_UPDATE` or `lidar_slam/SlamCommand/DISABLE_SLAM_MAP_UPDATE` can be published to '*slam_command*' topic to enable or disable SLAM map update. During normal SLAM behavior, map update is enabled, which means SLAM performs keypoints registration in the odometry frame to aggregate previous frames to be able to run localization in this local map. However, it is possibe to disable this map update and to run localization only in the fixed keypoints map. This can be useful when the map has been pre-optimized with PGO and we don't want to pollute it with unreliable new frames.

#### Setting SLAM pose from GPS pose guess

If you want to run another bag on the same zone to refine the SLAM map or to run localization only with the previously built map, you need to give an approximate new init pose to SLAM if trajectory is not continuous with end pose. You can send `lidar_slam/SlamCommand/SET_SLAM_POSE_FROM_GPS` command to '*slam_command*' topic to use the last received GPS pose as a pose guess for SLAM.

NOTE: To be able to use this command, SLAM and GPS coordinates must be precisely linked with a valid TF tree. Be sure you already called [pose graph optimization](#slam-pose-graph-optimization-pgo-with-gps-prior) or at least [map/odom calibration](#map-gps--odom-slam-calibration).

#### Running SLAM on same zone

To sum up, if you want to run SLAM on same zone, use :
```bash
roslaunch lidar_slam slam.launch gps:=true  # Start SLAM node and enable GPS use.
...  # Run 1st real test or bag file
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 2"    # Trigger PGO : optimize SLAM map and compute GPS/SLAM calibration
(rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 8")  # Disable SLAM map update (optional)
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 4"    # If the starting pose of the new bag does not match with last SLAM pose, use GPS pose as initial guess
...  # Run 2nd real test or bag file
```

## About the published TF tree

Here is an example of the complete TF tree that can be maintained by different nodes as well as descriptions of each frame (default frame names) :

```bash
utm
└─ enu
   └─ map
      └─ odom
         └─ base_link
            ├─ lidar
            └─ gps
```

- **utm**: "world" ENU fixed frame, corresponding to the origin of the current considered UTM zone/band in which GPS coordinates are projected into.
- **enu**: local ENU fixed frame attached to 1st received GPS position in UTM coordinates, easier to use than **utm** because UTM coordinates can grow very large, leading to floating points discretization errors. The static TF `utm -> enu` is published by `gps_conversions/gps_to_utm` node.
- **map**: first received full 6D GPS pose. It defines the origin of the local map. If GPS does not provide orientation, pitch and heading can be estimated from motion. The static TF `enu -> map` is published by `gps_conversions/gps_to_utm` node.
- **odom**: origin of the SLAM. The TF `map -> odom` can be published by a custom node, by `lidar_slam/lidar_slam_node` node (in case of GPS/SLAM auto-calibration or PGO), or manually set with tf2 static publishers in [`launch/slam.launch`](launch/slam.launch) (in case of pre-defined calibration).
- **base_link**: current pose computed by SLAM algorithm (here `base_link` is the tracking frame). The TF `odom -> base_link` can be published by `lidar_slam/lidar_slam_node` node.
- **lidar**: pose of the LiDAR sensor on the moving base. The TF `base_link -> lidar` should be published by a `tf2_ros/static_transform_publisher` node.
- **gps**: pose of the GPS sensor on the moving base. The TF `base_link -> gps` should be published by a `tf2_ros/static_transform_publisher` node.