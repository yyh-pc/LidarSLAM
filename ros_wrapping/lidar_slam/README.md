# lidar_slam

- [lidar_slam](#lidarslam)
  - [LiDAR SLAM node](#lidar-slam-node)
    - [Description and basic usage](#description-and-basic-usage)
    - [More advanced usage](#more-advanced-usage)
  - [Optional GPS use](#optional-gps-use)
    - [SLAM output as GPS antenna pose](#slam-output-as-gps-antenna-pose)
    - [GPS/SLAM calibration](#gpsslam-calibration)
    - [SLAM pose graph optimization (PGO) with GPS prior](#slam-pose-graph-optimization-pgo-with-gps-prior)
    - [Running SLAM on same zone at different times (e.g. refining/aggregating map or running localization only on fixed map)](#running-slam-on-same-zone-at-different-times-eg-refiningaggregating-map-or-running-localization-only-on-fixed-map)
      - [Enabling/disabling SLAM map udpate](#enablingdisabling-slam-map-udpate)
      - [Setting SLAM pose from GPS pose guess](#setting-slam-pose-from-gps-pose-guess)
      - [Running SLAM on same zone](#running-slam-on-same-zone)
  - [About the published TF tree](#about-the-published-tf-tree)

Wrapping for Kitware LiDAR-only SLAM. It can also use GPS data to publish SLAM output pose as GPS coordinates, or to correct SLAM trajectory and map.

## LiDAR SLAM node

### Description and basic usage

The raw SLAM node subscribes to velodyne pointclouds, and publishes the lidar odometry (as well as TF) of the current pose with respect to the initial pose (= 1st LiDAR frame pose).

SLAM node supports massive configuration from ROS parameter server (even if default values are already provided). An example of configuration file can be found in [`params/slam_config.yaml`](params/slam_config.yaml). All parameters have to be set as private parameters.

To start only raw LiDAR SLAM, just start *lidar_slam_node*:
```bash
rosrun lidar_slam lidar_slam_node
```
If you want to specify parameters, you should consider using a launchfile.

### More advanced usage

SLAM is often used with a multi-sensor system, for navigation or mapping purposes. Therefore, for easier fusion procedure, it can be usefull to output SLAM pose as a world GPS coordinate. In that case, consider using the given launchfile :
- To start SLAM when replaying rosbag file, run :
```bash
rosbag play --clock <my_bag_file>
roslaunch lidar_slam slam.launch
```
- When using it in real conditions, use :
```bash
roslaunch lidar_slam slam.launch use_sim_time:=false
```

This launch file will start a *lidar_slam_node*, a pre-configured RViz session, and GPS/UTM conversions nodes to publish SLAM pose as a GPS coordinate in WGS84 format, with the prior that full GPS pose and GPS/LiDAR calibration are correctly known and set (see [GPS/SLAM calibration](#gpsslam-calibration) section below).

Input pointcloud fix must be a *sensor_msgs/PointCloud2* message (of points `velodyne_pointcloud::PointXYZIR`) published on topic '*velodyne_points*'. Input GPS fix must be a *gps_common/GPSFix* message published on topic '*gps_fix*'.
It outputs the SLAM pose as a *nav_msgs/Odometry* message on topic '*slam_odom*' and as a *sensor_msgs/NavSatFix* message on topic '*slam_fix*', and publishes the corresponding transforms on TF server.

## Optional GPS use

### SLAM output as GPS antenna pose

LiDAR SLAM is often used when GPS is unreliable, by providing a continuous trajectory for a short time period. However, the output SLAM pose matches with LiDAR sensor pose, which may be quite different from GPS antenna pose. To easily substitute GPS with SLAM in these no satellite-signal zones, it is possible to tell the SLAM node to output the odometry (and TF) corresponding to GPS sensor pose instead of LiDAR's.

To enable this GPS antenna pose publication instead of LiDAR's, set `gps/output_gps_pose` parameter to `true`, give the name (`output_gps_pose_frame_id`) of the new frame attached to GPS antenna, and provide the transform between GPS and LiDAR sensors.

### GPS/SLAM calibration

To be able to publish local SLAM odometry as GPS coordinates, it is necessary to link SLAM initial pose to a 3D GPS pose (position + orientation) transformed into cartesian space with UTM projection. However, full 3D GPS pose is rarely known (we generally only have 3D position and sometimes an approximate yaw heading angle), and the calibration from GPS antenna to LiDAR sensor is always approximate (3D translation offset is more/less correct, but 3D rotation is unknown).

If these full 3D transforms are perfectly known, fill them in [`launch/slam.launch`](launch/slam.launch) to ensure that the correct transformation is applied to output SLAM pose to the WGS84 format.

If these transforms are unknown, the SLAM node can try to auto-compute them for you. If this GPS/SLAM calibration is enabled, the node subscribes to the topic '*gps_odom*' records the last GPS and SLAM positions (and forgets the ones older than a pre-defined timeout threshold). The calibration process can be triggered at any time by publishing the `lidar_slam/SlamCommand/GPS_SLAM_CALIBRATION` command to '*slam_command*' topic. When triggered, the recorded SLAM and GPS trajectories are aligned with rigid ICP matching, giving the global 3D static transform to link a SLAM pose into GPS coordinates, published on TF server.
1. NOTE: During this auto-calibration process, GPS position should be precise enough to guarantee a robust calibration.
2. NOTE: As registration is done via ICP without any other prior, the trajectories need to have some turns in order to fully constrain the problem. If the movement is only following a straight line, 1 rotation remains unconstrained, and could lead to serious artefacts.
3. NOTE: To fix this straight line case, a supplementary prior can be introduced, imposing for example the output calibration to have no roll angle (hypothesis of flat ground plane in front direction). However, if ground is not flat, it could also lead to bad calibration.
4. NOTE: Timestamps are currently not used for calibration, as GPS and SLAM clocks are not synchronized. It could be a nice future improvement.

To enable this GPS/SLAM auto-calibration, use option `gps:=true` :
```bash
roslaunch lidar_slam slam.launch gps:=true  # Start SLAM node and enable GPS use.
...
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 0"  # Trigger GPS/SLAM calibration
```

### SLAM pose graph optimization (PGO) with GPS prior

If some GPS positions are available, they can be used to optimize the SLAM trajectory by correcting drift error accumulated over time. PGO can be triggered at any time by publishing the `lidar_slam/SlamCommand/GPS_SLAM_POSE_GRAPH_OPTIMIZATION` command to '*slam_command*' topic. When triggered, the GPS positions and their associated covariances can be used as priors to optimize the SLAM pose graph with g2o framework. SLAM maps will also be corrected. The GPS/LiDAR calibration will also be computed and published as a static TF (more precise than the global ICP calibration process).

NOTE: This PGO is not real-time, and should therefore be run when system is not or slowly moving.

To enable this GPS/SLAM pose graph optimization, enable logging of previous poses, covariances and keypoints (`slam/logging_timeout != 0`) and use option `gps:=true`:
```bash
roslaunch lidar_slam slam.launch gps:=true  # Start SLAM node and enable GPS use.
...
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 2"  # Trigger PGO
```

### Running SLAM on same zone at different times (e.g. refining/aggregating map or running localization only on fixed map)

#### Enabling/disabling SLAM map udpate

At any time, command `lidar_slam/SlamCommand/ENABLE_SLAM_MAP_UPDATE` or `lidar_slam/SlamCommand/DISABLE_SLAM_MAP_UPDATE` can be published to '*slam_command*' topic to enable or disable SLAM map update. During normal SLAM behavior, map update is enabled, which means SLAM performs keypoints registration in the world to aggregate previous frames to be able to run localization in this map. However, it is possibe to disable this map update and to run localization only in the fixed keypoints map. This can be usefull when the map has been pre-optimized with PGO and we don't want to pollute it with unreliable new frames.

#### Setting SLAM pose from GPS pose guess

If you want to run another bag on the same zone to refine the SLAM map or to run localization only with the previously built map, you need to give an approximate new init pose to SLAM if trajectory is not continuous with end pose. You can send `lidar_slam/SlamCommand/SET_SLAM_POSE_FROM_NEXT_GPS` command to '*slam_command*' topic to use the next received GPS pose as a pose guess for SLAM.

NOTE: To be able to use this command, SLAM and GPS coordinates must be precisely linked with a valid TF tree. Be sure you already called [SLAM pose graph optimization](#slam-pose-graph-optimization-pgo-with-gps-prior).

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

Here is the complete TF tree maintained by different nodes as well as descriptions of each frame (default frame names) :

```bash
utm
|__ gps_init
    |__ gps
    |__ slam_init
        |__ velodyne
            |__ (slam)
```

- **utm**: "world" ENU fixed frame, corresponding to the origin of the current considered UTM zone/band in which GPS coordinates are projected into.
- **gps_init**: first received GPS pose. It defines the origin of the local map, easier to use than UTM frame (because coordinates in UTM frames can be very large, leading to floating points discretization errors). If the GPS data provides an orientation, this frame is oriented alongside this direction. Otherwise, the frame orientation remains unchanged, and corresponds to UTM East-North-Up (ENU) coordinates, offset by the 1st GPS position. The static TF `utm -> gps_init` is published by `gps_conversions/gps_to_utm` node.
- **gps**: current GPS pose (pose (or position if orientation isn't provided) of the GPS antenna). The TF `gps_init -> gps` is published by `gps_conversions/gps_to_utm` node.
- **slam_init**: initial pose of the SLAM, which is the pose of the velodyne sensor of the first received pointcloud. The static TF `gps_init -> slam_init` is published either by `lidar_slam/lidar_slam_node` node (in case of GPS/SLAM auto-calibration) or manually set with tf2 static publishers in [`launch/slam.launch`](launch/slam.launch) (in case of pre-defined calibration).
- **velodyne**: current pose of the velodyne sensor, according to SLAM. The TF `slam_init -> velodyne` is published by `lidar_slam/lidar_slam_node` node.
- **slam**: (optional) current pose of the GPS antenna, according to SLAM. This TF is published only if `gps/output_gps_pose` is enabled. This pose can be directly compared to the *gps* frame, and should exactly match if SLAM was perfect. The static TF `velodyne -> slam` is published by `lidar_slam/lidar_slam_node` node according to `gps/gps_to_lidar_offset` parameter value.