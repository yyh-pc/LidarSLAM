# lidar_slam

- [lidar_slam](#lidar_slam)
  - [LiDAR SLAM node](#lidar-slam-node)
    - [Description and usage](#description-and-usage)
    - [GPS/SLAM calibration](#gpsslam-calibration)
    - [About the published TF tree](#about-the-published-tf-tree)
  - [Pose graph optimization node](#pose-graph-optimization-node)

Wrapping for Kitware LiDAR-only SLAM. It can also use GPS data to publish SLAM output pose as GPS coordinates, or to correct SLAM trajectory during post-processing.

## LiDAR SLAM node

### Description and usage

The raw SLAM node subscribes to velodyne pointclouds, and publishes the lidar odometry (as well as TF) of the current pose with respect to the initial pose.

SLAM node supports massive configuration from ROS parameter server (even if default values are already provided). An example of configuration file can be found in `params/slam_config.yaml`. All parameters have to be set as private parameters.

To start *lidar_slam_node* when replaying rosbag file, run :
```bash
rosbag play --clock <my_bag_file>
roslaunch lidar_slam slam.launch
```

When using it in real conditions, use :
```bash
roslaunch lidar_slam slam.launch use_sim_time:=false
```

This launch file will start a *lidar_slam_node*, a pre-configured RViz session, and GPS/UTM conversions nodes to publish SLAM pose as a GPS coordinate in WGS84 format, with the prior that full GPS pose and GPS/LiDAR calibration are correctly known and set (see [GPS/SLAM calibration](#gpsslam-calibration) section below). Input pointcloud fix must be a *sensor_msgs/PointCloud2* message (of points `velodyne_pointcloud::PointXYZIR`) published on topic '*velodyne_points*'. Input GPS fix must be a *gps_common/GPSFix* message published on topic '*gps_fix*'.

### GPS/SLAM calibration

To be able to publish local SLAM odometry as GPS coordinates, it is necessary to link SLAM initial pose to a 3D GPS pose (position + orientation) transformed into cartesian space with UTM projection. However, full 3D GPS pose is rarely known (we generally only have 3D position and sometimes an approximate yaw heading angle), and the calibration from GPS antenna to LiDAR sensor is always approximate (3D translation offset is more/less correct, but 3D rotation is unknown).

If these full 3D transforms are perfectly known, fill them in `launch/slam.launch` to ensure that the correct transformation is applied to output SLAM pose to the WGS84 format.

If these transforms are unknown, the SLAM node can try to auto-compute them for you. If this GPS/SLAM calibration is enabled, the node subscribes to the topic '*gps_odom*' records the last GPS and SLAM positions (and forgets the ones older than a pre-defined timeout threshold). The calibration process can be triggered at any time by publishing an empty message to '*run_gps_slam_calibration*' topic. When triggered, the recorded SLAM and GPS trajectories are aligned with ICP matching, giving the full 3D static transform to link a SLAM pose into GPS coordinates, published on TF server.
1. NOTE: During this auto-calibration process, GPS position should be precise enough to guarantee a robust calibration.
2. NOTE: As registration is done via ICP without any other prior, the trajectories need to have some turns in order to fully constrain the problem. If the movement is only following a straight line, 1 rotation remains unconstrained, and could lead to serious artefacts.
3. NOTE: To fix this straight line case, a supplementary prior can be introduced, imposing for example the output calibration to have no roll angle (hypothesis of flat ground plane in front direction). However, if ground is not flat, it could also lead to bad calibration.

To enable this GPS/SLAM auto-calibration, use option `gps:=true` :
```bash
roslaunch lidar_slam slam.launch gps:=true  # Start SLAM node and records GPS/SLAM previous positions.
...
rostopic pub -1 /run_gps_slam_calibration std_msgs/Empty "{}"  # Trigger GPS/SLAM calibration
```

### About the published TF tree

Here is the complete TF tree maintained by different nodes as well as descriptions of each frame :

```bash
utm
|__ gps_init
    |__ gps
    |__ slam_init
        |__ velodyne
```

- **utm**: "world" fixed frame, corresponding to the origin of the current considered UTM zone/band in which GPS coordinates are projected into.
- **gps_init**: first received GPS pose. It defines the origin of the local map, easier to use than UTM frame (because coordinates in UTM frames can be very large, leading to floating points discretization errors). If the GPS data provides an orientation, this frame is oriented alongside this direction. Otherwise, the frame orientation remains unchanged, and corresponds to UTM East-North-Up (ENU) coordinates, offset by the 1st GPS position. The static TF `utm -> gps_init` is published by `gps_conversions/gps_to_utm` node.
- **gps**: current GPS pose (pose of the GPS antenna). The TF `gps_init -> gps` is published by `gps_conversions/gps_to_utm` node.
- **slam_init**: initial pose of the SLAM, which is the pose of the velodyne sensor of the first received pointcloud. The static TF `gps_init -> slam_init` is published either by `lidar_slam/lidar_slam_node` node (in case of GPS/SLAM auto-calibration) or manually set with tf2 static publishers in `lidar_slam/launch/slam.launch` (in case of pre-defined calibration).
- **velodyne**: current pose of the velodyne sensor, according to SLAM. The TF `slam_init -> velodyne` is published by `lidar_slam/lidar_slam_node` node.


## Pose graph optimization node

This post-processing node records GPS and SLAM trajectories, and tries to correct SLAM drift by matching SLAM poses to temporal closest GPS poses. The pose graph optimization is achieved with G2O optimization framework. The goal is then to use this corrected SLAM trajectory to correct once and for all SLAM mapping, and enable a localization only mode using this pre-built valid map.

WARNING: This node is still under heavy development, its use may be buggy.