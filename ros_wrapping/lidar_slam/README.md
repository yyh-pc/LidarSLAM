# lidar_slam

- [lidar_slam](#lidar_slam)
  - [LiDAR SLAM node](#lidar-slam-node)
    - [Description and basic usage](#description-and-basic-usage)
    - [More advanced usage](#more-advanced-usage)
  - [Optional GPS use](#optional-gps-use)
    - [Map (GPS) / Odom (SLAM) calibration](#map-gps--odom-slam-calibration)
    - [SLAM pose graph optimization (PGO) with GPS prior](#slam-pose-graph-optimization-pgo-with-gps-prior)
    - [Running SLAM on same zone at different times (e.g. refining/aggregating map or running localization only on fixed map)](#running-slam-on-same-zone-at-different-times-eg-refiningaggregating-map-or-running-localization-only-on-fixed-map)
      - [Setting SLAM pose from GPS pose guess](#setting-slam-pose-from-gps-pose-guess)
      - [Running SLAM on same zone](#running-slam-on-same-zone)
  - [About the published TF tree](#about-the-published-tf-tree)
  - [Points aggregation](#points-aggregation)

Wrapping for Kitware LiDAR-only SLAM. It can also use GPS data to publish SLAM output pose as GPS coordinates, or to correct SLAM trajectory and map.

## LiDAR SLAM node

### Description and basic usage

The SLAM node subscribes to one or several topics of LiDAR pointclouds, and computes current pose of the tracked frame relative to the fixed odometry frame. It can output various data, such as SLAM pose (as Odometry msg or TF), keypoint maps, etc.

SLAM node supports massive configuration from ROS parameter server (even if default values are already provided). Examples of yaml configuration files can be found in [`params/`](params/). All parameters have to be set as private parameters.

To start only raw LiDAR SLAM, just start *lidar_slam_node*:
```bash
rosrun lidar_slam lidar_slam_node
```
If you want to specify parameters, you should consider using a launchfile. Two launch files are provided in the folder [`launch/`](launch/).

With velodyne launch file:
- To start SLAM when replaying a velodyne rosbag file, run :
```bash
roslaunch lidar_slam slam_velodyne.launch   # in 1st shell
rosbag play --clock <my_bag_file>  # in 2nd shell
```
- When using it in real live conditions, use :
```bash
roslaunch lidar_slam slam_velodyne.launch use_sim_time:=false
```

With ouster launch file:
- To start SLAM when replaying a ouster rosbag file, run :
```bash
roslaunch lidar_slam slam_ouster.launch   # in 1st shell
rosbag play --clock <my_bag_file>  # in 2nd shell
```
- When using it in real live conditions, use :
```bash
roslaunch lidar_slam slam_ouster.launch replay:=false
```

These launch files will start :

* the *lidar_slam_node* which implements the SLAM process,
* a pre-configured RViz session,
* the *lidar_conversion_node* which converts the driver point type to expected SLAM use (see next paragraph),
* (optional) The Lidar drivers if required (see vlp16_driver and os_driver parameters),
* (optional) GPS/UTM conversions nodes to publish SLAM pose as a GPS coordinate in WGS84 format (if `gps` arg is enabled). This uses the prior that full GPS pose and GPS/LiDAR calibration are correctly known and set (see [GPS/SLAM calibration](#gpsslam-calibration) section below for more info).


### More advanced usage

#### Detailed pipeline

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
- Current target keypoints maps (i.e submaps) as *sensor_msgs/PointCloud2* on topics '*submaps/{edges,planes,blobs}*';
- registered and undistorted point cloud from current frame, in odometry frame, as *sensor_msgs/PointCloud2* on topic '*slam_registered_points*';
- confidence estimations on pose output, as *lidar_slam/Confidence* custom message on topic '*slam_confidence*'. It contains the pose covariance, an overlap estimation, the number of matched keypoints, a binary estimator to check motion limitations and the computation time.

UTM/GPS conversion node can output SLAM pose as a *gps_common/GPSFix* message on topic '*slam_fix*'.

**NOTE** : It is possible to track any *tracking_frame* in *odometry_frame*, using a pointcloud expressed in an *lidar_frame*. However, please ensure that a valid TF tree is beeing published to link *lidar_frame* to *tracking_frame*.

#### Online configuration

##### Map update modes

At any time, commands `lidar_slam/SlamCommand/DISABLE_SLAM_MAP_UPDATE`, `lidar_slam/SlamCommand/ENABLE_SLAM_MAP_EXPANSION` and `lidar_slam/SlamCommand/ENABLE_SLAM_MAP_UPDATE` can be published to '*slam_command*' topic to change SLAM map update mode.
- `DISABLE_SLAM_MAP_UPDATE` : when an initial map is loaded, it is kept untouched through the SLAM process.
- `ENABLE_SLAM_MAP_EXPANSION` : when an initial map is loaded, its points are remained untouched but new points can be added if they lay in an unexplored area
- `ENABLE_SLAM_MAP_UPDATE` : the map is updated at any time

_NOTE_ : if no initial map is loaded, ENABLE_SLAM_MAP_EXPANSION and ENABLE_SLAM_MAP_UPDATE will have the same effect.

Example :
```bash
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 9"
```

##### Save maps

The current maps can be saved as PCD at any time publishing the command `SAVE_KEYPOINTS_MAPS` (to save the whole maps) or the command `SAVE_FILTERED_KEYPOINTS_MAPS` (to remove the potential moving objects) to `slam_command` topic:

```bash
rostopic pub -1 /slam_command lidar_slam/SlamCommand "{command: 16, string_arg: /path/to/maps/prefix}"
```
OR
```bash
rostopic pub -1 /slam_command lidar_slam/SlamCommand "{command: 17, string_arg: /path/to/maps_filtered/prefix}"
```

##### Set pose
At any time, a pose message (`PoseWithCovarianceStamped`) can be sent through the topic `set_slam_pose` to reset the current pose

## Optional GPS use

If GPS use is enabled, *LidarSlamNode* subscribes to the GPS odometry on topic '*gps_odom*', and records the most recent GPS positions. To use GPS data, we transform GPS WGS84 fix into cartesian space using UTM projection. This can be used to estimate calibration between GPS and SLAM trajectories, or post-optimize SLAM trajectory with pose graph optimization (PGO).

To use GPS data :
- enable gps use : `external_sensors/gps/use_gps = true`:
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
roslaunch lidar_slam slam_velodyne.launch gps:=true  # Start SLAM node and enable GPS use.
...
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 0"  # Trigger GPS/SLAM calibration
```

### SLAM pose graph optimization (PGO) with GPS prior

Available GPS positions can also be used to optimize the SLAM trajectory by correcting drift error accumulated over time. The GPS positions and their associated covariances can be used as priors to optimize the SLAM pose graph with g2o framework. SLAM maps will also be corrected. The [map/odom calibration](#map-gps--odom-slam-calibration) will also be computed and published as a static TF (but should be more precise than the global ICP calibration process).

PGO can be triggered at any time by publishing the `lidar_slam/SlamCommand/GPS_SLAM_POSE_GRAPH_OPTIMIZATION` command to '*slam_command*' topic.

NOTE: This PGO is not real-time, and should therefore be run when system is not or slowly moving.

To enable PGO, you can use option `gps:=true` :
```bash
roslaunch lidar_slam slam_velodyne.launch gps:=true  # Start SLAM node and enable GPS use.
...
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 20"  # Trigger PGO
```

### Running SLAM on same zone at different times (e.g. refining/aggregating map or running localization only on fixed map)

#### Setting SLAM pose from GPS pose guess

If you want to run another bag on the same zone to refine the SLAM map or to run localization only with the previously built map, you need to give an approximate new init pose to SLAM if trajectory is not continuous with end pose. You can send `lidar_slam/SlamCommand/SET_SLAM_POSE_FROM_GPS` command to '*slam_command*' topic to use the last received GPS position in a pose guess for SLAM.

NOTE: To be able to use this command, SLAM and GPS coordinates must be precisely linked with a valid TF tree. Be sure you already called [pose graph optimization](#slam-pose-graph-optimization-pgo-with-gps-prior) or at least [map/odom calibration](#map-gps--odom-slam-calibration). Moreover, the orientation will be set as odometry frame.

#### Running SLAM on same zone

To sum up, if you want to run SLAM on same zone, use :
```bash
roslaunch lidar_slam slam_velodyne.launch gps:=true  # Start SLAM node and enable GPS use.
...  # Run 1st real test or bag file
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 20"   # Trigger PGO : optimize SLAM map and compute GPS/SLAM calibration
(rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 8")  # Disable SLAM map update (optional)
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 2"    # If the starting pose of the new bag does not match with last SLAM pose, use GPS position to set an initial guess. Warning, if the orientation has changed to much, SLAM may not converge.
...  # Run 2nd real test or bag file
```

## Optional Landmarks use

### Local optimization

If tags use is enabled, *LidarSlamNode* subscribes to the tag odometry on topic '*tag_detections*', and records the most recent tag relative poses.

One can use landmarks with a camera to detect them. The use conditions are the following :
1. The landmarks info must come under the shape of `apriltag_ros` messages.
2. The default reception topic is `tag_detections`.
3. The landmark detector (e.g. a camera) transform relatively to *tracking_frame* must be supplied as a `TF2 static transform`.
4. The parameter `use_tags` must be set to `true`.
5. Log enough tag measurements so an interpolation can be performed to synchronize them with the Lidar. `max_measures >= N`, with N being the number of the newest tag detections that must be stored so that when the SLAM needs a synchronized measure it finds two bounding measures in the N ones. It mostly depends on the memory capacity and the camera frequency.

***WARNING***: A virtual package of apriltag_ros exists in this wrapping for compilation issue and must be ignored if one wants to use the actual apriltag_ros package.

**NOTE** : You can specify the tag topic directly in the launch command with the argument : `tags_topic:="your_tag_topic"`.

Then, the lidar_slam node will receive the tag detection messages and a constraint will be built to be included in the local optimization together with the internal geometric constraints.

Various relative parameters are available :
1. `weight` : floating value to mitigate the tag impact on the optimization relatively to geometric matches.
2. `position_only=true` : only use the tags' positions and not the orientations provided by the detector. Orientation is usually less accurate.
3. `saturation_distance` : reject the measurements that seem clearly wrong relatively to the geometric matches.
4. `publish_tags=true` : publish the tag as TF2 transform for visualization purposes mostly (parameter ).
5. `landmarks_file_path` : name of a file to load, to initially set absolute poses of the tags. The tag constraints are built relatively to these absolute reference poses and they will never be modified along iterations. If no file is loaded, the reference poses in the constraints are computed using previous detections and are refined along iterations or reset if the tag is not seen in a while. Note that if a file is loaded in mapping mode and some drift appears, it can lead to some jumps and a map distortion. The file format must be csv with one header line : *id,x,y,z,roll,pitch,yaw,cov0,[...],cov35*.

All these parameters are described in the supplied config files.

***WARNING***: Make sure no error occured in the file loading step in the terminal output before supplying data.

### Pose graph optimization

If the tags were activated, one can run a post optimization, using the tags to close loops and to create a consistent map. The previous recommandations for landmarks integration in local optimizations hold. Moreover, the LiDAR states logging must be enabled : `slam/logging_timeout > t`, t being the duration on which LiDAR measurements can be stored. The oldest measurements are forgotten. Only the remaining poses are used to build and optimize the graph and to rebuild the map.

If you want to run SLAM and then to optimize the graph using landmarks, one pipeline can be :
```bash
roslaunch lidar_slam slam_velodyne.launch tags_topic:="your_tag_topic" # Start SLAM (tags use must be enabled).
...  # Run 1st real test or bag file
...  # Stop the acquisition or pause the system
rostopic pub -1 /slam_command lidar_slam/SlamCommand "{command: 20, string_arg: path/to/landmarksAbsolutePoses.csv}"    # Trigger PGO : optimize SLAM map and update last pose (if fix_last is set to false)
rostopic pub -1 /slam_command lidar_slam/SlamCommand "{command: 16, string_arg: /path/to/maps/prefix}" # Save keypoint maps for further use
...  # Run 2nd real test or bag file
```

**NOTE** : After one pose graph optimization one should be able to proceed with the acquisition/replay.

**NOTE** : If the SLAM was running with initially loaded tag poses (`landmarks_file_path` is not empty), and one wants to run the pose graph optimization, the file parameter is not compulsory, one can just launch :
```bash
rostopic pub -1 /slam_command lidar_slam/SlamCommand "command: 20"
```
Again, running the SLAM in mapping mode with initially loaded tag poses can be dangerous though. If you don't have any absolute tag poses to supply, the last estimated tag poses will be used. It means the map will be updated so it fits the last frames.

## About the published TF tree

Here is an example of the complete TF tree that can be maintained by different nodes as well as descriptions of each frame (default frame names) :

```bash
utm
└─ enu
   └─ map
      └─ odom
         └─ base_link
            ├─ lidar
            ├─ landmark_detector
            └─ gps
```

- **utm**: "world" ENU fixed frame, corresponding to the origin of the current considered UTM zone/band in which GPS coordinates are projected into.
- **enu**: local ENU fixed frame attached to 1st received GPS position in UTM coordinates, easier to use than **utm** because UTM coordinates can grow very large, leading to floating points discretization errors. The static TF `utm -> enu` is published by `gps_conversions/gps_to_utm` node.
- **map**: first received full 6D GPS pose. It defines the origin of the local map. If GPS does not provide orientation, pitch and heading can be estimated from motion. The static TF `enu -> map` is published by `gps_conversions/gps_to_utm` node.
- **odom**: origin of the SLAM. The TF `map -> odom` can be published by a custom node, by `lidar_slam/lidar_slam_node` node (in case of GPS/SLAM auto-calibration or PGO), or manually set with tf2 static publishers in [`launch/slam_velodyne.launch`](launch/slam_velodyne.launch) (in case of pre-defined calibration).
- **base_link**: current pose computed by SLAM algorithm (here `base_link` is the tracking frame). The TF `odom -> base_link` can be published by `lidar_slam/lidar_slam_node` node.
- **lidar**: pose of the LiDAR sensor on the moving base. The TF `base_link -> lidar` should be published by a `tf2_ros/static_transform_publisher` node.
- **landmark_detector**: pose of the landmark detector on the moving base. The TF `base_link -> landmark_detector` must be published by a `tf2_ros/static_transform_publisher` node.
- **gps**: pose of the GPS sensor on the moving base. The TF `base_link -> gps` should be published by a `tf2_ros/static_transform_publisher` node.

# Points aggregation

Another node called **aggregation_node** is included in the **lidar_slam** package and allows to aggregate all points from all frames into a unique pointcloud with a defined resolution : the points are stored in a voxel grid to supply a downsampled output cloud. It also allows to reject moving objects. The output pointcloud is published on the topic *aggregated_cloud*. It requires the output *registered_points* of the node **lidar_slam_node** to be enabled.

**aggregation_node** has 3 parameters :

- *leaf_size* : corresponds to the size of a voxel in meters in which to store one unique point. It is equivalent to the required mean distance between nearest neighbors. The maximum distance between nearest distance to downsample the cloud would be 2 * leaf_size
- *max_size* : corresponds to the maximum size of the voxel grid and so the maximum width of the output cloud to tackle possible memory issues.
- *min_points_per_voxel* : corresponds to the minimum number of frames which should have a point in a voxel to consider this voxel is not a moving object. It has been seen more than *min_points_per_voxel*, so it is considered "enough" to be a static object. All voxels that have been seen less than *min_points_per_voxel* times are not included to the output cloud.

This node can answer to a service called **save_pc** to save the pointcloud on disk as a PCD file. The command should be :

```bash
rosservice call lidar_slam/save_pc "prefix/path/where/to/save/the/cloud" 0
```

To save the pointcloud as a PCD ASCII file at *prefix/path/where/to/save/the/cloud_CurrentTime.pcd*.
All possible formats are :
- **ASCII** : 0
- **Binary** : 1
- **Binary compressed** : 2

**NOTE** : **aggregation_node** can be directly run from the launch files, adding the argument *aggregation:=true* to the launch command.