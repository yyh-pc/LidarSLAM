# lidar_conversions

## Description

Helper package to convert raw pointclouds output by common LiDAR drivers to the pointcloud format expected by the SLAM algorithm.

The SLAM algorithm expects input pointclouds as *sensor_msgs/PointCloud2* messages. These pointclouds should have the following fields:
- **x**, **y**, **z** (`float`) : point coordinates
- **time** (`double`) : time offset to add to the pointcloud header timestamp to get approximate point-wise aquisition timestamp
- **intensity** (`float`) : intensity/reflectivity of the point
- **laser_id** (`uint16`) : numeric identifier of the laser ring that shot this point. The lowest/bottom laser ring should be 0, and it should increase upward.
- **device_id** (`uint8`) : numeric identifier of the LiDAR device/sensor. This id should be the same for all points of the cloud aquired by the same sensor.
- **label** (`uint8`) : optional input, not yet used.

Especially, the point-wise timestamps are necessary if undistortion is enabled in SLAM. The nodes of this package are able to compute approximate timestamps when those are not available in the input pointcloud.

SLAM expects that the lowest/bottom laser ring is 0, and is increasing upward. If this is not your case, you can use the `laser_id_mapping` to correct it in the output cloud.

Currently, this package implements the following nodes :
- **velodyne_conversion_node** : converts pointclouds output by Velodyne spinning sensors using the [ROS Velodyne driver](https://github.com/ros-drivers/velodyne) to SLAM pointcloud format.
- **robosense_conversion_node** : converts pointclouds output by RoboSense spinning sensors using the [ROS RoboSense-LiDAR driver](https://github.com/RoboSense-LiDAR/ros_rslidar) to SLAM pointcloud format. This has been tested only with RS16 sensor, and could need additional changes to support other RS sensors.

## Usage

Direct usage :

```bash
rosrun lidar_conversions velodyne_conversion_node
```

Example of launchfile for a multi-lidar setup:

```xml
<launch>
  <!-- LiDAR pointclouds conversions.
       The 'rpm' and 'timestamp_first_packet' parameters are only used to
       generate approximate point-wise timestamps if 'time' field is not usable.
       These 2 parameters should be set to the same values as ROS Velodyne/RSLidar drivers'. -->

  <node name="velodyne_conversion" pkg="lidar_conversions" type="velodyne_conversion_node" output="screen">
    <param name="device_id" value="0"/>
    <param name="rpm" value="600."/>
    <param name="timestamp_first_packet" value="false"/>
    <remap from="lidar_points" to="velodyne_lidar_points"/>
  </node>

  <node name="robosense_conversion" pkg="lidar_conversions" type="robosense_conversion_node" output="screen">
    <rosparam param="laser_id_mapping">[0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8]</rosparam>
    <param name="device_id" value="1"/>
    <param name="rpm" value="600."/>
    <remap from="lidar_points" to="robosense_lidar_points"/>
  </node>
</launch>
```