#ifndef LIDAR_SLAM_NODE_H
#define LIDAR_SLAM_NODE_H

// ROS
#include <ros/ros.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf2_ros/transform_broadcaster.h>

// PCL
#include <pcl_ros/point_cloud.h>

// SLAM
#include <Slam.h>

class LidarSlamNode
{
public:
  using PointV = velodyne_pointcloud::PointXYZIR;  ///< Pointcloud published by velodyne driver
  using PointS = Slam::Point;                      ///< Pointcloud needed by SLAM

  LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  void scanCallback(const pcl::PointCloud<PointV>& cloud);

private:
  Slam slam_;
  std::vector<size_t> laserIdMapping_;

  // ROS publishers & subscribers
  ros::Publisher debugCloudPub_;
  ros::Subscriber cloudSub_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
};

#endif // LIDAR_SLAM_NODE_H