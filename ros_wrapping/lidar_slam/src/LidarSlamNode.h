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

  using PointV = velodyne_pointcloud::PointXYZIR;
  using CloudV = pcl::PointCloud<PointV>;  ///< Pointcloud published by velodyne driver
  using PointS = Slam::Point;    
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM

  LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  void scanCallback(const CloudV& cloud);

private:

  CloudS::Ptr convertToSlamPointCloud(const CloudV& cloudV);

  void publishTfPoseCovar(const pcl::PCLHeader& headerCloudV, 
                          const Transform& worldTransform, 
                          const std::vector<double>& poseCovar);

  Slam slam_;
  std::vector<size_t> laserIdMapping_;

  // ROS publishers & subscribers
  ros::Publisher debugCloudPub_;
  ros::Publisher poseCovarPub_;
  ros::Subscriber cloudSub_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
};

#endif // LIDAR_SLAM_NODE_H