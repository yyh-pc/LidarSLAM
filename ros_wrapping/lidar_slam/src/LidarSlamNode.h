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

  void publishFeaturesMaps(const CloudS::Ptr& cloudS);

  void SetSlamParameters(ros::NodeHandle& priv_nh);

  // SLAM stuff
  Slam LidarSlam;
  std::vector<size_t> LaserIdMapping;
  double LidarFreq = 10.;

  // ROS publishers & subscribers
  std::string SlamOriginFrameId = "slam_init";
  ros::Publisher PoseCovarPub;
  ros::Subscriber CloudSub;
  tf2_ros::TransformBroadcaster TfBroadcaster;

  // Debug publishers
  ros::Publisher DebugCloudPub;
  ros::Publisher EdgesPub, PlanarsPub, BlobsPub;
  bool PublishEdges = false, PublishPlanars = false, PublishBlobs = false;

};

#endif // LIDAR_SLAM_NODE_H