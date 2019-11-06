#ifndef LIDAR_SLAM_NODE_H
#define LIDAR_SLAM_NODE_H

// ROS
#include <ros/ros.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
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

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     New lidar frame callback, running SLAM and publishing TF.
   * @param[in] cloud New Lidar Frame, published by velodyne_pointcloud/cloud_node.
   */
  void ScanCallback(const CloudV& cloud);

private:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Convert a Velodyne pointcloud to the slam expected pointcloud format.
   * @param[in] cloudV Velodyne pointcloud, published by velodyne_pointcloud/cloud_node.
   * @return    The converted slam pointcloud.
   *
   * Velodyne pointcloud has fields : x, y, z, intensity (float), ring (uint16).
   * Slam pointcloud has fields     : x, y, z, intensity (uint8), laserId (uint8), time (double).
   */
  CloudS::Ptr ConvertToSlamPointCloud(const CloudV& cloudV);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Publish TF and PoseWithCovariance.
   * @param[in] headerCloudV   Header to use to fill seq and stamp of output msgs.
   * @param[in] worldTransform Transform from slam_init to velodyne to send.
   * @param[in] poseCovar      Covariance associated to full 6 dof pose.
   *
   * NOTE : poseCovar encodes covariance for dof in this order : (rX, rY, rZ, X, Y, Z)
   */
  void PublishTfPoseCovar(const pcl::PCLHeader& headerCloudV,
                          const Transform& worldTransform,
                          const std::vector<double>& poseCovar);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Publish additionnal info, such as Slam or features pointclouds.
   * @param[in] cloudS Slam pointcloud.
   */
  void PublishFeaturesMaps(const CloudS::Ptr& cloudS);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Get and fill Slam parameters from ROS parameters server.
   * @param[in] priv_nh Private ROS node handle to access parameters.
   */
  void SetSlamParameters(ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------

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