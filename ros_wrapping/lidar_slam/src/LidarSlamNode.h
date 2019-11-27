#ifndef LIDAR_SLAM_NODE_H
#define LIDAR_SLAM_NODE_H

// ROS
#include <ros/ros.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

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

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optionnal GPS odom callback, accumulating poses for SLAM/GPS calibration.
   * @param[in] msg Converted GPS pose with its associated covariance.
   */
  void GpsCallback(const nav_msgs::Odometry& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Triggers GpsSlamCalibration(), which runs GPS/SLAM calibration
   *            from recorded GPS and SLAM poses, and publish static TF to link
   *            SlamOriginFrameId to GPS frame.
   * @param[in] msg (Unused)
   */
  void RunGpsSlamCalibrationCallback(const std_msgs::Empty&);

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
   * @param[in] slamToLidar Transform from slam_init to velodyne to send.
   * @param[in] poseCovar      Covariance associated to full 6 dof pose.
   *
   * NOTE : poseCovar encodes covariance for dof in this order : (rX, rY, rZ, X, Y, Z)
   */
  void PublishTfOdom(const pcl::PCLHeader& headerCloudV,
                     const Transform& slamToLidar,
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
  /*!
   * @brief Run GPS/SLAM calibration from recorded GPS and SLAM poses, and
   *        publish static TF to link SlamOriginFrameId to GPS frame.
   */
  void GpsSlamCalibration();

  //----------------------------------------------------------------------------

  // SLAM stuff
  Slam LidarSlam;
  std::vector<size_t> LaserIdMapping;
  double LidarFreq = 10.;
  int Verbosity = 1;

  // Basic publishers & subscribers
  std::string SlamOriginFrameId = "slam_init";  ///< Frame id of SLAM map origin.
  std::string SlamOutputFrameId;  ///< Frame id of current SLAM pose (default : use frame_id of the input pointcloud).
  ros::Publisher PoseCovarPub;
  ros::Subscriber CloudSub;
  tf2_ros::TransformBroadcaster TfBroadcaster;

  // Optionnal publication of slam pose centered on GPS antenna instead of LiDAR sensor.
  bool OutputGpsPose = false;                 ///< Output GPS antenna pose instead of LiDAR's.
  std::string OutputGpsPoseFrameId = "slam";  ///< Frame id of the GPS antenna pose computed by SLAM if OutputGpsPose=true.
  Eigen::Isometry3d LidarToGpsOffset = Eigen::Isometry3d::Identity(); ///< Pose of the GPS antenna in LiDAR coordinates.

  // Optionnal use of GPS data to calibrate output SLAM pose to world coordinates.
  std::string GpsOriginFrameId = "gps_init";  ///< Storage of GPS odometry msg frame_id.
  bool CalibrateSlamGps = false;              ///< Enable GPS/SLAM calibration, and therefore
  bool CalibrationNoRoll = false;             // DEBUG
  double CalibrationPoseTimeout = 15.;        ///< [s] GPS/SLAM poses older than that are forgotten.
  std::deque<Transform> SlamPoses;            ///< Buffer of last computed SLAM poses.
  std::deque<Transform> GpsPoses;             ///< Buffer of last received GPS poses.
  ros::Subscriber GpsOdomSub;
  ros::Subscriber GpsSlamCalibrationSub;
  tf2_ros::StaticTransformBroadcaster StaticTfBroadcaster;

  // Debug publishers
  ros::Publisher GpsPathPub, SlamPathPub;
  ros::Publisher DebugCloudPub;
  ros::Publisher EdgesPub, PlanarsPub, BlobsPub;
  bool PublishIcpTrajectories = false;
  bool PublishEdges = false, PublishPlanars = false, PublishBlobs = false;
};

#endif // LIDAR_SLAM_NODE_H