//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-10-24
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#ifndef LIDAR_SLAM_NODE_H
#define LIDAR_SLAM_NODE_H

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <lidar_slam/SlamCommand.h>
#include <lidar_slam/Confidence.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

// SLAM
#include <LidarSlam/Slam.h>

class LidarSlamNode
{
public:

  using PointS = LidarSlam::Slam::Point;
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
   * @brief     Destructor.
   *
   * Used to shut down external spinners
   */
  ~LidarSlamNode();

  //----------------------------------------------------------------------------
  /*!
   * @brief     New main LiDAR frame callback, running SLAM and publishing TF.
   * @param[in] cloud New frame, published by conversion node.
   *
   * Input pointcloud must have following fields :
   *  - x, y, z (float): point coordinates
   *  - time (double): time offset to add to the pointcloud header timestamp to
   *    get approximate point-wise acquisition timestamp
   *  - intensity (float): intensity/reflectivity of the point
   *  - laser_id (uint16): numeric identifier of the laser ring that shot this point.
   *    The lowest/bottom laser ring should be 0, and it should increase upward.
   *  - device_id (uint8): numeric identifier of the LiDAR device/sensor.
   *    This id should be the same for all points of the cloud acquired by the same sensor.
   *  - label (uint8): optional input, not yet used.
   */
  virtual void ScanCallback(const CloudS::Ptr cloudS_ptr);

  //----------------------------------------------------------------------------
  /*!
   * @brief     New secondary lidar frame callback, buffered to be latered processed by SLAM.
   * @param[in] cloud New frame, published by conversion node.
   *
   * Input pointcloud must have following fields :
   *  - x, y, z (float): point coordinates
   *  - time (double): time offset to add to the pointcloud header timestamp to
   *    get approximate point-wise acquisition timestamp
   *  - intensity (float): intensity/reflectivity of the point
   *  - laser_id (uint16): numeric identifier of the laser ring that shot this point.
   *    The lowest/bottom laser ring should be 0, and it should increase upward.
   *  - device_id (uint8): numeric identifier of the LiDAR device/sensor.
   *    This id should be the same for all points of the cloud acquired by the same sensor.
   *  - label (uint8): optional input, not yet used.
   */
  virtual void SecondaryScanCallback(const CloudS::Ptr cloudS_ptr);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional GPS odom callback, accumulating poses.
   * @param[in] msg Converted GPS pose with its associated covariance.
   */
  void GpsCallback(const nav_msgs::Odometry& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Optional tag detection callback, adding a landmark relative pose to the SLAM
   * @param[in] msg april tag node output message
   */
  void TagCallback(const apriltag_ros::AprilTagDetectionArray& tagInfo);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Set SLAM pose from external guess.
   * @param[in] msg The pose to use.
   *
   * NOTE: A valid TF tree must link msg.header.frame_id to OdometryFrameId.
   * NOTE: The covariance is not used yet.
   */
  void SetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Receive an external command to process, such as pose graph
   *            optimization, GPS/SLAM calibration, set SLAM pose, save maps etc.
   * @param[in] msg The command message.
   */
  void SlamCommandCallback(const lidar_slam::SlamCommand& msg);

protected:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Update transform offset between BASE and LIDAR using TF2
   * @param[in] lidarFrameId The input LiDAR pointcloud frame_id.
   * @param[in] lidarDeviceId The numerical identifier of the LiDAR sensor.
   */
  bool UpdateBaseToLidarOffset(const std::string& lidarFrameId, uint8_t lidarDeviceId);

  //----------------------------------------------------------------------------
  /*!
   * @brief Publish SLAM outputs as requested by user.
   *
   * It is possible to send :
   *  - pose and covariance as Odometry msg or TF
   *  - extracted keypoints from current frame
   *  - keypoints maps
   *  - undistorted input points registered in odometry frame
   */
  void PublishOutput();

  //----------------------------------------------------------------------------
  /*!
   * @brief Get and fill Slam parameters from ROS parameters server.
   */
  void SetSlamParameters();

  //----------------------------------------------------------------------------
  /*!
   * @brief Fill the SLAM initial state with the given initial maps, pose and
   *        landmarks.
   */
  void SetSlamInitialState();

  //----------------------------------------------------------------------------
  /*!
   * @brief Get and fill landmarks managers with absolute pose information
   *        provided by the user in a csv file.
   *        The fields must be : idx, x, y, z, roll, pitch, yaw, cov0, ..., cov35
   *        The delimiters can be "," ";" " " "/t"
   *        /!\ order matters
   *        A header line can be added
   */
  void LoadLandmarks(const std::string& path);

  //----------------------------------------------------------------------------
  /*!
   * @brief Build an id for the april tag output message
   *        if it gives the info of one landmark, the id is the one of this landMark
   *        if it gives the info of a tag bundle, the id is built as [idN [...] id1 id0]
   */
  int BuildId(const std::vector<int>& ids);

  // Publish static tf to link world (UTM) frame to SLAM origin
  // PGO must have been run, so we can average
  // the correspondant poses (GPS/LidarSLAM) distances to get the offset
  void BroadcastGpsOffset();

  //----------------------------------------------------------------------------

  // SLAM stuff
  LidarSlam::Slam LidarSlam;
  std::vector<CloudS::Ptr> Frames;

  // ROS node handles, subscribers and publishers
  ros::NodeHandle &Nh, &PrivNh;
  std::vector<ros::Subscriber> CloudSubs;
  ros::Subscriber SlamCommandSub, SetPoseSub;
  std::unordered_map<int, ros::Publisher> Publishers;
  std::unordered_map<int, bool> Publish;

  // TF stuff
  std::string OdometryFrameId = "odom";       ///< Frame in which SLAM odometry and maps are expressed.
  std::string TrackingFrameId = "base_link";  ///< Frame to track (ensure a valid TF tree is published).
  std::string GpsFrameId = "GPS"; ///< Frame to represent GPS positions.
  ros::Time GpsLastTime;
  tf2_ros::Buffer TfBuffer;
  tf2_ros::TransformListener TfListener;
  tf2_ros::TransformBroadcaster TfBroadcaster;
  tf2_ros::StaticTransformBroadcaster StaticTfBroadcaster;

  // External sensors
  // Multithreaded reception of sensor data
  std::shared_ptr<ros::AsyncSpinner> ExternalSpinnerPtr;
  ros::CallbackQueue ExternalQueue;
  // Landmarks
  bool UseTags = false;
  ros::Subscriber LandmarksSub;
  bool PublishTags = false;
  LidarSlam::ExternalSensors::GpsMeasurement LastGpsMeas;

  // GPS
  bool UseGps = false; ///< Enable GPS data logging for Pose Graph Optimization or GPS/SLAM calibration.
  Eigen::Isometry3d BaseToGpsOffset = Eigen::Isometry3d::Identity();  ///< Pose of the GPS antenna in BASE coordinates.
  ros::Subscriber GpsOdomSub;
};

#endif // LIDAR_SLAM_NODE_H