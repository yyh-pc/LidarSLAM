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

#include "LidarSlamNode.h"
#include "ros_transform_utils.h"
#include <LidarSlam/GlobalTrajectoriesRegistration.h>

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

enum Output
{
  POSE_ODOM,             // Publish SLAM pose as an Odometry msg on 'slam_odom' topic (default : true).
  POSE_TF,               // Publish SLAM pose as a TF from 'odometry_frame' to 'tracking_frame' (default : true).
  POSE_PREDICTION_ODOM,  // Publish latency-corrected SLAM pose as an Odometry msg on 'slam_predicted_odom' topic.
  POSE_PREDICTION_TF,    // Publish latency-corrected SLAM pose as a TF from 'odometry_frame' to '<tracking_frame>_prediction'.

  EDGES_MAP,             // Publish edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/edges'.
  PLANES_MAP,            // Publish plane keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/planes'.
  BLOBS_MAP,             // Publish blob keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/blobs'.

  EDGE_KEYPOINTS,       // Publish extracted edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/edges'.
  PLANE_KEYPOINTS,      // Publish extracted plane keypoints from current frame as a PointCloud2 msg to topic 'keypoints/planes'.
  BLOB_KEYPOINTS,       // Publish extracted blob keypoints from current frame as a PointCloud2 msg to topic 'keypoints/blobs'.

  SLAM_REGISTERED_POINTS,// Publish SLAM pointcloud as LidarPoint PointCloud2 msg to topic 'slam_registered_points'.

  PGO_PATH,              // Publish optimized SLAM trajectory as Path msg to 'pgo_slam_path' latched topic.
  ICP_CALIB_SLAM_PATH,   // Publish ICP-aligned SLAM trajectory as Path msg to 'icp_slam_path' latched topic.
  ICP_CALIB_GPS_PATH     // Publish ICP-aligned GPS trajectory as Path msg to 'icp_gps_path' latched topic.
};

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
  , TfListener(TfBuffer)
{
  // ***************************************************************************
  // Init SLAM state

  // Get SLAM params
  this->SetSlamParameters();

  // Load initial SLAM maps if requested
  std::string mapsPathPrefix = priv_nh.param<std::string>("maps/initial_maps", "");
  if (!mapsPathPrefix.empty())
  {
    ROS_INFO_STREAM("Loading initial keypoints maps from PCD.");
    this->LidarSlam.LoadMapsFromPCD(mapsPathPrefix);
  }

  // Freeze SLAM maps if requested
  if (!priv_nh.param("maps/update_maps", true))
  {
    this->LidarSlam.SetUpdateMap(false);
    ROS_WARN_STREAM("Disabling SLAM maps update.");
  }

  // Set initial SLAM pose if requested
  std::vector<double> initialPose;
  if (priv_nh.getParam("maps/initial_pose", initialPose) && initialPose.size() == 6)
  {
    LidarSlam::Transform pose(Eigen::Map<const Eigen::Vector6d>(initialPose.data()));
    this->LidarSlam.SetWorldTransformFromGuess(pose);
    ROS_INFO_STREAM("Setting initial SLAM pose to:\n" << pose.GetMatrix());
  }

  // Use GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  priv_nh.getParam("gps/use_gps", this->UseGps);

  // ***************************************************************************
  // Init ROS publishers

  #define initPublisher(publisher, topic, type, rosParam, publishDefault, queue, latch)   \
    priv_nh.param(rosParam, this->Publish[publisher], publishDefault);                    \
    if (this->Publish[publisher])                                                         \
      this->Publishers[publisher] = nh.advertise<type>(topic, queue, latch);

  priv_nh.param("output/pose/tf",           this->Publish[POSE_TF],            true);
  priv_nh.param("output/pose/predicted_tf", this->Publish[POSE_PREDICTION_TF], false);
  initPublisher(POSE_ODOM,            "slam_odom",           nav_msgs::Odometry, "output/pose/odom",           true,  1, false);
  initPublisher(POSE_PREDICTION_ODOM, "slam_predicted_odom", nav_msgs::Odometry, "output/pose/predicted_odom", false, 1, false);

  initPublisher(EDGES_MAP,  "maps/edges",  CloudS, "output/maps/edges",  true, 1, false);
  initPublisher(PLANES_MAP, "maps/planes", CloudS, "output/maps/planes", true, 1, false);
  initPublisher(BLOBS_MAP,  "maps/blobs",  CloudS, "output/maps/blobs",  true, 1, false);

  initPublisher(EDGE_KEYPOINTS,  "keypoints/edges",  CloudS, "output/keypoints/edges",  true, 1, false);
  initPublisher(PLANE_KEYPOINTS, "keypoints/planes", CloudS, "output/keypoints/planes", true, 1, false);
  initPublisher(BLOB_KEYPOINTS,  "keypoints/blobs",  CloudS, "output/keypoints/blobs",  true, 1, false);

  initPublisher(SLAM_REGISTERED_POINTS, "slam_registered_points", CloudS, "output/registered_points", true, 1, false);

  if (this->UseGps)
  {
    initPublisher(PGO_PATH,            "pgo_slam_path", nav_msgs::Path, "gps/pgo/publish_path",              false, 1, true);
    initPublisher(ICP_CALIB_SLAM_PATH, "icp_slam_path", nav_msgs::Path, "gps/calibration/publish_icp_paths", false, 1, true);
    initPublisher(ICP_CALIB_GPS_PATH,  "icp_gps_path",  nav_msgs::Path, "gps/calibration/publish_icp_paths", false, 1, true);
  }

  // ***************************************************************************
  // Init ROS subscribers

  // LiDAR inputs
  std::vector<std::string> lidarTopics;
  if (!priv_nh.getParam("input", lidarTopics))
    lidarTopics.push_back(priv_nh.param<std::string>("input", "lidar_points"));
  this->CloudSubs.push_back(nh.subscribe(lidarTopics[0], 1, &LidarSlamNode::ScanCallback, this));
  ROS_INFO_STREAM("Using LiDAR frames on topic '" << lidarTopics[0] << "'");
  for (unsigned int lidarTopicId = 1; lidarTopicId < lidarTopics.size(); lidarTopicId++)
  {
    this->CloudSubs.push_back(nh.subscribe(lidarTopics[lidarTopicId], 1, &LidarSlamNode::SecondaryScanCallback, this));
    ROS_INFO_STREAM("Using secondary LiDAR frames on topic '" << lidarTopics[lidarTopicId] << "'");
  }

  // Set SLAM pose from external guess
  this->SetPoseSub = nh.subscribe("set_slam_pose", 1, &LidarSlamNode::SetPoseCallback, this);

  // SLAM commands
  this->SlamCommandSub = nh.subscribe("slam_command", 1, &LidarSlamNode::SlamCommandCallback, this);

  // Init logging of GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  if (this->UseGps)
    this->GpsOdomSub = nh.subscribe("gps_odom", 1, &LidarSlamNode::GpsCallback, this);

  ROS_INFO_STREAM(BOLD_GREEN("LiDAR SLAM is ready !"));
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallback(const CloudS::Ptr cloudS_ptr)
{
  if(cloudS_ptr->empty())
  {
    ROS_WARN_STREAM("Input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  // Update TF from BASE to LiDAR
  this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id);

  // Set the SLAM main input frame at first position
  this->Frames.insert(this->Frames.begin(), cloudS_ptr);

  // Run SLAM : register new frame and update localization and map.
  this->LidarSlam.AddFrames(this->Frames);
  this->Frames.clear();

  // Publish SLAM output as requested by user
  this->PublishOutput();
}

//------------------------------------------------------------------------------
void LidarSlamNode::SecondaryScanCallback(const CloudS::Ptr cloudS_ptr)
{
  if(cloudS_ptr->empty())
  {
    ROS_WARN_STREAM("Secondary input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  // Update TF from BASE to LiDAR for this device
  this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id);

  // Add new frame to SLAM input frames
  this->Frames.push_back(cloudS_ptr);
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::Odometry& msg)
{
  // If GPS/SLAM calibration is needed, save GPS pose for later use
  if (this->UseGps)
  {
    // Add new pose and its covariance to buffer
    const auto& c = msg.pose.covariance;
    std::array<double, 9> gpsCovar = {c[ 0], c[ 1], c[ 2],
                                      c[ 6], c[ 7], c[ 8],
                                      c[12], c[13], c[14]};
    this->GpsPoses.emplace_back(Utils::PoseMsgToTransform(msg.pose.pose, msg.header.stamp.toSec(), msg.header.frame_id));
    this->GpsCovars.emplace_back(gpsCovar);

    double loggingTimeout = this->LidarSlam.GetLoggingTimeout();
    // If a timeout is defined, forget too old data
    if (loggingTimeout > 0)
    {
      // Forget all previous poses older than loggingTimeout
      while (this->GpsPoses.back().time - this->GpsPoses.front().time > loggingTimeout)
      {
        this->GpsPoses.pop_front();
        this->GpsCovars.pop_front();
      }
    }

    // Update BASE to GPS offset
    // Get the latest transform (we expect a static transform, so timestamp does not matter)
    Utils::Tf2LookupTransform(this->BaseToGpsOffset, this->TfBuffer, this->TrackingFrameId, msg.child_frame_id);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  // Get transform between msg frame and odometry frame
  Eigen::Isometry3d msgFrameToOdom;
  if (Utils::Tf2LookupTransform(msgFrameToOdom, this->TfBuffer, msg.header.frame_id, this->OdometryFrameId, msg.header.stamp))
  {
    // Compute pose in odometry frame and set SLAM pose
    Eigen::Isometry3d odomToBase = msgFrameToOdom.inverse() * Utils::PoseMsgToTransform(msg.pose.pose).GetIsometry();
    this->LidarSlam.SetWorldTransformFromGuess(LidarSlam::Transform(odomToBase));
    ROS_WARN_STREAM("SLAM pose set to :\n" << odomToBase.matrix());
    // TODO: properly deal with covariance: rotate it, pass it to SLAM, notify trajectory jump?
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SlamCommandCallback(const lidar_slam::SlamCommand& msg)
{
  // Parse command
  switch(msg.command)
  {
    // Run GPS/SLAM calibration
    case lidar_slam::SlamCommand::GPS_SLAM_CALIBRATION:
      this->GpsSlamCalibration();
      break;

    // Run SLAM pose graph optimization with GPS positions prior
    case lidar_slam::SlamCommand::GPS_SLAM_POSE_GRAPH_OPTIMIZATION:
      // TODO : run PGO in separated thread
      this->PoseGraphOptimization();
      break;

    // Set SLAM pose from last received GPS pose
    // NOTE : This function should only be called after PGO or SLAM/GPS calib have been triggered.
    case lidar_slam::SlamCommand::SET_SLAM_POSE_FROM_GPS:
    {
      if (!this->UseGps)
      {
        ROS_ERROR_STREAM("Cannot set SLAM pose from GPS as GPS logging has not been enabled. "
                         "Please set 'gps/use_gps' private parameter to 'true'.");
        return;
      }
      if (this->GpsPoses.empty())
      {
        ROS_ERROR_STREAM("Cannot set SLAM pose from GPS as no GPS pose has been received yet.");
        return;
      }
      LidarSlam::Transform& mapToGps = this->GpsPoses.back();
      Eigen::Isometry3d mapToOdom;
      Utils::Tf2LookupTransform(mapToOdom, this->TfBuffer, mapToGps.frameid, this->OdometryFrameId, ros::Time(mapToGps.time));
      Eigen::Isometry3d odomToBase = mapToOdom.inverse() * mapToGps.GetIsometry() * this->BaseToGpsOffset.inverse();
      this->LidarSlam.SetWorldTransformFromGuess(LidarSlam::Transform(odomToBase, mapToGps.time, mapToGps.frameid));
      ROS_WARN_STREAM("SLAM pose set from GPS pose to :\n" << odomToBase.matrix());
      break;
    }

    // Enable SLAM maps update
    case lidar_slam::SlamCommand::ENABLE_SLAM_MAP_UPDATE:
      this->LidarSlam.SetUpdateMap(true);
      ROS_WARN_STREAM("Enabling SLAM maps update.");
      break;

    // Disable SLAM maps update
    case lidar_slam::SlamCommand::DISABLE_SLAM_MAP_UPDATE:
      this->LidarSlam.SetUpdateMap(false);
      ROS_WARN_STREAM("Disabling SLAM maps update.");
      break;

    // Save SLAM keypoints maps to PCD files
    case lidar_slam::SlamCommand::SAVE_KEYPOINTS_MAPS:
    {
      ROS_INFO_STREAM("Saving keypoints maps to PCD.");
      int pcdFormatInt = this->PrivNh.param("maps/export_pcd_format", static_cast<int>(LidarSlam::PCDFormat::BINARY_COMPRESSED));
      LidarSlam::PCDFormat pcdFormat = static_cast<LidarSlam::PCDFormat>(pcdFormatInt);
      if (pcdFormat != LidarSlam::PCDFormat::ASCII &&
          pcdFormat != LidarSlam::PCDFormat::BINARY &&
          pcdFormat != LidarSlam::PCDFormat::BINARY_COMPRESSED)
      {
        ROS_ERROR_STREAM("Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
        pcdFormat = LidarSlam::PCDFormat::BINARY_COMPRESSED;
      }
      this->LidarSlam.SaveMapsToPCD(msg.string_arg, pcdFormat);
      break;
    }

    // Load SLAM keypoints maps from PCD files
    case lidar_slam::SlamCommand::LOAD_KEYPOINTS_MAPS:
      ROS_INFO_STREAM("Loading keypoints maps from PCD.");
      this->LidarSlam.LoadMapsFromPCD(msg.string_arg);
      break;

    // Unkown command
    default:
      ROS_ERROR_STREAM("Unknown SLAM command : " << (unsigned int) msg.command);
      break;
  }
}

//==============================================================================
//   Special SLAM commands
//==============================================================================

//------------------------------------------------------------------------------
void LidarSlamNode::GpsSlamCalibration()
{
  if (!this->UseGps)
  {
    ROS_ERROR_STREAM("Cannot run GPS/SLAM calibration as GPS logging has not been enabled. "
                     "Please set 'gps/use_gps' private parameter to 'true'.");
    return;
  }

  // Transform to modifiable vectors
  std::vector<LidarSlam::Transform> odomToBasePoses = this->LidarSlam.GetTrajectory();
  std::vector<LidarSlam::Transform> worldToGpsPoses(this->GpsPoses.begin(), this->GpsPoses.end());

  if (worldToGpsPoses.size() < 2 && odomToBasePoses.size() < 2)
  {
    ROS_ERROR_STREAM("Not enough points to run SLAM/GPS calibration "
                      "(only got " << odomToBasePoses.size() << " slam points "
                      "and " << worldToGpsPoses.size() << " gps points).");
    return;
  }
  // If we have enough GPS and SLAM points, run calibration
  ROS_INFO_STREAM("Running SLAM/GPS calibration with " << odomToBasePoses.size() << " slam points and "
                                                       << worldToGpsPoses.size() << " gps points.");

  // If a sensors offset is given, use it to compute real GPS antenna position in SLAM origin coordinates
  if (!this->BaseToGpsOffset.isApprox(Eigen::Isometry3d::Identity()))
  {
    if (this->LidarSlam.GetVerbosity() >= 2)
    {
      std::cout << "Transforming LiDAR pose acquired by SLAM to GPS antenna pose using LIDAR to GPS antenna offset :\n"
                << this->BaseToGpsOffset.matrix() << std::endl;
    }
    for (LidarSlam::Transform& odomToGpsPose : odomToBasePoses)
    {
      Eigen::Isometry3d odomToBase = odomToGpsPose.GetIsometry();
      odomToGpsPose.SetIsometry(odomToBase * this->BaseToGpsOffset);
    }
  }
  // At this point, we now have GPS antenna poses in SLAM coordinates.
  const std::vector<LidarSlam::Transform>& odomToGpsPoses = odomToBasePoses;

  // Run calibration : compute transform from SLAM to WORLD
  LidarSlam::GlobalTrajectoriesRegistration registration;
  registration.SetNoRoll(this->PrivNh.param("gps/calibration/no_roll", false));  // DEBUG
  registration.SetVerbose(this->LidarSlam.GetVerbosity() >= 2);
  Eigen::Isometry3d worldToOdom;
  if (!registration.ComputeTransformOffset(odomToGpsPoses, worldToGpsPoses, worldToOdom))
  {
    ROS_ERROR_STREAM("GPS/SLAM calibration failed.");
    return;
  }
  const std::string& gpsFrameId = worldToGpsPoses.back().frameid;
  ros::Time latestTime = ros::Time(std::max(worldToGpsPoses.back().time, odomToGpsPoses.back().time));

  // Publish ICP-matched trajectories
  // GPS antenna trajectory acquired from GPS in WORLD coordinates
  if (this->Publish[ICP_CALIB_GPS_PATH])
  {
    nav_msgs::Path gpsPath;
    gpsPath.header.frame_id = gpsFrameId;
    gpsPath.header.stamp = latestTime;
    for (const LidarSlam::Transform& pose: worldToGpsPoses)
    {
      gpsPath.poses.emplace_back(Utils::TransformToPoseStampedMsg(pose));
    }
    this->Publishers[ICP_CALIB_GPS_PATH].publish(gpsPath);
  }
  // GPS antenna trajectory acquired from SLAM in WORLD coordinates
  if (this->Publish[ICP_CALIB_SLAM_PATH])
  {
    nav_msgs::Path slamPath;
    slamPath.header.frame_id = gpsFrameId;
    slamPath.header.stamp = latestTime;
    for (const LidarSlam::Transform& pose: odomToGpsPoses)
    {
      LidarSlam::Transform worldToGpsPose(worldToOdom * pose.GetIsometry(), pose.time, pose.frameid);
      slamPath.poses.emplace_back(Utils::TransformToPoseStampedMsg(worldToGpsPose));
    }
    this->Publishers[ICP_CALIB_SLAM_PATH].publish(slamPath);
  }

  // Publish static tf with calibration to link world (UTM) frame to SLAM odometry origin
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = latestTime;
  tfStamped.header.frame_id = gpsFrameId;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = Utils::TransformToTfMsg(LidarSlam::Transform(worldToOdom));
  this->StaticTfBroadcaster.sendTransform(tfStamped);

  Eigen::Vector3d xyz = worldToOdom.translation();
  Eigen::Vector3d ypr = LidarSlam::Utils::RotationMatrixToRPY(worldToOdom.linear()).reverse();
  ROS_INFO_STREAM(BOLD_GREEN("Global transform from '" << gpsFrameId << "' to '" << this->OdometryFrameId << "' " <<
                  "successfully estimated to :\n" << worldToOdom.matrix() << "\n" <<
                  "(tf2 static transform : " << xyz.transpose() << " " << ypr.transpose() << " " << gpsFrameId << " " << this->OdometryFrameId << ")"));
}

//------------------------------------------------------------------------------
void LidarSlamNode::PoseGraphOptimization()
{
  if (!this->UseGps)
  {
    ROS_ERROR_STREAM("Cannot run pose graph optimization as GPS logging has not been enabled. "
                     "Please set 'gps/use_gps' private parameter to 'true'.");
    return;
  }

  // Transform to modifiable vectors
  std::vector<LidarSlam::Transform> worldToGpsPositions(this->GpsPoses.begin(), this->GpsPoses.end());
  std::vector<std::array<double, 9>> worldToGpsCovars(this->GpsCovars.begin(), this->GpsCovars.end());

  // Run pose graph optimization
  Eigen::Isometry3d gpsToBaseOffset = this->BaseToGpsOffset.inverse();
  std::string pgoG2oFile = this->PrivNh.param("gps/pgo/g2o_file", std::string(""));
  this->LidarSlam.RunPoseGraphOptimization(worldToGpsPositions, worldToGpsCovars,
                                           gpsToBaseOffset, pgoG2oFile);

  // Update GPS/LiDAR calibration
  this->BaseToGpsOffset = gpsToBaseOffset.inverse();

  // Publish static tf with calibration to link world (UTM) frame to SLAM origin
  LidarSlam::Transform odomToBase = this->LidarSlam.GetWorldTransform();
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = ros::Time(odomToBase.time);
  tfStamped.header.frame_id = worldToGpsPositions.back().frameid;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = Utils::TransformToTfMsg(LidarSlam::Transform(gpsToBaseOffset));
  this->StaticTfBroadcaster.sendTransform(tfStamped);

  // Publish optimized SLAM trajectory
  if (this->Publish[PGO_PATH])
  {
    nav_msgs::Path optimSlamTraj;
    optimSlamTraj.header.frame_id = this->OdometryFrameId;
    optimSlamTraj.header.stamp = ros::Time(odomToBase.time);
    std::vector<LidarSlam::Transform> optimizedSlamPoses = this->LidarSlam.GetTrajectory();
    for (const LidarSlam::Transform& pose: optimizedSlamPoses)
      optimSlamTraj.poses.emplace_back(Utils::TransformToPoseStampedMsg(pose));
    this->Publishers[PGO_PATH].publish(optimSlamTraj);
  }

  // Update display
  this->PublishOutput();
}

//==============================================================================
//   Utilities
//==============================================================================

//------------------------------------------------------------------------------
void LidarSlamNode::UpdateBaseToLidarOffset(const std::string& lidarFrameId, uint8_t lidarDeviceId)
{
  // If tracking frame is different from input frame, get TF from LiDAR to BASE
  if (lidarFrameId != this->TrackingFrameId)
  {
    // We expect a static transform between BASE and LIDAR, so we don't care
    // about timestamp and get only the latest transform
    Eigen::Isometry3d baseToLidar;
    if (Utils::Tf2LookupTransform(baseToLidar, this->TfBuffer, this->TrackingFrameId, lidarFrameId))
      this->LidarSlam.SetBaseToLidarOffset(baseToLidar, lidarDeviceId);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishOutput()
{
  // Publish SLAM pose
  if (this->Publish[POSE_ODOM] || this->Publish[POSE_TF])
  {
    // Get SLAM pose
    LidarSlam::Transform odomToBase = this->LidarSlam.GetWorldTransform();

    // Publish as odometry msg
    if (this->Publish[POSE_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(odomToBase.time);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId;
      odomMsg.pose.pose = Utils::TransformToPoseMsg(odomToBase);
      auto covar = this->LidarSlam.GetTransformCovariance();
      std::copy(covar.begin(), covar.end(), odomMsg.pose.covariance.begin());
      this->Publishers[POSE_ODOM].publish(odomMsg);
    }

    // Publish as TF from OdometryFrameId to TrackingFrameId
    if (this->Publish[POSE_TF])
    {
      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.stamp = ros::Time(odomToBase.time);
      tfMsg.header.frame_id = this->OdometryFrameId;
      tfMsg.child_frame_id = this->TrackingFrameId;
      tfMsg.transform = Utils::TransformToTfMsg(odomToBase);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }

  // Publish latency compensated SLAM pose
  if (this->Publish[POSE_PREDICTION_ODOM] || this->Publish[POSE_PREDICTION_TF])
  {
    // Get latency corrected SLAM pose
    LidarSlam::Transform odomToBasePred = this->LidarSlam.GetLatencyCompensatedWorldTransform();

    // Publish as odometry msg
    if (this->Publish[POSE_PREDICTION_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(odomToBasePred.time);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      odomMsg.pose.pose = Utils::TransformToPoseMsg(odomToBasePred);
      auto covar = this->LidarSlam.GetTransformCovariance();
      std::copy(covar.begin(), covar.end(), odomMsg.pose.covariance.begin());
      this->Publishers[POSE_PREDICTION_ODOM].publish(odomMsg);
    }

    // Publish as TF from OdometryFrameId to <TrackingFrameId>_prediction
    if (this->Publish[POSE_PREDICTION_TF])
    {
      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.stamp = ros::Time(odomToBasePred.time);
      tfMsg.header.frame_id = this->OdometryFrameId;
      tfMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      tfMsg.transform = Utils::TransformToTfMsg(odomToBasePred);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }

  // Publish a pointcloud only if required and if someone is listening to it to spare bandwidth.
  #define publishPointCloud(publisher, pc)                                            \
    if (this->Publish[publisher] && this->Publishers[publisher].getNumSubscribers())  \
      this->Publishers[publisher].publish(pc);

  // Keypoints maps
  publishPointCloud(EDGES_MAP,  this->LidarSlam.GetMap(LidarSlam::EDGE));
  publishPointCloud(PLANES_MAP, this->LidarSlam.GetMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_MAP,  this->LidarSlam.GetMap(LidarSlam::BLOB));

  // Current keypoints
  publishPointCloud(EDGE_KEYPOINTS,  this->LidarSlam.GetKeypoints(LidarSlam::EDGE));
  publishPointCloud(PLANE_KEYPOINTS, this->LidarSlam.GetKeypoints(LidarSlam::PLANE));
  publishPointCloud(BLOB_KEYPOINTS,  this->LidarSlam.GetKeypoints(LidarSlam::BLOB));

  // debug cloud
  publishPointCloud(SLAM_REGISTERED_POINTS, this->LidarSlam.GetOutputFrame());
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters()
{
  #define SetSlamParam(type, rosParam, slamParam) { type val; if (this->PrivNh.getParam(rosParam, val)) this->LidarSlam.Set##slamParam(val); }

  // General
  SetSlamParam(bool,   "slam/2d_mode", TwoDMode)
  SetSlamParam(bool,   "slam/use_blobs", UseBlobs)
  SetSlamParam(int,    "slam/verbosity", Verbosity)
  SetSlamParam(int,    "slam/n_threads", NbThreads)
  SetSlamParam(double, "slam/logging_timeout", LoggingTimeout)
  SetSlamParam(double, "slam/max_distance_for_ICP_matching", MaxDistanceForICPMatching)
  int egoMotionMode;
  if (this->PrivNh.getParam("slam/ego_motion", egoMotionMode))
  {
    LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(egoMotionMode);
    if (egoMotion != LidarSlam::EgoMotionMode::NONE &&
        egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION &&
        egoMotion != LidarSlam::EgoMotionMode::REGISTRATION &&
        egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION)
    {
      ROS_ERROR_STREAM("Invalid ego-motion mode (" << egoMotionMode << "). Setting it to 'MOTION_EXTRAPOLATION'.");
      egoMotion = LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION;
    }
    LidarSlam.SetEgoMotion(egoMotion);
  }
  int undistortionMode;
  if (this->PrivNh.getParam("slam/undistortion", undistortionMode))
  {
    LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(undistortionMode);
    if (undistortion != LidarSlam::UndistortionMode::NONE &&
        undistortion != LidarSlam::UndistortionMode::ONCE &&
        undistortion != LidarSlam::UndistortionMode::REFINED)
    {
      ROS_ERROR_STREAM("Invalid undistortion mode (" << undistortion << "). Setting it to 'REFINED'.");
      undistortion = LidarSlam::UndistortionMode::REFINED;
    }
    LidarSlam.SetUndistortion(undistortion);
  }
  int pointCloudStorage;
  if (this->PrivNh.getParam("slam/logging_storage", pointCloudStorage))
  {
    LidarSlam::PointCloudStorageType storage = static_cast<LidarSlam::PointCloudStorageType>(pointCloudStorage);
    if (storage != LidarSlam::PointCloudStorageType::PCL_CLOUD &&
        storage != LidarSlam::PointCloudStorageType::OCTREE_COMPRESSED &&
        storage != LidarSlam::PointCloudStorageType::PCD_ASCII &&
        storage != LidarSlam::PointCloudStorageType::PCD_BINARY &&
        storage != LidarSlam::PointCloudStorageType::PCD_BINARY_COMPRESSED)
    {
      ROS_ERROR_STREAM("Incorrect pointcloud logging type value (" << storage << "). Setting it to 'PCL'.");
      storage = LidarSlam::PointCloudStorageType::PCL_CLOUD;
    }
    LidarSlam.SetLoggingStorage(storage);
  }

  // Frame Ids
  this->PrivNh.param("odometry_frame", this->OdometryFrameId, this->OdometryFrameId);
  this->LidarSlam.SetWorldFrameId(this->OdometryFrameId);
  this->PrivNh.param("tracking_frame", this->TrackingFrameId, this->TrackingFrameId);
  this->LidarSlam.SetBaseFrameId(this->TrackingFrameId);

  // Ego motion
  SetSlamParam(int,    "slam/ego_motion_registration/LM_max_iter", EgoMotionLMMaxIter)
  SetSlamParam(int,    "slam/ego_motion_registration/ICP_max_iter", EgoMotionICPMaxIter)
  SetSlamParam(int,    "slam/ego_motion_registration/line_distance_nbr_neighbors", EgoMotionLineDistanceNbrNeighbors)
  SetSlamParam(int,    "slam/ego_motion_registration/minimum_line_neighbor_rejection", EgoMotionMinimumLineNeighborRejection)
  SetSlamParam(int,    "slam/ego_motion_registration/plane_distance_nbr_neighbors", EgoMotionPlaneDistanceNbrNeighbors)
  SetSlamParam(double, "slam/ego_motion_registration/line_distance_factor", EgoMotionLineDistancefactor)
  SetSlamParam(double, "slam/ego_motion_registration/plane_distance_factor1", EgoMotionPlaneDistancefactor1)
  SetSlamParam(double, "slam/ego_motion_registration/plane_distance_factor2", EgoMotionPlaneDistancefactor2)
  SetSlamParam(double, "slam/ego_motion_registration/max_line_distance", EgoMotionMaxLineDistance)
  SetSlamParam(double, "slam/ego_motion_registration/max_plane_distance", EgoMotionMaxPlaneDistance)
  SetSlamParam(double, "slam/ego_motion_registration/init_saturation_distance", EgoMotionInitSaturationDistance)
  SetSlamParam(double, "slam/ego_motion_registration/final_saturation_distance", EgoMotionFinalSaturationDistance)

  // Localization
  SetSlamParam(int,    "slam/localization/LM_max_iter", LocalizationLMMaxIter)
  SetSlamParam(int,    "slam/localization/ICP_max_iter", LocalizationICPMaxIter)
  SetSlamParam(int,    "slam/localization/line_distance_nbr_neighbors", LocalizationLineDistanceNbrNeighbors)
  SetSlamParam(int,    "slam/localization/minimum_line_neighbor_rejection", LocalizationMinimumLineNeighborRejection)
  SetSlamParam(int,    "slam/localization/plane_distance_nbr_neighbors", LocalizationPlaneDistanceNbrNeighbors)
  SetSlamParam(double, "slam/localization/line_distance_factor", LocalizationLineDistancefactor)
  SetSlamParam(double, "slam/localization/plane_distance_factor1", LocalizationPlaneDistancefactor1)
  SetSlamParam(double, "slam/localization/plane_distance_factor2", LocalizationPlaneDistancefactor2)
  SetSlamParam(double, "slam/localization/max_line_distance", LocalizationMaxLineDistance)
  SetSlamParam(double, "slam/localization/max_plane_distance", LocalizationMaxPlaneDistance)
  SetSlamParam(double, "slam/localization/init_saturation_distance", LocalizationInitSaturationDistance)
  SetSlamParam(double, "slam/localization/final_saturation_distance", LocalizationFinalSaturationDistance)

  // Keyframes
  SetSlamParam(double, "slam/keyframes/distance_threshold", KfDistanceThreshold)
  SetSlamParam(double, "slam/keyframes/angle_threshold", KfAngleThreshold)

  // Rolling grids
  double size;
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size_edges", size))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::EDGE, size);
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size_planes", size))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::PLANE, size);
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size_blobs", size))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::BLOB, size);
  SetSlamParam(double, "slam/voxel_grid/resolution", VoxelGridResolution)
  SetSlamParam(int,    "slam/voxel_grid/size", VoxelGridSize)

  // Keypoint extractors
  auto InitKeypointsExtractor = [this](auto& ke, const std::string& prefix)
  {
    #define SetKeypointsExtractorParam(type, rosParam, keParam) {type val; if (this->PrivNh.getParam(rosParam, val)) ke->Set##keParam(val);}
    SetKeypointsExtractorParam(int,   "slam/n_threads", NbThreads)
    SetKeypointsExtractorParam(int,   prefix + "neighbor_width", NeighborWidth)
    SetKeypointsExtractorParam(float, prefix + "min_distance_to_sensor", MinDistanceToSensor)
    SetKeypointsExtractorParam(float, prefix + "min_beam_surface_angle", MinBeamSurfaceAngle)
    SetKeypointsExtractorParam(float, prefix + "plane_sin_angle_threshold", PlaneSinAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_sin_angle_threshold", EdgeSinAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_depth_gap_threshold", EdgeDepthGapThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_saliency_threshold", EdgeSaliencyThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_intensity_gap_threshold", EdgeIntensityGapThreshold)
  };
  // Multi-LiDAR devices
  std::vector<int> deviceIds;
  if (this->PrivNh.getParam("slam/ke/device_ids", deviceIds))
  {
    ROS_INFO_STREAM("Multi-LiDAR devices setup");
    for (auto deviceId: deviceIds)
    {
      // Init Keypoint extractor with default params
      auto ke = std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>();

      // Change default parameters using ROS parameter server
      std::string prefix = "slam/ke/device_" + std::to_string(deviceId) + "/";
      InitKeypointsExtractor(ke, prefix);

      // Add extractor to SLAM
      this->LidarSlam.SetKeyPointsExtractor(ke, deviceId);
      ROS_INFO_STREAM("Adding keypoint extractor for LiDAR device " << deviceId);
    }
  }
  // Single LiDAR device
  else
  {
    ROS_INFO_STREAM("Single LiDAR device setup");
    auto ke = std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>();
    InitKeypointsExtractor(ke, "slam/ke/");
    this->LidarSlam.SetKeyPointsExtractor(ke);
  }
}