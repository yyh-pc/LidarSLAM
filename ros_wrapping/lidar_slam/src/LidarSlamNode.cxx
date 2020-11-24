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

  EDGES_MAP,             // Publish edges keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/edges'.
  PLANES_MAP,            // Publish planes keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/planes'.
  BLOBS_MAP,             // Publish blobs keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/blobs'.

  EDGES_KEYPOINTS,       // Publish extracted edges keypoints from current frame as a PointCloud2 msg to topic 'keypoints/edges'.
  PLANES_KEYPOINTS,      // Publish extracted planes keypoints from current frame as a PointCloud2 msg to topic 'keypoints/planes'.
  BLOBS_KEYPOINTS,       // Publish extracted blobs keypoints from current frame as a PointCloud2 msg to topic 'keypoints/blobs'.

  SLAM_CLOUD,            // Publish SLAM pointcloud as LidarPoint PointCloud2 msg to topic 'slam_cloud'.

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
  // Get SLAM params
  this->SetSlamParameters(priv_nh);

  // Init laserIdMapping
  std::vector<int> intLaserIdMapping;
  int nLasers;
  // Try to get it directly from ROS param
  if (priv_nh.getParam("laser_id_mapping", intLaserIdMapping))
  {
    this->LaserIdMapping.assign(intLaserIdMapping.begin(), intLaserIdMapping.end());
    ROS_INFO_STREAM("[SLAM] Using laser_id_mapping from ROS param.");
  }
  // Or only try to get number of lasers to build linear mapping
  else if (priv_nh.getParam("n_lasers", nLasers))
  {
    this->LaserIdMapping.resize(nLasers);
    std::iota(this->LaserIdMapping.begin(), this->LaserIdMapping.end(), 0);
    ROS_INFO_STREAM("[SLAM] Using 0->" << nLasers - 1 << " linear laser_id_mapping from ROS param.");
  }
  // Otherwise, n_lasers will be guessed from 1st frame
  else
  {
    ROS_WARN_STREAM("[SLAM] No laser_id_mapping nor n_lasers params found : "
                    "n_lasers will be guessed from 1st frame to build linear mapping.");
  }

  // Get LiDAR frequency
  priv_nh.getParam("lidar_frequency", this->LidarFreq);

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

  initPublisher(EDGES_MAP,  "maps/edges",  CloudS, "output/maps/edges",  false, 1, false);
  initPublisher(PLANES_MAP, "maps/planes", CloudS, "output/maps/planes", false, 1, false);
  initPublisher(BLOBS_MAP,  "maps/blobs",  CloudS, "output/maps/blobs",  false, 1, false);

  initPublisher(EDGES_KEYPOINTS,  "keypoints/edges",  CloudS, "output/keypoints/edges",  false, 1, false);
  initPublisher(PLANES_KEYPOINTS, "keypoints/planes", CloudS, "output/keypoints/planes", false, 1, false);
  initPublisher(BLOBS_KEYPOINTS,  "keypoints/blobs",  CloudS, "output/keypoints/blobs",  false, 1, false);

  initPublisher(SLAM_CLOUD, "slam_cloud", CloudS, "output/debug/cloud", false, 1, false);

  if (this->UseGps)
  {
    initPublisher(PGO_PATH,            "pgo_slam_path", nav_msgs::Path, "gps/pgo/publish_path",              false, 1, true);
    initPublisher(ICP_CALIB_SLAM_PATH, "icp_slam_path", nav_msgs::Path, "gps/calibration/publish_icp_paths", false, 1, true);
    initPublisher(ICP_CALIB_GPS_PATH,  "icp_gps_path",  nav_msgs::Path, "gps/calibration/publish_icp_paths", false, 1, true);
  }

  // ***************************************************************************
  // Init ROS subscribers
  this->CloudSub = nh.subscribe("velodyne_points", 1, &LidarSlamNode::ScanCallback, this);
  this->SlamCommandSub = nh.subscribe("slam_command", 1,  &LidarSlamNode::SlamCommandCallback, this);

  // Init logging of GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  if (this->UseGps)
    this->GpsOdomSub = nh.subscribe("gps_odom", 1, &LidarSlamNode::GpsCallback, this);

  ROS_INFO_STREAM(BOLD_GREEN("LiDAR SLAM is ready !"));
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallback(const CloudV& cloudV)
{
  // Init LaserIdMapping if not already done
  if (this->LaserIdMapping.empty())
  {
    // Iterate through pointcloud to find max ring
    unsigned int maxRing = 0;
    for (const PointV& point : cloudV)
    {
      if (point.ring > maxRing)
        maxRing = point.ring;
    }

    // Init LaserIdMapping with linear mapping
    this->LaserIdMapping.resize(maxRing + 1);
    std::iota(this->LaserIdMapping.begin(), this->LaserIdMapping.end(), 0);
    ROS_INFO_STREAM("[SLAM] Using 0->" << maxRing << " linear laser_id_mapping.");
  }

  // Convert pointcloud PointV type to expected PointS type
  CloudS::Ptr cloudS = this->ConvertToSlamPointCloud(cloudV);

  // If no tracking frame is set, track input pointcloud origin
  if (this->TrackingFrameId.empty())
    this->TrackingFrameId = cloudS->header.frame_id;
  // Update TF from BASE to LiDAR
  this->UpdateBaseToLidarOffset(cloudS->header.frame_id, cloudS->header.stamp);

  // Run SLAM : register new frame and update localization and map.
  this->LidarSlam.AddFrame(cloudS, this->LaserIdMapping);

  // Publish SLAM output as requested by user
  this->PublishOutput();
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
    this->GpsPoses.emplace_back(PoseMsgToTransform(msg.pose.pose, msg.header.stamp.toSec(), msg.header.frame_id));
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
    Tf2LookupTransform(this->BaseToGpsOffset, this->TfBuffer, this->TrackingFrameId, msg.child_frame_id);
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
      Transform& mapToGps = this->GpsPoses.back();
      Eigen::Isometry3d mapToOdom;
      Tf2LookupTransform(mapToOdom, this->TfBuffer, mapToGps.frameid, this->OdometryFrameId, ros::Time(mapToGps.time));
      Eigen::Isometry3d odomToBase = mapToOdom.inverse() * mapToGps.GetIsometry() * this->BaseToGpsOffset.inverse();
      this->LidarSlam.SetWorldTransformFromGuess(Transform(odomToBase, mapToGps.time, mapToGps.frameid));
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
      PCDFormat pcdFormat = static_cast<PCDFormat>(this->PrivNh.param("maps_saving/pcd_format", static_cast<int>(PCDFormat::BINARY_COMPRESSED)));
      if (pcdFormat != PCDFormat::ASCII && pcdFormat != PCDFormat::BINARY && pcdFormat != PCDFormat::BINARY_COMPRESSED)
      {
        ROS_ERROR_STREAM("Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
        pcdFormat = PCDFormat::BINARY_COMPRESSED;
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
  std::vector<Transform> odomToBasePoses = this->LidarSlam.GetTrajectory();
  std::vector<Transform> worldToGpsPoses(this->GpsPoses.begin(), this->GpsPoses.end());

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
    for (Transform& odomToGpsPose : odomToBasePoses)
    {
      Eigen::Isometry3d odomToBase = odomToGpsPose.GetIsometry();
      odomToGpsPose.SetIsometry(odomToBase * this->BaseToGpsOffset);
    }
  }
  // At this point, we now have GPS antenna poses in SLAM coordinates.
  const std::vector<Transform>& odomToGpsPoses = odomToBasePoses;

  // Run calibration : compute transform from SLAM to WORLD
  GlobalTrajectoriesRegistration registration;
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
    for (const Transform& pose: worldToGpsPoses)
    {
      gpsPath.poses.emplace_back(TransformToPoseStampedMsg(pose));
    }
    this->Publishers[ICP_CALIB_GPS_PATH].publish(gpsPath);
  }
  // GPS antenna trajectory acquired from SLAM in WORLD coordinates
  if (this->Publish[ICP_CALIB_SLAM_PATH])
  {
    nav_msgs::Path slamPath;
    slamPath.header.frame_id = gpsFrameId;
    slamPath.header.stamp = latestTime;
    for (const Transform& pose: odomToGpsPoses)
    {
      Transform worldToGpsPose(worldToOdom * pose.GetIsometry(), pose.time, pose.frameid);
      slamPath.poses.emplace_back(TransformToPoseStampedMsg(worldToGpsPose));
    }
    this->Publishers[ICP_CALIB_SLAM_PATH].publish(slamPath);
  }

  // Publish static tf with calibration to link world (UTM) frame to SLAM odometry origin
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = latestTime;
  tfStamped.header.frame_id = gpsFrameId;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = TransformToTfMsg(Transform(worldToOdom));
  this->StaticTfBroadcaster.sendTransform(tfStamped);

  Eigen::Vector3d xyz = worldToOdom.translation();
  Eigen::Vector3d ypr = RotationMatrixToRPY(worldToOdom.linear());
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
  std::vector<Transform> worldToGpsPositions(this->GpsPoses.begin(), this->GpsPoses.end());
  std::vector<std::array<double, 9>> worldToGpsCovars(this->GpsCovars.begin(), this->GpsCovars.end());

  // Run pose graph optimization
  Eigen::Isometry3d gpsToBaseOffset = this->BaseToGpsOffset.inverse();
  std::string pgoG2oFile = this->PrivNh.param("gps/pgo/g2o_file", std::string(""));
  this->LidarSlam.RunPoseGraphOptimization(worldToGpsPositions, worldToGpsCovars,
                                           gpsToBaseOffset, pgoG2oFile);

  // Update GPS/LiDAR calibration
  this->BaseToGpsOffset = gpsToBaseOffset.inverse();

  // Publish static tf with calibration to link world (UTM) frame to SLAM origin
  Transform odomToBase = this->LidarSlam.GetWorldTransform();
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = ros::Time(odomToBase.time);
  tfStamped.header.frame_id = worldToGpsPositions.back().frameid;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = TransformToTfMsg(Transform(gpsToBaseOffset));
  this->StaticTfBroadcaster.sendTransform(tfStamped);

  // Publish optimized SLAM trajectory
  if (this->Publish[PGO_PATH])
  {
    nav_msgs::Path optimSlamTraj;
    optimSlamTraj.header.frame_id = this->OdometryFrameId;
    optimSlamTraj.header.stamp = ros::Time(odomToBase.time);
    std::vector<Transform> optimizedSlamPoses = this->LidarSlam.GetTrajectory();
    for (const Transform& pose: optimizedSlamPoses)
      optimSlamTraj.poses.emplace_back(TransformToPoseStampedMsg(pose));
    this->Publishers[PGO_PATH].publish(optimSlamTraj);
  }

  // Update display
  this->PublishOutput();
}

//==============================================================================
//   Utilities
//==============================================================================

//------------------------------------------------------------------------------
LidarSlamNode::CloudS::Ptr LidarSlamNode::ConvertToSlamPointCloud(const CloudV& cloudV) const
{
  // Init SLAM pointcloud
  CloudS::Ptr cloudS(new CloudS);
  cloudS->resize(cloudV.size());
  cloudS->header = cloudV.header;

  // Helpers to estimate frameAdvancement
  auto wrapMax = [](double x, double max) {return std::fmod(max + std::fmod(x, max), max);};
  auto advancement = [](const PointV& velodynePoint) {return (M_PI - std::atan2(velodynePoint.y, velodynePoint.x)) / (2 * M_PI);};
  const double initAdvancement = advancement(cloudV.front());
  std::vector<double> previousAdvancementPerRing(this->LaserIdMapping.size(), -1);

  // Build SLAM pointcloud
  for(unsigned int i = 0; i < cloudV.size(); i++)
  {
    const PointV& velodynePoint = cloudV[i];
    PointS& slamPoint = cloudS->at(i);

    // Get normalized angle (in [0-1]), with angle 0 being first point direction
    double frameAdvancement = advancement(velodynePoint);
    frameAdvancement = wrapMax(frameAdvancement - initAdvancement, 1.);
    // If we detect overflow, correct it
    if (frameAdvancement < previousAdvancementPerRing[velodynePoint.ring])
      frameAdvancement += 1;
    previousAdvancementPerRing[velodynePoint.ring] = frameAdvancement;

    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laser_id = velodynePoint.ring;
    slamPoint.time = frameAdvancement / this->LidarFreq; // time is 0 for first point, and should match LiDAR period for last point for a complete scan.
    slamPoint.device_id = 0;
  }
  return cloudS;
}

//------------------------------------------------------------------------------
void LidarSlamNode::UpdateBaseToLidarOffset(const std::string& lidarFrameId, uint64_t pclStamp)
{
  // If tracking frame is different from input frame, get TF from LiDAR to BASE
  if (lidarFrameId != this->TrackingFrameId)
  {
    // We expect a static transform between BASE and LIDAR, so we don't care
    // about timestamp and get only the latest transform
    Eigen::Isometry3d baseToLidar;
    if (Tf2LookupTransform(baseToLidar, this->TfBuffer, this->TrackingFrameId, lidarFrameId))
      this->LidarSlam.SetBaseToLidarOffset(baseToLidar);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishOutput()
{
  // Publish SLAM pose
  if (this->Publish[POSE_ODOM] || this->Publish[POSE_TF])
  {
    // Get SLAM pose
    Transform odomToBase = this->LidarSlam.GetWorldTransform();

    // Publish as odometry msg
    if (this->Publish[POSE_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(odomToBase.time);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId;
      odomMsg.pose.pose = TransformToPoseMsg(odomToBase);
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
      tfMsg.transform = TransformToTfMsg(odomToBase);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }

  // Publish latency compensated SLAM pose
  if (this->Publish[POSE_PREDICTION_ODOM] || this->Publish[POSE_PREDICTION_TF])
  {
    // Get latency corrected SLAM pose
    Transform odomToBasePred = this->LidarSlam.GetLatencyCompensatedWorldTransform();

    // Publish as odometry msg
    if (this->Publish[POSE_PREDICTION_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(odomToBasePred.time);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      odomMsg.pose.pose = TransformToPoseMsg(odomToBasePred);
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
      tfMsg.transform = TransformToTfMsg(odomToBasePred);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }

  // Publish a pointcloud only if required and if someone is listening to it to spare bandwidth.
  #define publishPointCloud(publisher, pc)                                            \
    if (this->Publish[publisher] && this->Publishers[publisher].getNumSubscribers())  \
      this->Publishers[publisher].publish(pc);

  // Keypoints maps
  publishPointCloud(EDGES_MAP,  this->LidarSlam.GetEdgesMap());
  publishPointCloud(PLANES_MAP, this->LidarSlam.GetPlanarsMap());
  publishPointCloud(BLOBS_MAP,  this->LidarSlam.GetBlobsMap());

  // Current keypoints
  publishPointCloud(EDGES_KEYPOINTS,  this->LidarSlam.GetEdgesKeypoints());
  publishPointCloud(PLANES_KEYPOINTS, this->LidarSlam.GetPlanarsKeypoints());
  publishPointCloud(BLOBS_KEYPOINTS,  this->LidarSlam.GetBlobsKeypoints());

  // debug cloud
  publishPointCloud(SLAM_CLOUD, this->LidarSlam.GetOutputFrame());
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters(ros::NodeHandle& priv_nh)
{
  #define SetSlamParam(type, rosParam, slamParam) { type val; if (priv_nh.getParam(rosParam, val)) this->LidarSlam.Set##slamParam(val); }

  // General
  SetSlamParam(bool,   "slam/fast_slam", FastSlam)
  SetSlamParam(int,    "slam/verbosity", Verbosity)
  SetSlamParam(int,    "slam/n_threads", NbThreads)
  SetSlamParam(double, "slam/logging_timeout", LoggingTimeout)
  SetSlamParam(double, "slam/max_distance_for_ICP_matching", MaxDistanceForICPMatching)
  int egoMotionMode;
  if (priv_nh.getParam("slam/ego_motion", egoMotionMode))
  {
    EgoMotionMode egoMotion = static_cast<EgoMotionMode>(egoMotionMode);
    if (egoMotion != EgoMotionMode::NONE         && egoMotion != EgoMotionMode::MOTION_EXTRAPOLATION &&
        egoMotion != EgoMotionMode::REGISTRATION && egoMotion != EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION)
    {
      ROS_ERROR_STREAM("Invalid ego-motion mode (" << egoMotionMode << "). Setting it to 'MOTION_EXTRAPOLATION'.");
      egoMotion = EgoMotionMode::MOTION_EXTRAPOLATION;
    }
    LidarSlam.SetEgoMotion(egoMotion);
  }
  int undistortionMode;
  if (priv_nh.getParam("slam/undistortion", undistortionMode))
  {
    UndistortionMode undistortion = static_cast<UndistortionMode>(undistortionMode);
    if (undistortion != UndistortionMode::NONE && undistortion != UndistortionMode::APPROXIMATED && undistortion != UndistortionMode::OPTIMIZED)
    {
      ROS_ERROR_STREAM("Invalid undistortion mode (" << undistortion << "). Setting it to 'APPROXIMATED'.");
      undistortion = UndistortionMode::APPROXIMATED;
    }
    LidarSlam.SetUndistortion(undistortion);
  }
  int pointCloudStorage;
  if (priv_nh.getParam("slam/logging_storage", pointCloudStorage))
  {
    PointCloudStorageType storage = static_cast<PointCloudStorageType>(pointCloudStorage);
    if (storage != PCL_CLOUD && storage != OCTREE_COMPRESSED &&
        storage != PCD_ASCII && storage != PCD_BINARY && storage != PCD_BINARY_COMPRESSED)
    {
      ROS_ERROR_STREAM("Incorrect pointcloud logging type value (" << storage << "). Setting it to 'PCL'.");
      storage = PCL_CLOUD;
    }
    LidarSlam.SetLoggingStorage(storage);
  }

  // Frame Ids
  priv_nh.param("odometry_frame", this->OdometryFrameId, this->OdometryFrameId);
  this->LidarSlam.SetWorldFrameId(this->OdometryFrameId);
  if (priv_nh.getParam("tracking_frame", this->TrackingFrameId))
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
  SetSlamParam(double, "slam/ego_motion_registration/init_loss_scale", EgoMotionInitLossScale)
  SetSlamParam(double, "slam/ego_motion_registration/final_loss_scale", EgoMotionFinalLossScale)

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
  SetSlamParam(double, "slam/localization/init_loss_scale", LocalizationInitLossScale)
  SetSlamParam(double, "slam/localization/final_loss_scale", LocalizationFinalLossScale)

  // Rolling grids
  SetSlamParam(double, "slam/voxel_grid/leaf_size_edges", VoxelGridLeafSizeEdges)
  SetSlamParam(double, "slam/voxel_grid/leaf_size_planes", VoxelGridLeafSizePlanes)
  SetSlamParam(double, "slam/voxel_grid/leaf_size_blobs", VoxelGridLeafSizeBlobs)
  SetSlamParam(double, "slam/voxel_grid/resolution", VoxelGridResolution)
  SetSlamParam(int,    "slam/voxel_grid/size", VoxelGridSize)

  // Keypoints extraction
  #define SetKeypointsExtractorParam(type, rosParam, keParam) {type val; if (priv_nh.getParam(rosParam, val)) this->LidarSlam.GetKeyPointsExtractor()->Set##keParam(val);}
  SetKeypointsExtractorParam(int,    "slam/ke/neighbor_width", NeighborWidth)
  SetKeypointsExtractorParam(double, "slam/ke/min_distance_to_sensor", MinDistanceToSensor)
  SetKeypointsExtractorParam(double, "slam/ke/angle_resolution", AngleResolution)
  SetKeypointsExtractorParam(double, "slam/ke/plane_sin_angle_threshold", PlaneSinAngleThreshold)
  SetKeypointsExtractorParam(double, "slam/ke/edge_sin_angle_threshold", EdgeSinAngleThreshold)
  // SetKeypointsExtractorParam(double, "slam/ke/dist_to_line_threshold", DistToLineThreshold)
  SetKeypointsExtractorParam(double, "slam/ke/edge_depth_gap_threshold", EdgeDepthGapThreshold)
  SetKeypointsExtractorParam(double, "slam/ke/edge_saliency_threshold", EdgeSaliencyThreshold)
  SetKeypointsExtractorParam(double, "slam/ke/edge_intensity_gap_threshold", EdgeIntensityGapThreshold)
}

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_slam");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // create lidar slam node, which subscribes to pointclouds
  LidarSlamNode slam(nh, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}