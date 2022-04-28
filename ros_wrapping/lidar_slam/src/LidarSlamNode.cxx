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

#include <LidarSlam/Utilities.h>

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

  EDGES_SUBMAP,          // Publish edge keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/edges'.
  PLANES_SUBMAP,         // Publish plane keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/planes'.
  BLOBS_SUBMAP,          // Publish blob keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/blobs'.

  EDGE_KEYPOINTS,        // Publish extracted edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/edges'.
  PLANE_KEYPOINTS,       // Publish extracted plane keypoints from current frame as a PointCloud2 msg to topic 'keypoints/planes'.
  BLOB_KEYPOINTS,        // Publish extracted blob keypoints from current frame as a PointCloud2 msg to topic 'keypoints/blobs'.

  SLAM_REGISTERED_POINTS,// Publish SLAM pointcloud as LidarPoint PointCloud2 msg to topic 'slam_registered_points'.

  CONFIDENCE,            // Publish confidence estimators on output pose to topic 'slam_confidence'.

  PGO_PATH,              // Publish optimized SLAM trajectory as Path msg to 'pgo_slam_path' latched topic.
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

  // Load initial Landmarks poses if requested
  std::string lmpath = priv_nh.param<std::string>("external_sensors/landmark_detector/landmarks_file_path", "");
  if (!lmpath.empty())
  {
    ROS_INFO_STREAM("Loading initial landmarks info from CSV.");
    this->LoadLandmarks(lmpath);
    this->LidarSlam.SetLandmarkConstraintLocal(false);
  }
  else
    this->LidarSlam.SetLandmarkConstraintLocal(true);

  // Set initial SLAM pose if requested
  std::vector<double> initialPose;
  if (priv_nh.getParam("maps/initial_pose", initialPose) && initialPose.size() == 6)
  {
    Eigen::Isometry3d poseTransform = LidarSlam::Utils::XYZRPYtoIsometry(initialPose.data());
    this->LidarSlam.SetWorldTransformFromGuess(poseTransform);
    ROS_INFO_STREAM("Setting initial SLAM pose to:\n" << poseTransform.matrix());
  }

  // Use GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  priv_nh.getParam("external_sensors/gps/use_gps", this->UseGps);
  // Use tags data for local optimization.
  this->UseTags = priv_nh.param("external_sensors/landmark_detector/use_tags", false);

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

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::EDGE))
  {
    initPublisher(EDGES_MAP,  "maps/edges",  CloudS, "output/maps/edges",  true, 1, false);
    initPublisher(EDGES_SUBMAP,  "submaps/edges",  CloudS, "output/submaps/edges",  true, 1, false);
    initPublisher(EDGE_KEYPOINTS,  "keypoints/edges",  CloudS, "output/keypoints/edges",  true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::PLANE))
  {
    initPublisher(PLANES_MAP, "maps/planes", CloudS, "output/maps/planes", true, 1, false);
    initPublisher(PLANES_SUBMAP, "submaps/planes", CloudS, "output/submaps/planes", true, 1, false);
    initPublisher(PLANE_KEYPOINTS, "keypoints/planes", CloudS, "output/keypoints/planes", true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::BLOB))
  {
    initPublisher(BLOBS_MAP,  "maps/blobs",  CloudS, "output/maps/blobs",  true, 1, false);
    initPublisher(BLOBS_SUBMAP,  "submaps/blobs",  CloudS, "output/submaps/blobs",  true, 1, false);
    initPublisher(BLOB_KEYPOINTS,  "keypoints/blobs",  CloudS, "output/keypoints/blobs",  true, 1, false);
  }

  initPublisher(SLAM_REGISTERED_POINTS, "slam_registered_points", CloudS, "output/registered_points", true, 1, false);

  initPublisher(CONFIDENCE, "slam_confidence", lidar_slam::Confidence, "output/confidence", true, 1, false);

  if (this->UseGps || this->UseTags)
  {
    initPublisher(PGO_PATH, "pgo_slam_path", nav_msgs::Path, "graph/publish_path", false, 1, true);
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

  // Init logging of landmark data
  if (this->UseTags)
  {
    // Create an external independent spinner to get the landmarks' info in a parallel way
    ros::SubscribeOptions ops;
    ops.initByFullCallbackType<apriltag_ros::AprilTagDetectionArray>("tag_detections", 200,
                                                                      boost::bind(&LidarSlamNode::TagCallback, this, boost::placeholders::_1));
    ops.callback_queue = &this->ExternalQueue;
    this->LandmarksSub = nh.subscribe(ops);
    this->ExternalSpinnerPtr = std::make_shared<ros::AsyncSpinner>(ros::AsyncSpinner(this->LidarSlam.GetNbThreads(), &this->ExternalQueue));
    this->ExternalSpinnerPtr->start();
  }

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

  // Compute time offset
  // Get ROS frame reception time
  double TimeFrameReceptionPOSIX = ros::Time::now().toSec();
  // Get last acquired point timestamp
  double TimeLastPoint = LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp) + cloudS_ptr->back().time;
  // Compute offset
  double potentialOffset = TimeLastPoint - TimeFrameReceptionPOSIX;
  // Get current offset
  double absCurrentOffset = std::abs(this->LidarSlam.GetSensorTimeOffset());
  // If the current computed offset is more accurate, replace it
  if (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset)
    this->LidarSlam.SetSensorTimeOffset(potentialOffset);

  // Update TF from BASE to LiDAR
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id))
    return;

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
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id))
    return;

  // Add new frame to SLAM input frames
  this->Frames.push_back(cloudS_ptr);
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::Odometry& gpsMsg)
{
    if (!this->UseGps)
      return;

    // Transform to apply to points represented in GPS frame to express them in base frame
    Eigen::Isometry3d baseToGps;
    if (Utils::Tf2LookupTransform(baseToGps, this->TfBuffer, this->TrackingFrameId, gpsMsg.header.frame_id, gpsMsg.header.stamp))
    {
      ROS_INFO_STREAM("Adding GPS info");
      // Get gps pose
      this->LastGpsMeas.Position = Utils::PoseMsgToIsometry(gpsMsg.pose.pose).translation();
      // Get gps timestamp
      this->LastGpsMeas.Time = gpsMsg.header.stamp.sec + gpsMsg.header.stamp.nsec * 1e-9;

      // Get tag covariance
      bool ValidCovariance = false;
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
        {
          this->LastGpsMeas.Covariance(i, j) = gpsMsg.pose.covariance[i * 6 + j];
          if (abs(this->LastGpsMeas.Covariance(i, j)) > 1e-10)
            ValidCovariance = true;
        }
      }
      if (!ValidCovariance)
        this->LastGpsMeas.Covariance = Eigen::Matrix3d::Identity() * 1e-4;

      if (!this->LidarSlam.GpsHasData())
        this->LidarSlam.SetGpsCalibration(baseToGps);

      // Add gps measurement to measurements list
      this->LidarSlam.AddGpsMeasurement(this->LastGpsMeas);
      this->GpsLastTime = ros::Time(this->LastGpsMeas.Time);
      this->GpsFrameId = gpsMsg.header.frame_id;
    }
    else
      ROS_WARN_STREAM("The transform between the GPS and the tracking frame was not found -> GPS info ignored");
}

//------------------------------------------------------------------------------
int LidarSlamNode::BuildId(const std::vector<int>& ids)
{
  int id = ids[0];
  if (ids.size() > 1)
  {
    id = 0;
    int power = 0;
    for (unsigned int i = 0; i < ids.size(); ++i)
    {
      power += int(std::log10(ids[i]));
      id += std::pow(10, power) * ids[i];
    }
  }
  return id;
}

//------------------------------------------------------------------------------
void LidarSlamNode::TagCallback(const apriltag_ros::AprilTagDetectionArray& tagsInfo)
{
  if (!this->UseTags)
    return;
  for (auto& tagInfo : tagsInfo.detections)
  {
    // Transform to apply to points represented in detector frame to express them in base frame
    Eigen::Isometry3d baseToLmDetector;
    if (Utils::Tf2LookupTransform(baseToLmDetector, this->TfBuffer, this->TrackingFrameId, tagInfo.pose.header.frame_id, tagInfo.pose.header.stamp))
    {
      ROS_INFO_STREAM("Adding tag info");
      LidarSlam::ExternalSensors::LandmarkMeasurement lm;
      // Get tag pose
      lm.TransfoRelative = Utils::PoseMsgToIsometry(tagInfo.pose.pose.pose);
      // Get tag timestamp
      lm.Time = tagInfo.pose.header.stamp.sec + tagInfo.pose.header.stamp.nsec * 1e-9;

      // Get tag covariance
      bool ValidCovariance = false;
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
        {
          lm.Covariance(i, j) = tagInfo.pose.pose.covariance[i * 6 + j];
          if (abs(lm.Covariance(i, j)) > 1e-10)
            ValidCovariance = true;
        }
      }
      if (!ValidCovariance)
        lm.Covariance = Eigen::Matrix6d::Identity() * 1e-4;

      // Compute tag ID
      int id = this->BuildId(tagInfo.id);

      // Add detector calibration for the first tag detection
      if (!this->LidarSlam.LmHasData())
        this->LidarSlam.SetLmDetectorCalibration(baseToLmDetector);

      // Add tag detection to measurements
      this->LidarSlam.AddLandmarkMeasurement(lm, id);

      if (this->PublishTags)
      {
        // Publish tf
        geometry_msgs::TransformStamped tfStamped;
        tfStamped.header.stamp = tagInfo.pose.header.stamp;
        tfStamped.header.frame_id = this->TrackingFrameId;
        tfStamped.child_frame_id = "tag_" + std::to_string(id);
        tfStamped.transform = Utils::IsometryToTfMsg(lm.TransfoRelative);
        this->TfBroadcaster.sendTransform(tfStamped);
      }
    }
    else
      ROS_WARN_STREAM("The transform between the landmark detector and the tracking frame was not found -> landmarks info ignored");
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::LoadLandmarks(const std::string& path)
{
  // Check the file
  if (path.substr(path.find_last_of(".") + 1) != "csv")
  {
    ROS_ERROR_STREAM("The landmarks file is not csv : landmarks absolute constraint can not be used");
    return;
  }

  std::ifstream lmFile(path);
  if (lmFile.fail())
  {
    ROS_ERROR_STREAM("The landmarks csv file " << path << " was not found : landmarks absolute constraint can not be used");
    return;
  }

  const unsigned int fieldsNumber = 43;
  // Check which delimiter is used
  std::string lmStr;
  std::vector<std::string> delimiters = {";", ",", " ", "\t"};
  std::string delimiter;
  getline (lmFile, lmStr);
  std::vector<std::string> fields;

  for (const auto& del : delimiters)
  {
    fields.clear();
    size_t pos = 0;
    std::string headerLine = lmStr;
    pos = headerLine.find(del);
    while (pos != std::string::npos)
    {
      fields.push_back(headerLine.substr(0, pos));
      headerLine.erase(0, pos + del.length());
      pos = headerLine.find(del);
    }
    // If there is some element after the last delimiter, add it
    if (!headerLine.substr(0, pos).empty())
      fields.push_back(headerLine.substr(0, pos));
    // Check that the number of fields is correct
    if (fields.size() == fieldsNumber)
    {
      delimiter = del;
      break;
    }
  }
  if (fields.size() != fieldsNumber)
  {
    ROS_WARN_STREAM("The landmarks csv file is ill formed : " << fields.size() << " fields were found (" << fieldsNumber
                    << " expected), landmarks absolute poses will not be used");
    return;
  }

  int lineIdx = 1;
  int ntags = 0;
  do
  {
    size_t pos = 0;
    std::vector<std::string> lm;
    while ((pos = lmStr.find(delimiter)) != std::string::npos)
    {
      // Remove potential extra spaces after the delimiter
      unsigned int charIdx = 0;
      while (charIdx < lmStr.size() && lmStr[charIdx] == ' ')
        ++charIdx;
      lm.push_back(lmStr.substr(charIdx, pos));
      lmStr.erase(0, pos + delimiter.length());
    }
    lm.push_back(lmStr.substr(0, pos));
    if (lm.size() != fieldsNumber)
    {
      ROS_WARN_STREAM("landmark on line " + std::to_string(lineIdx) + " is not correct -> Skip");
      ++lineIdx;
      continue;
    }

    // Check numerical values in the studied line
    bool numericalIssue = false;
    for (std::string field : lm)
    {
      try
      {
        std::stof(field);
      }
      catch(std::invalid_argument& e)
      {
        numericalIssue = true;
        break;
      }
    }
    if (numericalIssue)
    {
      // if this is the first line, it might be a header line,
      // else, print a warning
      if (lineIdx > 1)
        ROS_WARN_STREAM("landmark on line " + std::to_string(lineIdx) + " contains a not numerical value -> Skip");
      ++lineIdx;
      continue;
    }

    // Build measurement
    // Set landmark id
    int id = std::stoi(lm[0]);
    // Fill pose
    Eigen::Vector6d absolutePose;
    absolutePose << std::stof(lm[1]), std::stof(lm[2]), std::stof(lm[3]), std::stof(lm[4]), std::stof(lm[5]), std::stof(lm[6]);
    // Fill covariance
    Eigen::Matrix6d absolutePoseCovariance;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        absolutePoseCovariance(i, j) = std::stof(lm[7 + 6 * i + j]);
    }
    // Add a new landmark manager for absolute constraint computing
    this->LidarSlam.AddLandmarkManager(id, absolutePose, absolutePoseCovariance);
    ROS_INFO_STREAM("Tag #" << id << " initialized to \n" << absolutePose.transpose());
    ++ntags;
    ++lineIdx;
  }
  while (getline (lmFile, lmStr));

  lmFile.close();
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  // Get transform between msg frame and odometry frame
  Eigen::Isometry3d msgFrameToOdom;
  if (Utils::Tf2LookupTransform(msgFrameToOdom, this->TfBuffer, msg.header.frame_id, this->OdometryFrameId, msg.header.stamp))
  {
    // Compute pose in odometry frame and set SLAM pose
    Eigen::Isometry3d odomToBase = msgFrameToOdom.inverse() * Utils::PoseMsgToIsometry(msg.pose.pose);
    this->LidarSlam.SetWorldTransformFromGuess(odomToBase);
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
    // Set SLAM pose from last received GPS pose
    // NOTE : This function should only be called after PGO or SLAM/GPS calib have been triggered.
    case lidar_slam::SlamCommand::GPS_SLAM_CALIBRATION:
    {
      if (!this->UseGps || !this->LidarSlam.GpsHasData())
      {
        ROS_ERROR_STREAM("Cannot set SLAM pose from GPS"
                         "Please check that 'external_sensors/gps/use_gps' private parameter is set to 'true'."
                         "and that GPS data have been received.");
        return;
      }
      this->LidarSlam.CalibrateWithGps();
      ROS_WARN_STREAM("SLAM pose set using GPS pose to :\n" << this->LidarSlam.GetLastState().Isometry.matrix());
      // Broadcast new calibration offset (GPS reference frame (i.e. generally UTM) to odom)
      this->BroadcastGpsOffset();
      break;
    }

    // Set SLAM pose from last received GPS pose
    // NOTE : This function should only be called after PGO or SLAM/GPS calib have been triggered.
    case lidar_slam::SlamCommand::SET_SLAM_POSE_FROM_GPS:
    {
      if (!this->UseGps || !this->LidarSlam.GpsHasData())
      {
        ROS_ERROR_STREAM("Cannot set SLAM pose from GPS"
                          "Please check that 'external_sensors/gps/use_gps' private parameter is set to 'true'."
                          "and that GPS data have been received.");
        return;
      }

      LidarSlam::ExternalSensors::GpsMeasurement& meas = this->LastGpsMeas;
      // Get position of Lidar in UTM
      Eigen::Vector3d position = this->LidarSlam.GetGpsCalibration().inverse() * meas.Position;
      // Get position of Lidar in odometry frame
      position = this->LidarSlam.GetGpsOffset() * position;
      // Orientation is supposed to be close to odometry frame
      // Warning : this hypothesis can be totally wrong and lead to bad registrations
      Eigen::Isometry3d pose = this->LidarSlam.GetLogStates().front().Isometry;
      pose.translation() = position;
      this->LidarSlam.SetWorldTransformFromGuess(pose);
      ROS_WARN_STREAM("SLAM pose set from GPS pose to :\n" << pose.matrix());
      break;
    }

    // Disable SLAM maps update
    case lidar_slam::SlamCommand::DISABLE_SLAM_MAP_UPDATE:
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::NONE);
      ROS_WARN_STREAM("Disabling SLAM maps update.");
      break;

    // Enable the agregation of keypoints to a fixed initial map
    case lidar_slam::SlamCommand::ENABLE_SLAM_MAP_EXPANSION:
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP);
      ROS_WARN_STREAM("Enabling SLAM maps expansion with new keypoints.");
      break;

    // Enable the update of the map with new keypoints
    case lidar_slam::SlamCommand::ENABLE_SLAM_MAP_UPDATE:
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::UPDATE);
      ROS_WARN_STREAM("Enabling SLAM maps update with new keypoints.");
      break;

    // Save SLAM keypoints maps to PCD files
    case lidar_slam::SlamCommand::SAVE_KEYPOINTS_MAPS:
    {
      ROS_INFO_STREAM("Saving keypoints maps to PCD.");
      if (this->LidarSlam.GetMapUpdate() == LidarSlam::MappingMode::NONE)
        ROS_WARN_STREAM("The initially loaded maps were not modified but are saved anyway.");
      int pcdFormatInt = this->PrivNh.param("maps/export_pcd_format", static_cast<int>(LidarSlam::PCDFormat::BINARY_COMPRESSED));
      LidarSlam::PCDFormat pcdFormat = static_cast<LidarSlam::PCDFormat>(pcdFormatInt);
      if (pcdFormat != LidarSlam::PCDFormat::ASCII &&
          pcdFormat != LidarSlam::PCDFormat::BINARY &&
          pcdFormat != LidarSlam::PCDFormat::BINARY_COMPRESSED)
      {
        ROS_ERROR_STREAM("Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
        pcdFormat = LidarSlam::PCDFormat::BINARY_COMPRESSED;
      }
      this->LidarSlam.SaveMapsToPCD(msg.string_arg, pcdFormat, false);
      break;
    }

    // Save SLAM keypoints submaps to PCD files
    case lidar_slam::SlamCommand::SAVE_FILTERED_KEYPOINTS_MAPS:
    {
      ROS_INFO_STREAM("Saving keypoints submaps to PCD.");
      int pcdFormatInt = this->PrivNh.param("maps/export_pcd_format", static_cast<int>(LidarSlam::PCDFormat::BINARY_COMPRESSED));
      LidarSlam::PCDFormat pcdFormat = static_cast<LidarSlam::PCDFormat>(pcdFormatInt);
      if (pcdFormat != LidarSlam::PCDFormat::ASCII &&
          pcdFormat != LidarSlam::PCDFormat::BINARY &&
          pcdFormat != LidarSlam::PCDFormat::BINARY_COMPRESSED)
      {
        ROS_ERROR_STREAM("Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
        pcdFormat = LidarSlam::PCDFormat::BINARY_COMPRESSED;
      }
      this->LidarSlam.SaveMapsToPCD(msg.string_arg, pcdFormat, true);
      break;
    }

    // Load SLAM keypoints maps from PCD files
    case lidar_slam::SlamCommand::LOAD_KEYPOINTS_MAPS:
      ROS_INFO_STREAM("Loading keypoints maps from PCD.");
      this->LidarSlam.LoadMapsFromPCD(msg.string_arg);
      break;

    case lidar_slam::SlamCommand::OPTIMIZE_GRAPH:
      if ((!this->UseGps && !this->UseTags) || this->LidarSlam.GetSensorMaxMeasures() < 2 || this->LidarSlam.GetLoggingTimeout() < 0.2)
      {
        ROS_ERROR_STREAM("Cannot optimize pose graph as sensor info logging has not been enabled. "
                         "Please make sure that 'external_sensors/landmark_detector/use_tags' OR 'external_sensors/gps/use_gps' private parameter is set to 'true', "
                         "and that 'external_sensors/landmark_detector/weight' and 'slam/logging/timeout' private parameters are set to convenient values.");
        break;
      }

      if (this->LidarSlam.LmHasData())
      {
        if (!msg.string_arg.empty())
        {
          ROS_INFO_STREAM("Loading the absolute landmark poses");
          this->LoadLandmarks(msg.string_arg);
        }
        else if (this->LidarSlam.GetLandmarkConstraintLocal())
          ROS_WARN_STREAM("No absolute landmark poses are supplied : the last estimated poses will be used");
      }

      ROS_INFO_STREAM("Optimizing the pose graph");
      this->LidarSlam.OptimizeGraph();
      // Broadcast new calibration offset (GPS to base)
      // if GPS used
      if (this->LidarSlam.GpsHasData())
        this->BroadcastGpsOffset();
      // Publish new trajectory
      if (this->Publish[PGO_PATH])
      {
        nav_msgs::Path optimSlamTraj;
        optimSlamTraj.header.frame_id = this->OdometryFrameId;
        std::list<LidarSlam::LidarState> optimizedSlamStates = this->LidarSlam.GetLogStates();
        optimSlamTraj.header.stamp = ros::Time(optimizedSlamStates.back().Time);
        for (const LidarSlam::LidarState& s: optimizedSlamStates)
          optimSlamTraj.poses.emplace_back(Utils::IsometryToPoseStampedMsg(s.Isometry, s.Time, this->OdometryFrameId));
        this->Publishers[PGO_PATH].publish(optimSlamTraj);
      }

      break;

    // Unknown command
    default:
      ROS_ERROR_STREAM("Unknown SLAM command : " << (unsigned int) msg.command);
      break;
  }
}

//==============================================================================
//   Utilities
//==============================================================================

//------------------------------------------------------------------------------
bool LidarSlamNode::UpdateBaseToLidarOffset(const std::string& lidarFrameId, uint8_t lidarDeviceId)
{
  // If tracking frame is different from input frame, get TF from LiDAR to BASE
  if (lidarFrameId != this->TrackingFrameId)
  {
    // We expect a static transform between BASE and LIDAR, so we don't care
    // about timestamp and get only the latest transform
    Eigen::Isometry3d baseToLidar;
    if (Utils::Tf2LookupTransform(baseToLidar, this->TfBuffer, this->TrackingFrameId, lidarFrameId))
      this->LidarSlam.SetBaseToLidarOffset(baseToLidar, lidarDeviceId);
    else
      return false;
  }
  return true;
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishOutput()
{
  LidarSlam::LidarState& currentState = this->LidarSlam.GetLastState();
  double currentTime = currentState.Time;
  // Publish SLAM pose
  if (this->Publish[POSE_ODOM] || this->Publish[POSE_TF])
  {
    // Publish as odometry msg
    if (this->Publish[POSE_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(currentTime);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId;
      odomMsg.pose.pose = Utils::IsometryToPoseMsg(currentState.Isometry);
      // Note : in eigen 3.4 iterators are available on matrices directly
      //        >> std::copy(currentState.Covariance.begin(), currentState.Covariance.end(), confidenceMsg.covariance.begin());
      // For now the only way is to copy or iterate on indices :
      for (unsigned int i = 0; i < currentState.Covariance.size(); ++i)
        odomMsg.pose.covariance[i] = currentState.Covariance(i);
      this->Publishers[POSE_ODOM].publish(odomMsg);
    }

    // Publish as TF from OdometryFrameId to TrackingFrameId
    if (this->Publish[POSE_TF])
    {
      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.stamp = ros::Time(currentTime);
      tfMsg.header.frame_id = this->OdometryFrameId;
      tfMsg.child_frame_id = this->TrackingFrameId;
      tfMsg.transform = Utils::IsometryToTfMsg(currentState.Isometry);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }

  // Publish latency compensated SLAM pose
  if (this->Publish[POSE_PREDICTION_ODOM] || this->Publish[POSE_PREDICTION_TF])
  {
    double predTime = currentState.Time + this->LidarSlam.GetLatency();
    Eigen::Isometry3d predTransfo = this->LidarSlam.GetLatencyCompensatedWorldTransform();
    // Publish as odometry msg
    if (this->Publish[POSE_PREDICTION_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(predTime);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      odomMsg.pose.pose = Utils::IsometryToPoseMsg(predTransfo);
      // Note : in eigen 3.4 iterators are available on matrices directly
      //        >> std::copy(currentState.Covariance.begin(), currentState.Covariance.end(), confidenceMsg.covariance.begin());
      // for now the only way is to copy or iterate on indices :
      for (unsigned int i = 0; i < currentState.Covariance.size(); ++i)
        odomMsg.pose.covariance[i] = currentState.Covariance(i);
      this->Publishers[POSE_PREDICTION_ODOM].publish(odomMsg);
    }

    // Publish as TF from OdometryFrameId to <TrackingFrameId>_prediction
    if (this->Publish[POSE_PREDICTION_TF])
    {
      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.stamp = ros::Time(predTime);
      tfMsg.header.frame_id = this->OdometryFrameId;
      tfMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      tfMsg.transform = Utils::IsometryToTfMsg(predTransfo);
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

  // Keypoints submaps
  publishPointCloud(EDGES_SUBMAP,  this->LidarSlam.GetTargetSubMap(LidarSlam::EDGE));
  publishPointCloud(PLANES_SUBMAP, this->LidarSlam.GetTargetSubMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_SUBMAP,  this->LidarSlam.GetTargetSubMap(LidarSlam::BLOB));

  // Current keypoints
  publishPointCloud(EDGE_KEYPOINTS,  this->LidarSlam.GetKeypoints(LidarSlam::EDGE));
  publishPointCloud(PLANE_KEYPOINTS, this->LidarSlam.GetKeypoints(LidarSlam::PLANE));
  publishPointCloud(BLOB_KEYPOINTS,  this->LidarSlam.GetKeypoints(LidarSlam::BLOB));

  // Registered aggregated (and optionally undistorted) input scans points
  publishPointCloud(SLAM_REGISTERED_POINTS, this->LidarSlam.GetRegisteredFrame());

  // Overlap estimation
  if (this->Publish[CONFIDENCE])
  {
    // Get SLAM pose
    lidar_slam::Confidence confidenceMsg;
    confidenceMsg.header.stamp = ros::Time(currentTime);
    confidenceMsg.header.frame_id = this->OdometryFrameId;
    confidenceMsg.overlap = this->LidarSlam.GetOverlapEstimation();
    confidenceMsg.computation_time = this->LidarSlam.GetLatency();
    // Note : in eigen 3.4, iterators are available on matrices directly
    //        >> std::copy(currentState.Covariance.begin(), currentState.Covariance.end(), confidenceMsg.covariance.begin());
    for (unsigned int i = 0; i < currentState.Covariance.size(); ++i)
      confidenceMsg.covariance[i] = currentState.Covariance(i);
    confidenceMsg.nb_matches = this->LidarSlam.GetTotalMatchedKeypoints();
    confidenceMsg.comply_motion_limits = this->LidarSlam.GetComplyMotionLimits();
    this->Publishers[CONFIDENCE].publish(confidenceMsg);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters()
{
  #define SetSlamParam(type, rosParam, slamParam) { type val; if (this->PrivNh.getParam(rosParam, val)) this->LidarSlam.Set##slamParam(val); }
  // General
  SetSlamParam(bool,   "slam/2d_mode", TwoDMode)
  SetSlamParam(int,    "slam/verbosity", Verbosity)
  SetSlamParam(int,    "slam/n_threads", NbThreads)
  SetSlamParam(double, "slam/logging/timeout", LoggingTimeout)
  SetSlamParam(bool,   "slam/logging/only_keyframes", LogOnlyKeyframes)
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
    this->LidarSlam.SetEgoMotion(egoMotion);
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
  if (this->PrivNh.getParam("slam/logging/storage_type", pointCloudStorage))
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
    #define EnableKeypoint(kType) \
    { \
      bool enabled = false; \
      std::string name = LidarSlam::KeypointTypeNames.at(kType); \
      if (this->PrivNh.getParam(prefix + "enable/"+ name, enabled)) \
        this->LidarSlam.EnableKeypointType(kType, enabled); \
      if (enabled) \
        ROS_INFO_STREAM("Keypoint of type " + name + " enabled"); \
      else \
        ROS_INFO_STREAM("Keypoint of type " + name + " disabled"); \
    }
    EnableKeypoint(LidarSlam::Keypoint::EDGE);
    EnableKeypoint(LidarSlam::Keypoint::PLANE);
    EnableKeypoint(LidarSlam::Keypoint::BLOB);
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

  // Ego motion
  SetSlamParam(int,    "slam/ego_motion_registration/ICP_max_iter", EgoMotionICPMaxIter)
  SetSlamParam(int,    "slam/ego_motion_registration/LM_max_iter", EgoMotionLMMaxIter)
  SetSlamParam(double, "slam/ego_motion_registration/max_neighbors_distance", EgoMotionMaxNeighborsDistance)
  SetSlamParam(int,    "slam/ego_motion_registration/edge_nb_neighbors", EgoMotionEdgeNbNeighbors)
  SetSlamParam(int,    "slam/ego_motion_registration/edge_min_nb_neighbors", EgoMotionEdgeMinNbNeighbors)
  SetSlamParam(double, "slam/ego_motion_registration/edge_max_model_error", EgoMotionEdgeMaxModelError)
  SetSlamParam(int,    "slam/ego_motion_registration/plane_nb_neighbors", EgoMotionPlaneNbNeighbors)
  SetSlamParam(double, "slam/ego_motion_registration/planarity_threshold", EgoMotionPlanarityThreshold)
  SetSlamParam(double, "slam/ego_motion_registration/plane_max_model_error", EgoMotionPlaneMaxModelError)
  SetSlamParam(double, "slam/ego_motion_registration/init_saturation_distance", EgoMotionInitSaturationDistance)
  SetSlamParam(double, "slam/ego_motion_registration/final_saturation_distance", EgoMotionFinalSaturationDistance)

  // Localization
  SetSlamParam(int,    "slam/localization/ICP_max_iter", LocalizationICPMaxIter)
  SetSlamParam(int,    "slam/localization/LM_max_iter", LocalizationLMMaxIter)
  SetSlamParam(double, "slam/localization/max_neighbors_distance", LocalizationMaxNeighborsDistance)
  SetSlamParam(int,    "slam/localization/edge_nb_neighbors", LocalizationEdgeNbNeighbors)
  SetSlamParam(int,    "slam/localization/edge_min_nb_neighbors", LocalizationEdgeMinNbNeighbors)
  SetSlamParam(double, "slam/localization/edge_max_model_error", LocalizationEdgeMaxModelError)
  SetSlamParam(int,    "slam/localization/plane_nb_neighbors", LocalizationPlaneNbNeighbors)
  SetSlamParam(double, "slam/localization/planarity_threshold", LocalizationPlanarityThreshold)
  SetSlamParam(double, "slam/localization/plane_max_model_error", LocalizationPlaneMaxModelError)
  SetSlamParam(int,    "slam/localization/blob_nb_neighbors", LocalizationBlobNbNeighbors)
  SetSlamParam(double, "slam/localization/init_saturation_distance", LocalizationInitSaturationDistance)
  SetSlamParam(double, "slam/localization/final_saturation_distance", LocalizationFinalSaturationDistance)

  // External sensors
  SetSlamParam(float,  "external_sensors/max_measures", SensorMaxMeasures)
  SetSlamParam(float,  "external_sensors/time_threshold", SensorTimeThreshold)
  SetSlamParam(float,  "external_sensors/landmark_detector/weight", LandmarkWeight)
  SetSlamParam(float,  "external_sensors/landmark_detector/saturation_distance", LandmarkSaturationDistance)
  SetSlamParam(bool,   "external_sensors/landmark_detector/position_only", LandmarkPositionOnly)
  this->PrivNh.getParam("external_sensors/landmark_detector/publish_tags", this->PublishTags);

  // Graph parameters
  SetSlamParam(std::string, "graph/g2o_file_name", G2oFileName)
  SetSlamParam(bool,   "graph/fix_first", FixFirstVertex)
  SetSlamParam(bool,   "graph/fix_last", FixLastVertex)

  // Confidence estimators
  // Overlap
  SetSlamParam(float,  "slam/confidence/overlap/sampling_ratio", OverlapSamplingRatio)
  // Motion limitations (hard constraints to detect failure)
  std::vector<float> acc;
  if (this->PrivNh.getParam("slam/confidence/motion_limits/acceleration", acc) && acc.size() == 2)
    this->LidarSlam.SetAccelerationLimits(Eigen::Map<const Eigen::Array2f>(acc.data()));
  std::vector<float> vel;
  if (this->PrivNh.getParam("slam/confidence/motion_limits/velocity", vel) && vel.size() == 2)
    this->LidarSlam.SetVelocityLimits(Eigen::Map<const Eigen::Array2f>(vel.data()));
  SetSlamParam(float, "slam/confidence/motion_limits/time_window_duration", TimeWindowDuration)

  // Keyframes
  SetSlamParam(double, "slam/keyframes/distance_threshold", KfDistanceThreshold)
  SetSlamParam(double, "slam/keyframes/angle_threshold", KfAngleThreshold)

  // Maps
  int mapUpdateMode;
  if (this->PrivNh.getParam("slam/voxel_grid/update_maps", mapUpdateMode))
  {
    LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mapUpdateMode);
    if (mapUpdate != LidarSlam::MappingMode::NONE &&
        mapUpdate != LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP &&
        mapUpdate != LidarSlam::MappingMode::UPDATE)
    {
      ROS_ERROR_STREAM("Invalid map update mode (" << mapUpdateMode << "). Setting it to 'UPDATE'.");
      mapUpdate = LidarSlam::MappingMode::UPDATE;
    }
    this->LidarSlam.SetMapUpdate(mapUpdate);
  }
  double size;
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size_edges", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::EDGE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::EDGE, size);
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size_planes", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::PLANE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::PLANE, size);
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size_blobs", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::BLOB))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::BLOB, size);
  SetSlamParam(double, "slam/voxel_grid/resolution", VoxelGridResolution)
  SetSlamParam(int,    "slam/voxel_grid/size", VoxelGridSize)
  SetSlamParam(double, "slam/voxel_grid/decaying_threshold", VoxelGridDecayingThreshold)
  SetSlamParam(int,    "slam/voxel_grid/min_frames_per_voxel", VoxelGridMinFramesPerVoxel)
  for (auto k : LidarSlam::KeypointTypes)
  {
    if (!this->LidarSlam.KeypointTypeEnabled(k))
      continue;
    int samplingMode;
    if (this->PrivNh.getParam("slam/voxel_grid/sampling_mode/" + LidarSlam::KeypointTypeNames.at(k), samplingMode))
    {
      LidarSlam::SamplingMode sampling = static_cast<LidarSlam::SamplingMode>(samplingMode);
      if (sampling != LidarSlam::SamplingMode::FIRST &&
          sampling != LidarSlam::SamplingMode::LAST &&
          sampling != LidarSlam::SamplingMode::MAX_INTENSITY &&
          sampling != LidarSlam::SamplingMode::CENTER_POINT &&
          sampling != LidarSlam::SamplingMode::CENTROID)
      {
        ROS_ERROR_STREAM("Invalid sampling mode (" << samplingMode << ") for "
                                                   << LidarSlam::Utils::Plural(LidarSlam::KeypointTypeNames.at(k))
                                                   << ". Setting it to 'MAX_INTENSITY'.");
        sampling = LidarSlam::SamplingMode::MAX_INTENSITY;
      }
      this->LidarSlam.SetVoxelGridSamplingMode(k, sampling);
    }
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::BroadcastGpsOffset()
{
  Eigen::Isometry3d offset = this->LidarSlam.GetGpsOffset().inverse();
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = this->GpsLastTime;
  tfStamped.header.frame_id = this->GpsFrameId;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = Utils::IsometryToTfMsg(offset);
  this->StaticTfBroadcaster.sendTransform(tfStamped);
}