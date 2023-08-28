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


#ifdef USE_CV_BRIDGE
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/image_encodings.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

std::vector<std::string> DELIMITERS = {";", ",", " ", "\t"};

enum Output
{
  POSE_ODOM,                 // Publish SLAM pose as an Odometry msg on 'slam_odom' topic (default : true).
  POSE_TF,                   // Publish SLAM pose as a TF from 'odometry_frame' to 'tracking_frame' (default : true).
  POSE_PREDICTION_ODOM,      // Publish latency-corrected SLAM pose as an Odometry msg on 'slam_predicted_odom' topic.
  POSE_PREDICTION_TF,        // Publish latency-corrected SLAM pose as a TF from 'odometry_frame' to '<tracking_frame>_prediction'.

  EDGES_MAP,                 // Publish edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/edges'.
  INTENSITY_EDGES_MAP,       // Publish intensity edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/intensity_edges'.
  PLANES_MAP,                // Publish plane keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/planes'.
  BLOBS_MAP,                 // Publish blob keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/blobs'.

  EDGES_SUBMAP,              // Publish edge keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/edges'.
  INTENSITY_EDGES_SUBMAP,    // Publish intensity edge keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/intensity_edges'.
  PLANES_SUBMAP,             // Publish plane keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/planes'.
  BLOBS_SUBMAP,              // Publish blob keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/blobs'.

  EDGE_KEYPOINTS,            // Publish extracted edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/edges'.
  INTENSITY_EDGE_KEYPOINTS,  // Publish extracted intensity edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/intensity_edges'.
  PLANE_KEYPOINTS,           // Publish extracted plane keypoints from current frame as a PointCloud2 msg to topic 'keypoints/planes'.
  BLOB_KEYPOINTS,            // Publish extracted blob keypoints from current frame as a PointCloud2 msg to topic 'keypoints/blobs'.

  SLAM_REGISTERED_POINTS,    // Publish SLAM pointcloud as LidarPoint PointCloud2 msg to topic 'slam_registered_points'.

  CONFIDENCE,                // Publish confidence estimators on output pose to topic 'slam_confidence'.

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
  this->SetSlamInitialState();

  // Use GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  this->UseExtSensor[LidarSlam::GPS] = priv_nh.param("external_sensors/gps/enable", false);
  this->LidarSlam.EnablePGOConstraint(LidarSlam::PGO_GPS, this->UseExtSensor[LidarSlam::GPS]);
  // Use tags data for local optimization.
  this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR] = priv_nh.param("external_sensors/landmark_detector/enable", false);
  this->LidarSlam.EnablePGOConstraint(LidarSlam::LANDMARK, this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR]);
  // Use camera rgb images in local optimization.
  this->UseExtSensor[LidarSlam::CAMERA] = priv_nh.param("external_sensors/camera/enable", false);
  // Use external poses in local optimization or in graph optimization
  this->UseExtSensor[LidarSlam::POSE] = priv_nh.param("external_sensors/external_poses/enable", false);
  this->LidarSlam.EnablePGOConstraint(LidarSlam::PGO_EXT_POSE, this->UseExtSensor[LidarSlam::POSE]);

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

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::INTENSITY_EDGE))
  {
    initPublisher(INTENSITY_EDGES_MAP,  "maps/intensity_edges",  CloudS, "output/maps/intensity_edges",  true, 1, false);
    initPublisher(INTENSITY_EDGES_SUBMAP,  "submaps/intensity_edges",  CloudS, "output/submaps/intensity_edges",  true, 1, false);
    initPublisher(INTENSITY_EDGE_KEYPOINTS,  "keypoints/intensity_edges",  CloudS, "output/keypoints/intensity_edges",  true, 1, false);
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

  if (this->UseExtSensor[LidarSlam::GPS] ||
      this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR] ||
      this->UseExtSensor[LidarSlam::POSE])
  {
    initPublisher(PGO_PATH, "pgo_slam_path", nav_msgs::Path, "graph/publish_path", false, 1, true);
  }

  // Set frequency of output pose (all poses are published at the end of the frames process)
  priv_nh.param("output/pose/frequency", this->TrajFrequency, -1.);

  // ***************************************************************************
  // Init ROS subscribers

  // LiDAR inputs
  std::vector<std::string> lidarTopics;
  ros::Subscriber sub;
  if (!priv_nh.getParam("input", lidarTopics))
    lidarTopics.push_back(priv_nh.param<std::string>("input", "lidar_points"));
  // this代表当前LidarSlamNode实例

  // this->CloudSubs.push_back(nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 1, &LidarSlamNode::ScanCallbackTest, this));
  this->CloudSubs.push_back(nh.subscribe("hesai/pandar", 1, &LidarSlamNode::ScanCallbackTest, this));
  // ROS_INFO_STREAM("Using LiDAR frames on topic '" << lidarTopics[0] << "'");
  // for (unsigned int lidarTopicId = 1; lidarTopicId < lidarTopics.size(); lidarTopicId++)
  // {
  //   this->CloudSubs.push_back(nh.subscribe(lidarTopics[lidarTopicId], 1, &LidarSlamNode::SecondaryScanCallback, this));
  //   ROS_INFO_STREAM("Using secondary LiDAR frames on topic '" << lidarTopics[lidarTopicId] << "'");
  // }

  // Set SLAM pose from external guess
  this->SetPoseSub = nh.subscribe("set_slam_pose", 1, &LidarSlamNode::SetPoseCallback, this);

  // SLAM commands
  this->SlamCommandSub = nh.subscribe("slam_command", 1, &LidarSlamNode::SlamCommandCallback, this);

  // Init logging of GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  // Perfect synchronization is not required as GPS data are not used in SLAM local process
  if (this->UseExtSensor[LidarSlam::GPS])
    this->GpsOdomSub = nh.subscribe("gps_odom", 1, &LidarSlamNode::GpsCallback, this);

  // Init logging of landmark data and/or Camera data
  if (this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR] ||
      this->UseExtSensor[LidarSlam::CAMERA] ||
      this->UseExtSensor[LidarSlam::POSE])
  {
    // Create an external independent spinner to get the landmarks and/or camera info in a parallel way

    if (this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR])
    {
      ros::SubscribeOptions ops;
      ops.initByFullCallbackType<apriltag_ros::AprilTagDetectionArray>("tag_detections", 200,
                                                                        boost::bind(&LidarSlamNode::TagCallback, this, boost::placeholders::_1));
      ops.callback_queue = &this->ExternalQueue;
      this->LandmarksSub = nh.subscribe(ops);
    }

    if (this->UseExtSensor[LidarSlam::CAMERA])
    {
      ros::SubscribeOptions opsImage;
      opsImage.initByFullCallbackType<sensor_msgs::Image>("camera", 10, boost::bind(&LidarSlamNode::ImageCallback,
                                                          this, boost::placeholders::_1));
      opsImage.callback_queue = &this->ExternalQueue;
      this->CameraSub = nh.subscribe(opsImage);

      ros::SubscribeOptions opsCameraInfo;
      opsCameraInfo.initByFullCallbackType<sensor_msgs::CameraInfo>("camera_info", 10, boost::bind(&LidarSlamNode::CameraInfoCallback,
                                                                    this, boost::placeholders::_1));
      opsCameraInfo.callback_queue = &this->ExternalQueue;
      this->CameraInfoSub = nh.subscribe(opsCameraInfo);
    }

    if (this->UseExtSensor[LidarSlam::POSE])
    {
      ros::SubscribeOptions ops;
      ops.initByFullCallbackType<geometry_msgs::PoseWithCovarianceStamped>("ext_poses", 10,
                                                                           boost::bind(&LidarSlamNode::ExtPoseCallback,
                                                                           this, boost::placeholders::_1));
      ops.callback_queue = &this->ExternalQueue;
      this->ExtPoseSub = nh.subscribe(ops);
    }

    this->ExternalSpinnerPtr = std::make_shared<ros::AsyncSpinner>(ros::AsyncSpinner(this->LidarSlam.GetNbThreads(), &this->ExternalQueue));
    this->ExternalSpinnerPtr->start();
  }

  ROS_INFO_STREAM(BOLD_GREEN("LiDAR SLAM is ready !"));
}

//------------------------------------------------------------------------------
LidarSlamNode::~LidarSlamNode()
{
    // Gracefully stop spinner
    // Not stopping async spinner can lead to boost::lock error on shutdown
    if (ExternalSpinnerPtr)
        this->ExternalSpinnerPtr->stop();
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallbackTest(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  std::cout << "----ros time------" << ros::Time::now().toSec() << std::endl;
  pcl::PointCloud<Point3> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  CloudS::Ptr cloudS_ptr(new CloudS);

  int plsize = pl_orig.points.size();
  cloudS_ptr->header.frame_id = "base_link";
  cloudS_ptr->header.stamp = msg->header.stamp.toSec() * 1000000;
  cloudS_ptr->header.seq = msg->header.seq;

  std::cout << "----msg stamp------" <<  cloudS_ptr->header.stamp << std::endl;

  for (int i = 0; i < plsize; i++)
  {
    PointS lidar_point;
    lidar_point.x = pl_orig.points[i].x;
    lidar_point.y = pl_orig.points[i].y;
    lidar_point.z = pl_orig.points[i].z;
    //?:这里有个问题是，点云的时间戳是否需要乘上这个time_scale
    lidar_point.time = pl_orig.points[i].timestamp -  msg->header.stamp.toSec();
    // cout << "~~~~~~~test time: " << lidar_point.time << endl;
    //?:bag 包里点云好像没有intensity属性
    lidar_point.intensity = pl_orig.points[i].intensity;
    lidar_point.laser_id = pl_orig.points[i].ring;
    lidar_point.device_id = 0;
    lidar_point.label = 0;

    cloudS_ptr->points.push_back(lidar_point);
  }
  cout << cloudS_ptr->back().time << endl;

    if (!this->SlamEnabled)
    return;

  if(cloudS_ptr->empty())
  {
    ROS_WARN_STREAM("Input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  this->MainLidarId = cloudS_ptr->header.frame_id;
  this->StartTime = ros::Time::now().toSec();

  // Update TF from BASE to LiDAR
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id))
    return;

  // Set the SLAM main input frame at first position
  this->Frames.insert(this->Frames.begin(), cloudS_ptr);

  // Run SLAM : register new frame and update localization and map.
  this->LidarSlam.AddFrames(this->Frames);

  // TMP
  // Check if SLAM has failed
  if (this->LidarSlam.IsRecovery())
  {
    // TMP : in the future, the user should have a look
    // at the result to validate recovery
    // Check if the SLAM can go on and pose has to be displayed
    if (this->LidarSlam.GetOverlapEstimation() > 0.2f &&
        this->LidarSlam.GetPositionErrorStd()  < 0.1f)
    {
      ROS_WARN_STREAM("Getting out of recovery mode");
      // Frame is relocalized, reset params
      this->LidarSlam.EndRecovery();
    }
    else
      ROS_WARN_STREAM("Still waiting for recovery");
  }
  else if (this->LidarSlam.HasFailed())
  {
    ROS_WARN_STREAM("SLAM has failed : entering recovery mode :\n"
                    << "\t -Maps will not be updated\n"
                    << "\t -Egomotion and undistortion are disabled\n"
                    << "\t -The number of ICP iterations is increased\n"
                    << "\t -The maximum distance between a frame point and a map target point is increased");
    // Enable recovery mode :
    // Last frames are removed
    // Maps are not updated
    // Param are tuned to handle bigger motions
    // Warning : real time is not ensured
    this->LidarSlam.StartRecovery(this->RecoveryTime);
  }

  // Publish SLAM output as requested by user
  // !:加上这个可视化程序就会导致程序死机，需要解决
  this->PublishOutput();

  this->Frames.clear();
}

void LidarSlamNode::ScanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  std::cout << "----ros time------" << ros::Time::now().toSec() << std::endl;
  // 这里能不能把它改成接收msg格式的话题信息，然后再在转化为CloudS::Ptr形式
  pcl::PointCloud<Point2> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  std::cout << "----point num------" <<  pl_orig.points.size() << std::endl;
  int plsize = pl_orig.points.size();

  CloudS::Ptr cloudS_ptr(new CloudS);
  cloudS_ptr->header.frame_id = "base_link";
  // 时间单位：微秒
  cout << "header.stamp " << msg->header.stamp << endl;
  cloudS_ptr->header.stamp = msg->header.stamp.toNSec() * 1e-3;
  cout << "header.stamp " << cloudS_ptr->header.stamp << endl;

  cloudS_ptr->header.seq = msg->header.seq;
  for (int i = 0; i < plsize; i++)
  {
    PointS lidar_point;
    lidar_point.x = pl_orig.points[i].x;
    lidar_point.y = pl_orig.points[i].y;
    lidar_point.z = pl_orig.points[i].z;
    //?:这里有个问题是，点云的时间戳是否需要乘上这个time_scale
    lidar_point.time = pl_orig.points[i].time * 1e-6;    //unit:s
    cout << "time-----" << pl_orig.points[i].time << endl;
    lidar_point.intensity = pl_orig.points[i].intensity;
    lidar_point.laser_id = pl_orig.points[i].ring;
    lidar_point.device_id = 0;
    lidar_point.label = 0;

    cloudS_ptr->points.push_back(lidar_point);
  }

  std::cout << "----point S num------" << cloudS_ptr->points.size() << std::endl;


  if (!this->SlamEnabled)
    return;

  if(cloudS_ptr->empty())
  {
    ROS_WARN_STREAM("Input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  this->MainLidarId = cloudS_ptr->header.frame_id;

  this->StartTime = ros::Time::now().toSec();

  // if (!this->LidarTimePosix)
  if (1)
  {
    std::cout << "time compute" << std::endl;
    // Compute time offset
    // Get ROS frame reception time
    double TimeFrameReceptionPOSIX = ros::Time::now().toSec();
    std::cout << " now time: " << ros::Time::now().toSec() << std::endl;
    // Get last acquired point timestamp
    std::cout << " msg time: " << LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp) << std::endl;
    std::cout << " back time: " << cloudS_ptr->back().time << std::endl;
    double TimeLastPoint = LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp) + cloudS_ptr->back().time;
    // double TimeLastPoint = msg->header.stamp.toSec() + cloudS_ptr->back().time;
    std::cout << " final time: " << TimeLastPoint << std::endl;
    // Compute offset
    double potentialOffset = TimeLastPoint - TimeFrameReceptionPOSIX;
    std::cout << " potentialOffset: " << potentialOffset << std::endl;
    // Get current offset
    double absCurrentOffset = std::abs(this->LidarSlam.GetSensorTimeOffset());
    std::cout << " absCurrentOffset: " << absCurrentOffset << std::endl;
    // If the current computed offset is more accurate, replace it
    if (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset) {
      this->LidarSlam.SetSensorTimeOffset(potentialOffset + this->SensorTimeOffset);
      cout << "********time compute is use" << endl;
    }

  }
  else
    this->LidarSlam.SetSensorTimeOffset(this->SensorTimeOffset);

  // Update TF from BASE to LiDAR
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id, cloudS_ptr->front().device_id))
    return;

  // Set the SLAM main input frame at first position
  this->Frames.insert(this->Frames.begin(), cloudS_ptr);

  // Run SLAM : register new frame and update localization and map.
  // ?:这里的LidarSlam是在LidarSlamNode中声明的Slam类的实例化对象
  this->LidarSlam.AddFrames(this->Frames);

  // TMP
  // Check if SLAM has failed
  if (this->LidarSlam.IsRecovery())
  {
    // TMP : in the future, the user should have a look
    // at the result to validate recovery
    // Check if the SLAM can go on and pose has to be displayed
    if (this->LidarSlam.GetOverlapEstimation() > 0.2f &&
        this->LidarSlam.GetPositionErrorStd()  < 0.1f)
    {
      ROS_WARN_STREAM("Getting out of recovery mode");
      // Frame is relocalized, reset params
      this->LidarSlam.EndRecovery();
    }
    else
      ROS_WARN_STREAM("Still waiting for recovery");
  }
  else if (this->LidarSlam.HasFailed())
  {
    ROS_WARN_STREAM("SLAM has failed : entering recovery mode :\n"
                    << "\t -Maps will not be updated\n"
                    << "\t -Egomotion and undistortion are disabled\n"
                    << "\t -The number of ICP iterations is increased\n"
                    << "\t -The maximum distance between a frame point and a map target point is increased");
    // Enable recovery mode :
    // Last frames are removed
    // Maps are not updated
    // Param are tuned to handle bigger motions
    // Warning : real time is not ensured
    this->LidarSlam.StartRecovery(this->RecoveryTime);
  }

  // Publish SLAM output as requested by user
  // !:加上这个可视化程序就会导致程序死机，需要解决
  this->PublishOutput();

  this->Frames.clear();
}

//------------------------------------------------------------------------------
void LidarSlamNode::SecondaryScanCallback(const CloudS::Ptr cloudS_ptr)
{
  if (!this->SlamEnabled)
    return;

  if (cloudS_ptr->empty())
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
void LidarSlamNode::ImageCallback(const sensor_msgs::Image& imageMsg)
{
  if (!this->SlamEnabled)
    return;

  #ifdef USE_CV_BRIDGE
  if (!this->UseExtSensor[LidarSlam::CAMERA])
    return;

  // Transform to apply to points represented in GPS frame to express them in base frame
  Eigen::Isometry3d baseToCamera;
  if (Utils::Tf2LookupTransform(baseToCamera, this->TfBuffer, this->TrackingFrameId, imageMsg.header.frame_id, imageMsg.header.stamp))
  {
    if (!this->LidarSlam.CameraCanBeUsedLocally())
      this->LidarSlam.SetCameraCalibration(baseToCamera);

    cv_bridge::CvImagePtr cvPtr;
    try
    {
      cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Camera info cannot be used -> cv_bridge exception: %s", e.what());
      return;
    }

    ROS_INFO_STREAM("Adding Camera info : "<< std::fixed << std::setprecision(9) << imageMsg.header.stamp.sec + imageMsg.header.stamp.nsec * 1e-9);
    // Add camera measurement to measurements list
    LidarSlam::ExternalSensors::Image image;
    image.Time = imageMsg.header.stamp.sec + imageMsg.header.stamp.nsec * 1e-9;
    image.Data = cvPtr->image;
    this->LidarSlam.AddCameraImage(image);
  }
  #else
  static_cast<void>(imageMsg);
  ROS_WARN_STREAM("cv_bridge was not found so images cannot be processed, camera will not be used");
  #endif
}

//------------------------------------------------------------------------------
void LidarSlamNode::CameraInfoCallback(const sensor_msgs::CameraInfo& calibMsg)
{
  if (!this->SlamEnabled)
    return;

  #ifdef USE_CV_BRIDGE
  // The intrinsic calibration must not changed so we can only use
  // the camera info until the camera is ready to be used
  if (this->LidarSlam.CameraCanBeUsedLocally())
    return;

  Eigen::Matrix3f k;
  for (int i = 0; i < 9; ++i)
    k(int(i / 3), i % 3) = calibMsg.K[i];

  this->LidarSlam.SetCameraIntrinsicCalibration(k);
  #else
  static_cast<void>(calibMsg);
  #endif
}

//------------------------------------------------------------------------------
void LidarSlamNode::ExtPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& poseMsg)
{
  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::POSE])
    return;

  // Set calibration
  Eigen::Isometry3d baseToPose;
  if(Utils::Tf2LookupTransform(baseToPose, this->TfBuffer, this->TrackingFrameId,
                               poseMsg.header.frame_id, poseMsg.header.stamp))
    this->LidarSlam.SetPoseCalibration(baseToPose);

  // Set frame ID for optional calibration
  this->ExtPoseFrameId = poseMsg.header.frame_id;

  ROS_INFO_STREAM("Adding external pose info");
  // Get external pose
  LidarSlam::ExternalSensors::PoseMeasurement poseMeas;
  poseMeas.Pose = Utils::PoseMsgToIsometry(poseMsg.pose.pose);
  // Get external pose timestamp
  poseMeas.Time = poseMsg.header.stamp.sec + poseMsg.header.stamp.nsec * 1e-9;

  // Get Pose covariance
  // ROS covariance message is row major
  // Eigen matrix is col major by default
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
      poseMeas.Covariance(i, j) = poseMsg.pose.covariance[i * 6 + j];
  }
  // Correct covariance if needed
  if (!LidarSlam::Utils::isCovarianceValid(poseMeas.Covariance))
    poseMeas.Covariance = LidarSlam::Utils::CreateDefaultCovariance(2e-2, 5. * M_PI/180.); // 2cm, 5°

  // Add pose measurement to measurements list
  this->LidarSlam.AddPoseMeasurement(poseMeas);
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::Odometry& gpsMsg)
{
  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::GPS])
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

    // Get GPS covariance
    // ROS covariance message is row major
    // Eigen matrix is col major by default
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
        this->LastGpsMeas.Covariance(i, j) = gpsMsg.pose.covariance[i * 6 + j];
    }
    // Correct GPS covariance if needed
    if (!LidarSlam::Utils::isCovarianceValid(this->LastGpsMeas.Covariance))
      this->LastGpsMeas.Covariance = Eigen::Matrix3d::Identity() * 4e-4; // 2cm

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
  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR])
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
      // ROS covariance message is row major
      // Eigen matrix is col major by default
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
          lm.Covariance(i, j) = tagInfo.pose.pose.covariance[i * 6 + j];
      }
      // Correct tag covariance if needed
      if (!LidarSlam::Utils::isCovarianceValid(lm.Covariance))
        lm.Covariance = LidarSlam::Utils::CreateDefaultCovariance(1e-2, LidarSlam::Utils::Deg2Rad(1)); // 1cm, 1°

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
std::vector<std::vector<std::string>> LidarSlamNode::ReadCSV(const std::string& path,
                                                             unsigned int nbFields,
                                                             unsigned int nbHeaderLines)
{
  // Check the file
  if (path.substr(path.find_last_of(".") + 1) != "csv")
  {
    ROS_ERROR_STREAM("The file is not CSV! Cancel loading");
    return {};
  }

  std::ifstream lmFile(path);
  if (lmFile.fail())
  {
    ROS_ERROR_STREAM("The CSV file " << path << " was not found!");
    return {};
  }

  // Check which delimiter is used
  std::string lmStr;
  std::string delimiter;
  std::vector<std::string> fields;

  // Get the first data line (after header)
  for (int i = 0; i <= nbHeaderLines; ++i)
    std::getline(lmFile, lmStr);

  for (const auto& del : DELIMITERS)
  {
    fields.clear();
    size_t pos = 0;
    std::string firstLine = lmStr;
    pos = firstLine.find(del);
    while (pos != std::string::npos)
    {
      fields.push_back(firstLine.substr(0, pos));
      firstLine.erase(0, pos + del.length());
      pos = firstLine.find(del);
    }
    // If there is some element after the last delimiter, add it
    if (!firstLine.substr(0, pos).empty())
      fields.push_back(firstLine.substr(0, pos));
    // Check that the number of fields is correct
    if (fields.size() == nbFields)
    {
      delimiter = del;
      break;
    }
  }
  if (fields.size() != nbFields)
  {
    ROS_WARN_STREAM("The CSV file is ill formed : " << fields.size() << " fields were found (" << nbFields
                    << " expected), the loading is cancelled");
    return {};
  }

  std::vector<std::vector<std::string>> lines;

  int lineIdx = nbHeaderLines;
  do
  {
    size_t pos = 0;
    std::vector<std::string> sentence;
    while ((pos = lmStr.find(delimiter)) != std::string::npos)
    {
      // Remove potential extra spaces after the delimiter
      unsigned int charIdx = 0;
      while (charIdx < lmStr.size() && lmStr[charIdx] == ' ')
        ++charIdx;
      sentence.push_back(lmStr.substr(charIdx, pos));
      lmStr.erase(0, pos + delimiter.length());
    }
    sentence.push_back(lmStr.substr(0, pos));
    if (sentence.size() != nbFields)
    {
      ROS_WARN_STREAM("data on line " + std::to_string(lineIdx) + " of the CSV file is not correct -> Skip");
      ++lineIdx;
      continue;
    }

    // Check numerical values in the studied line
    bool numericalIssue = false;
    for (std::string field : sentence)
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
      ROS_WARN_STREAM("Data on line " + std::to_string(lineIdx) + " contains a not numerical value -> Skip");
      ++lineIdx;
      continue;
    }

    lines.push_back(sentence);
    ++lineIdx;
  }
  while (std::getline(lmFile, lmStr));

  lmFile.close();
  return lines;
}

//------------------------------------------------------------------------------
std::string LidarSlamNode::ReadPoses(const std::string& path)
{
  std::vector<std::vector<std::string>> lines = this->ReadCSV(path, 13, 2);
  if (lines.empty())
  {
    ROS_ERROR_STREAM("Cannot read file :" << path << ", poses are not loaded");
    return "";
  }

  // Get frame ID
  std::ifstream lmFile(path);
  std::string frameID;
  std::getline(lmFile, frameID);
  for (const auto& del : DELIMITERS)
  {
    if (frameID.find(del) != std::string::npos)
    {
      ROS_ERROR_STREAM("Frame ID is not specified in the first line of the CSV file, stop loading");
      return "";
    }
  }

  for (auto& l : lines)
  {
    // Build pose measurement
    LidarSlam::ExternalSensors::PoseMeasurement poseMeas;
    // Time
    poseMeas.Time = std::stod(l[0]);
    // Translation
    for (int i = 0; i < 3; ++i)
      poseMeas.Pose(i, 3) = std::stof(l[i + 1]);
    // Rotation (format is col major to represent the orientation axis)
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
        poseMeas.Pose.linear()(i, j) = std::stof(l[i + j*3 + 4]);
    }
    this->LidarSlam.AddPoseMeasurement(poseMeas);
  }
  return frameID;
}

//------------------------------------------------------------------------------
void LidarSlamNode::ReadTags(const std::string& path)
{
  std::vector<std::vector<std::string>> lines = this->ReadCSV(path, 43, 1);
  for (auto& l : lines)
  {
    // Build measurement
    // Set landmark id
    int id = std::stoi(l[0]);
    // Fill pose
    Eigen::Vector6d absolutePose;
    absolutePose << std::stof(l[1]), std::stof(l[2]), std::stof(l[3]), std::stof(l[4]), std::stof(l[5]), std::stof(l[6]);
    // Fill covariance
    Eigen::Matrix6d absolutePoseCovariance;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        absolutePoseCovariance(i, j) = std::stof(l[7 + 6 * i + j]);
    }
    // Add a new landmark manager for absolute constraint computing
    this->LidarSlam.AddLandmarkManager(id, absolutePose, absolutePoseCovariance);
    ROS_INFO_STREAM("Tag #" << id << " initialized to \n" << absolutePose.transpose());
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  // Get offset between Lidar SLAM ref frame (odom)
  // and pose message reference frame (msg.header.frame_id)
  Eigen::Isometry3d offset;
  if (Utils::Tf2LookupTransform(offset, this->TfBuffer, this->OdometryFrameId, msg.header.frame_id, msg.header.stamp))
  {
    // Compute pose in odometry frame and set SLAM pose
    Eigen::Isometry3d poseInOdom = offset * Utils::PoseMsgToIsometry(msg.pose.pose);
    this->LidarSlam.SetTworld(poseInOdom);
    ROS_WARN_STREAM("SLAM pose set to :\n" << poseInOdom.matrix());
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
      if (!this->UseExtSensor[LidarSlam::GPS] || !this->LidarSlam.GpsHasData())
      {
        ROS_ERROR_STREAM("Cannot set SLAM pose from GPS"
                         "Please check that 'external_sensors/gps/enable' private parameter is set to 'true'."
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
      if (!this->UseExtSensor[LidarSlam::GPS] || !this->LidarSlam.GpsHasData())
      {
        ROS_ERROR_STREAM("Cannot set SLAM pose from GPS"
                          "Please check that 'external_sensors/gps/enable' private parameter is set to 'true'."
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
      this->LidarSlam.SetTworld(pose);
      ROS_WARN_STREAM("SLAM pose set from GPS pose to :\n" << pose.matrix());
      break;
    }

    // Disable SLAM maps update
    case lidar_slam::SlamCommand::DISABLE_SLAM_MAP_UPDATE:
    {
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::NONE);
      ROS_WARN_STREAM("Disabling SLAM maps update.");
      break;
    }

    // Enable the agregation of keypoints to a fixed initial map
    case lidar_slam::SlamCommand::ENABLE_SLAM_MAP_EXPANSION:
    {
      if (this->LidarSlam.IsRecovery())
        ROS_ERROR_STREAM("Cannot unable map expansion in recovery mode!");
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP);
      ROS_WARN_STREAM("Enabling SLAM maps expansion with new keypoints.");
      break;
    }

    // Enable the update of the map with new keypoints
    case lidar_slam::SlamCommand::ENABLE_SLAM_MAP_UPDATE:
    {
      if (this->LidarSlam.IsRecovery())
        ROS_ERROR_STREAM("Cannot unable map update in recovery mode!");
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::UPDATE);
      ROS_WARN_STREAM("Enabling SLAM maps update with new keypoints.");
      break;
    }

    // Reset the SLAM internal state.
    case lidar_slam::SlamCommand::RESET_SLAM:
    {
      ROS_WARN_STREAM("Resetting the SLAM internal state.");
      this->LidarSlam.Reset(true);
      this->SetSlamInitialState();
      break;
    }

    // Enable/Disable the SLAM process
    case lidar_slam::SlamCommand::SWITCH_ON_OFF:
    {
      if (this->SlamEnabled)
        ROS_WARN_STREAM("Disabling the SLAM process");
      else
        ROS_WARN_STREAM("Enabling again the SLAM process");

      this->SlamEnabled = !this->SlamEnabled;
      break;
    }

    // Save current trajectory tracking base frame
    case lidar_slam::SlamCommand::SAVE_TRAJECTORY:
    {
      if (msg.string_arg.empty())
      {
        ROS_ERROR_STREAM("No path is specified, the trajectory cannot be saved");
        return;
      }
      std::list<LidarSlam::LidarState> states = this->LidarSlam.GetLogStates();
      if (states.empty())
      {
        ROS_WARN_STREAM("No current trajectory logged, nothing will be saved");
        return;
      }
      ROS_INFO_STREAM("Saving current trajectory of base frame as " << msg.string_arg);
      std::ofstream fout(msg.string_arg);
      fout << this->TrackingFrameId << "\n";
      fout << "t,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2\n";
      for (auto& s : states)
        fout << s;
      fout.close();
      break;
    }

    // Save current trajectory tracking Lidar
    case lidar_slam::SlamCommand::SAVE_LIDAR_TRAJECTORY:
    {
      if (msg.string_arg.empty())
      {
        ROS_ERROR_STREAM("No path is specified, the trajectory cannot be saved");
        return;
      }
      Eigen::Isometry3d baseToLidar;
      if (!Utils::Tf2LookupTransform(baseToLidar, this->TfBuffer, this->TrackingFrameId, this->MainLidarId))
      {
        ROS_ERROR_STREAM("No transform from base to Lidar : cannot save Lidar trajectory.");
        return;
      }
      std::list<LidarSlam::LidarState> states = this->LidarSlam.GetLogStates();
      if (states.empty())
      {
        ROS_WARN_STREAM("No current trajectory logged, nothing will be saved");
        return;
      }
      ROS_INFO_STREAM("Saving current trajectory of the Lidar sensor as " << msg.string_arg);
      std::ofstream fout(msg.string_arg);
      fout << this->MainLidarId << "\n";
      fout << "t,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2\n";
      for (auto& s : states)
      {
        s.Isometry = s.Isometry * baseToLidar;
        fout << s;
      }
      fout.close();
      break;
    }
    // Save SLAM keypoints maps to PCD files
    case lidar_slam::SlamCommand::SAVE_KEYPOINTS_MAPS:
    {
      ROS_INFO_STREAM("Saving keypoint maps as PCD files in " << msg.string_arg);
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
    {
      ROS_INFO_STREAM("Loading keypoints maps from PCD.");
      this->LidarSlam.LoadMapsFromPCD(msg.string_arg);
      break;
    }

    case lidar_slam::SlamCommand::OPTIMIZE_GRAPH:
    {
      if ((!this->UseExtSensor[LidarSlam::GPS] &&
           !this->UseExtSensor[LidarSlam::LANDMARK_DETECTOR] &&
           !this->UseExtSensor[LidarSlam::POSE]) ||
           this->LidarSlam.GetSensorMaxMeasures() < 2 || this->LidarSlam.GetLoggingTimeout() < 0.2)
      {
        ROS_ERROR_STREAM("Cannot optimize pose graph as sensor info logging has not been enabled. "
                         "Please make sure that one external sensor has been enabled "
                         "and that the external sensor and 'slam/logging/timeout'"
                         "private parameters are set to convenient values.");
        break;
      }

      if (this->LidarSlam.LmHasData())
      {
        if (!msg.string_arg.empty())
        {
          ROS_INFO_STREAM("Loading the absolute landmark poses");
          this->ReadTags(msg.string_arg);
        }
        else if (this->LidarSlam.GetLandmarkConstraintLocal())
          ROS_WARN_STREAM("No absolute landmark poses are supplied : the last estimated poses will be used");
      }

      ROS_INFO_STREAM("Optimizing the pose graph");
      if (!this->LidarSlam.OptimizeGraph())
        break;
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
    }

    case lidar_slam::SlamCommand::SWITCH_SENSOR:
    {
      // Get sensor to enable/disable
      LidarSlam::ExternalSensor sensor;
      try
      {
        sensor = static_cast<LidarSlam::ExternalSensor>(std::stoi(msg.string_arg));
      }
      catch(...)
      {
        ROS_WARN_STREAM("External sensor #" << msg.string_arg << " does not exist.");
        break;
      }

      std::string onOff;
      onOff= this->UseExtSensor[sensor]? "Disabling " : "Enabling ";
      ROS_INFO_STREAM(onOff << LidarSlam::ExternalSensorNames.at(sensor));
      this->UseExtSensor[sensor] = !this->UseExtSensor[sensor];
      break;
    }

    case lidar_slam::SlamCommand::CALIBRATE_WITH_POSES:
    {
      // If an input file is provided, load the poses
      if (!msg.string_arg.empty())
      {
        // Clear current pose manager
        this->LidarSlam.ResetSensor(true, LidarSlam::ExternalSensor::POSE);
        // Fill external pose manager with poses from a CSV file
        this->ExtPoseFrameId = this->ReadPoses(msg.string_arg);
        if (this->ExtPoseFrameId.empty())
          return;
      }
      else if (!this->LidarSlam.PoseHasData())
      {
        ROS_ERROR_STREAM("No file provided and no poses registered, "
                         << "the calibration cannot be performed");
        break;
      }
      // Calibrate the external poses with current SLAM trajectory
      this->LidarSlam.CalibrateWithExtPoses(this->PlanarTrajectory);
      // Get the calibration
      Eigen::Isometry3d calibration = this->LidarSlam.GetPoseCalibration();

      //Publish new static TF
      geometry_msgs::TransformStamped tfStamped;
      tfStamped.header.stamp = ros::Time(ros::Time::now().toSec());
      tfStamped.header.frame_id = this->OdometryFrameId;
      tfStamped.child_frame_id = this->ExtPoseFrameId;
      tfStamped.transform = Utils::IsometryToTfMsg(calibration);
      this->StaticTfBroadcaster.sendTransform(tfStamped);

      ROS_INFO_STREAM("Calibration estimated to :\n" << calibration.matrix());
      // Clean the pose manager in the SLAM
      this->LidarSlam.ResetSensor(true, LidarSlam::ExternalSensor::POSE);
      break;
    }

    // Unknown command
    default:
    {
      ROS_ERROR_STREAM("Unknown SLAM command : " << (unsigned int) msg.command);
      break;
    }
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
  // Get current SLAM poses in WORLD coordinates at the specified frequency
  std::vector<LidarSlam::LidarState> lastStates = this->LidarSlam.GetLastStates(this->TrajFrequency);
  double computationTime = (ros::Time::now().toSec() - this->StartTime);
  // Publish SLAM pose
  if (this->Publish[POSE_ODOM] || this->Publish[POSE_TF])
  {
    for (const auto& state : lastStates)
    {
      // Publish as odometry msg
      if (this->Publish[POSE_ODOM])
      {
        nav_msgs::Odometry odomMsg;
        odomMsg.header.stamp = ros::Time(state.Time);
        odomMsg.header.frame_id = this->OdometryFrameId;
        odomMsg.child_frame_id = this->TrackingFrameId;
        odomMsg.pose.pose = Utils::IsometryToPoseMsg(state.Isometry);
        // Note : in eigen 3.4 iterators are available on matrices directly
        //        >> std::copy(state.Covariance.begin(), state.Covariance.end(), confidenceMsg.covariance.begin());
        // For now the only way is to copy or iterate on indices :
        for (unsigned int i = 0; i < state.Covariance.size(); ++i)
          odomMsg.pose.covariance[i] = state.Covariance(i);

        this->Publishers[POSE_ODOM].publish(odomMsg);
      }

      // Publish as TF from OdometryFrameId to TrackingFrameId
      if (this->Publish[POSE_TF])
      {
        geometry_msgs::TransformStamped tfMsg;
        tfMsg.header.stamp = ros::Time(state.Time);
        tfMsg.header.frame_id = this->OdometryFrameId;
        tfMsg.child_frame_id = this->TrackingFrameId;
        tfMsg.transform = Utils::IsometryToTfMsg(state.Isometry);
        this->TfBroadcaster.sendTransform(tfMsg);
      }

      // Enable subscribers to receive those messages
      // Warning : this may alter cout working in this code area
      ros::Duration(1e-6).sleep();
    }
  }

  // Publish latency compensated SLAM pose
  if (this->Publish[POSE_PREDICTION_ODOM] || this->Publish[POSE_PREDICTION_TF])
  {
    double predTime = lastStates.back().Time + computationTime;
    Eigen::Isometry3d predIsometry = this->LidarSlam.GetTworld(predTime);

    // Publish as odometry msg
    if (this->Publish[POSE_PREDICTION_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(predTime);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      odomMsg.pose.pose = Utils::IsometryToPoseMsg(predIsometry);
      for (unsigned int i = 0; i < lastStates.back().Covariance.size(); ++i)
        odomMsg.pose.covariance[i] = lastStates.back().Covariance(i);
      this->Publishers[POSE_PREDICTION_ODOM].publish(odomMsg);
    }

    // Publish as TF from OdometryFrameId to <TrackingFrameId>_prediction
    if (this->Publish[POSE_PREDICTION_TF])
    {
      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.stamp = ros::Time(predTime);
      tfMsg.header.frame_id = this->OdometryFrameId;
      tfMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      tfMsg.transform = Utils::IsometryToTfMsg(predIsometry);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }

  // Publish a pointcloud only if required and if someone is listening to it to spare bandwidth.
  #define publishPointCloud(publisher, pc)                                            \
    if (this->Publish[publisher] && this->Publishers[publisher].getNumSubscribers())  \
      this->Publishers[publisher].publish(pc);

  // Keypoints maps
  publishPointCloud(EDGES_MAP,  this->LidarSlam.GetMap(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGES_MAP,  this->LidarSlam.GetMap(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANES_MAP, this->LidarSlam.GetMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_MAP,  this->LidarSlam.GetMap(LidarSlam::BLOB));

  // Keypoints submaps
  publishPointCloud(EDGES_SUBMAP,  this->LidarSlam.GetTargetSubMap(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGES_SUBMAP,  this->LidarSlam.GetTargetSubMap(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANES_SUBMAP, this->LidarSlam.GetTargetSubMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_SUBMAP,  this->LidarSlam.GetTargetSubMap(LidarSlam::BLOB));

  // Current keypoints
  publishPointCloud(EDGE_KEYPOINTS,  this->LidarSlam.GetKeypoints(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGE_KEYPOINTS,  this->LidarSlam.GetKeypoints(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANE_KEYPOINTS, this->LidarSlam.GetKeypoints(LidarSlam::PLANE));
  publishPointCloud(BLOB_KEYPOINTS,  this->LidarSlam.GetKeypoints(LidarSlam::BLOB));

  // Registered aggregated (and optionally undistorted) input scans points
  publishPointCloud(SLAM_REGISTERED_POINTS, this->LidarSlam.GetRegisteredFrame());

  // Overlap estimation
  if (this->Publish[CONFIDENCE])
  {
    // Get SLAM pose
    lidar_slam::Confidence confidenceMsg;
    confidenceMsg.header.stamp = ros::Time(lastStates.back().Time);
    confidenceMsg.header.frame_id = this->OdometryFrameId;
    confidenceMsg.overlap = this->LidarSlam.GetOverlapEstimation();
    confidenceMsg.computation_time = computationTime;
    // Note : in eigen 3.4, iterators are available on matrices directly
    //        >> std::copy(lastStates.back().Covariance.begin(), lastStates.back().Covariance.end(), confidenceMsg.covariance.begin());
    for (unsigned int i = 0; i < lastStates.back().Covariance.size(); ++i)
      confidenceMsg.covariance[i] = lastStates.back().Covariance(i);
    confidenceMsg.nb_matches = this->LidarSlam.GetTotalMatchedKeypoints();
    confidenceMsg.comply_motion_limits = this->LidarSlam.GetComplyMotionLimits();
    confidenceMsg.std_position_error = this->LidarSlam.GetPositionErrorStd();
    confidenceMsg.failure = this->LidarSlam.HasFailed() || this->LidarSlam.IsRecovery();
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
  int interpolationModel;
  if (this->PrivNh.getParam("slam/interpolation_model", interpolationModel))
  {
    if (interpolationModel != LidarSlam::Interpolation::Model::LINEAR &&
        interpolationModel != LidarSlam::Interpolation::Model::QUADRATIC &&
        interpolationModel != LidarSlam::Interpolation::Model::CUBIC)
    {
      ROS_ERROR_STREAM("Invalid interpolation model (" << interpolationModel << "). Setting it to 'LINEAR'.");
      interpolationModel = LidarSlam::Interpolation::Model::LINEAR;
    }
    LidarSlam.SetInterpolation(static_cast<LidarSlam::Interpolation::Model>(interpolationModel));
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
    SetKeypointsExtractorParam(int,   prefix + "neighbors_side_nb", MinNeighNb)
    SetKeypointsExtractorParam(float, prefix + "neighbors_radius", MinNeighRadius)
    SetKeypointsExtractorParam(float, prefix + "min_distance_to_sensor", MinDistanceToSensor)
    SetKeypointsExtractorParam(float, prefix + "max_distance_to_sensor", MaxDistanceToSensor)
    SetKeypointsExtractorParam(float, prefix + "min_beam_surface_angle", MinBeamSurfaceAngle)
    SetKeypointsExtractorParam(float, prefix + "min_azimuth", AzimuthMin)
    SetKeypointsExtractorParam(float, prefix + "max_azimuth", AzimuthMax)
    SetKeypointsExtractorParam(float, prefix + "plane_sin_angle_threshold", PlaneSinAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_sin_angle_threshold", EdgeSinAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_depth_gap_threshold", EdgeDepthGapThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_nb_gap_points", EdgeNbGapPoints)
    SetKeypointsExtractorParam(float, prefix + "edge_intensity_gap_threshold", EdgeIntensityGapThreshold)
    SetKeypointsExtractorParam(int,   prefix + "max_points", MaxPoints)
    SetKeypointsExtractorParam(float, prefix + "voxel_grid_resolution", VoxelResolution)
    SetKeypointsExtractorParam(float, prefix + "input_sampling_ratio", InputSamplingRatio)
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
    EnableKeypoint(LidarSlam::Keypoint::INTENSITY_EDGE);
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
  this->LidarTimePosix = this->PrivNh.param("external_sensors/lidar_is_posix", false);
  SetSlamParam(float,   "external_sensors/time_offset", SensorTimeOffset)
  this->SensorTimeOffset = this->LidarSlam.GetSensorTimeOffset();
  SetSlamParam(float,  "external_sensors/landmark_detector/weight", LandmarkWeight)
  SetSlamParam(float,  "external_sensors/landmark_detector/saturation_distance", LandmarkSaturationDistance)
  SetSlamParam(bool,   "external_sensors/landmark_detector/position_only", LandmarkPositionOnly)
  this->PublishTags    = this->PrivNh.param("external_sensors/landmark_detector/publish_tags", false);
  SetSlamParam(float,   "external_sensors/camera/weight", CameraWeight)
  SetSlamParam(float,   "external_sensors/camera/saturation_distance", CameraSaturationDistance)
  this->PlanarTrajectory = this->PrivNh.param("external_sensors/calibration/planar_trajectory", this->PlanarTrajectory);

  // Graph parameters
  SetSlamParam(std::string, "graph/g2o_file_name", G2oFileName)
  SetSlamParam(float,       "graph/covariance_scale", CovarianceScale)
  SetSlamParam(int,         "graph/iterations_nb", NbGraphIterations)

  // Confidence estimators
  // Overlap
  SetSlamParam(float,  "slam/confidence/overlap/sampling_ratio", OverlapSamplingRatio)
  // Motion limitations (hard constraints to detect failure)
  std::vector<float> acc;
  if (this->PrivNh.getParam("slam/confidence/motion_limits/acceleration", acc) && acc.size() == 2)
  {
    Eigen::Array2f acc_array = Eigen::Array2f(acc.data());
    this->LidarSlam.SetAccelerationLimits(acc_array);
  }
  std::vector<float> vel;
  if (this->PrivNh.getParam("slam/confidence/motion_limits/velocity", vel) && vel.size() == 2)
  {
    Eigen::Array2f vel_array = Eigen::Array2f(vel.data());
    this->LidarSlam.SetVelocityLimits(vel_array);
  }
  std::vector<float> pos;
  if (this->PrivNh.getParam("slam/confidence/motion_limits/pose", pos) && pos.size() == 2)
  {
    Eigen::Array2f pos_array = Eigen::Array2f(pos.data());
    this->LidarSlam.SetPoseLimits(pos_array);
  }

  SetSlamParam(int, "slam/confidence/window", ConfidenceWindow)
  SetSlamParam(float, "slam/confidence/overlap/gap_threshold", OverlapDerivativeThreshold)
  SetSlamParam(float, "slam/confidence/position_error/threshold", PositionErrorThreshold)
  this->RecoveryTime = this->PrivNh.param("slam/confidence/failure_detector/recovery_time", this->RecoveryTime);
  SetSlamParam(bool,  "slam/confidence/failure_detector/enable", FailureDetectionEnabled)

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
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size/edges", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::EDGE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::EDGE, size);
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size/intensity_edges", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::INTENSITY_EDGE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::INTENSITY_EDGE, size);
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size/planes", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::PLANE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::PLANE, size);
  if (this->PrivNh.getParam("slam/voxel_grid/leaf_size/blobs", size) && this->LidarSlam.KeypointTypeEnabled(LidarSlam::BLOB))
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
void LidarSlamNode::SetSlamInitialState()
{
  // Load initial SLAM maps if requested
  std::string mapsPathPrefix = this->PrivNh.param<std::string>("maps/initial_maps", "");
  if (!mapsPathPrefix.empty())
  {
    ROS_INFO_STREAM("Loading initial keypoints maps from PCD.");
    this->LidarSlam.LoadMapsFromPCD(mapsPathPrefix);
  }

  // Load initial Landmarks poses if requested
  std::string lmpath =
    this->PrivNh.param<std::string>("external_sensors/landmark_detector/landmarks_file_path", "");
  if (!lmpath.empty())
  {
    ROS_INFO_STREAM("Loading initial landmarks info from CSV.");
    this->ReadTags(lmpath);
    this->LidarSlam.SetLandmarkConstraintLocal(false);
  }
  else
    this->LidarSlam.SetLandmarkConstraintLocal(true);

  // Set initial SLAM pose if requested
  // Setting initial SLAM pose is equivalent to move odom frame
  // so the first pose corresponds to the input in this new frame
  // T_base = offset * T_base_new
  // offset = T_base * T_base_new^-1
  // T_base is identity at initialization
  std::vector<double> initialPose;
  if (this->PrivNh.getParam("maps/initial_pose", initialPose) && initialPose.size() == 6)
  {
    Eigen::Isometry3d initialTransform = LidarSlam::Utils::XYZRPYtoIsometry(initialPose.data());
    this->LidarSlam.TransformOdom(initialTransform.inverse());
    ROS_INFO_STREAM("Setting initial SLAM pose to:\n" << initialTransform.matrix());
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
