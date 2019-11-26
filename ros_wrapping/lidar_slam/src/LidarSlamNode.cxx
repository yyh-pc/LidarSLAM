#include "LidarSlamNode.h"
#include "GlobalTrajectoriesRegistration.h"

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Get SLAM params
  this->SetSlamParameters(priv_nh);

  // ***************************************************************************
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
    for (int i = 0; i < nLasers; i++)
      this->LaserIdMapping[i] = i;
    ROS_INFO_STREAM("[SLAM] Using 0->" << nLasers << " linear laser_id_mapping from ROS param.");
  }
  // Otherwise, n_lasers will be guessed from 1st frame
  else
  {
    ROS_WARN_STREAM("[SLAM] No laser_id_mapping nor n_lasers params found : "
                    "n_lasers will be guessed from 1st frame to build linear mapping.");
  }

  // ***************************************************************************
  // Get LiDAR frequency
  priv_nh.getParam("lidar_frequency", this->LidarFreq);

  // Get verbose mode
  priv_nh.getParam("verbose", this->Verbose);
  this->LidarSlam.SetVerbose(this->Verbose);

  // Get frame IDs
  priv_nh.getParam("slam_origin_frame", this->SlamOriginFrameId);
  priv_nh.getParam("slam_output_frame", this->SlamOutputFrameId);

  // ***************************************************************************
  // Init optionnal publication of slam pose centered on GPS antenna instead of LiDAR sensor
  priv_nh.getParam("gps/output_gps_pose", this->OutputGpsPose);
  priv_nh.getParam("gps/output_gps_pose_frame_id", this->OutputGpsPoseFrameId);
  std::vector<double> GpsToLidarOffset;
  priv_nh.getParam("gps/gps_to_lidar_offset", GpsToLidarOffset);
  Transform GpsToLidarTransform(0., GpsToLidarOffset[0], GpsToLidarOffset[1], GpsToLidarOffset[2],
                                    GpsToLidarOffset[3], GpsToLidarOffset[4], GpsToLidarOffset[5]);
  this->LidarToGpsOffset = GpsToLidarTransform.GetIsometry().inverse();

  // Init optionnal use of GPS data to calibrate output SLAM pose to world coordinates.
  priv_nh.getParam("gps/calibration/enable", this->CalibrateSlamGps);
  if (this->CalibrateSlamGps)
  {
    this->GpsOdomSub = nh.subscribe("gps_odom", 1, &LidarSlamNode::GpsCallback, this);
    this->GpsSlamCalibrationSub = nh.subscribe("run_gps_slam_calibration", 1, &LidarSlamNode::RunGpsSlamCalibrationCallback, this);
  }
  priv_nh.getParam("gps/calibration/pose_timeout", this->CalibrationPoseTimeout);
  priv_nh.getParam("gps/calibration/no_roll", this->CalibrationNoRoll);

  // ***************************************************************************
  // Init debug publishers
  priv_nh.getParam("publish_features_maps/edges", this->PublishEdges);
  priv_nh.getParam("publish_features_maps/planars", this->PublishPlanars);
  priv_nh.getParam("publish_features_maps/blobs", this->PublishBlobs);
  priv_nh.getParam("gps/calibration/publish_icp_trajectories", this->PublishIcpTrajectories);
  if (this->PublishIcpTrajectories)
  {
    this->GpsPathPub = nh.advertise<nav_msgs::Path>("icp_gps", 1, true);
    this->SlamPathPub = nh.advertise<nav_msgs::Path>("icp_slam", 1, true);
  }
  if (this->PublishEdges)
    this->EdgesPub = nh.advertise<CloudS>("edges_features", 1);
  if (this->PublishPlanars)
    this->PlanarsPub = nh.advertise<CloudS>("planars_features", 1);
  if (this->PublishBlobs)
    this->BlobsPub = nh.advertise<CloudS>("blobs_features", 1);
  this->DebugCloudPub = nh.advertise<CloudS>("debug_cloud", 1);

  // ***************************************************************************
  // Init ROS subscribers and publishers
  this->PoseCovarPub = nh.advertise<nav_msgs::Odometry>("slam_odom", 1);
  this->CloudSub = nh.subscribe("velodyne_points", 1, &LidarSlamNode::ScanCallback, this);

  ROS_INFO_STREAM("\033[1;32mLiDAR SLAM is ready !\033[0m");
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallback(const CloudV& cloudV)
{
  // Init this->LaserIdMapping if not already done
  if (!this->LaserIdMapping.size())
  {
    // Iterate through pointcloud to find max ring
    int nLasers = 0;
    for(const PointV& point : cloudV)
    {
      if (point.ring > nLasers)
        nLasers = point.ring;
    }
    ++nLasers;

    // Init this->LaserIdMapping with linear mapping
    this->LaserIdMapping.resize(nLasers);
    for (int i = 0; i < nLasers; i++)
      this->LaserIdMapping[i] = i;

    ROS_INFO_STREAM("[SLAM] Using 0->" << nLasers << " linear laser_id_mapping.");
  }

  // Convert pointcloud PointV type to expected PointS type
  CloudS::Ptr cloudS = this->ConvertToSlamPointCloud(cloudV);

  // Run SLAM : register new frame and update position and mapping.
  this->LidarSlam.AddFrame(cloudS, this->LaserIdMapping);

  // Get the computed world transform so far
  Transform slamToLidar = this->LidarSlam.GetWorldTransform();
  slamToLidar.time = pcl_conversions::fromPCL(cloudV.header).stamp.toSec();
  std::vector<double> poseCovar = this->LidarSlam.GetTransformCovariance();

  // Publish TF, pose and covariance
  this->PublishTfOdom(cloudV.header, slamToLidar, poseCovar);

  // Publish optional debug info
  this->PublishFeaturesMaps(cloudS);

  // If GPS/SLAM calibration is needed, save SLAM pose for later use
  if (this->CalibrateSlamGps)
  {
    // Add new pose to buffer
    slamToLidar.time = pcl_conversions::fromPCL(cloudV.header).stamp.toSec();
    this->SlamPoses.push_back(slamToLidar);
    // Forget all previous poses older than CalibrationPoseTimeout
    while (this->SlamPoses.front().time < slamToLidar.time - this->CalibrationPoseTimeout)
      this->SlamPoses.pop_front();
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::Odometry& msg)
{
  // If GPS/SLAM calibration is needed, save GPS pose for later use
  if (this->CalibrateSlamGps)
  {
    // Add new pose to buffer
    this->GpsOriginFrameId = msg.header.frame_id;
    double time = msg.header.stamp.toSec();
    Eigen::Translation3d trans(msg.pose.pose.position.x,
                               msg.pose.pose.position.y,
                               msg.pose.pose.position.z);
    Eigen::Quaterniond rot(msg.pose.pose.orientation.w,
                           msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z);
    this->GpsPoses.push_back(Transform(time, trans, rot));

    // Forget all previous poses older than CalibrationPoseTimeout
    while (this->GpsPoses.front().time < time - this->CalibrationPoseTimeout)
      this->GpsPoses.pop_front();
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::RunGpsSlamCalibrationCallback(const std_msgs::Empty&)
{
  if (this->CalibrateSlamGps)
  {
    if (this->GpsPoses.size() >= 2 && this->SlamPoses.size() >= 2)
    {
      ROS_INFO_STREAM("Running SLAM/GPS calibration with " << this->SlamPoses.size() << " slam points and "
                                                           << this->GpsPoses.size() << " gps points.");
      // If we have enough GPS and SLAM points, run calibration
      // TODO : run calibration in separated thread
      this->GpsSlamCalibration();
    }
    else
      ROS_ERROR_STREAM("Not enough points to run SLAM/GPS calibration "
                       "(only got " << this->SlamPoses.size() << " slam points "
                       "and " << this->GpsPoses.size() << " gps points).");
  }
  else
  {
    ROS_ERROR_STREAM("Cannot run GPS/SLAM calibration as it has not been enabled. "
                     "Please set 'gps_calibration/enable' private parameter to 'true'.");
  }
}

//------------------------------------------------------------------------------
LidarSlamNode::CloudS::Ptr LidarSlamNode::ConvertToSlamPointCloud(const CloudV& cloudV)
{
  // Init SLAM pointcloud
  CloudS::Ptr cloudS(new CloudS);
  cloudS->resize(cloudV.size());
  cloudS->header = cloudV.header;
  if (!this->SlamOutputFrameId.empty())
    cloudS->header.frame_id = this->SlamOutputFrameId;

  // Get approximate timestamp of the first point
  double stampInit = pcl_conversions::fromPCL(cloudV.header).stamp.toSec();  // timestamp of last Velodyne raw packet
  stampInit -= 1. / this->LidarFreq;  // approximate timestamp of first Velodyne raw packet

  // Build SLAM pointcloud
  for(unsigned int i = 0; i < cloudV.size(); i++)
  {
    const PointV& velodynePoint = cloudV[i];
    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laserId = velodynePoint.ring;
    double frameAdvancement = (M_PI + std::atan2(velodynePoint.y, velodynePoint.x)) / (M_PI * 2);
    slamPoint.time = stampInit + frameAdvancement / this->LidarFreq;
    // slamPoint.time = frameAdvancement;
    cloudS->at(i) = slamPoint;
  }
  return cloudS;
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishTfOdom(const pcl::PCLHeader& headerCloudV,
                                  const Transform& slamToLidar,
                                  const std::vector<double>& poseCovar)
{
  // publish TF from SlamOriginFrameId to PointCloud frame_id (raw SLAM output)
  geometry_msgs::TransformStamped tfMsg;
  tfMsg.header = pcl_conversions::fromPCL(headerCloudV);
  tfMsg.header.frame_id = this->SlamOriginFrameId;
  tfMsg.child_frame_id = this->SlamOutputFrameId.empty() ? headerCloudV.frame_id : this->SlamOutputFrameId;
  tfMsg.transform.translation.x = slamToLidar.x();
  tfMsg.transform.translation.y = slamToLidar.y();
  tfMsg.transform.translation.z = slamToLidar.z();
  Eigen::Quaterniond q = slamToLidar.GetRotation();
  tfMsg.transform.rotation.x = q.x();
  tfMsg.transform.rotation.y = q.y();
  tfMsg.transform.rotation.z = q.z();
  tfMsg.transform.rotation.w = q.w();
  this->TfBroadcaster.sendTransform(tfMsg);

  Transform slamPose = slamToLidar;

  // If we need to output GPS antenna pose instead of LiDAR's, transform LiDAR pose and covariance
  if (this->OutputGpsPose)
  {
    // Publish once TF from LiDAR to GPS antenna
    if (this->LidarSlam.GetNbrFrameProcessed() == 1)
    {
      geometry_msgs::TransformStamped TfLidarToSlam;
      TfLidarToSlam.header = tfMsg.header;
      TfLidarToSlam.header.frame_id = tfMsg.child_frame_id;
      TfLidarToSlam.child_frame_id = this->OutputGpsPoseFrameId;
      Eigen::Translation3d trans(this->LidarToGpsOffset.translation());
      TfLidarToSlam.transform.translation.x = trans.x();
      TfLidarToSlam.transform.translation.y = trans.y();
      TfLidarToSlam.transform.translation.z = trans.z();
      Eigen::Quaterniond rot(this->LidarToGpsOffset.linear());
      TfLidarToSlam.transform.rotation.x = rot.x();
      TfLidarToSlam.transform.rotation.y = rot.y();
      TfLidarToSlam.transform.rotation.z = rot.z();
      TfLidarToSlam.transform.rotation.w = rot.w();
      this->StaticTfBroadcaster.sendTransform(TfLidarToSlam);
    }

    // Transform pose
    Eigen::Isometry3d slamToGpsPose(slamToLidar.GetIsometry() * this->LidarToGpsOffset);
    slamPose = Transform(slamToLidar.time, slamToGpsPose);

    // TODO Transform covariance to correct lever arm induced by LidarToGpsOffset
  }

  // publish pose with covariance
  nav_msgs::Odometry odomMsg;
  odomMsg.header = tfMsg.header;
  odomMsg.child_frame_id = this->OutputGpsPose ? this->OutputGpsPoseFrameId : tfMsg.child_frame_id;
  odomMsg.pose.pose.position.x = slamPose.x();
  odomMsg.pose.pose.position.y = slamPose.y();
  odomMsg.pose.pose.position.z = slamPose.z();
  q = slamPose.GetRotation();
  odomMsg.pose.pose.orientation.x = q.x();
  odomMsg.pose.pose.orientation.y = q.y();
  odomMsg.pose.pose.orientation.z = q.z();
  odomMsg.pose.pose.orientation.w = q.w();
  // Reshape covariance from parameters (rX, rY, rZ, X, Y, Z) to (X, Y, Z, rX, rY, rZ)
  const std::vector<double>& c = poseCovar;
  odomMsg.pose.covariance = {c[21], c[22], c[23],   c[18], c[19], c[20],
                             c[27], c[28], c[29],   c[24], c[25], c[26],
                             c[33], c[34], c[35],   c[30], c[31], c[32],

                             c[ 3], c[ 4], c[ 5],   c[ 0], c[ 1], c[ 2],
                             c[ 9], c[10], c[11],   c[ 6], c[ 7], c[ 8],
                             c[15], c[16], c[17],   c[12], c[13], c[14]};
  this->PoseCovarPub.publish(odomMsg);
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishFeaturesMaps(const CloudS::Ptr& cloudS)
{
  // Publish pointcloud only if someone is listening to it to spare bandwidth.
  if (this->DebugCloudPub.getNumSubscribers())
    this->DebugCloudPub.publish(cloudS);

  pcl::PCLHeader msgHeader = cloudS->header;
  msgHeader.frame_id = this->SlamOriginFrameId;

  // Publish edges only if recquired and if someone is listening to it.
  if (this->PublishEdges && this->EdgesPub.getNumSubscribers())
  {
    CloudS::Ptr edgesCloud = this->LidarSlam.GetEdgesMap();
    edgesCloud->header = msgHeader;
    this->EdgesPub.publish(edgesCloud);
  }

  // Publish planars only if recquired and if someone is listening to it.
  if (this->PublishPlanars && this->PlanarsPub.getNumSubscribers())
  {
    CloudS::Ptr planarsCloud = this->LidarSlam.GetPlanarsMap();
    planarsCloud->header = msgHeader;
    this->PlanarsPub.publish(planarsCloud);
  }

  // Publish blobs only if recquired and if someone is listening to it.
  if (this->PublishBlobs && this->BlobsPub.getNumSubscribers())
  {
    CloudS::Ptr blobsCloud = this->LidarSlam.GetBlobsMap();
    blobsCloud->header = msgHeader;
    this->BlobsPub.publish(blobsCloud);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters(ros::NodeHandle& priv_nh)
{
  // common
  bool fastSlam, undistortion;
  double maxDistanceForICPMatching;
  if (priv_nh.getParam("slam/fast_slam", fastSlam))
    LidarSlam.SetFastSlam(fastSlam);
  if (priv_nh.getParam("slam/undistortion", undistortion))
    LidarSlam.SetUndistortion(undistortion);
  if (priv_nh.getParam("slam/max_distance_for_ICP_matching", maxDistanceForICPMatching))
    LidarSlam.SetMaxDistanceForICPMatching(maxDistanceForICPMatching);

  // ego motion
  int egoMotionLMMaxIter, egoMotionICPMaxIter, egoMotionLineDistanceNbrNeighbors, egoMotionMinimumLineNeighborRejection, egoMotionPlaneDistanceNbrNeighbors;
  double egoMotionLineDistancefactor, egoMotionPlaneDistancefactor1, egoMotionPlaneDistancefactor2, egoMotionMaxLineDistance, egoMotionMaxPlaneDistance, egoMotionInitLossScale, egoMotionFinalLossScale;
  if (priv_nh.getParam("slam/ego_motion_LM_max_iter", egoMotionLMMaxIter))
    LidarSlam.SetEgoMotionLMMaxIter(egoMotionLMMaxIter);
  if (priv_nh.getParam("slam/ego_motion_ICP_max_iter", egoMotionICPMaxIter))
    LidarSlam.SetEgoMotionICPMaxIter(egoMotionICPMaxIter);
  if (priv_nh.getParam("slam/ego_motion_line_distance_nbr_neighbors", egoMotionLineDistanceNbrNeighbors))
    LidarSlam.SetEgoMotionLineDistanceNbrNeighbors(egoMotionLineDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/ego_motion_minimum_line_neighbor_rejection", egoMotionMinimumLineNeighborRejection))
    LidarSlam.SetEgoMotionMinimumLineNeighborRejection(egoMotionMinimumLineNeighborRejection);
  if (priv_nh.getParam("slam/ego_motion_line_distance_factor", egoMotionLineDistancefactor))
    LidarSlam.SetEgoMotionLineDistancefactor(egoMotionLineDistancefactor);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_nbr_neighbors", egoMotionPlaneDistanceNbrNeighbors))
    LidarSlam.SetEgoMotionPlaneDistanceNbrNeighbors(egoMotionPlaneDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_factor1", egoMotionPlaneDistancefactor1))
    LidarSlam.SetEgoMotionPlaneDistancefactor1(egoMotionPlaneDistancefactor1);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_factor2", egoMotionPlaneDistancefactor2))
    LidarSlam.SetEgoMotionPlaneDistancefactor2(egoMotionPlaneDistancefactor2);
  if (priv_nh.getParam("slam/ego_motion_max_line_distance", egoMotionMaxLineDistance))
    LidarSlam.SetEgoMotionMaxLineDistance(egoMotionMaxLineDistance);
  if (priv_nh.getParam("slam/ego_motion_max_plane_distance", egoMotionMaxPlaneDistance))
    LidarSlam.SetEgoMotionMaxPlaneDistance(egoMotionMaxPlaneDistance);
  if (priv_nh.getParam("slam/ego_motion_init_loss_scale", egoMotionInitLossScale))
    LidarSlam.SetEgoMotionInitLossScale(egoMotionInitLossScale);
  if (priv_nh.getParam("slam/ego_motion_final_loss_scale", egoMotionFinalLossScale))
    LidarSlam.SetEgoMotionFinalLossScale(egoMotionFinalLossScale);

  // mapping
  int mappingLMMaxIter, mappingICPMaxIter, mappingLineDistanceNbrNeighbors, mappingMinimumLineNeighborRejection, mappingPlaneDistanceNbrNeighbors;
  double mappingLineDistancefactor, mappingPlaneDistancefactor1, mappingPlaneDistancefactor2, mappingMaxLineDistance, mappingMaxPlaneDistance, mappingLineMaxDistInlier, mappingInitLossScale, mappingFinalLossScale;
  if (priv_nh.getParam("slam/mapping_LM_max_iter", mappingLMMaxIter))
    LidarSlam.SetMappingLMMaxIter(mappingLMMaxIter);
  if (priv_nh.getParam("slam/mapping_ICP_max_iter", mappingICPMaxIter))
    LidarSlam.SetMappingICPMaxIter(mappingICPMaxIter);
  if (priv_nh.getParam("slam/mapping_line_distance_nbr_neighbors", mappingLineDistanceNbrNeighbors))
    LidarSlam.SetMappingLineDistanceNbrNeighbors(mappingLineDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/mapping_minimum_line_neighbor_rejection", mappingMinimumLineNeighborRejection))
    LidarSlam.SetMappingMinimumLineNeighborRejection(mappingMinimumLineNeighborRejection);
  if (priv_nh.getParam("slam/mapping_line_distance_factor", mappingLineDistancefactor))
    LidarSlam.SetMappingLineDistancefactor(mappingLineDistancefactor);
  if (priv_nh.getParam("slam/mapping_plane_distance_nbr_neighbors", mappingPlaneDistanceNbrNeighbors))
    LidarSlam.SetMappingPlaneDistanceNbrNeighbors(mappingPlaneDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/mapping_plane_distance_factor1", mappingPlaneDistancefactor1))
    LidarSlam.SetMappingPlaneDistancefactor1(mappingPlaneDistancefactor1);
  if (priv_nh.getParam("slam/mapping_plane_distance_factor2", mappingPlaneDistancefactor2))
    LidarSlam.SetMappingPlaneDistancefactor2(mappingPlaneDistancefactor2);
  if (priv_nh.getParam("slam/mapping_max_line_distance", mappingMaxLineDistance))
    LidarSlam.SetMappingMaxLineDistance(mappingMaxLineDistance);
  if (priv_nh.getParam("slam/mapping_max_plane_distance", mappingMaxPlaneDistance))
    LidarSlam.SetMappingMaxPlaneDistance(mappingMaxPlaneDistance);
  if (priv_nh.getParam("slam/mapping_line_max_dist_inlier", mappingLineMaxDistInlier))
    LidarSlam.SetMappingLineMaxDistInlier(mappingLineMaxDistInlier);
  if (priv_nh.getParam("slam/mapping_init_loss_scale", mappingInitLossScale))
    LidarSlam.SetMappingInitLossScale(mappingInitLossScale);
  if (priv_nh.getParam("slam/mapping_final_loss_scale", mappingFinalLossScale))
    LidarSlam.SetMappingFinalLossScale(mappingFinalLossScale);

  // rolling grids
  double voxelGridLeafSizeEdges, voxelGridLeafSizePlanes, voxelGridLeafSizeBlobs, voxelGridSize, voxelGridResolution;
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_edges", voxelGridLeafSizeEdges))
    LidarSlam.SetVoxelGridLeafSizeEdges(voxelGridLeafSizeEdges);
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_planes", voxelGridLeafSizePlanes))
    LidarSlam.SetVoxelGridLeafSizePlanes(voxelGridLeafSizePlanes);
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_blobs", voxelGridLeafSizeBlobs))
    LidarSlam.SetVoxelGridLeafSizeBlobs(voxelGridLeafSizeBlobs);
  if (priv_nh.getParam("slam/voxel_grid_size", voxelGridSize))
    LidarSlam.SetVoxelGridSize(voxelGridSize);
  if (priv_nh.getParam("slam/voxel_grid_resolution", voxelGridResolution))
    LidarSlam.SetVoxelGridResolution(voxelGridResolution);
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsSlamCalibration()
{
  // Transform to modifiable vectors
  std::vector<Transform> slamToLidarPoses(this->SlamPoses.begin(), this->SlamPoses.end());
  std::vector<Transform> worldToGpsPoses(this->GpsPoses.begin(), this->GpsPoses.end());

  // If a sensors offset is given, use it to compute real GPS antenna position in SLAM coordinates
  if (!this->LidarToGpsOffset.isApprox(Eigen::Isometry3d::Identity()))
  {
    if (this->Verbose >= 2)
      std::cout << "Transforming LiDAR pose acquired by SLAM to GPS antenna pose using LIDAR to GPS antenna offset :"
                << std::endl << this->LidarToGpsOffset.matrix() << std::endl;
    for (Transform& slamToGpsPose : slamToLidarPoses)
    {
      Eigen::Isometry3d slamToLidar = slamToGpsPose.GetIsometry();
      Eigen::Isometry3d slamToGps(slamToLidar * this->LidarToGpsOffset);
      slamToGpsPose = Transform(slamToGpsPose.time, slamToGps);
    }
  }
  // At this point, we now have GPS antenna poses in SLAM coordinates.
  std::vector<Transform>& slamToGpsPoses = slamToLidarPoses;

  // Run calibration : compute transform from SLAM to WORLD
  GlobalTrajectoriesRegistration registration;
  registration.SetNoRoll(this->CalibrationNoRoll);  // DEBUG
  registration.SetVerbose(this->Verbose >= 2);
  Eigen::Isometry3d worldToSlam;
  if (!registration.ComputeTransformOffset(slamToGpsPoses, worldToGpsPoses, worldToSlam))
  {
    ROS_ERROR_STREAM("GPS/SLAM calibration failed.");
    return;
  }

  // Publish ICP-matched trajectories
  if (this->PublishIcpTrajectories)
  {
    // GPS antenna trajectory acquired from GPS in WORLD coordinates
    nav_msgs::Path gpsPath;
    gpsPath.header.frame_id = this->GpsOriginFrameId;
    gpsPath.header.stamp = ros::Time::now();
    for (const Transform& pose: worldToGpsPoses)
    {
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.stamp = ros::Time(pose.time);
      poseStamped.header.frame_id = gpsPath.header.frame_id;
      poseStamped.pose.position.x = pose.x();
      poseStamped.pose.position.y = pose.y();
      poseStamped.pose.position.z = pose.z();
      Eigen::Quaterniond rot = pose.GetRotation();
      poseStamped.pose.orientation.w = rot.w();
      poseStamped.pose.orientation.x = rot.x();
      poseStamped.pose.orientation.y = rot.y();
      poseStamped.pose.orientation.z = rot.z();
      gpsPath.poses.push_back(poseStamped);
    }
    this->GpsPathPub.publish(gpsPath);
    // GPS antenna trajectory acquired from SLAM in WORLD coordinates
    nav_msgs::Path slamPath;
    slamPath.header = gpsPath.header;
    for (const Transform& pose: slamToGpsPoses)
    {
      Transform newPose(pose.time, worldToSlam * pose.GetIsometry());
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.stamp = ros::Time(newPose.time);
      poseStamped.header.frame_id = gpsPath.header.frame_id;
      poseStamped.pose.position.x = newPose.x();
      poseStamped.pose.position.y = newPose.y();
      poseStamped.pose.position.z = newPose.z();
      Eigen::Quaterniond rot = newPose.GetRotation();
      poseStamped.pose.orientation.w = rot.w();
      poseStamped.pose.orientation.x = rot.x();
      poseStamped.pose.orientation.y = rot.y();
      poseStamped.pose.orientation.z = rot.z();
      slamPath.poses.push_back(poseStamped);
    }
    this->SlamPathPub.publish(slamPath);
  }

  // Publish static tf with calibration to link world (UTM) frame to SLAM origin
  Eigen::Translation3d translation(worldToSlam.translation());
  Eigen::Quaterniond rotation(worldToSlam.rotation());
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = ros::Time::now();
  tfStamped.header.frame_id = this->GpsOriginFrameId;
  tfStamped.child_frame_id = this->SlamOriginFrameId;
  tfStamped.transform.translation.x = translation.x();
  tfStamped.transform.translation.y = translation.y();
  tfStamped.transform.translation.z = translation.z();
  tfStamped.transform.rotation.x = rotation.x();
  tfStamped.transform.rotation.y = rotation.y();
  tfStamped.transform.rotation.z = rotation.z();
  tfStamped.transform.rotation.w = rotation.w();
  this->StaticTfBroadcaster.sendTransform(tfStamped);

  Eigen::Vector3d xyz = worldToSlam.translation();
  Eigen::Vector3d ypr = worldToSlam.linear().eulerAngles(2, 1, 0);
  ROS_INFO_STREAM("\033[1;32m" <<
                  "Global transform from GPS ('" << this->GpsOriginFrameId << "') "
                  "to SLAM ('" << this->SlamOriginFrameId << "') "
                  "successfully estimated to :\n" << worldToSlam.matrix() <<
                  "\n(tf2 static transform : " <<
                  xyz(0) << " " << xyz(1) << " " << xyz(2) << " " <<
                  ypr(0) << " " << ypr(1) << " " << ypr(2) <<
                  " /" << this->GpsOriginFrameId << " /" << this->SlamOriginFrameId <<
                  ")\033[0m");
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