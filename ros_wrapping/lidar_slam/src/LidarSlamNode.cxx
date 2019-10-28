#include "LidarSlamNode.h"

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Get SLAM params
  SetSlamParameters(priv_nh);

  // Init laserIdMapping
  std::vector<int> intLaserIdMapping;
  int nLasers;
  // Try to get it directly from ROS param
  if (priv_nh.getParam("laser_id_mapping", intLaserIdMapping))
  {
    laserIdMapping_.assign(intLaserIdMapping.begin(), intLaserIdMapping.end());
    ROS_INFO_STREAM("[SLAM] Using laser_id_mapping from ROS param.");
  }
  // Or only try to get number of lasers to build linear mapping
  else if (priv_nh.getParam("n_lasers", nLasers))
  {
    laserIdMapping_.resize(nLasers);
    for (int i = 0; i < nLasers; i++)
      laserIdMapping_[i] = i;
    ROS_INFO_STREAM("[SLAM] Using 0->" << nLasers << " linear laser_id_mapping from ROS param.");
  }
  // Otherwise, n_lasers will be guessed from 1st frame
  else
  {
    ROS_WARN_STREAM("[SLAM] No laser_id_mapping nor n_lasers params found : "
                    "n_lasers will be guessed from 1st frame to build linear mapping.");
  }

  // Init optional publishers
  priv_nh.getParam("publish_features_maps/edges", publishEdges_);
  priv_nh.getParam("publish_features_maps/planars", publishPlanars_);
  priv_nh.getParam("publish_features_maps/blobs", publishBlobs_);
  if (publishEdges_)
    edgesPub_ = nh.advertise<CloudS>("edges_features", 1);
  if (publishPlanars_)
    planarsPub_ = nh.advertise<CloudS>("planars_features", 1);
  if (publishBlobs_)
    blobsPub_ = nh.advertise<CloudS>("blobs_features", 1);
  debugCloudPub_ = nh.advertise<CloudS>("debug_cloud", 1);

  // Init ROS subscribers and publishers
  priv_nh.getParam("slam_origin_frame", slamOriginFrameId_);
  poseCovarPub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose", 1);
  cloudSub_ = nh.subscribe("velodyne_points", 1, &LidarSlamNode::scanCallback, this);

  ROS_INFO_STREAM("\033[1;32m LiDAR SLAM is ready ! \033[0m");
}

//------------------------------------------------------------------------------
void LidarSlamNode::scanCallback(const CloudV& cloudV)
{
  // Init laserIdMapping_ if not already done
  if (!laserIdMapping_.size())
  {
    // Iterate through pointcloud to find max ring
    int nLasers = 0;
    for(const PointV& point : cloudV)
    {
      if (point.ring > nLasers)
        nLasers = point.ring;
    }
    ++nLasers;

    // Init laserIdMapping_ with linear mapping
    laserIdMapping_.resize(nLasers);
    for (int i = 0; i < nLasers; i++)
      laserIdMapping_[i] = i;

    ROS_INFO_STREAM("[SLAM] Using 0->" << nLasers << " linear laser_id_mapping.");
  }

  // Convert pointcloud PointV type to expected PointS type
  CloudS::Ptr cloudS = convertToSlamPointCloud(cloudV);

  // Run SLAM : register new frame and update position and mapping.
  slam_.AddFrame(cloudS, laserIdMapping_);

  // Get the computed world transform so far
  Transform worldTransform = slam_.GetWorldTransform();
  std::vector<double> poseCovar = slam_.GetTransformCovariance();

  // Publish TF, pose and covariance
  publishTfPoseCovar(cloudV.header, worldTransform, poseCovar);

  // Publish optional debug info
  publishFeaturesMaps(cloudS);
}

//------------------------------------------------------------------------------
LidarSlamNode::CloudS::Ptr LidarSlamNode::convertToSlamPointCloud(const CloudV& cloudV)
{
  CloudS::Ptr cloudS(new CloudS);
  cloudS->resize(cloudV.size());
  cloudS->header = cloudV.header;
  for(unsigned int i = 0; i < cloudV.size(); i++)
  {
    const PointV& velodynePoint = cloudV[i];
    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laserId = velodynePoint.ring;
    slamPoint.time =  std::atan2(velodynePoint.y, velodynePoint.x);
    cloudS->at(i) = slamPoint;
  }
  return cloudS;
}

//------------------------------------------------------------------------------
void LidarSlamNode::publishTfPoseCovar(const pcl::PCLHeader& headerCloudV, 
                                       const Transform& worldTransform, 
                                       const std::vector<double>& poseCovar)
{
  // publish worldTransform
  geometry_msgs::TransformStamped tfMsg;
  pcl_conversions::fromPCL(headerCloudV, tfMsg.header);
  tfMsg.header.frame_id = slamOriginFrameId_;
  tfMsg.child_frame_id = headerCloudV.frame_id;
  tfMsg.transform.translation.x = worldTransform.x;
  tfMsg.transform.translation.y = worldTransform.y;
  tfMsg.transform.translation.z = worldTransform.z;
  tf2::Quaternion q;
  q.setRPY(worldTransform.rx, worldTransform.ry, worldTransform.rz);
  tfMsg.transform.rotation.x = q.x();
  tfMsg.transform.rotation.y = q.y();
  tfMsg.transform.rotation.z = q.z();
  tfMsg.transform.rotation.w = q.w();
  tfBroadcaster_.sendTransform(tfMsg);

  // publish pose with covariance
  geometry_msgs::PoseWithCovarianceStamped poseCovarMsg;
  poseCovarMsg.header = tfMsg.header;
  poseCovarMsg.pose.pose.orientation = tfMsg.transform.rotation;
  poseCovarMsg.pose.pose.position.x = worldTransform.x;
  poseCovarMsg.pose.pose.position.y = worldTransform.y;
  poseCovarMsg.pose.pose.position.z = worldTransform.z;
  // Reshape covariance from parameters (rX, rY, rZ, X, Y, Z) to (X, Y, Z, rX, rY, rZ)
  const std::vector<double>& c = poseCovar;
  poseCovarMsg.pose.covariance = {c[21], c[22], c[23],   c[18], c[19], c[20],
                                  c[27], c[28], c[29],   c[24], c[25], c[26],
                                  c[33], c[34], c[35],   c[30], c[31], c[32],

                                  c[ 3], c[ 4], c[ 5],   c[ 0], c[ 1], c[ 2],
                                  c[ 9], c[10], c[11],   c[ 6], c[ 7], c[ 8],
                                  c[15], c[16], c[17],   c[12], c[13], c[14]};
  poseCovarPub_.publish(poseCovarMsg);
}

void LidarSlamNode::publishFeaturesMaps(const CloudS::Ptr& cloudS)
{
  // Publish pointcloud only if someone is listening to it to spare bandwidth.
  if (debugCloudPub_.getNumSubscribers())
    debugCloudPub_.publish(cloudS);

  pcl::PCLHeader msgHeader = cloudS->header;
  msgHeader.frame_id = slamOriginFrameId_;

  // Publish edges only if recquired and if someone is listening to it.
  if (publishEdges_ && edgesPub_.getNumSubscribers())
  {
    CloudS::Ptr edgesCloud = slam_.GetEdgesMap();
    edgesCloud->header = msgHeader;
    edgesPub_.publish(edgesCloud);
  }

  // Publish planars only if recquired and if someone is listening to it.
  if (publishPlanars_ && planarsPub_.getNumSubscribers())
  {
    CloudS::Ptr planarsCloud = slam_.GetPlanarsMap();
    planarsCloud->header = msgHeader;
    planarsPub_.publish(planarsCloud);
  }

  // Publish blobs only if recquired and if someone is listening to it.
  if (publishBlobs_ && blobsPub_.getNumSubscribers())
  {
    CloudS::Ptr blobsCloud = slam_.GetBlobsMap();
    blobsCloud->header = msgHeader;
    blobsPub_.publish(blobsCloud);
  }
}

void LidarSlamNode::SetSlamParameters(ros::NodeHandle& priv_nh)
{
  // common
  bool fastSlam, undistortion;
  double maxDistanceForICPMatching;
  if (priv_nh.getParam("slam/fast_slam", fastSlam))
    slam_.SetFastSlam(fastSlam);
  if (priv_nh.getParam("slam/undistortion", undistortion))
    slam_.SetUndistortion(undistortion);
  if (priv_nh.getParam("slam/max_distance_for_ICP_matching", maxDistanceForICPMatching))
    slam_.SetMaxDistanceForICPMatching(maxDistanceForICPMatching);

  // ego motion
  int egoMotionLMMaxIter, egoMotionICPMaxIter, egoMotionLineDistanceNbrNeighbors, egoMotionMinimumLineNeighborRejection, egoMotionPlaneDistanceNbrNeighbors;
  double egoMotionLineDistancefactor, egoMotionPlaneDistancefactor1, egoMotionPlaneDistancefactor2, egoMotionMaxLineDistance, egoMotionMaxPlaneDistance, egoMotionInitLossScale, egoMotionFinalLossScale;
  if (priv_nh.getParam("slam/ego_motion_LM_max_iter", egoMotionLMMaxIter))
    slam_.SetEgoMotionLMMaxIter(egoMotionLMMaxIter);
  if (priv_nh.getParam("slam/ego_motion_ICP_max_iter", egoMotionICPMaxIter))
    slam_.SetEgoMotionICPMaxIter(egoMotionICPMaxIter);
  if (priv_nh.getParam("slam/ego_motion_line_distance_nbr_neighbors", egoMotionLineDistanceNbrNeighbors))
    slam_.SetEgoMotionLineDistanceNbrNeighbors(egoMotionLineDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/ego_motion_minimum_line_neighbor_rejection", egoMotionMinimumLineNeighborRejection))
    slam_.SetEgoMotionMinimumLineNeighborRejection(egoMotionMinimumLineNeighborRejection);
  if (priv_nh.getParam("slam/ego_motion_line_distance_factor", egoMotionLineDistancefactor))
    slam_.SetEgoMotionLineDistancefactor(egoMotionLineDistancefactor);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_nbr_neighbors", egoMotionPlaneDistanceNbrNeighbors))
    slam_.SetEgoMotionPlaneDistanceNbrNeighbors(egoMotionPlaneDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_factor1", egoMotionPlaneDistancefactor1))
    slam_.SetEgoMotionPlaneDistancefactor1(egoMotionPlaneDistancefactor1);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_factor2", egoMotionPlaneDistancefactor2))
    slam_.SetEgoMotionPlaneDistancefactor2(egoMotionPlaneDistancefactor2);
  if (priv_nh.getParam("slam/ego_motion_max_line_distance", egoMotionMaxLineDistance))
    slam_.SetEgoMotionMaxLineDistance(egoMotionMaxLineDistance);
  if (priv_nh.getParam("slam/ego_motion_max_plane_distance", egoMotionMaxPlaneDistance))
    slam_.SetEgoMotionMaxPlaneDistance(egoMotionMaxPlaneDistance);
  if (priv_nh.getParam("slam/ego_motion_init_loss_scale", egoMotionInitLossScale))
    slam_.SetEgoMotionInitLossScale(egoMotionInitLossScale);
  if (priv_nh.getParam("slam/ego_motion_final_loss_scale", egoMotionFinalLossScale))
    slam_.SetEgoMotionFinalLossScale(egoMotionFinalLossScale);

  // mapping
  int mappingLMMaxIter, mappingICPMaxIter, mappingLineDistanceNbrNeighbors, mappingMinimumLineNeighborRejection, mappingPlaneDistanceNbrNeighbors;
  double mappingLineDistancefactor, mappingPlaneDistancefactor1, mappingPlaneDistancefactor2, mappingMaxLineDistance, mappingMaxPlaneDistance, mappingLineMaxDistInlier, mappingInitLossScale, mappingFinalLossScale;
  if (priv_nh.getParam("slam/mapping_LM_max_iter", mappingLMMaxIter))
    slam_.SetMappingLMMaxIter(mappingLMMaxIter);
  if (priv_nh.getParam("slam/mapping_ICP_max_iter", mappingICPMaxIter))
    slam_.SetMappingICPMaxIter(mappingICPMaxIter);
  if (priv_nh.getParam("slam/mapping_line_distance_nbr_neighbors", mappingLineDistanceNbrNeighbors))
    slam_.SetMappingLineDistanceNbrNeighbors(mappingLineDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/mapping_minimum_line_neighbor_rejection", mappingMinimumLineNeighborRejection))
    slam_.SetMappingMinimumLineNeighborRejection(mappingMinimumLineNeighborRejection);
  if (priv_nh.getParam("slam/mapping_line_distance_factor", mappingLineDistancefactor))
    slam_.SetMappingLineDistancefactor(mappingLineDistancefactor);
  if (priv_nh.getParam("slam/mapping_plane_distance_nbr_neighbors", mappingPlaneDistanceNbrNeighbors))
    slam_.SetMappingPlaneDistanceNbrNeighbors(mappingPlaneDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/mapping_plane_distance_factor1", mappingPlaneDistancefactor1))
    slam_.SetMappingPlaneDistancefactor1(mappingPlaneDistancefactor1);
  if (priv_nh.getParam("slam/mapping_plane_distance_factor2", mappingPlaneDistancefactor2))
    slam_.SetMappingPlaneDistancefactor2(mappingPlaneDistancefactor2);
  if (priv_nh.getParam("slam/mapping_max_line_distance", mappingMaxLineDistance))
    slam_.SetMappingMaxLineDistance(mappingMaxLineDistance);
  if (priv_nh.getParam("slam/mapping_max_plane_distance", mappingMaxPlaneDistance))
    slam_.SetMappingMaxPlaneDistance(mappingMaxPlaneDistance);
  if (priv_nh.getParam("slam/mapping_line_max_dist_inlier", mappingLineMaxDistInlier))
    slam_.SetMappingLineMaxDistInlier(mappingLineMaxDistInlier);
  if (priv_nh.getParam("slam/mapping_init_loss_scale", mappingInitLossScale))
    slam_.SetMappingInitLossScale(mappingInitLossScale);
  if (priv_nh.getParam("slam/mapping_final_loss_scale", mappingFinalLossScale))
    slam_.SetMappingFinalLossScale(mappingFinalLossScale);

  // rolling grids
  double voxelGridLeafSizeEdges, voxelGridLeafSizePlanes, voxelGridLeafSizeBlobs, voxelGridSize, voxelGridResolution;
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_edges", voxelGridLeafSizeEdges))
    slam_.SetVoxelGridLeafSizeEdges(voxelGridLeafSizeEdges);
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_planes", voxelGridLeafSizePlanes))
    slam_.SetVoxelGridLeafSizePlanes(voxelGridLeafSizePlanes);
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_blobs", voxelGridLeafSizeBlobs))
    slam_.SetVoxelGridLeafSizeBlobs(voxelGridLeafSizeBlobs);
  if (priv_nh.getParam("slam/voxel_grid_size", voxelGridSize))
    slam_.SetVoxelGridSize(voxelGridSize);
  if (priv_nh.getParam("slam/voxel_grid_resolution", voxelGridResolution))
    slam_.SetVoxelGridResolution(voxelGridResolution);
}
