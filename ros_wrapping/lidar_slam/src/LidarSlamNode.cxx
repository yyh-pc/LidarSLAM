#include "LidarSlamNode.h"

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // TODO : Init laser id mapping and nLasers with ROS param
  size_t nLasers = 16;
  laserIdMapping_.resize(nLasers);
  for (unsigned int i = 0; i < laserIdMapping_.size(); i++)
    laserIdMapping_[i] = i;

  // init ROS stuff
  debugCloudPub_ = nh.advertise<pcl::PointCloud<PointS>>("debug_cloud", 1);
  cloudSub_ = nh.subscribe("/velodyne_points", 1, &LidarSlamNode::scanCallback, this);
}

//------------------------------------------------------------------------------
void LidarSlamNode::scanCallback(const pcl::PointCloud<PointV>& cloudV)
{
  // Convert pointcloud PointV type to expected PointS type
  pcl::PointCloud<PointS>::Ptr cloudS(new pcl::PointCloud<PointS>);
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

  // publish converted PC2
  debugCloudPub_.publish(cloudS);

  // Register new frame and update position and mapping.
  slam_.AddFrame(cloudS, laserIdMapping_);

  // Get the computed world transform so far
  Transform worldTransform = slam_.GetWorldTransform();
  std::vector<double> covar = slam_.GetTransformCovariance();

  // TODO publish worldTransform and covar
  geometry_msgs::TransformStamped tfStamped;
  pcl_conversions::fromPCL(cloudV.header, tfStamped.header);
  tfStamped.header.frame_id = "slam_init";
  tfStamped.child_frame_id = cloudV.header.frame_id;
  tfStamped.transform.translation.x = worldTransform.x;
  tfStamped.transform.translation.y = worldTransform.y;
  tfStamped.transform.translation.z = worldTransform.z;
  tf2::Quaternion q;
  q.setRPY(worldTransform.rx, worldTransform.ry, worldTransform.rz);
  tfStamped.transform.rotation.x = q.x();
  tfStamped.transform.rotation.y = q.y();
  tfStamped.transform.rotation.z = q.z();
  tfStamped.transform.rotation.w = q.w();
  tfBroadcaster_.sendTransform(tfStamped);
}
