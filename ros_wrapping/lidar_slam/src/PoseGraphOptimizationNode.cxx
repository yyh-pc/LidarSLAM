#include "PoseGraphOptimizationNode.h"
#include "ros_transform_utils.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

#include <chrono>

//------------------------------------------------------------------------------
PoseGraphOptimizationNode::PoseGraphOptimizationNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Init algo parameters
  bool verbose;
  int nbIterations;
  double timeOffset;
  std::string g2oFileName;
  std::vector<double> gpsCalib;
  if (priv_nh.getParam("n_iterations", nbIterations))
    this->Algo.SetNbIteration(nbIterations);
  if (priv_nh.getParam("time_offset", timeOffset))
    this->Algo.SetTimeOffset(timeOffset);
  if (priv_nh.getParam("verbose", verbose))
    this->Algo.SetVerbose(verbose);
  if (priv_nh.getParam("g2o_file_name", g2oFileName))
  {
    this->Algo.SetSaveG2OFile(!g2oFileName.empty());
    this->Algo.SetG2OFileName(g2oFileName);
  }
  if (priv_nh.getParam("tf_gps_to_sensor", gpsCalib))
    this->Algo.SetGpsToSensorCalibration(gpsCalib[0], gpsCalib[1], gpsCalib[2], gpsCalib[3], gpsCalib[4], gpsCalib[5]);

  // Init ROS subscribers and publishers
  this->OptimSlamPosesPub = nh.advertise<nav_msgs::Path>("optim_slam_poses", 1);
  this->SlamPoseSub = nh.subscribe("slam_odom", 10, &PoseGraphOptimizationNode::SlamPoseCallback, this);
  this->GpsPoseSub = nh.subscribe("gps_odom", 10, &PoseGraphOptimizationNode::GpsPoseCallback, this);
  this->RunOptimizationSub = nh.subscribe("run_pose_graph_optim", 1, &PoseGraphOptimizationNode::RunOptimizationCallback, this);

  ROS_INFO_STREAM("\033[1;32mPose graph optimization is ready !\033[0m");
}

//------------------------------------------------------------------------------
void PoseGraphOptimizationNode::SlamPoseCallback(const nav_msgs::Odometry& msg)
{
  // Unpack SLAM pose
  Transform pose = PoseMsgToTransform(msg.pose.pose, msg.header.stamp.toSec(), msg.header.frame_id);
  this->SlamPoses.push_back(pose);

  // Unpack covariance
  std::array<double, 36> covar;
  std::copy(msg.pose.covariance.begin(), msg.pose.covariance.end(), covar.begin());
  this->SlamCovariances.push_back(covar);
}

//------------------------------------------------------------------------------
void PoseGraphOptimizationNode::GpsPoseCallback(const nav_msgs::Odometry& msg)
{
  // // DEBUG drop 49 messages over 50
  // if (msg.header.seq % 50)
  //   return;
  // // DEBUG drop all messages with seq index between 50 and 300, each 300 messages
  // if (50 < msg.header.seq % 300)
  //   return;

  // Unpack GPS pose
  Transform pose = PoseMsgToTransform(msg.pose.pose, msg.header.stamp.toSec(), msg.header.frame_id);
  this->GpsPoses.push_back(pose);

  // Unpack position covariance only
  const auto& c = msg.pose.covariance; 
  std::array<double, 9> covar = {c[ 0], c[ 1], c[ 2],
                                 c[ 6], c[ 7], c[ 8],
                                 c[12], c[13], c[14]};
  this->GpsCovariances.push_back(covar);
}

//------------------------------------------------------------------------------
void PoseGraphOptimizationNode::RunOptimizationCallback(const std_msgs::Empty&)
{
  // Init timer
  ROS_INFO_STREAM("Pose graph optimization started.");
  std::chrono::duration<double, std::milli> chrono_ms;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

  // Run pose graph optimization
  std::vector<Transform> optimizedSlamPoses;
  if (!this->Algo.Process(this->SlamPoses, this->GpsPoses,
                          this->SlamCovariances, this->GpsCovariances,
                          optimizedSlamPoses))
  {
    ROS_ERROR_STREAM("Pose graph optimization failed.");
    return;
  }

  // Display duration info
  chrono_ms = std::chrono::high_resolution_clock::now() - start;
  ROS_INFO_STREAM("\033[1;32mPose graph optimization succeeded\033[0m (" << chrono_ms.count() << " ms)");

  // Publish optimized SLAM trajectory
  nav_msgs::Path optimSlamTraj;
  optimSlamTraj.header.frame_id = this->GpsPoses[0].frameid;
  optimSlamTraj.header.stamp = ros::Time::now();
  for (const Transform& pose: optimizedSlamPoses)
    optimSlamTraj.poses.push_back(TransformToPoseStampedMsg(pose));
  this->OptimSlamPosesPub.publish(optimSlamTraj);
}

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  // Init ROS node.
  ros::init(argc, argv, "pose_graph_optimization");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create pose graph optimization node,
  // which subscribes to GPS and LiDAR SLAM poses.
  PoseGraphOptimizationNode node(nh, priv_nh);

  // Handle callbacks until shut down.
  ros::spin();

  return 0;
}