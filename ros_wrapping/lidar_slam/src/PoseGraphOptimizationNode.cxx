#include "PoseGraphOptimizationNode.h"
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <chrono>

//------------------------------------------------------------------------------
PoseGraphOptimizationNode::PoseGraphOptimizationNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : TfListener(TfBuffer)
{
  // Init algo parameters
  this->Algo.SetGPSCalibration(0., 0., 0.);
  this->Algo.SetNbIteration(30);
  this->Algo.SetVerbose(true);
  this->Algo.SetG2OFileName("/home/nicolas/pose_graph_opitmization.g2o");
  this->Algo.SetSaveG2OFile(true);
  this->Algo.SetTimeOffset(0.);

  // Init ROS subscribers and publishers
  this->OptimSlamPosesPub = nh.advertise<nav_msgs::Path>("optim_slam_poses", 1);
  this->SlamPoseSub = nh.subscribe("slam_pose", 10, &PoseGraphOptimizationNode::SlamPoseCallback, this);
  this->GpsPoseSub = nh.subscribe("gps_odom", 10, &PoseGraphOptimizationNode::GpsPoseCallback, this);
  this->RunOptimizationSub = nh.subscribe("run_pose_graph_optim", 1, &PoseGraphOptimizationNode::RunOptimizationCallback, this);

  ROS_INFO_STREAM("\033[1;32mPose graph optimization is ready !\033[0m");
}

//------------------------------------------------------------------------------
void PoseGraphOptimizationNode::SlamPoseCallback(const nav_msgs::Odometry& msg)
{
  // Transform pose to output frame
  geometry_msgs::TransformStamped tfStamped;
  geometry_msgs::Pose slamPose;
  tfStamped = this->TfBuffer.lookupTransform(this->OutputFrameId, msg.header.frame_id, msg.header.stamp, ros::Duration(1.));
  tf2::doTransform(msg.pose.pose, slamPose, tfStamped);

  // Unpack SLAM pose
  double time = msg.header.stamp.toSec();
  Eigen::Translation3d trans(slamPose.position.x,
                             slamPose.position.y,
                             slamPose.position.z);
  Eigen::Quaterniond rot(slamPose.orientation.w,
                         slamPose.orientation.x,
                         slamPose.orientation.y,
                         slamPose.orientation.z);
  Transform pose(time, trans, rot);

  // Unpack covariance
  // CHECK DoF order, (X, Y, Z, rX, rY, rZ) or (rX, rY, rZ, X, Y, Z)
  // std::array<double, 36> covar;
  // std::copy(covar.begin(), covar.end(), (double*) msg.pose.covariance.begin());
  const auto& c = msg.pose.covariance; 
  std::array<double, 36> covar = {c[21], c[22], c[23],   c[18], c[19], c[20],
                                  c[27], c[28], c[29],   c[24], c[25], c[26],
                                  c[33], c[34], c[35],   c[30], c[31], c[32],

                                  c[ 3], c[ 4], c[ 5],   c[ 0], c[ 1], c[ 2],
                                  c[ 9], c[10], c[11],   c[ 6], c[ 7], c[ 8],
                                  c[15], c[16], c[17],   c[12], c[13], c[14]};

  // Save for later optimization
  this->SlamPoses.push_back(pose);
  this->SlamCovariances.push_back(covar);
}

//------------------------------------------------------------------------------
void PoseGraphOptimizationNode::GpsPoseCallback(const nav_msgs::Odometry& msg)
{
  // DEBUG drop 19 messages over 20
  // if (msg.header.seq % 50)
  //   return;

  // Transform pose to output frame
  geometry_msgs::TransformStamped tfStamped;
  geometry_msgs::Pose gpsPose;
  tfStamped = this->TfBuffer.lookupTransform(this->OutputFrameId, msg.header.frame_id, msg.header.stamp, ros::Duration(1.));
  tf2::doTransform(msg.pose.pose, gpsPose, tfStamped);

  // Unpack GPS pose
  double time = msg.header.stamp.toSec();
  Eigen::Translation3d trans(gpsPose.position.x,
                             gpsPose.position.y,
                             gpsPose.position.z);
  Eigen::Quaterniond rot(gpsPose.orientation.w,
                         gpsPose.orientation.x,
                         gpsPose.orientation.y,
                         gpsPose.orientation.z);
  Transform pose(time, trans, rot);

  // Unpack covariance
  const auto& c = msg.pose.covariance; 
  std::array<double, 9> covar = {c[ 0], c[ 1], c[ 2],
                                 c[ 6], c[ 7], c[ 8],
                                 c[12], c[13], c[14]};
  // std::array<double, 9> covar = {1e-3,    0,    0,
  //                                   0, 1e-3,    0,
  //                                   0,    0, 1e-3};

  // Save for later optimization
  this->GpsPoses.push_back(pose);
  this->GpsCovariances.push_back(covar);
}

//------------------------------------------------------------------------------
void PoseGraphOptimizationNode::RunOptimizationCallback(const std_msgs::Empty& msg)
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

  // DEBUG
  std::cout << "1st GPS point    : "
            << GpsPoses[0].x << ", "
            << GpsPoses[0].y << ", "
            << GpsPoses[0].z << std::endl;
  std::cout << "1st SLAM point    : "
            << SlamPoses[0].x << ", "
            << SlamPoses[0].y << ", "
            << SlamPoses[0].z << std::endl;
  std::cout << "1st Optimized point    : "
            << optimizedSlamPoses[0].x << ", "
            << optimizedSlamPoses[0].y << ", "
            << optimizedSlamPoses[0].z << std::endl << std::endl;
  std::cout << "Last GPS point    : "
            << GpsPoses.back().x << ", "
            << GpsPoses.back().y << ", "
            << GpsPoses.back().z << std::endl;
  std::cout << "Last SLAM point    : "
            << SlamPoses.back().x << ", "
            << SlamPoses.back().y << ", "
            << SlamPoses.back().z << std::endl;
  std::cout << "Last Optimized point    : "
            << optimizedSlamPoses.back().x << ", "
            << optimizedSlamPoses.back().y << ", "
            << optimizedSlamPoses.back().z << std::endl << std::endl;

  // Publish optimized SLAM trajectory
  nav_msgs::Path optimSlamTraj;
  optimSlamTraj.header.frame_id = this->OutputFrameId;
  optimSlamTraj.header.stamp = ros::Time::now();
  for (const Transform& pose: optimizedSlamPoses)
  {
    // Create new pose msg
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = this->OutputFrameId;
    poseMsg.header.stamp = ros::Time(pose.time);

    // Fill position
    poseMsg.pose.position.x = pose.x;
    poseMsg.pose.position.y = pose.y;
    poseMsg.pose.position.z = pose.z;

    // Fill orientation
    Eigen::Quaterniond quat(Eigen::AngleAxisd(pose.rz, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pose.ry, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(pose.rx, Eigen::Vector3d::UnitX()));
    poseMsg.pose.orientation.w = quat.w();
    poseMsg.pose.orientation.x = quat.x();
    poseMsg.pose.orientation.y = quat.y();
    poseMsg.pose.orientation.z = quat.z();

    // Add to trajectory
    optimSlamTraj.poses.push_back(poseMsg);
  }
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