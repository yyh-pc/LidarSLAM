#ifndef POSE_GRAPH_OPTIMIZATION_NODE_H
#define POSE_GRAPH_OPTIMIZATION_NODE_H

// ROS stuff
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

// SLAM lib
#include <PoseGraphOptimization.h>

class PoseGraphOptimizationNode
{
public:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  PoseGraphOptimizationNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Lidar SLAM pose callback, accumulating poses for later pose graph optimization.
   * @param[in] msg New LiDAR SLAM pose with its associated covariance.
   */
  void SlamPoseCallback(const nav_msgs::Odometry& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     GPS pose callback, accumulating poses for later pose graph optimization.
   * @param[in] msg New converted GPS pose with its associated covariance.
   */
  void GpsPoseCallback(const nav_msgs::Odometry& msg);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Run pose graph optimization from GPS and SLAM poses, and publish optimized LiDAR trajectory.
   * @param[in] msg (Unused)
   */
  void RunOptimizationCallback(const std_msgs::Empty&);

private:

  //----------------------------------------------------------------------------

  // SLAM pose graph optimization
  PoseGraphOptimization Algo;
  std::vector<Transform> SlamPoses;
  std::vector<Transform> GpsPoses;
  std::vector<std::array<double, 36>> SlamCovariances;
  std::vector<std::array<double, 9>> GpsCovariances;

  // ROS publishers & subscribers
  ros::Subscriber SlamPoseSub;
  ros::Subscriber GpsPoseSub;
  ros::Subscriber RunOptimizationSub;
  ros::Publisher OptimSlamPosesPub;

  std::string GpsFrameId;
};

#endif // POSE_GRAPH_OPTIMIZATION_NODE_H