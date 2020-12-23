//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2020-12-22
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

#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <LidarSlam/LidarPoint.h>

namespace lidar_conversions
{

/**
 * @class RobosenseToLidarNode aims at converting pointclouds published by RSLidar
 * ROS driver to the expected SLAM pointcloud format.
 *
 * The ROS RSLidar driver can be found here :
 * https://github.com/RoboSense-LiDAR/ros_rslidar
 */
class RobosenseToLidarNode
{
public:
  using PointRS = pcl::PointXYZI;
  using CloudRS = pcl::PointCloud<PointRS>;  ///< Pointcloud published by rslidar driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param nh      Public ROS node handle, used to init publisher/subscriber.
   * @param priv_nh Private ROS node handle, used to access parameters.
   */
  RobosenseToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing RSLidar PointCloud as SLAM LidarPoint.
   * @param cloud New Lidar Frame, published by rslidar_pointcloud/cloud_node.
   */
  void Callback(const CloudRS& cloud);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber Listener;
  ros::Publisher Talker;

  // Useful variables for approximate point-wise timestamps computation
  // These parameters should be set to the same values as ROS RSLidar driver's.
  double Rpm = 600;  ///< Spinning speed of sensor [rpm]
};

}  // end of namespace lidar_conversions
