//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2020-12-10
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
#include <velodyne_point.h>
#include <LidarSlam/LidarPoint.h>

namespace lidar_conversions
{

/**
 * @class LivoxToLidarNode aims at converting pointclouds published by ROS
 * livox driver to the expected SLAM pointcloud format.
 *
 * The ROS livox driver can be found here :
 * https://github.com/Livox-SDK/livox_ros_driver
 */
class LivoxToLidarNode
{
public:
  using PointL = pcl::PointXYZI;
  using CloudL = pcl::PointCloud<PointL>;  ///< Pointcloud published by velodyne driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param nh      Public ROS node handle, used to init publisher/subscriber.
   * @param priv_nh Private ROS node handle, used to access parameters.
   */
  LivoxToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing Velodyne PointCloud as SLAM LidarPoint.
   * @param cloud New Lidar Frame, published by velodyne_pointcloud/transform_node.
   */
  void Callback(const CloudL& cloud);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber and publisher
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber Listener;
  ros::Publisher Talker;

  int DeviceId = 0;  ///< LiDAR device identifier to set for each point.
};

}  // end of namespace lidar_conversions
