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

#ifndef VELODYNE_SLAM_NODE_H
#define VELODYNE_SLAM_NODE_H

// ROS
#include <ros/ros.h>
#include <velodyne_pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <LidarSlam/Slam.h>

class VelodyneToSlamNode
{
public:
  using PointS = LidarSlam::LidarPoint;    
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using PointV = velodyne_pcl::PointXYZIRT;
  using CloudV = pcl::PointCloud<PointV>;  ///< Pointcloud published by velodyne driver and decoded by pcl

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  VelodyneToSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     New lidar frame callback, running Conversion and publishing PointCloud with LidarPoint Formatting.
   * @param[in] cloud New Lidar Frame, published by velodyne_pointcloud/transform_node.
   */
  void Callback(const CloudV& cloud);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscribers and publishers
  ros::NodeHandle &Nh, &PrivNh;
  ros::Subscriber Listener;
  ros::Publisher Talker;
  double LidarFreq;
};

#endif // VELODYNE_SLAM_NODE_H
