//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-11-19
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

#ifndef ODOM_TO_GPS_NODE_H
#define ODOM_TO_GPS_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>

class OdomToGpsNode
{
public:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  OdomToGpsNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     Odometry callback, projecting odometry (X/Y/Z) to GPS (Lat/Lon/Alt) coordinates using TF server.
   * @param[in] msg Pose to convert to GPS coordinates.
   */
  void OdometryCallback(const nav_msgs::Odometry& msg);

private:

  // ROS publishers & subscribers
  ros::Subscriber OdomSub;
  ros::Publisher GpsFixPub;
  tf2_ros::Buffer TfBuffer;
  tf2_ros::TransformListener TfListener;

  // Local variables
  std::string UtmBandLetter;  ///< MGRS latitude band letter.
  int UtmZoneNumber;          ///< UTM longitude zone number.

  // Parameters
  std::string UtmFrameId = "utm";  ///< Frame id of fixed to the UTM coordinates.
};

#endif  // ODOM_TO_GPS_NODE_H