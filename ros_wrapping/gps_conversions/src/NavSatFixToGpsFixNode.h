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

#ifndef NAV_SAT_FIX_TO_GPS_FIX_NODE_H
#define NAV_SAT_FIX_TO_GPS_FIX_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

class NavSatFixToGpsFixNode
{
public:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  NavSatFixToGpsFixNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     GPS pose callback, converting sensor_msgs::NavSatFix to gps_common::GPSFix.
   * @param[in] msg GPS fix with other user-defined data.
   */
  void NavSatFixCallback(const sensor_msgs::NavSatFix& msg);

private:

  // ROS publishers & subscribers
  ros::Subscriber NavSatFixSub;
  ros::Publisher GpsFixPub;
};

#endif // NAV_SAT_FIX_TO_GPS_FIX_NODE_H