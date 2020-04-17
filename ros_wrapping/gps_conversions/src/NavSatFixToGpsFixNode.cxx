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

#include "NavSatFixToGpsFixNode.h"
#include <gps_common/GPSFix.h>

//------------------------------------------------------------------------------
NavSatFixToGpsFixNode::NavSatFixToGpsFixNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Init ROS publishers & subscribers
  this->GpsFixPub = nh.advertise<gps_common::GPSFix>("gps_fix", 10);
  this->NavSatFixSub = nh.subscribe("nav_sat_fix", 10, &NavSatFixToGpsFixNode::NavSatFixCallback, this);

  ROS_INFO_STREAM("\033[1;32mConversion node from sensor_msgs::NavSatFix to gps_common::GPSFix is ready !\033[0m");
}

//------------------------------------------------------------------------------
void NavSatFixToGpsFixNode::NavSatFixCallback(const sensor_msgs::NavSatFix& msg)
{
  gps_common::GPSFix gpsFix;
  gpsFix.header = msg.header;

  // GPS status
  gpsFix.status.header = msg.header;
  gpsFix.status.status = msg.status.STATUS_FIX;
  gpsFix.status.position_source = gpsFix.status.SOURCE_GPS;
  gpsFix.status.orientation_source = gpsFix.status.SOURCE_NONE;
  gpsFix.status.motion_source = gpsFix.status.SOURCE_NONE;

  // GPS fix, orientation and speed
  gpsFix.latitude = msg.latitude;
  gpsFix.longitude = msg.longitude;
  gpsFix.altitude = msg.altitude;

  // Dilution Of Precision
  gpsFix.hdop = -1;
  gpsFix.gdop = -1;
  gpsFix.pdop = -1;
  gpsFix.vdop = -1;
  gpsFix.tdop = -1;

  // Uncertainty of measurement, 95% confidence (that's why x2)
  gpsFix.err_horz = sqrt(msg.position_covariance[0]) + sqrt(msg.position_covariance[4]);
  gpsFix.err_vert = sqrt(msg.position_covariance[8]) * 2.;

  // Position covariance [m^2] defined relative to a tangential plane through 
  // the reported position. The components are East, North, and Up (ENU), in
  // row-major order.
  gpsFix.position_covariance_type = msg.position_covariance_type;
  gpsFix.position_covariance = msg.position_covariance;

  this->GpsFixPub.publish(gpsFix);
}

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  // Init ROS node.
  ros::init(argc, argv, "navsatfix_to_gpsfix");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create node
  NavSatFixToGpsFixNode node(nh, priv_nh);

  // Handle callbacks until shut down.
  ros::spin();

  return 0;
}