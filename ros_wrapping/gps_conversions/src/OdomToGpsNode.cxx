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

#include "OdomToGpsNode.h"
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//------------------------------------------------------------------------------
OdomToGpsNode::OdomToGpsNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : TfListener(TfBuffer)
{
  // Get parameters from ROS param server
  priv_nh.getParam("utm_frame_id", this->UtmFrameId);
  priv_nh.getParamCached("/utm_zone", this->UtmZoneNumber);
  priv_nh.getParamCached("/utm_band", this->UtmBandLetter);

  // Init ROS publishers & subscribers
  this->GpsFixPub = nh.advertise<sensor_msgs::NavSatFix>("fix", 10);
  this->OdomSub = nh.subscribe("odom", 10, &OdomToGpsNode::OdometryCallback, this);

  ROS_INFO_STREAM("\033[1;32mOdom to GPS converter is ready !\033[0m");
}

//------------------------------------------------------------------------------
void OdomToGpsNode::OdometryCallback(const nav_msgs::Odometry& msg)
{
  // Transform pose to UTM X/Y/Z coordinates
  geometry_msgs::TransformStamped tfStamped;
  geometry_msgs::Pose gpsPose;
  try
  {
    tfStamped = this->TfBuffer.lookupTransform(this->UtmFrameId, msg.header.frame_id,
                                               msg.header.stamp, ros::Duration(1.));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::doTransform(msg.pose.pose, gpsPose, tfStamped);

  // Get UTM zone/band
  // TODO Maybe refresh zone/band only if a big gap is detected in odometry pose
  if (!ros::param::getCached("/utm_zone", this->UtmZoneNumber) ||
      !ros::param::getCached("/utm_band", this->UtmBandLetter))
  {
    ROS_ERROR_STREAM("UTM zone/band is unset in rosparam server.");
    return;
  }

  // Convert to GPS Lat/Lon/Alt WGS84 format
  geodesy::UTMPoint utmPoint(gpsPose.position.x,
                             gpsPose.position.y,
                             gpsPose.position.z,
                             (uint8_t) this->UtmZoneNumber,
                             this->UtmBandLetter[0]);
  geographic_msgs::GeoPoint gpsPoint = geodesy::toMsg(utmPoint);

  // Fill and send NavSatFix msg
  sensor_msgs::NavSatFix navSatFix;
  navSatFix.header = msg.header;
  navSatFix.header.frame_id = msg.child_frame_id;
  navSatFix.altitude = gpsPoint.altitude;
  navSatFix.longitude = gpsPoint.longitude;
  navSatFix.latitude = gpsPoint.latitude;
  navSatFix.status.status = navSatFix.status.STATUS_FIX;  // STATUS_NO_FIX ?
  navSatFix.position_covariance_type = navSatFix.COVARIANCE_TYPE_APPROXIMATED;  // COVARIANCE_TYPE_KNOWN ?
  const boost::array<double, 36>& c = msg.pose.covariance;
  navSatFix.position_covariance = {c[ 0], c[ 1], c[ 2],
                                   c[ 6], c[ 7], c[ 8],
                                   c[12], c[13], c[14]};
  this->GpsFixPub.publish(navSatFix);
}

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  // Init ROS node.
  ros::init(argc, argv, "odom_to_gps");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create node
  OdomToGpsNode node(nh, priv_nh);

  // Handle callbacks until shut down.
  ros::spin();

  return 0;
}