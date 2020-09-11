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
#include <gps_common/GPSFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace
{
  //! @brief Convert degrees angle to radians.
  inline double Rad2Deg(double rad) { return rad * 180. / M_PI; }

  //! @brief Convert variance to RMS error at 95% confidence.
  inline double VarToRms95(double var) { return std::sqrt(var) * 2.; }
}

//------------------------------------------------------------------------------
OdomToGpsNode::OdomToGpsNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : TfListener(TfBuffer)
{
  // Get parameters from ROS param server
  priv_nh.getParam("utm_frame_id", this->UtmFrameId);
  priv_nh.getParamCached("/utm_zone", this->UtmZoneNumber);
  priv_nh.getParamCached("/utm_band", this->UtmBandLetter);

  // Init ROS publishers & subscribers
  this->GpsFixPub = nh.advertise<gps_common::GPSFix>("fix", 10);
  this->OdomSub = nh.subscribe("odom", 10, &OdomToGpsNode::OdometryCallback, this);

  ROS_INFO_STREAM("\033[1;32mOdom to GPS converter is ready !\033[0m");
}

//------------------------------------------------------------------------------
void OdomToGpsNode::OdometryCallback(const nav_msgs::Odometry& msg)
{
  // Get transform between local odom frame and UTM frame
  geometry_msgs::TransformStamped tfStamped;
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

  // Transform local pose to UTM X/Y/Z coordinates
  geometry_msgs::PoseWithCovarianceStamped gpsPose;
  gpsPose.header = msg.header;
  gpsPose.header.frame_id = this->UtmFrameId;
  tf2::doTransform(msg.pose.pose, gpsPose.pose.pose, tfStamped);
  tf2::Transform t;
  tf2::fromMsg(tfStamped.transform, t);
  gpsPose.pose.covariance = tf2::transformCovariance(msg.pose.covariance, t);

  // Get UTM zone/band
  // TODO Maybe refresh zone/band only if a big gap is detected in odometry pose
  if (!ros::param::getCached("/utm_zone", this->UtmZoneNumber) ||
      !ros::param::getCached("/utm_band", this->UtmBandLetter))
  {
    ROS_ERROR_STREAM("UTM zone/band is unset in rosparam server.");
    return;
  }

  // Convert to GPS Lat/Lon/Alt WGS84 format
  geodesy::UTMPoint utmPoint(gpsPose.pose.pose.position.x,
                             gpsPose.pose.pose.position.y,
                             gpsPose.pose.pose.position.z,
                             (uint8_t) this->UtmZoneNumber,
                             this->UtmBandLetter[0]);
  geographic_msgs::GeoPoint gpsPoint = geodesy::toMsg(utmPoint);

  // Fill and send gps_common::GPSFix msg

  // Header and status
  gps_common::GPSFix gpsFix;
  gpsFix.header.stamp = msg.header.stamp;
  gpsFix.header.frame_id = msg.child_frame_id;
  gpsFix.status.header = gpsFix.header;
  gpsFix.status.status = gpsFix.status.STATUS_FIX;

  // Position (degrees, degrees, meters)
  gpsFix.altitude  = gpsPoint.altitude;
  gpsFix.longitude = gpsPoint.longitude;
  gpsFix.latitude  = gpsPoint.latitude;

  // Orientation in ENU frame (in degrees)
  tf2::Quaternion quatENU;
  tf2::fromMsg(gpsPose.pose.pose.orientation, quatENU);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quatENU).getRPY(roll, pitch, yaw);
  gpsFix.roll  = Rad2Deg(roll);
  gpsFix.pitch = Rad2Deg(pitch);
  gpsFix.dip   = Rad2Deg(yaw);     // ENU heading angle (counter-clockwise, 0 = east)
  gpsFix.track = 90 - gpsFix.dip;  // True bearing angle (clockwise, 0 = north)

  // Position covariance
  const boost::array<double, 36>& c = gpsPose.pose.covariance;
  gpsFix.position_covariance = {c[ 0], c[ 1], c[ 2],
                                c[ 6], c[ 7], c[ 8],
                                c[12], c[13], c[14]};
  gpsFix.position_covariance_type = gpsFix.COVARIANCE_TYPE_APPROXIMATED;  // COVARIANCE_TYPE_KNOWN ?

  // Position error in meters (95% uncertainty)
  gpsFix.err      = VarToRms95(c[0] + c[7] + c[14]);
  gpsFix.err_horz = VarToRms95(c[0] + c[7]);
  gpsFix.err_vert = VarToRms95(c[14]);

  // Orientation error in degrees (95% uncertainty)
  gpsFix.err_pitch = Rad2Deg(VarToRms95(c[21]));
  gpsFix.err_roll  = Rad2Deg(VarToRms95(c[28]));
  gpsFix.err_dip   = Rad2Deg(VarToRms95(c[35]));
  gpsFix.err_track = gpsFix.err_dip;

  // Set speed info if previous message is available
  if (this->PreviousGpsPose.header.stamp.toSec() > 0)
  {
    // Get useful alias
    double dt = gpsPose.header.stamp.toSec() - this->PreviousGpsPose.header.stamp.toSec();
    const auto& currPos = gpsPose.pose.pose.position;
    const auto& prevPos = this->PreviousGpsPose.pose.pose.position;

    // Estimate current speed
    gpsFix.speed = std::sqrt(std::pow(currPos.x - prevPos.x, 2) + std::pow(currPos.y - prevPos.y, 2)) / dt;
    gpsFix.climb = (currPos.z - prevPos.z) / dt;

    // Estimate current speed error
    // TODO : Use also gpsFix.err_time ? Use also previous position error ?
    gpsFix.err_speed = gpsFix.err_horz / dt;
    gpsFix.err_climb = gpsFix.err_vert / dt;
  }

  // Dilution Of Precision (not available)
  gpsFix.hdop = -1;
  gpsFix.gdop = -1;
  gpsFix.pdop = -1;
  gpsFix.vdop = -1;
  gpsFix.tdop = -1;

  this->GpsFixPub.publish(gpsFix);
  this->PreviousGpsPose = gpsPose;
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