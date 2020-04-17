//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-11-18
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

#include "GpsToUtmNode.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace
{
  //! @brief Convert degrees angle to radians.
  inline constexpr double Deg2Rad(double deg) { return deg * M_PI / 180.; }

  /*!
  * @brief Convert True bearing angle in degrees to ENU heading angle in radians.
  * @param trueBearingDegrees True bearing angle (clockwise, 0 = north) in degrees.
  * @return ENU heading angle (counter-clockwise, 0 = east) in radians.
  */
  inline double TrueDegToEnuRad(double trueBearingDegrees) {return Deg2Rad(90 - trueBearingDegrees);}

  //! @brief Check if UTMPose is valid or not
  inline bool IsUtmPosValid(const geodesy::UTMPose& utm) { return utm.position.northing || utm.position.easting; }

  //! @brief Convert RMS error at 95% confidence to variance.
  inline double Rms95ToVar(double rms95) { return rms95 * rms95 / 4; }

  //! @brief Smooth value according to distance moved to avoid oscillations at low speed.
  inline double SmoothWithDistance(double newValue, double previousValue, double distance)
  {
    // We estimate the new value to be precise enough if we moved at least 0.5 meter from previous position.
    double innovation = std::min(distance / 0.5, 1.);
    return innovation * newValue + (1 - innovation) * previousValue;
  }

  //! @brief Extract roll, pitch and yaw angles from quaternion msg
  inline void QuaternionMsgToRPY(const geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw)
  {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  //! @brief Check if quaternion msg is valid
  inline bool IsValidQuaternionMsg(const geometry_msgs::Quaternion& q)
  {
    return std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.) < tf::QUATERNION_TOLERANCE;
  }

  template<typename T, typename U>
  inline void FillXYZfromXYZ(T& out, const U& in) { out.x = in.x; out.y = in.y; out.z = in.z; }

  template<typename T, typename U>
  inline void FillXYZfromENU(T& out, const U& in) { out.x = in.easting; out.y = in.northing; out.z = in.altitude; }
}

//------------------------------------------------------------------------------
GpsToUtmNode::GpsToUtmNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Get parameters from ROS param server
  priv_nh.getParam("utm_frame_id", this->UtmFrameId);
  priv_nh.getParam("local_enu_frame_id", this->LocalEnuFrameId);
  priv_nh.getParam("local_map_frame_id", this->LocalMapFrameId);
  priv_nh.getParam("child_frame_id", this->ChildFrameId);
  priv_nh.getParam("publish_local_map_tf", this->PublishLocalMapTf);
  priv_nh.getParam("origin_on_first_pose", this->OriginOnFirstPose);
  priv_nh.getParam("time_offset", this->TimeOffset);

  // Init ROS publishers & subscribers
  this->UtmPosePub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);
  this->GpsPoseSub = nh.subscribe("gps_fix", 10, &GpsToUtmNode::GpsPoseCallback, this);

  ROS_INFO_STREAM("\033[1;32mGPS to UTM converter is ready !\033[0m");
}

//------------------------------------------------------------------------------
void GpsToUtmNode::GpsPoseCallback(const gps_common::GPSFix& msg)
{
  // Convert (lat, lon, alt) to (X, Y, Z)
  geographic_msgs::GeoPoint gpsPoint;
  gpsPoint.latitude = msg.latitude;
  gpsPoint.longitude = msg.longitude;
  gpsPoint.altitude = msg.altitude;
  geodesy::UTMPoint utmPoint(gpsPoint);
  geodesy::UTMPose utmPose(utmPoint, geometry_msgs::Quaternion());

  // Normal case, if full orientation is defined.
  if (msg.roll || msg.pitch || msg.dip)
  {
    utmPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Deg2Rad(msg.roll), Deg2Rad(msg.pitch), Deg2Rad(msg.dip));
    this->ProcessUtmPose(msg, utmPose);
  }

  // If only bearing angle is defined.
  else if (msg.track)
  {
    utmPose.orientation = tf::createQuaternionMsgFromYaw(TrueDegToEnuRad(msg.track));
    this->ProcessUtmPose(msg, utmPose);
  }

  // If orientation is not defined, we will compute heading and pitch from motion
  // with next GPS position.
  else
  {
    // Check that previous pose is no too old to be used, otherwise reset buffer and exit.
    if (std::abs((msg.header.stamp - this->PreviousMsg.header.stamp).toSec()) > 1.)
    {
      ROS_WARN_STREAM("Time jump detected : reseting GPS orientation guess from motion.");
    }

    // Otherwise, if previous point is valid, use it to compute orientation
    else if (IsUtmPosValid(this->PreviousGpsPose))
    {
      ROS_WARN_STREAM_ONCE("Guessing GPS heading from movement.");

      // Translation motion since last pose
      double dE = utmPose.position.easting - this->PreviousGpsPose.position.easting;
      double dN = utmPose.position.northing - this->PreviousGpsPose.position.northing;
      double dU = utmPose.position.altitude - this->PreviousGpsPose.position.altitude;
      double d = std::sqrt(dE * dE + dN * dN + dU * dU);

      // Compute heading and pitch from motion (ENU coordinates)
      double heading = std::atan2(dN, dE);
      double pitch = -std::asin(dU / d);

      // Smooth angles with previous orientations if available
      if (IsValidQuaternionMsg(this->PreviousGpsPose.orientation))
      {
        double prevRoll, prevPitch, prevYaw;
        QuaternionMsgToRPY(this->PreviousGpsPose.orientation, prevRoll, prevPitch, prevYaw);
        // Smooth angles according to distance moved to avoid oscillations at low speed.
        heading = SmoothWithDistance(heading, prevYaw, d);
        pitch = SmoothWithDistance(pitch, prevPitch, d);
      }

      // Update orientation with smoothed values
      this->PreviousGpsPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch, heading);

      // Save computed orientation for next smoothing step
      utmPose.orientation = this->PreviousGpsPose.orientation;

      // Process the completed previous pose
      this->ProcessUtmPose(this->PreviousMsg, this->PreviousGpsPose);
    }

    // Save current point for next step.
    this->PreviousMsg = msg;
    this->PreviousGpsPose = utmPose;
  }
}

//------------------------------------------------------------------------------
void GpsToUtmNode::ProcessUtmPose(const gps_common::GPSFix& msg, const geodesy::UTMPose& utmPose)
{
  // Store UTM zone/band and publish it to rosparam if it changed
  if ((utmPose.position.zone != this->UtmZone) || (utmPose.position.band != this->UtmBand))
  {
    this->UtmZone = utmPose.position.zone;
    this->UtmBand = utmPose.position.band;
    std::string utmBandLetter(&this->UtmBand, 1);
    ros::param::set("utm_zone", (int) this->UtmZone);
    ros::param::set("utm_band", utmBandLetter);
    ROS_WARN_STREAM("UTM zone/band changed to " << (int) utmPose.position.zone << utmPose.position.band << " and saved to rosparam.");
  }

  // Save 1st GPS pose
  if (!IsUtmPosValid(this->FirstGpsPose))
    this->FirstGpsPose = utmPose;

  // Fill odometry header
  nav_msgs::Odometry odomMsg;
  odomMsg.header.stamp = msg.header.stamp + ros::Duration(this->TimeOffset);
  // Frame id : UTM or local map
  odomMsg.header.frame_id = this->OriginOnFirstPose ? this->LocalMapFrameId : this->UtmFrameId;
  // Child frame id : original name or given one
  odomMsg.child_frame_id = this->ChildFrameId.empty() ? msg.header.frame_id : this->ChildFrameId;

  // Set pose
  if (this->OriginOnFirstPose)
  {
    // Transform from local ENU frame to 1st GPS pose frame (= local map)
    tf::Quaternion Hquat; tf::quaternionMsgToTF(this->FirstGpsPose.orientation, Hquat);
    tf::Transform H(Hquat.inverse());

    // Apply transform to current GPS pose expressed in local ENU frame
    tf::Vector3 trans(utmPose.position.easting - this->FirstGpsPose.position.easting,
                      utmPose.position.northing - this->FirstGpsPose.position.northing,
                      utmPose.position.altitude - this->FirstGpsPose.position.altitude);
    tf::Quaternion rot; tf::quaternionMsgToTF(utmPose.orientation, rot);
    tf::Transform Xenu(rot, trans);
    tf::Transform Xmap = H * Xenu;

    // Convert to ROS msg
    geometry_msgs::Transform tfMsg; tf::transformTFToMsg(Xmap, tfMsg);
    FillXYZfromXYZ(odomMsg.pose.pose.position, tfMsg.translation);
    odomMsg.pose.pose.orientation = tfMsg.rotation;
  }
  else
  {
    FillXYZfromENU(odomMsg.pose.pose.position, utmPose.position);
    odomMsg.pose.pose.orientation = utmPose.orientation;
  }

  // Fill pose covariance from msg
  const boost::array<double, 9>& c = msg.position_covariance;
  odomMsg.pose.covariance = {c[0], c[1], c[2],   0, 0, 0,
                             c[3], c[4], c[5],   0, 0, 0,
                             c[6], c[7], c[8],   0, 0, 0,

                             0, 0, 0,   Rms95ToVar(msg.err_roll),  0, 0,
                             0, 0, 0,   0, Rms95ToVar(msg.err_pitch), 0,
                             0, 0, 0,   0, 0, Rms95ToVar(msg.err_dip)};

  // Set speed
  odomMsg.twist.twist.linear.x = msg.speed;
  odomMsg.twist.twist.linear.z = msg.climb;
  odomMsg.twist.covariance[0] = Rms95ToVar(msg.err_speed);
  odomMsg.twist.covariance[14] = Rms95ToVar(msg.err_climb);

  this->UtmPosePub.publish(odomMsg);

  // Publish TF
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header = odomMsg.header;
  tfStamped.child_frame_id = odomMsg.child_frame_id;
  FillXYZfromXYZ(tfStamped.transform.translation, odomMsg.pose.pose.position);
  tfStamped.transform.rotation = odomMsg.pose.pose.orientation;
  this->TfBroadcaster.sendTransform(tfStamped);

  // Publish static TF to fit 1st GPS pose to local map if needed
  if (this->PublishLocalMapTf)
  {
    // Send transform from UTM to local ENU frame
    tfStamped.header.frame_id = this->UtmFrameId;
    tfStamped.child_frame_id = this->LocalEnuFrameId;
    FillXYZfromENU(tfStamped.transform.translation, this->FirstGpsPose.position);
    tfStamped.transform.rotation = tf::createQuaternionMsgFromYaw(0);  // identity rotation
    this->StaticTfBroadcaster.sendTransform(tfStamped);

    // Send transform from 1st GPS pose (local map) to current GPS pose
    tfStamped.header.frame_id = this->UtmFrameId;
    tfStamped.child_frame_id = this->LocalMapFrameId;
    FillXYZfromENU(tfStamped.transform.translation, this->FirstGpsPose.position);
    tfStamped.transform.rotation = this->FirstGpsPose.orientation;
    this->StaticTfBroadcaster.sendTransform(tfStamped);

    this->PublishLocalMapTf = false;  // these static TF must be sent only once.
    ROS_INFO_STREAM("Static TF sent from '" << this->UtmFrameId << "' to '" << this->LocalEnuFrameId << "'.");
    ROS_INFO_STREAM("Static TF sent from '" << this->UtmFrameId << "' to '" << this->LocalMapFrameId << "'.");
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  // Init ROS node.
  ros::init(argc, argv, "gps_to_utm");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create node
  GpsToUtmNode node(nh, priv_nh);

  // Handle callbacks until shut down.
  ros::spin();

  return 0;
}