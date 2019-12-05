#include "GpsToUtmNode.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

//------------------------------------------------------------------------------
/*!
 * @brief Convert True bearing angle in degrees to ENU heading angle in radians.
 * @param trueBearingDegrees True bearing angle (clockwise, 0 = north) in degrees.
 * @return ENU heading angle (counter-clockwise, 0 = east) in radians.
 */
inline double trueDegToEnuRad(double trueBearingDegrees)
{
 return (90 - trueBearingDegrees) * M_PI / 180.;
}

//------------------------------------------------------------------------------
GpsToUtmNode::GpsToUtmNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Get parameters from ROS param server
  priv_nh.getParam("frame_id", this->FrameId);
  priv_nh.getParam("child_frame_id", this->ChildFrameId);
  priv_nh.getParam("map_frame_id", this->MapFrameId);
  priv_nh.getParam("tf_offset_to_map", this->PublishTfToMap);
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
  UtmPose utmPose(utmPoint, msg.track);

  // Normal case, if heading is defined.
  if (msg.track)
    this->ProcessUtmPose(msg, utmPose);

  // If heading is not defined, we will compute it from movement with next GPS position.
  else
  {
    ROS_WARN_STREAM_ONCE("Guessing GPS heading from movement.");

    if (this->PreviousGpsPose.isValid())
    {
      // Check that previous pose is no too old to be used, otherwise reset buffer and exit.
      if (std::abs((msg.header.stamp - this->PreviousMsg.header.stamp).toSec()) > 1.)
      {
        ROS_WARN_STREAM("Time jump detected : resetting GPS bearing guess from movement.");
        // Save current point for next step.
        this->PreviousMsg = msg;
        this->PreviousGpsPose = utmPose;
        return;
      }

      // Compute bearing from movement
      double heading = atan2(utmPose.northing - this->PreviousGpsPose.northing,
                             utmPose.easting - this->PreviousGpsPose.easting);
      double bearing = 90 - heading * 180. / M_PI;

      // Smooth bearing according to distance moved to avoid oscillations at low speed.
      // We estimate the computed bearing to be precise enough if we moved at least 0.5 meter from previous position.
      double distance = std::sqrt(pow(utmPose.northing - this->PreviousGpsPose.northing, 2) +
                                  pow(utmPose.easting - this->PreviousGpsPose.easting, 2));
      double innovation = std::min(distance / 0.5, 1.);
      double previousBearing = this->PreviousGpsPose.bearing ? this->PreviousGpsPose.bearing : bearing;
      this->PreviousGpsPose.bearing = innovation * bearing + (1 - innovation) * previousBearing;

      // Save current bearing for next step smoothing
      utmPose.bearing = this->PreviousGpsPose.bearing;

      // Process the completed previous pose
      this->ProcessUtmPose(this->PreviousMsg, this->PreviousGpsPose);
    }

    // Save current point for next step.
    this->PreviousMsg = msg;
    this->PreviousGpsPose = utmPose;
  }
}

//------------------------------------------------------------------------------
void GpsToUtmNode::ProcessUtmPose(const gps_common::GPSFix& msg, const UtmPose& utmPose)
{
  // Store UTM zone/band and publish it to rosparam if it changed
  if ((utmPose.zone != this->UtmZone) || (utmPose.band != this->UtmBand))
  {
    this->UtmZone = utmPose.zone;
    this->UtmBand = utmPose.band;
    std::string utmBandLetter(&this->UtmBand, 1);
    ros::param::set("/utm_zone", (int) this->UtmZone);
    ros::param::set("/utm_band", utmBandLetter);
    ROS_WARN_STREAM("UTM zone/band changed to " << (int) utmPose.zone << utmPose.band << " and saved to rosparam.");
  }

  // Save 1st GPS pose
  if (!this->FirstGpsPose.isValid())
    this->FirstGpsPose = utmPose;

  // Fill odometry header
  nav_msgs::Odometry odomMsg;
  odomMsg.header = msg.header;
  odomMsg.header.stamp += ros::Duration(this->TimeOffset);
  if (!this->FrameId.empty())
    odomMsg.header.frame_id = this->FrameId;
  if (this->OriginOnFirstPose)
    odomMsg.header.frame_id = this->MapFrameId;
  if (!this->ChildFrameId.empty())
    odomMsg.child_frame_id = this->ChildFrameId;

  // Set pose and speed
  // TODO use 3D orientation from input msg
  if (this->OriginOnFirstPose)
  {
    double firstTrueHeading = trueDegToEnuRad(this->FirstGpsPose.bearing);
    double dHeading = trueDegToEnuRad(utmPose.bearing) - firstTrueHeading,
           dEasting = utmPose.easting - this->FirstGpsPose.easting,
           dNorthing = utmPose.northing - this->FirstGpsPose.northing;
    odomMsg.pose.pose.position.x =  dEasting * cos(firstTrueHeading) + dNorthing * sin(firstTrueHeading);
    odomMsg.pose.pose.position.y = -dEasting * sin(firstTrueHeading) + dNorthing * cos(firstTrueHeading);
    odomMsg.pose.pose.position.z = utmPose.altitude  - this->FirstGpsPose.altitude;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(dHeading);
  }
  else
  {
    odomMsg.pose.pose.position.x = utmPose.easting;
    odomMsg.pose.pose.position.y = utmPose.northing;
    odomMsg.pose.pose.position.z = utmPose.altitude;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(trueDegToEnuRad(utmPose.bearing));
  }
  odomMsg.twist.twist.linear.x = msg.speed;

  // Fill covariance from msg
  const boost::array<double, 9>& c = msg.position_covariance;
  odomMsg.pose.covariance = {c[0], c[1], c[2],   0, 0, 0,
                             c[3], c[4], c[5],   0, 0, 0,
                             c[6], c[7], c[8],   0, 0, 0,

                             0, 0, 0,   pow(msg.err_roll / 2, 2), 0, 0,
                             0, 0, 0,   0, pow(msg.err_pitch / 2, 2), 0,
                             0, 0, 0,   0, 0, pow(msg.err_track / 2, 2)};

  this->UtmPosePub.publish(odomMsg);

  // Publish TF
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header = odomMsg.header;
  tfStamped.child_frame_id = odomMsg.child_frame_id;
  tfStamped.transform.translation.x = odomMsg.pose.pose.position.x;
  tfStamped.transform.translation.y = odomMsg.pose.pose.position.y;
  tfStamped.transform.translation.z = odomMsg.pose.pose.position.z;
  tfStamped.transform.rotation = odomMsg.pose.pose.orientation;
  this->TfBroadcaster.sendTransform(tfStamped);

  // Publish static TF to fit 1st GPS pose to local map if needed
  if (this->PublishTfToMap)
  {
    // Fill TF header
    tfStamped.header.frame_id = this->FrameId;
    tfStamped.child_frame_id = this->MapFrameId;
  
    // Set pose and speed
    tfStamped.transform.translation.x = this->FirstGpsPose.easting;
    tfStamped.transform.translation.y = this->FirstGpsPose.northing;
    tfStamped.transform.translation.z = this->FirstGpsPose.altitude;
    tfStamped.transform.rotation = tf::createQuaternionMsgFromYaw(trueDegToEnuRad(this->FirstGpsPose.bearing));

    // Send TF
    this->StaticTfBroadcaster.sendTransform(tfStamped);
    this->PublishTfToMap = false;  // this static TF must be sent only once.
    ROS_INFO_STREAM("Static TF sent from '" << tfStamped.header.frame_id 
                                << "' to '" << tfStamped.child_frame_id << "'.");
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