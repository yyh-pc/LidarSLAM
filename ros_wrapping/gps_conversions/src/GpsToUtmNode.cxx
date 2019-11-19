#include "GpsToUtmNode.h"

#include <geodesy/utm.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

//------------------------------------------------------------------------------
GpsToUtmNode::GpsToUtmNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Get parameters from ROS param server
  priv_nh.getParam("frame_id", this->FrameId);
  priv_nh.getParam("child_frame_id", this->ChildFrameId);
  priv_nh.getParam("map_frame_id", this->MapFrameId);
  priv_nh.getParam("tf_offset_to_map", this->PublishTfToMap);
  priv_nh.getParam("origin_on_first_pose", this->OriginOnFirstPose);

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

  // Store UTM zone/band and publish it to rosparam if it changed
  if ((utmPoint.zone != this->UtmZone) || (utmPoint.band != this->UtmBand))
  {
    this->UtmZone = utmPoint.zone;
    this->UtmBand = utmPoint.band;
    std::string utmBandLetter(&this->UtmBand, 1);
    ros::param::set("/utm_zone", (int) this->UtmZone);
    ros::param::set("/utm_band", utmBandLetter);
    ROS_WARN_STREAM("UTM zone/band changed to " << (int) utmPoint.zone << utmPoint.band << " and saved to rosparam.");
  }

  // Save 1st GPS pose
  if (!this->firstGpsPose.isValid())
    this->firstGpsPose = UtmPose(utmPoint.easting, utmPoint.northing, utmPoint.altitude, msg.track);

  // Fill odometry header
  nav_msgs::Odometry odomMsg;
  odomMsg.header = msg.header;
  odomMsg.header.stamp = ros::Time::now();  // TODO : use corrected msg time
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
    double firstTrueHeading = (90 - this->firstGpsPose.heading) * M_PI / 180.;
    double dHeading = (90 - msg.track) * M_PI / 180. - firstTrueHeading,
           dEasting = utmPoint.easting - this->firstGpsPose.easting,
           dNorthing = utmPoint.northing - this->firstGpsPose.northing;
    odomMsg.pose.pose.position.x =  dEasting * cos(firstTrueHeading) + dNorthing * sin(firstTrueHeading);
    odomMsg.pose.pose.position.y = -dEasting * sin(firstTrueHeading) + dNorthing * cos(firstTrueHeading);
    odomMsg.pose.pose.position.z = utmPoint.altitude  - this->firstGpsPose.altitude;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(dHeading);
  }
  else
  {
    odomMsg.pose.pose.position.x = utmPoint.easting;
    odomMsg.pose.pose.position.y = utmPoint.northing;
    odomMsg.pose.pose.position.z = utmPoint.altitude;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw((90 - msg.track) * M_PI / 180.);
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
    tfStamped.transform.translation.x = this->firstGpsPose.easting;
    tfStamped.transform.translation.y = this->firstGpsPose.northing;
    tfStamped.transform.translation.z = this->firstGpsPose.altitude;
    tfStamped.transform.rotation = tf::createQuaternionMsgFromYaw((90 - this->firstGpsPose.heading) * M_PI / 180.);

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