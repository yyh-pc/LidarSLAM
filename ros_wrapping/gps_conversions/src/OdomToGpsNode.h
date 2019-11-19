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