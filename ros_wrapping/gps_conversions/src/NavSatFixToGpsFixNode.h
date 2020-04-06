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