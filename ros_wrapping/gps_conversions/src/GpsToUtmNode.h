#ifndef GPS_TO_UTM_NODE_H
#define GPS_TO_UTM_NODE_H

#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

struct UtmPose
{
  double easting = 0.;      ///< [m] X direction, pointing east.
  double northing = 0.;     ///< [m] Y direction, pointing north.
  double altitude = 0.;     ///< [m] Z direction, pointing up.
  double heading = 0.;  ///< [deg] clockwise, 0 is north.

  UtmPose() = default;

  UtmPose(double easting_, double northing_, double altitude_, double heading_)
    : easting(easting_), northing(northing_), altitude(altitude_), heading(heading_)
  {}

  bool isValid() const { return (easting || northing || altitude || heading); }
};


class GpsToUtmNode
{
public:

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] nh      Public ROS node handle, used to init publisher/subscribers.
   * @param[in] priv_nh Private ROS node handle, used to access parameters.
   */
  GpsToUtmNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  //----------------------------------------------------------------------------
  /*!
   * @brief     GPS pose callback, projecting GPS (Lon/Lat/Alt) to UTM (X/Y/Z)
   * @param[in] msg New GPS position with its associated covariance.
   */
  void GpsPoseCallback(const gps_common::GPSFix& msg);

private:

  // ROS publishers & subscribers
  ros::Subscriber GpsPoseSub;
  ros::Publisher UtmPosePub;
  tf2_ros::TransformBroadcaster TfBroadcaster;
  tf2_ros::StaticTransformBroadcaster StaticTfBroadcaster;

  // Parameters
  std::string FrameId;
  std::string ChildFrameId;
  std::string MapFrameId = "map";

  bool PublishTfToMap = false;  ///< true: publish a static TF from FrameId to MapFrameId to match 1st GPS pose to local map. 
  bool OriginOnFirstPose = false;  ///< false: publish gps pose in UTM coordinates. true: publish pose in MapFrameId (= 1st GPS pose).
  UtmPose firstGpsPose;  ///< 1st GPS pose received, used only if OriginOnFirstPose = true.
};

#endif // GPS_TO_UTM_NODE_H