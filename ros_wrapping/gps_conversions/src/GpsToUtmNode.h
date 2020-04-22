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

#ifndef GPS_TO_UTM_NODE_H
#define GPS_TO_UTM_NODE_H

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <gps_common/GPSFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
   * @brief     GPS pose callback, projecting GPS (Lon/Lat/Alt) to UTM (X/Y/Z) and checking orientation.
   * @param[in] msg GPS pose or position with its associated covariance.
   */
  void GpsPoseCallback(const gps_common::GPSFix& msg);

private:

  //------------------------------------------------------------------------------
  void ProcessUtmPose(const gps_common::GPSFix& msg, const geodesy::UTMPose& utmPose);

  // ROS publishers & subscribers
  ros::Subscriber GpsPoseSub;
  ros::Publisher UtmPosePub;
  tf2_ros::TransformBroadcaster TfBroadcaster;
  tf2_ros::StaticTransformBroadcaster StaticTfBroadcaster;

  // UTM Band/zone
  char UtmBand;     ///< MGRS latitude band letter.
  uint8_t UtmZone;  ///< UTM longitude zone number.

  // Previous GPS poses storage for internal computation
  geodesy::UTMPose FirstGpsPose;        ///< 1st GPS pose received, only used if OriginOnFirstPose = true.
  geodesy::UTMPose PreviousGpsPose;     ///< Previous GPS pose received.
  gps_common::GPSFix PreviousMsg;       ///< Previous message received, only used if heading is not defined.

  // Parameters
  std::string UtmFrameId = "utm";       ///< UTM grid zone origin.
  std::string LocalEnuFrameId = "enu";  ///< ENU frame attached to 1st GPS position received.
  std::string LocalMapFrameId = "map";  ///< Local map frame attached to 1st GPS pose received.
  std::string ChildFrameId;             ///< Frame attached to moving system whose GPS poses are received (default : input GPSFix frame_id).
  double TimeOffset = 0.;               ///< Output odom time = GPS time + TimeOffset
  bool PublishGpsOdomTf = false;        ///< If true, publish also GPS odom as TF.
  bool PublishLocalMapTf = false;       ///< If true, publish 2 static TF from UtmFrameId to LocalEnuFrameId or LocalMapFrameId.
  bool OriginOnFirstPose = false;       ///< false: publish gps pose in UTM coordinates. true: publish pose in MapFrameId (= 1st GPS pose).
};

#endif // GPS_TO_UTM_NODE_H