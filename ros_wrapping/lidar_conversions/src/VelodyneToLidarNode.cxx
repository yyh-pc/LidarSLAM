//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2020-12-10
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

#include "VelodyneToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

VelodyneToLidarNode::VelodyneToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // Get laser ID mapping
  this->PrivNh.param("laser_id_mapping", this->LaserIdMapping, this->LaserIdMapping);

  //  Get LiDAR id
  this->PrivNh.param("device_id", this->DeviceId, this->DeviceId);

  //  Get LiDAR spinning speed and first timestamp option
  this->PrivNh.param("rpm", this->Rpm, this->Rpm);
  this->PrivNh.param("timestamp_first_packet", this->TimestampFirstPacket, this->TimestampFirstPacket);

  // Init ROS publisher
  this->Talker = nh.advertise<CloudS>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = nh.subscribe("velodyne_points", 1, &VelodyneToLidarNode::Callback, this);

  ROS_INFO_STREAM(BOLD_GREEN("Velodyne data converter is ready !"));
}

//------------------------------------------------------------------------------
void VelodyneToLidarNode::Callback(const CloudV& cloudV)
{
  // If input cloud is empty, ignore it
  if (cloudV.empty())
  {
    ROS_ERROR_STREAM("Input Velodyne pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudV.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudV, cloudS);

  // Check wether to use custom laser ID mapping or leave it untouched
  bool useLaserIdMapping = !this->LaserIdMapping.empty();

  // Check if time field looks properly set
  // If first and last points have same timestamps, this is not normal
  bool isTimeValid = cloudV.back().time - cloudV.front().time > 1e-8;
  if (!isTimeValid)
    ROS_WARN_STREAM("Invalid 'time' field, it will be built from azimuth advancement.");

  // Helper to estimate frameAdvancement in case time field is invalid
  Utils::SpinningFrameAdvancementEstimator frameAdvancementEstimator;

  // Build SLAM pointcloud
  for (const PointV& velodynePoint : cloudV)
  {
    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laser_id = useLaserIdMapping ? this->LaserIdMapping[velodynePoint.ring] : velodynePoint.ring;
    slamPoint.device_id = this->DeviceId;

    // Use time field if available
    // time is the offset to add to header.stamp to get point-wise timestamp
    if (isTimeValid)
      slamPoint.time = velodynePoint.time;

    // Build approximate point-wise timestamp from azimuth angle
    // 'frameAdvancement' is 0 for first point, and should match 1 for last point
    // for a 360 degrees scan at ideal spinning frequency.
    // 'time' is the offset to add to 'header.stamp' to get approximate point-wise timestamp.
    // By default, 'header.stamp' is the timestamp of the last Veloydne packet,
    // but user can choose the first packet timestamp using parameter 'timestamp_first_packet'.
    else
    {
      double frameAdvancement = frameAdvancementEstimator(slamPoint);
      slamPoint.time = (this->TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) / this->Rpm * 60.;
    }

    cloudS.push_back(slamPoint);
  }

  this->Talker.publish(cloudS);
}

}  // end of namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::VelodyneToLidarNode v2s(n, priv_nh);

  ros::spin();

  return 0;
}
