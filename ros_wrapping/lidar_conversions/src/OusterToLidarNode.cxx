//==============================================================================
// Copyright 2021-2022 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2022-05-10
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

#include "OusterToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

OusterToLidarNode::OusterToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
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
  this->Listener = nh.subscribe("ouster/points", 1, &OusterToLidarNode::Callback, this);

  ROS_INFO_STREAM(BOLD_GREEN("Ouster data converter is ready !"));
}

//------------------------------------------------------------------------------
void OusterToLidarNode::Callback(const CloudV& cloudO)
{
  // If input cloud is empty, ignore it
  if (cloudO.empty())
  {
    ROS_ERROR_STREAM("Input Velodyne pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudO.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudO, cloudS);

  // Check wether to use custom laser ID mapping or leave it untouched
  bool useLaserIdMapping = !this->LaserIdMapping.empty();

  // Helper to estimate frameAdvancement in case time field is invalid
  Utils::SpinningFrameAdvancementEstimator frameAdvancementEstimator;

  // Build SLAM pointcloud
  for (const PointO& ousterPoint : cloudO)
  {
    // Remove no return points
    if (ousterPoint.getVector3fMap().norm() < 1e-3)
      continue;

    PointS slamPoint;
    slamPoint.x = ousterPoint.x;
    slamPoint.y = ousterPoint.y;
    slamPoint.z = ousterPoint.z;
    slamPoint.intensity = ousterPoint.reflectivity;
    slamPoint.laser_id = useLaserIdMapping ? this->LaserIdMapping[ousterPoint.ring] : ousterPoint.ring;
    slamPoint.device_id = this->DeviceId;

    // Build approximate point-wise timestamp from azimuth angle
    // 'frameAdvancement' is 0 for first point, and should match 1 for last point
    // for a 360 degrees scan at ideal spinning frequency.
    // 'time' is the offset to add to 'header.stamp' to get approximate point-wise timestamp.
    // By default, 'header.stamp' is the timestamp of the last Veloydne packet,
    // but user can choose the first packet timestamp using parameter 'timestamp_first_packet'.
    double frameAdvancement = frameAdvancementEstimator(slamPoint);
    slamPoint.time = (this->TimestampFirstPacket ? frameAdvancement : frameAdvancement - 1) / this->Rpm * 60.;

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
  ros::init(argc, argv, "ouster_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::OusterToLidarNode v2s(n, priv_nh);

  ros::spin();

  return 0;
}
