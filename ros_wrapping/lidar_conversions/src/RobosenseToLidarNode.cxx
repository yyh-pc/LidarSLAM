//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2020-12-22
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

#include "RobosenseToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{
namespace
{
  // Mapping between RSLidar laser id and vertical laser id
  // TODO add laser ID mappings for RS32, RSBPEARL and RSBPEARL_MINI ?
  const std::array<uint16_t, 16> LASER_ID_MAPPING_RS16 = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};
}

RobosenseToLidarNode::RobosenseToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // Get LiDAR frequency
  this->PrivNh.param("lidar_frequency", this->LidarFreq, 10.0);

  // Init ROS publisher
  this->Talker = nh.advertise<CloudS>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = nh.subscribe("rslidar_points", 1, &RobosenseToLidarNode::Callback, this);

  ROS_INFO_STREAM(BOLD_GREEN("RSLidar data converter is ready !"));
}

//------------------------------------------------------------------------------
void RobosenseToLidarNode::Callback(const CloudRS& cloudRS)
{
  // If input cloud is empty, ignore it
  if (cloudRS.empty())
  {
    ROS_ERROR_STREAM("Input RSLidar pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudRS.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudRS, cloudS);
  cloudS.is_dense = true;

  // Helper to estimate frame advancement
  Utils::SpinningFrameAdvancementEstimator frameAdvancementEstimator;

  // Build SLAM pointcloud
  for (unsigned int i = 0; i < cloudRS.size(); ++i)
  {
    const PointRS& rsPoint = cloudRS[i];

    // Check that input point does not have NaNs
    if (!Utils::IsFinite(rsPoint))
      continue;

    // In case of dual returns mode, check that the second return is not identical to the first
    // CHECK this operation for other sensors than RS16
    if (!cloudS.empty() && std::equal(rsPoint.data, rsPoint.data + 3, cloudS.back().data))
      continue;

    // Copy coordinates and intensity
    PointS slamPoint;
    slamPoint.x = rsPoint.x;
    slamPoint.y = rsPoint.y;
    slamPoint.z = rsPoint.z;
    slamPoint.intensity = rsPoint.intensity;

    // Compute laser ID
    // If we are using RS16 sensor, we need to correct the laser number
    // CHECK this operation for other sensors than RS16
    uint16_t laser_id = i / cloudRS.width;
    slamPoint.laser_id = (cloudRS.height == 16) ? LASER_ID_MAPPING_RS16[laser_id] : laser_id;

    // Build approximate timestamp from azimuth angle
    // time is 0 for first point, and should match LiDAR period for last point
    // for a 360 degrees scan.
    slamPoint.time = frameAdvancementEstimator(slamPoint) / this->LidarFreq;

    cloudS.push_back(slamPoint);
  }

  // Publish pointcloud only if non empty
  if (!cloudS.empty())
    this->Talker.publish(cloudS);
}

}  // end of namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rslidar_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::RobosenseToLidarNode rs2s(n, priv_nh);

  ros::spin();

  return 0;
}
