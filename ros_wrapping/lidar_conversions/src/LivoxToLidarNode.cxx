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

#include "LivoxToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

LivoxToLidarNode::LivoxToLidarNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  //  Get LiDAR id
  this->PrivNh.param("device_id", this->DeviceId, this->DeviceId);

  // Init ROS publisher
  this->Talker = nh.advertise<CloudS>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = nh.subscribe("livox/lidar", 1, &LivoxToLidarNode::Callback, this);

  ROS_INFO_STREAM(BOLD_GREEN("Livox data converter is ready !"));
}

//------------------------------------------------------------------------------
void LivoxToLidarNode::Callback(const CloudL& cloudL)
{
  // If input cloud is empty, ignore it
  if (cloudL.empty())
  {
    ROS_ERROR_STREAM("Input Livox pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudL.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudL, cloudS);

  // Helper to estimate frameAdvancement in case time field is invalid
  Utils::SpinningFrameAdvancementEstimator frameAdvancementEstimator;

  // Build SLAM pointcloud
  double prevTime = -0.1;
  for (const PointL& livoxPoint : cloudL)
  {
    PointS slamPoint;
    slamPoint.x = livoxPoint.x;
    slamPoint.y = livoxPoint.y;
    slamPoint.z = livoxPoint.z;
    if (slamPoint.getVector3fMap().norm() < 1e-6)
      continue;
    slamPoint.intensity = livoxPoint.intensity;
    slamPoint.laser_id = 0;
    slamPoint.device_id = this->DeviceId;

    slamPoint.time = prevTime + 0.1/300000.; // Supposing 10 Hz and 300 000 points
    prevTime = slamPoint.time;

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
  ros::init(argc, argv, "livox_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_conversions::LivoxToLidarNode v2s(n, priv_nh);

  ros::spin();

  return 0;
}
