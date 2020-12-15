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

#include <pcl_conversions/pcl_conversions.h>
#include "VelodyneToSlamNode.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

VelodyneToSlamNode::VelodyneToSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // Get LiDAR frequency
  this->PrivNh.param("lidar_frequency", this->LidarFreq, 10.0);
  // ***************************************************************************
  // Init ROS publisher
  this->Talker = nh.advertise<CloudS>("lidar_points", 1);

  // ***************************************************************************
  // Init ROS subscribers
  this->Listener = nh.subscribe("velodyne_points", 1, &VelodyneToSlamNode::Callback, this);

  ROS_INFO_STREAM(BOLD_GREEN("Velodyne data converter is ready !"));
}

//------------------------------------------------------------------------------
void VelodyneToSlamNode::Callback(const CloudV& cloudV)
{
  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudV.size());
  cloudS.header = cloudV.header;
  
  // Check if time field looks properly set
  // If first and last points have same timestamps, this is not normal
  
  bool isTimeValid = cloudV.back().time - cloudV.front().time > 1e-8;
  if (!isTimeValid)
    ROS_WARN_STREAM("Invalid 'time' field, it will be built from azimuth advancement.\n");

  // Helpers to estimate frameAdvancement in case time field is invalid
  auto wrapMax = [](double x, double max) {return std::fmod(max + std::fmod(x, max), max);};
  auto advancement = [](const PointV& velodynePoint) {return (M_PI - std::atan2(velodynePoint.y, velodynePoint.x)) / (2 * M_PI);};
  const double initAdvancement = advancement(cloudV.front());
  std::map<int, double> previousAdvancementPerRing;

  // Build SLAM pointcloud
  for(const PointV& velodynePoint: cloudV)
  {
    PointS slamPoint;

    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laser_id = velodynePoint.ring;
    slamPoint.device_id = 0;
    
    // Use time field is available
   // time is the offset to add to header.stamp to get point-wise timestamp
   if (isTimeValid)
     slamPoint.time = velodynePoint.time;

   // Try to build approximate timestamp from azimuth angle
   // time is 0 for first point, and should match LiDAR period for last point for a complete scan.
   else
   {
     // Get normalized angle (in [0-1]), with angle 0 being first point direction
     double frameAdvancement = advancement(velodynePoint);
     frameAdvancement = wrapMax(frameAdvancement - initAdvancement, 1.);
     // If we detect overflow, correct it
     // If current laser_id (ring) is not in map, the following line will insert it, associating it to value 0.0.
     if (frameAdvancement < previousAdvancementPerRing[velodynePoint.ring])
       frameAdvancement += 1;
     previousAdvancementPerRing[velodynePoint.ring] = frameAdvancement;
     slamPoint.time = frameAdvancement / this->LidarFreq;
   }
   
   cloudS.push_back(slamPoint);
  }

  this->Talker.publish(cloudS);
}
//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_conversion");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  VelodyneToSlamNode v2s(n, priv_nh);

  ros::spin();

  return 0;
}
