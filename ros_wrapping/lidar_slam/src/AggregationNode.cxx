//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2022-08-26
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

#include "AggregationNode.h"

#include <LidarSlam/Utilities.h>

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
AggregationNode::AggregationNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  // ***************************************************************************
  // Init ROS publisher
  // aggregated points with specified density
  this->PointsPublisher = this->Nh.advertise<CloudS>("aggregated_cloud", 10, false);

  // Init ROS subscriber
  // Lidar frame undistorted
  this->FrameSubscriber = this->Nh.subscribe("slam_registered_points", 1, &AggregationNode::Callback, this);

  // Init rolling grid with parameters
  this->DenseMap = std::make_shared<LidarSlam::RollingGrid>();
  // Set maps parameters
  this->DenseMap->SetVoxelResolution(5.);
  this->DenseMap->SetGridSize(500);
  this->DenseMap->SetLeafSize(0.1);


  ROS_INFO_STREAM("Aggregation node is ready !");
}

//------------------------------------------------------------------------------
void AggregationNode::Callback(const CloudS::Ptr registeredCloud)
{
  // Aggregated points from all frames
  this->DenseMap->Add(registeredCloud, true);
  CloudS::Ptr aggregatedCloud = this->DenseMap->Get(true);
  aggregatedCloud->header = registeredCloud->header;
  // Publish them
  this->PointsPublisher.publish(aggregatedCloud);
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "aggregation");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  AggregationNode slam(nh, priv_nh);

  // Handle callbacks until shut down
  ros::spin();

  return 0;
}