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
#include <LidarSlam/PointCloudStorage.h>

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/filesystem.hpp>

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

  // Init service
  this->SaveService = nh.advertiseService("lidar_slam/save_pc", &AggregationNode::SavePointcloudService, this);

  // Init rolling grid with parameters
  this->DenseMap = std::make_shared<LidarSlam::RollingGrid>();
  // Voxel size
  float leafSize = this->PrivNh.param("leaf_size", 0.1);
  this->DenseMap->SetLeafSize(leafSize);
  // Maximum size in voxels -> The second dimension of
  // the rolling grid is not needed in this context
  this->DenseMap->SetGridSize(3);
  float maxSize = this->PrivNh.param("max_size", 200.) / 3.;
  this->DenseMap->SetVoxelResolution(maxSize);
  // Min number of frames seeing a voxel to extract it
  int minNbPointsPerVoxel = this->PrivNh.param("min_points_per_voxel", 2);
  this->DenseMap->SetMinFramesPerVoxel(minNbPointsPerVoxel);

  ROS_INFO_STREAM("Aggregation node is ready !");
}

//------------------------------------------------------------------------------
void AggregationNode::Callback(const CloudS::Ptr registeredCloud)
{
  // Aggregated points from all frames
  this->DenseMap->Add(registeredCloud, true);
  this->Pointcloud = this->DenseMap->Get(true);
  this->Pointcloud->header = registeredCloud->header;
  // Publish them
  this->PointsPublisher.publish(this->Pointcloud);
}

//------------------------------------------------------------------------------
bool AggregationNode::SavePointcloudService(lidar_slam::save_pcRequest& req, lidar_slam::save_pcResponse& res)
{
  std::string outputPrefix = req.output_prefix_path.empty()? std::getenv("HOME") : req.output_prefix_path;
  boost::filesystem::path outputPrefixPath(outputPrefix);
  if (!boost::filesystem::exists(outputPrefixPath.parent_path()))
  {
    ROS_WARN_STREAM("Output folder does not exist, saving to home folder :" << std::getenv("HOME"));
    outputPrefixPath = boost::filesystem::path(std::getenv("HOME")) / boost::filesystem::path(outputPrefixPath.stem());
  }

  if (req.format > 2 || req.format < 0)
    req.format = 0;
  LidarSlam::PCDFormat f = static_cast<LidarSlam::PCDFormat>(req.format);
  std::cout << ros::Time().toSec() << std::endl;
  std::string outputFilePath = outputPrefixPath.string() + "_" + std::to_string(int(ros::Time::now().toSec())) + ".pcd";
  LidarSlam::savePointCloudToPCD<PointS>(outputFilePath, *this->Pointcloud, f);
  ROS_INFO_STREAM("Pointcloud saved to " << outputFilePath);
  res.success = true;
  return true;
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