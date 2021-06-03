//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Julia Sanchez (Kitware SAS)
// Creation date: 2021-06-01
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

#pragma once

// LOCAL
#include "LidarSlam/KDTreePCLAdaptor.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/Enums.h"

// PCL
#include <pcl/point_cloud.h>

namespace LidarSlam
{

namespace Confidence
{

using Point = LidarPoint;
using PointCloud = pcl::PointCloud<Point>;
using KDTree = KDTreePCLAdaptor<Point>;

// Compute the LCP estimator (overlap estimator) for the registration of the points of cloud
// onto some target stored in kdTrees relatively to the leafSizes of the targetted point cloud
// It corresponds to the number of points from cloud which have a neighbor in the target
// relatively to the leafSizes thresholds
// (see http://geometry.cs.ucl.ac.uk/projects/2014/super4PCS/ for more info)
// In this LCP extension, we also check the distance between nearest neighbors to make a smooth estimator
float LCPEstimator(PointCloud::ConstPtr cloud, const std::map<Keypoint, KDTree>& kdTrees, const std::map<Keypoint, float>& leafSizes, int nbThreads = 1);

} // enf of Confidence namespace

} // end of LidarSlam namespace