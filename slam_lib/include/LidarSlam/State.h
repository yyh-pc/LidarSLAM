//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2021-11-10
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

#include <Eigen/Geometry>
#include "LidarSlam/PointCloudStorage.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/Enums.h"
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{
// Structure containing one state
// with all local information
struct LidarState
{
  using Point = LidarPoint;
  using PCStoragePtr = std::shared_ptr<PointCloudStorage<Point>>;

  // Transform to go from Sensor frame to tracking frame
  Eigen::UnalignedIsometry3d BaseToSensor = Eigen::UnalignedIsometry3d::Identity();
  // Pose transform in world coordinates
  Eigen::UnalignedIsometry3d Isometry = Eigen::UnalignedIsometry3d::Identity();
  // [s] Timestamp of data
  double Time = 0.;
  // Covariance of current pose
  Eigen::Matrix6d Covariance = Utils::CreateDefaultCovariance();
  // Index to link the pose graph (G2O)
  unsigned int Index = 0;

  LidarState() = default;
  LidarState(const Eigen::UnalignedIsometry3d& isometry, double time = 0.,
             const Eigen::Matrix6d& covariance = Utils::CreateDefaultCovariance())
    : Isometry(isometry),
      Time(time),
      Covariance(covariance) {};
  // Keypoints extracted at current pose, undistorted and expressed in BASE coordinates
  std::map<Keypoint, PCStoragePtr> Keypoints;
  // Keypoints extracted at current pose and expressed in BASE coordinates
  std::map<Keypoint, PCStoragePtr> RawKeypoints;
  bool IsKeyFrame = true;
  friend std::ostream& operator<<(std::ostream& os, const LidarState& state)
  {
    os << std::fixed << std::setprecision(9) << state.Time << std::scientific << ",";
    Eigen::Vector3d t = state.Isometry.translation();
    os << t(0) << "," << t(1) << "," << t(2) << ",";
    Eigen::Matrix3d rot = state.Isometry.linear();
    os << rot(0,0) << "," << rot(1,0) << "," << rot(2,0) << ","
       << rot(0,1) << "," << rot(1,1) << "," << rot(2,1) << ","
       << rot(0,2) << "," << rot(1,2) << "," << rot(2,2) << "\n";
    return os;
  }
};

struct PoseStamped
{
  Eigen::UnalignedIsometry3d Pose = Eigen::UnalignedIsometry3d::Identity();
  double Time = 0.;

  PoseStamped() = default;
  PoseStamped(const Eigen::UnalignedIsometry3d& pose, double time): Pose(pose), Time(time){}
};

} // end of LidarSlam namespace