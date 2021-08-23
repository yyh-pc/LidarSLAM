//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Julia Sanchez (Kitware SAS)
// Creation date: 2021-03-15
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

#include "LidarSlam/SensorConstraints.h"

namespace LidarSlam
{
namespace SensorConstraints
{
// ---------------------------------------------------------------------------
bool WheelOdometryManager::ComputeConstraint(double lidarTime, bool verbose)
{
  this->ResetResidual();

  if (!this->CanBeUsed())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);

  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, verbose);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate odometry measurement at LiDAR timestamp
  double rt = (lidarTime - bounds.first->Time) / (bounds.second->Time - bounds.first->Time);
  double currDistance = (1 - rt) * bounds.first->Distance + rt * bounds.second->Distance;

  // Build odometry residual
  // If there is no memory of previous poses
  // The current measure is taken as reference
  if (this->RefDistance > 1e9)
  {
    if (verbose)
      PRINT_INFO("No previous wheel odometry measure : no constraint added to optimization")
    // Update reference distance for next frames
    this->RefDistance = currDistance;
    return false;
  }

  // If there is memory of a previous pose
  double distDiff = std::abs(currDistance - this->RefDistance);
  this->Residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(this->PreviousPose.translation(), distDiff);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  if(verbose && !this->Relative)
    PRINT_INFO("Adding absolute wheel odometry residual : " << distDiff << " m travelled since first frame.")
  if (verbose && this->Relative)
    PRINT_INFO("Adding relative wheel odometry residual : " << distDiff << " m travelled since last frame.")

  // Update reference distance if relative mode enabled
  if (this->Relative)
    this->RefDistance = currDistance;

  return true;
}

// ---------------------------------------------------------------------------
bool ImuManager::ComputeConstraint(double lidarTime, bool verbose)
{
  this->ResetResidual();

  if (!this->CanBeUsed())
    return false;

  // Compute reference gravity vector
  if (this->GravityRef.norm() < 1e-6)
    this->ComputeGravityRef(Utils::Deg2Rad(5.f));

  std::lock_guard<std::mutex> lock(this->Mtx);
  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, verbose);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate gravity measurement at LiDAR timestamp
  double rt = (lidarTime - bounds.first->Time) / (bounds.second->Time - bounds.first->Time);
  Eigen::Vector3d gravityDirection = (1 - rt) * bounds.first->Acceleration.normalized() + rt * bounds.second->Acceleration.normalized();
  // Normalize interpolated gravity vector
  if (gravityDirection.norm() > 1e-6) // Check to ensure consistent IMU measure
    gravityDirection.normalize();
  else
    return false;

  // Build gravity constraint
  this->Residual.Cost = CeresCostFunctions::ImuGravityAlignmentResidual::Create(this->GravityRef, gravityDirection);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  PRINT_INFO("\t Adding gravity residual with gravity reference : " << this->GravityRef.transpose())
  return true;
}

// ---------------------------------------------------------------------------
void ImuManager::ComputeGravityRef(double deltaAngle)
{
  std::lock_guard<std::mutex> lock(this->Mtx);
  // Init histogram 2D (phi and theta)
  int NPhi = std::ceil(2 * M_PI / deltaAngle);
  int NTheta = std::ceil(M_PI / deltaAngle);
  std::vector<std::vector<std::vector<GravityMeasurement*>>> histogram(NPhi, std::vector<std::vector<GravityMeasurement*>>(NTheta));

  // Store acceleration vector indices in histogram
  for (auto& meas : this->Measures)
  {
    Eigen::Vector3d AccelDirection = meas.Acceleration.normalized();
    int idxPhi = ( std::atan2(AccelDirection.y(), AccelDirection.x()) + M_PI ) / deltaAngle;
    int idxTheta = ( std::acos(AccelDirection.z()) ) / deltaAngle;
    histogram[idxPhi][idxTheta].push_back(&meas);
  }
  // Get bin containing most points
  int bestPhi = 0;
  int bestTheta = 0;
  for (int idxPhi = 0; idxPhi < NPhi; ++idxPhi)
  {
    for (int idxTheta = 0; idxTheta < NTheta; ++idxTheta)
    {
      if (histogram[idxPhi][idxTheta].size() > histogram[bestPhi][bestTheta].size())
      {
        bestPhi = idxPhi;
        bestTheta = idxTheta;
      }
    }
  }

  // Compute mean of acceleration vectors in this bin
  this->GravityRef = Eigen::Vector3d::Zero();
  for (auto& itAcc : histogram[bestPhi][bestTheta])
    this->GravityRef += itAcc->Acceleration.normalized();
  this->GravityRef.normalize();
}

} // end of SensorConstraints namespace
} // end of LidarSlam namespace