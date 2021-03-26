//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2021-03-01
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
#include <ceres/ceres.h>
#include "LidarSlam/Utilities.h"
#include "LidarSlam/CeresCostFunctions.h"

namespace LidarSlam
{
// Helper class to optimize the LidarSlam problem
class LocalOptimizer
{
public:

  //! Estimation of registration error
  struct RegistrationError
  {
    // Estimation of the maximum position error
    double PositionError = 0.;
    // Direction of the maximum position error
    Eigen::Vector3d PositionErrorDirection = Eigen::Vector3d::Zero();

    // Estimation of the maximum orientation error (in radians)
    double OrientationError = 0.;
    // Direction of the maximum orientation error
    Eigen::Vector3d OrientationErrorDirection = Eigen::Vector3d::Zero();

    // Covariance matrix encoding the estimation of the pose's errors about the 6-DoF parameters
    // (DoF order : X, Y, Z, rX, rY, rZ)
    Eigen::Matrix6d Covariance = Eigen::Matrix6d::Zero();
  };

  //----------------------------------------------------------------------------
  // Set params
  // Set Levenberg Marquardt maximum number of iterations
  void SetLMMaxIter(unsigned int iter);

  // Set number of threads
  void SetNbThreads(unsigned int n);

  // Set prior pose
  void SetPosePrior(const Eigen::Isometry3d& posePrior);

  // Add Sensor residuals to residuals vector
  void AddResiduals(std::vector<CeresTools::Residual>& sensorRes);

  // Clear all residuals
  void Clear();

  // Build and optimize the Ceres problem
  ceres::Solver::Summary Solve();

  // Get optimization results
  Eigen::Isometry3d GetOptimizedPose()  const { return Utils::XYZRPYtoIsometry(this->PoseArray); }

  // Estimate registration error
  RegistrationError EstimateRegistrationError();
  //----------------------------------------------------------------------------

private:
  
  // Max number of threads to use to parallelize computations
  unsigned int NbThreads = 1;

  // Maximum number of iteration
  unsigned int LMMaxIter = 15;

  // DoF to optimize (= output)
  Eigen::Vector6d PoseArray;  ///< Pose parameters to optimize (XYZRPY)

  // Residuals vector
  // These residuals must involve the full 6D pose array (X, Y, Z, rX, rY, rZ)
  std::vector<CeresTools::Residual> Residuals;
  
  std::unique_ptr<ceres::Problem> Problem;
};

} // end of LidarSlam namespace