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

#include "LidarSlam/LocalOptimizer.h"
#include "LidarSlam/CeresCostFunctions.h"

namespace LidarSlam
{

//----------------------------------------------------------------------------
// Set params
//----------------------------------------------------------------------------
// Set Levenberg Marquardt maximum number of iterations
void LocalOptimizer::SetLMMaxIter(const unsigned int maxIt)
{
  this->LMMaxIter = maxIt;
}

// Set maximum number of iterations for Levenberg Marquardt algorithm
void LocalOptimizer::SetNbThreads(const unsigned int nbThreads)
{
  this->NbThreads = nbThreads;
}

void LocalOptimizer::SetPosePrior(const Eigen::Isometry3d& posePrior)
{
  // Convert isometry to 6D state vector : X, Y, Z, rX, rY, rZ
  this->PoseArray = Utils::IsometryToXYZRPY(posePrior);
}

//----------------------------------------------------------------------------
// Set residuals
//----------------------------------------------------------------------------

void LocalOptimizer::AddResiduals(std::vector<CeresTools::Residual>& lidarRes)
{
  this->Residuals.insert(this->Residuals.end(), lidarRes.begin(), lidarRes.end());
}

//----------------------------------------------------------------------------
// Clear all residuals at each ICP step
void LocalOptimizer::Clear()
{
  this->Residuals.clear();
}

//----------------------------------------------------------------------------
ceres::Solver::Summary LocalOptimizer::Solve()
{
  this->Problem = std::make_unique<ceres::Problem>();
  for (CeresTools::Residual& res : this->Residuals)
  {
    if (res.Cost)
      this->Problem->AddResidualBlock(res.Cost, res.Robustifier, this->PoseArray.data());
  }

  for (CeresTools::Residual& res : this->Residuals)
  {
    if (res.Cost)
      this->Problem->AddResidualBlock(res.Cost, res.Robustifier, this->PoseArray.data());
  }

  ceres::Solver::Options options;
  options.max_num_iterations = this->LMMaxIter;
  options.linear_solver_type = ceres::DENSE_QR;  // TODO : try also DENSE_NORMAL_CHOLESKY or SPARSE_NORMAL_CHOLESKY
  options.minimizer_progress_to_stdout = false;
  options.num_threads = this->NbThreads;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &(*this->Problem), &summary);
  return summary;
}

//----------------------------------------------------------------------------
LocalOptimizer::RegistrationError LocalOptimizer::EstimateRegistrationError()
{
  RegistrationError err;

  // Covariance computation options
  ceres::Covariance::Options covOptions;
  covOptions.apply_loss_function = true;
  covOptions.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
  covOptions.null_space_rank = -1;
  covOptions.num_threads = this->NbThreads;

  // Computation of the variance-covariance matrix
  ceres::Covariance covarianceSolver(covOptions);
  std::vector<std::pair<const double*, const double*>> covarianceBlocks;
  const double* paramBlock = this->PoseArray.data();
  covarianceBlocks.emplace_back(paramBlock, paramBlock);
  covarianceSolver.Compute(covarianceBlocks, &(*this->Problem));
  covarianceSolver.GetCovarianceBlock(paramBlock, paramBlock, err.Covariance.data());

  // Estimate max position/orientation errors and directions from covariance
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPosition(err.Covariance.topLeftCorner<3, 3>());
  err.PositionError = std::sqrt(eigPosition.eigenvalues()(2));
  err.PositionErrorDirection = eigPosition.eigenvectors().col(2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOrientation(err.Covariance.bottomRightCorner<3, 3>());
  err.OrientationError = Utils::Rad2Deg(std::sqrt(eigOrientation.eigenvalues()(2)));
  err.OrientationErrorDirection = eigOrientation.eigenvectors().col(2);

  return err;
}

} // end of LidarSlam namespace