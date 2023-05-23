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

#include <LidarSlam/ConfidenceEstimators.h>
#include <LidarSlam/Utilities.h>

namespace LidarSlam
{
namespace Confidence
{
//-----------------------------------------------------------------------------
// Estimator functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void Estimator::SetWindowSize(unsigned int size)
{
  this->Values.SetMaxWindowSize(size);
  this->Averages.SetMaxWindowSize(size);
}

//-----------------------------------------------------------------------------
void Estimator::Reset()
{
  this->Values.Clear();
  this->Averages.Clear();
  this->Derivative = FLT_MAX;
}

//-----------------------------------------------------------------------------
void Estimator::AddValue(float value, double timestamp)
{
  this->Values.AddValue(value, timestamp);
  this->Derivative = FLT_MAX;
  if (this->Average())
    this->Derivate();
}

//-----------------------------------------------------------------------------
bool Estimator::Average()
{
  if (!this->Values.Full())
    return false;

  this->Averages.AddValue(this->Values.Sum() / this->Values.Size(), this->Values.Back().Time);

  return true;
}

//-----------------------------------------------------------------------------
bool Estimator::Derivate()
{
  if (!this->Averages.Full())
    return false;

  this->Derivative = (this->Averages.Back().Value - this->Averages.Front().Value) /
                     (this->Averages.Back().Time - this->Averages.Front().Time);

  return true;
}

//-----------------------------------------------------------------------------
// FailDetector functions
//-----------------------------------------------------------------------------
void FailDetector::Reset()
{
  this->LocalizationValid = true;
  this->CounterUnvalidity = 0;
  this->OverlapEst.Reset();
  this->PositionErrorEst.Reset();
  this->SuspiciousMotion.Clear();
}

//-----------------------------------------------------------------------------
void FailDetector::SetWindowSize(unsigned int size)
{
  this->OverlapEst.SetWindowSize(size);
  this->PositionErrorEst.SetWindowSize(size);
  this->SuspiciousMotion.SetMaxWindowSize(size);
}

//-----------------------------------------------------------------------------
void FailDetector::AddConfidence(float overlap, float positionError,
                                 bool goodMotion, bool localizationValid,
                                 double timestamp)
{
  this->OverlapEst.AddValue(overlap, timestamp);
  this->PositionErrorEst.AddValue(positionError, timestamp);
  if (!goodMotion || !localizationValid)
    ++this->CounterUnvalidity;
  else
    this->CounterUnvalidity = 0;
  this->SuspiciousMotion.AddValue(!goodMotion);
  this->LocalizationValid = localizationValid;
}

//-----------------------------------------------------------------------------
bool FailDetector::HasFailed()
{
  // Check divergence
  if (this->CounterUnvalidity > this->MaxUnvalidityNb)
  {
    PRINT_WARNING("Failure : SLAM might be diverging");
    return true;
  }

  float ovpDer = this->OverlapEst.GetDerivative();
  if (std::abs(ovpDer - FLT_MAX) < 1e-4)
    return false;

  // Check bad local minimum
  if (this->SuspiciousMotion.Sum() &&
      -ovpDer > this->OverlapDerivativeThreshold)
  {
    PRINT_WARNING("Failure : bad local minimum found, the map might be corrupted");
    return true;
  }

  float posErDer =  this->PositionErrorEst.GetDerivative();
  if (std::abs(posErDer - FLT_MAX) < 1e-4)
    return false;

  // Check missing degree of liberty
  if (this->SuspiciousMotion.Sum() &&
      this->PositionErrorEst.GetAverage() > this->PositionErrorThreshold)
  {
    PRINT_WARNING("Failure : missing degree of liberty constraint (e.g. corridor or field contexts)");
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// LCP function
//-----------------------------------------------------------------------------
float LCPEstimator(PointCloud::ConstPtr cloud,
                   const std::map<Keypoint, std::shared_ptr<RollingGrid>>& maps,
                   float subsamplingRatio,
                   int nbThreads,
                   bool proba)
{
  // Number of points to process
  int nbPoints = cloud->size() * subsamplingRatio;
  if (nbPoints == 0 || maps.empty())
    return -1.;

  // Iterate on all points of input cloud to process
  float lcp = 0.;
  #pragma omp parallel for num_threads(nbThreads) reduction(+:lcp)
  for (int n = 0; n < nbPoints; ++n)
  {
    // Compute the LCP contribution of the current point
    const auto& point = cloud->at(n / subsamplingRatio);
    float bestProba = 0.;
    for (const auto& map : maps)
    {
      // Get nearest neighbor
      int nnIndex;
      float nnSqDist;
      if (map.second->GetSubMapKdTree().KnnSearch(point.data, 1, &nnIndex, &nnSqDist))
      {
        if (!proba && nnSqDist < std::pow(map.second->GetLeafSize(), 2))
        {
          bestProba = 1;
          break;
        }
        // We use a Gaussian like estimation for each point fitted in target leaf space
        // to check the probability that one cloud point has a neighbor in the target
        // Probability = 1 if the two points are superimposed
        // Probability < 0.011 if the distance is g.t. the leaf size
        float sqLCPThreshold = std::pow(map.second->GetLeafSize() / 3.f, 2);
        float currentProba = std::exp( -nnSqDist / (2.f * sqLCPThreshold) );
        if (currentProba > bestProba)
          bestProba = currentProba;
      }
    }
    lcp += bestProba;
  }
  return lcp / nbPoints;
}

//-----------------------------------------------------------------------------
MotionChecker::MotionChecker()
{
  this->Poses.SetMaxWindowSize(10);
  this->Poses.SetMaxWindowDuration(FLT_MAX);
}

//-----------------------------------------------------------------------------
void MotionChecker::Reset()
{
  this->Poses.Clear();
  this->Pose.Zero();
  this->Velocity.Zero();
  this->Acceleration.Zero();
  this->PrevMotionDirection.Zero();
}

//-----------------------------------------------------------------------------
Eigen::Array2f MotionChecker::GetMotion(const Eigen::Isometry3d& TWindow)
{
  // Compute angular part
  float angle = Eigen::AngleAxisd(TWindow.linear()).angle();
  // Rotation angle in [0, pi]
  if (angle > M_PI)
    angle = 2 * M_PI - angle;
  angle = Utils::Rad2Deg(angle);
  // Compute linear part
  float distance = TWindow.translation().norm();

  return {distance, angle};
}

//-----------------------------------------------------------------------------
void MotionChecker::SetNewPose(const Eigen::Isometry3d& pose, double time)
{
  if (this->Poses.Empty())
  {
    // Disable all motion constraints
    this->Pose.Zero();
    this->Velocity.Zero();
    this->Acceleration.Zero();
    this->PrevMotionDirection.Zero();
    // Add new pose to stored values
    this->Poses.AddValue(pose, time);
    return;
  }

  // Compute motion between the two last poses
  Eigen::Isometry3d TWindow = this->Poses.Back().Value.inverse() * pose;

  this->ChangeDirection = this->PrevMotionDirection.norm() > 1e-6 &&
                          TWindow.translation().dot(this->PrevMotionDirection) < -0.1; // margin for stops
  this->PrevMotionDirection = TWindow.translation().normalized();

  this->Pose = this->GetMotion(TWindow);
  // Add new pose into stored values for next inputs
  this->Poses.AddValue(pose, time);

  if (!this->Poses.Full())
  {
    // Disable velocity/acceleration constraints
    this->Velocity.Zero();
    this->Acceleration.Zero();
    return;
  }

  // Compute motion between the two bounds of the window
  TWindow = this->Poses.Front().Value.inverse() * pose;
  Eigen::Array2f motion = this->GetMotion(TWindow);
  double deltaTime      = std::abs(time - this->Poses.Front().Time);

  // Compute velocity
  // Store previous one for acceleration
  Eigen::Array2f prevVel = this->Velocity;
  this->Velocity = {motion(0) / deltaTime, motion(1) / deltaTime};

  // Do not compute acceleration if there is no previous velocity
  if (prevVel[0] > FLT_MAX - 1)
  {
    this->Acceleration.Zero();
    return;
  }

  // Compute local acceleration in BASE
  this->Acceleration = (this->Velocity - prevVel) / deltaTime;

  if (this->Verbose)
  {
    SET_COUT_FIXED_PRECISION(3)
    PRINT_INFO("Pose         = " << this->Pose[0] << " m,  "
                                 << this->Pose[1] << " °");
    PRINT_INFO("Velocity     = " << this->Velocity[0] << " m/s,  "
                                 << this->Velocity[1] << " °/s");
    PRINT_INFO("Acceleration = " << this->Acceleration[0] << " m/s2, "
                                 << this->Acceleration[1] << " °/s2");
    if (this->ChangeDirection) { PRINT_INFO("Changing direction"); }
    else { PRINT_INFO("Keeping direction"); }
    RESET_COUT_FIXED_PRECISION
  }
}

//-----------------------------------------------------------------------------
bool MotionChecker::isMotionValid()
{
  if (this->ChangeDirection)
    return false;

  // Check if metrics have been computed
  if (this->Acceleration[0] > FLT_MAX - 1)
  {
    PRINT_WARNING("Motion validity could not be checked");
    return true;
  }

  // Check pose compliance
  bool complyPoseLimits = this->Pose[0] < FLT_MAX - 1 ? (this->Pose.abs() < this->PoseLimits).all()
                                                      : true;
  // Check velocity compliance
  bool complyVelocityLimits = this->Velocity[0] < FLT_MAX - 1 ? (this->Velocity.abs() < this->VelocityLimits).all()
                                                              : true;
  // Check acceleration compliance
  bool complyAccelerationLimits = this->Acceleration[0] < FLT_MAX - 1 ? (this->Acceleration.abs() < this->AccelerationLimits).all()
                                                                      : true;

  // Check limits
  return complyPoseLimits     &&
         complyVelocityLimits &&
         complyAccelerationLimits;
}

} // end of Confidence namespace
} // end of LidarSlam namespace