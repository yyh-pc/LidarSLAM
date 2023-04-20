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

} // end of Confidence namespace
} // end of LidarSlam namespace