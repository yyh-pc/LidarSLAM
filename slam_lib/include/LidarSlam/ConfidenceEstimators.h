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

// STD
#include <cfloat>
#include <list>

// LOCAL
#include "LidarSlam/RollingGrid.h"
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

// Queue container to keep only the newest elements
// relatively to a number or a time duration
template <typename T>
class LastValues
{
struct TimedValue
{
  T Value;
  double Time;
};

public:
  LastValues() = default;

  // Interface basic list functions
  void Clear() {this->Values.clear();}
  unsigned int Size() const {return this->Values.size();}
  double Duration() const {return this->Empty()? -1 : std::abs(this->Values.Back().Time - this->Values.Front().Time);}
  bool Empty() const {return this->Values.empty();}
  bool Full() const {return this->Values.size() == this->MaxWindowSize;}
  TimedValue Back() const {return this->Values.back();}
  TimedValue Front() const {return this->Values.front();}

  // Getters/Setters
  // Parameters
  unsigned int GetMaxWindowSize() const {return this->MaxWindowSize;}
  void SetMaxWindowSize(unsigned int size) {this->MaxWindowSize = size;}

  double GetMaxWindowDuration() const {return this->MaxWindowDuration;}
  void SetMaxWindowDuration(double duration) {this->MaxWindowDuration = duration;}

  // Add new value to the list
  void AddValue(T value, double timestamp = -1.)
  {
    // Add new value
    this->Values.push_back({value, timestamp});

    // Remove old values
    if (this->Values.size() > this->MaxWindowSize)
      this->Values.pop_front();

    while (std::abs(timestamp - this->Values.front().Time) > this->MaxWindowDuration)
      this->Values.pop_front();
  };

  // Helper function to sum the values in the list
  T Sum()
  {
    T sum = T(0);
    for (const auto& v: this->Values)
      sum += v.Value;
    return sum;
  }

private:
  // Maximum number of values to store
  unsigned int MaxWindowSize = 10;
  // Maximum duration between stored values
  double MaxWindowDuration = FLT_MAX;

  // Container
  std::list<TimedValue> Values;
};

// Container for one specific confidence metric
// Allows to store a window of N values and to compute average and derivative
class Estimator
{
public:
  // Constructor
  Estimator() = default;

  // Empty data
  void Reset();

  // Getters/Setters
  // Parameters
  unsigned int GetWindowSize() const {return this->WindowSize;}
  void SetWindowSize(unsigned int size);

  // Main behavior function
  // Results
  float GetAverage() const {return this->Averages.Back().Value;}
  float GetDerivative() const {return this->Derivative;}

  // Add new value to estimator
  // Compute average and derivative on the window
  void AddValue(float value, double timestamp);

private:
  // Number of values onto which to perform an
  // average and to compute the derivatives.
  // If too high, some failure cases might be not detected
  // If too low, too many failure cases might be detected
  unsigned int WindowSize = 10;

  // Storage struct for values, averages
  // (to denoise the values) and derivative
  LastValues<float> Values;
  LastValues<float> Averages;
  float Derivative = FLT_MAX;

  // Helpers
  bool Average();
  bool Derivate();
};

// Manager for failure detection
// Check failure from different confidence estimators
class FailDetector
{
public:
  FailDetector() = default;

  void Reset();

  // Getters/Setters
  // Parameters
  unsigned int GetWindowSize() const {return this->OverlapEst.GetWindowSize();}
  void SetWindowSize(unsigned int size);

  float GetOverlapDerivativeThreshold() const {return this->OverlapDerivativeThreshold;}
  void SetOverlapDerivativeThreshold(float overlapThresh) {std::abs(this->OverlapDerivativeThreshold = overlapThresh);}

  float GetPositionErrorThreshold() const {return this->PositionErrorThreshold;}
  void SetPositionErrorThreshold(float posErrorThresh) {this->PositionErrorThreshold = posErrorThresh;}

  // Data
  float GetOverlapAverage() const {return this->OverlapEst.GetAverage();}
  float GetPositionErrorAverage() const {return this->PositionErrorEst.GetAverage();}

  float GetOverlapDerivative() const {return this->OverlapEst.GetDerivative();}
  float GetPositionErrorDerivative() const {return this->PositionErrorEst.GetDerivative();}

  // Main behavior functions
  // Add the confidence metrics to process them (denoise + derivate)
  void AddConfidence(float overlap, float positionError,
                     bool goodMotion, bool matches,
                     double timeStamp);
  // Check if the SLAM has failed from the metrics
  // This functions checks if their is a lack of degree of liberty,
  // if the map has doubled or if the pose is diverging
  bool HasFailed();

private:
  // Data
  Estimator OverlapEst;
  Estimator PositionErrorEst;
  LastValues<bool> SuspiciousMotion;
  bool LocalizationValid = true;
  int CounterUnvalidity = 0;

  // Parameters
  // Note : those parameters are combined to trigger a failure
  // Threshold on overlap derivative
  float OverlapDerivativeThreshold = 0.04f; // rate [0, 1]
  // Threshold on position error
  float PositionErrorThreshold = 0.15f; // distance m
  // Maximum number of consecutive motion failure
  int MaxUnvalidityNb = 5;
};

// Compute the LCP estimator (overlap estimator) for the registration of a
// pointcloud onto some prebuilt maps.
// It corresponds to the number of points from cloud which have a neighbor in
// the submaps relatively to the resolution of the maps.
// (see http://geometry.cs.ucl.ac.uk/projects/2014/super4PCS/ for more info)
// In this LCP extension, we also check the distance between nearest neighbors
// to make a smooth estimator.
// To accelerate the process, the ratio of points (between 0 and 1) from the
// input cloud to compute overlap on can be specified.
// It returns a valid overlap value between 0 and 1, or -1 if the overlap could
// not be computed (not enough points).
// If proba is disabled, the overlap estimator is simply based on the ratio of the points
// having a close neighbor in the maps
float LCPEstimator(PointCloud::ConstPtr cloud,
                   const std::map<Keypoint, std::shared_ptr<RollingGrid>>& maps,
                   float subsamplingRatio = 1.,
                   int nbThreads = 1,
                   bool proba = true);

} // enf of Confidence namespace
} // end of LidarSlam namespace