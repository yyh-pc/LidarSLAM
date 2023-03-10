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

#pragma once

#include "LidarSlam/CeresCostFunctions.h" // for residual structure + ceres
#include "LidarSlam/Utilities.h"
#include "LidarSlam/State.h"
#include "LidarSlam/InterpolationModels.h"

#include <list>
#include <cfloat>
#include <mutex>
#include "LidarSlam/LidarPoint.h"

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#endif

#ifdef USE_GTSAM
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

// gtsam uses symbols to create vertex indices in the graph
// It allows to manage automatically the access to specific vertices
// (lidar poses, gps poses, landmarks, etc.)
// User only has to set the indices for each vertex type. They usually follow time sequence.
// see https://gtsam.org/tutorials/intro.html#magicparlabel-65663
using gtsam::symbol_shorthand::X; // 6D pose  (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Velocity (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias     (ax,ay,az,gx,gy,gz)
#endif

namespace LidarSlam
{

#define SetSensorMacro(name,type) void Set##name (type _arg) { this->name = _arg; }
#define GetSensorMacro(name,type) type Get##name () const { return this->name; }

namespace ExternalSensors
{

// ---------------------------------------------------------------------------
// MEASUREMENTS
// ---------------------------------------------------------------------------

struct LandmarkMeasurement
{
  double Time = 0.;
  // Relative transform between the detector and the tag
  Eigen::Isometry3d TransfoRelative = Eigen::Isometry3d::Identity();
  Eigen::Matrix6d Covariance = Eigen::Matrix6d::Identity();
  // No covariance is attached to pose measurement for now
  // Constant one can be used for pose graph optimizations
};

// ---------------------------------------------------------------------------
struct WheelOdomMeasurement
{
  double Time = 0.;
  double Distance = 0.;
};

// ---------------------------------------------------------------------------
struct GravityMeasurement
{
  double Time = 0.;
  Eigen::Vector3d Acceleration = Eigen::Vector3d::Zero();
};

// ---------------------------------------------------------------------------
struct GpsMeasurement
{
  double Time = 0.;
  Eigen::Vector3d Position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d Covariance = Eigen::Matrix3d::Identity();
};

// ---------------------------------------------------------------------------
struct PoseMeasurement
{
  double Time = 0.;
  Eigen::Isometry3d Pose = Eigen::Isometry3d::Identity();
  Eigen::Matrix6d Covariance = 0.05 * Eigen::Matrix6d::Identity();
};

// ---------------------------------------------------------------------------
struct ImuMeasurement
{
  double Time = 0.;
  Eigen::Vector3d Acceleration  = Eigen::Vector3d::Zero();
  Eigen::Vector3d AngleVelocity = Eigen::Vector3d::Zero();
  // NOTE : Imu preintegrator makes the assumption the
  // covariance is fixed for all IMU measurements so we
  // attach the covariances to the IMU manager
};

// ---------------------------------------------------------------------------
struct Image
{
  double Time = 0.;
  #ifdef USE_OPENCV
  cv::Mat Data;
  #endif
};

// SENSOR MANAGERS
// ---------------------------------------------------------------------------

// Base class to derive all external sensors
// Contains some tools for time synchronization
// data management and general parameters

// ---------------------------------------------------------------------------
template <typename T>
class SensorManager
{
public:
  SensorManager(const std::string& name = "BaseSensor")
  : SensorName(name), PreviousIt(Measures.begin()), ClosestIt(Measures.begin()) {}

  SensorManager(double timeOffset, double timeThreshold, unsigned int maxMeas,
                bool verbose = false, const std::string& name = "BaseSensor")
  : TimeOffset(timeOffset),
    TimeThreshold(timeThreshold),
    MaxMeasures(maxMeas),
    Verbose(verbose),
    SensorName(name),
    PreviousIt(Measures.begin()),
    ClosestIt(Measures.begin())
  {}

  // -----------------Setters/Getters-----------------
  GetSensorMacro(SensorName, std::string)
  SetSensorMacro(SensorName, std::string)

  GetSensorMacro(Weight, double)
  SetSensorMacro(Weight, double)

  GetSensorMacro(Calibration, Eigen::Isometry3d)
  SetSensorMacro(Calibration, const Eigen::Isometry3d&)

  GetSensorMacro(TimeOffset, double)
  SetSensorMacro(TimeOffset, double)

  GetSensorMacro(TimeThreshold, double)
  SetSensorMacro(TimeThreshold, double)

  GetSensorMacro(SaturationDistance, float)
  SetSensorMacro(SaturationDistance, float)

  GetSensorMacro(Verbose, bool)
  SetSensorMacro(Verbose, bool)

  GetSensorMacro(MaxMeasures, unsigned int)
  void SetMaxMeasures(unsigned int maxMeas)
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    this->MaxMeasures = maxMeas;
    while (this->Measures.size() > this->MaxMeasures)
    {
      if (this->PreviousIt == this->Measures.begin())
        ++this->PreviousIt;
      if (this->ClosestIt == this->Measures.begin())
        ++this->ClosestIt;
      this->Measures.pop_front();
    }
  }

  std::list<T> GetMeasures() const
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    return this->Measures;
  }

  GetSensorMacro(Residual, CeresTools::Residual)

  // -----------------Basic functions-----------------

  // ------------------
  // Add one measure at a time in measures list
  void AddMeasurement(const T& m)
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    this->Measures.emplace_back(m);
    if (this->Measures.size() > this->MaxMeasures)
    {
      if (this->PreviousIt == this->Measures.begin())
        ++this->PreviousIt;
      if (this->ClosestIt == this->Measures.begin())
        ++this->ClosestIt;
      this->Measures.pop_front();
    }
  }

  // ------------------
  void Reset(bool resetMeas = false)
  {
    this->ResetResidual();
    std::lock_guard<std::mutex> lock(this->Mtx);
    if (resetMeas)
      this->Measures.clear();
    this->PreviousIt = this->Measures.begin();
    this->ClosestIt  = this->Measures.begin();
  }

  // ------------------
  // Check if sensor can be used in tight SLAM optimization
  // The weight must be not null and the measures list must contain
  // at leat 2 elements to be able to interpolate
  bool CanBeUsedLocally() const
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    return this->Weight > 1e-6 && this->Measures.size() > 1;
  }

  // ------------------
  // Check if sensor has enough data to be interpolated
  // (the measures list must contain at leat 2 elements)
  bool HasData() const
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    return this->Measures.size() > 1;
  }

  // Compute the interpolated measure to be synchronized with SLAM output (at lidarTime)
  // 'trackTime' allows to keep a time track and to speed up multiple searches
  // when following chronological order
  virtual bool ComputeSynchronizedMeasure(double lidarTime, T& synchMeas, bool trackTime = true) = 0;
  // Compute the constraint associated to the measurement
  virtual bool ComputeConstraint(double lidarTime) = 0;

protected:
  // ------------------
  // Reset the current residual
  void ResetResidual()
  {
    this->Residual.Cost.reset();
    this->Residual.Robustifier.reset();
  }

  // ------------------
  // Get the measurements before and after the input lidarTime
  // If trackTime is enabled, the search is optimized for chronological input time values
  // windowSize allows to choose the number of neighboring measures to take
  // The output interval is [prev, post]
  std::pair<typename std::list<T>::iterator, typename std::list<T>::iterator> GetMeasureBounds(double lidarTime,
                                                                                               bool trackTime = true,
                                                                                               unsigned int windowSize = 2)
  {
    // Check if the measurements can be interpolated (or slightly extrapolated)
    if (lidarTime < this->Measures.front().Time || lidarTime > this->Measures.back().Time + this->TimeThreshold)
    {
      if (this->Verbose)
        PRINT_INFO(std::fixed << std::setprecision(9)
                   << "\t Measures contained in : [" << this->Measures.front().Time << ","
                   << this->Measures.back().Time <<"]\n"
                   << "\t -> " << this->SensorName << " not used"
                   << std::scientific)
      return std::make_pair(this->Measures.begin(), this->Measures.begin());
    }

    auto prevIt = this->PreviousIt;
    // Reset if the timeline has been modified (and if there is memory of a previous pose)
    if (prevIt == this->Measures.end() || prevIt->Time > lidarTime)
      prevIt = this->Measures.begin();

    auto postIt = prevIt;
    // Get iterator pointing to the first measurement after LiDAR time
    if (prevIt == this->Measures.begin())
    {
      // If after reset or for first search, use upper_bound function
      postIt = std::upper_bound(this->Measures.begin(),
                                this->Measures.end(),
                                lidarTime,
                                [&](double time, const T& measure) {return time < measure.Time;});
    }
    else
    {
      // If in the continuity of search, directly look for closest measurements
      while (postIt->Time < lidarTime && postIt != this->Measures.end())
        ++postIt;
    }

    // Get iterator pointing to the last measurement before LiDAR time
    prevIt = postIt;
    --prevIt;

    // Check the time between the 2 closest measurements to input time
    // Reset bound if interval is too high
    if (!this->CheckBounds(prevIt, postIt))
      return std::make_pair(this->Measures.begin(), this->Measures.begin());

    // Update the previous iterator for next call
    if (trackTime)
      this->PreviousIt = prevIt;

    // Update the closest iterator for extra needs
    this->ClosestIt = std::abs(prevIt->Time - lidarTime) < std::abs(postIt->Time - lidarTime) ? prevIt : postIt;

    // Increase the window until the number of data is reached or no data can be added anymore
    for (unsigned int nbData = 2; nbData < windowSize; ++nbData)
    {
      // Check the next oldest neighbor
      bool prevIsValid = this->CheckBounds(std::prev(prevIt), prevIt);
      // Check the next newest neighbor
      bool nextIsValid = this->CheckBounds(postIt, std::next(postIt));

      // Break if none is valid
      if (!prevIsValid && !nextIsValid)
        break;

      // Select the best valid one
      if (prevIsValid &&
          (!nextIsValid || std::abs(std::prev(prevIt)->Time - lidarTime) <
                           std::abs(std::next(postIt)->Time - lidarTime)))
        --prevIt;
      else
        ++postIt;
    }

    return std::make_pair(prevIt, postIt);
  }

  // ------------------
  virtual bool CheckBounds(typename std::list<T>::iterator prevIt, typename std::list<T>::iterator postIt)
  {
    if (prevIt == this->Measures.begin() || prevIt == this->Measures.end() ||
        postIt == this->Measures.begin() || postIt == this->Measures.end())
      return false;
    // Check if the time between the 2 measurements is too short
    if (postIt->Time - prevIt->Time < 1e-6)
    {
      if (this->Verbose)
        PRINT_INFO("\t" << this->SensorName << " measures cannot be used for interpolation (they are identical)")
      return false;
    }
    // Check if the time between the 2 measurements is too long
    if (postIt->Time - prevIt->Time > this->TimeThreshold)
    {
      if (this->Verbose)
        PRINT_INFO("\t" << this->SensorName << " measures cannot be used for interpolation (too much time difference)")
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  bool TimeInBounds(double time)
  {
    // Check if the correspondant iterator had been found before
    if (this->PreviousIt == this->Measures.begin() ||
        this->PreviousIt == this->Measures.end())
      return false;
    // Check if time lays in previous bounds
    if ((time < this->PreviousIt->Time) ||
        (time > std::next(this->PreviousIt)->Time))
      return false;
    return true;
  }

protected:
  // Measures stored
  std::list<T> Measures;
  // Weight to apply to sensor info when used in local optimization
  double Weight = 0.;
  // Calibration transform with base_link and the sensor
  Eigen::Isometry3d Calibration = Eigen::Isometry3d::Identity();
  // Time offset to make external sensors/Lidar correspondance
  // time_lidar = time_ext + offset
  double TimeOffset = 0.;
  // Time threshold between 2 measures to consider they can be interpolated
  double TimeThreshold = 0.5;
  // Threshold distance to not take into account the constraint
  // This distance is used in the constraint robustifier
  float SaturationDistance = 5.f;
  // Measures length limit
  // The oldest measures are forgotten
  unsigned int MaxMeasures = 1e6;
  // Verbose boolean to enable/disable debug info
  bool Verbose = false;
  // Sensor name for output
  std::string SensorName;
  // Iterator pointing to the last measure used
  // This allows to keep a time track
  typename std::list<T>::iterator PreviousIt;
  // Iterator pointing to the closest measure of the last input time
  typename std::list<T>::iterator ClosestIt;
  // Resulting residual
  CeresTools::Residual Residual;
  // Mutex to handle the data from outside the library
  mutable std::mutex Mtx;
};

// ---------------------------------------------------------------------------
// A wheel odometer allows to get a translation information
// For now, this manager is designed for cases where there is no rotation
// in the whole trajectory (e.g. Lidar following a cable)
// It builds a constraint comparing translation of base between
// two successive poses or from a reference pose
class WheelOdometryManager : public SensorManager<WheelOdomMeasurement>
{
public:
  WheelOdometryManager(const std::string& name = "Wheel odometer"): SensorManager(name){}
  WheelOdometryManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
                       bool verbose = false, const std::string& name = "Wheel odometer")
  : SensorManager(timeOffset, timeThresh, maxMeas, verbose, name) {this->Weight = w;}

  void Reset(bool resetMeas = false);

  //Setters/Getters
  GetSensorMacro(PreviousPose, Eigen::Isometry3d)
  SetSensorMacro(PreviousPose, const Eigen::Isometry3d&)

  GetSensorMacro(Relative, bool)
  SetSensorMacro(Relative, bool)

  GetSensorMacro(RefDistance, double)
  SetSensorMacro(RefDistance, double)

  // Compute the interpolated measure to be synchronized with SLAM output (at lidarTime)
  bool ComputeSynchronizedMeasure(double lidarTime, WheelOdomMeasurement& synchMeas, bool trackTime = true) override;
  // Wheel odometry constraint (unoriented)
  // Can be relative since last frame or absolute since first pose
  bool ComputeConstraint(double lidarTime) override;

private:
  // Members used when using the relative distance with last estimated pose
  Eigen::Isometry3d PreviousPose = Eigen::Isometry3d::Identity();
  double RefDistance = FLT_MAX;
  // Boolean to indicate whether to compute an absolute constraint (since first frame)
  // or relative constraint (since last acquired frame)
  bool Relative = false;
};

// ---------------------------------------------------------------------------
// An IMU can supply the gravity direction when measuring the whole acceleration
// when velocity is constant.
// This manager allows to create a local constraint to align gravity vectors between SLAM poses
class ImuGravityManager : public SensorManager<GravityMeasurement>
{
public:
  ImuGravityManager(const std::string& name = "IMU"): SensorManager(name){}
  ImuGravityManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
                    bool verbose = false, const std::string& name = "IMU")
  : SensorManager(timeOffset, timeThresh, maxMeas, verbose, name) {this->Weight = w;}

  void Reset(bool resetMeas = false);

  //Setters/Getters
  GetSensorMacro(GravityRef, Eigen::Vector3d)
  SetSensorMacro(GravityRef, const Eigen::Vector3d&)

  // Compute the interpolated measure to be synchronized with SLAM output (at lidarTime)
  bool ComputeSynchronizedMeasure(double lidarTime, GravityMeasurement& synchMeas, bool trackTime = true) override;

  // Compute the interpolated measure to be synchronized with SLAM output (at lidarTime) in base frame
  bool ComputeSynchronizedMeasureBase(double lidarTime, GravityMeasurement& synchMeas, bool trackTime = true);

  // IMU constraint (gravity)
  bool ComputeConstraint(double lidarTime) override;
  // Compute Reference gravity vector from IMU measurements
  void ComputeGravityRef(double deltaAngle);

private:
  Eigen::Vector3d GravityRef = Eigen::Vector3d::Zero();
};

// ---------------------------------------------------------------------------
// Landmarks can be detected by an external sensor (e.g. camera)
// This manager allows to create a local constraint with a reference absolute pose for the landmark
// or with previous observed poses of the landmark (like a usual keypoint)
class LandmarkManager: public SensorManager<LandmarkMeasurement>
{
public:
  LandmarkManager(const std::string& name = "Tag detector") : SensorManager(name){}
  LandmarkManager(const LandmarkManager& lmManager);
  LandmarkManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
                  Interpolation::Model model = Interpolation::Model::LINEAR, bool positionOnly = true,
                  bool verbose = false, const std::string& name = "Tag detector");

  void operator=(const LandmarkManager& lmManager);

  void Reset(bool resetMeas = false);

  // Setters/Getters
  // The absolute pose can be set from outside the lib
  // or will be detected online, averaging the previous detections
  GetSensorMacro(AbsolutePose, Eigen::Vector6d)
  GetSensorMacro(AbsolutePoseCovariance, Eigen::Matrix6d)

  GetSensorMacro(PositionOnly, bool)
  SetSensorMacro(PositionOnly, bool)

  GetSensorMacro(CovarianceRotation, bool)
  SetSensorMacro(CovarianceRotation, bool)

  Interpolation::Model GetInterpolationModel() const {return this->Interpolator.GetModel();}
  void SetInterpolationModel(Interpolation::Model model) {this->Interpolator.SetModel(model);}

  // Set the initial absolute pose
  // NOTE : the absolute pose can be updated if UpdateAbsolutePose is called
  void SetAbsolutePose(const Eigen::Vector6d& pose, const Eigen::Matrix6d& cov);

  // Compute the interpolated measure (landmark pose) to be synchronized with SLAM output at lidarTime
  bool ComputeSynchronizedMeasure(double lidarTime, LandmarkMeasurement& synchMeas, bool trackTime = true) override;

  // Compute the interpolated measure (landmark pose)
  // to be synchronized with SLAM output at lidarTimenfrom base frame
  bool ComputeSynchronizedMeasureBase(double lidarTime, LandmarkMeasurement& synchMeas, bool trackTime = true);

  // Landmark constraint
  bool ComputeConstraint(double lidarTime) override;

  // Update the absolute pose in case the tags are used as relative constraints
  // (i.e, no absolute poses of the tags are supplied)
  bool UpdateAbsolutePose(const Eigen::Isometry3d& baseTransform, double lidarTime);
  bool NeedsReferencePoseRefresh(double lidarTime);

private:
  bool HasBeenUsed(double lidarTime);

private:
  // Absolute pose of the landmark in the global frame
  Eigen::Vector6d AbsolutePose = Eigen::Vector6d::Zero();
  Eigen::Matrix6d AbsolutePoseCovariance = Eigen::Matrix6d::Zero();
  // Relative transform (detector/landmark) stored to be used when updating the absolute pose
  // It represents the transform between the detector and the landmark
  // i.e. detector to landmark, no calibration.
  Eigen::Isometry3d RelativeTransform = Eigen::Isometry3d::Identity();
  // Boolean to check the absolute pose has been loaded
  // or if the tag has already been seen
  bool HasAbsolutePose = false;
  std::pair<double, double> LastUpdateTimes = {FLT_MAX, FLT_MAX};
  // Counter to check how many frames the tag was seen on
  // This is used to average the pose in case the absolute poses
  // were not supplied initially and are updated (cf. UpdateAbsolutePose)
  int Count = 0;
  // The constraint created can use the whole position (orientation + position) -> false
  // or only the position -> true (if the orientation is not reliable enough)
  bool PositionOnly = true;
  // Allow to rotate the covariance
  // Can be disabled if the covariance is fixed or not used (e.g. for local constraint)
  bool CovarianceRotation = false;
  // Interpolator to get the relative pose between timestamps
  Interpolation::Trajectory Interpolator;
};

// ---------------------------------------------------------------------------
// GPS manager contains GPS sensor positions
// This manager can be used to build a pose graph
// GPS measurements are represented in a specific world referential frame (e.g. ENU)
// An offset transform links the GPS referential and the Lidar SLAM referential frame (e.g. first pose)
// This offset must be set from outside this library and can be computed using the GPS data and some lidar SLAM poses
class GpsManager: public SensorManager<GpsMeasurement>
{
public:
  GpsManager(const std::string& name = "GPS") : SensorManager(name){}
  GpsManager(const GpsManager& gpsManager);
  GpsManager(double timeOffset, double timeThresh, unsigned int maxMeas,
             bool verbose = false, const std::string& name = "GPS")
  : SensorManager(timeOffset, timeThresh, maxMeas, verbose, name) {}

  void operator=(const GpsManager& gpsManager);

  // Setters/Getters
  GetSensorMacro(Offset, Eigen::Isometry3d)
  SetSensorMacro(Offset, const Eigen::Isometry3d&)

  // Compute the interpolated measure (GPS position in GPS referential) to be synchronized with SLAM output at lidarTime
  bool ComputeSynchronizedMeasure(double lidarTime, GpsMeasurement& synchMeas, bool trackTime = true) override;

  // Compute the interpolated measure (GPS position in SLAM referential) to be synchronized with SLAM output at lidarTime
  // The measures data track GPS sensor frame (not base frame) but are represented in the same referential
  bool ComputeSynchronizedMeasureOffset(double lidarTime, GpsMeasurement& synchMeas, bool trackTime = true);

  bool ComputeConstraint(double lidarTime) override;

  bool CanBeUsedLocally() const {return false;}

private:
  // Offset transform to link GPS global frame and Lidar SLAM global frame
  // GPS referential to base referential
  Eigen::Isometry3d Offset = Eigen::Isometry3d::Identity();
};

// ---------------------------------------------------------------------------
class PoseManager: public SensorManager<PoseMeasurement>
{
public:
  PoseManager(const std::string& name = "Pose sensor") : SensorManager(name){}

  PoseManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
              Interpolation::Model model = Interpolation::Model::LINEAR,
              bool verbose = false, const std::string& name = "Pose sensor")
  : SensorManager(timeOffset, timeThresh, maxMeas, verbose, name)
  {this->Weight = w; this->Interpolator.SetModel(model);}

  void Reset(bool resetMeas = false);

  // Setters/Getters
  GetSensorMacro(PrevLidarTime, double)
  SetSensorMacro(PrevLidarTime, double)

  GetSensorMacro(PrevPoseTransform, Eigen::Isometry3d)
  SetSensorMacro(PrevPoseTransform, const Eigen::Isometry3d&)

  GetSensorMacro(CovarianceRotation, bool)
  SetSensorMacro(CovarianceRotation, bool)

  GetSensorMacro(DistanceThreshold, double)
  SetSensorMacro(DistanceThreshold, double)

  Interpolation::Model GetInterpolationModel() const {return this->Interpolator.GetModel();}
  void SetInterpolationModel(Interpolation::Model model) {this->Interpolator.SetModel(model);}

  // Compute the interpolated measure (pose of the external pose sensor's)
  // to be synchronized with SLAM output at lidarTime
  bool ComputeSynchronizedMeasure(double lidarTime, PoseMeasurement& synchMeas, bool trackTime = true) override;

  // Compute the interpolated measure (base pose)
  // to be synchronized with SLAM output at lidarTime : calibration is applied
  bool ComputeSynchronizedMeasureBase(double lidarTime, PoseMeasurement& synchMeas, bool trackTime = true);

  bool ComputeConstraint(double lidarTime) override;

  // Get pose at a specific timestamp
  Eigen::Isometry3d GetPose(double time = -1);

protected:
  double PrevLidarTime = -1.;
  Eigen::Isometry3d PrevPoseTransform = Eigen::Isometry3d::Identity();
  // Allow to rotate the covariance
  // Can be disabled if the covariance is fixed or not used (e.g. for local constraint)
  bool CovarianceRotation = false;
  // Distance threshold [m] between 2 measures to consider they can be interpolated
  double DistanceThreshold = 0.;

  // Check time and motion difference of bounds
  // Do not use the 2 measures if time difference is too long and motion difference is too large
  bool CheckBounds(std::list<PoseMeasurement>::iterator prevIt, std::list<PoseMeasurement>::iterator postIt) override;
  // Interpolator used to get poses between measurements
  Interpolation::Trajectory Interpolator;
};

// ---------------------------------------------------------------------------
// IMU raw data (accelerations and angular rates) must be preprocessed to be able to integrate them into the SLAM optimization
// This manager allows to preintegrate them. The main problem is to correctly estimate the bias which is a slowly varying error
// made on raw measurements. This bias can be corrected at each new SLAM pose output using this manager.
// This manager is derived from PoseManager so that once the data are preintegrated, the pose manager
// can be used to compute synchronized data and a local constraint
// NOTE : init values and useful constants were found here : https://github.com/TixiaoShan/LIO-SAM/
class ImuManager: public PoseManager
{
public:
  ImuManager(const std::string& name = "IMU")
  : PoseManager(name){this->Reset();}

  ImuManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
             Interpolation::Model model = Interpolation::Model::LINEAR,
             Eigen::Isometry3d initBasePose = Eigen::Isometry3d::Identity(),
             bool verbose = false, const std::string& name = "IMU")
  : PoseManager(w, timeOffset, timeThresh, maxMeas, model, verbose, name)
  {
    this->InitBasePose = initBasePose;
    this->Reset();
  }

  // ---------------------------------------------------------------------------

  // Setters/Getters
  GetSensorMacro(Gravity, Eigen::Vector3d)
  SetSensorMacro(Gravity, const Eigen::Vector3d&)

  GetSensorMacro(Frequency, float)
  SetSensorMacro(Frequency, float)

  GetSensorMacro(InitBasePose, Eigen::Isometry3d)
  void SetInitBasePose(const Eigen::Isometry3d& initPose)
  {
    this->InitBasePose = initPose;
    this->Reset();
  }

  GetSensorMacro(ResetThreshold, unsigned int)
  SetSensorMacro(ResetThreshold, unsigned int)

  GetSensorMacro(NLag, int)
  SetSensorMacro(NLag, int)

  // ---------------------------------------------------------------------------
  void Reset(bool resetMeas = false)
  {
    this->SensorManager::Reset(resetMeas);
    if (resetMeas)
    {
      std::lock_guard<std::mutex> lock(this->Mtx);
      this->RawMeasures.clear();
    }
    #ifdef USE_GTSAM
    this->OptimizedSlamStates.clear();
    this->OptimizedSlamStates.resize(this->ResetThreshold);

    this->BiasCovarianceDiag << 6.4356659353532566e-05, 6.4356659353532566e-05, 6.4356659353532566e-05,
                                3.5640318696367613e-05, 3.5640318696367613e-05, 3.5640318696367613e-05;

    // Initialize preintegrator
    // Set measurement noise and gravity (in world reference frame)
    // boost smart pointer mandatory to initialize preintegrators
    boost::shared_ptr<gtsam::PreintegrationParams> p = boost::make_shared<gtsam::PreintegrationParams>(-this->Gravity);
    p->accelerometerCovariance = this->AccCovariance;
    p->gyroscopeCovariance     = this->GyrCovariance;
    // Set integration noise (from velocities to positions)
    p->integrationCovariance = std::pow(1e-4, 2) * Eigen::Matrix3d::Identity();
    // Initialize pose isometry with 0 velocity
    this->OptimizedSlamStates[0] = gtsam::NavState(gtsam::Pose3((this->InitBasePose * this->Calibration).matrix()), Eigen::Vector3d::Zero());
    this->PrevLidarTime = -1.;
    // Initialize a null bias
    this->Bias = gtsam::imuBias::ConstantBias(Eigen::Vector6d::Zero());
    // Initialize preintegrators with noise and bias
    this->Preintegrator = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, this->Bias);
    this->PreintegratorNewData = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, this->Bias);
    this->ResetOptimization();
    this->TimeIdx = 0;
    #endif
  }

  // --------------------------------------------------------------------------
  // Add one measure at a time in measures list
  // The measure added (acc, vel) is not of the same type as the one used in optimization (pose)
  // so we overload the function AddMeasurement
  // WARNING for postprocess : preintegration can be heavy,
  // resulting poses are only used in local SLAM optimization,
  // we advice to not add measures that are far away from studied SLAM time
  using PoseManager::AddMeasurement;
  void AddMeasurement(const ImuMeasurement& m)
  {
    #ifdef USE_GTSAM
    std::lock_guard<std::mutex> lock(this->Mtx);
    // Update preintegration with new measurement
    // If this is the first measurement,
    // dt is approximated using frequency set externally
    double dt = this->RawMeasures.empty()? 1. / this->Frequency : m.Time - this->RawMeasures.back().Time;
    this->PreintegratorNewData->integrateMeasurement(m.Acceleration, m.AngleVelocity, dt);
    // Store raw measurement
    this->RawMeasures.emplace_back(m);
    // Use gravity, new data, previously optimized SLAM pose, previously optimized SLAM velocity
    // and previously optimized bias to derive new IMU pose
    // and store it into the measures list
    gtsam::NavState imuState = this->PreintegratorNewData->predict(this->OptimizedSlamStates[0], this->Bias);
    PoseMeasurement imuPose;
    imuPose.Pose.linear() = imuState.R();
    imuPose.Pose.translation() = imuState.t();
    imuPose.Time = m.Time;
    this->Measures.emplace_back(imuPose);
    if (this->Measures.size() > this->MaxMeasures)
    {
      if (this->PreviousIt == this->Measures.begin())
        ++this->PreviousIt;
      if (this->ClosestIt == this->Measures.begin())
        ++this->ClosestIt;
      this->Measures.pop_front();
      this->RawMeasures.pop_front();
    }
    return;
    #endif
    static_cast<void>(m);
  }

  #ifdef USE_GTSAM
  // --------------------------------------------------------------------------
  void RestartGraph(gtsam::noiseModel::Gaussian::shared_ptr priorPoseNoise,
                    gtsam::noiseModel::Gaussian::shared_ptr priorVelNoise,
                    gtsam::noiseModel::Gaussian::shared_ptr priorBiasNoise)
  {
    this->Graph = gtsam::ISAM2();
    // Create factors
    gtsam::NonlinearFactorGraph graphFactors;
    // Prior pose
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), this->OptimizedSlamStates[0].pose(), priorPoseNoise);
    graphFactors.add(priorPose);
    // Prior velocity
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), this->OptimizedSlamStates[0].v(), priorVelNoise);
    graphFactors.add(priorVel);
    // Prior bias
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), this->Bias, priorBiasNoise);
    graphFactors.add(priorBias);

    // Create init values
    gtsam::Values initValues;
    initValues.insert(X(0), this->OptimizedSlamStates[0].pose());
    initValues.insert(V(0), this->OptimizedSlamStates[0].v());
    initValues.insert(B(0), this->Bias);

    // Add factors and value to ISAM graph and optimize once
    this->Graph.update(graphFactors, initValues);
  }

  // --------------------------------------------------------------------------
  void ResetOptimization()
  {
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    this->Graph = gtsam::ISAM2(optParameters);
  }

  // --------------------------------------------------------------------------
  // Update measures between the ith and the (i + 1)th pose using IMU measurements
  using ImuMeasIt  = std::list<ImuMeasurement>::iterator;
  using PoseMeasIt = std::list<PoseMeasurement>::iterator;
  std::pair<ImuMeasIt, PoseMeasIt> UpdateMeasures(std::pair<ImuMeasIt, PoseMeasIt> startIterators,
                                                  int iCurr, int iNext = -1)
  {
    // 1_ Get measure iterators corresponding to the ith graph vertex
    // Those iterators define the start of the measures update when preintegrating
    auto itMeasStartUpdate = startIterators.second;
    auto itRawMeasStartUpdate = startIterators.first;

    double timeIcurr = this->Idx2Time[iCurr];
    gtsam::imuBias::ConstantBias bias = this->Idx2Bias[iCurr];

    auto lastMeasureIt = this->Measures.end();
    --lastMeasureIt;
    auto lastRawMeasureIt = this->RawMeasures.end();
    --lastRawMeasureIt;

    // We are looking for iterators of measures corresponding to the one just after lidar state timestamp
    while (itMeasStartUpdate != lastMeasureIt && itMeasStartUpdate->Time < timeIcurr)
    {
      ++itMeasStartUpdate;
      ++itRawMeasStartUpdate;
    }

    // Reset preintegrator
    this->PreintegratorNewData->resetIntegrationAndSetBias(bias);
    // Add first value after lidar state timestamp
    this->PreintegratorNewData->integrateMeasurement(itRawMeasStartUpdate->Acceleration,
                                                     itRawMeasStartUpdate->AngleVelocity,
                                                     itRawMeasStartUpdate->Time - timeIcurr);

    // Update Measures list
    // Compute new predicted IMU pose for current timestamp
    gtsam::NavState imuState = this->PreintegratorNewData->predict(this->OptimizedSlamStates[iCurr], bias);
    // Update measure value with new pose
    itMeasStartUpdate->Pose.matrix() = imuState.pose().matrix();
    // Init last time for future dt computation
    double lastImuTime = itMeasStartUpdate->Time;

    // Update start iterators
    ++itMeasStartUpdate;
    ++itRawMeasStartUpdate;

    // Get measure iterators corresponding to the measure just after next SLAM timestamp (i.e. next graph vertex)
    // This iterator defines the end of the measures update when preintegrating (end exluded)
    auto itRawMeasEndUpdate = itRawMeasStartUpdate;
    if (iNext > 0)
    {
      double timeInext = this->Idx2Time[iNext];
      while (itRawMeasEndUpdate != lastRawMeasureIt && itRawMeasEndUpdate->Time < timeInext)
        ++itRawMeasEndUpdate;
    }
    else
      itRawMeasEndUpdate = this->RawMeasures.end();


    // 2_ Refill
    // Init iterator to measures list
    auto itMeasure = itMeasStartUpdate;
    int n = 0;
    for (auto itRawMeasure = itRawMeasStartUpdate; itRawMeasure != itRawMeasEndUpdate; ++itRawMeasure)
    {
      // Readd values to preintegrator of new data
      this->PreintegratorNewData->integrateMeasurement(itRawMeasure->Acceleration,
                                                       itRawMeasure->AngleVelocity,
                                                       itRawMeasure->Time - lastImuTime);

      // Update Measures list
      // Compute new predicted IMU pose for current timestamp
      gtsam::NavState imuState = this->PreintegratorNewData->predict(this->OptimizedSlamStates[iCurr], bias);
      // Update measure value with new pose
      itMeasure->Pose.matrix() = imuState.pose().matrix();
      // Update last time for dt computation
      lastImuTime = itRawMeasure->Time;

      // Update Measures list iterator
      ++itMeasure;
      ++n;
    }

    // Return last iterators changed
    return {--itRawMeasEndUpdate, --itMeasure};
  }
  #endif

  // --------------------------------------------------------------------------
  // Use last SLAM pose to update IMU poses
  bool Update(const LidarState& state)
  {
    #ifdef USE_GTSAM
    // Check time
    if (this->TimeIdx != 0 &&
        (state.Time - this->PrevLidarTime < 0 ||
         state.Time - this->PrevLidarTime > this->TimeThreshold))
    {
      PRINT_WARNING("There was a time cut in Lidar info at "
                    << std::fixed << std::setprecision(12) << this->PrevLidarTime << std::scientific
                    << " : resetting IMU")
      this->TimeIdx = 0;
      return false;
    }

    if (state.Time - this->PrevLidarTime < 1e-6)
    {
      if (this->Verbose)
        PRINT_INFO("SLAM time has not changed : not updating IMU")
      return true;
    }

    // Synchronize lidar times to IMU times if needed
    double prevLidarTimeSynch = this->PrevLidarTime - this->TimeOffset;
    double lidarTimeSynch = state.Time - this->TimeOffset;

    // Lock mutex to handle RawMeasures and Measures lists
    std::lock_guard<std::mutex> lock(this->Mtx);

    // First raw measure must be older than previous lidar time
    // and last measure must be newer than current lidar time
    if (this->RawMeasures.empty() ||
        this->RawMeasures.front().Time > lidarTimeSynch + 1e-6 ||
        (this->TimeIdx != 0 && this->RawMeasures.front().Time > prevLidarTimeSynch + 1e-6) ||
        this->RawMeasures.back().Time  < lidarTimeSynch - 1e-6)
    {
      PRINT_WARNING("Could not find IMU synchronized measures between " << std::fixed << std::setprecision(16) << prevLidarTimeSynch
                     << " and " << lidarTimeSynch << " -> IMU data not updated, it may drift" << std::scientific)
      this->TimeIdx = 0;
      return false;
    }

    // Init graph with first lidar slam state received
    if (this->TimeIdx == 0)
    {
      this->OptimizedSlamStates[0] = gtsam::NavState(gtsam::Pose3((state.Isometry * this->Calibration).matrix()), Eigen::Vector3d::Zero());
      this->PrevLidarTime = state.Time;
      this->RestartGraph(this->InitPoseNoise, this->InitVelNoise, this->InitBiasNoise);
      this->Idx2Time[this->TimeIdx] = lidarTimeSynch;
      this->Idx2Bias[this->TimeIdx] = this->Bias;

      // If some measures were already stored, crop them from the current timestamp to speed up searches
      // Get IMU measure iterators corresponding to the measurement received just before the current timestamp
      auto itRawCurrent = this->RawMeasures.end();
      auto itCurrent = this->Measures.end();
      --itRawCurrent;
      --itCurrent;
      while (itRawCurrent->Time > lidarTimeSynch)
      {
        --itRawCurrent;
        --itCurrent;
      }

      int initMeasuresSize = this->Measures.size();
      this->Measures = std::list<PoseMeasurement>(itCurrent, this->Measures.end());
      // Update previous measure iterator for searches
      this->PreviousIt = this->Measures.begin();
      this->ClosestIt = this->Measures.begin();
      this->RawMeasures = std::list<ImuMeasurement>(itRawCurrent, this->RawMeasures.end());

      if (this->Verbose)
        PRINT_INFO("IMU measures cropped to " << std::fixed << std::setprecision(12) << lidarTimeSynch << std::scientific << "   "
                   << initMeasuresSize - this->Measures.size() << "   measures forgotten");

      // Update measures from first SLAM pose
      auto startIts = std::make_pair(this->RawMeasures.begin(), this->Measures.begin());
      this->UpdateMeasures(startIts, 0);

      // Update time index
      this->TimeIdx = 1;

      return true;
    }

    // Reset the graph every ResetThreshold lidar frames
    if (this->TimeIdx >= this->ResetThreshold)
    {
      gtsam::noiseModel::Gaussian::shared_ptr lastPoseNoise, lastVelNoise, lastBiasNoise;
      lastPoseNoise = gtsam::noiseModel::Gaussian::Covariance(this->Graph.marginalCovariance(X(this->TimeIdx-1)));
      lastVelNoise  = gtsam::noiseModel::Gaussian::Covariance(this->Graph.marginalCovariance(V(this->TimeIdx-1)));
      lastBiasNoise = gtsam::noiseModel::Gaussian::Covariance(this->Graph.marginalCovariance(B(this->TimeIdx-1)));
      auto lastOptimizedSlamState = this->OptimizedSlamStates[this->TimeIdx-1];
      this->PrevLidarTime = state.Time;
      this->Idx2Time[0] = lidarTimeSynch;
      this->Idx2Bias[0] = this->Bias;
      this->OptimizedSlamStates.clear();
      this->OptimizedSlamStates.resize(this->ResetThreshold);
      this->OptimizedSlamStates[0] = lastOptimizedSlamState;
      this->ResetOptimization();
      this->RestartGraph(lastPoseNoise, lastVelNoise, lastBiasNoise);
      this->TimeIdx = 1;
      return true;
    }
    // 0_ Add new IMU measurements to preintegrator until lidarState time

    // 0_0 Init iterators
    // Get last raw measure iterator
    auto lastRawMeasureIt = this->RawMeasures.end();
    --lastRawMeasureIt;
    // Init search at beginning
    auto itRawMeasStartUpdate = this->RawMeasures.begin();

    // 0_1 Get raw measure iterator corresponding to the one just after previous lidar state timestamp
    // This iterator defines the start of the preintegration
    while (itRawMeasStartUpdate->Time < prevLidarTimeSynch)
      ++itRawMeasStartUpdate;

    // Reset preintegrator
    this->Preintegrator->resetIntegrationAndSetBias(this->Bias);
    // Add first value after lidar state timestamp
    this->Preintegrator->integrateMeasurement(itRawMeasStartUpdate->Acceleration,
                                              itRawMeasStartUpdate->AngleVelocity,
                                              itRawMeasStartUpdate->Time - prevLidarTimeSynch);
    // Init last time for future dt computation
    double lastImuTime = itRawMeasStartUpdate->Time;

    // Update start iterator
    ++itRawMeasStartUpdate;

    // Get measure iterators corresponding to the measure just after current SLAM timestamp
    // This iterator defines the end of the measures update when preintegrating (end exluded)
    auto itRawMeasEndUpdate = itRawMeasStartUpdate;
    while (itRawMeasEndUpdate != lastRawMeasureIt && itRawMeasEndUpdate->Time < lidarTimeSynch)
      ++itRawMeasEndUpdate;

    // 0_2 Fill preintegrator from last SLAM state to current SLAM state
    for (auto itRawMeasure = itRawMeasStartUpdate; itRawMeasure != itRawMeasEndUpdate; ++itRawMeasure)
    {
      // Add values to preintegrator
      this->Preintegrator->integrateMeasurement(itRawMeasure->Acceleration,
                                                itRawMeasure->AngleVelocity,
                                                itRawMeasure->Time - lastImuTime);
      // Update last time for dt computation
      lastImuTime = itRawMeasure->Time;
    }
    --itRawMeasEndUpdate;
    // Integrate last measurement for the remaining time until current SLAM timestamp
    this->Preintegrator->integrateMeasurement(itRawMeasEndUpdate->Acceleration,
                                              itRawMeasEndUpdate->AngleVelocity,
                                              lidarTimeSynch - lastImuTime);
    // NOTE : A gtsam graph is composed of :
    // - factors : functions that link elements
    // - values : initial values of the elements for pose graph optimization
    // See https://gtsam.org/tutorials/intro.html#magicparlabel-65438

    // 1_ Add new factors to graph
    gtsam::NonlinearFactorGraph graphFactors;
    // 1_1 SLAM pose factor (as prior factor)
    // Create SLAM factor (the graph tracks IMU sensor poses)
    Eigen::Vector6d initPose = Utils::IsometryToXYZRPY(state.Isometry);
    Eigen::Matrix6d rotCov = state.Covariance;
    CeresTools::RotateCovariance(initPose, rotCov, this->Calibration, false); // new = init * calib
    gtsam::PriorFactor<gtsam::Pose3> poseFactor(X(this->TimeIdx), gtsam::Pose3((state.Isometry * this->Calibration).matrix()), state.Covariance);
    // Add SLAM pose to factors
    graphFactors.add(poseFactor);
    // 1_2_ IMU factor
    // Get preintegrator reference
    const gtsam::PreintegratedImuMeasurements& preintegrator = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*this->Preintegrator);
    // Create IMU graph factor
    gtsam::ImuFactor imuFactor(X(this->TimeIdx - 1), V(this->TimeIdx - 1),
                               X(this->TimeIdx)    , V(this->TimeIdx),
                               B(this->TimeIdx - 1),
                               preintegrator);
    // Add imu to factors
    graphFactors.add(imuFactor);
    // 1_3_ Bias factor (as between factor)
    // Create noise
    // CHECK : what is done here, why isn't it constant?
    gtsam::noiseModel::Diagonal::shared_ptr biasNoise =
      gtsam::noiseModel::Diagonal::Sigmas(std::sqrt(this->Preintegrator->deltaTij()) * this->BiasCovarianceDiag);
    // Create bias factor
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> biasFactor(B(this->TimeIdx - 1), B(this->TimeIdx),
                                                                  gtsam::imuBias::ConstantBias(),
                                                                  biasNoise);
    // Add bias to factors
    graphFactors.add(biasFactor);

    // 2_ Create new init values for graph optimization
    gtsam::Values initValues;
    // Preintegrated IMU data
    gtsam::NavState preintIMU = this->Preintegrator->predict(this->OptimizedSlamStates[this->TimeIdx - 1], this->Bias);
    initValues.insert(X(this->TimeIdx), preintIMU.pose());
    initValues.insert(V(this->TimeIdx), preintIMU.v());
    // Bias
    initValues.insert(B(this->TimeIdx), this->Bias);

    // 3_ Optimize and update members with result
    // Add previously computed factors and init values to graph
    this->Graph.update(graphFactors, initValues);
    this->Graph.update(); // CHECK : useful?
    gtsam::Values graphValues = this->Graph.calculateEstimate();

    // Update optimized poses
    for (unsigned int i = 0; i <= this->TimeIdx; ++i)
      this->OptimizedSlamStates[i] = gtsam::NavState(graphValues.at<gtsam::Pose3>(X(i)),
                                                     graphValues.at<gtsam::Vector3>(V(i)));

    // Get last bias from result
    this->Bias = graphValues.at<gtsam::imuBias::ConstantBias>(B(this->TimeIdx));

    // 4_ Update PreintegratorNewData with new bias
    auto startIts = std::make_pair(this->RawMeasures.begin(), this->Measures.begin());
    int lastIdx = std::max(0, int(this->TimeIdx) - this->NLag);
    for (int i = 0; i < lastIdx; ++i)
      startIts = this->UpdateMeasures(startIts, i, i + 1);

    this->UpdateMeasures(startIts, lastIdx);

    // 5_ Update time index and last slam state time for next input
    this->Idx2Time[this->TimeIdx] = lidarTimeSynch;
    this->Idx2Bias[this->TimeIdx] = this->Bias;
    ++this->TimeIdx;
    this->PrevLidarTime = state.Time;

    // Reset the graph every ResetThreshold lidar frames
    if (this->TimeIdx >= this->ResetThreshold)
    {
      gtsam::noiseModel::Gaussian::shared_ptr lastPoseNoise, lastVelNoise, lastBiasNoise;
      lastPoseNoise = gtsam::noiseModel::Gaussian::Covariance(this->Graph.marginalCovariance(X(this->TimeIdx-1)));
      lastVelNoise  = gtsam::noiseModel::Gaussian::Covariance(this->Graph.marginalCovariance(V(this->TimeIdx-1)));
      lastBiasNoise = gtsam::noiseModel::Gaussian::Covariance(this->Graph.marginalCovariance(B(this->TimeIdx-1)));
      auto lastOptimizedSlamState = this->OptimizedSlamStates[this->TimeIdx-1];
      this->Idx2Time[0] = lidarTimeSynch;
      this->Idx2Bias[0] = this->Bias;
      this->OptimizedSlamStates.clear();
      this->OptimizedSlamStates.resize(this->ResetThreshold);
      this->OptimizedSlamStates[0] = lastOptimizedSlamState;
      this->ResetOptimization();
      this->RestartGraph(lastPoseNoise, lastVelNoise, lastBiasNoise);
      this->TimeIdx = 1;
    }

    return true;
    #endif
    static_cast<void>(state);
    return false;
  }

private:
  // ---------------Preintegration relative members---------------
  // List of old raw measurements received
  std::list<ImuMeasurement> RawMeasures;
  // Covariance of raw measurement
  // NOTE : As covariance is fixed for all raw measurements,
  // it is attached to the manager and not to the measurements
  Eigen::Matrix3d AccCovariance = std::pow(3.9939570888238808e-03, 2) * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d GyrCovariance = std::pow(1.5636343949698187e-03, 2) * Eigen::Matrix3d::Identity();
  // Estimated gravity (m/s^2)
  Eigen::Vector3d Gravity = {0., 0., -9.80511}; // default : z upward
  // Frequency of the IMU
  float Frequency = 100.f; // Used when dt is not available
  // Initial pose of BASE
  Eigen::Isometry3d InitBasePose = Eigen::Isometry3d::Identity();
  #ifdef USE_GTSAM
  // SLAM poses optimized using IMU preintegration constraint
  std::vector<gtsam::NavState> OptimizedSlamStates;
  // IMU bias on acceleration and angular velocity
  gtsam::imuBias::ConstantBias Bias = gtsam::imuBias::ConstantBias(Eigen::Vector6d::Zero());
  // Preintegrator for data since last lidar time
  // Used to get poses at IMU frequency
  // NOTE : Those poses can then be used in SLAM optimization
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> PreintegratorNewData;

  // ---------------Pose graph relative members---------------
  // Preintegrator until last lidar time
  // Used to create the IMU factor in the graph
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> Preintegrator;
  // ISAM2 optimizer allow to keep the pose graph and only update it
  // at each new information input
  gtsam::ISAM2 Graph;
  // Bias error used to recompute the bias after a new SLAM pose is computed
  Eigen::Vector6d BiasCovarianceDiag;
  // Prior factors settings for pose graph optimization
  // Prior pose noise : rad, rad, rad, m, m, m
  gtsam::noiseModel::Gaussian::shared_ptr InitPoseNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
  // Prior velocity noise (m/s)
  // --> check 1e4 wanted?
  gtsam::noiseModel::Gaussian::shared_ptr InitVelNoise  = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
  // Prior acceleration noise (m/s^2)
  // 1e-2 ~ 1e-3 recommended by LIO-SAM
  gtsam::noiseModel::Gaussian::shared_ptr InitBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
  // Map to link vertex indices with timestamps
  std::map<unsigned int, double> Idx2Time;
  // Map to link vertex indices with bias
  std::map<unsigned int, gtsam::imuBias::ConstantBias> Idx2Bias;
  #endif
  // Timestamp index in graph
  unsigned int TimeIdx = 0;
  // Add threshold as number of lidar frames received
  // to reset the graph in order to lighten graph optimization process
  // IMU data and lidar data older than ResetThrehold lidar frames won't impact
  // new IMU pose estimations
  unsigned int ResetThreshold = 400;
  // Number of Lidar SLAM states to wait before using it as preintegration start
  // The wait is needed in case the new Lidar SLAM poses orientation is not accurate enough
  // A little orientation error at the begining leads to a bad gravity compensation
  // and big preintegrated pose errors.
  // The IMU graph optimization will allow to correct this orientation error when adding
  // new Lidar SLAM states.
  int NLag = 3;
};

// ---------------------------------------------------------------------------
class CameraManager: public SensorManager<Image>
{
  // Usefull types
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;

public:
  CameraManager(const std::string& name = "Camera") : SensorManager(name){}
  CameraManager(const CameraManager& cameraManager);
  CameraManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
                bool verbose = false, const std::string& name = "Camera")
  : SensorManager(timeOffset, timeThresh, maxMeas, verbose, name){this->Weight = w;}

  // Setters/Getters
  GetSensorMacro(PrevLidarTime, double)
  SetSensorMacro(PrevLidarTime, double)

  GetSensorMacro(PrevPoseTransform, Eigen::Isometry3d)
  SetSensorMacro(PrevPoseTransform, const Eigen::Isometry3d&)

  GetSensorMacro(PrevLidarFrame, PointCloud::Ptr)
  SetSensorMacro(PrevLidarFrame, PointCloud::Ptr)

  GetSensorMacro(SaturationDistance, float)
  SetSensorMacro(SaturationDistance, float)

  GetSensorMacro(Residuals, const std::vector<CeresTools::Residual>&)

  // Setters/Getters
  GetSensorMacro(IntrinsicCalibration, Eigen::Matrix3f)
  SetSensorMacro(IntrinsicCalibration, const Eigen::Matrix3f&)

  // Get the closest image to lidar timestamp
  bool ComputeSynchronizedMeasure(double lidarTime, Image& synchImage, bool trackTime = true) override;

  // Compute camera residuals based on pixel matches
  bool ComputeConstraint(double lidarTime) override;

  // ------------------
  // Check if camera can be used in tight SLAM optimization
  // The weight must be not null, the measures list must contain
  // at leat 2 elements to be able to interpolate and the intrinsic
  // calibration must have been provided.
  bool CanBeUsedLocally()
  {
    std::lock_guard<std::mutex> lock(this->Mtx);
    return this->Weight > 1e-6 && this->Measures.size() > 1 &&
           !this->IntrinsicCalibration.isIdentity(1e-6);
  }

private:
  std::vector<CeresTools::Residual> Residuals;
  double PrevLidarTime = 0.;
  // Previous Lidar frame represented in base frame
  PointCloud::Ptr PrevLidarFrame;
  Eigen::Isometry3d PrevPoseTransform = Eigen::Isometry3d::Identity();
  Eigen::Matrix3f IntrinsicCalibration = Eigen::Matrix3f::Identity();
  // Pixelic distance threshold to not take into account the pixel matches
  // because some of them might be wrong
  // This distance is used in a robustifier to weight the camera residuals
  float SaturationDistance = 10.f;
};

} // end of ExternalSensors namespace
} // end of LidarSlam namespace