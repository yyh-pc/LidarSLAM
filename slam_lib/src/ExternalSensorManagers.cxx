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

#include "LidarSlam/ExternalSensorManagers.h"

namespace LidarSlam
{
namespace ExternalSensors
{

// ---------------------------------------------------------------------------
// Wheel odometer
// ---------------------------------------------------------------------------

void WheelOdometryManager::Reset(bool resetMeas)
{
  this->SensorManager::Reset(resetMeas);
  this->SetRefDistance(FLT_MAX);
  this->PreviousPose = Eigen::Isometry3d::Identity();
}

// ---------------------------------------------------------------------------
bool WheelOdometryManager::ComputeSynchronizedMeasure(double lidarTime, WheelOdomMeasurement& synchMeas, bool trackTime)
{
  if (!this->CanBeUsedLocally())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);

  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, trackTime);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate odometry measurement at LiDAR timestamp
  synchMeas.Time = lidarTime;
  double rt = Utils::Normalize(lidarTime, bounds.first->Time, bounds.second->Time);
  synchMeas.Distance = (1.0 - rt) * bounds.first->Distance + rt * bounds.second->Distance;

  return true;
}

// ---------------------------------------------------------------------------
bool WheelOdometryManager::ComputeConstraint(double lidarTime)
{
  this->ResetResidual();

  // Compute synchronized measures
  WheelOdomMeasurement synchMeas; // Virtual measure with synchronized timestamp
  if (!ComputeSynchronizedMeasure(lidarTime, synchMeas))
    return false;

  // Build odometry residual
  // If there is no memory of previous poses
  // The current measure is taken as reference
  if (this->RefDistance > 1e9)
  {
    if (this->Verbose)
      PRINT_INFO("No previous wheel odometry measure : no constraint added to optimization")
    // Update reference distance for next frames
    this->RefDistance = synchMeas.Distance;
    return false;
  }

  // If there is memory of a previous pose
  double distDiff = std::abs(synchMeas.Distance - this->RefDistance);
  this->Residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(this->PreviousPose.translation(), distDiff);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  if(this->Verbose && !this->Relative)
    PRINT_INFO("Adding absolute wheel odometry residual : " << distDiff << " m travelled since first frame.")
  if (this->Verbose && this->Relative)
    PRINT_INFO("Adding relative wheel odometry residual : " << distDiff << " m travelled since last frame.")

  // Update reference distance if relative mode enabled
  if (this->Relative)
    this->RefDistance = synchMeas.Distance;

  return true;
}

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------

void ImuGravityManager::Reset(bool resetMeas)
{
  this->SensorManager::Reset(resetMeas);
  this->SetGravityRef(Eigen::Vector3d::Zero());
}

// ---------------------------------------------------------------------------
bool ImuGravityManager::ComputeSynchronizedMeasure(double lidarTime, GravityMeasurement& synchMeas, bool trackTime)
{
  if (!this->CanBeUsedLocally())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, trackTime);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate gravity measurement at LiDAR timestamp
  synchMeas.Time = lidarTime;
  double rt = Utils::Normalize(lidarTime, bounds.first->Time, bounds.second->Time);
  synchMeas.Acceleration = (1.0 - rt) * bounds.first->Acceleration.normalized() + rt * bounds.second->Acceleration.normalized();
  // Normalize interpolated gravity vector
  if (synchMeas.Acceleration.norm() > 1e-6) // Check to ensure consistent IMU measure
    synchMeas.Acceleration.normalize();
  else
    return false;

  return true;
}

// ---------------------------------------------------------------------------
bool ImuGravityManager::ComputeSynchronizedMeasureBase(double lidarTime, GravityMeasurement& synchMeas, bool trackTime)
{
  if (!this->ComputeSynchronizedMeasure(lidarTime, synchMeas, trackTime))
    return false;

  // Represent gravity in base frame
  synchMeas.Acceleration = this->Calibration * synchMeas.Acceleration;

  return true;
}

// ---------------------------------------------------------------------------
bool ImuGravityManager::ComputeConstraint(double lidarTime)
{
  this->ResetResidual();

  // Compute reference gravity vector
  if (this->GravityRef.norm() < 1e-6)
    this->ComputeGravityRef(Utils::Deg2Rad(5.f));

  // Compute synchronized measures in base frame
  GravityMeasurement synchMeas; // Virtual measure with synchronized timestamp and calibration applied
  if (!ComputeSynchronizedMeasureBase(lidarTime, synchMeas))
    return false;

  // Build gravity constraint
  this->Residual.Cost = CeresCostFunctions::ImuGravityAlignmentResidual::Create(this->GravityRef, synchMeas.Acceleration);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  if (this->Verbose)
    PRINT_INFO("\t Adding gravity residual with gravity reference : " << this->GravityRef.transpose())
  return true;
}

// ---------------------------------------------------------------------------
void ImuGravityManager::ComputeGravityRef(double deltaAngle)
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

// ---------------------------------------------------------------------------
// Landmark manager
// ---------------------------------------------------------------------------

void LandmarkManager::Reset(bool resetMeas)
{
  this->SensorManager::Reset(resetMeas);
  this->AbsolutePose = Eigen::Vector6d::Zero();
  this->AbsolutePoseCovariance = Eigen::Matrix6d::Zero();
  this->RelativeTransform = Eigen::Isometry3d::Identity();
  this->HasAbsolutePose = false;
  this->LastUpdateTimes = {FLT_MAX, FLT_MAX};
  this->Count = 0;
}

// ---------------------------------------------------------------------------
LandmarkManager::LandmarkManager(double w, double timeOffset, double timeThresh, unsigned int maxMeas,
                                 Interpolation::Model model, bool positionOnly,
                                 bool verbose, const std::string& name)
                : SensorManager(timeOffset, timeThresh, maxMeas, verbose, name),
                  PositionOnly(positionOnly),
                  CovarianceRotation(false)
{
  this->Weight = w;
  this->Interpolator.SetModel(model);
}

// ---------------------------------------------------------------------------
LandmarkManager::LandmarkManager(const LandmarkManager& lmManager)
                : LandmarkManager(lmManager.GetWeight(),
                                  lmManager.GetTimeOffset(),
                                  lmManager.GetTimeThreshold(),
                                  lmManager.GetMaxMeasures(),
                                  lmManager.GetInterpolationModel(),
                                  lmManager.GetPositionOnly(),
                                  lmManager.GetVerbose(),
                                  lmManager.GetSensorName())
{
  this->CovarianceRotation = lmManager.GetCovarianceRotation();
  this->Measures           = lmManager.GetMeasures();
  this->PreviousIt         = this->Measures.begin();
  this->ClosestIt          = this->Measures.begin();
  // Parameters only useful in local optimization
  this->Weight             = lmManager.GetWeight();
  this->SaturationDistance = lmManager.GetSaturationDistance();
}

// ---------------------------------------------------------------------------
void LandmarkManager::operator=(const LandmarkManager& lmManager)
{
  this->SensorName         = lmManager.GetSensorName();
  this->TimeOffset         = lmManager.GetTimeOffset();
  this->TimeThreshold      = lmManager.GetTimeThreshold();
  this->Verbose            = lmManager.GetVerbose();
  this->MaxMeasures        = lmManager.GetMaxMeasures();
  this->SetInterpolationModel(lmManager.GetInterpolationModel());
  this->PositionOnly       = lmManager.GetPositionOnly();
  this->CovarianceRotation = lmManager.GetCovarianceRotation();
  this->Measures           = lmManager.GetMeasures();
  this->PreviousIt         = this->Measures.begin();
  // Parameters only useful in local optimization
  this->Weight             = lmManager.GetWeight();
  this->SaturationDistance = lmManager.GetSaturationDistance();
}

// ---------------------------------------------------------------------------
void LandmarkManager::SetAbsolutePose(const Eigen::Vector6d& pose, const Eigen::Matrix6d& cov = Eigen::Matrix6d::Identity())
{
  this->AbsolutePose = pose;
  this->AbsolutePoseCovariance = cov;
  this->HasAbsolutePose = true;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::HasBeenUsed(double lidarTime)
{
  // Absolute is used to discard the initial case LastUpdateTimes.second = infinity
  return std::abs(lidarTime - this->LastUpdateTimes.second) < 1e-6;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::NeedsReferencePoseRefresh(double lidarTime)
{
  return this->HasBeenUsed(lidarTime) &&
         (!this->HasAbsolutePose || this->LastUpdateTimes.second - this->LastUpdateTimes.first > this->TimeThreshold);
}

// ---------------------------------------------------------------------------
bool LandmarkManager::UpdateAbsolutePose(const Eigen::Isometry3d& baseTransform, double lidarTime)
{
  if (!this->HasBeenUsed(lidarTime))
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // If it is the first time the tag is detected
  // or if the last time the tag has been seen was long ago
  // (re)set the absolute pose using the current base transform and
  // the relative transform measured
  if (NeedsReferencePoseRefresh(lidarTime))
  {
    this->AbsolutePose = Utils::IsometryToXYZRPY(baseTransform * this->Calibration * this->RelativeTransform);
    this->HasAbsolutePose = true;
    this->Count = 1;
  }
  // If it has already been seen, the absolute pose is updated averaging the computed poses
  else
  {
    Eigen::Vector6d newAbsolutePose = Utils::IsometryToXYZRPY(baseTransform * this->Calibration * this->RelativeTransform);
    this->AbsolutePose = ( (this->AbsolutePose * this->Count) + newAbsolutePose ) / (this->Count + 1);
    ++this->Count;
  }
  return true;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::ComputeSynchronizedMeasure(double lidarTime, LandmarkMeasurement& synchMeas, bool trackTime)
{
  if (!this->HasData())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  lidarTime -= this->TimeOffset;

  // If data are not initialized or are out of the bounds, recompute interpolation
  if (!this->TimeInBounds(lidarTime))
  {
    // Get window bounds + update ClosestIt
    auto bounds = this->GetMeasureBounds(lidarTime, trackTime, this->Interpolator.GetNbRequiredData());
    if (bounds.first == bounds.second)
      return false;
    std::vector<PoseStamped> ctrlPoses;
    ctrlPoses.reserve(std::distance(bounds.first, bounds.second) + 1);
    for (auto it = bounds.first; it != std::next(bounds.second); ++it)
      ctrlPoses.emplace_back(it->TransfoRelative, it->Time);
    this->Interpolator.BuildModel(ctrlPoses);
  }

  // Fill measure
  synchMeas.Time = lidarTime;
  synchMeas.Covariance = this->ClosestIt->Covariance;
  // Interpolate landmark relative pose at LiDAR timestamp
  synchMeas.TransfoRelative = this->Interpolator(lidarTime);
  // Rotate covariance if required
  if (this->CovarianceRotation)
  {
    Eigen::Isometry3d update = this->ClosestIt->TransfoRelative.inverse() * synchMeas.TransfoRelative;
    Eigen::Vector6d xyzrpy = Utils::IsometryToXYZRPY(this->ClosestIt->TransfoRelative);
    synchMeas.Covariance = CeresTools::RotateCovariance(xyzrpy, this->ClosestIt->Covariance, update); // new = init * update
  }

  // Update RelativeTransform for AbsolutePose update
  if (trackTime)
    this->RelativeTransform = synchMeas.TransfoRelative;

  return true;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::ComputeSynchronizedMeasureBase(double lidarTime, LandmarkMeasurement& synchMeas, bool trackTime)
{
  if (!this->ComputeSynchronizedMeasure(lidarTime, synchMeas, trackTime))
    return false;

  // Rotate covariance with calibration if covariance rotation required
  if (this->CovarianceRotation)
  {
    Eigen::Vector6d xyzrpy = Utils::IsometryToXYZRPY(synchMeas.TransfoRelative);
    synchMeas.Covariance = CeresTools::RotateCovariance(xyzrpy, synchMeas.Covariance, this->Calibration, true); // new = calib * init
  }

  // Represent relative pose from base frame
  synchMeas.TransfoRelative = this->Calibration * synchMeas.TransfoRelative;

  return true;
}

// ---------------------------------------------------------------------------
bool LandmarkManager::ComputeConstraint(double lidarTime)
{
  this->ResetResidual();

  if (!this->CanBeUsedLocally())
    return false;

  // Compute synchronized measures from base frame
  LandmarkMeasurement synchMeas; // Virtual measure with synchronized timestamp and calibration applied
  if (!this->ComputeSynchronizedMeasureBase(lidarTime, synchMeas))
    return false;

  // Last times the tag was used
  // this is used to update the absolute reference tag pose when the
  // sensor absolute pose will be estimated (if required).
  this->LastUpdateTimes.first = this->LastUpdateTimes.second;
  this->LastUpdateTimes.second = lidarTime;

  // Check if the absolute pose has been computed
  // If not, the next tag detection is waited
  if (!this->HasAbsolutePose)
  {
    if (this->Verbose)
      PRINT_WARNING("\t No absolute pose, waiting for next detection")
    return false;
  }

  // Build constraint
  // NOTE : the covariances are not used because the uncertainty is not comparable with common keypoint constraints
  // The user must play with the weight parameter to get the best result depending on the tag detection accuracy.
  if (this->PositionOnly)
    this->Residual.Cost = CeresCostFunctions::LandmarkPositionResidual::Create(synchMeas.TransfoRelative, this->AbsolutePose);
  else
    this->Residual.Cost = CeresCostFunctions::LandmarkResidual::Create(synchMeas.TransfoRelative, this->AbsolutePose);

  // Use a robustifier to limit the contribution of an outlier tag detection (the tag may have been moved)
  // Tukey loss applied on residual square:
  //   rho(residual^2) = a^2 / 3 * ( 1 - (1 - residual^2 / a^2)^3 )   for residual^2 <= a^2,
  //   rho(residual^2) = a^2 / 3                                      for residual^2 >  a^2.
  // a is the scaling parameter of the function
  // See http://ceres-solver.org/nnls_modeling.html#theory for details
  auto* robustifier = new ceres::TukeyLoss(this->SaturationDistance);

  // Weight the contribution of the given match by its reliability
  // WARNING : in CERES version < 2.0.0, the Tukey loss is badly implemented, so we have to correct the weight by a factor 2
  // See https://github.com/ceres-solver/ceres-solver/commit/6da364713f5b78ddf15b0e0ad92c76362c7c7683 for details
  // This is important for covariance scaling
  #if (CERES_VERSION_MAJOR < 2)
    this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, 2.0 * this->Weight, ceres::TAKE_OWNERSHIP));
  // If Ceres version >= 2.0.0, the Tukey loss is corrected.
  #else
    this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, this->Weight, ceres::TAKE_OWNERSHIP));
  #endif

  return true;
}

// ---------------------------------------------------------------------------
// GPS
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
GpsManager::GpsManager(const GpsManager& gpsManager)
           : GpsManager(gpsManager.GetTimeOffset(),
                        gpsManager.GetTimeThreshold(),
                        gpsManager.GetMaxMeasures(),
                        gpsManager.GetVerbose(),
                        gpsManager.GetSensorName())
{
  this->Measures = gpsManager.GetMeasures();
  this->PreviousIt = this->Measures.begin();
}

// ---------------------------------------------------------------------------
void GpsManager::operator=(const GpsManager& gpsManager)
{
  this->SensorName = gpsManager.GetSensorName();
  this->TimeOffset = gpsManager.GetTimeOffset();
  this->Verbose = gpsManager.GetVerbose();
  this->TimeThreshold = gpsManager.GetTimeThreshold();
  this->MaxMeasures = gpsManager.GetMaxMeasures();
  this->Measures = gpsManager.GetMeasures();
  this->PreviousIt = this->Measures.begin();
}

// ---------------------------------------------------------------------------
bool GpsManager::ComputeSynchronizedMeasure(double lidarTime, GpsMeasurement& synchMeas, bool trackTime)
{
  if (!this->HasData())
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, trackTime);
  if (bounds.first == bounds.second)
    return false;
  // Interpolate landmark relative pose at LiDAR timestamp
  synchMeas.Time = lidarTime;
  synchMeas.Position = bounds.first->Position + lidarTime * (bounds.second->Position - bounds.first->Position) / (bounds.second->Time - bounds.first->Time);
  synchMeas.Covariance = bounds.first->Covariance;

  return true;
}

// ---------------------------------------------------------------------------
bool GpsManager::ComputeSynchronizedMeasureOffset(double lidarTime, GpsMeasurement& synchMeas, bool trackTime)
{
  if (!this->ComputeSynchronizedMeasure(lidarTime, synchMeas, trackTime))
    return false;

  // Apply offset to represent the GPS measurement in SLAM reference frame
  synchMeas.Position = this->Offset * synchMeas.Position;
  // Rotate covariance
  synchMeas.Covariance = this->GetOffset().linear() * synchMeas.Covariance;

  return true;
}

// ---------------------------------------------------------------------------
bool GpsManager::ComputeConstraint(double lidarTime)
{
  static_cast<void>(lidarTime);
  PRINT_WARNING("No local constraint can/should be added from GPS as they are absolute measurements");
  return false;
}

// ---------------------------------------------------------------------------
// 3D POSE
// ---------------------------------------------------------------------------

void PoseManager::Reset(bool resetMeas)
{
  this->SensorManager::Reset(resetMeas);
  this->PrevLidarTime = 0.;
  this->PrevPoseTransform = Eigen::Isometry3d::Identity();
  this->Interpolator.Reset();
}

// ---------------------------------------------------------------------------
bool PoseManager::ComputeSynchronizedMeasure(double lidarTime, PoseMeasurement& synchMeas, bool trackTime)
{
  if (this->Measures.size() <= 1)
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;

  if (!this->TimeInBounds(lidarTime))
  {
    // Get window bounds + update ClosestIt
    auto bounds = this->GetMeasureBounds(lidarTime, trackTime, this->Interpolator.GetNbRequiredData());

    if (bounds.first == bounds.second)
      return false;
    std::vector<PoseStamped> ctrlPoses;
    ctrlPoses.reserve(std::distance(bounds.first, bounds.second) + 1);
    for (auto it = bounds.first; it != std::next(bounds.second); ++it)
      ctrlPoses.emplace_back(it->Pose, it->Time);
    this->Interpolator.BuildModel(ctrlPoses);
  }

  synchMeas.Time = lidarTime;
  // Interpolate external pose at LiDAR timestamp
  synchMeas.Pose = this->Interpolator(lidarTime);
  // Rotate covariance if required
  if (this->CovarianceRotation)
  {
    Eigen::Isometry3d update = this->ClosestIt->Pose.inverse() * synchMeas.Pose;
    Eigen::Vector6d pose = Utils::IsometryToXYZRPY(this->ClosestIt->Pose);
    synchMeas.Covariance = this->ClosestIt->Covariance;
    CeresTools::RotateCovariance(pose, synchMeas.Covariance, update);
  }

  return true;
}

// ---------------------------------------------------------------------------
bool PoseManager::ComputeSynchronizedMeasureBase(double lidarTime, PoseMeasurement& synchMeas, bool trackTime)
{
  if (!this->ComputeSynchronizedMeasure(lidarTime, synchMeas, trackTime))
    return false;

  // Rotated covariance for calibration if required
  if (this->CovarianceRotation)
  {
    Eigen::Vector6d pose = Utils::IsometryToXYZRPY(synchMeas.Pose);
    CeresTools::RotateCovariance(pose, synchMeas.Covariance, this->Calibration.inverse());
  }

  // Apply calibration
  synchMeas.Pose = synchMeas.Pose * this->Calibration.inverse();

  return true;
}

// ---------------------------------------------------------------------------
bool PoseManager::ComputeConstraint(double lidarTime)
{
  this->ResetResidual();

  if (!this->CanBeUsedLocally())
    return false;

  // Compute synchronized measures representing base frame
  PoseMeasurement synchPrevMeas; // Virtual measure with synchronized timestamp and calibration applied
  // NOTE : If PrevLidarTime has not been set (initialization), no synchronized measure should be found
  if (!this->ComputeSynchronizedMeasureBase(this->PrevLidarTime, synchPrevMeas))
    return false;

  PoseMeasurement synchMeas; // Virtual measure with synchronized timestamp and calibration applied
  if (!this->ComputeSynchronizedMeasureBase(lidarTime, synchMeas))
    return false;

  // Deduce measured relative transform
  Eigen::Vector6d TrelMeas = Utils::IsometryToXYZRPY(synchPrevMeas.Pose.inverse() * synchMeas.Pose);

  this->Residual.Cost = CeresCostFunctions::ExternalPoseResidual::Create(TrelMeas, this->PrevPoseTransform);

  // Use a robustifier to limit the contribution of an outlier tag detection (the tag may have been moved)
  // Tukey loss applied on residual square:
  //   rho(residual^2) = a^2 / 3 * ( 1 - (1 - residual^2 / a^2)^3 )   for residual^2 <= a^2,
  //   rho(residual^2) = a^2 / 3                                      for residual^2 >  a^2.
  // a is the scaling parameter of the function
  // See http://ceres-solver.org/nnls_modeling.html#theory for details
  auto* robustifier = new ceres::TukeyLoss(this->SaturationDistance);

  // Weight the contribution of the given match by its reliability
  // WARNING : in CERES version < 2.0.0, the Tukey loss is badly implemented, so we have to correct the weight by a factor 2
  // See https://github.com/ceres-solver/ceres-solver/commit/6da364713f5b78ddf15b0e0ad92c76362c7c7683 for details
  // This is important for covariance scaling
  #if (CERES_VERSION_MAJOR < 2)
    this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, 2.0 * this->Weight, ceres::TAKE_OWNERSHIP));
  // If Ceres version >= 2.0.0, the Tukey loss is corrected.
  #else
    this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, this->Weight, ceres::TAKE_OWNERSHIP));
  #endif

  return true;
}

// ---------------------------------------------------------------------------
bool PoseManager::CheckBounds(std::list<PoseMeasurement>::iterator prevIt, std::list<PoseMeasurement>::iterator postIt)
{
  if (prevIt == this->Measures.begin() || prevIt == this->Measures.end() ||
      postIt == this->Measures.begin() || postIt == this->Measures.end())
    return false;
  // If the time between the 2 measurements is too short
  // Do not use the current measures
  if (postIt->Time - prevIt->Time < 1e-6)
  {
    if (this->Verbose)
      PRINT_INFO("\t pose measures cannot be used for interpolation (they are identical)")
    return false;
  }
  // If the time between the 2 measurements is too long and the motion is too large
  // Do not use the current measures
  if (postIt->Time - prevIt->Time > this->TimeThreshold)
  {
    bool smallMotion = false;
    // If DistanceThreshold is set, check also the motion diffrence
    // If the motion is too large, do not use the current measures and return false.
    // If the motion is smaller than the DistanceThreshold, the two measures can be interpolated. Return true.
    if (this->DistanceThreshold > 0)
    {
      Eigen::Isometry3d motionTwoMeas = prevIt->Pose.inverse() * postIt->Pose;
      double transTwoMeas = motionTwoMeas.translation().norm();
      if (transTwoMeas < this->DistanceThreshold)
        smallMotion = true;
    }
    if (!smallMotion)
    {
      if (this->Verbose)
          PRINT_INFO(std::fixed << std::setprecision(9)
                                << "\t Pose measures at time " << prevIt->Time << " and " << postIt->Time
                                << " cannot be used for interpolation (too much time difference and too big motion difference)\n"
                                << std::scientific)
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Get pose at a specific timestamp using the IMU
Eigen::Isometry3d PoseManager::GetPose(double time)
{
  std::lock_guard<std::mutex> lock(this->Mtx);
  if (this->Measures.empty())
  {
    PRINT_WARNING("No sensor data, pose cannot be supplied")
    return Eigen::Isometry3d::Identity();
  }
  PoseMeasurement synchMeas;
  // Get synchronized pose with calibration applied
  // trackTime is false because GetPose can be called at any time
  // for any input timestamp so there is no chronological order
  bool trackTime = false;
  if (time >= 0 && this->ComputeSynchronizedMeasureBase(time, synchMeas, trackTime))
    return synchMeas.Pose;
  else
    return this->Measures.back().Pose;
}

// Camera
// ---------------------------------------------------------------------------
bool CameraManager::ComputeSynchronizedMeasure(double lidarTime, Image& synchMeas, bool trackTime)
{
  if (this->Measures.size() <= 1)
    return false;

  std::lock_guard<std::mutex> lock(this->Mtx);
  // Compute the two closest measures to current Lidar frame
  lidarTime -= this->TimeOffset;
  auto bounds = this->GetMeasureBounds(lidarTime, trackTime);
  if (bounds.first == bounds.second)
    return false;

  // Get closest measure to LiDAR timestamp
  float diffTime1 = std::abs(bounds.first->Time - lidarTime);
  float diffTime2 = std::abs(bounds.second->Time - lidarTime);
  synchMeas.Time = diffTime1 < diffTime2 ? bounds.first->Time : bounds.second->Time;
  synchMeas.Data = diffTime1 < diffTime2 ? bounds.first->Data : bounds.second->Data;

  return true;
}

// ---------------------------------------------------------------------------
bool CameraManager::ComputeConstraint(double lidarTime)
{
  this->ResetResidual();

  if (!this->CanBeUsedLocally() || !this->PrevLidarFrame)
    return false;

  // Get synchronized image
  Image prevImage;
  if (!this->ComputeSynchronizedMeasure(this->PrevLidarTime, prevImage))
    return false;

  Image currentImage;
  if (!this->ComputeSynchronizedMeasure(lidarTime, currentImage))
    return false;

  // Reject if one image per Lidar frame has not been found
  if (std::abs(currentImage.Time - prevImage.Time) < 1e-6)
    return false;

  this->Residuals.clear();
  this->Residuals.reserve(this->PrevLidarFrame->size());

  std::vector<cv::Point2f> prevPixels, currPixels;
  std::vector<int> pointIndices;
  // Extract pixels that contain a Lidar point
  for (unsigned int ptIdx = 0; ptIdx < this->PrevLidarFrame->size(); ++ptIdx)
  {
    const Eigen::Vector3f& pt = this->PrevLidarFrame->at(ptIdx).getVector3fMap();
    if (pt.x() <= 0)
      continue;
    // Compute pixel in previous image corresponding to point
    Eigen::Vector2f prevPix = (this->IntrinsicCalibration * (this->Calibration.cast<float>().inverse() * pt)).hnormalized();

    // Add constraint for this pixel if it exists
    if (prevPix[0] >= 0 && prevPix[0] < prevImage.Data.cols &&
        prevPix[1] >= 0 && prevPix[1] < prevImage.Data.rows)
    {
      prevPixels.push_back({prevPix[0], prevPix[1]});
      pointIndices.push_back(ptIdx);
    }
  }

  // Convert images to grayscale
  cv::Mat flow(currentImage.Data.size(), CV_32FC2);
  cv::Mat prevImageGray;
  cvtColor(prevImage.Data, prevImageGray, cv::COLOR_BGR2GRAY);
  cv::Mat currentImageGray;
  cvtColor(currentImage.Data, currentImageGray, cv::COLOR_BGR2GRAY);
  // Compute sparse optical flow with opencv
  // on pixels which contain a Lidar point
  std::vector<uchar> status;
  std::vector<float> err;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
  cv::calcOpticalFlowPyrLK(prevImageGray, currentImageGray, prevPixels, currPixels, status, err, cv::Size(15,15), 2, criteria);

  std::vector<bool> validity(prevPixels.size(), false);

  // Struct to store left and right neighbors on previous and current images
  std::unordered_map<int, int> leftNeighbors;
  std::vector<int> scanLines(prevPixels.size());
  // Check validity for each pixel match
  for (unsigned int i = 0; i < prevPixels.size(); ++i)
  {
    // Shortcut to point
    auto& point = this->PrevLidarFrame->at(pointIndices[i]);

    // Remove non consistent matches
    // No optical flow
    if (!status[i])
      continue;

    auto prevLeft = leftNeighbors.count(point.laser_id) ? prevPixels[leftNeighbors[point.laser_id]]
                                                        : prevPixels[i];

    auto currLeft = leftNeighbors.count(point.laser_id) ? currPixels[leftNeighbors[point.laser_id]]
                                                        : currPixels[i];

    leftNeighbors[point.laser_id] = i;
    scanLines[i] = point.laser_id;

    // Pixels not on image
    if ((currPixels[i].x < 0 && currPixels[i].x >= prevImage.Data.cols) ||
        (currPixels[i].y < 0 && currPixels[i].y >= prevImage.Data.rows))
      continue;

    // Optical flow too high
    if (cv::norm(prevPixels[i] - currPixels[i]) > 0.2 * prevImage.Data.rows)
      continue;

    // Geometric inconsistency between left and current pixels
    float prevDist = cv::norm(prevPixels[i] - prevLeft);
    float currDist = cv::norm(currPixels[i] - currLeft);
    if (currDist > std::max(5.f, 2.f * prevDist))
    {
      validity[leftNeighbors[point.laser_id]] = false;
      continue;
    }

    float radius = 10.f;
    bool uniformArea = false;
    if (currPixels[i].x > radius && currPixels[i].x < prevImage.Data.cols - radius &&
        currPixels[i].y > radius && currPixels[i].y < prevImage.Data.rows - radius)
    {
      float color = prevImageGray.at<float>(int(currPixels[i].y), int(currPixels[i].x));
      for (int radX = -radius/2; radX <= radius/2; ++radX)
      {
        for (int radY = -radius/2; radY <= radius/2; ++radY)
        {
          int X = int(currPixels[i].x) + radX;
          int Y = int(currPixels[i].y) + radY;
          uniformArea = uniformArea && prevImageGray.at<float>(Y, X) == color;
          if (uniformArea)
            break;
        }
        if (uniformArea)
          break;
      }

      if (uniformArea)
        continue;
    }

    validity[i] = true;
  }

  // Create constraint for each pixel match
  for (unsigned int i = 0; i < prevPixels.size(); ++i)
  {
    if (!validity[i])
      continue;

    Eigen::Vector3f pt = this->PrevLidarFrame->at(pointIndices[i]).getVector3fMap();
    Eigen::Vector2f pix = Eigen::Vector2f({currPixels[i].x, currPixels[i].y});
    this->Residual.Cost = CeresCostFunctions::CameraResidual::Create(pt, pix, this->PrevPoseTransform, this->Calibration, this->IntrinsicCalibration);

    // Use a robustifier to limit the contribution of an outlier tag detection (the tag may have been moved)
    // Tukey loss applied on residual square:
    //   rho(residual^2) = a^2 / 3 * ( 1 - (1 - residual^2 / a^2)^3 )   for residual^2 <= a^2,
    //   rho(residual^2) = a^2 / 3                                      for residual^2 >  a^2.
    // a is the scaling parameter of the function
    // See http://ceres-solver.org/nnls_modeling.html#theory for details
    auto* robustifier = new ceres::TukeyLoss(this->SaturationDistance);

    // Weight the contribution of the given match by its reliability
    // WARNING : in CERES version < 2.0.0, the Tukey loss is badly implemented, so we have to correct the weight by a factor 2
    // See https://github.com/ceres-solver/ceres-solver/commit/6da364713f5b78ddf15b0e0ad92c76362c7c7683 for details
    // This is important for covariance scaling
    #if (CERES_VERSION_MAJOR < 2)
      this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, 2.0 * this->Weight, ceres::TAKE_OWNERSHIP));
    // If Ceres version >= 2.0.0, the Tukey loss is corrected.
    #else
      this->Residual.Robustifier.reset(new ceres::ScaledLoss(robustifier, this->Weight, ceres::TAKE_OWNERSHIP));
    #endif

    // Add current pix to pix residual to residuals vector
    this->Residuals.emplace_back(this->Residual);
  }

  return true;
}

} // end of ExternalSensors namespace
} // end of LidarSlam namespace