#include "LidarSlam/SensorConstraints.h"

namespace LidarSlam
{
namespace SensorConstraints
{

void WheelOdometryManager::ComputeWheelAbsoluteConstraint(double lidarTime)
{
  this->ResetResidual();

  if (!this->CanBeUsed())
    return;

  lidarTime -= this->TimeOffset;
  // Check if odometry measurements were taken around Lidar Frame acquisition
  if (lidarTime < this->Measures.front().Time || lidarTime > this->Measures.back().Time)
  {
    PRINT_WARNING("No odometry measure corresponds to the current frame acquisition (times don't match)");
    return;
  }

  // Reset if the timeline has been modified
  if (this->PreviousIdx >= 0 && this->Measures[this->PreviousIdx].Time > lidarTime)
    this->PreviousIdx = -1;

  // Get index of last odometry measurement before LiDAR time
  int currIdx = this->PreviousIdx;
  while (this->Measures[currIdx + 1].Time < lidarTime)
    currIdx++;

  // Interpolate odometry measurement at LiDAR timestamp (between currIdx and currIdx + 1 measures)
  double rt = (lidarTime - this->Measures[currIdx].Time) / (this->Measures[currIdx + 1].Time - this->Measures[currIdx].Time);
  double currDistance = (1 - rt) * this->Measures[currIdx].Distance + rt * this->Measures[currIdx + 1].Distance;

  // Build odometry residual

  // If there is no memory of previous poses
  if (this->PreviousIdx == -1)
  {
    std::cout << "No previous wheel odometry measure : no constraint added to optimization" << std::endl;
    // Update index and distance for next frame
    this->PreviousIdx = currIdx;
    this->PreviousDistance = currDistance;
    return;
  }

  // If there is memory of a previous pose
  this->Residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(this->PreviousPose.translation(), currDistance - this->PreviousDistance);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  std::cout << "Adding wheel odometry residual : " << currDistance - this->PreviousDistance << " m travelled since first frame." << std::endl;

  // Update index for next frame
  this->PreviousIdx = currIdx;
}

void WheelOdometryManager::ComputeWheelOdomConstraint(double lidarTime)
{
  this->ResetResidual();

  if (!this->CanBeUsed())
    return;
  // Index of measurements used for this frame
  lidarTime -= this->TimeOffset;

  // Check if odometry measurements were taken around Lidar Frame acquisition
  if (lidarTime < this->Measures.front().Time || lidarTime > this->Measures.back().Time)
  {
    PRINT_WARNING("No odometry measure corresponds to the current frame acquisition (times don't match)");
    return;
  }

  // Reset if the timeline has been modified
  if (this->PreviousIdx >= 0 && this->Measures[this->PreviousIdx].Time > lidarTime)
    this->PreviousIdx = -1;

  unsigned int currIdx = this->PreviousIdx;
  // Get index of last odometry measurement before LiDAR time
  while (this->Measures[currIdx + 1].Time < lidarTime)
    currIdx++;

  // Interpolate odometry measurement at LiDAR timestamp (between currIdx and currIdx + 1)
  double rt = (lidarTime - this->Measures[currIdx].Time) / (this->Measures[currIdx + 1].Time - this->Measures[currIdx].Time);
  double currDistance = (1 - rt) * this->Measures[currIdx].Distance + rt * this->Measures[currIdx + 1].Distance;

  // Build odometry residual
  // If there is no memory of previous poses
  if (this->PreviousIdx == -1)
  {
    std::cout << "No previous wheel odometry measure : no constraint added to optimization" << std::endl;
    // Update index and distance for next frame
    this->PreviousIdx = currIdx;
    this->PreviousDistance = currDistance;
    return;
  }

  // If there is memory of a previous pose
  double distDiff = std::abs(currDistance - this->PreviousDistance);
  this->Residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(this->PreviousPose.translation(), distDiff);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));
  std::cout << "Adding relative wheel odometry residual : " << distDiff << " m travelled since last frame." << std::endl;

  // Update index and distance for next frame
  this->PreviousIdx = currIdx;
  this->PreviousDistance = currDistance;
}

void ImuManager::ComputeGravityConstraint(double lidarTime)
{
  this->ResetResidual();

  if (!this->CanBeUsed())
    return;

  lidarTime -= this->TimeOffset;
  if (lidarTime < this->Measures.front().Time || lidarTime > this->Measures.back().Time)
  {
    PRINT_WARNING("No IMU measure corresponds to the current frame acquisition (times don't match)");
    return;
  }

  // Compute reference gravity vector
  if (this->GravityRef.norm() < 1e-6)
    this->ComputeGravityRef(Utils::Deg2Rad(5.f));

  // Reset if the timeline has been modified
  if (this->PreviousIdx >= 0 && this->Measures[this->PreviousIdx].Time > lidarTime)
    this->PreviousIdx = -1;

  // Index of measurement used for this frame
  int currIdx = this->PreviousIdx;

  // Get index of last IMU measurement before LiDAR time
  while (this->Measures[currIdx + 1].Time < lidarTime)
    currIdx++;

  // Interpolate gravity measurement at LiDAR timestamp
  double rt = (lidarTime - this->Measures[currIdx].Time) / (this->Measures[currIdx + 1].Time - this->Measures[currIdx].Time);
  Eigen::Vector3d gravityDirection = (1 - rt) * this->Measures[currIdx].Acceleration.normalized() + rt * this->Measures[currIdx + 1].Acceleration.normalized();
  // Normalize interpolated gravity vector
  if (gravityDirection.norm() > 1e-6) // Check to insure consistent IMU measure
    gravityDirection.normalize();
  else
    return;

  // Build gravity constraint
  this->Residual.Cost = CeresCostFunctions::ImuGravityAlignmentResidual::Create(this->GravityRef, gravityDirection);
  this->Residual.Robustifier.reset(new ceres::ScaledLoss(NULL, this->Weight, ceres::TAKE_OWNERSHIP));

  this->PreviousIdx = currIdx;
}

void ImuManager::ComputeGravityRef(double deltaAngle)
{
  // Init histogram 2D (phi and theta)
  int NPhi = std::ceil(2 * M_PI / deltaAngle);
  int NTheta = std::ceil(M_PI / deltaAngle);
  std::vector<std::vector<std::vector<int>>> histogram(NPhi, std::vector<std::vector<int>>(NTheta));

  // Store acceleration vector indices in histogram
  for (unsigned int idxAcc = 0; idxAcc < this->Measures.size(); ++idxAcc)
  {
    Eigen::Vector3d AccelDirection = this->Measures[idxAcc].Acceleration.normalized();
    int idxPhi = ( std::atan2(AccelDirection.y(), AccelDirection.x()) + M_PI ) / deltaAngle;
    int idxTheta = ( std::acos(AccelDirection.z()) ) / deltaAngle;
    histogram[idxPhi][idxTheta].push_back(idxAcc);
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
  for (int idxAcc : histogram[bestPhi][bestTheta])
    this->GravityRef += this->Measures[idxAcc].Acceleration.normalized();
  this->GravityRef.normalize();
  std::cout << "Gravity vector : " << this->GravityRef.transpose() << std::endl;
}

} // end of SensorConstraints namespace
} // end of LidarSlam namespace