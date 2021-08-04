//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2018-03-27
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

#ifndef VTK_SLAM_H
#define VTK_SLAM_H

// VTK
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

// LOCAL
#include <LidarSlam/Slam.h>

// This custom macro is needed to make the SlamManager time agnostic.
// The SlamManager needs to know when RequestData is called, if it's due
// to a new timestep being requested or due to SLAM parameters being changed.
// By keeping track of the last time the parameters have been modified there is
// no ambiguity anymore. This mecanimsm is similar to the one used by the
// paraview filter PlotDataOverTime.
#define vtkCustomSetMacro(name, type)                                                            \
virtual void Set##name(type _arg)                                                                \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg);  \
  if (this->SlamAlgo->Get##name() != _arg)                                                       \
  {                                                                                              \
    this->SlamAlgo->Set##name(_arg);                                                             \
    this->ParametersModificationTime.Modified();                                                 \
  }                                                                                              \
}
#define vtkCustomSetMacroNoCheck(name, type)                                                     \
virtual void Set##name(type _arg)                                                                \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg);  \
  this->SlamAlgo->Set##name(_arg);                                                               \
  this->ParametersModificationTime.Modified();                                                   \
}

#define vtkCustomGetMacro(name, type)                                                            \
virtual type Get##name()                                                                         \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): returning " << #name " of "       \
                << this->SlamAlgo->Get##name());                                                 \
  return this->SlamAlgo->Get##name();                                                            \
}

class vtkSpinningSensorKeypointExtractor;
class vtkTable;

class vtkSlam : public vtkPolyDataAlgorithm
{
public:
  static vtkSlam* New();
  vtkTypeMacro(vtkSlam, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent) override;

  virtual vtkMTimeType GetMTime() override;

  // ---------------------------------------------------------------------------
  //   General stuff and flags
  // ---------------------------------------------------------------------------

  void Reset();

  // Initialization
  void SetInitialMap(const std::string& mapsPathPrefix);
  void SetInitialPoseTranslation(double x, double y, double z);
  void SetInitialPoseRotation(double roll, double pitch, double yaw);

  // Getters / Setters
  vtkGetMacro(AutoDetectInputArrays, bool)
  vtkSetMacro(AutoDetectInputArrays, bool)

  vtkGetMacro(TimeToSecondsFactorSetting, double)
  vtkSetMacro(TimeToSecondsFactorSetting, double)

  vtkGetMacro(AdvancedReturnMode, bool)
  virtual void SetAdvancedReturnMode(bool _arg);

  vtkGetMacro(OutputCurrentKeypoints, bool)
  vtkSetMacro(OutputCurrentKeypoints, bool)

  vtkGetMacro(OutputKeypointsMaps, bool)
  vtkSetMacro(OutputKeypointsMaps, bool)

  vtkGetMacro(MapsUpdateStep, unsigned int)
  vtkSetMacro(MapsUpdateStep, unsigned int)

  vtkGetMacro(OutputKeypointsInWorldCoordinates, bool)
  vtkSetMacro(OutputKeypointsInWorldCoordinates, bool)

  vtkCustomGetMacro(UseBlobs, bool)
  vtkCustomSetMacro(UseBlobs, bool)

  vtkCustomGetMacro(Verbosity, int)
  vtkCustomSetMacro(Verbosity, int)

  vtkCustomGetMacro(NbThreads, int)
  vtkCustomSetMacro(NbThreads, int)

  virtual int GetEgoMotion();
  virtual void SetEgoMotion(int mode);

  virtual int GetUndistortion();
  virtual void SetUndistortion(int mode);

  // Set measurements to Slam algo
  virtual void SetSensorData(const std::string& fileName);

  // ---------------------------------------------------------------------------
  //   BASE to LIDAR transform
  // ---------------------------------------------------------------------------

  virtual void SetBaseToLidarTranslation(double x, double y, double z);

  virtual void SetBaseToLidarRotation(double rx, double ry, double rz);

  // ---------------------------------------------------------------------------
  //   Optimization parameters
  // ---------------------------------------------------------------------------

  vtkCustomGetMacro(TwoDMode, bool)
  vtkCustomSetMacro(TwoDMode, bool)

  // Get/Set EgoMotion
  vtkCustomGetMacro(EgoMotionLMMaxIter, unsigned int)
  vtkCustomSetMacro(EgoMotionLMMaxIter, unsigned int)

  vtkCustomGetMacro(EgoMotionICPMaxIter, unsigned int)
  vtkCustomSetMacro(EgoMotionICPMaxIter, unsigned int)

  vtkCustomGetMacro(EgoMotionMaxNeighborsDistance, double)
  vtkCustomSetMacro(EgoMotionMaxNeighborsDistance, double)

  vtkCustomGetMacro(EgoMotionEdgeNbNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionEdgeNbNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionEdgeMinNbNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionEdgeMinNbNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionEdgePcaFactor, double)
  vtkCustomSetMacro(EgoMotionEdgePcaFactor, double)

  vtkCustomGetMacro(EgoMotionEdgeMaxModelError, double)
  vtkCustomSetMacro(EgoMotionEdgeMaxModelError, double)

  vtkCustomGetMacro(EgoMotionPlaneNbNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionPlaneNbNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionPlanePcaFactor1, double)
  vtkCustomSetMacro(EgoMotionPlanePcaFactor1, double)

  vtkCustomGetMacro(EgoMotionPlanePcaFactor2, double)
  vtkCustomSetMacro(EgoMotionPlanePcaFactor2, double)

  vtkCustomGetMacro(EgoMotionPlaneMaxModelError, double)
  vtkCustomSetMacro(EgoMotionPlaneMaxModelError, double)

  vtkCustomGetMacro(EgoMotionInitSaturationDistance, double)
  vtkCustomSetMacro(EgoMotionInitSaturationDistance, double)

  vtkCustomGetMacro(EgoMotionFinalSaturationDistance, double)
  vtkCustomSetMacro(EgoMotionFinalSaturationDistance, double)

  // Get/Set Localization
  vtkCustomGetMacro(LocalizationLMMaxIter, unsigned int)
  vtkCustomSetMacro(LocalizationLMMaxIter, unsigned int)

  vtkCustomGetMacro(LocalizationICPMaxIter, unsigned int)
  vtkCustomSetMacro(LocalizationICPMaxIter, unsigned int)

  vtkCustomGetMacro(LocalizationMaxNeighborsDistance, double)
  vtkCustomSetMacro(LocalizationMaxNeighborsDistance, double)

  vtkCustomGetMacro(LocalizationEdgeNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationEdgeNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationEdgeMinNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationEdgeMinNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationEdgePcaFactor, double)
  vtkCustomSetMacro(LocalizationEdgePcaFactor, double)

  vtkCustomGetMacro(LocalizationEdgeMaxModelError, double)
  vtkCustomSetMacro(LocalizationEdgeMaxModelError, double)

  vtkCustomGetMacro(LocalizationPlaneNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationPlaneNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationPlanePcaFactor1, double)
  vtkCustomSetMacro(LocalizationPlanePcaFactor1, double)

  vtkCustomGetMacro(LocalizationPlanePcaFactor2, double)
  vtkCustomSetMacro(LocalizationPlanePcaFactor2, double)

  vtkCustomGetMacro(LocalizationPlaneMaxModelError, double)
  vtkCustomSetMacro(LocalizationPlaneMaxModelError, double)

  vtkCustomGetMacro(LocalizationBlobNbNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationBlobNbNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationInitSaturationDistance, double)
  vtkCustomSetMacro(LocalizationInitSaturationDistance, double)

  vtkCustomGetMacro(LocalizationFinalSaturationDistance, double)
  vtkCustomSetMacro(LocalizationFinalSaturationDistance, double)

  vtkCustomGetMacro(WheelOdomWeight, double)
  vtkCustomSetMacro(WheelOdomWeight, double)

  vtkCustomGetMacro(GravityWeight, double)
  vtkCustomSetMacro(GravityWeight, double)

  // ---------------------------------------------------------------------------
  //   Keypoints extractor, Key frames and Maps parameters
  // ---------------------------------------------------------------------------

  // Keypoints extractor
  vtkGetObjectMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor)
  virtual void SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor*);

  // Key frames
  vtkCustomGetMacro(KfDistanceThreshold, double)
  vtkCustomSetMacro(KfDistanceThreshold, double)

  vtkCustomGetMacro(KfAngleThreshold, double)
  vtkCustomSetMacro(KfAngleThreshold, double)

  // Set RollingGrid Parameters

  virtual int GetVoxelGridSamplingMode(LidarSlam::Keypoint k);
  virtual void SetVoxelGridSamplingMode(LidarSlam::Keypoint k, int sm);

  // For edges
  virtual int GetVoxelGridSamplingModeEdges() { return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::EDGE); }
  virtual void SetVoxelGridSamplingModeEdges(int sm)  { this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::EDGE, sm);  }

  // For planes
  virtual int GetVoxelGridSamplingModePlanes() { return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::PLANE); }
  virtual void SetVoxelGridSamplingModePlanes(int sm) { this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::PLANE, sm); }

  // For blobs
  virtual int GetVoxelGridSamplingModeBlobs() { return this->GetVoxelGridSamplingMode(LidarSlam::Keypoint::BLOB); }
  virtual void SetVoxelGridSamplingModeBlobs(int sm)  { this->SetVoxelGridSamplingMode(LidarSlam::Keypoint::BLOB, sm);  }

  virtual void SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s);

  // For edges
  virtual void SetVoxelGridLeafSizeEdges(double s)  { this->SetVoxelGridLeafSize(LidarSlam::Keypoint::EDGE, s);  }

  // For planes
  virtual void SetVoxelGridLeafSizePlanes(double s) { this->SetVoxelGridLeafSize(LidarSlam::Keypoint::PLANE, s); }

  // For blobs
  virtual void SetVoxelGridLeafSizeBlobs(double s)  { this->SetVoxelGridLeafSize(LidarSlam::Keypoint::BLOB, s);  }

  vtkCustomSetMacroNoCheck(VoxelGridSize, int)
  vtkCustomSetMacroNoCheck(VoxelGridResolution, double)

  // ---------------------------------------------------------------------------
  //   Confidence estimator parameters
  // ---------------------------------------------------------------------------

  vtkCustomGetMacro(OverlapSamplingRatio, double)
  virtual void SetOverlapSamplingRatio (double ratio);

  // Motion constraints
  virtual void SetAccelerationLimits(float linearAcc, float angularAcc);
  virtual void SetVelocityLimits(float linearVel, float angularVel);

  virtual void SetTimeWindowDuration(float time);
  vtkCustomGetMacro(TimeWindowDuration, float)

protected:
  vtkSlam();

  int FillInputPortInformation(int port, vtkInformation* info) override;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkSlam(const vtkSlam&) = delete;
  void operator=(const vtkSlam&) = delete;

  // ---------------------------------------------------------------------------
  //   Useful helpers
  // ---------------------------------------------------------------------------

  // Identify input arrays to use
  void IdentifyInputArrays(vtkPolyData* poly, vtkTable* calib);

  // Convert LiDAR calibration to laser id mapping
  std::vector<size_t> GetLaserIdMapping(vtkTable* calib);

  // Add current SLAM pose and covariance in WORLD coordinates to Trajectory.
  void AddCurrentPoseToTrajectory();

  // Convert VTK PolyData to PCL pointcloud
  // Returns true if all input points are valid (null coordinates), false otherwise
  bool PolyDataToPointCloud(vtkPolyData* poly,
                            LidarSlam::Slam::PointCloud::Ptr pc,
                            const std::vector<size_t>& laserIdMapping) const;

  // Convert PCL pointcloud to VTK PolyData
  void PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr pc,
                            vtkPolyData* poly) const;

  // ---------------------------------------------------------------------------
  //   Member attributes
  // ---------------------------------------------------------------------------

protected:

  // Keeps track of the time the parameters have been modified
  // This will enable the SlamManager to be time-agnostic
  // MTime is a much more general mecanism so we can't rely on it
  vtkTimeStamp ParametersModificationTime;

  std::unique_ptr<LidarSlam::Slam> SlamAlgo;
  vtkSpinningSensorKeypointExtractor* KeyPointsExtractor = nullptr;

private:

  // Polydata which represents the computed trajectory
  vtkSmartPointer<vtkPolyData> Trajectory;

  // If enabled, advanced return mode will add arrays to outputs showing some
  // additional results or info of the SLAM algorithm such as :
  //  - Trajectory : matching summary, localization error summary
  //  - Output transformed frame : saliency, planarity, intensity gap, keypoint validity
  //  - Extracted keypoints : ICP matching results
  bool AdvancedReturnMode = false;

  // If enabled, SLAM filter will output keypoints maps.
  // Otherwise, these filter outputs are left empty to save time.
  bool OutputKeypointsMaps = true;
  // Update keypoints maps only each MapsUpdateStep frame
  // (ex: every frame, every 2 frames, 3 frames, ...)
  unsigned int MapsUpdateStep = 1;

  // If enabled, SLAM filter will output keypoints extracted from current
  // frame. Otherwise, these filter outputs are left empty to save time.
  bool OutputCurrentKeypoints = true;

  // If disabled, return raw keypoints extracted from current frame in BASE
  // coordinates, without undistortion. If enabled, return keypoints in WORLD
  // coordinates, optionally undistorted if undistortion is activated.
  // Only used if OutputCurrentKeypoints = true.
  bool OutputKeypointsInWorldCoordinates = true;

  // Arrays to use (depending on LiDAR model) to fill points data
  bool AutoDetectInputArrays = true;   ///< If true, try to auto-detect arrays to use. Otherwise, user needs to specify them.
  std::string TimeArrayName;           ///< Point measurement timestamp
  std::string IntensityArrayName;      ///< Point intensity/reflectivity values
  std::string LaserIdArrayName;        ///< Laser ring id
  std::string VerticalCalibArrayName;  ///< Calibration column used to sort laser rings by elevation angle
  double TimeToSecondsFactor;          ///< Coef to apply to TimeArray values to express time in seconds
  double TimeToSecondsFactorSetting;   ///< Duplicated parameter used to store the value set by user

  // SLAM initialization
  std::string InitMapPrefix; ///< Path prefix of initial maps
  Eigen::Vector6d InitPose;  ///< Initial pose of the SLAM

  // Internal variable to store overlap sampling ratio when advanced return mode is disabled.
  float OverlapSamplingRatio = 0.25;
  // Internal variable to store window time to estimate local velocity when advanced return mode is disabled.
  float TimeWindowDuration = 0.5;
};

#endif // VTK_SLAM_H
