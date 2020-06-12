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

// This custom macro is needed to make the SlamManager time agnostic
// The SlamManager need to know when RequestData is call, if it's due
// to a new timestep been requested or due to Slam parameters been changed.
// By keeping track of the last time the parameters been modified there is
// no ambiguty anymore. This mecanimsm is similar to the one usedby the paraview filter
// PlotDataOverTime
#define vtkCustomSetMacro(name, type)                                                            \
virtual void Set##name(type _arg)                                                                \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg);  \
  if (this->SlamAlgo->Get##name() != _arg)                                                       \
  {                                                                                              \
    this->SlamAlgo->Set##name(_arg);                                                             \
    this->Modified();                                                                            \
    this->ParametersModificationTime.Modified();                                                 \
  }                                                                                              \
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

class VTK_EXPORT vtkSlam : public vtkPolyDataAlgorithm
{
public:
  static vtkSlam* New();
  vtkTypeMacro(vtkSlam, vtkPolyDataAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent);

  // ---------------------------------------------------------------------------
  //   General stuff and flags
  // ---------------------------------------------------------------------------

  void Reset();

  vtkGetMacro(AdvancedReturnMode, bool)
  virtual void SetAdvancedReturnMode(bool _arg);

  vtkGetMacro(OutputCurrentKeypoints, bool)
  vtkSetMacro(OutputCurrentKeypoints, bool)

  vtkGetMacro(OutputKeypointsMaps, bool)
  vtkSetMacro(OutputKeypointsMaps, bool)

  vtkGetMacro(OutputKeypointsInWorldCoordinates, bool)
  vtkSetMacro(OutputKeypointsInWorldCoordinates, bool)

  vtkCustomGetMacro(FastSlam, bool)
  vtkCustomSetMacro(FastSlam, bool)

  vtkCustomGetMacro(Verbosity, int)
  vtkCustomSetMacro(Verbosity, int)

  vtkCustomGetMacro(NbThreads, int)
  vtkCustomSetMacro(NbThreads, int)

  int GetEgoMotion();
  void SetEgoMotion(int mode);

  int GetUndistortion();
  void SetUndistortion(int mode);

  // ---------------------------------------------------------------------------
  //   BASE to LIDAR transform
  // ---------------------------------------------------------------------------

  void SetBaseToLidarTranslation(double x, double y, double z);

  void SetBaseToLidarRotation(double rx, double ry, double rz);

  // ---------------------------------------------------------------------------
  //   Optimization parameters
  // ---------------------------------------------------------------------------

  vtkCustomGetMacro(MaxDistanceForICPMatching, double)
  vtkCustomSetMacro(MaxDistanceForICPMatching, double)

  // Get/Set EgoMotion
  vtkCustomGetMacro(EgoMotionLMMaxIter, unsigned int)
  vtkCustomSetMacro(EgoMotionLMMaxIter, unsigned int)

  vtkCustomGetMacro(EgoMotionICPMaxIter, unsigned int)
  vtkCustomSetMacro(EgoMotionICPMaxIter, unsigned int)

  vtkCustomGetMacro(EgoMotionLineDistanceNbrNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionLineDistanceNbrNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionMinimumLineNeighborRejection, unsigned int)
  vtkCustomSetMacro(EgoMotionMinimumLineNeighborRejection, unsigned int)

  vtkCustomGetMacro(EgoMotionLineDistancefactor, double)
  vtkCustomSetMacro(EgoMotionLineDistancefactor, double)

  vtkCustomGetMacro(EgoMotionPlaneDistanceNbrNeighbors, unsigned int)
  vtkCustomSetMacro(EgoMotionPlaneDistanceNbrNeighbors, unsigned int)

  vtkCustomGetMacro(EgoMotionPlaneDistancefactor1, double)
  vtkCustomSetMacro(EgoMotionPlaneDistancefactor1, double)

  vtkCustomGetMacro(EgoMotionPlaneDistancefactor2, double)
  vtkCustomSetMacro(EgoMotionPlaneDistancefactor2, double)

  vtkCustomGetMacro(EgoMotionMaxLineDistance, double)
  vtkCustomSetMacro(EgoMotionMaxLineDistance, double)

  vtkCustomGetMacro(EgoMotionMaxPlaneDistance, double)
  vtkCustomSetMacro(EgoMotionMaxPlaneDistance, double)

  vtkCustomGetMacro(EgoMotionInitLossScale, double)
  vtkCustomSetMacro(EgoMotionInitLossScale, double)

  vtkCustomGetMacro(EgoMotionFinalLossScale, double)
  vtkCustomSetMacro(EgoMotionFinalLossScale, double)

  // Get/Set Localization
  vtkCustomGetMacro(LocalizationLMMaxIter, unsigned int)
  vtkCustomSetMacro(LocalizationLMMaxIter, unsigned int)

  vtkCustomGetMacro(LocalizationICPMaxIter, unsigned int)
  vtkCustomSetMacro(LocalizationICPMaxIter, unsigned int)

  vtkCustomGetMacro(LocalizationLineDistanceNbrNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationLineDistanceNbrNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationMinimumLineNeighborRejection, unsigned int)
  vtkCustomSetMacro(LocalizationMinimumLineNeighborRejection, unsigned int)

  vtkCustomGetMacro(LocalizationLineDistancefactor, double)
  vtkCustomSetMacro(LocalizationLineDistancefactor, double)

  vtkCustomGetMacro(LocalizationPlaneDistanceNbrNeighbors, unsigned int)
  vtkCustomSetMacro(LocalizationPlaneDistanceNbrNeighbors, unsigned int)

  vtkCustomGetMacro(LocalizationPlaneDistancefactor1, double)
  vtkCustomSetMacro(LocalizationPlaneDistancefactor1, double)

  vtkCustomGetMacro(LocalizationPlaneDistancefactor2, double)
  vtkCustomSetMacro(LocalizationPlaneDistancefactor2, double)

  vtkCustomGetMacro(LocalizationMaxLineDistance, double)
  vtkCustomSetMacro(LocalizationMaxLineDistance, double)

  vtkCustomGetMacro(LocalizationMaxPlaneDistance, double)
  vtkCustomSetMacro(LocalizationMaxPlaneDistance, double)

  vtkCustomGetMacro(LocalizationLineMaxDistInlier, double)
  vtkCustomSetMacro(LocalizationLineMaxDistInlier, double)

  vtkCustomGetMacro(LocalizationInitLossScale, double)
  vtkCustomSetMacro(LocalizationInitLossScale, double)

  vtkCustomGetMacro(LocalizationFinalLossScale, double)
  vtkCustomSetMacro(LocalizationFinalLossScale, double)

  // ---------------------------------------------------------------------------
  //   Rolling grid parameters and Keypoints extractor
  // ---------------------------------------------------------------------------

  // Key points extractor
  vtkGetObjectMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor)
  virtual void SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor*);

  // Set RollingGrid Parameters
  void SetVoxelGridLeafSizeEdges(double size);
  void SetVoxelGridLeafSizePlanes(double size);
  void SetVoxelGridLeafSizeBlobs(double size);
  void SetVoxelGridSize(int size);
  void SetVoxelGridResolution(double resolution);

protected:
  vtkSlam();

  int FillInputPortInformation(int port, vtkInformation* info) override;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  // Keeps track of the time the parameters have been modified
  // This will enable the SlamManager to be time-agnostic
  // MTime is a much more general mecanism so we can't rely on it
  vtkTimeStamp ParametersModificationTime;

  std::shared_ptr<Slam> SlamAlgo;
  vtkSpinningSensorKeypointExtractor* KeyPointsExtractor = nullptr;

private:
  vtkSlam(const vtkSlam&) = delete;
  void operator=(const vtkSlam&) = delete;

  // Polydata which represents the trajectory computed
  vtkSmartPointer<vtkPolyData> Trajectory;
  std::vector<size_t> GetLaserIdMapping(vtkTable* calib);

  // Add current SLAM pose and covariance in WORLD coordinates to Trajectory.
  void AddCurrentPoseToTrajectory();

  // If enabled, advanced return mode will add arrays to outputs showing some
  // additional results or info of the SLAM algorithm such as :
  //  - Trajectory : matching summary, localization error summary
  //  - Output transformed frame : saliency, planarity, intensity gap, keypoint validity
  //  - Extracted keypoints : ICP matching results
  bool AdvancedReturnMode = false;

  // If enabled, SLAM filter will output keypoints maps.
  // Otherwise, these filter outputs are left empty to save time.
  bool OutputKeypointsMaps = true;

  // If enabled, SLAM filter will output keypoints extracted from current
  // frame. Otherwise, these filter outputs are left empty to save time.
  bool OutputCurrentKeypoints = true;

  // If disabled, return raw keypoints extracted from current frame in BASE
  // coordinates, without undistortion. If enabled, return keypoints in WORLD
  // coordinates, optionally undistorted if undistortion is activated.
  // Only used if OutputCurrentKeypoints = true.
  bool OutputKeypointsInWorldCoordinates = true;
};

#endif // VTK_SLAM_H