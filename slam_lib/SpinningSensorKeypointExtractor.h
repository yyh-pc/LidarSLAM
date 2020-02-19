//=========================================================================
//
// Copyright 2018 Kitware, Inc.
// Author: Guilbert Pierre (spguilbert@gmail.com)
//         Laurenson Nick (nlaurenson5@gmail.com)
// Date: 03-27-2018
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
//=========================================================================
#ifndef SpinningSensorKeypointExtractor_H
#define SpinningSensorKeypointExtractor_H

#include <vector>
#include <unordered_map>

#include <pcl/point_cloud.h>

#include "LidarPoint.h"

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

class SpinningSensorKeypointExtractor
{
public:
  using Point = PointXYZTIId;
  using PointCloud = pcl::PointCloud<Point>;

  GetMacro(NeighborWidth, int)
  SetMacro(NeighborWidth, int)

  GetMacro(MinDistanceToSensor, double)
  SetMacro(MinDistanceToSensor, double)

  GetMacro(AngleResolution, double)
  SetMacro(AngleResolution, double)

  GetMacro(PlaneSinAngleThreshold, double)
  SetMacro(PlaneSinAngleThreshold, double)

  GetMacro(EdgeSinAngleThreshold, double)
  SetMacro(EdgeSinAngleThreshold, double)

  GetMacro(EdgeDepthGapThreshold, double)
  SetMacro(EdgeDepthGapThreshold, double)

  GetMacro(EdgeSaliencyThreshold, double)
  SetMacro(EdgeSaliencyThreshold, double)

  GetMacro(EdgeIntensityGapThreshold, double)
  SetMacro(EdgeIntensityGapThreshold, double)

  GetMacro(FarestKeypointDist, double)
  Eigen::Vector3d GetMinPoint() { return this->MinPoint.cast<double>();}
  Eigen::Vector3d GetMaxPoint() { return this->MaxPoint.cast<double>();}

  GetMacro(NLasers, int)

  PointCloud::Ptr GetEdgePoints() { return this->EdgesPoints; }
  PointCloud::Ptr GetPlanarPoints() { return this->PlanarsPoints; }
  PointCloud::Ptr GetBlobPoints() { return this->BlobsPoints; }

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature
  void ComputeKeyPoints(const PointCloud::Ptr& pc, const std::vector<size_t>& laserIdMapping);

  // Function to enable to have some inside on why a given point was detected as a keypoint
  std::unordered_map<std::string, std::vector<double>> GetDebugArray();

private:

  // Reset all mumbers variables that are
  // used during the process of a frame.
  void PrepareDataForNextFrame();

  // Convert the input vtk-format pointcloud
  // into a pcl-pointcloud format. scan lines
  // will also be sorted by their vertical angles
  void ConvertAndSortScanLines();

  // Compute the curvature of the scan lines
  // The curvature is not the one of the surface
  // that intersected the lines but the curvature
  // of the scan lines taken in an isolated way
  void ComputeCurvature();

  // Invalid the points with bad criteria from
  // the list of possible future keypoints.
  // This points correspond to planar surface
  // roughtly parallel to laser beam and points
  // close to a gap created by occlusion
  void InvalidPointWithBadCriteria();

  // Labelizes point to be a keypoints or not
  void SetKeyPointsLabels();

  // Check if scanLine is almost empty
  inline bool IsScanLineAlmostEmpty(size_t nScanLinePts) { return nScanLinePts < 2 * this->NeighborWidth + 1; }

  // Width of the neighborhood used to compute discrete differential operators
  int NeighborWidth = 4;

  // Minimal point/sensor sensor to consider a point as valid
  double MinDistanceToSensor = 3.0;  // [m]

  // Maximal angle resolution of the lidar azimutal resolution.
  // (default value to VLP-16. We add an extra 20%)
  double AngleResolution = DEG2RAD(0.4);  // [rad]

  // Sharpness threshold to select a planar keypoint
  double PlaneSinAngleThreshold = 0.5;  // sin(30°) (selected if sin angle is less than threshold)

  // Sharpness threshold to select an edge keypoint
  double EdgeSinAngleThreshold = 0.86;  // ~sin(60°) (selected, if sin angle is more than threshold)
  double DistToLineThreshold = 0.20;  // [m]

  // Threshold upon depth gap in neighborhood to select an edge keypoint
  double EdgeDepthGapThreshold = 0.15;  // [m]

  // Threshold upon saliency of a neighborhood to select an edge keypoint
  double EdgeSaliencyThreshold = 1.5;  // [m]

  // Threshold upon intensity gap to select an edge keypoint
  double EdgeIntensityGapThreshold = 50.;

  // Threshold upon sphericity of a neighborhood to select a blob point
  double SphericityThreshold = 0.35;  // CHECK : unused

  // Coef to apply to the incertitude radius of the blob neighborhood
  double IncertitudeCoef = 3.0;  // CHECK : unused

  // Mapping of the lasers id
  std::vector<size_t> LaserIdMapping;

  // Number of lasers scan lines composing the pointcloud
  unsigned int NLasers = 0;

  // Distance to the farest keypoint
  double FarestKeypointDist;
  // Minimum and maximum keypoints coordinates
  Eigen::Array3f MinPoint, MaxPoint;

  // Curvature and other differential operations (scan by scan, point by point)
  std::vector<std::vector<double>> Angles;
  std::vector<std::vector<double>> DepthGap;
  std::vector<std::vector<double>> Saliency;
  std::vector<std::vector<double>> IntensityGap;
  std::vector<std::vector<uint8_t>> IsPointValid;
  std::vector<std::vector<uint8_t>> Label;

  // Mapping between keypoints and their corresponding index in pclCurrentFrameByScan
  std::vector<std::pair<int, int>> EdgesIndex;
  std::vector<std::pair<int, int>> PlanarIndex;
  std::vector<std::pair<int, int>> BlobIndex;

  // Extracted keypoints of current frame
  PointCloud::Ptr EdgesPoints;
  PointCloud::Ptr PlanarsPoints;
  PointCloud::Ptr BlobsPoints;

  // Current point cloud stored in two differents formats
  PointCloud::Ptr pclCurrentFrame;
  std::vector<PointCloud::Ptr> pclCurrentFrameByScan;
};

#endif // SpinningSensorKeypointExtractor_H