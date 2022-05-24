//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Laurenson Nick (Kitware SAS)
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

#include "LidarSlam/Utilities.h"
#include "LidarSlam/SpinningSensorKeypointExtractor.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

namespace LidarSlam
{

namespace
{
//-----------------------------------------------------------------------------
bool LineFitting::FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                            const std::vector<int>& indices)
{
  // Compute PCA to determine best line approximation of the points distribution
  // and save points centroid in Position
  Eigen::Vector3f eigVals;
  Eigen::Matrix3f eigVecs;
  Utils::ComputeMeanAndPCA(cloud, indices, this->Position, eigVecs, eigVals);

  // Get Direction as main eigen vector
  this->Direction = eigVecs.col(2);

  // Compute line features
  float LineLength = (cloud[indices[0]].getVector3fMap() - cloud[indices[indices.size() - 1]].getVector3fMap()).norm();
  float LineWidth = 0;
  for (int idx: indices)
    LineWidth = std::max(LineWidth, this->DistanceToPoint(cloud[idx].getVector3fMap()));


  // If the line is too wide (MSE too high) respectively to its length,
  // we consider the neighborhood as not flat enough
  // If the line is too short, it is considered not trutworthy enough
  if (LineWidth > this->MaxLineWidth || LineLength < this->MinLineLength)
    return false;

  return true;
}

//-----------------------------------------------------------------------------
inline float LineFitting::DistanceToPoint(Eigen::Vector3f const& point) const
{
  return ((point - this->Position).cross(this->Direction)).norm();
}
} // end of anonymous namespace

//-----------------------------------------------------------------------------
float SpinningSensorKeypointExtractor::GetVoxelResolution() const
{
  if (this->Keypoints.empty())
    return -1.;
  return this->Keypoints.begin()->second.GetVoxelResolution();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetVoxelResolution(float res)
{
  for (auto& kptsVG : this->Keypoints)
    kptsVG.second.SetVoxelResolution(res);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::Enable(const std::vector<Keypoint>& kptTypes)
{
  for (auto& en : this->Enabled)
    en.second = false;
  for (auto& k : kptTypes)
    this->Enabled[k] = true;
}

//-----------------------------------------------------------------------------
SpinningSensorKeypointExtractor::PointCloud::Ptr SpinningSensorKeypointExtractor::GetKeypoints(Keypoint k)
{
  if (!this->Enabled.count(k) || !this->Enabled[k])
  {
    PRINT_ERROR("Unable to get keypoints of type " << KeypointTypeNames.at(k));
    return PointCloud::Ptr();
  }

  PointCloud::Ptr keypoints = this->Keypoints.at(k).GetCloud(this->MaxPoints);
  Utils::CopyPointCloudMetadata(*this->Scan, *keypoints);
  return keypoints;
}


//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  // Split whole pointcloud into separate laser ring clouds
  this->ConvertAndSortScanLines();

  // Initialize the features vectors and keypoints
  this->PrepareDataForNextFrame();

  // Invalidate points with bad criteria
  this->InvalidateNotUsablePoints();

  // Compute keypoints scores
  this->ComputeCurvature();

  // Labelize and extract keypoints
  // Warning : order matters
  if (this->Enabled[Keypoint::BLOB])
    this->ComputeBlobs();
  if (this->Enabled[Keypoint::PLANE])
    this->ComputePlanes();
  if (this->Enabled[Keypoint::EDGE])
    this->ComputeEdges();
  if (this->Enabled[Keypoint::INTENSITY_EDGE])
    this->ComputeIntensityEdges();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Clear previous scan lines
  for (auto& scanLineCloud: this->ScanLines)
  {
    // Use clear() if pointcloud already exists to avoid re-allocating memory.
    // No worry as ScanLines is never shared with outer scope.
    if (scanLineCloud)
      scanLineCloud->clear();
    else
      scanLineCloud.reset(new PointCloud);
  }

  // Separate pointcloud into different scan lines
  for (const Point& point: *this->Scan)
  {
    // Ensure that there are enough available scan lines
    while (point.laser_id >= this->ScanLines.size())
      this->ScanLines.emplace_back(new PointCloud);

    // Add the current point to its corresponding laser scan
    this->ScanLines[point.laser_id]->push_back(point);
  }

  // Save the number of lasers
  this->NbLaserRings = this->ScanLines.size();

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution)
    this->EstimateAzimuthalResolution();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Initialize the features vectors with the correct length
  this->Angles.resize(this->NbLaserRings);
  this->Saliency.resize(this->NbLaserRings);
  this->DepthGap.resize(this->NbLaserRings);
  this->IntensityGap.resize(this->NbLaserRings);
  this->IsPointValid.resize(this->NbLaserRings);
  this->Label.resize(this->NbLaserRings);

  // Initialize the scan lines features vectors with the correct length
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    size_t nbPoint = this->ScanLines[scanLine]->size();
    this->IsPointValid[scanLine].assign(nbPoint, true);  // set all points as valid
    this->Label[scanLine].assign(nbPoint, KeypointFlags().reset());  // set all flags to 0
    this->Angles[scanLine].assign(nbPoint, -1.);
    this->Saliency[scanLine].assign(nbPoint, -1.);
    this->DepthGap[scanLine].assign(nbPoint, -1.);
    this->IntensityGap[scanLine].assign(nbPoint, -1.);
  }

  // Reset voxel grids
  LidarPoint minPt, maxPt;
  pcl::getMinMax3D(*this->Scan, minPt, maxPt);
  for (auto k : KeypointTypes)
  {
    if (this->Keypoints.count(k))
      this->Keypoints[k].Clear();
    if (this->Enabled[k])
      this->Keypoints[k].Init(minPt.getVector3fMap(), maxPt.getVector3fMap());
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidateNotUsablePoints()
{
  // Max angle between Lidar Ray and hyptothetic plane normal
  const float angleBeamNormal = Utils::Deg2Rad(90 - this->MinBeamSurfaceAngle);
  // If the azimuthal angle was estimated and is plausible,
  // it is used to invalidate points representing occluded areas border.
  // Otherwise, we use a default angle (Velodyne 10Hz resolution)
  float azimuthalResolution = this->AzimuthalResolution;
  if (azimuthalResolution < 1e-6 || M_PI / 4 < azimuthalResolution)
  {
    PRINT_WARNING("Unable to estimate the azimuthal resolution angle: using 0.2°");
    azimuthalResolution = Utils::Deg2Rad(0.2);
  }
  // Coeff to multiply to point depth, in order to obtain the maximal distance
  // between two neighbors of the same Lidar ray on a plane
  const float maxPosDiffCoeff = std::sin(azimuthalResolution) / std::cos(azimuthalResolution + angleBeamNormal);

  // Loop over scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) firstprivate(maxPosDiffCoeff)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    const int Npts = scanLineCloud.size();

    // If the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      for (int index = 0; index < Npts; ++index)
        this->IsPointValid[scanLine][index] = false;
      continue;
    }

    // Invalidate first and last points: because of undistortion, the depth of
    // first and last points can not be compared
    for (int index = 0; index < this->NeighborWidth; ++index)
    {
      this->IsPointValid[scanLine][index] = false;
      this->IsPointValid[scanLine][Npts - 1 - index] = false;
    }

    // Loop over remaining points of the scan line
    for (int index = this->NeighborWidth; index < Npts - this->NeighborWidth; ++index)
    {
      const auto& currentPoint = scanLineCloud[index].getVector3fMap();
      const float L = currentPoint.norm();
      // Invalidate points which are too close from the sensor
      if (L < this->MinDistanceToSensor)
      {
        this->IsPointValid[scanLine][index] = false;
        continue;
      }

      // Compute maximal acceptable distance of two consecutive neighbors
      // aquired by the same laser, considering they lay on the same plane which
      // is not too oblique relatively to Lidar ray.
      // Check that this expected distance is not below range measurements noise.
      const float maxPosDiff = std::max(L * maxPosDiffCoeff, 0.02f);
      const float sqMaxPosDiff = maxPosDiff * maxPosDiff;

      // Invalidate occluded points due to depth gap or parallel beam.
      // If the distance between two successive points is bigger than the
      // expected length, it means that there is a depth gap. In this case, we
      // must invalidate the farthests points which belong to the occluded area.
      const auto& nextPoint = scanLineCloud[index + 1].getVector3fMap();
      if ((nextPoint - currentPoint).squaredNorm() > sqMaxPosDiff)
      {
        // If current point is the closest, next part is invalidated, starting from next point
        if (L < nextPoint.norm())
        {
          this->IsPointValid[scanLine][index + 1] = false;
          for (int i = index + 1; i < index + this->NeighborWidth; ++i)
          {
            const auto& Y  = scanLineCloud[i].getVector3fMap();
            const auto& Yn = scanLineCloud[i + 1].getVector3fMap();
            // If there is a new gap in the neighborhood,
            // the remaining points of the neighborhood are kept.
            if ((Yn - Y).squaredNorm() > sqMaxPosDiff)
              break;
            // Otherwise, the current neighbor point is disabled
            this->IsPointValid[scanLine][i + 1] = false;
          }
        }
        // If current point is the farthest, invalidate previous part, starting from current point
        else
        {
          this->IsPointValid[scanLine][index] = false;
          for (int i = index - 1; i > index - this->NeighborWidth; --i)
          {
            const auto& Yp = scanLineCloud[i].getVector3fMap();
            const auto&  Y = scanLineCloud[i + 1].getVector3fMap();
            // If there is a new gap in the neighborhood,
            // the remaining points of the neighborhood are kept.
            if ((Y - Yp).squaredNorm() > sqMaxPosDiff)
              break;
            // Otherwise, the previous neighbor point is disabled
            this->IsPointValid[scanLine][i] = false;
          }
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  // loop over scans lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
      continue;

    // loop over points in the current scan line
    // TODO : deal with spherical case : index=0 is neighbor of index=Npts-1
    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // Skip curvature computation for invalid points
      if (!this->IsPointValid[scanLine][index])
        continue;

      // Fill left and right neighborhoods, from central point to sides.
      // /!\ The way the neighbors are added to the vectors matters,
      // especially when computing the saliency
      std::vector<int> leftNeighbors(this->NeighborWidth);
      std::vector<int> rightNeighbors(this->NeighborWidth);
      for (int j = index - 1; j >= index - this->NeighborWidth; --j)
        leftNeighbors[index - 1 - j] = j;
      for (int j = index + 1; j <= index + this->NeighborWidth; ++j)
        rightNeighbors[j - index - 1] = j;

      // Compute intensity gap
      float gap = scanLineCloud[index].intensity - scanLineCloud[index - 1].intensity;
      float depthGap = (scanLineCloud[index - 1].getVector3fMap() - scanLineCloud[index].getVector3fMap()).norm();
      if (gap > this->EdgeIntensityGapThreshold && depthGap < 0.1)
      {
        // Compute mean intensity on the left
        float meanIntensityLeft = 0;
        for (int indexLeft : leftNeighbors)
          meanIntensityLeft += scanLineCloud[indexLeft].intensity;
        meanIntensityLeft /= this->NeighborWidth;
        // Compute mean intensity on the right
        float meanIntensityRight = 0;
        for (int indexRight : rightNeighbors)
          meanIntensityRight += scanLineCloud[indexRight].intensity;
        meanIntensityRight /= this->NeighborWidth;
        this->IntensityGap[scanLine][index] = std::abs(meanIntensityLeft - meanIntensityRight);
      }

      // central point
      const Point& currentPoint = scanLineCloud[index];
      const Eigen::Vector3f centralPoint = currentPoint.getVector3fMap();

      // We will compute the line that fits the neighbors located before the current point.
      // We will do the same for the neighbors located after the current point.
      // We will then compute the angle between these two lines as an approximation
      // of the "sharpness" of the current point.
      LineFitting leftLine, rightLine;

      // Fit line on the left and right neighborhoods and
      // Indicate if they are flat or not
      const bool leftFlat = leftLine.FitPCAAndCheckConsistency(scanLineCloud, leftNeighbors);
      const bool rightFlat = rightLine.FitPCAAndCheckConsistency(scanLineCloud, rightNeighbors);

      // Measurement of the depth gap
      float distLeft = 0., distRight = 0.;

      // If both neighborhoods are flat, we can compute the angle between them
      // as an approximation of the sharpness of the current point
      if (leftFlat && rightFlat)
      {
        // We check that the current point is not too far from its
        // neighborhood lines. This is because we don't want a point
        // to be considered as an angle point if it is due to gap
        distLeft =  leftLine.DistanceToPoint(centralPoint);
        distRight = rightLine.DistanceToPoint(centralPoint);

        // If current point is not too far from estimated lines,
        // save the sin of angle between these two lines
        // Compute angles
        if ((distLeft < this->DistToLineThreshold) && (distRight < this->DistToLineThreshold))
          this->Angles[scanLine][index] = (leftLine.Direction.cross(rightLine.Direction)).norm();
      }

      // Here one side of the neighborhood is non flat.
      // Hence it is not worth to estimate the sharpness.
      // Only the gap will be considered here.
      // CHECK : looks strange to estimate depth gap without considering current point
      else if (!leftFlat && rightFlat)
      {
        distLeft = std::numeric_limits<float>::max();
        for (const auto& leftNeighborId: leftNeighbors)
        {
          const auto& leftNeighbor = scanLineCloud[leftNeighborId].getVector3fMap();
          distLeft = std::min(distLeft, rightLine.DistanceToPoint(leftNeighbor));
        }
      }
      else if (leftFlat && !rightFlat)
      {
        distRight = std::numeric_limits<float>::max();
        for (const auto& rightNeighborId: rightNeighbors)
        {
          const auto& rightNeighbor = scanLineCloud[rightNeighborId].getVector3fMap();
          distRight = std::min(distRight, leftLine.DistanceToPoint(rightNeighbor));
        }
      }

      // No neighborhood is flat.
      // We will compute saliency of the current keypoint from its far neighbors.
      else
      {
        // Compute salient point score
        const float sqCurrDepth = centralPoint.squaredNorm();
        bool hasLeftEncounteredDepthGap = false;
        bool hasRightEncounteredDepthGap = false;

        std::vector<int> farNeighbors;
        farNeighbors.reserve(2 * this->NeighborWidth);

        // The salient point score is the distance between the current point
        // and the points that have a depth gap with the current point
        // CHECK : consider only consecutive far neighbors, starting from the central point.
        for (const auto& leftNeighborId: leftNeighbors)
        {
          // Left neighborhood depth gap computation
          if (std::abs(scanLineCloud[leftNeighborId].getVector3fMap().squaredNorm() - sqCurrDepth) > std::pow(this->EdgeDepthGapThreshold, 2))
          {
            hasLeftEncounteredDepthGap = true;
            farNeighbors.emplace_back(leftNeighborId);
          }
          else if (hasLeftEncounteredDepthGap)
            break;
        }
        for (const auto& rightNeighborId: rightNeighbors)
        {
          // Right neigborhood depth gap computation
          if (std::abs(scanLineCloud[rightNeighborId].getVector3fMap().squaredNorm() - sqCurrDepth) > std::pow(this->EdgeDepthGapThreshold, 2))
          {
            hasRightEncounteredDepthGap = true;
            farNeighbors.emplace_back(rightNeighborId);
          }
          else if (hasRightEncounteredDepthGap)
            break;
        }

        // If there are enough neighbors with a big depth gap,
        // we propose to compute the saliency of the current point
        // as the distance between the line that roughly fits the far neighbors
        // with a depth gap and the current point
        if (farNeighbors.size() > static_cast<unsigned int>(this->NeighborWidth))
        {
          LineFitting farNeighborsLine;
          if(farNeighborsLine.FitPCAAndCheckConsistency(scanLineCloud, farNeighbors))
            this->Saliency[scanLine][index] = farNeighborsLine.DistanceToPoint(centralPoint);
        }
      }

      // Store max depth gap
      this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::AddKptsUsingCriterion (Keypoint k,
                                                             const std::vector<std::vector<float>>& values,
                                                             float threshold,
                                                             bool threshIsMax,
                                                             double weightBasis)
{
  // Loop over the scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanlineIdx = 0; scanlineIdx < static_cast<int>(this->NbLaserRings); ++scanlineIdx)
  {
    const int Npts = this->ScanLines[scanlineIdx]->size();

    // If the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
      continue;

    // If threshIsMax : ascending order (lowest first)
    // If threshIsMin : descending order (greatest first)
    std::vector<size_t> sortedValuesIndices = Utils::SortIdx(values[scanlineIdx], threshIsMax);

    for (const auto& index: sortedValuesIndices)
    {
      // If the point is invalid, continue
      if (!this->IsPointValid[scanlineIdx][index] || values[scanlineIdx][index] < 0)
        continue;

      // Check criterion threshold
      bool valueAboveThresh = values[scanlineIdx][index] > threshold;
      // If criterion is not respected, break loop as indices are sorted.
      if ((threshIsMax && valueAboveThresh) ||
          (!threshIsMax && !valueAboveThresh))
        break;
      if (this->Label[scanlineIdx][index][Keypoint::EDGE])
        continue;

      // The points with the lowest weight have priority for extraction
      float weight = threshIsMax? weightBasis + values[scanlineIdx][index] / threshold : weightBasis - values[scanlineIdx][index] / values[scanlineIdx][0];

      #pragma omp critical
      {
        // Indicate the type of the keypoint to debug and to exclude double edges
        this->Label[scanlineIdx][index].set(k);
        // Add keypoint
        this->Keypoints[k].AddPoint(this->ScanLines[scanlineIdx]->at(index), weight);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputePlanes()
{
  this->AddKptsUsingCriterion(Keypoint::PLANE, this->Angles, this->PlaneSinAngleThreshold);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeEdges()
{
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->DepthGap, this->EdgeDepthGapThreshold, false, 1);
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->Angles, this->EdgeSinAngleThreshold, false, 2);
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->Saliency, this->EdgeSaliencyThreshold, false, 3);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  this->AddKptsUsingCriterion(Keypoint::INTENSITY_EDGE, this->IntensityGap, this->EdgeIntensityGapThreshold, false);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeBlobs()
{
  for (unsigned int scanLine = 0; scanLine < this->NbLaserRings; ++scanLine)
  {
    for (unsigned int index = 0; index < this->ScanLines[scanLine]->size(); ++index)
    {
      if (this->IsPointValid[scanLine][index])
        this->Keypoints[Keypoint::BLOB].AddPoint(this->ScanLines[scanLine]->at(index));
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::EstimateAzimuthalResolution()
{
  // Compute horizontal angle values between successive points
  std::vector<float> angles;
  angles.reserve(this->Scan->size());
  for (const PointCloud::Ptr& scanLine : this->ScanLines)
  {
    for (unsigned int index = 1; index < scanLine->size(); ++index)
    {
      // Compute horizontal angle between two measurements
      // WARNING: to be correct, the points need to be in the LIDAR sensor
      // coordinates system, where the sensor is spinning around Z axis.
      Eigen::Map<const Eigen::Vector2f> p1(scanLine->at(index - 1).data);
      Eigen::Map<const Eigen::Vector2f> p2(scanLine->at(index).data);
      float angle = std::abs(std::acos(p1.dot(p2) / (p1.norm() * p2.norm())));

      // Keep only angles greater than 0 to avoid dual return issues
      if (angle > 1e-4)
        angles.push_back(angle);
    }
  }

  // A minimum number of angles is needed to get a trustable estimator
  if (angles.size() < 100)
  {
    PRINT_WARNING("Not enough points to estimate azimuthal resolution");
    return;
  }

  // Estimate azimuthal resolution from these angles
  std::sort(angles.begin(), angles.end());
  unsigned int maxInliersIdx = angles.size();
  float maxAngle = Utils::Deg2Rad(5.);
  float medianAngle = 0.;
  // Iterate until only angles between direct LiDAR beam neighbors remain.
  // The max resolution angle is decreased at each iteration.
  while (maxAngle > 1.8 * medianAngle)
  {
    maxInliersIdx = std::upper_bound(angles.begin(), angles.begin() + maxInliersIdx, maxAngle) - angles.begin();
    medianAngle = angles[maxInliersIdx / 2];
    maxAngle = std::min(medianAngle * 2., maxAngle / 1.8);
  }
  this->AzimuthalResolution = medianAngle;
  std::cout << "LiDAR's azimuthal resolution estimated to " << Utils::Rad2Deg(this->AzimuthalResolution) << "°" << std::endl;
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vector2d)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vector2d, int flag)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]][flag];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<float>> map;
  map["sin_angle"]      = get1DVector(this->Angles);
  map["saliency"]       = get1DVector(this->Saliency);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["intensity_edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::INTENSITY_EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  map["valid"]          = get1DVector(this->IsPointValid);
  return map;
}

} // end of LidarSlam namespace