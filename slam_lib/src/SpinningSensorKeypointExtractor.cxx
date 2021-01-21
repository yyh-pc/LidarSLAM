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

namespace LidarSlam
{

namespace
{
//-----------------------------------------------------------------------------
struct LineFitting
{
  //! Fitting using PCA
  bool FitPCA(const SpinningSensorKeypointExtractor::PointCloud& cloud,
              const std::vector<int>& indices);

  //! Fitting using very local line and check if this local line is consistent
  //! in a more global neighborhood
  bool FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                 const std::vector<int>& indices);

  //! Compute the squared distance of a point to the fitted line
  inline double SquaredDistanceToPoint(Eigen::Vector3d const& point) const;

  // Direction and position
  Eigen::Vector3d Direction;
  Eigen::Vector3d Position;

  //! Max distance allowed from the farest point to estimated line to be considered as real line
  double MaxDistance = 0.02;  // [m]

  //! Max angle allowed between consecutive segments in the neighborhood to be considered as line
  double MaxAngle = DEG2RAD(40.);  // [rad]
};

//-----------------------------------------------------------------------------
bool LineFitting::FitPCA(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                         const std::vector<int>& indices)
{
  // Compute PCA to determine best line approximation of the points distribution
  // and save points centroid in Position
  Eigen::Vector3d eigVals;
  Eigen::Matrix3d eigVecs;
  Utils::ComputeMeanAndPCA(cloud, indices, this->Position, eigVecs, eigVals);

  // Get Direction as main eigen vector
  this->Direction = eigVecs.col(2);

  // If a point of the neighborhood is too far from the fitted line,
  // we consider the neighborhood as non flat
  bool isLineFittingAccurate = true;
  const double squaredMaxDistance = this->MaxDistance * this->MaxDistance;
  for (const auto& pointId: indices)
  {
    Eigen::Vector3d point = cloud[pointId].getVector3fMap().cast<double>();
    if (this->SquaredDistanceToPoint(point) > squaredMaxDistance)
    {
      isLineFittingAccurate = false;
      break;
    }
  }
  return isLineFittingAccurate;
}

//-----------------------------------------------------------------------------
bool LineFitting::FitPCAAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                            const std::vector<int>& indices)
{
  const double maxSinAngle = std::sin(this->MaxAngle);
  bool isLineFittingAccurate = true;

  // First check if the neighborhood is approximately straight
  const Eigen::Vector3f U = (cloud[1].getVector3fMap() - cloud[0].getVector3fMap()).normalized();
  for (unsigned int i = 1; i < indices.size() - 1; i++)
  {
    const Eigen::Vector3f V = (cloud[indices[i + 1]].getVector3fMap() - cloud[indices[i]].getVector3fMap()).normalized();
    const double sinAngle = (U.cross(V)).norm();
    if (sinAngle > maxSinAngle)
    {
      isLineFittingAccurate = false;
      break;
    }
  }

  // Then fit with PCA (only if isLineFittingAccurate is true)
  return isLineFittingAccurate && this->FitPCA(cloud, indices);
}

//-----------------------------------------------------------------------------
inline double LineFitting::SquaredDistanceToPoint(Eigen::Vector3d const& point) const
{
  return ((point - this->Position).cross(this->Direction)).squaredNorm();
}
} // end of anonymous namespace

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->pclCurrentFrame = pc;

  // Split whole pointcloud into separate laser ring clouds
  this->ConvertAndSortScanLines();

  // Initialize the features vectors and keypoints
  this->PrepareDataForNextFrame();

  // Invalid points with bad criteria
  this->InvalidPointWithBadCriteria();

  // Compute keypoints scores
  this->ComputeCurvature();

  // Labelize keypoints
  this->SetKeyPointsLabels();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Clear previous scan lines
  for (auto& scanLineCloud: this->pclCurrentFrameByScan)
  {
    // Use clear() if pointcloud already exists to avoid re-allocating memory.
    // No worry as pclCurrentFrameByScan is never shared with outer scope.
    if (scanLineCloud)
      scanLineCloud->clear();
    else
      scanLineCloud.reset(new PointCloud);
  }

  // Separate pointcloud into different scan lines
  for (const Point& point: *this->pclCurrentFrame)
  {
    // Ensure that there are enough available scan lines
    while (point.laser_id >= this->pclCurrentFrameByScan.size())
      this->pclCurrentFrameByScan.emplace_back(new PointCloud);

    // Add the current point to its corresponding laser scan
    this->pclCurrentFrameByScan[point.laser_id]->push_back(point);
  }

  // Save the number of lasers
  this->NLasers = this->pclCurrentFrameByScan.size();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Do not use clear(), otherwise weird things could happen if outer program
  // uses these pointers
  this->EdgesPoints.reset(new PointCloud);
  this->PlanarsPoints.reset(new PointCloud);
  this->BlobsPoints.reset(new PointCloud);
  Utils::CopyPointCloudMetadata(*this->pclCurrentFrame, *this->EdgesPoints);
  Utils::CopyPointCloudMetadata(*this->pclCurrentFrame, *this->PlanarsPoints);
  Utils::CopyPointCloudMetadata(*this->pclCurrentFrame, *this->BlobsPoints);

  // Initialize the features vectors with the correct length
  this->Angles.resize(this->NLasers);
  this->Saliency.resize(this->NLasers);
  this->DepthGap.resize(this->NLasers);
  this->IntensityGap.resize(this->NLasers);
  this->IsPointValid.resize(this->NLasers);
  this->Label.resize(this->NLasers);

  // Initialize the scan lines features vectors with the correct length
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    size_t nbPoint = this->pclCurrentFrameByScan[scanLine]->size();
    this->IsPointValid[scanLine].assign(nbPoint, KeypointFlags().set());  // set all flags to 1
    this->Label[scanLine].assign(nbPoint, KeypointFlags().reset());  // set all flags to 0
    this->Angles[scanLine].assign(nbPoint, 0.);
    this->Saliency[scanLine].assign(nbPoint, 0.);
    this->DepthGap[scanLine].assign(nbPoint, 0.);
    this->IntensityGap[scanLine].assign(nbPoint, 0.);
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidPointWithBadCriteria()
{
  const double expectedCoeff = 10.;

  // loop over scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) firstprivate(expectedCoeff)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->pclCurrentFrameByScan[scanLine]);
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      for (int index = 0; index < Npts; ++index)
        this->IsPointValid[scanLine][index].reset();
      continue;
    }

    // invalidate first and last points
    // CHECK why ?
    for (int index = 0; index <= this->NeighborWidth; ++index)
      this->IsPointValid[scanLine][index].reset();
    for (int index = Npts - 1 - this->NeighborWidth - 1; index < Npts; ++index)
      this->IsPointValid[scanLine][index].reset();

    // loop over points into the scan line
    for (int index = this->NeighborWidth; index < Npts - this->NeighborWidth - 1; ++index)
    {
      // double precision is useless as PCL points coordinates are internally stored as float
      const Eigen::Vector3f& previousPoint = scanLineCloud[index - 1].getVector3fMap();
      const Eigen::Vector3f& currentPoint  = scanLineCloud[index    ].getVector3fMap();
      const Eigen::Vector3f& nextPoint     = scanLineCloud[index + 1].getVector3fMap();

      const double L = currentPoint.norm();
      const double Ln = nextPoint.norm();
      const double dLn = (nextPoint - currentPoint).norm();
      const double dLp = (currentPoint - previousPoint).norm();

      // The expected length between two firings of the same laser is the
      // distance along the same circular arc. It depends only on radius value
      // and the angular resolution of the sensor.
      const double expectedLength = this->AngleResolution * L;

      // Invalid occluded points due to depth gap.
      // If the distance between two successive points is bigger than the
      // expected length, it means that there is a depth gap.
      if (dLn > expectedCoeff * expectedLength)
      {
        // We must invalidate the points which belong to the occluded area (farthest).
        // If current point is the closest, invalid next part, starting from next point
        if (L < Ln)
        {
          this->IsPointValid[scanLine][index + 1].reset();
          for (int i = index + 2; i <= index + this->NeighborWidth; ++i)
          {
            const Eigen::Vector3f& Y  = scanLineCloud[i - 1].getVector3fMap();
            const Eigen::Vector3f& Yn = scanLineCloud[i].getVector3fMap();

            // If there is a gap in the neighborhood, we do not invalidate the rest of it.
            if ((Yn - Y).norm() > expectedCoeff * expectedLength)
            {
              break;
            }
            // Otherwise, do not use next point
            this->IsPointValid[scanLine][i].reset();
          }
        }
        // If current point is the farest, invalid previous part, starting from current point
        else
        {
          this->IsPointValid[scanLine][index].reset();
          for (int i = index - this->NeighborWidth; i < index; ++i)
          {
            const Eigen::Vector3f& Yp = scanLineCloud[i].getVector3fMap();
            const Eigen::Vector3f&  Y = scanLineCloud[i + 1].getVector3fMap();

            // If there is a gap in the neighborhood, we do not invalidate the rest of it.
            if ((Y - Yp).norm() > expectedCoeff * expectedLength)
            {
              break;
            }
            // Otherwise, do not use previous point
            this->IsPointValid[scanLine][i].reset();
          }
        }
      }

      // Invalid points which are too close from the sensor
      if (L < this->MinDistanceToSensor)
      {
        this->IsPointValid[scanLine][index].reset();
      }

      // Invalid points which are on a planar surface nearly parallel to the
      // laser beam direction
      else if ((dLp > 0.25 * expectedCoeff * expectedLength) &&
               (dLn > 0.25 * expectedCoeff * expectedLength))
      {
        this->IsPointValid[scanLine][index].reset();
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  const double sqDistToLineThreshold = this->DistToLineThreshold * this->DistToLineThreshold;  // [m²]
  const double sqDepthDistCoeff = 0.25;
  const double minDepthGapDist = 1.5;  // [m]

  // loop over scans lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) \
          firstprivate(sqDistToLineThreshold, sqDepthDistCoeff, minDepthGapDist)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *(this->pclCurrentFrameByScan[scanLine]);
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      continue;
    }

    // loop over points in the current scan line
    // TODO : deal with spherical case : index=0 is neighbor of index=Npts-1
    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // Skip curvature computation for invalid points
      if (this->IsPointValid[scanLine][index].none())
      {
        continue;
      }

      // central point
      const Point& currentPoint = scanLineCloud[index];
      const Eigen::Vector3d centralPoint = currentPoint.getVector3fMap().cast<double>();

      // compute intensity gap
      // CHECK : do not use currentPoint.intensity?
      const Point& previousPoint = scanLineCloud[index - 1];
      const Point& nextPoint = scanLineCloud[index + 1];
      this->IntensityGap[scanLine][index] = std::abs(nextPoint.intensity - previousPoint.intensity);

      // We will compute the line that fits the neighbors located before the current point.
      // We will do the same for the neighbors located after the current point.
      // We will then compute the angle between these two lines as an approximation
      // of the "sharpness" of the current point.
      std::vector<int> leftNeighbors(this->NeighborWidth);
      std::vector<int> rightNeighbors(this->NeighborWidth);
      LineFitting leftLine, rightLine;

      // Fill left and right neighborhoods, from central point to sides.
      // /!\ The way the neighbors are added to the vectors matters,
      // especially when computing the saliency
      for (int j = index - 1; j >= index - this->NeighborWidth; --j)
        leftNeighbors[index - 1 - j] = j;
      for (int j = index + 1; j <= index + this->NeighborWidth; ++j)
        rightNeighbors[j - index - 1] = j;

      // Fit line on the left and right neighborhoods and
      // Indicate if they are flat or not
      const bool leftFlat = leftLine.FitPCAAndCheckConsistency(scanLineCloud, leftNeighbors);
      const bool rightFlat = rightLine.FitPCAAndCheckConsistency(scanLineCloud, rightNeighbors);

      // Measurement of the depth gap
      double distLeft = 0., distRight = 0.;

      // If both neighborhoods are flat, we can compute the angle between them
      // as an approximation of the sharpness of the current point
      if (leftFlat && rightFlat)
      {
        // We check that the current point is not too far from its
        // neighborhood lines. This is because we don't want a point
        // to be considered as an angle point if it is due to gap
        distLeft = leftLine.SquaredDistanceToPoint(centralPoint);
        distRight = rightLine.SquaredDistanceToPoint(centralPoint);

        // If current point is not too far from estimated lines,
        // save the sin of angle between these two lines
        if ((distLeft < sqDistToLineThreshold) && (distRight < sqDistToLineThreshold))
          this->Angles[scanLine][index] = (leftLine.Direction.cross(rightLine.Direction)).norm();
      }

      // Here one side of the neighborhood is non flat.
      // Hence it is not worth to estimate the sharpness.
      // Only the gap will be considered here.
      // CHECK : looks strange to estimate depth gap without considering current point
      else if (!leftFlat && rightFlat)
      {
        distLeft = std::numeric_limits<double>::max();
        for (const auto& leftNeighborId: leftNeighbors)
        {
          const auto& leftNeighbor = scanLineCloud[leftNeighborId].getVector3fMap();
          distLeft = std::min(distLeft, rightLine.SquaredDistanceToPoint(leftNeighbor));
        }
        distLeft *= sqDepthDistCoeff;
      }
      else if (leftFlat && !rightFlat)
      {
        distRight = std::numeric_limits<double>::max();
        for (const auto& rightNeighborId: rightNeighbors)
        {
          const auto& rightNeighbor = scanLineCloud[rightNeighborId].getVector3fMap();
          distRight = std::min(distRight, leftLine.SquaredDistanceToPoint(rightNeighbor));
        }
        distRight *= sqDepthDistCoeff;
      }

      // No neighborhood is flat.
      // We will compute saliency of the current keypoint from its far neighbors.
      else
      {
        // Compute salient point score
        const double currDepth = centralPoint.norm();
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
          if (std::abs(scanLineCloud[leftNeighborId].getVector3fMap().norm() - currDepth) > minDepthGapDist)
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
          if (std::abs(scanLineCloud[rightNeighborId].getVector3fMap().norm() - currDepth) > minDepthGapDist)
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
          farNeighborsLine.FitPCA(scanLineCloud, farNeighbors);
          this->Saliency[scanLine][index] = farNeighborsLine.SquaredDistanceToPoint(centralPoint);
        }
      }

      // Store max depth gap
      this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetKeyPointsLabels()
{
  const double squaredEdgeSaliencythreshold = this->EdgeSaliencyThreshold * this->EdgeSaliencyThreshold;
  const double squaredEdgeDepthGapThreshold = this->EdgeDepthGapThreshold * this->EdgeDepthGapThreshold;

  // loop over the scan lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided) \
          firstprivate(squaredEdgeSaliencythreshold, squaredEdgeDepthGapThreshold)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NLasers); ++scanLine)
  {
    const int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(Npts))
    {
      continue;
    }

    // Sort the curvature score in a decreasing order
    std::vector<size_t> sortedDepthGapIdx  = Utils::SortIdx(this->DepthGap[scanLine]);
    std::vector<size_t> sortedAnglesIdx    = Utils::SortIdx(this->Angles[scanLine]);
    std::vector<size_t> sortedSaliencyIdx  = Utils::SortIdx(this->Saliency[scanLine]);
    std::vector<size_t> sortedIntensityGap = Utils::SortIdx(this->IntensityGap[scanLine]);

    // Add edge according to criterion
    auto addEdgesUsingCriterion = [this, scanLine, Npts](const std::vector<size_t>& sortedValuesIdx,
                                                         const std::vector<std::vector<double>>& values,
                                                         double threshold,
                                                         int invalidNeighborhoodSize)
    {
      for (const auto& index: sortedValuesIdx)
      {
        // Check criterion threshold
        // If criterion is not respected, break loop as indices are sorted in decreasing order.
        if (values[scanLine][index] < threshold)
          break;

        // If the point is invalid as edge, continue
        if (!this->IsPointValid[scanLine][index][Keypoint::EDGE])
          continue;

        // Else indicate that the point is an edge
        this->Label[scanLine][index].set(Keypoint::EDGE);

        // Invalid its neighbors
        const int indexBegin = std::max(0,        static_cast<int>(index - invalidNeighborhoodSize));
        const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + invalidNeighborhoodSize));
        for (int j = indexBegin; j <= indexEnd; ++j)
          this->IsPointValid[scanLine][j].reset(Keypoint::EDGE);
      }
    };

    // Edges using depth gap
    addEdgesUsingCriterion(sortedDepthGapIdx, this->DepthGap, squaredEdgeDepthGapThreshold, this->NeighborWidth - 1);
    // Edges using angles
    addEdgesUsingCriterion(sortedAnglesIdx, this->Angles, this->EdgeSinAngleThreshold, this->NeighborWidth);
    // Edges using saliency
    addEdgesUsingCriterion(sortedSaliencyIdx, this->Saliency, squaredEdgeSaliencythreshold, this->NeighborWidth - 1);
    // Edges using intensity
    addEdgesUsingCriterion(sortedIntensityGap, this->IntensityGap, this->EdgeIntensityGapThreshold, 1);

    // Planes (using angles)
    for (int k = Npts - 1; k >= 0; --k)
    {
      size_t index = sortedAnglesIdx[k];
      const double sinAngle = this->Angles[scanLine][index];

      // thresh
      if (sinAngle > this->PlaneSinAngleThreshold)
        break;

      // if the point is invalid as plane, continue
      if (!this->IsPointValid[scanLine][index][Keypoint::PLANE])
        continue;

      // else indicate that the point is a planar one
      this->Label[scanLine][index].set(Keypoint::PLANE);

      // Invalid its neighbors so that we don't have too
      // many planar keypoints in the same region. This is
      // required because of the k-nearest search + plane
      // approximation realized in the odometry part. Indeed,
      // if all the planar points are on the same scan line the
      // problem is degenerated since all the points are distributed
      // on a line.
      const int indexBegin = std::max(0,        static_cast<int>(index - 4));
      const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + 4));
      for (int j = indexBegin; j <= indexEnd; ++j)
        this->IsPointValid[scanLine][j].reset(Keypoint::PLANE);
    }

    // Blobs Points
    // CHECK : why using only 1 point over 3?
    // TODO : disable blobs if not required
    for (int index = 0; index < Npts; index += 3)
    {
      if (this->IsPointValid[scanLine][index][Keypoint::BLOB])
        this->Label[scanLine][index].set(Keypoint::BLOB);
    }
  }

  auto addKeypoints = [this](Keypoint type, PointCloud::Ptr& keypoints)
  {
    for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
    {
      const PointCloud& scanLineCloud = *(this->pclCurrentFrameByScan[scanLine]);
      for (unsigned int index = 0; index < scanLineCloud.size(); ++index)
      {
        if (this->Label[scanLine][index][type])
        {
          this->IsPointValid[scanLine][index].set(type);
          keypoints->push_back(scanLineCloud[index]);
        }
      }
    }
  };
  addKeypoints(Keypoint::EDGE, this->EdgesPoints);
  addKeypoints(Keypoint::PLANE, this->PlanarsPoints);
  addKeypoints(Keypoint::BLOB, this->BlobsPoints);
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<double>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vector2d)
  {
    std::vector<double> v(this->pclCurrentFrame->size());
    std::vector<int> indexByScanLine(this->NLasers, 0);
    for (unsigned int i = 0; i < this->pclCurrentFrame->size(); i++)
    {
      const auto& laserId = this->pclCurrentFrame->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vector2d, int flag)
  {
    std::vector<double> v(this->pclCurrentFrame->size());
    std::vector<int> indexByScanLine(this->NLasers, 0);
    for (unsigned int i = 0; i < this->pclCurrentFrame->size(); i++)
    {
      const auto& laserId = this->pclCurrentFrame->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]][flag];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<double>> map;
  map["sin_angle"]      = get1DVector(this->Angles);
  map["saliency"]       = get1DVector(this->Saliency);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  map["edge_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::EDGE);
  map["plane_validity"] = get1DVectorFromFlag(this->IsPointValid, Keypoint::PLANE);
  map["blob_validity"]  = get1DVectorFromFlag(this->IsPointValid, Keypoint::BLOB);
  return map;
}

} // end of LidarSlam namespace