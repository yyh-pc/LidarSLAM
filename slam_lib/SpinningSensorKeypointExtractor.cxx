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
#include "SpinningSensorKeypointExtractor.h"

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace
{
//-----------------------------------------------------------------------------
template <typename T>
std::vector<size_t> sortIdx(const std::vector<T> &v)
{
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

  return idx;
}

//-----------------------------------------------------------------------------
struct LineFitting
{
  //! Fitting using PCA
  bool FitPCA(std::vector<Eigen::Vector3d> const& points);

  //! Fitting using very local line and check if this local line is consistent
  //! in a more global neighborhood
  bool FitPCAAndCheckConsistency(std::vector<Eigen::Vector3d> const& points);

  //! Poor but fast fitting using extremities of the distribution
  void FitFast(std::vector<Eigen::Vector3d> const& points);

  // Direction and position
  Eigen::Vector3d Direction;
  Eigen::Vector3d Position;
  Eigen::Matrix3d SemiDist;

  // Constants
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  const double MaxDistance = 0.02;  // [m]
  const double MaxSinAngle = 0.65;  // ~sin(40°)
};

//-----------------------------------------------------------------------------
bool LineFitting::FitPCA(std::vector<Eigen::Vector3d> const& points)
{
  // Compute PCA to determine best line approximation of the points distribution
  Eigen::MatrixXd data(points.size(), 3);
  for (unsigned int k = 0; k < points.size(); k++)
  {
    data.row(k) = points[k];
  }
  // Position
  this->Position = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - this->Position.transpose();
  Eigen::Matrix3d varianceCovariance = centered.transpose() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(varianceCovariance);

  // Direction
  this->Direction = eig.eigenvectors().col(2).normalized();

  // Semi distance matrix
  // (polar form associated to a bilinear symmetric positive semi-definite matrix)
  this->SemiDist = (this->I3 - this->Direction * this->Direction.transpose());

  // If a point of the neighborhood is too far from the fitting line,
  // we consider the neighborhood as non flat
  bool isLineFittingAccurate = true;
  const double squaredMaxDistance = this->MaxDistance * this->MaxDistance;
  for (const Eigen::Vector3d& point: points)
  {
    const Eigen::Vector3d distToMean = point - this->Position;
    const double d = distToMean.transpose() * this->SemiDist * distToMean;
    if (d > squaredMaxDistance)
    {
      isLineFittingAccurate = false;
      break;
    }
  }
  return isLineFittingAccurate;
}

//-----------------------------------------------------------------------------
bool LineFitting::FitPCAAndCheckConsistency(std::vector<Eigen::Vector3d> const& points)
{
  bool isLineFittingAccurate = true;

  // first check if the neighborhood is straight
  const Eigen::Vector3d U = (points[1] - points[0]).normalized();
  for (unsigned int index = 1; index < points.size() - 1; index++)
  {
    const Eigen::Vector3d V = (points[index + 1] - points[index]).normalized();
    const double sinAngle = (U.cross(V)).norm();
    if (sinAngle > this->MaxSinAngle)
    {
      isLineFittingAccurate = false;
      break;
    }
  }

  // Then fit with PCA (only if isLineFittingAccurate is true)
  return isLineFittingAccurate && this->FitPCA(points);
}

//-----------------------------------------------------------------------------
void LineFitting::FitFast(std::vector<Eigen::Vector3d> const& points)
{
  // Take the two extrem points of the neighborhood
  // i.e the farest and the closest to the current point
  const Eigen::Vector3d& U = points.front();
  const Eigen::Vector3d& V = points.back();

  // direction
  this->Direction = (V - U).normalized();

  // position
  this->Position = U;

  // Semi distance matrix
  // (polar form associated to a bilinear symetric positive semi-definite matrix)
  this->SemiDist = (this->I3 - this->Direction * this->Direction.transpose());
  this->SemiDist = this->SemiDist.transpose() * this->SemiDist;
}
}


//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Reset the pcl format pointcloud to store the new frame
  this->pclCurrentFrameByScan.resize(this->NLasers);
  for (auto& scanLineCloud: this->pclCurrentFrameByScan)
  {
    scanLineCloud.reset(new PointCloud);
  }

  // TODO : clear pointclouds instead of resetting shared_ptr
  this->EdgesPoints.reset(new PointCloud);
  this->PlanarsPoints.reset(new PointCloud);
  this->BlobsPoints.reset(new PointCloud);
  this->EdgesPoints->header = this->pclCurrentFrame->header;
  this->PlanarsPoints->header = this->pclCurrentFrame->header;
  this->BlobsPoints->header = this->pclCurrentFrame->header;

  this->Angles.clear();
  this->Angles.resize(this->NLasers);
  this->SaillantPoint.clear();
  this->SaillantPoint.resize(this->NLasers);
  this->DepthGap.clear();
  this->DepthGap.resize(this->NLasers);
  this->IntensityGap.clear();
  this->IntensityGap.resize(this->NLasers);
  this->IsPointValid.clear();
  this->IsPointValid.resize(this->NLasers);
  this->Label.clear();
  this->Label.resize(this->NLasers);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Get frame duration
  double frameStartTime, frameEndTime;
  for (Point const& point: *this->pclCurrentFrame)
  {
    frameStartTime = std::min(frameStartTime, point.time);
    frameEndTime = std::max(frameEndTime, point.time);
  }
  const double frameDuration = frameEndTime - frameStartTime;

  // Separate pointcloud into different scan lines
  // Modify the point so that:
  // - laserId is corrected with the laserIdMapping
  // - time become a relative advancement time (between 0 and 1)
  for (Point const& oldPoint: *this->pclCurrentFrame)
  {
    int id = this->LaserIdMapping[oldPoint.laserId];
    Point newPoint(oldPoint);
    newPoint.laserId = id;
    newPoint.time = (oldPoint.time - frameStartTime) / frameDuration;  // CHECK unused

    // add the current point to its corresponding laser scan
    this->pclCurrentFrameByScan[id]->push_back(newPoint);
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc,
                                                       const std::vector<size_t>& laserIdMapping)
{
  if (this->LaserIdMapping.empty())
  {
    this->NLasers = laserIdMapping.size();
    this->LaserIdMapping = laserIdMapping;
  }
  this->pclCurrentFrame = pc;
  this->PrepareDataForNextFrame();
  this->ConvertAndSortScanLines();
  // Initialize the vectors with the correct length
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    size_t nbPoint = this->pclCurrentFrameByScan[scanLine]->size();
    this->IsPointValid[scanLine].resize(nbPoint, 1);
    this->Label[scanLine].resize(nbPoint, 0);
    this->Angles[scanLine].resize(nbPoint, 0);
    this->SaillantPoint[scanLine].resize(nbPoint, 0);
    this->DepthGap[scanLine].resize(nbPoint, 0);
    this->IntensityGap[scanLine].resize(nbPoint, 0);
  }

  // Invalid points with bad criteria
  this->InvalidPointWithBadCriteria();

  // compute keypoints scores
  this->ComputeCurvature();

  // labelize keypoints
  this->SetKeyPointsLabels();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  const double squaredDistToLineThreshold = this->DistToLineThreshold * this->DistToLineThreshold;  // [m²]
  const double squaredDepthDistCoeff = 0.25;
  const double minDepthGapDist = 1.5;  // [m]

  // loop over scans lines
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    // We will compute the line that fits the neighbors located before the current one.
    // We will do the same for the neighbors located after the current point.
    // We will then compute the angle between these two lines as an approximation
    // of the "sharpness" of the current point.
    // TODO :is it really necessary to init these objects before loop ?
    std::vector<Eigen::Vector3d> leftNeighbors(this->NeighborWidth);
    std::vector<Eigen::Vector3d> rightNeighbors(this->NeighborWidth);
    std::vector<Eigen::Vector3d> farNeighbors;
    farNeighbors.reserve(2 * this->NeighborWidth);

    LineFitting leftLine, rightLine, farNeighborsLine;

    const int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (Npts < 2 * this->NeighborWidth + 1)
    {
      continue;
    }

    // loop over points in the current scan line
    // TODO : deal with spherical case : index=0 is neighbor of index=Npts-1
    for (int index = this->NeighborWidth; (index + this->NeighborWidth) < Npts; ++index)
    {
      // Skip curvature computation for invalid points
      if (this->IsPointValid[scanLine][index] == 0)
      {
        continue;
      }

      // central point
      const Point& currentPoint = this->pclCurrentFrameByScan[scanLine]->points[index];
      const Eigen::Vector3d centralPoint(currentPoint.x, currentPoint.y, currentPoint.z);

      // compute intensity gap
      // CHECK : do not use currentPoint.intensity?
      const Point& previousPoint = this->pclCurrentFrameByScan[scanLine]->points[index - 1];
      const Point& nextPoint = this->pclCurrentFrameByScan[scanLine]->points[index + 1];
      this->IntensityGap[scanLine][index] = std::abs(nextPoint.intensity - previousPoint.intensity);

      // Fill left and right neighborhoods
      // /!\ The way the neighbors are added to the vectors matters,
      // especially when computing the saillancy
      for (int j = index - this->NeighborWidth; j < index; ++j)
      {
        const Point& point = this->pclCurrentFrameByScan[scanLine]->points[j];
        leftNeighbors[j - index + this->NeighborWidth] << point.x, point.y, point.z;
      }
      for (int j = index + 1; j <= index + this->NeighborWidth; ++j)
      {
        const Point& point = this->pclCurrentFrameByScan[scanLine]->points[j];
        rightNeighbors[j - index - 1] << point.x, point.y, point.z;
      }

      // Fit line on the neighborhood and
      // Indicate if the left and right side neighborhoods of the current point are flat or not
      // TODO add current point to neighborhood ?
      const bool leftFlat = leftLine.FitPCAAndCheckConsistency(leftNeighbors);
      const bool rightFlat = rightLine.FitPCAAndCheckConsistency(rightNeighbors);

      // Measurement of the depth gap
      double distLeft = 0., distRight = 0.;

      // If both neighborhoods are flat, we can compute the angle between them
      // as an approximation of the sharpness of the current point
      if (rightFlat && leftFlat)
      {
        // We check that the current point is not too far from its
        // neighborhood lines. This is because we don't want a point
        // to be considered as a angles point if it is due to gap
        const Eigen::Vector3d distToLeft = centralPoint - leftLine.Position,
                              distToRight = centralPoint - rightLine.Position;
        distLeft = distToLeft.transpose() * leftLine.SemiDist * distToLeft;
        distRight = distToRight.transpose() * rightLine.SemiDist * distToRight;

        // If current point is not too far from estimated lines,
        // save the sin of angle between these two lines
        if ((distLeft < squaredDistToLineThreshold) && (distRight < squaredDistToLineThreshold))
          this->Angles[scanLine][index] = std::abs((leftLine.Direction.cross(rightLine.Direction)).norm());
      }

      // Here one side of the neighborhood is non flat.
      // Hence it is not worth to estimate the sharpness.
      // Only the gap will be considered here.
      else if (rightFlat && !leftFlat)
      {
        distLeft = std::numeric_limits<double>::max();
        for (const Eigen::Vector3d& leftNeighbor: leftNeighbors)
        {
          const Eigen::Vector3d leftNeighborToRightLine = leftNeighbor - rightLine.Position;
          distLeft = std::min(distLeft, (leftNeighborToRightLine.transpose() * rightLine.SemiDist * leftNeighborToRightLine)(0));
        }
        distLeft *= squaredDepthDistCoeff;
      }
      else if (!rightFlat && leftFlat)
      {
        distRight = std::numeric_limits<double>::max();
        for (const Eigen::Vector3d& rightNeighbor: rightNeighbors)
        {
          const Eigen::Vector3d rightNeighborToLeftLine = rightNeighbor - leftLine.Position;
          distRight = std::min(distRight, (rightNeighborToLeftLine.transpose() * leftLine.SemiDist * rightNeighborToLeftLine)(0));
        }
        distRight *= squaredDepthDistCoeff;
      }

      // No neighborhood is flat.
      // We will try to compute saillancy of the current keypoint
      // from its far neighbors.
      else
      {
        // Compute saillant point score
        const double currDepth = centralPoint.norm();
        bool hasLeftEncounteredDepthGap = false;
        bool hasRightEncounteredDepthGap = false;
        farNeighbors.clear();

        // The saillant point score is the distance between the current point
        // and the points that have a depth gap with the current point
        // CHECK : consider only consecutive far neighbors, starting from the central point.
        for (int reversedNeighIndex = leftNeighbors.size() - 1; reversedNeighIndex >= 0; --reversedNeighIndex)
        {
          // Left neighborhood depth gap computation
          const Eigen::Vector3d& leftNeighbor = leftNeighbors[reversedNeighIndex];
          if (std::abs(leftNeighbor.norm() - currDepth) > minDepthGapDist)
          {
            hasLeftEncounteredDepthGap = true;
            farNeighbors.emplace_back(leftNeighbor);
          }
          else if (hasLeftEncounteredDepthGap)
            break;
        }
        for (const Eigen::Vector3d& rightNeighbor: rightNeighbors)
        {
          // Right neigborhood depth gap computation
          if (std::abs(rightNeighbor.norm() - currDepth) > minDepthGapDist)
          {
            hasRightEncounteredDepthGap = true;
            farNeighbors.emplace_back(rightNeighbor);
          }
          else if (hasRightEncounteredDepthGap)
            break;
        }
        // // CHECK : consider all far neighbors.
        // for (int reversedNeighIndex = leftNeighbors.size() - 1; reversedNeighIndex >= 0; --reversedNeighIndex)
        // {
        //   // Left neighborhood depth gap computation
        //   const Eigen::Vector3d& leftNeighbor = leftNeighbors[reversedNeighIndex];
        //   if (std::abs(leftNeighbor.norm() - currDepth) > minDepthGapDist)
        //     farNeighbors.emplace_back(leftNeighbor);
        // }
        // for (const Eigen::Vector3d& rightNeighbor: rightNeighbors)
        // {
        //   // Right neigborhood depth gap computation
        //   if (std::abs(rightNeighbor.norm() - currDepth) > minDepthGapDist)
        //     farNeighbors.emplace_back(rightNeighbor);
        // }

        // If there are enough neighbors with a big depth gap,
        // we propose to compute the saillancy of the current point
        // as the distance between the line that fits the far neighbors
        // with a depth gap and the current point
        if (farNeighbors.size() > this->NeighborWidth)
        {
          farNeighborsLine.FitPCA(farNeighbors);
          const Eigen::Vector3d currentPointToFarLineDist = centralPoint - farNeighborsLine.Position;
          this->SaillantPoint[scanLine][index] = currentPointToFarLineDist.transpose() * farNeighborsLine.SemiDist * currentPointToFarLineDist;
        }
      }

      // Store max depth gap
      this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::InvalidPointWithBadCriteria()
{
  // loop over scan lines
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    const int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (Npts < 3 * this->NeighborWidth)
    {
      continue;
    }
    // invalidate first and last points
    for (int index = 0; index <= this->NeighborWidth; ++index)
    {
      this->IsPointValid[scanLine][index] = 0;
    }
    for (int index = Npts - 1 - this->NeighborWidth - 1; index < Npts; ++index)
    {
      this->IsPointValid[scanLine][index] = 0;
    }

    // loop over points into the scan line
    for (int index = this->NeighborWidth; index <  Npts - this->NeighborWidth - 1; ++index)
    {
      const Point& previousPoint = this->pclCurrentFrameByScan[scanLine]->points[index - 1],
                   currentPoint = this->pclCurrentFrameByScan[scanLine]->points[index],
                   nextPoint = this->pclCurrentFrameByScan[scanLine]->points[index + 1];

      // double precision is useless as PCL points coordinates are internally stored as float
      const Eigen::Vector3f& X = currentPoint.getVector3fMap(),
                             Xn = nextPoint.getVector3fMap(),
                             Xp = previousPoint.getVector3fMap();

      const double L = X.norm();
      const double Ln = Xn.norm();
      const double dLn = (Xn - X).norm();
      const double dLp = (X - Xp).norm();

      // The expected length between two firings of the same laser depends on
      // the distance and the angular resolution of the sensor.
      const double expectedLength = 2.0 * std::tan(this->AngleResolution / 2.0) * L;
      const double ratioExpectedLength = 10.0;

      // if the length between the two firing
      // is more than n-th the expected length
      // it means that there is a gap. We now must
      // determine if the gap is due to the geometry of
      // the scene or if the gap is due to an occluded area
      if (dLn > ratioExpectedLength * expectedLength)
      {
        // Project the next point onto the
        // sphere of center 0 and radius =
        // norm of the current point. If the
        // gap has disappeared it means that
        // the gap was due to an occlusion
        // Eigen::Vector3d Xproj = L / Ln * Xn;
        // Eigen::Vector3d dXproj = Xproj - X;

        // it is a depth gap, invalidate the part which belongs
        // to the occluded area (farest)
        // invalid next part
        if (L < Ln)
        {
          for (int i = index + 2; i <= index + this->NeighborWidth; ++i)
          {
            const Point& previous = this->pclCurrentFrameByScan[scanLine]->points[i - 1],
                         current = this->pclCurrentFrameByScan[scanLine]->points[i];
            const Eigen::Vector3f& Yp = previous.getVector3fMap(),
                                   Y = current.getVector3fMap();
            const Eigen::Vector3f dY = Y - Yp;

            // If there is a gap in the neihborhood, we do not invalidate the rest of neihborhood.
            if (dY.norm() > ratioExpectedLength * expectedLength)
            {
              break;
            }
            this->IsPointValid[scanLine][i] = 0;
          }
          this->IsPointValid[scanLine][index + 1] = 0;
        }
        // invalid previous part
        else
        {
          for (int i = index - this->NeighborWidth; i < index; ++i)
          {
            const Point& current = this->pclCurrentFrameByScan[scanLine]->points[i],
                         next = this->pclCurrentFrameByScan[scanLine]->points[i + 1];
            const Eigen::Vector3f& Y = current.getVector3fMap(),
                                   Yn = next.getVector3fMap();
            const Eigen::Vector3f dY = Yn - Y;

            // if there is a gap in the neihborhood
            // we do not invalidate the rest of neihborhood
            if (dY.norm() > ratioExpectedLength * expectedLength)
            {
              break;
            }
            this->IsPointValid[scanLine][i] = 0;
          }
          this->IsPointValid[scanLine][index] = 0;
        }
      }

      // Invalid points which are too close from the sensor
      if (L < this->MinDistanceToSensor)
      {
        this->IsPointValid[scanLine][index] = 0;
      }

      // Invalid points which are on a planar surface nearly parallel to the
      // laser beam direction
      else if ((dLp > 1 / 4.0 * ratioExpectedLength * expectedLength) &&
               (dLn > 1 / 4.0 * ratioExpectedLength * expectedLength))
      {
        this->IsPointValid[scanLine][index] = 0;
      }
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetKeyPointsLabels()
{
  this->EdgesIndex.clear();
  this->PlanarIndex.clear();
  this->BlobIndex.clear();
  const double squaredEdgeDepthGapThreshold = this->EdgeDepthGapThreshold * this->EdgeDepthGapThreshold;

  // loop over the scan lines
  for (unsigned int scanLine = 0; scanLine < this->NLasers; ++scanLine)
  {
    const int Npts = this->pclCurrentFrameByScan[scanLine]->size();

    // if the line is almost empty, skip it
    if (Npts < 3 * this->NeighborWidth)
    {
      continue;
    }

    // We split the validity of points between the edges
    // keypoints and planar keypoints. This allows to take
    // some points as planar keypoints even if they are close
    // to an edge keypoint.
    std::vector<double> IsPointValidForPlanar = this->IsPointValid[scanLine];

    // Sort the curvature score in a decreasing order
    std::vector<size_t> sortedDepthGapIdx = sortIdx<double>(this->DepthGap[scanLine]);
    std::vector<size_t> sortedAnglesIdx = sortIdx<double>(this->Angles[scanLine]);
    std::vector<size_t> sortedSaillancyIdx = sortIdx<double>(this->SaillantPoint[scanLine]);
    std::vector<size_t> sortedIntensityGap = sortIdx<double>(this->IntensityGap[scanLine]);

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

        // If the point is invalid continue
        if (this->IsPointValid[scanLine][index] == 0)
          continue;

        // Else indicate that the point is an edge
        this->Label[scanLine][index] = 4;
        this->EdgesIndex.emplace_back(scanLine, index);
        //IsPointValidForPlanar[index] = 0;

        // Invalid its neighborhod
        const int indexBegin = std::max(0,        static_cast<int>(index - invalidNeighborhoodSize));
        const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + invalidNeighborhoodSize));
        for (int j = indexBegin; j <= indexEnd; ++j)
        {
          this->IsPointValid[scanLine][j] = 0;
        }
      }
    };

    // Edges using depth gap
    addEdgesUsingCriterion(sortedDepthGapIdx, this->DepthGap, squaredEdgeDepthGapThreshold, this->NeighborWidth - 1);
    // Edges using angles
    addEdgesUsingCriterion(sortedAnglesIdx, this->Angles, this->EdgeSinAngleThreshold, this->NeighborWidth);
    // Edges using saillancy
    addEdgesUsingCriterion(sortedSaillancyIdx, this->SaillantPoint, this->EdgeSaillancyThreshold, this->NeighborWidth - 1);
    // Edges using intensity
    addEdgesUsingCriterion(sortedIntensityGap, this->IntensityGap, this->EdgeIntensityGapThreshold, 1);

    // Planes (using angles)
    for (int k = Npts - 1; k >= 0; --k)
    {
      size_t index = sortedAnglesIdx[k];
      double sinAngle = this->Angles[scanLine][index];

      // thresh
      if (sinAngle > this->PlaneSinAngleThreshold)
      {
        break;
      }

      // if the point is invalid continue
      if (IsPointValidForPlanar[index] == 0)
      {
        continue;
      }

      // else indicate that the point is a planar one
      if ((this->Label[scanLine][index] != 4) && (this->Label[scanLine][index] != 3))
        this->Label[scanLine][index] = 2;
      this->PlanarIndex.emplace_back(scanLine, index);
      IsPointValidForPlanar[index] = 0;
      this->IsPointValid[scanLine][index] = 0;

      // Invalid its neighbor so that we don't have too
      // many planar keypoints in the same region. This is
      // required because of the k-nearest search + plane
      // approximation realized in the odometry part. Indeed,
      // if all the planar points are on the same scan line the
      // problem is degenerated since all the points are distributed
      // on a line.
      const int indexBegin = std::max(0,        static_cast<int>(index - 4));
      const int indexEnd   = std::min(Npts - 1, static_cast<int>(index + 4));
      for (int j = indexBegin; j <= indexEnd; ++j)
      {
        IsPointValidForPlanar[j] = 0;
      }
    }

    // Blobs Points
    // CHECK : why using only 1 point over 3?
    // TODO : disable blobs if not required
    // TODO : check if point is valid before adding it
    for (int k = 0; k < Npts; k = k + 3)
    {
      this->BlobIndex.emplace_back(scanLine, k);
    }
  }

  // add keypoints in increasing scan id order
  std::sort(this->EdgesIndex.begin(), this->EdgesIndex.end());
  std::sort(this->PlanarIndex.begin(), this->PlanarIndex.end());
  std::sort(this->BlobIndex.begin(), this->BlobIndex.end());

  // fill the keypoints vectors and compute the max dist keypoints
  this->FarestKeypointDist = 0.0;
  this->MinPoint = std::numeric_limits<float>::max();
  this->MaxPoint = std::numeric_limits<float>::min();
  for (const auto& edgeIndex: this->EdgesIndex)
  {
    const Point& p = this->pclCurrentFrameByScan[edgeIndex.first]->points[edgeIndex.second];
    this->EdgesPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(p.x * p.x + p.y * p.y + p.z * p.z));
    pcl::Array3fMapConst pointXYZ = p.getArray3fMap();
    this->MinPoint = this->MinPoint.min(pointXYZ);
    this->MaxPoint = this->MaxPoint.max(pointXYZ);
  }
  for (const auto& planarIndex: this->PlanarIndex)
  {
    const Point& p = this->pclCurrentFrameByScan[planarIndex.first]->points[planarIndex.second];
    this->PlanarsPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(p.x * p.x + p.y * p.y + p.z * p.z));
    pcl::Array3fMapConst pointXYZ = p.getArray3fMap();
    this->MinPoint = this->MinPoint.min(pointXYZ);
    this->MaxPoint = this->MaxPoint.max(pointXYZ);
  }
  for (const auto& blobIndex: this->BlobIndex)
  {
    const Point& p = this->pclCurrentFrameByScan[blobIndex.first]->points[blobIndex.second];
    this->BlobsPoints->push_back(p);
    this->FarestKeypointDist = std::max(this->FarestKeypointDist, static_cast<double>(p.x * p.x + p.y * p.y + p.z * p.z));
    pcl::Array3fMapConst pointXYZ = p.getArray3fMap();
    this->MinPoint = this->MinPoint.min(pointXYZ);
    this->MaxPoint = this->MaxPoint.max(pointXYZ);
  }
  this->FarestKeypointDist = std::sqrt(this->FarestKeypointDist);
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<double>> SpinningSensorKeypointExtractor::GetDebugArray()
{
  auto get1DVector =  [this](std::vector<std::vector<double>> const& array)
  {
    std::vector<double> v(this->pclCurrentFrame->size());
    std::vector<int> indexPerByScanLine(this->NLasers, 0);
    for (int i = 0; i < this->pclCurrentFrame->size(); i++)
    {
      double laserId = this->LaserIdMapping[this->pclCurrentFrame->points[i].laserId];
      v[i] = array[laserId][indexPerByScanLine[laserId]];
      indexPerByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<double>> map;
  map["angles_line"]    = get1DVector(this->Angles);
  map["saillant_point"] = get1DVector(this->SaillantPoint);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["is_point_valid"] = get1DVector(this->IsPointValid);
  map["keypoint_label"] = get1DVector(this->Label);
  return map;
}
