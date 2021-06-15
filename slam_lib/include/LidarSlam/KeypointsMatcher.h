//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2021-03-01
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

#include "LidarSlam/KDTreePCLAdaptor.h"
#include "LidarSlam/CeresCostFunctions.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/MotionModel.h"
#include "LidarSlam/Utilities.h"
#include "LidarSlam/Enums.h"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>

namespace LidarSlam
{

// Helper class to match edge/planar/blob keypoints and build ceres residuals
class KeypointsMatcher
{
public:
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;
  using KDTree = KDTreePCLAdaptor<Point>;

  //! Structure to easily set all ICP/LM parameters
  struct Parameters
  {
    // Max number of threads to use to parallelize computations
    unsigned int NbThreads = 1;

    // When searching edge keypoint nearest neighbors, we can follow different
    // strategies to keep only relevant matches, instead of taking all k-nearest points.
    // If false, the method GetRansacLineNeighbors() will be used.
    // If true, the method GetPerRingLineNeighbors() will be used.
    bool SingleEdgePerRing = false;

    // The max distance allowed between a current keypoint and its neighborhood
    // from the map (or previous frame) to build an ICP match.
    // If the distance is over this limit, no match residual will be built.
    double MaxDistanceForICPMatching = 5.;

    // Min number of matches
    // Below this threshold, we consider that there are not enough matches to
    // provide good enough optimization results, and registration is aborted.
    unsigned int MinNbrMatchedKeypoints = 20;

    // When computing the point <-> line and point <-> plane distance in the ICP,
    // the kNearest edge/plane points of the current point are selected to
    // approximate the line/plane using a PCA.
    // If one of the k-nearest points is too far, the neigborhood is rejected.
    // We also perform a filter upon the ratio of the eigen values of the
    // covariance matrix of the neighborhood to check if the points are
    // distributed upon a line or a plane.
    unsigned int LineDistanceNbrNeighbors = 10; //< initial number of neighbor edge points searched to approximate the corresponding line
    unsigned int MinimumLineNeighborRejection = 4;  //< number of neighbor edge points required to approximate the corresponding line after filtering startegy
    double LineDistancefactor = 5.0; //< PCA eigenvalues ratio to consider a neighborhood fits a line model : V2 >= factor * V1
    double MaxLineDistance = 0.2; //< maximum RMSE between target keypoints and their fitted line

    unsigned int PlaneDistanceNbrNeighbors = 5; //< number of neighbors planar points required to approximate the corresponding plane
    double PlaneDistancefactor1 = 35.0; //< PCA eigenvalues ratio to consider a neighborhood fits a plane model :
    double PlaneDistancefactor2 = 8.0;  //<     V1 >= factor1 * V0 and V2 <= factor2 * V1
    double MaxPlaneDistance = 0.2; //< maximum RMSE between target keypoints and their fitted plane

    unsigned int BlobDistanceNbrNeighbors = 25; //< number of blob neighbors required to approximate the corresponding ellipsoid

    // Maximum distance (in meters) beyond which the residual errors are
    // saturated to robustify the optimization against outlier constraints.
    // The residuals will be robustified by Tukey loss at scale sqrt(SatDist).
    double SaturationDistance = 1.;
  };

  //! Result of matching for one set of keypoints
  struct MatchingResults
  {
    //! Result of the keypoint matching, explaining rejection cause of matching failure.
    enum MatchStatus : uint8_t
    {
      SUCCESS = 0,              ///< Keypoint has been successfully matched
      NOT_ENOUGH_NEIGHBORS = 1, ///< Not enough neighbors to match keypoint
      NEIGHBORS_TOO_FAR = 2,    ///< Neighbors are too far to match keypoint
      BAD_PCA_STRUCTURE = 3,    ///< PCA eigenvalues analysis discards neighborhood fit to model
      INVALID_NUMERICAL = 4,    ///< Optimization parameter computation has numerical invalidity
      MSE_TOO_LARGE = 5,        ///< Mean squared error to model is too important to accept fitted model
      UNKOWN = 6,               ///< Unkown status (matching probably not performed yet)
      nStatus = 7
    };

    //! Match status and quality weight of each keypoint
    struct MatchInfo
    {
      MatchStatus Status;
      double Weight;
      CeresTools::Residual Cost;
    };

    // Vector of residual functions to add to ceres problem
    std::vector<CeresTools::Residual> Residuals;

    // Matching result of each keypoint
    std::vector<MatchStatus> Rejections;
    std::vector<double> Weights;
    // Histogram of the matching rejection causes
    std::array<int, MatchStatus::nStatus> RejectionsHistogram = {};

    // Number of successful matches (shortcut to RejectionsHistogram[SUCCESS])
    unsigned int NbMatches() const { return this->RejectionsHistogram[SUCCESS]; }

    void Reset(const unsigned int N)
    {
      this->Weights.assign(N, 0.);
      this->Rejections.assign(N, MatchingResults::MatchStatus::UNKOWN);
      this->RejectionsHistogram.fill(0);
      this->Residuals.assign(N, CeresTools::Residual());
    }
  };

  //----------------------------------------------------------------------------

  // Init matcher
  // It needs matching parameters and the prior transform to apply to keypoints
  KeypointsMatcher(const Parameters& params, const Eigen::Isometry3d& posePrior);

  // Build point-to-neighborhood residuals
  MatchingResults BuildMatchResiduals(const PointCloud::Ptr& currPoints,
                                      const KDTree& prevPoints,
                                      Keypoint keypointType);

  //----------------------------------------------------------------------------

private:

  // Build ICP match residual functions.
  // To recover the motion, we have to minimize the function
  //   f(R, T) = sum(d(edge_kpt, line)^2) + sum(d(plane_kpt, plane)^2) + sum(d(blob_kpt, blob)^2)
  // In all cases, the squared Mahalanobis distance between the keypoint and the line/plane/blob can be written :
  //   (R * X + T - P).t * A.t * A * (R * X + T - P)
  // Where :
  // - (R, T) is the rigid transform to optimize 
  // - X is the key point
  // - P is the mean point of the line/plane/blob neighborhood
  // - A is the distance operator:
  //    * A = (I - u*u.t) for a line with u being the unit tangent vector of the line.
  //    * A = (n*n.t) for a plane with n being its normal.
  //    * A = C^{-1/2} is the squared information matrix, aka stiffness matrix, where 
  //      C is the covariance matrix encoding the shape of the neighborhood for a blob.
  // - weight attenuates the distance function for outliers
  CeresTools::Residual BuildResidual(const Eigen::Matrix3d& A, const Eigen::Vector3d& P, const Eigen::Vector3d& X, double weight = 1.);

  // Match the current keypoint with its neighborhood in the map / previous
  MatchingResults::MatchInfo BuildLineMatch(const KDTree& kdtreePreviousEdges, const Point& p);
  MatchingResults::MatchInfo BuildPlaneMatch(const KDTree& kdtreePreviousPlanes, const Point& p);
  MatchingResults::MatchInfo BuildBlobMatch(const KDTree& kdtreePreviousBlobs, const Point& p);

  // Instead of taking the k-nearest neigbors we will take specific neighbor
  // using the particularities of the lidar sensor
  void GetPerRingLineNeighbors(const KDTree& kdtreePreviousEdges, const double pos[3],
                               unsigned int knearest, std::vector<int>& validKnnIndices,
                               std::vector<float>& validKnnSqDist) const;

  // Instead of taking the k-nearest neighbors we will take specific neighbor
  // using a sample consensus model
  void GetRansacLineNeighbors(const KDTree& kdtreePreviousEdges, const double pos[3],
                              unsigned int knearest, double maxDistInlier,
                              std::vector<int>& validKnnIndices,
                              std::vector<float>& validKnnSqDist) const;

  //----------------------------------------------------------------------------

private:

  const Parameters Params;

  // Initialization of DoF to optimize
  const Eigen::Isometry3d PosePrior;  ///< Initial guess of the pose to optimize
};

} // end of LidarSlam namespace