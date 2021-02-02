//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2020-10-16
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
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/MotionModel.h"
#include "LidarSlam/Utilities.h"
#include "LidarSlam/Enums.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <pcl/point_cloud.h>

namespace LidarSlam
{

// Helper class to register a set of edge/plane/blob keypoints onto
// another to estimate the transformation between them.
// Firstly, a matching step is performed : we need to build the point-to-line,
// point-to-plane and point-to-blob residuals that will be optimized.
// Then, we use CERES Levenberg-Marquardt optimization to minimize the problem.
class KeypointsRegistration
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

    // **** ICP Parameters ****

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
    double MaxLineDistance = 0.2; //< maximum distance between keypoints and their computed line

    unsigned int PlaneDistanceNbrNeighbors = 5; //< number of neighbors planar points required to approximate the corresponding plane
    double PlaneDistancefactor1 = 35.0; //< PCA eigenvalues ratio to consider a neighborhood fits a plane model :
    double PlaneDistancefactor2 = 8.0;  //<     V1 >= factor1 * V0 and V2 <= factor2 * V1
    double MaxPlaneDistance = 0.2; //< maximum distance between keypoints and their computed plane

    unsigned int BlobDistanceNbrNeighbors = 25; //< number of blob neighbors required to approximate the corresponding ellipsoid

    // **** LM optimization Parameters ****

    // Maximum number of iteration
    unsigned int LMMaxIter = 15;

    // Arctan loss scale factor to saturate costs according to their quality.
    // The loss function used is L(quality) = scale * arctan(quality / scale)
    // with quality being the confidence associated to each keypoints match.
    double LossScale = 0.5;
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

    // Matching result of each keypoint
    std::vector<MatchStatus> Rejections;
    // Histogram of the matching rejection causes
    std::array<int, MatchStatus::nStatus> RejectionsHistogram = {};

    // Number of successful matches (shortcut to RejectionsHistogram[SUCCESS])
    unsigned int NbMatches() const { return RejectionsHistogram[SUCCESS]; }
  };

  //! Estimation of registration error
  struct RegistrationError
  {
    // Estimation of the maximum position error
    double PositionError = 0.;
    // Direction of the maximum position error
    Eigen::Vector3d PositionErrorDirection = Eigen::Vector3d::Zero();

    // Estimation of the maximum orientation error (in radians)
    double OrientationError = 0.;
    // Direction of the maximum orientation error
    Eigen::Vector3d OrientationErrorDirection = Eigen::Vector3d::Zero();

    // Covariance matrix encoding the estimation of the pose's errors about the 6-DoF parameters
    // (DoF order : X, Y, Z, rX, rY, rZ)
    Eigen::Matrix6d Covariance = Eigen::Matrix6d::Zero();
  };

  //----------------------------------------------------------------------------

  // Init ICP-LM optimizer.
  //  - if undistortion is NONE :
  //    FirstPose is the only considered transform, rigidly optimized.
  //    SecondPose is not even used, nor is time info.
  //  - if undistortion is APPROXIMATED :
  //    FirstPose and SecondPose are first used as fixed priors for linear undistortion during ICP,
  //    then only SecondPose is rigidly optimized.
  KeypointsRegistration(const Parameters& params,
                        UndistortionMode undistortion,
                        const Eigen::Isometry3d& firstPosePrior,
                        const Eigen::Isometry3d& secondPosePrior = Eigen::Isometry3d::Identity(),
                        double firstPoseTime = 0.,
                        double secondPoseTime = 1.);

  // Build point-to-neighborhood residuals
  MatchingResults BuildAndMatchResiduals(const PointCloud::Ptr& currPoints,
                                         const KDTree& prevPoints,
                                         Keypoint keypointType);

  // Optimize the Ceres problem
  ceres::Solver::Summary Solve();

  // Get optimization results
  Eigen::Isometry3d GetOptimizedFirstPose()  const { return Utils::XYZRPYtoIsometry(this->FirstPoseArray); }
  Eigen::Isometry3d GetOptimizedSecondPose() const { return Utils::XYZRPYtoIsometry(this->SecondPoseArray); }

  // Estimate registration error
  // If undistortion is disabled, this error is estimated for FirstPose.
  // If undistortion is enabled, this error is estimated for SecondPose.
  RegistrationError EstimateRegistrationError();

  //----------------------------------------------------------------------------

private:

  // Add an ICP match residual.
  // To recover the motion, we have to minimize the function
  //   f(R, T) = sum(d(edge_kpt, line)^2) + sum(d(plane_kpt, plane)^2) + sum(d(blob_kpt, blob)^2)
  // In all cases, the squared distance between the keypoint and the line/plane/blob can be written :
  //    (R * X + T - P).t * A * (R * X + T - P)
  // Where :
  // - X is the key point
  // - P is the mean point of the line/plane/blob neighborhood
  // - A is the squared distance operator :
  //    * A = (I - n*n.t)^2 for a line with n being a director vector of the line.
  //    * A = (n*n.t) for a plane with n being the normal.
  //    * A is the symmetric variance-covariance matrix encoding the shape of
  //      the neighborhood for a blob.
  // - time stores the time acquisition (used only if undistortion is enabled)
  // - weight attenuates the distance function for outliers
  void AddIcpResidual(const Eigen::Matrix3d& A, const Eigen::Vector3d& P, const Eigen::Vector3d& X, double time, double weight = 1.);

  // Helper to compute point positions according to undistortion mode.
  // - pInit will be the initial position in sensor coordinates, on which we
  //   need to apply the transforms to optimize. This position will be used
  //   in optimization.
  // - pFinal will be estimated using the given prior to correspond to the
  //   approximate point location in global coordinates system. This transformed
  //   position can be used to find approximate nearest neighbors in map.
  void ComputePointInitAndFinalPose(const Point& p, Eigen::Vector3d& pInit, Eigen::Vector3d& pFinal) const;

  // Match the current keypoint with its neighborhood in the map / previous
  // frame to estimate P and A.
  // From this match we will compute the point-to-neighborhood distance function:
  //   (R * X + T - P).t * A * (R * X + T - P)
  // where P is the mean point of the neighborhood, A is the squared distance operator,
  // X is the current point position and (R, T) the transform to optimize.
  MatchingResults::MatchStatus BuildLineMatch(const KDTree& kdtreePreviousEdges, const Point& p);
  MatchingResults::MatchStatus BuildPlaneMatch(const KDTree& kdtreePreviousPlanes, const Point& p);
  MatchingResults::MatchStatus BuildBlobMatch(const KDTree& kdtreePreviousBlobs, const Point& p);

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

  const UndistortionMode Undistortion;

  // The problem to build and optimize
  ceres::Problem Problem;

  // Initialization of DoF to optimize
  const Eigen::Isometry3d FirstPosePrior;   ///< Initial guess of the first pose to optimize
  const Eigen::Isometry3d SecondPosePrior;  ///< Initial guess of the second pose to optimize (only used if undistortion is enabled)

  // DoF to optimize (= output)
  Eigen::Vector6d FirstPoseArray;   ///< First pose to optimize (XYZRPY)
  Eigen::Vector6d SecondPoseArray;  ///< Second pose to optimize (XYZRPY) (only used if undistortion is enabled)

  // Frame pose interpolator (only used if undistortion is enabled)
  const LinearTransformInterpolator<double> WithinFrameMotionPrior;
};

} // end of LidarSlam namespace