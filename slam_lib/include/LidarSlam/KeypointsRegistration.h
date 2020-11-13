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

#ifndef KEYPOINTS_REGISTRATION_H
#define KEYPOINTS_REGISTRATION_H

#include "LidarSlam/KDTreePCLAdaptor.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/MotionModel.h"
#include "LidarSlam/Utilities.h"
#include "LidarSlam/Enums.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <pcl/point_cloud.h>

// Helper class to register one set of edges/planes/blobs keypoints onto
// another to estimate the 6D transformation between them.
// Firstly, a matching step is perfomed : we need to build the point-to-line,
// point-to-plane and point-to-blob residuals that will be optimized.
// Then, we use CERES Levenberg-Marquardt optimization to minimize the problem.
class KeypointsRegistration
{
public:
  using Point = PointXYZTIId;
  using PointCloud = pcl::PointCloud<Point>;
  using KDTree = KDTreePCLAdaptor<Point>;

  //! Structure to easily set all ICP/LM parameters
  struct Parameters
  {
    unsigned int NbThreads = 1;

    // **** ICP Parameters ****

    bool SingleEdgePerRing = false;

    // The max distance allowed between a keypoint from the current frame and its
    // neighborhood from the map (or previous frame) to build an ICP match.
    // If the distance is over this limit, no match residual will be built.
    double MaxDistanceForICPMatching = 5.;

    // Min number of matches
    // Below this threshold, we consider that there are not enough matches to
    // provide good enough optimization results, and registration is aborted.
    unsigned int MinNbrMatchedKeypoints = 20;

    // When computing the point <-> line and point <-> plane distance
    // in the ICP, the kNearest edges/planes points of the current
    // points are selected to approximate the line/plane using a PCA
    // If the one of the k-nearest points is too far the neigborhood
    // is rejected. We also make a filter upon the ratio of the eigen
    // values of the variance-covariance matrix of the neighborhood
    // to check if the points are distributed upon a line or a plane
    unsigned int LineDistanceNbrNeighbors = 10; //< number of neighbors edge points required to approximate the corresponding line
    unsigned int MinimumLineNeighborRejection = 4;
    double LineDistancefactor = 5.0;
    double MaxLineDistance = 0.2; //< maximum distance between keypoints and their computed line

    unsigned int PlaneDistanceNbrNeighbors = 5; //< number of neighbors planar points required to approximate the corresponding plane
    double PlaneDistancefactor1 = 35.0; //< PCA eigenvalues ratio to consider a neighborhood fits a plane model :
    double PlaneDistancefactor2 = 8.0;  //<     V2 < factor2 * V1  and  V1 > factor1 * V0
    double MaxPlaneDistance = 0.2; //< maximum distance between keypoints and their computed plane

    unsigned int BlobDistanceNbrNeighbors = 25; //< number of blob neighbors required to approximate the corresponding ellipsoid

    // **** LM optimization Parameters ****

    // Maximum number of iteration
    unsigned int LMMaxIter = 15;

    // Arctan loss scale factor to saturate costs according to their quality.
    // The loss function used is L(quality) = scale * arctan(quality / scale)
    // with quality is the quality of each keypoints match.
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

    // Number of successful matches (shortcut to RejectionsHistogram[SUCCESS])
    unsigned int NbMatches;
    // Matching result of each keypoint
    std::vector<MatchStatus> Rejections;
    // Histogram of the matching rejection causes
    std::array<int, MatchStatus::nStatus> RejectionsHistogram;
  };

  //! Estimation of registration error
  struct RegistrationError
  {
    // Estimation of the maximum position error
    double PositionError;
    // Direction of the maximum position error
    Eigen::Vector3d PositionErrorDirection;

    // Estimation of the maximum orientation error (in radians)
    double OrientationError;
    // Direction of the maximum orientation error
    Eigen::Vector3d OrientationErrorDirection;

    // Covariance matrix encoding the estimation of the pose's errors about the 6-DoF parameters
    // (DoF order : rX, rY, rZ, X, Y, Z)
    Eigen::Matrix6d Covariance;
  };

  //----------------------------------------------------------------------------

  KeypointsRegistration(const Parameters& params,
                        UndistortionMode undistortion,
                        const Eigen::Isometry3d& endPosePrior,
                        const Eigen::Isometry3d& startPosePrior = Eigen::Isometry3d::Identity());

  // Build point-to-neighborhood residuals
  MatchingResults BuildAndMatchResiduals(const PointCloud::Ptr& currPoints,
                                         const KDTree& prevPoints,
                                         Keypoint keypointType);

  // Optimize the Ceres problem
  ceres::Solver::Summary Solve();

  // Get optimization results
  Eigen::Isometry3d GetOptimizedEndPose() const { return RPYXYZtoIsometry(this->EndPoseArray); }
  Eigen::Isometry3d GetOptimizedStartPose() const { return RPYXYZtoIsometry(this->StartPoseArray); }

  // Estimate registration error
  RegistrationError EstimateRegistrationError();

  //----------------------------------------------------------------------------

private:

  // Add an ICP match residual.
  // To recover the motion, we have to minimize the function
  //   f(R, T) = sum(d(point, line)^2) + sum(d(point, plane)^2) + sum(d(point, blob)^2)
  // In all cases, the distance between the point and the line/plane/blob can be written :
  //    (R * X + T - P).t * A * (R * X + T - P)
  // Where :
  // - X is the key point
  // - P is a point of the line/plane/blob
  // - A is the distance operator :
  //    * A = (I - n*n.t)^2 for a line with n being a director vector of the line.
  //    * A = (n*n.t) for a plane with n being the normal.
  // - time store the time acquisition (used only if undistortion is enabled)
  // - weight will attenuate the distance function for outliers
  void AddIcpResidual(const Eigen::Matrix3d& A, const Eigen::Vector3d& P, const Eigen::Vector3d& X, double time, double weight = 1.);

  // Helper to compute point positions according to undistortion mode.
  // - pInit will be the initial position in sensor coordinates, on which we
  //   need to apply the transforms to optimize. This position will be used
  //   in optimization.
  // - pFinal will be estimated using the given prior to correspond to the
  //   approximate point location in global coordinates system. This position
  //   can be used to find approximate nearest neighbors in map.
  void ComputePointInitAndFinalPose(const Point& p, Eigen::Vector3d& pInit, Eigen::Vector3d& pFinal) const;

  // Match the current keypoint with its neighborhood in the map / previous
  // frame. From this match we compute the point-to-neighborhood distance function:
  //  (R * X + T - P).t * A * (R * X + T - P)
  // Where P is the mean point of the neighborhood and A is the symmetric
  // variance-covariance matrix encoding the shape of the neighborhood
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

  // Initialization of DoF to optimize
  const Eigen::Isometry3d EndPosePrior;
  const Eigen::Isometry3d StartPosePrior;

  // Frame pose interpolator, only used if undistortion is enabled
  const LinearTransformInterpolator<double> WithinFrameMotionPrior;

  // The problem to build and optimize
  ceres::Problem Problem;

  // DoF to optimize (= output)
  Eigen::Vector6d EndPoseArray;   ///< Pose at the end of frame (RPYXYZ)
  Eigen::Vector6d StartPoseArray; ///< Pose at the beginning of frame (RPYXYZ), only used if undistortion is enabled
};

#endif // KEYPOINTS_REGISTRATION_H
