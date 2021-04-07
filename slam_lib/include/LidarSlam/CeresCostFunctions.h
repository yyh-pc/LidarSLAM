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

#pragma once

// LOCAL
#include "LidarSlam/MotionModel.h"
// CERES
#include <ceres/ceres.h>
// EIGEN
#include <Eigen/Geometry>

namespace LidarSlam
{
namespace CeresTools
{
struct Residual
{
  std::shared_ptr<ceres::CostFunction> Cost;
  std::shared_ptr<ceres::LossFunction> Robustifier;
};
}
namespace CeresCostFunctions
{

namespace Utils
{
namespace
{
//------------------------------------------------------------------------------
/**
 * \brief Build rotation matrix from euler angles.
 *
 * It estimates R using the Euler-Angle mapping between R^3 and SO(3) :
 *   R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
 */
template <typename T>
Eigen::Matrix<T, 3, 3> RotationMatrixFromRPY(const T& rx, const T& ry, const T& rz)
{
  const T cx = ceres::cos(rx);  const T sx = ceres::sin(rx);
  const T cy = ceres::cos(ry);  const T sy = ceres::sin(ry);
  const T cz = ceres::cos(rz);  const T sz = ceres::sin(rz);

  Eigen::Matrix<T, 3, 3> R;
  R << cy*cz,  sx*sy*cz-cx*sz,  cx*sy*cz+sx*sz,
       cy*sz,  sx*sy*sz+cx*cz,  cx*sy*sz-sx*cz,
         -sy,           sx*cy,           cx*cy;
  return R;
}
} // end of anonymous namespace
} // end of Utils namespace

//------------------------------------------------------------------------------
/**
 * \class MahalanobisDistanceAffineIsometryResidual
 * \brief Cost function to optimize the affine isometry transformation
 *        (rotation and translation) so that the mahalanobis distance
 *        between a point X and its neighborhood is minimized.
 *
 * More precisely, in case the user is interested in implementing a cost function of the form
 * cost(x) = (x - P)^T C^{-1} (x - P) where, P is a mean and C is a covariance matrix,
 * then, A = C^{-1/2}, i.e the matrix A is the square root of the inverse of the covariance, 
 * also known as the stiffness matrix.
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with euler angles : rX, rY, rZ
 *
 * It outputs a 3D residual block.
 */
struct MahalanobisDistanceAffineIsometryResidual
{
  MahalanobisDistanceAffineIsometryResidual(const Eigen::Matrix3d& argA,
                                            const Eigen::Vector3d& argP,
                                            const Eigen::Vector3d& argX)
    : A(argA)
    , P(argP)
    , X(argX)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    // Get translation part
    Eigen::Map<const Vector3T> trans(&w[0]);

    // Get rotation part, in a static way.
    // The idea is that all residual functions will need to evaluate those
    // sin/cos so we only compute them once each time the parameters change.
    static Matrix3T rot = Matrix3T::Identity();
    static T lastRot[3] = {T(-1.), T(-1.), T(-1.)};
    if (!std::equal(w + 3, w + 6, lastRot))
    {
      rot = Utils::RotationMatrixFromRPY(w[3], w[4], w[5]);
      std::copy(w + 3, w + 6, lastRot);
    }

    // Transform point with rotation and translation
    const Vector3T Y = rot * X + trans;

    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = A * (Y - P);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static std::shared_ptr<ceres::CostFunction> Create(const Eigen::Matrix3d& argA,
                                                     const Eigen::Vector3d& argP,
                                                     const Eigen::Vector3d& argX)
  {
    return std::shared_ptr<ceres::AutoDiffCostFunction<MahalanobisDistanceAffineIsometryResidual, 3, 6>>(
                       new ceres::AutoDiffCostFunction<MahalanobisDistanceAffineIsometryResidual, 3, 6>
                       (new MahalanobisDistanceAffineIsometryResidual(argA, argP, argX))
                       );
  }

private:
  const Eigen::Matrix3d A;
  const Eigen::Vector3d P;
  const Eigen::Vector3d X;
};

//------------------------------------------------------------------------------
/**
 * \class MahalanobisDistanceInterpolatedMotionResidual
 * \brief Cost function to optimize the affine isometry transformations H0=(R0, T0) and H1=(R1, T1)
 *        at timestamps t0=0 and t1=1 so that the linearly interpolated transform
 *        (R, T) = (R0^(1-t) * R1^t, (1 - t) T0 + t T1)
 *        applied to X (acquired at time t) minimizes the mahalanobis distance.
 *
 * More precisely, in case the user is interested in implementing a cost function of the form
 * cost(x) = (x - P)^T C^{-1} (x - P) where, P is a mean vector and C is a covariance matrix,
 * then, A = C^{-1/2}, i.e the matrix A is the square root of the inverse of the covariance, 
 * also known as the stiffness matrix.
 *
 * This function takes two 6D parameters blocks :
 *  1) First isometry H0 :
 *   - 3 parameters (0, 1, 2) to encode translation T0 : X, Y, Z
 *   - 3 parameters (3, 4, 5) to encode rotation R0 with euler angles : rX, rY, rZ
 *  2) Second isometry H1 :
 *   - 3 parameters (6, 7, 8) to encode translation T1 : X, Y, Z
 *   - 3 parameters (9, 10, 11) to encode rotation R1 with euler angles : rX, rY, rZ
 *
 * It outputs a 3D residual block.
 */
struct MahalanobisDistanceInterpolatedMotionResidual
{
  MahalanobisDistanceInterpolatedMotionResidual(const Eigen::Matrix3d& argA,
                                                const Eigen::Vector3d& argP,
                                                const Eigen::Vector3d& argX,
                                                double argTime)
    : A(argA)
    , P(argP)
    , X(argX)
    , Time(argTime)
  {}

  template <typename T>
  bool operator()(const T* const w0, const T* const w1, T* residual) const
  {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Create H0 / H1 transforms in static way.
    // The idea is that all residual functions will need to
    // evaluate those variables so we will only compute then
    // once each time the parameters values change
    static Isometry3T H0 = Isometry3T::Identity(), H1 = Isometry3T::Identity();
    static LinearTransformInterpolator<T> transformInterpolator;
    static T lastW0[6] = {T(-1.), T(-1.), T(-1.), T(-1.), T(-1.), T(-1.)};
    static T lastW1[6] = {T(-1.), T(-1.), T(-1.), T(-1.), T(-1.), T(-1.)};

    // Update H0 if needed
    if (!std::equal(w0, w0 + 6, lastW0))
    {
      H0.linear() << Utils::RotationMatrixFromRPY(w0[3], w0[4], w0[5]);
      H0.translation() << w0[0], w0[1], w0[2];
      transformInterpolator.SetH0(H0);
      std::copy(w0, w0 + 6, lastW0);
    }

    // Update H1 if needed
    if (!std::equal(w1, w1 + 6, lastW1))
    {
      H1.linear() << Utils::RotationMatrixFromRPY(w1[3], w1[4], w1[5]);
      H1.translation() << w1[0], w1[1], w1[2];
      transformInterpolator.SetH1(H1);
      std::copy(w1, w1 + 6, lastW1);
    }

    // Compute the transform to apply to X depending on (R0, T0) and (R1, T1).
    // The applied isometry will be the linear interpolation between them :
    // (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
    const Isometry3T H = transformInterpolator(Time);
    
    // Transform point with rotation and translation
    const Vector3T Y = H.linear() * X + H.translation();
    
    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = A * (Y - P);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static std::shared_ptr<ceres::CostFunction> Create(const Eigen::Matrix3d& argA,
                                                     const Eigen::Vector3d& argP,
                                                     const Eigen::Vector3d& argX,
                                                     double argTime)
  {
    return std::shared_ptr<ceres::AutoDiffCostFunction<MahalanobisDistanceInterpolatedMotionResidual, 3, 6, 6>>(
                       new ceres::AutoDiffCostFunction<MahalanobisDistanceInterpolatedMotionResidual, 3, 6, 6>
                       (new MahalanobisDistanceInterpolatedMotionResidual(argA, argP, argX, argTime))
                       );
  }

private:
  const Eigen::Matrix3d A;
  const Eigen::Vector3d P;
  const Eigen::Vector3d X;
  const double Time;
};


//------------------------------------------------------------------------------
/**
 * \class OdometerDistanceResidual
 * \brief Cost function to optimize the translation of the affine isometry
 *        transformation (rotation and translation) so that the distance 
 *        from a previous known pose corresponds to an external sensor odometry measure.
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - [unused] 3 last parameters to encode rotation with euler angles : rX, rY, rZ
 * 
 * It outputs a 1D residual block.
 */
struct OdometerDistanceResidual
{
  OdometerDistanceResidual(const Eigen::Vector3d& previousPos,
                           double distanceToPreviousPose)
    : PreviousPos(previousPos)
    , DistanceToPreviousPose(distanceToPreviousPose)
  {}

  template <typename T>
  bool operator()(const T* const t, T* residual) const
  {
    // Get translation part
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    Eigen::Map<const Vector3T> pos(t);

    // Compute residual
    T motionSqNorm = (pos - PreviousPos).squaredNorm();
    T motionNorm = (motionSqNorm < 1e-6) ? T(0) : ceres::sqrt(motionSqNorm);
    residual[0] = motionNorm - DistanceToPreviousPose;
    return true;
  }

  // Factory to hide the construction of the CostFunction
  // object from the client code.
  static std::shared_ptr<ceres::CostFunction> Create(const Eigen::Vector3d& previousPos,
                                                     double distanceToPreviousPose)
  {
    return std::shared_ptr<ceres::AutoDiffCostFunction<OdometerDistanceResidual, 1, 6>>(
                       new ceres::AutoDiffCostFunction<OdometerDistanceResidual, 1, 6>
                       (new OdometerDistanceResidual(previousPos, distanceToPreviousPose))
                       );
  }

private:
  const Eigen::Vector3d PreviousPos;
  const double DistanceToPreviousPose;
};

//------------------------------------------------------------------------------
/**
 * \class ImuGravityAlignmentResidual
 * \brief Cost function to optimize the orientation of the affine
 *        isometry transformation (rotation and translation) so
 *        that the gravity vector corresponds to a reference.
 *        This gravity vector is usually supplied by an IMU.
 *
 * This function takes one 6D parameters block :
 *   - [unused] 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with euler angles : rX, rY, rZ
 * 
 * It outputs a 3D residual block.
 */
struct ImuGravityAlignmentResidual
{
  ImuGravityAlignmentResidual(const Eigen::Vector3d& refGravityDir,
                              const Eigen::Vector3d& currGravityDir)
    : ReferenceGravityDirection(refGravityDir)
    , CurrentGravityDirection(currGravityDir)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    // Get rotation part 
    Matrix3T rot = Utils::RotationMatrixFromRPY(w[3], w[4], w[5]);

    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = rot * CurrentGravityDirection - ReferenceGravityDirection;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static std::shared_ptr<ceres::CostFunction> Create(const Eigen::Vector3d& referenceGravityDirection,
                                                     const Eigen::Vector3d& currentGravityDirection)
  {
    return std::shared_ptr<ceres::AutoDiffCostFunction<ImuGravityAlignmentResidual, 3, 6>>(
                       new ceres::AutoDiffCostFunction<ImuGravityAlignmentResidual, 3, 6>
                       (new ImuGravityAlignmentResidual(referenceGravityDirection, currentGravityDirection))
                       );
  }

private:
  const Eigen::Vector3d ReferenceGravityDirection;
  const Eigen::Vector3d CurrentGravityDirection;
};

} // end of namespace CeresCostFunctions
} // end of LidarSlam namespace