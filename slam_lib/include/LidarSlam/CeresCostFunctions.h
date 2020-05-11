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

#ifndef CERES_COST_FUNCTIONS_H
#define CERES_COST_FUNCTIONS_H

// LOCAL
#include "LidarSlam/MotionModel.h"
// CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>
// EIGEN
#include <Eigen/Dense>

namespace CostFunctions
{
template <typename T>
Eigen::Matrix<T, 3, 3> RotationMatrixFromRPY(const T& rx, const T& ry, const T& rz)
{
  const T crx = ceres::cos(rx);  const T srx = ceres::sin(rx);
  const T cry = ceres::cos(ry);  const T sry = ceres::sin(ry);
  const T crz = ceres::cos(rz);  const T srz = ceres::sin(rz);

  Eigen::Matrix<T, 3, 3> R;
  R << cry*crz,  srx*sry*crz-crx*srz,  crx*sry*crz+srx*srz,
       cry*srz,  srx*sry*srz+crx*crz,  crx*sry*srz-srx*crz,
          -sry,              srx*cry,              crx*cry;
  return R;
}

/**
 * \class MahalanobisDistanceAffineIsometryResidual
 * \brief Cost function to optimize the affine isometry transformation
 *        (rotation and translation) that minimizes the mahalanobis distance
 *        between a point X and its neighborhood encoded by the mean point C
 *        and the variance covariance matrix A
 *
 * It takes one 6D parameters block :
 *   - 3 first parameters to encode rotation with euler angles : rX, rY, rZ
 *   - 3 last parameters to encode translation : X, Y, Z
 */
//-----------------------------------------------------------------------------
struct MahalanobisDistanceAffineIsometryResidual
{
public:
  MahalanobisDistanceAffineIsometryResidual(const Eigen::Matrix3d& argA,
                                            const Eigen::Vector3d& argC,
                                            const Eigen::Vector3d& argX,
                                            double argWeight)
    : A(argA)
    , C(argC)
    , X(argX)
    , Weight(argWeight)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    // Get translation part
    Eigen::Map<const Vector3T> trans(&w[3]);

    // Get rotation part, in a static way.
    // The idea is that all residual functions will need to evaluate those
    // sin/cos so we only compute them once each time the parameters change.
    static Matrix3T rot = Matrix3T::Identity();
    static T lastRot[3] = {T(-1.), T(-1.), T(-1.)};
    if (!std::equal(w, w + 3, lastRot))
    {
      rot = RotationMatrixFromRPY(w[0], w[1], w[2]);
      std::copy(w, w + 3, lastRot);
    }

    // Compute Y = R(theta) * X + T - C
    const Vector3T Y = rot * X + trans - C;

    // Compute final residual value which is:
    //   Ht * A * H with H = R(theta)X + T
    const T squaredResidual = Weight * (Y.transpose() * A * Y)(0);

    // Since t -> sqrt(t) is not differentiable in 0, we check the value of the
    // distance infenitesimale part. If it is not finite, it means that the
    // first order derivative has been evaluated in 0
    residual[0] = squaredResidual < 1e-6 ? T(0) : ceres::sqrt(squaredResidual);

    return true;
  }

private:
  const Eigen::Matrix3d A;
  const Eigen::Vector3d C;
  const Eigen::Vector3d X;
  const double Weight;
};

/**
* \class MahalanobisDistanceLinearDistortionResidual
* \brief Cost function to minimize to estimate the rotation R1 and translation T1 so that:
         The linearly interpolated transform:
         (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
         applies to X acquired at time t minimizes the mahalanobis distance.
*/
//-----------------------------------------------------------------------------
struct MahalanobisDistanceLinearDistortionResidual
{
public:
  MahalanobisDistanceLinearDistortionResidual(const Eigen::Matrix3d& argA,
                                              const Eigen::Vector3d& argC,
                                              const Eigen::Vector3d& argX,
                                              const Eigen::Vector3d& argT0,
                                              const Eigen::Matrix3d& argR0,
                                              double argTime,
                                              double argLambda)
  {
    this->A = argA;
    this->R0 = argR0;
    this->C = argC;
    this->X = argX;
    this->T0 = argT0;
    this->time = argTime;
    this->lambda = argLambda;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> Ac = this->A.cast<T>();
    Eigen::Matrix<T, 3, 3> R0c = this->R0.cast<T>();
    Eigen::Matrix<T, 3, 1> Xc = this->X.cast<T>();
    Eigen::Matrix<T, 3, 1> Cc = this->C.cast<T>();
    Eigen::Matrix<T, 3, 1> T0c = this->T0.cast<T>();

    Eigen::Matrix<T, 3, 1> T1c(w[3], w[4], w[5]);

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Compute final rotation value
    Eigen::Matrix<T, 3, 3> R1c;
    R1c << cry*crz,  srx*sry*crz-crx*srz,  crx*sry*crz+srx*srz,
           cry*srz,  srx*sry*srz+crx*crz,  crx*sry*srz-srx*crz,
              -sry,              srx*cry,              crx*cry;

    // Now, compute the rotation and translation to
    // apply to X depending on (R0, T0) and (R1, T1)
    // The applied isometry will be the linear
    // interpolation between these two transforms:
    // (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
    Eigen::Matrix<T, 4, 4> H = LinearTransformInterpolation<T>(R0c, T0c, R1c, T1c, T(this->time));
    Eigen::Matrix<T, 3, 3> Rc = H.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> Tc = H.block(0, 3, 3, 1);

    // Compute final residual value which is:
    // Yt * A * Y with Y = R(theta) * X + T - C
    Eigen::Matrix<T, 3, 1> Y = Rc * Xc + Tc  - Cc;
    T squaredResidual = T(lambda) * (Y.transpose() * Ac * Y)(0);

    // since t -> sqrt(t) is not differentiable
    // in 0, we check the value of the distance
    // infenitesimale part. If it is not finite
    // it means that the first order derivative
    // has been evaluated in 0
    if (squaredResidual < T(1e-6))
    {
      residual[0] = T(0);
    }
    else
    {
      residual[0] = ceres::sqrt(squaredResidual);
    }
    return true;
  }

private:
  Eigen::Matrix3d A;
  Eigen::Matrix3d R0;
  Eigen::Vector3d T0;
  Eigen::Vector3d C;
  Eigen::Vector3d X;
  double time;
  double lambda;
};

/**
* \class MahalanobisDistanceIsometryAndLinearDistortionResidual
* \brief Cost function to minimize to estimate the rotation R1 and translation T1 so that:
         The linearly interpolated transform:
         (R, T) = (R1 * R1^t, t * R1 * T + T)
         applies to X acquired at time t minimizes the mahalanobis distance.
*/
//-----------------------------------------------------------------------------
struct MahalanobisDistanceIsometryAndLinearDistortionResidual
{
public:
  MahalanobisDistanceIsometryAndLinearDistortionResidual(const Eigen::Matrix3d& argA,
                                                         const Eigen::Vector3d& argC,
                                                         const Eigen::Vector3d& argX,
                                                         double argTime,
                                                         double argLambda)
  {
    this->A = argA;
    this->C = argC;
    this->X = argX;
    this->time = argTime;
    this->lambda = argLambda;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> Ac = this->A.cast<T>();
    Eigen::Matrix<T, 3, 1> Xc = this->X.cast<T>();
    Eigen::Matrix<T, 3, 1> Cc = this->C.cast<T>();

    Eigen::Matrix<T, 3, 1> T1c(w[3], w[4], w[5]);

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Compute final rotation value
    Eigen::Matrix<T, 3, 3> R1c;
    R1c << cry*crz,  srx*sry*crz-crx*srz,  crx*sry*crz+srx*srz,
           cry*srz,  srx*sry*srz+crx*crz,  crx*sry*srz-srx*crz,
              -sry,              srx*cry,              crx*cry;

    // Now, compute the rotation and translation to
    // apply to X depending on (R0, T0) and (R1, T1)
    // The applied isometry will be the linear
    // interpolation between these two transforms:
    // (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
    Eigen::Matrix<T, 4, 4> H = LinearTransformInterpolation<T>(R1c, T1c, R1c * R1c, R1c * T1c + T1c, T(this->time));
    Eigen::Matrix<T, 3, 3> Rc = H.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> Tc = H.block(0, 3, 3, 1);

    // Compute final residual value which is:
    // Yt * A * Y with Y = R(theta) * X + T - C
    Eigen::Matrix<T, 3, 1> Y = Rc * Xc + Tc  - Cc;
    T squaredResidual = T(lambda) * (Y.transpose() * Ac * Y)(0);

    // since t -> sqrt(t) is not differentiable
    // in 0, we check the value of the distance
    // infenitesimale part. If it is not finite
    // it means that the first order derivative
    // has been evaluated in 0
    if (squaredResidual < T(1e-6))
    {
      residual[0] = T(0);
    }
    else
    {
      residual[0] = ceres::sqrt(squaredResidual);
    }
    return true;
  }

private:
  Eigen::Matrix3d A;
  Eigen::Vector3d C;
  Eigen::Vector3d X;
  double time;
  double lambda;
};

/**
 * \class MahalanobisDistanceInterpolatedMotionResidual
 * \brief Cost function to optimize the isometries (R0, T0) and  (R1, T1) so that:
 *        The linearly interpolated transform:
 *        (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
 *        applies to X acquired at time t minimizes the mahalanobis distance.
 *
 * It takes two 6D parameters blocks :
 *  1) First isometry H0 :
 *   - 3 parameters (0, 1, 2) to encode rotation R0 with euler angles : rX, rY, rZ
 *   - 3 parameters (3, 4, 5) to encode translation T0 : X, Y, Z
 *  2) Second isometry H1 :
 *   - 3 parameters (6, 7, 8) to encode rotation R1 with euler angles : rX, rY, rZ
 *   - 3 parameters (9, 10, 11) to encode translation T1 : X, Y, Z
 */
//-----------------------------------------------------------------------------
struct MahalanobisDistanceInterpolatedMotionResidual
{
public:
  MahalanobisDistanceInterpolatedMotionResidual(const Eigen::Matrix3d& argA,
                                                const Eigen::Vector3d& argC,
                                                const Eigen::Vector3d& argX,
                                                double argTime,
                                                double argWeight)
    : A(argA)
    , C(argC)
    , X(argX)
    , Time(argTime)
    , Weight(argWeight)
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
      H0.linear() << RotationMatrixFromRPY(w0[0], w0[1], w0[2]);
      H0.translation() << w0[3], w0[4], w0[5];
      transformInterpolator.SetH0(H0);
      std::copy(w0, w0 + 6, lastW0);
    }

    // Update H1 if needed
    if (!std::equal(w1, w1 + 6, lastW1))
    {
      H1.linear() << RotationMatrixFromRPY(w1[0], w1[1], w1[2]);
      H1.translation() << w1[3], w1[4], w1[5];
      transformInterpolator.SetH1(H1);
      std::copy(w1, w1 + 6, lastW1);
    }

    // Compute the transform to apply to X depending on (R0, T0) and (R1, T1).
    // The applied isometry will be the linear interpolation between them :
    // (R, T) = (R0^(1-t) * R1^t, (1 - t)T0 + tT1)
    const Isometry3T H = transformInterpolator(T(this->Time));

    // Compute final residual value which is:
    //  Yt * A * Y with Y = R(theta) * X + T - C
    const Vector3T Y = H.linear() * this->X + H.translation() - this->C;
    const T squaredResidual = this->Weight * (Y.transpose() * this->A * Y)(0);

    // Since t -> sqrt(t) is not differentiable in 0, we check the value of the
    // distance infenitesimale part. If it is not finite, it means that the
    // first order derivative has been evaluated in 0
    residual[0] = squaredResidual < 1e-6 ? T(0) : ceres::sqrt(squaredResidual);

    return true;
  }

private:
  const Eigen::Matrix3d A;
  const Eigen::Vector3d C;
  const Eigen::Vector3d X;
  const double Time;
  const double Weight;
};

/**
* \class FrobeniusDistanceRotationCalibrationResidual
* \brief Cost function to minimize to estimate the calibration rotation between two sensors
*        based on their trajectory. To do that, we exploit the "solid-system"
*        constraint that links the coordinate reference frame of the two sensors.
*
* Using the geometric constraints that come from the solid-assumption, it is
* possible to estimate the 3-DoF rotation calibration (R in SO(3)) between
* two sensors by using the estimation of their poses over the time.
*
* Let's t0 and t1 be two times
* Let's R be the orientation of the sensor 1 according to the sensor 2 reference frame
* Since we have the solid-assumption, R is constant and not time depending
*
* Let's P0 (resp P1) in SO(3) be the orientation of the sensor 1 at time t0 (resp t1)
*
* Let's Q0 (resp Q1) in SO(3) be the orientation of the sensor 2 at time t0 (resp t1)
*
* From these two temporal points on the poses "trajectory", it is possible to express
* the change of orientation between the sensor 1 at time t0 and sensor 1 at time t1
* using two differents way
*
* 1- By using the poses of the sensor 1 at time t0 and t1:
*    dR0 = P0' * P1
*
* 2- By using the solid-assumption and firstly express the point in the
*    other sensor reference frame using the calibration parameters. Then, using
*    method 1 for the second sensor and finally using the calibration again to
*    dR1 = R' * Q0' * Q1 * R
*
* And finally, we are looking for R that satisfy:
* dR1 = dR0
*
* This lead to a non-linear least square problem that can be solved using a
* levenberg-marquardt algorithm. The non-linearity comes from R belonging to
* SO(3) manifold. We estimate R using the Euler-Angle mapping between R^3 and SO(3)
* R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
*/
//-----------------------------------------------------------------------------
struct FrobeniusDistanceRotationCalibrationResidual
{
public:
  FrobeniusDistanceRotationCalibrationResidual(const Eigen::Matrix3d& argP1, const Eigen::Matrix3d& argP2,
                                               const Eigen::Matrix3d& argQ1, const Eigen::Matrix3d& argQ2)
  {
    this->P1 = argP1; this->P2 = argP2;
    this->Q1 = argQ1; this->Q2 = argQ2;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> P1j = this->P1.cast<T>();
    Eigen::Matrix<T, 3, 3> P2j = this->P2.cast<T>();
    Eigen::Matrix<T, 3, 3> Q1j = this->Q1.cast<T>();
    Eigen::Matrix<T, 3, 3> Q2j = this->Q2.cast<T>();

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R0;
    R0 << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
          cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
             -sry,               srx*cry,               crx*cry;

    // Compute the residual matrix
    Eigen::Matrix<T, 3, 3> ResidualMatrix = R0.transpose() * Q1j.transpose() * Q2j * R0 - P1j.transpose() * P2j;

    // Compute final residual value which is the frobenius norme
    // of the residual matrix
    T squaredResidual = (ResidualMatrix.transpose() * ResidualMatrix).trace();

    // since t -> sqrt(t) is not differentiable
    // in 0, we check the value of the distance
    // infenitesimale part. If it is not finite
    // it means that the first order derivative
    // has been evaluated in 0
    if (squaredResidual < T(1e-6))
    {
      residual[0] = T(0);
    }
    else
    {
      residual[0] = ceres::sqrt(squaredResidual);
    }

    return true;
  }

private:
  Eigen::Matrix3d P1, P2, Q1, Q2;
};

/**
* \class FrobeniusDistanceRotationAndTranslationCalibrationResidual
* \brief Cost function to minimize to estimate the calibration rotation and translation
*        between two sensors based on their trajectory. To do that, we exploit the
*        "solid-system" constraint that links the coordinate reference frame of the two sensors.
*
* Using the geometric constraints that come from the solid-assumption, it is
* possible to estimate the 6-DoF calibration (R in SO(3), T in R^3) between
* two sensors by using the estimation of their poses over the time.
*
* Let's t0 and t1 be two times
* Let's R, T be the pose of the sensor 1 according to the sensor 2 reference frame
* Since we have the solid-assumption, R and T are constant and not time depending
*
* Let's P0 (resp P1) in SO(3) be the orientation of the sensor 1 at time t0 (resp t1)
* Let's V0 (resp V1) in R^3 be the position of the sensor 1 at time t0 (resp t1)
*
* Let's Q0 (resp Q1) in SO(3) be the orientation of the sensor 2 at time t0 (resp t1)
* Let's U0 (resp U1) in R^3 be the position of the sensor 2 at time t0 (resp t1)
*
* From these two temporal points on the poses "trajectory", it is possible to express
* the change of reference frame between the sensor 1 at time t0 and sensor 1 at time t1
* using two differents way
*
* 1- By using the poses of the sensor 1 at time t0 and t1:
*    dR0 = P0' * P1
*    dT0 = P0' * (V1 - V0)
*
* 2- By using the solid-assumption and firstly express the point in the
*    other sensor reference frame using the calibration parameters. Then, using
*    method 1 for the second sensor and finally using the calibration again to
*    dR1 = R' * Q0' * Q1 * R
*    dT1 = R0' * (Q0' * (Q1 * T + (U1 - U0)) - T)
*
* And finally, we are looking for R and T that satisfies:
* dR1 = dR0
* dT1 = dT0
*
* This lead to a non-linear least square problem that can be solved using a
* levenberg-marquardt algorithm. The non-linearity comes from R belonging to
* SO(3) manifold. We estimate R using the Euler-Angle mapping between R^3 and SO(3)
* R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
*/
//-----------------------------------------------------------------------------
struct FrobeniusDistanceRotationAndTranslationCalibrationResidual
{
public:
  FrobeniusDistanceRotationAndTranslationCalibrationResidual(const Eigen::Matrix3d& argP1, const Eigen::Matrix3d& argP2,
                                                             const Eigen::Matrix3d& argQ1, const Eigen::Matrix3d& argQ2,
                                                             const Eigen::Vector3d& argV1, const Eigen::Vector3d& argV2,
                                                             const Eigen::Vector3d& argU1, const Eigen::Vector3d& argU2)
  {
    this->P1 = argP1; this->P2 = argP2;
    this->Q1 = argQ1; this->Q2 = argQ2;
    this->V1 = argV1; this->V2 = argV2;
    this->U1 = argU1; this->U2 = argU2;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 3> P1j = this->P1.cast<T>();
    Eigen::Matrix<T, 3, 3> P2j = this->P2.cast<T>();
    Eigen::Matrix<T, 3, 3> Q1j = this->Q1.cast<T>();
    Eigen::Matrix<T, 3, 3> Q2j = this->Q2.cast<T>();
    Eigen::Matrix<T, 3, 1> U1j = this->U1.cast<T>();
    Eigen::Matrix<T, 3, 1> U2j = this->U2.cast<T>();
    Eigen::Matrix<T, 3, 1> V1j = this->V1.cast<T>();
    Eigen::Matrix<T, 3, 1> V2j = this->V2.cast<T>();

    Eigen::Matrix<T, 3, 1> dX(w[3], w[4], w[5]);

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R0;
    R0 << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
          cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
             -sry,               srx*cry,               crx*cry;

    Eigen::Matrix<T, 3, 1> T1 = R0.transpose() * (Q1j.transpose() * (Q2j * dX + (U2j - U1j)) - dX);
    Eigen::Matrix<T, 3, 1> T2 = P1j.transpose() * (V2j - V1j);
    T squaredResidual = ((T1 - T2).transpose() * (T1 - T2))(0);

    // since t -> sqrt(t) is not differentiable
    // in 0, we check the value of the distance
    // infenitesimale part. If it is not finite
    // it means that the first order derivative
    // has been evaluated in 0
    if (squaredResidual < T(1e-6))
    {
      residual[0] = T(0);
    }
    else
    {
      residual[0] = ceres::sqrt(squaredResidual);
    }

    return true;
  }

private:
  Eigen::Matrix3d P1, P2, Q1, Q2;
  Eigen::Vector3d V1, V2, U1, U2;
};

/**
* \class EuclideanDistanceAffineIsometryResidual
* \brief Cost function to minimize to estimate the rotation and translation
*        between two sensors based on their trajectory. To do that we minimize
*        the euclidean distance between matched points from the trajectory
*        according to the rotational and translational parameters. It results
*        an affine transform "best" fit the second trajectory upon the second
*
* Let's t in R be the current time
* Let's X be the position of the sensor 1 at time t
* Met's Y be the position of the sensor 2 at time t
*
* We want to estimate R0 in SO(3) and T0 in R^3 such that
* Y = RX + T
*
* This lead to a non-linear least square problem that can be solved using a
* levenberg-marquardt algorithm. The non-linearity comes from R belonging to
* SO(3) manifold. We estimate R using the Euler-Angle mapping between R^3 and SO(3)
* R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
*/
//-----------------------------------------------------------------------------
struct EuclideanDistanceAffineIsometryResidual
{
public:
  EuclideanDistanceAffineIsometryResidual(const Eigen::Vector3d& argX, const Eigen::Vector3d& argY)
  {
    this->X = argX;
    this->Y = argY;
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    // Convert internal double matrix
    // to a Jet matrix for auto diff calculous
    Eigen::Matrix<T, 3, 1> Xj = this->X.cast<T>();
    Eigen::Matrix<T, 3, 1> Yj = this->Y.cast<T>();

    Eigen::Matrix<T, 3, 1> dX(w[3], w[4], w[5]);

    // store sin / cos values for this angle
    T crx = ceres::cos(w[0]); T srx = ceres::sin(w[0]);
    T cry = ceres::cos(w[1]); T sry = ceres::sin(w[1]);
    T crz = ceres::cos(w[2]); T srz = ceres::sin(w[2]);

    // Create current rotation
    Eigen::Matrix<T, 3, 3> R0;
    R0 << cry*crz, (srx*sry*crz-crx*srz), (crx*sry*crz+srx*srz),
          cry*srz, (srx*sry*srz+crx*crz), (crx*sry*srz-srx*crz),
             -sry,               srx*cry,               crx*cry;

    Eigen::Matrix<T, 3, 1> L = R0 * Xj + dX - Yj;
    T squaredResidual = (L.transpose() * L)(0);

    // since t -> sqrt(t) is not differentiable
    // in 0, we check the value of the distance
    // infenitesimale part. If it is not finite
    // it means that the first order derivative
    // has been evaluated in 0
    if (squaredResidual < T(1e-6))
    {
      residual[0] = T(0);
    }
    else
    {
      residual[0] = ceres::sqrt(squaredResidual);
    }

    return true;
  }

private:
  Eigen::Vector3d X, Y;
};
}

#endif // CERES_COST_FUNCTIONS_H