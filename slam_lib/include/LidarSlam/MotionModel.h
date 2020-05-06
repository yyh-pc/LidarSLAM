//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2019-04-08
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

#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

// EIGEN
#include <Eigen/Dense>

// CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>

/**
* \struct AffineIsometry
* \brief represents a bijective transformation from a 3D
*        euclidean affine space into another whose associated
*        linear application is an orthogonal morphism
*/
struct AffineIsometry
{
  AffineIsometry() = default;
  AffineIsometry(const Eigen::Matrix3d& argR, const Eigen::Vector3d& argT, double argTime);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  double time = 0;
};

/**
* \struct SampledSensorPath
* \brief represents the sampled sensor path estimated
*        using SLAM or measured using GPS / IMU data.
*        The orientation and position of the sensor for
*        a given time t that does not correspond to a
*        sampled time will be interpolated using linear
*        or spline interpolation (in R^3 and SO(3))
*/
struct SampledSensorPath
{
  std::vector<AffineIsometry> Samples = std::vector<AffineIsometry>(2);

  // return the affine isometry corresponding
  // to the requested time using a spline or
  // linear interpolation
  AffineIsometry operator()(double t) const;
};


/**
 * \struct LinearTransformInterpolator
 * \brief Linear interpolator to estimate an intermediate transform between two isometries.
 * 
 * At t=0, the first isometry is returned, at t=1 the second.
 * The translation will be interpolated linearly and the rotation spherical linearly.
 */
template <typename T>
struct LinearTransformInterpolator
{
  // Useful types
  using Vector3T = Eigen::Matrix<T, 3, 1>;
  using QuaternionT = Eigen::Quaternion<T>;
  using Translation3T = Eigen::Translation<T, 3>;
  using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

  LinearTransformInterpolator() = default;

  LinearTransformInterpolator(const Isometry3T& H0, const Isometry3T& H1)
    : Rot0(H0.linear()), Rot1(H1.linear()), Trans0(H0.translation()), Trans1(H1.translation()) {}
  
  // Set transforms
  void SetTransforms(const Isometry3T& H0, const Isometry3T& H1) { SetH0(H0); SetH1(H1); }
  void SetH0(const Isometry3T& H0) { Rot0 = QuaternionT(H0.linear()) ; Trans0 = H0.translation(); }
  void SetH1(const Isometry3T& H1) { Rot1 = QuaternionT(H1.linear()) ; Trans1 = H1.translation(); }

  // Return the affine isometry linearly interpolated at the requested time between H0 (t=0) and H1 (t=1).
  Isometry3T operator()(T t) const
  {
    return Translation3T(Trans0 + t * (Trans1 - Trans0))  // Translation part : linear interpolation
            * Rot0.slerp(t, Rot1);                        // Rotation part : spherical interpolation
  }

private:
  QuaternionT Rot0, Rot1;
  Vector3T Trans0, Trans1;
};

/**
 * \brief Interpolate spherical linearly between two isometries.
 * 
 * At t=t0, the first isometry is returned, at t=t1 the second.
 * The translation will be interpolated linearly and the rotation spherical linearly.
 */
Eigen::Isometry3d LinearInterpolation(const Eigen::Isometry3d& H0, const Eigen::Isometry3d& H1,
                                      double t, double t0 = 0., double t1 = 1.);

/**
* \class LinearTransformInterpolation
* \brief Perform the linear interpolation between
*        two isometric affine transforms. The rotational
*        part is interpolated using a SLERP and the translation
*        part is linearly interpolated. The function is templated
*        to be usable with dual number and autodifferentiation
*/
//-----------------------------------------------------------------------------
template <typename T>
Eigen::Matrix<T, 4, 4> LinearTransformInterpolation(const Eigen::Matrix<T, 3, 3>& R0, const Eigen::Matrix<T, 3, 1>& T0,
                                                    const Eigen::Matrix<T, 3, 3>& R1, const Eigen::Matrix<T, 3, 1>& T1,
                                                    T s)
{
  // Linearly interpolate the translation part
  Eigen::Matrix<T, 3, 1> Tc = (T(1.0) - s) * T0 + s * T1;

  // SLERP interpolation of the rotational part
  T r0[9], r1[9];
  T angle_axis_r0[3], angle_axis_r1[3];

  // column major
  for (unsigned int j = 0; j < 3; ++j)
  {
    for (unsigned int i = 0; i < 3; ++i)
    {
      r0[j + 3 * i] = R0(i, j);
      r1[j + 3 * i] = R1(i, j);
    }
  }

  T q0[4], q1[4], q[4];
  // Rotation matrix to quaternions
  ceres::RotationMatrixToAngleAxis(r0, angle_axis_r0);
  ceres::RotationMatrixToAngleAxis(r1, angle_axis_r1);
  ceres::AngleAxisToQuaternion(angle_axis_r0, q0);
  ceres::AngleAxisToQuaternion(angle_axis_r1, q1);

  // Canonical scalar product on quaternion
  T dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];

  // To prevent the SLERP interpolation to take the long path
  // we first check their relative orientation. If the angle
  // is superior to 90 we take the opposite quaternion which
  // is closer and represents the same rotation
  if(dot < T(0.0))
  {
    dot = -dot;
    for (unsigned int k = 0; k < 4; ++k)
    {
      q1[k] = T(-1.0) * q1[k];
    }
  }

  // To avoid division by zero, perform a linear interpolation (LERP), if our
  // quarternions are nearly in the same direction, otherwise resort
  // to spherical linear interpolation. In the limiting case (for small
  // angles), SLERP is equivalent to LERP.
  T t1, t2;
  if ((T(1.0) - ceres::abs(dot)) < T(1e-6))
  {
    t1 = T(1.0) - s;
    t2 = s;
  }
  else
  {
    // Angle (defined by the canonical scalar product for quaternions)
    // between the two quaternions
    const T theta = ceres::acos(dot);
    t1 = ceres::sin((T(1.0) - s) * theta) / ceres::sin(theta);
    t2 = ceres::sin(s * theta) / ceres::sin(theta);
  }
  for (unsigned int k = 0; k < 4; ++k)
  {
    q[k] = t1 * q0[k] + t2 * q1[k];
  }
  T r[9];
  ceres::QuaternionToRotation(q, r);

  Eigen::Matrix<T, 4, 4> H = Eigen::Matrix<T, 4, 4>::Zero();
  // column major
  for (unsigned int j = 0; j < 3; ++j)
  {
    for (unsigned int i = 0; i < 3; ++i)
    {
      H(i, j) = r[i + 3 * j];
    }
    H(j, 3) = Tc(j);
  }
  H(3, 3) = T(1);
  return H;
}

#endif // MOTION_MODEL_H