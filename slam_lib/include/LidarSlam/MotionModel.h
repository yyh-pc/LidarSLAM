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

#include <Eigen/Geometry>

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

#endif // MOTION_MODEL_H