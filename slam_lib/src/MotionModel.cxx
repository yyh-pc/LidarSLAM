//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
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

// LOCAL
#include "LidarSlam/MotionModel.h"

//-----------------------------------------------------------------------------
AffineIsometry::AffineIsometry(const Eigen::Matrix3d& argR, const Eigen::Vector3d& argT, double argTime)
  : R(argR)
  , T(argT)
  , time(argTime)
{}

//-----------------------------------------------------------------------------
AffineIsometry SampledSensorPath::operator()(double time) const
{
  Eigen::Matrix4d H = LinearTransformInterpolation<double>(this->Samples[0].R, this->Samples[0].T,
                                                           this->Samples[1].R, this->Samples[1].T,
                                                           time);
  return AffineIsometry(H.block(0, 0, 3, 3), H.block(0, 3, 3, 1), time);
}

//-----------------------------------------------------------------------------
Eigen::Isometry3d LinearInterpolation(const Eigen::Isometry3d& H0, const Eigen::Isometry3d& H1, double t, double t0, double t1)
{
  const double time = (t - t0) / (t1 - t0);
  Eigen::Quaterniond rot(Eigen::Quaterniond(H0.rotation()).slerp(time, Eigen::Quaterniond(H1.rotation())));
  Eigen::Translation3d trans(H0.translation() + time * (H1.translation() - H0.translation()));
  return trans * rot;
}