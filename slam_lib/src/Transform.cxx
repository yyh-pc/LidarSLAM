//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2019-11-06
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

#include "LidarSlam/Transform.h"

//------------------------------------------------------------------------------
Transform::Transform(double x, double y, double z, double roll, double pitch, double yaw,
                     double t, const std::string& frame)
  : time(t)
  , frameid(frame)
{
  Eigen::Translation3d trans(x, y, z);
  Eigen::Quaterniond rot(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  this->transform = trans * rot;
}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Matrix<double, 6, 1>& data,
                     double t, const std::string& frame)
  : time(t)
  , frameid(frame)
{
  Eigen::Translation3d trans(data[0], data[1], data[2]);
  Eigen::Quaterniond rot(Eigen::AngleAxisd(data[5], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(data[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(data[3], Eigen::Vector3d::UnitX()));
  this->transform = trans * rot;
}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy,
                     double t, const std::string& frame)
  : time(t)
  , frameid(frame)
{
  Eigen::Quaterniond rot(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
  this->transform = Eigen::Translation3d(trans) * rot;
}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Isometry3d& transform,
                     double t, const std::string& frame)
  : transform(transform)
  , time(t)
  , frameid(frame)
{}

//------------------------------------------------------------------------------
Transform::Transform(const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot,
                     double t, const std::string& frame)
  : transform(trans * rot.normalized())
  , time(t)
  , frameid(frame)
{}