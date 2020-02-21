#include "Transform.h"

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