#include "Transform.h"

//------------------------------------------------------------------------------
Transform::Transform(double t, double x, double y, double z, double roll, double pitch, double yaw)
  : time(t)
{
  Eigen::Translation3d trans(x, y, z);
  Eigen::Quaterniond rot(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  this->isometry = trans * rot;
}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Matrix<double, 6, 1>& data)
  : time(t)
{
  Eigen::Translation3d trans(data[0], data[1], data[2]);
  Eigen::Quaterniond rot(Eigen::AngleAxisd(data[5], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(data[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(data[3], Eigen::Vector3d::UnitX()));
  this->isometry = trans * rot;
}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy)
  : time(t)
{
  Eigen::Quaterniond rot(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
  this->isometry = Eigen::Translation3d(trans) * rot;
}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Isometry3d& transform)
  : time(t)
  , isometry(transform)
{}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot)
  : time(t)
  , isometry(trans * rot.normalized())
{}