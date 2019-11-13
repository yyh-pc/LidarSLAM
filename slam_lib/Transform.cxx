#include "Transform.h"

//------------------------------------------------------------------------------
Transform::Transform(double t_, double x_, double y_, double z_, double rx_, double ry_, double rz_)
  : time(t_),
    x(x_), y(y_), z(z_),
    rx(rx_), ry(ry_), rz(rz_)
{}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Matrix<double, 6, 1>& data)
  : time(t),
    x(data[0]), y(data[1]), z(data[2]),
    rx(data[3]), ry(data[4]), rz(data[5])
{}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy)
  : time(t),
    x(trans(0)), y(trans(1)), z(trans(2)),
    rx(rpy(0)), ry(rpy(1)), rz(rpy(2))
{}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Isometry3d& transform)
  : time(t)
{
  Eigen::Translation3d trans(transform.translation());
  x = trans.x();
  y = trans.y();
  z = trans.z();
  Eigen::Vector3d rpy = transform.rotation().eulerAngles(2, 1, 0);
  rx = rpy(0);
  ry = rpy(1);
  rz = rpy(2);
}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot)
  : time(t),
    x(trans.x()), y(trans.y()), z(trans.z())
{
  Eigen::Vector3d rpy = rot.normalized().matrix().eulerAngles(2, 1, 0);
  rx = rpy(0);
  ry = rpy(1);
  rz = rpy(2);
}

//------------------------------------------------------------------------------
Eigen::Isometry3d Transform::GetIsometry() const
{
  Eigen::Translation3d trans = this->GetTranslation();
  Eigen::Quaterniond rot = this->GetRotation();
  return Eigen::Isometry3d(trans * rot);
}

//------------------------------------------------------------------------------
Eigen::Translation3d Transform::GetTranslation() const
{
  return Eigen::Translation3d(x, y, z);
}

//------------------------------------------------------------------------------
Eigen::Quaterniond Transform::GetRotation() const
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
}