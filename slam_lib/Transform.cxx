#include "Transform.h"

//------------------------------------------------------------------------------
Transform::Transform(double t, double x, double y, double z, double roll, double pitch, double yaw)
  : time(t),
    x(x), y(y), z(z),
    rx(roll), ry(pitch), rz(yaw)
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
  Eigen::Vector3d ypr = transform.rotation().eulerAngles(2, 1, 0);
  rx = ypr(2);
  ry = ypr(1);
  rz = ypr(0);
}

//------------------------------------------------------------------------------
Transform::Transform(double t, const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot)
  : time(t),
    x(trans.x()), y(trans.y()), z(trans.z())
{
  Eigen::Vector3d ypr = rot.normalized().matrix().eulerAngles(2, 1, 0);
  rx = ypr(2);
  ry = ypr(1);
  rz = ypr(0);
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