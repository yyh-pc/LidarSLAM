#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <Eigen/Geometry>

struct Transform
{
  double time = 0;
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();

  //----------------------------------------------------------------------------

  Transform() = default;

  //! Uses Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(double t, double x, double y, double z, double rx, double ry, double rz);

  Transform(double t, const Eigen::Matrix<double, 6, 1>& data);

  //! Uses roll/pitch/yaw Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(double t, const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy);

  Transform(double t, const Eigen::Isometry3d& transform);

  Transform(double t, const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot);

  //----------------------------------------------------------------------------

  //! Direct access to translation (position) part.
  double& x() {return this->isometry(0, 3);}
  double& y() {return this->isometry(1, 3);}
  double& z() {return this->isometry(2, 3);}
  double x() const {return this->isometry(0, 3);}
  double y() const {return this->isometry(1, 3);}
  double z() const {return this->isometry(2, 3);}

  Eigen::Isometry3d GetIsometry() const {return this->isometry;}

  Eigen::Vector3d GetPosition() const {return this->isometry.translation();}

  Eigen::Translation3d GetTranslation() const {return Eigen::Translation3d(this->isometry.translation());}

  Eigen::Quaterniond GetRotation() const {return Eigen::Quaterniond(this->isometry.linear());}

  Eigen::Matrix4d GetMatrix() const {return this->isometry.matrix();}

};

#endif // TRANSFORM_H