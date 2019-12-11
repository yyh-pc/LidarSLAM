#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <Eigen/Geometry>

struct Transform
{
  std::string frameid = "";   ///< Name of the frame coordinates the transform is represented into.
  double time = 0;            ///< [s] Timestamp of this transform.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();  ///< Isometry representing pose/transform

  //----------------------------------------------------------------------------

  Transform() = default;

  //! Uses Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(double x, double y, double z, double rx, double ry, double rz, double t = 0., const std::string& frame = "");

  //! Uses (X, Y, Z, rX, rY, rZ) as input. Uses Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(const Eigen::Matrix<double, 6, 1>& data, double t = 0., const std::string& frame = "");

  //! Uses roll/pitch/yaw Euler angles ZYX convention to build isometry (= non fixed axis YPR convention).
  Transform(const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy, double t = 0., const std::string& frame = "");

  Transform(const Eigen::Isometry3d& transform, double t = 0., const std::string& frame = "");

  Transform(const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot, double t = 0., const std::string& frame = "");

  //----------------------------------------------------------------------------

  //! Direct access to translation (position) part.
  double& x() {return this->transform(0, 3);}
  double& y() {return this->transform(1, 3);}
  double& z() {return this->transform(2, 3);}
  double x() const {return this->transform(0, 3);}
  double y() const {return this->transform(1, 3);}
  double z() const {return this->transform(2, 3);}

  Eigen::Isometry3d& GetIsometry() {return this->transform;}
  Eigen::Isometry3d GetIsometry() const {return this->transform;}

  Eigen::Vector3d GetPosition() const {return this->transform.translation();}

  Eigen::Translation3d GetTranslation() const {return Eigen::Translation3d(this->transform.translation());}

  Eigen::Quaterniond GetRotation() const {return Eigen::Quaterniond(this->transform.linear());}

  Eigen::Matrix4d GetMatrix() const {return this->transform.matrix();}

};

#endif // TRANSFORM_H