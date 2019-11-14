#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <Eigen/Geometry>

struct Transform
{
  double time = 0;
  union
  {
    double position[3] = { 0.0, 0.0, 0.0 };
    struct {
      double x;
      double y;
      double z;
    };
  };
  union
  {
    double orientation[3] = { 0.0, 0.0, 0.0 };
    struct {
      double rx;
      double ry;
      double rz;
    };
  };

  Transform() = default;

  Transform(double t, double x, double y, double z, double rx, double ry, double rz);

  Transform(double t, const Eigen::Matrix<double, 6, 1>& data);

  Transform(double t, const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy);

  Transform(double t, const Eigen::Isometry3d& transform);

  Transform(double t, const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot);

  Eigen::Isometry3d GetIsometry() const;

  Eigen::Translation3d GetTranslation() const;

  Eigen::Quaterniond GetRotation() const;
};

#endif // TRANSFORM_H