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

  Transform(double t, const Eigen::Matrix<double, 6, 1>& data)
    : time(t),
      x(data[0]), y(data[1]), z(data[2]),
      rx(data[3]), ry(data[4]), rz(data[5]) {}
  
  Transform(double t, const Eigen::Vector3d& trans, const Eigen::Vector3d& rpy)
    : time(t),
      x(trans(0)), y(trans(1)), z(trans(2)),
      rx(rpy(0)), ry(rpy(1)), rz(rpy(2)) {}
  
  Transform(double t, const Eigen::Translation3d& trans, const Eigen::Quaterniond& rot)
    : time(t),
      x(trans.x()), y(trans.y()), z(trans.z())
  {
    Eigen::Vector3d rpy = rot.normalized().matrix().eulerAngles(2, 1, 0);
    rx = rpy(0);
    ry = rpy(1);
    rz = rpy(2);
  }
};