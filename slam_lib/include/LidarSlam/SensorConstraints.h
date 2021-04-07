#include "LidarSlam/CeresCostFunctions.h" // for residual structure + ceres
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{
namespace SensorConstraints
{

struct WheelOdomMeasurement
{
  double Time = 0.;
  double Distance = 0.;
};

struct GravityMeasurement
{
  double Time = 0.;
  Eigen::Vector3d Acceleration = Eigen::Vector3d::Zero();
};

// Wheel odometry constraint (unoriented)
bool GetWheelOdomConstraint(double lidarTime, double weight, const std::vector<WheelOdomMeasurement>& measures,
                            const Eigen::Isometry3d& previousTworld, CeresTools::Residual& residual);

// Wheel absolute abscisse constraint (unoriented)
bool GetWheelAbsoluteConstraint(double lidarTime, double weight, const std::vector<WheelOdomMeasurement>& measures,
                                CeresTools::Residual& residual);

// IMU constraint (gravity)
bool GetGravityConstraint(double lidarTime, double weight, const std::vector<GravityMeasurement>& measures,
                          const Eigen::Vector3d& gravityRef, CeresTools::Residual& residual);

Eigen::Vector3d ComputeGravityRef(const std::vector<GravityMeasurement>& measures, double deltaAngle);

} // end of SensorConstraints namespace
} // end of LidarSlam namespace