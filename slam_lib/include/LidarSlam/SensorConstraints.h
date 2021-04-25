#include "LidarSlam/CeresCostFunctions.h" // for residual structure + ceres
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{
  
#define SetSensorMacro(name,type) void Set##name (type _arg) { this->name = _arg; }
#define GetSensorMacro(name,type) type Get##name () const { return this->name; }

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

template <typename T>
class SensorManager
{
public:
  //Setters/Getters
  GetSensorMacro(Weight, double)
  SetSensorMacro(Weight, double)

  GetSensorMacro(TimeOffset, double)
  SetSensorMacro(TimeOffset, double)

  GetSensorMacro(Measures, std::vector<T>)
  SetSensorMacro(Measures, const std::vector<T>&)

  GetSensorMacro(Residual, CeresTools::Residual)
  void ResetResidual()
  {
    this->Residual.Cost.reset();
    this->Residual.Robustifier.reset();
  }

  // Add one measure at a time in measures vector
  void AddMeasurement(const T& m) {this->Measures.emplace_back(m);}
  void Reset()
  {
    this->ResetResidual();
    this->Measures.clear();
    this->PreviousIdx = -1;
    this->TimeOffset = 0.;
  }

  // Check if sensor can be used in optimization
  // The weight must be not null and the measures vector must contain elements
  bool CanBeUsed() {return this->Weight > 1e-6 && !this->Measures.empty();}

protected:
  // Index of measurement used for previous frame
  int PreviousIdx = -1;
  // Measures stored
  std::vector<T> Measures;
  // Weight to apply to sensor constraint
  double Weight = 0.;
  // Time offset to make external sensors/Lidar correspondance
  double TimeOffset = 0.;
  // Resulting residual
  CeresTools::Residual Residual;
};

class WheelOdometryManager : public SensorManager<WheelOdomMeasurement>
{
public:
  //Setters/Getters
  GetSensorMacro(PreviousPose, Eigen::Isometry3d)
  SetSensorMacro(PreviousPose, const Eigen::Isometry3d&)

  // Wheel odometry constraint (unoriented)
  void ComputeWheelOdomConstraint(double lidarTime);
  // Wheel absolute abscisse constraint (unoriented)
  void ComputeWheelAbsoluteConstraint(double lidarTime);

private:
  // Members used when using the relative distance with last estimated pose
  Eigen::Isometry3d PreviousPose = Eigen::Isometry3d::Identity();
  double PreviousDistance = 0.;
};

class ImuManager : public SensorManager<GravityMeasurement>
{
public:
  //Setters/Getters
  GetSensorMacro(GravityRef, Eigen::Vector3d)
  SetSensorMacro(GravityRef, const Eigen::Vector3d&)

  // IMU constraint (gravity)
  void ComputeGravityConstraint(double lidarTime);
  // Compute Reference gravity vector from IMU measurements
  void ComputeGravityRef(double deltaAngle);

private:
  Eigen::Vector3d GravityRef = Eigen::Vector3d::Zero();
};

} // end of SensorConstraints namespace
} // end of LidarSlam namespace