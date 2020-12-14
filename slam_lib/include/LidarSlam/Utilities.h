//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Laurenson Nick (Kitware SAS)
// Creation date: 2019-05-13
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#ifndef LIDAR_SLAM_UTILITIES_H
#define LIDAR_SLAM_UTILITIES_H

#include <pcl/point_cloud.h>

#include <Eigen/Eigenvalues>

#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <chrono>
#include <math.h>
#include <numeric>

//==============================================================================
//   Usefull macros or typedefs
//==============================================================================

// If M_PI is not defined, define it
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// Set cout to print floating values with fixed precision of a given number of decimals
#define SET_COUT_FIXED_PRECISION(decimals)                   \
  std::streamsize ss = std::cout.precision();                \
  std::cout << std::fixed << std::setprecision(decimals);

// Reset cout float printing state back to before
// NOTE : RESET_COUT_FIXED_PRECISION in the same scope as SET_COUT_FIXED_PRECISION
#define RESET_COUT_FIXED_PRECISION                           \
  std::cout << std::setprecision(ss);                        \
  std::cout.unsetf(std::ios::fixed | std::ios_base::floatfield);

// Print with colors on terminals that support ANSI color codes
// (Supported by UNIX systems and from Windows 10)
#define DEFAULT "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define GRAY    "\033[37m"
#define PRINT_COLOR(color, s) std::cout << color << s << DEFAULT << std::endl;
#define PRINT_WARNING(s) std::cerr << YELLOW << "[WARNING] " << s << DEFAULT << std::endl;
#define PRINT_ERROR(s)   std::cerr << RED    << "[ERROR] "   << s << DEFAULT << std::endl;

namespace Eigen
{
  ///! @brief 6x6 matrix of double
  using Matrix6d = Matrix<double, 6, 6>;

  ///! @brief 6D Vector of double
  using Vector6d = Matrix<double, 6, 1>;
}

namespace LidarSlam
{
namespace Utils
{
//==============================================================================
//   Common helpers
//==============================================================================

//------------------------------------------------------------------------------
/*!
 * @brief Sort a vector and return sorted indices
 * @param v The vector to sort
 * @param ascending If true, sort in ascending (increasing) order
 * @return The sorted indices such that the first index is the biggest input
 *         value and the last the smallest.
 */
template<typename T>
std::vector<size_t> SortIdx(const std::vector<T>& v, bool ascending=false)
{
  // Initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // Sort indices based on comparing values in v
  if (ascending)
    std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });
  else
    std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });

  return idx;
}

//==============================================================================
//   Geometry helpers
//==============================================================================

//------------------------------------------------------------------------------
/*!
 * @brief Convert from radians to degrees
 * @param rad The angle value, in radians
 * @return The angle value, in degrees
 */
template<typename T>
inline constexpr T Rad2Deg(const T& rad)
{
  return rad / M_PI * 180.;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert from degrees to radians
 * @param deg The angle value, in degrees
 * @return The angle value, in radians
 */
template<typename T>
inline constexpr T Deg2Rad(const T& deg)
{
  return deg / 180. * M_PI;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert roll, pitch and yaw euler angles to rotation matrix
 * @param roll Rotation on X axis
 * @param pitch Rotation on Y axis
 * @param yaw Rotation on Z axis
 * @return The 3x3 rotation matrix defined by : R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
inline Eigen::Matrix3d RPYtoRotationMatrix(double roll, double pitch, double yaw)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
}

//------------------------------------------------------------------------------
/*!
 * @brief Get Roll Pitch and Yaw euler angles from rotation matrix
 * @param rot The 3x3 rotation matrix, defined by : R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * @return Euler angles around X, Y and Z axes, according to ZYX convention.
 */
inline Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d& rot)
{
  // `rpy = rot.eulerAngles(2, 1, 0).reverse()`             returns angles in range [-PI:PI]x[-PI:PI]x[0:PI].
  // `rpy = Eigen::EulerAnglesZYXd(rot).angles().reverse()` returns angles in range [-PI:PI]x[-PI:PI]x[-PI:PI].
  // But these are bad. For first range, yaw angle cannot be negative : this
  // leads to un-necessary non trivial RPY decomposition, and to unstable
  // optimization result as we are not optimizing around 0.
  // For second ranges, there exist several RPY decomposition for the same
  // rotation (one of them being non-trivial too). Therefore the optimization
  // may also be unstable by oscillating between them.
  // We prefer to output angles in range [-PI:PI]x[-PI/2:PI/2]x[-PI:PI] : we
  // allow negative values to avoid oscillation artefacts, and minimize the
  // pitch angle to fix representation.
  Eigen::Vector3d rpy;
  rpy.x() = std::atan2(rot(2, 1), rot(2, 2));
  rpy.y() = -std::asin(rot(2, 0));
  rpy.z() = std::atan2(rot(1, 0), rot(0, 0));
  return rpy;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert translation and RPY to full rigid transform
 * @param x Translation along X axis
 * @param y Translation along Y axis
 * @param z Translation along Z axis
 * @param roll Rotation on X axis
 * @param pitch Rotation on Y axis
 * @param yaw Rotation on Z axis
 * @return The rigid transform (rotaion + translation)
 */
inline Eigen::Isometry3d XYZRPYtoIsometry(double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d transform;
  transform.linear() = RPYtoRotationMatrix(roll, pitch, yaw);  // Set rotation part
  transform.translation() = Eigen::Vector3d(x, y, z);          // Set translation part
  transform.makeAffine();                                      // Set the last row to [0 0 0 1]
  return transform;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert translation and RPY to full rigid transform
 * @param xyzrpy Translation (x, Y, Z) and Rotation (rX, rY, rZ)
 * @return The rigid transform (rotation + translation)
 */
template<typename T>
inline Eigen::Isometry3d XYZRPYtoIsometry(const T& xyzrpy)
{
  return XYZRPYtoIsometry(xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert RPY and translation to full rigid transform
 * @param rpyxyz Rotation (rX, rY, rZ) and Translation (x, Y, Z)
 * @return The rigid transform (rotation + translation)
 */
template<typename T>
inline Eigen::Isometry3d RPYXYZtoIsometry(const T& rpyxyz)
{
  return XYZRPYtoIsometry(rpyxyz[3], rpyxyz[4], rpyxyz[5], rpyxyz[0], rpyxyz[1], rpyxyz[2]);
}

//-----------------------------------------------------------------------------
/*!
 * @brief Get X, Y, Z, Roll, Pitch, Yaw from rigid transform
 * @param transform The rigid transform
 * @return 6D array, with translation (X, Y, Z) and rotation (rX, rY, rZ)
 */
inline Eigen::Vector6d IsometryToXYZRPY(const Eigen::Isometry3d& transform)
{
  Eigen::Vector6d xyzrpy;
  xyzrpy << transform.translation(), RotationMatrixToRPY(transform.linear());
  return xyzrpy;
}

//------------------------------------------------------------------------------
/*!
 * @brief Get Roll, Pitch, Yaw, X, Y, Z from rigid transform
 * @param transform The rigid transform
 * @return 6D array, with rotation (rX, rY, rZ) and translation (X, Y, Z)
 */
inline Eigen::Vector6d IsometryToRPYXYZ(const Eigen::Isometry3d& transform)
{
  Eigen::Vector6d rpyxyz;
  rpyxyz << RotationMatrixToRPY(transform.linear()), transform.translation();
  return rpyxyz;
}

//------------------------------------------------------------------------------
/*!
 * @brief Compute PCA of Nx3 data array and mean value
 * @param[in] data Nx3 array (e.g. stacked 3D points)
 * @param[out] mean Where to store mean value
 * @return The PCA
 */
inline Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ComputePCA(const Eigen::Matrix<double, Eigen::Dynamic, 3>& data,
                                                                 Eigen::Vector3d& mean)
{
  mean = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - mean.transpose();
  Eigen::Matrix3d varianceCovariance = centered.transpose() * centered;
  return Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(varianceCovariance);
}

//------------------------------------------------------------------------------
/*!
 * @brief Compute PCA of Nx3 data array and mean value
 * @param data Nx3 array (e.g. stacked 3D points)
 * @return The PCA
 */
inline Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ComputePCA(const Eigen::Matrix<double, Eigen::Dynamic, 3>& data)
{
  Eigen::Vector3d mean;
  return ComputePCA(data, mean);
}

//==============================================================================
//   PCL helpers
//==============================================================================

//------------------------------------------------------------------------------
/*!
 * @brief Apply a rigid transform to a point (in-place transformation).
 * @param p The point to transform, will be in-place transformed.
 * @param transform The rigid transform (rotation + translation) to apply.
 */
template<typename PointT>
inline void TransformPoint(PointT& p, const Eigen::Isometry3d& transform)
{
  p.getVector4fMap() = (transform * p.getVector4fMap().template cast<double>()).template cast<float>();
}

//------------------------------------------------------------------------------
/*!
 * @brief Apply a rigid transform to a point
 * @param p The point to transform
 * @param transform The rigid transform (rotation + translation) to apply.
 * @return The transformed point (all other fields are copied from p)
 */
template<typename PointT>
inline PointT TransformPoint(const PointT& p, const Eigen::Isometry3d& transform)
{
  PointT out(p);
  TransformPoint(out, transform);
  return out;
}

//------------------------------------------------------------------------------
/*!
 * @brief Copy pointcloud metadata to an other cloud
 * @param[in] from The pointcloud to copy info from
 * @param[out] to The pointcloud to copy info to
 */
template<typename PointT>
inline void CopyPointCloudMetadata(const pcl::PointCloud<PointT>& from, pcl::PointCloud<PointT>& to)
{
  to.header = from.header;
  to.is_dense = from.is_dense;
  to.sensor_orientation_ = from.sensor_orientation_;
  to.sensor_origin_ = from.sensor_origin_;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert PCL timestamp (in microseconds) to seconds
 * @param pclStampMs PCL timestamp, in microseconds
 * @return Timestamp in seconds
 */
inline constexpr double PclStampToSec(uint64_t pclStampMs)
{
  return pclStampMs * 1e-6;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert seconds to PCL timestamp (in microseconds)
 * @param seconds Timestamp, in seconds
 * @return PCL timestamp in microseconds, rounded up to closer integer microsecond
 */
inline uint64_t SecToPclStamp(double seconds)
{
  return std::round(seconds * 1e6);
}

//==============================================================================
//   Processing duration measurements
//==============================================================================

//------------------------------------------------------------------------------
// Anonymous namespace to avoid multiple-definitions thanks to internal linkage
// These variables will be defined locally in each translation unit.
namespace
{
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> startTimestamps;
  std::unordered_map<std::string, double> totalDurations;
  std::unordered_map<std::string, unsigned int> totalCalls;
} // end of anonymous namespace

//------------------------------------------------------------------------------
/*!
 * @brief Reset timers values.
 * 
 * NOTE: This only resets timers declared in a given compilation unit.
 */
inline void ResetTimers()
{
  startTimestamps.clear();
  totalDurations.clear();
  totalCalls.clear();
}

//------------------------------------------------------------------------------
/*!
 * @brief Init a timer.
 * @param timer The name of the timer
 */
inline void InitTime(const std::string& timer)
{
  startTimestamps[timer] = std::chrono::steady_clock::now();
}

//------------------------------------------------------------------------------
/*!
 * @brief Get the timer value.
 * @param timer The name of the timer
 * @return The duration value, in seconds, since the initialization of the timer
 * 
 * NOTE : This returns 0 if the counter has not been initialized yet.
 */
inline double GetTime(const std::string& timer)
{
  std::chrono::duration<double> chrono_ms = std::chrono::steady_clock::now() - startTimestamps[timer];
  return chrono_ms.count();
}

//------------------------------------------------------------------------------
/*!
 * @brief Print a given timer value and its average value
 * @param timer The name of the timer
 * 
 * NOTE : This returns 0 if the counter has not been initialized yet.
 */
inline void StopTimeAndDisplay(const std::string& timer)
{
  const double currentDuration = GetTime(timer);
  totalDurations[timer] += currentDuration;
  totalCalls[timer]++;
  double meanDurationMs = totalDurations[timer] * 1000. / totalCalls[timer];
  SET_COUT_FIXED_PRECISION(3);
  PRINT_COLOR(CYAN, "  -> " << timer << " took : " << currentDuration * 1000. << " ms (average : " << meanDurationMs << " ms)");
  RESET_COUT_FIXED_PRECISION;
}

}  // end of Utils namespace
}  // end of LidarSlam namespace

#endif // LIDAR_SLAM_UTILITIES_H