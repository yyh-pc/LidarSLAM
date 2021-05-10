//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2020-12-10
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

#pragma once

#include <pcl/point_cloud.h>
#include <cmath>
#include <map>

namespace lidar_conversions
{
namespace Utils
{

//------------------------------------------------------------------------------
/*!
 * @brief Copy pointcloud metadata to an other cloud
 * @param[in] from The pointcloud to copy info from
 * @param[out] to The pointcloud to copy info to
 */
template<typename PointI, typename PointO>
inline void CopyPointCloudMetadata(const pcl::PointCloud<PointI>& from, pcl::PointCloud<PointO>& to)
{
  to.header = from.header;
  to.is_dense = from.is_dense;
  to.sensor_orientation_ = from.sensor_orientation_;
  to.sensor_origin_ = from.sensor_origin_;
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if a PCL point is valid
 * @param point A PCL point, with possible NaN as coordinates.
 * @return true if all point coordinates are valid, false otherwise.
 */
template<typename PointT>
inline bool IsFinite(const PointT& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

//------------------------------------------------------------------------------
/*!
 * @struct Helper to estimate point-wise within frame advancement for a spinning
 * lidar sensor using azimuth angle.
 */
struct SpinningFrameAdvancementEstimator
{
  /*!
   * @brief Reset the estimator. This should be called when a new frame is received.
   */
  void Reset()
  {
    this->PreviousAdvancementPerRing.clear();
  }

  /*!
   * @brief Estimate point advancement within current frame using azimuth angle.
   * @param point the point to estimate its relative advancement. It MUST have a 'laser_id' field.
   * @return relative advancement in range [0;~1].
   *
   * This computation is based on azimuth angle of each measured point.
   * The first point will return a 0 advancement.
   * Advancement will increase clock-wise, reaching 1 when azimuth angle reaches
   * initial azimuth value. It may be greater than 1 if the frame spins more
   * than 360 degrees.
   *
   * NOTE: this estimation uses 2 priors:
   * - real azimuth is always strictly inceasing within each laser ring,
   * - real azimuth angle of any first point of a laser ring should be greater than initial azimuth.
   */
  template<typename PointT>
  double operator()(const PointT& point)
  {
    // Compute normalized azimuth angle (in range [0-1])
    double pointAdvancement = (M_PI - std::atan2(point.y, point.x)) / (2 * M_PI);

    // If this is the first point of the frame
    if (this->PreviousAdvancementPerRing.empty())
      this->InitAdvancement = pointAdvancement;

    // Get normalized angle (in [0-1]), with angle 0 being first point direction
    auto wrapMax = [](double x, double max) { return std::fmod(max + std::fmod(x, max), max); };
    double frameAdvancement = wrapMax(pointAdvancement - this->InitAdvancement, 1.);

    // If we detect overflow, correct it
    // If current laser_id is not in map, the following line will insert it,
    // associating it to value 0.0.
    if (frameAdvancement < this->PreviousAdvancementPerRing[point.laser_id])
      frameAdvancement += 1.;
    this->PreviousAdvancementPerRing[point.laser_id] = frameAdvancement;

    return frameAdvancement;
  }

private:
  double InitAdvancement;
  std::map<int, double> PreviousAdvancementPerRing;
};

}  // end of namespace Utils
}  // end of namespace lidar_conversions
