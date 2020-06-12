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

#ifndef LIDARPOINT_H
#define LIDARPOINT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct EIGEN_ALIGN16 _PointXYZTIId
{
  PCL_ADD_POINT4D // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  double time;
  std::uint8_t intensity;
  std::uint8_t laserId;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const _PointXYZTIId& p);

/** \brief A point structure representing Euclidean xyz coordinates, time,  intensity, and laserId.
  * \ingroup common
  */
struct PointXYZTIId : public _PointXYZTIId
{
  inline PointXYZTIId (const _PointXYZTIId &p)
  {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
    intensity = p.intensity;
    laserId = p.laserId;
  }

  inline PointXYZTIId ()
  {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    data[3] = 1.0f;
    time = 0.0;
    intensity = 0;
    laserId = 0;
  }

  friend std::ostream& operator << (std::ostream& os, const PointXYZTIId& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZTIId,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (double, time, time)
                                   (std::uint8_t, intensity, intensity)
                                   (std::uint8_t, laserId, laserId)
)
#endif // LIDARPOINT_H