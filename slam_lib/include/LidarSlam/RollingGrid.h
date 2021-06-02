//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2019-12-13
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

#include "LidarSlam/LidarPoint.h"

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

/*!
 * @brief Rolling voxel grid to store and access pointclouds of specific areas.
 *
 * The map reconstructed from the SLAM algorithm is stored in a voxel grid
 * which splits the space in different regions. From this voxel grid, it is
 * possible to only load the parts of the map which are pertinent when we run
 * the localization optimization step. Morevover, when a region of the space is
 * too far from the current sensor position, it is possible to remove the points
 * stored in this region and to move the voxel grid in a closest region of the
 * sensor position. This is used to decrease the memory used by the algorithm.
 */
class RollingGrid
{
public:

  // Usefull types
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;

  template<typename T>
  using Grid3D = std::vector<std::vector<std::vector<T>>>;

  //============================================================================
  //   Initialization and parameters setters
  //============================================================================

  //! Init a Rolling grid centered near a given position
  RollingGrid(const Eigen::Vector3d& position = Eigen::Vector3d::Zero());

  //! Reset map (clear voxels, reset position, ...)
  void Reset(const Eigen::Vector3d& position = Eigen::Vector3d::Zero());

  //! Remove all points from all voxels
  void Clear();

  //! Set grid size (number of voxels in each direction)
  //! NOTE: this may remove some points from the grid if size is decreased
  void SetGridSize(int size);
  GetMacro(GridSize, int)

  //! Set voxel resolution (resolution of each voxel, in meters)
  //! NOTE: this may remove some points from the grid if resolution is decreased
  void SetVoxelResolution(double resolution);
  GetMacro(VoxelResolution, double)

  SetMacro(LeafSize, double)
  GetMacro(LeafSize, double)

  //============================================================================
  //   Main use
  //============================================================================

  //! Extract all points in map lying in given bounding box
  PointCloud::Ptr Get(const Eigen::Array3d& minPoint, const Eigen::Array3d& maxPoint) const;

  //! Get all points
  PointCloud::Ptr Get() const;

  //! Get the current number of points in rolling grid
  unsigned int Size() const {return this->NbPoints; }

  //! Roll the grid so that input bounding box can fit it in rolled map
  void Roll(const Eigen::Array3d& minPoint, const Eigen::Array3d& maxPoint);

  //! Add some points to the grid
  //! If roll is true, the map is rolled first so that all new points to add can fit in rolled map.
  void Add(const PointCloud::Ptr& pointcloud, bool roll = true);

  //============================================================================
  //   Attributes and helper methods
  //============================================================================

private:

  //! [voxels] Size of the voxel grid: n*n*n voxels
  int GridSize = 50;

  //! [m/voxel] Resolution of a voxel
  double VoxelResolution = 10.;

  //! [m] Size of the leaf used to downsample the pointcloud with a VoxelGrid filter within each voxel
  double LeafSize = 0.2;

  //! VoxelGrid of pointcloud
  Grid3D<PointCloud::Ptr> Grid;

  //! [m, m, m] Current position of the center of the VoxelGrid
  Eigen::Array3d VoxelGridPosition;

  //! Current number of points stored in the rolling grid
  unsigned int NbPoints;

private:

  //! Compute the voxel coordinates in which fits a point
  template<typename T>
  inline Eigen::Array3i PositionToVoxel(const T& position) const
  {
    return (position / this->VoxelResolution).array().floor().template cast<int>();
  }
};

} // end of LidarSlam namespace