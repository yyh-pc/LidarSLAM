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

#include "LidarSlam/RollingGrid.h"
#include "LidarSlam/Utilities.h"

// A new PCL Point is added so we need to recompile PCL to be able to use
// filters (pcl::VoxelGrid) with this new type
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/filters/voxel_grid.h>

namespace LidarSlam
{

//==============================================================================
//   Initialization and parameters setters
//==============================================================================

//------------------------------------------------------------------------------
RollingGrid::RollingGrid(const Eigen::Vector3d& position)
{
  this->Reset(position);
}

//------------------------------------------------------------------------------
void RollingGrid::Reset(const Eigen::Vector3d& position)
{
  // Clear/reset empty voxel grid
  this->Clear();

  // Initialize VoxelGrid center position
  // Position is rounded down to be a multiple of resolution
  this->VoxelGridPosition = (position.array() / this->VoxelResolution).floor() * this->VoxelResolution;
}

//------------------------------------------------------------------------------
void RollingGrid::Clear()
{
  this->NbPoints = 0;
  this->Voxels.clear();
}

//------------------------------------------------------------------------------
void RollingGrid::SetGridSize(int size)
{
  // Resize voxel grid
  this->GridSize = size;

  // Clear current voxel grid and fill it back with points so that they now lie
  // in the right voxel
  PointCloud::Ptr prevMap = this->Get();
  this->Clear();
  if (!prevMap->empty())
    this->Add(prevMap);
}

//------------------------------------------------------------------------------
void RollingGrid::SetVoxelResolution(double resolution)
{
  this->VoxelResolution = resolution;

  // Round down VoxelGrid center position to be a multiple of resolution
  this->VoxelGridPosition = (this->VoxelGridPosition / this->VoxelResolution).floor() * this->VoxelResolution;

  // Move points so that they now lie in the right voxel
  PointCloud::Ptr prevMap = this->Get();
  this->Clear();
  if (!prevMap->empty())
    this->Add(prevMap);
}

//==============================================================================
//   Main use
//==============================================================================

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get(const Eigen::Array3d& minPoint, const Eigen::Array3d& maxPoint) const
{
  // Compute the position of the origin cell (0, 0, 0) of the grid
  Eigen::Array3i voxelGridOrigin = this->PositionToVoxel(this->VoxelGridPosition) - this->GridSize / 2;

  // Get sub-VoxelGrid bounds
  Eigen::Array3i intersectionMin = (this->PositionToVoxel(minPoint) - voxelGridOrigin).max(0);
  Eigen::Array3i intersectionMax = (this->PositionToVoxel(maxPoint) - voxelGridOrigin).min(this->GridSize - 1);

  // Get all voxel in intersection
  PointCloud::Ptr intersection(new PointCloud);
  for (const auto& kv : this->Voxels)
  {
    // Add points if the voxel lies within bounds
    Eigen::Array3i idx3d = this->To3d(kv.first);
    if (((intersectionMin <= idx3d) && (idx3d <= intersectionMax)).all())
      *intersection += *(kv.second);
  }

  return intersection;
}

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get() const
{
  // Merge all points into a single pointcloud
  PointCloud::Ptr intersection(new PointCloud);
  intersection->reserve(this->NbPoints);
  for (const auto& kv : this->Voxels)
    *intersection += *(kv.second);

  return intersection;
}

//------------------------------------------------------------------------------
void RollingGrid::Roll(const Eigen::Array3d& minPoint, const Eigen::Array3d& maxPoint)
{
  // Very basic implementation where the grid is not circular.
  // This only moves VoxelGrid so that the given bounding box can entirely fit in rolled map.

  // Compute how much the new frame does not fit in current grid
  double halfGridSize = static_cast<double>(this->GridSize) / 2 * this->VoxelResolution;
  Eigen::Array3d downOffset = minPoint - (VoxelGridPosition - halfGridSize);
  Eigen::Array3d upOffset   = maxPoint - (VoxelGridPosition + halfGridSize);
  Eigen::Array3d offset = (upOffset + downOffset) / 2;

  // Clamp the rolling movement so that it only moves what is really necessary
  offset = offset.max(downOffset.min(0)).min(upOffset.max(0));
  Eigen::Array3i voxelsOffset = (offset / this->VoxelResolution).round().cast<int>();

  // Exit if there is no need to roll
  if ((voxelsOffset == 0).all())
    return;

  // Fill new voxel grid
  unsigned int newNbPoints = 0;
  std::unordered_map<int, PointCloud::Ptr> newVoxels;
  for (const auto& kv : this->Voxels)
  {
    // Compute new voxel position
    Eigen::Array3i newIdx3d = this->To3d(kv.first) - voxelsOffset;

    // Move voxel and keep it only if it lies within bounds
    if (((0 <= newIdx3d) && (newIdx3d < this->GridSize)).all())
    {
      int newIdx1d = this->To1d(newIdx3d);
      newNbPoints += kv.second->size();
      newVoxels[newIdx1d] = std::move(kv.second);
    }
  }

  // Update the voxel grid
  this->NbPoints = newNbPoints;
  this->Voxels.swap(newVoxels);
  this->VoxelGridPosition += voxelsOffset.cast<double>() * this->VoxelResolution;
}

//------------------------------------------------------------------------------
void RollingGrid::Add(const PointCloud::Ptr& pointcloud, bool roll)
{
  if (pointcloud->empty())
  {
    PRINT_WARNING("Pointcloud is empty, voxel grid not updated.");
    return;
  }

  // Optionally roll the map so that all new points can fit in rolled map
  if (roll)
  {
    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*pointcloud, minPoint, maxPoint);
    this->Roll(minPoint.head<3>().cast<double>().array(), maxPoint.head<3>().cast<double>().array());
  }

  // Number of points added in each voxel
  std::unordered_map<int, unsigned int> addedPoints;

  // Compute the "position" of the lowest cell of the VoxelGrid in voxels dimensions
  Eigen::Array3i voxelGridOrigin = this->PositionToVoxel(this->VoxelGridPosition) - this->GridSize / 2;

  // Add points in the rolling grid
  for (const Point& point : *pointcloud)
  {
    // Find the voxel containing this point
    Eigen::Array3i cubeIdx = this->PositionToVoxel(point.getArray3fMap()) - voxelGridOrigin;

    // Add point to grid if it is indeed within bounds
    if (((0 <= cubeIdx) && (cubeIdx < this->GridSize)).all())
    {
      int id1d = this->To1d(cubeIdx);

      auto& voxel = this->Voxels[id1d];
      if (!voxel)
        voxel.reset(new PointCloud);
      voxel->push_back(point);

      addedPoints[id1d] += 1;
    }
  }

  // Filter the modified voxels
  // All the points belonging to the same voxel will be approximated
  // (i.e., downsampled) with their centroid. The mean operator is applied to
  // each field (X, Y, Z, intensity, time, ...).
  pcl::VoxelGrid<Point> downSizeFilter;
  downSizeFilter.setLeafSize(this->LeafSize, this->LeafSize, this->LeafSize);
  for (const auto& kv : addedPoints)
  {
    // Number of points in the voxel before filtering
    auto& voxel = this->Voxels[kv.first];
    unsigned int voxelPrevSize = voxel->size() - kv.second;
    // Downsample the current voxel
    downSizeFilter.setInputCloud(voxel);
    downSizeFilter.filter(*voxel);
    // Update the voxel's number of points
    this->NbPoints += voxel->size() - voxelPrevSize;
  }
}

//==============================================================================
//   Helpers
//==============================================================================

//------------------------------------------------------------------------------
int RollingGrid::To1d(const Eigen::Array3i& voxelId3d) const
{
  return voxelId3d.z() * this->GridSize * this->GridSize + voxelId3d.y() * this->GridSize + voxelId3d.x();
}

//------------------------------------------------------------------------------
Eigen::Array3i RollingGrid::To3d(int voxelId1d) const
{
  int z = voxelId1d / (this->GridSize * this->GridSize);
  voxelId1d -= z * this->GridSize * this->GridSize;
  int y = voxelId1d / this->GridSize;
  voxelId1d -= y * this->GridSize;
  int x = voxelId1d;
  return {x, y, z};
}

} // end of LidarSlam namespace