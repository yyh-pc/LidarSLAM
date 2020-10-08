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

// A new PCL Point is added so we need to recompile PCL to be able to use
// filters (pcl::VoxelGrid) with this new type
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/filters/voxel_grid.h>

namespace
{
  template<typename T>
  RollingGrid::Grid3D<T> InitGrid3D(unsigned int size, T defaultValue = T())
  {
    return RollingGrid::Grid3D<T>(size,
                                  std::vector<std::vector<T>>(size,
                                                              std::vector<T>(size,
                                                                             defaultValue)));
  }
}

//==============================================================================
//   Initialization and parameters setters
//==============================================================================

//------------------------------------------------------------------------------
RollingGrid::RollingGrid(const Eigen::Vector3d& position)
{
  // Create rolling grid
  this->Grid = InitGrid3D<PointCloud::Ptr>(this->GridSize);

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

  // Reset min and max points of current frame
  double halfGridSize = std::ceil(this->GridSize / 2) * this->VoxelResolution;
  this->MinPoint = this->VoxelGridPosition - halfGridSize;
  this->MaxPoint = this->VoxelGridPosition + halfGridSize;
}

//------------------------------------------------------------------------------
void RollingGrid::Clear()
{
  for (int x = 0; x < this->GridSize; x++)
    for (int y = 0; y < this->GridSize; y++)
      for (int z = 0; z < this->GridSize; z++)
      {
        // If voxel is not already initialized, allocate memory.
        // Otherwise, just clear data without freeing dedicating memory for faster processing.
        auto& voxel = this->Grid[x][y][z];
        if (voxel)
          voxel->clear();
        else
          voxel.reset(new PointCloud);
      }
}

//------------------------------------------------------------------------------
void RollingGrid::SetGridSize(int size)
{
  PointCloud::Ptr prevMap = this->Get();

  // Resize voxel grid
  this->GridSize = size;
  this->Grid = InitGrid3D<PointCloud::Ptr>(this->GridSize);
  // Clear current voxel grid and allocate new voxels
  this->Clear();

  // Add points back so that they now lie in the right voxel
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
  this->Add(prevMap);
}

//==============================================================================
//   Main use
//==============================================================================

//------------------------------------------------------------------------------
void RollingGrid::SetMinMaxPoints(const Eigen::Array3d& minPoint, const Eigen::Array3d& maxPoint)
{
  this->MinPoint = minPoint;
  this->MaxPoint = maxPoint;
}

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get(const Eigen::Vector3d& position) const
{
  // Compute the position of the new frame center in the grid
  Eigen::Array3i frameCenter = this->PositionToVoxel(position.array()) - this->PositionToVoxel(this->VoxelGridPosition) + this->GridSize / 2;

  // Get sub-VoxelGrid bounds
  Eigen::Array3i intersectionMin = (frameCenter + this->PositionToVoxel(this->MinPoint)).max(0);
  Eigen::Array3i intersectionMax = (frameCenter + this->PositionToVoxel(this->MaxPoint)).min(this->GridSize - 1);

  // Get all voxel in intersection
  PointCloud::Ptr intersection(new PointCloud);
  for (int x = intersectionMin.x(); x <= intersectionMax.x(); x++)
    for (int y = intersectionMin.y(); y <= intersectionMax.y(); y++)
      for (int z = intersectionMin.z(); z <= intersectionMax.z(); z++)
        *intersection += *(this->Grid[x][y][z]);

  return intersection;
}

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get() const
{
  // Merge all points into a single pointcloud
  PointCloud::Ptr intersection(new PointCloud);
  for (int x = 0; x < this->GridSize; x++)
    for (int y = 0; y < this->GridSize; y++)
      for (int z = 0; z < this->GridSize; z++)
        *intersection += *(this->Grid[x][y][z]);

  return intersection;
}

//------------------------------------------------------------------------------
void RollingGrid::Roll(const Eigen::Vector3d& position)
{
  // Very basic implementation where the grid is not circular.
  // This only moves VoxelGrid so that current frame can entirely fit in rolled map.

  // Compute how much the new frame does not fit in current grid
  double halfGridSize = static_cast<double>(this->GridSize) / 2 * this->VoxelResolution;
  Eigen::Array3d downOffset = (position.array() + MinPoint) - (VoxelGridPosition - halfGridSize);
  Eigen::Array3d upOffset   = (position.array() + MaxPoint) - (VoxelGridPosition + halfGridSize);
  Eigen::Array3d offset = (upOffset + downOffset) / 2;

  // Clamp the rolling movement so that it only moves what is really necessary
  offset = offset.max(downOffset.min(0)).min(upOffset.max(0));
  Eigen::Array3d voxelsOffset = (offset / this->VoxelResolution).round();

  // Update new rolling grid position
  this->VoxelGridPosition += voxelsOffset * this->VoxelResolution;

  // Shift the voxel grid to the -X direction.
  while (voxelsOffset.x() < 0)
  {
    for (int y = 0; y < this->GridSize; y++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int x = this->GridSize - 1; x > 0; x--)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x - 1][y][z]);
        }
        this->Grid[0][y][z].reset(new PointCloud);
      }
    }
    voxelsOffset.x()++;
  }

  // Shift the voxel grid to the +X direction.
  while (voxelsOffset.x() > 0)
  {
    for (int y = 0; y < this->GridSize; y++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int x = 0; x < this->GridSize - 1; x++)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x + 1][y][z]);
        }
        this->Grid[this->GridSize - 1][y][z].reset(new PointCloud);
      }
    }
    voxelsOffset.x()--;
  }

  // Shift the voxel grid to the -Y direction.
  while (voxelsOffset.y() < 0)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int y = this->GridSize - 1; y > 0; y--)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y - 1][z]);
        }
        this->Grid[x][0][z].reset(new PointCloud);
      }
    }
    voxelsOffset.y()++;
  }

  // Shift the voxel grid to the +Y direction.
  while (voxelsOffset.y() > 0)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int y = 0; y < this->GridSize - 1; y++)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y + 1][z]);
        }
        this->Grid[x][this->GridSize - 1][z].reset(new PointCloud);
      }
    }
    voxelsOffset.y()--;
  }

  // Shift the voxel grid to the -Z direction.
  while (voxelsOffset.z() < 0)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int y = 0; y < this->GridSize; y++)
      {
        for (int z = this->GridSize - 1; z > 0; z--)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y][z - 1]);
        }
        this->Grid[x][y][0].reset(new PointCloud);
      }
    }
    voxelsOffset.z()++;
  }

  // Shift the voxel grid to the +Z direction.
  while (voxelsOffset.z() > 0)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int y = 0; y < this->GridSize; y++)
      {
        for (int z = 0; z < this->GridSize - 1; z++)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y][z + 1]);
        }
        this->Grid[x][y][this->GridSize - 1].reset(new PointCloud);
      }
    }
    voxelsOffset.z()--;
  }
}

//------------------------------------------------------------------------------
void RollingGrid::Add(const PointCloud::Ptr& pointcloud)
{
  if (pointcloud->empty())
  {
    std::cerr << "[WARNING] Pointcloud is empty, voxel grid not updated.\n";
    return;
  }

  // Voxels to filter because new points were added
  Grid3D<uint8_t> voxelToFilter = InitGrid3D<uint8_t>(this->GridSize, 0);

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
      voxelToFilter[cubeIdx.x()][cubeIdx.y()][cubeIdx.z()] = 1;
      this->Grid[cubeIdx.x()][cubeIdx.y()][cubeIdx.z()]->push_back(point);
    }
  }

  // Filter the modified pointCloud
  pcl::VoxelGrid<Point> downSizeFilter;
  downSizeFilter.setLeafSize(this->LeafSize, this->LeafSize, this->LeafSize);
  for (int x = 0; x < this->GridSize; x++)
  {
    for (int y = 0; y < this->GridSize; y++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        if (voxelToFilter[x][y][z])
        {
          PointCloud::Ptr tmp(new PointCloud);
          downSizeFilter.setInputCloud(this->Grid[x][y][z]);
          downSizeFilter.filter(*tmp);
          this->Grid[x][y][z] = tmp;
        }
      }
    }
  }
}