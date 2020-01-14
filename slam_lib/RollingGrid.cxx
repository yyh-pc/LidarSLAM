#include "RollingGrid.h"

// A new PCL Point is added so we need to recompile PCL to be able to use
// filters (pcl::VoxelGrid) with this new type
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/filters/voxel_grid.h>

//------------------------------------------------------------------------------
RollingGrid::RollingGrid()
{
  this->Reset();
}

//------------------------------------------------------------------------------
RollingGrid::RollingGrid(double posX, double posY, double posZ)
{
  this->Reset();

  // Initialize VoxelGrid center position
  this->VoxelGridPosition[0] = std::floor(posX / this->VoxelResolution);
  this->VoxelGridPosition[1] = std::floor(posY / this->VoxelResolution);
  this->VoxelGridPosition[2] = std::floor(posZ / this->VoxelResolution);
}

//------------------------------------------------------------------------------
void RollingGrid::Reset()
{
  // Clear/reset empty voxel grid to right size
  this->SetGridSize(this->GridSize);
  // Reset VoxelGrid center position
  std::fill_n(this->VoxelGridPosition, 3, 0);
  // Reset min and max points of current frame
  int halfGridSize = std::ceil(this->GridSize / 2);
  std::fill_n(this->MinPoint, 3, -halfGridSize);
  std::fill_n(this->MaxPoint, 3, halfGridSize);
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
void RollingGrid::Roll(const Eigen::Vector3d& T)
{
  // Very basic implementation where the grid is not circular.
  // This only moves VoxelGrid so that current frame can entirely fit in rolled map.

  // Compute the position of the new frame center in the grid.
  int frameCenterX = std::floor(T.x() / this->VoxelResolution);
  int frameCenterY = std::floor(T.y() / this->VoxelResolution);
  int frameCenterZ = std::floor(T.z() / this->VoxelResolution);

  // Half size of the VoxelGrid, rounded up.
  int halfGridSize = (this->GridSize + 1) / 2;

  // Shift the voxel grid to the -X direction.
  while (frameCenterX + this->MinPoint[0] < this->VoxelGridPosition[0] - halfGridSize)
  {
    for (int y = 0; y < this->GridSize; y++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int x = this->GridSize - 1; x > 0; x--)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x - 1][y][z]);
        }
        this->Grid[0][y][z].reset(new PointCloud());
      }
    }
    this->VoxelGridPosition[0]--;
  }

  // Shift the voxel grid to the +X direction.
  while (frameCenterX + this->MaxPoint[0] > this->VoxelGridPosition[0] + halfGridSize)
  {
    for (int y = 0; y < this->GridSize; y++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int x = 0; x < this->GridSize - 1; x++)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x + 1][y][z]);
        }
        this->Grid[this->GridSize - 1][y][z].reset(new PointCloud());
      }
    }
    this->VoxelGridPosition[0]++;
  }

  // Shift the voxel grid to the -Y direction.
  while (frameCenterY + this->MinPoint[1] < this->VoxelGridPosition[1] - halfGridSize)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int y = this->GridSize - 1; y > 0; y--)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y - 1][z]);
        }
        this->Grid[x][0][z].reset(new PointCloud());
      }
    }
    this->VoxelGridPosition[1]--;
  }

  // Shift the voxel grid to the +Y direction.
  while (frameCenterY + this->MaxPoint[1] > this->VoxelGridPosition[1] + halfGridSize)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int y = 0; y < this->GridSize - 1; y++)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y + 1][z]);
        }
        this->Grid[x][this->GridSize - 1][z].reset(new PointCloud());
      }
    }
    this->VoxelGridPosition[1]++;
  }

  // Shift the voxel grid to the -Z direction.
  while (frameCenterZ + this->MinPoint[2] < this->VoxelGridPosition[2] - halfGridSize)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int y = 0; y < this->GridSize; y++)
      {
        for (int z = this->GridSize - 1; z > 0; z--)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y][z - 1]);
        }
        this->Grid[x][y][0].reset(new PointCloud());
      }
    }
    this->VoxelGridPosition[2]--;
  }

  // Shift the voxel grid to the +Z direction.
  while (frameCenterZ + this->MaxPoint[2] > this->VoxelGridPosition[2] + halfGridSize)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int y = 0; y < this->GridSize; y++)
      {
        for (int z = 0; z < this->GridSize - 1; z++)
        {
          this->Grid[x][y][z] = std::move(this->Grid[x][y][z + 1]);
        }
        this->Grid[x][y][this->GridSize - 1].reset(new PointCloud());
      }
    }
    this->VoxelGridPosition[2]++;
  }
}

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get(const Eigen::Vector3d& T)
{
  // Compute the position of the new frame center in the grid
  int frameCenterX = std::floor(T.x() / this->VoxelResolution) - (this->VoxelGridPosition[0] - this->GridSize / 2);
  int frameCenterY = std::floor(T.y() / this->VoxelResolution) - (this->VoxelGridPosition[1] - this->GridSize / 2);
  int frameCenterZ = std::floor(T.z() / this->VoxelResolution) - (this->VoxelGridPosition[2] - this->GridSize / 2);

  // Get sub-VoxelGrid bounds
  int minX = std::max<int>(frameCenterX + this->MinPoint[0], 0);
  int maxX = std::min<int>(frameCenterX + this->MaxPoint[0], this->GridSize - 1);
  int minY = std::max<int>(frameCenterY + this->MinPoint[1], 0);
  int maxY = std::min<int>(frameCenterY + this->MaxPoint[1], this->GridSize - 1);
  int minZ = std::max<int>(frameCenterZ + this->MinPoint[2], 0);
  int maxZ = std::min<int>(frameCenterZ + this->MaxPoint[2], this->GridSize - 1);

  // Get all voxel in intersection
  PointCloud::Ptr intersection(new PointCloud);
  for (int x = minX; x <= maxX; x++)
    for (int y = minY; y <= maxY; y++)
      for (int z = minZ; z <= maxZ; z++)
        *intersection += *(this->Grid[x][y][z]);

  return intersection;
}

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get()
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
void RollingGrid::Add(const PointCloud::Ptr& pointcloud)
{
  if (pointcloud->empty())
  {
    std::cerr << "[WARNING] Pointcloud is empty, voxel grid not updated." << std::endl;
    return;
  }

  // Voxels to filter because new points were added
  std::vector<std::vector<std::vector<uint8_t>>> voxelToFilter(
    this->GridSize, std::vector<std::vector<uint8_t>>(this->GridSize, std::vector<uint8_t>(this->GridSize, 0)));

  // Compute the position of the origin of the VoxelGrid
  int voxelGridOriginX = (this->VoxelGridPosition[0] - this->GridSize / 2);
  int voxelGridOriginY = (this->VoxelGridPosition[1] - this->GridSize / 2);
  int voxelGridOriginZ = (this->VoxelGridPosition[2] - this->GridSize / 2);

  // Add points in the rolling grid
  for (const Point& point : *pointcloud)
  {
    // Find the voxel containing this point
    int cubeIdxX = std::floor(point.x / this->VoxelResolution) - voxelGridOriginX;
    int cubeIdxY = std::floor(point.y / this->VoxelResolution) - voxelGridOriginY;
    int cubeIdxZ = std::floor(point.z / this->VoxelResolution) - voxelGridOriginZ;

    if (0 <= cubeIdxX && cubeIdxX < this->GridSize &&
        0 <= cubeIdxY && cubeIdxY < this->GridSize &&
        0 <= cubeIdxZ && cubeIdxZ < this->GridSize)
    {
      voxelToFilter[cubeIdxX][cubeIdxY][cubeIdxZ] = 1;
      this->Grid[cubeIdxX][cubeIdxY][cubeIdxZ]->push_back(point);
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
          PointCloud::Ptr tmp(new PointCloud());
          downSizeFilter.setInputCloud(this->Grid[x][y][z]);
          downSizeFilter.filter(*tmp);
          this->Grid[x][y][z] = tmp;
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void RollingGrid::SetMinMaxPoints(const Eigen::Vector3d& minPoint, const Eigen::Vector3d& maxPoint)
{
  this->MinPoint[0] = std::floor(minPoint[0] / this->VoxelResolution);
  this->MinPoint[1] = std::floor(minPoint[1] / this->VoxelResolution);
  this->MinPoint[2] = std::floor(minPoint[2] / this->VoxelResolution);
  this->MaxPoint[0] = std::ceil(maxPoint[0] / this->VoxelResolution);
  this->MaxPoint[1] = std::ceil(maxPoint[1] / this->VoxelResolution);
  this->MaxPoint[2] = std::ceil(maxPoint[2] / this->VoxelResolution);
}

//------------------------------------------------------------------------------
void RollingGrid::SetGridSize(int size)
{
  this->GridSize = size;
  // Resize voxel grid
  this->Grid.resize(this->GridSize);
  for (int x = 0; x < this->GridSize; x++)
  {
    this->Grid[x].resize(this->GridSize);
    for (int y = 0; y < this->GridSize; y++)
      this->Grid[x][y].resize(this->GridSize);
  }
  // Clear current voxel grid and allocate new voxels
  this->Clear();
}