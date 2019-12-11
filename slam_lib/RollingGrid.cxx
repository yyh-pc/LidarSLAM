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
  // Init empty grid of right size
  this->SetGridSize(this->GridSize);
}

//------------------------------------------------------------------------------
RollingGrid::RollingGrid(double posX, double posY, double posZ)
{
  // Init empty grid of right size
  this->SetGridSize(this->GridSize);
  // should initialize using Tworld + size / 2
  this->VoxelGridPosition[0] = static_cast<int>(posX);
  this->VoxelGridPosition[1] = static_cast<int>(posY);
  this->VoxelGridPosition[2] = static_cast<int>(posZ);
}

//------------------------------------------------------------------------------
void RollingGrid::Clear()
{
  for (int x = 0; x < this->GridSize; x++)
    for (int y = 0; y < this->GridSize; y++)
      for (int z = 0; z < this->GridSize; z++)
        this->Grid[x][y][z]->clear();
}

//------------------------------------------------------------------------------
void RollingGrid::Roll(const Eigen::Matrix<double, 6, 1>& T)
{
  // Very basic implementation where the grid is not circular

  // compute the position of the new frame center in the grid
  int frameCenterX = std::floor(T[3] / this->GridSize) + this->VoxelGridPosition[0];
  int frameCenterY = std::floor(T[4] / this->GridSize) + this->VoxelGridPosition[1];
  int frameCenterZ = std::floor(T[5] / this->GridSize) + this->VoxelGridPosition[2];

  // shift the voxel grid to the left
  while (frameCenterX - std::ceil(this->PointCloudSize / 2) <= 0)
  {
    for (int y = 0; y < this->GridSize; y++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int x = this->GridSize - 1; x > 0; x--)
        {
          this->Grid[x][y][z] = this->Grid[x - 1][y][z];
        }
        this->Grid[0][y][z].reset(new PointCloud());
      }
    }
    frameCenterX++;
    this->VoxelGridPosition[0]++;
  }

  // shift the voxel grid to the right
  while (frameCenterX + std::ceil(this->PointCloudSize / 2) >= this->GridSize - 1)
  {
    for (int y = 0; y < this->GridSize; y++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int x = 0; x < this->GridSize - 1; x++)
        {
          this->Grid[x][y][z] = this->Grid[x + 1][y][z];
        }
        this->Grid[this->GridSize - 1][y][z].reset(new PointCloud());
      }
    }
    frameCenterX--;
    this->VoxelGridPosition[0]--;
  }

  // shift the voxel grid to the bottom
  while (frameCenterY - std::ceil(this->PointCloudSize / 2) <= 0)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int y = this->GridSize - 1; y > 0; y--)
        {
          this->Grid[x][y][z] = this->Grid[x][y - 1][z];
        }
        this->Grid[x][0][z].reset(new PointCloud());
      }
    }
    frameCenterY++;
    this->VoxelGridPosition[1]++;
  }

  // shift the voxel grid to the top
  while (frameCenterY + std::ceil(this->PointCloudSize / 2) >= this->GridSize - 1)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int z = 0; z < this->GridSize; z++)
      {
        for (int y = 0; y < this->GridSize - 1; y++)
        {
          this->Grid[x][y][z] = this->Grid[x][y + 1][z];
        }
        this->Grid[x][this->GridSize - 1][z].reset(new PointCloud());
      }
    }
    frameCenterY--;
    this->VoxelGridPosition[1]--;
  }

  // shift the voxel grid to the "camera"
  while (frameCenterZ - std::ceil(this->PointCloudSize / 2) <= 0)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int y = 0; y < this->GridSize; y++)
      {
        for (int z = this->GridSize - 1; z > 0; z--)
        {
          this->Grid[x][y][z] = this->Grid[x][y][z - 1];
        }
        this->Grid[x][y][0].reset(new PointCloud());
      }
    }
    frameCenterZ++;
    this->VoxelGridPosition[2]++;
  }

  // shift the voxel grid to the "horizon"
  while (frameCenterZ + std::ceil(this->PointCloudSize / 2) >= this->GridSize - 1)
  {
    for (int x = 0; x < this->GridSize; x++)
    {
      for (int y = 0; y < this->GridSize; y++)
      {
        for (int z = 0; z < this->GridSize - 1; z++)
        {
          this->Grid[x][y][z] = this->Grid[x][y][z + 1];
        }
        this->Grid[x][y][this->GridSize - 1].reset(new PointCloud());
      }
    }
    frameCenterZ--;
    this->VoxelGridPosition[2]--;
  }
}

//------------------------------------------------------------------------------
RollingGrid::PointCloud::Ptr RollingGrid::Get(const Eigen::Matrix<double, 6, 1>& T)
{
  // compute the position of the new frame center in the grid
  int frameCenterX = std::floor(T[3] / this->GridSize) + this->VoxelGridPosition[0];
  int frameCenterY = std::floor(T[4] / this->GridSize) + this->VoxelGridPosition[1];
  int frameCenterZ = std::floor(T[5] / this->GridSize) + this->VoxelGridPosition[2];

  // Get sub-VoxelGrid bounds
  int minX = std::max<int>(frameCenterX - std::ceil(this->PointCloudSize / 2), 0);
  int maxX = std::min<int>(frameCenterX + std::ceil(this->PointCloudSize / 2), this->GridSize - 1);
  int minY = std::max<int>(frameCenterY - std::ceil(this->PointCloudSize / 2), 0);
  int maxY = std::min<int>(frameCenterY + std::ceil(this->PointCloudSize / 2), this->GridSize - 1);
  int minZ = std::max<int>(frameCenterZ - std::ceil(this->PointCloudSize / 2), 0);
  int maxZ = std::min<int>(frameCenterZ + std::ceil(this->PointCloudSize / 2), this->GridSize - 1);

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
    std::cout << "Pointcloud is empty, voxel grid not updated." << std::endl;
    return;
  }

  // Voxels to filter because new points were added
  std::vector<std::vector<std::vector<uint8_t>>> voxelToFilter(
    this->GridSize, std::vector<std::vector<uint8_t>>(this->GridSize, std::vector<uint8_t>(this->GridSize, 0)));

  // Add points in the rolling grid
  for (const Point& point : *pointcloud)
  {
    // find the closest coordinate
    int cubeIdxX = std::floor(point.x / this->GridSize) + this->VoxelGridPosition[0];
    int cubeIdxY = std::floor(point.y / this->GridSize) + this->VoxelGridPosition[1];
    int cubeIdxZ = std::floor(point.z / this->GridSize) + this->VoxelGridPosition[2];

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
void RollingGrid::SetPointCoudMaxRange(double maxdist)
{
  this->PointCloudSize = std::ceil(2 * maxdist / this->VoxelResolution);
}

//------------------------------------------------------------------------------
void RollingGrid::SetGridSize(int size)
{
  this->GridSize = size;
  this->Grid.resize(this->GridSize);
  for (int x = 0; x < this->GridSize; x++)
  {
    this->Grid[x].resize(this->GridSize);
    for (int y = 0; y < this->GridSize; y++)
    {
      this->Grid[x][y].resize(this->GridSize);
      for (int z = 0; z < this->GridSize; z++)
      {
        this->Grid[x][y][z].reset(new PointCloud());
      }
    }
  }
}