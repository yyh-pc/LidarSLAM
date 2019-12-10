#include "RollingGrid.h"

//------------------------------------------------------------------------------
RollingGrid::RollingGrid(double posX, double posY, double posZ)
{
  // should initialize using Tworld + size / 2
  this->VoxelGridPosition[0] = static_cast<int>(posX);
  this->VoxelGridPosition[1] = static_cast<int>(posY);
  this->VoxelGridPosition[2] = static_cast<int>(posZ);
}

//------------------------------------------------------------------------------
// roll the grid to enable adding new point cloud
void RollingGrid::Roll(Eigen::Matrix<double, 6, 1> &T)
{
  // Very basic implementation where the grid is not circular

  // compute the position of the new frame center in the grid
  int frameCenterX = std::floor(T[3] / this->VoxelSize) - this->VoxelGridPosition[0];
  int frameCenterY = std::floor(T[4] / this->VoxelSize) - this->VoxelGridPosition[1];
  int frameCenterZ = std::floor(T[5] / this->VoxelSize) - this->VoxelGridPosition[2];

  // shift the voxel grid to the left
  while (frameCenterX - std::ceil(this->PointCloudSize / 2) <= 0)
  {
    for (int j = 0; j < this->VoxelSize; j++)
    {
      for (int k = 0; k < this->VoxelSize; k++)
      {
        for (int i = this->VoxelSize - 1; i > 0; i--)
        {
          this->grid[i][j][k] = this->grid[i-1][j][k];
        }
        this->grid[0][j][k].reset(new PointCloud());
      }
    }
    frameCenterX++;
    this->VoxelGridPosition[0]--;
  }

  // shift the voxel grid to the right
  while (frameCenterX + std::ceil(this->PointCloudSize / 2) >= this->VoxelSize - 1)
  {
    for (int j = 0; j < this->VoxelSize; j++)
    {
      for (int k = 0; k < this->VoxelSize; k++)
      {
        for (int i = 0; i < this->VoxelSize - 1; i++)
        {
          this->grid[i][j][k] = this->grid[i+1][j][k];
        }
        this->grid[VoxelSize-1][j][k].reset(new PointCloud());
      }
    }
    frameCenterX--;
    this->VoxelGridPosition[0]++;
  }

  // shift the voxel grid to the bottom
  while (frameCenterY - std::ceil(this->PointCloudSize / 2) <= 0)
  {
    for (int i = 0; i < this->VoxelSize; i++)
    {
      for (int k = 0; k < this->VoxelSize; k++)
      {
        for (int j = this->VoxelSize - 1; j > 0; j--)
        {
          this->grid[i][j][k] = this->grid[i][j-1][k];
        }
        this->grid[i][0][k].reset(new PointCloud());
      }
    }
    frameCenterY++;
    this->VoxelGridPosition[1]--;
  }

  // shift the voxel grid to the top
  while (frameCenterY + std::ceil(this->PointCloudSize / 2) >= this->VoxelSize - 1)
  {
    for (int i = 0; i < this->VoxelSize; i++)
    {
      for (int k = 0; k < this->VoxelSize; k++)
      {
        for (int j = 0; j < this->VoxelSize - 1; j++)
        {
          this->grid[i][j][k] = this->grid[i][j+1][k];
        }
        this->grid[i][VoxelSize-1][k].reset(new PointCloud());
      }
    }
    frameCenterY--;
    this->VoxelGridPosition[1]++;
  }

  // shift the voxel grid to the "camera"
  while (frameCenterZ - std::ceil(this->PointCloudSize / 2) <= 0)
  {
    for (int i = 0; i < this->VoxelSize; i++)
    {
      for (int j = 0; j < this->VoxelSize; j++)
      {
        for (int k = this->VoxelSize - 1; k > 0; k--)
        {
          this->grid[i][j][k] = this->grid[i][j][k-1];
        }
        this->grid[i][j][0].reset(new PointCloud());
      }
    }
    frameCenterZ++;
    this->VoxelGridPosition[2]--;
  }

  // shift the voxel grid to the "horizon"
  while (frameCenterZ + std::ceil(this->PointCloudSize  / 2) >= this->VoxelSize - 1)
  {
    for (int i = 0; i < this->VoxelSize; i++)
    {
      for (int j = 0; j < this->VoxelSize; j++)
      {
        for (int k = 0; k < this->VoxelSize - 1; k++)
        {
          this->grid[i][j][k] = this->grid[i][j][k+1];
        }
        this->grid[i][j][VoxelSize-1].reset(new PointCloud());
      }
    }
    frameCenterZ--;
    this->VoxelGridPosition[2]++;
  }
}

//------------------------------------------------------------------------------
// get points arround T
RollingGrid::PointCloud::Ptr RollingGrid::Get(Eigen::Matrix<double, 6, 1> &T)
{
  // compute the position of the new frame center in the grid
  int frameCenterX = std::floor(T[3] / this->VoxelSize) - this->VoxelGridPosition[0];
  int frameCenterY = std::floor(T[4] / this->VoxelSize) - this->VoxelGridPosition[1];
  int frameCenterZ = std::floor(T[5] / this->VoxelSize) - this->VoxelGridPosition[2];

  PointCloud::Ptr intersection(new PointCloud);

  // Get all voxel in intersection should use ceil here
  for (int i = frameCenterX - std::ceil(this->PointCloudSize / 2); i <= frameCenterX + std::ceil(this->PointCloudSize / 2); i++)
  {
    for (int j = frameCenterY - std::ceil(this->PointCloudSize / 2); j <= frameCenterY + std::ceil(this->PointCloudSize / 2); j++)
    {
      for (int k = frameCenterZ - std::ceil(this->PointCloudSize / 2); k <= frameCenterZ + std::ceil(this->PointCloudSize / 2); k++)
      {
        if (i < 0 || i > (this->VoxelSize - 1) ||
            j < 0 || j > (this->VoxelSize - 1) ||
            k < 0 || k > (this->VoxelSize - 1))
        {
          continue;
        }
        PointCloud::Ptr voxel = this->grid[i][j][k];
        for (unsigned int l = 0; l < voxel->size(); l++)
        {
          intersection->push_back(voxel->at(l));
        }
      }
    }
  }
  return intersection;
}

//------------------------------------------------------------------------------
// get all points
RollingGrid::PointCloud::Ptr RollingGrid::Get()
{
  PointCloud::Ptr intersection(new PointCloud);

  // Get all voxel in intersection should use ceil here
  for (int i = 0; i < VoxelSize; i++)
  {
    for (int j = 0; j < VoxelSize; j++)
    {
      for (int k = 0; k < VoxelSize; k++)
      {
        PointCloud::Ptr voxel = this->grid[i][j][k];
        for (unsigned int l = 0; l < voxel->size(); l++)
        {
          intersection->push_back(voxel->at(l));
        }
      }
    }
  }
  return intersection;
}

//------------------------------------------------------------------------------
// add some points to the grid
void RollingGrid::Add(PointCloud::Ptr pointcloud)
{
  if (pointcloud->size() == 0)
  {
    std::cout << "Pointcloud empty, voxel grid not updated" << std::endl;
    return;
  }

  // Voxel to filter because new points were added
  std::vector<std::vector<std::vector<int> > > voxelToFilter(VoxelSize, std::vector<std::vector<int> >(VoxelSize, std::vector<int>(VoxelSize, 0)));

  // Add points in the rolling grid
  int outlier = 0; // point who are not in the rolling grid
  for (unsigned int i = 0; i < pointcloud->size(); i++)
  {
    Point& pts = pointcloud->points[i];
    // find the closest coordinate
    int cubeIdxX = std::floor(pts.x / this->VoxelSize) - this->VoxelGridPosition[0];
    int cubeIdxY = std::floor(pts.y / this->VoxelSize) - this->VoxelGridPosition[1];
    int cubeIdxZ = std::floor(pts.z / this->VoxelSize) - this->VoxelGridPosition[2];

    if (cubeIdxX >= 0 && cubeIdxX < this->VoxelSize &&
      cubeIdxY >= 0 && cubeIdxY < this->VoxelSize &&
      cubeIdxZ >= 0 && cubeIdxZ < this->VoxelSize)
    {
      voxelToFilter[cubeIdxX][cubeIdxY][cubeIdxZ] = 1;
      grid[cubeIdxX][cubeIdxY][cubeIdxZ]->push_back(pts);
    }
    else
    {
      outlier++;
    }
  }

  // Filter the modified pointCloud
  pcl::VoxelGrid<Point> downSizeFilter;
  downSizeFilter.setLeafSize(this->LeafSize, this->LeafSize, this->LeafSize);
  for (int i = 0; i < this->VoxelSize; i++)
  {
    for (int j = 0; j < this->VoxelSize; j++)
    {
      for (int k = 0; k < this->VoxelSize; k++)
      {
        if (voxelToFilter[i][j][k] == 1)
        {
          PointCloud::Ptr tmp(new PointCloud());
          downSizeFilter.setInputCloud(grid[i][j][k]);
          downSizeFilter.filter(*tmp);
          grid[i][j][k] = tmp;
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void RollingGrid::SetPointCoudMaxRange(const double maxdist)
{
  this->PointCloudSize = std::ceil(2 * maxdist / this->VoxelResolution);
}

//------------------------------------------------------------------------------
void RollingGrid::SetSize(int size)
{
  this->VoxelSize = size;
  grid.resize(this->VoxelSize);
  for (int i = 0; i < this->VoxelSize; i++)
  {
    grid[i].resize(this->VoxelSize);
    for (int j = 0; j < this->VoxelSize; j++)
    {
      grid[i][j].resize(this->VoxelSize);
      for (int k = 0; k < this->VoxelSize; k++)
      {
        grid[i][j][k].reset(new PointCloud());
      }
    }
  }
}