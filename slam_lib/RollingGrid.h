#ifndef ROLLING_GRID_H
#define ROLLING_GRID_H

// A new PCL Point is added so we need to recompile PCL to be able to use
// filters (pcl::VoxelGrid) with this new type
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include "LidarPoint.h"
#include <pcl/filters/voxel_grid.h>

// The map reconstructed from the slam algorithm is stored in a voxel grid
// which split the space in differents region. From this voxel grid it is possible
// to only load the parts of the map which are pertinents when we run the mapping
// optimization algorithm. Morevover, when a a region of the space is too far from
// the current sensor position it is possible to remove the points stored in this region
// and to move the voxel grid in a closest region of the sensor position. This is used
// to decrease the memory used by the algorithm
class RollingGrid
{
public:

  // Usefull types
  using Point = PointXYZTIId;
  using PointCloud = pcl::PointCloud<Point>;

  RollingGrid() = default;

  RollingGrid(double posX, double posY, double posZ);

  // roll the grid to enable adding new point cloud
  void Roll(Eigen::Matrix<double, 6, 1>& T);

  // get points arround T
  PointCloud::Ptr Get(Eigen::Matrix<double, 6, 1>& T);

  // get all points
  PointCloud::Ptr Get();

  // add some points to the grid
  void Add(PointCloud::Ptr pointcloud);

  void SetPointCoudMaxRange(const double maxdist);

  void SetSize(int size);

  void SetResolution(double resolution) { this->VoxelResolution = resolution; }

  void SetLeafSize(double size) { this->LeafSize = size; }

private:

  //! Size of the voxel grid: n*n*n voxels
  int VoxelSize = 50;

  //! Resolution of a voxel
  double VoxelResolution = 10;

  //! Size of a pointcloud in voxel
  int PointCloudSize = 25;

  //! Size of the leaf use to downsample the pointcloud
  double LeafSize = 0.2;

  //! VoxelGrid of pointcloud
  std::vector<std::vector<std::vector<PointCloud::Ptr>>> grid;

  // Position of the VoxelGrid
  int VoxelGridPosition[3] = { 0, 0, 0 };
};

#endif  // ROLLING_GRID_H