#ifndef ROLLING_GRID_H
#define ROLLING_GRID_H

#include "LidarPoint.h"

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

/*!
 * @brief Rolling voxel grid to store and access pointclouds of specific areas.
 *
 * The map reconstructed from the SLAM algorithm is stored in a voxel grid
 * which splits the space in different regions. From this voxel grid, it is
 * possible to only load the parts of the map which are pertinent when we run
 * the mapping optimization algorithm. Morevover, when a region of the space is
 * too far from the current sensor position, it is possible to remove the points
 * stored in this region and to move the voxel grid in a closest region of the
 * sensor position. This is used to decrease the memory used by the algorithm.
 */
class RollingGrid
{
public:

  // Usefull types
  using Point = PointXYZTIId;
  using PointCloud = pcl::PointCloud<Point>;

  RollingGrid();

  RollingGrid(double posX, double posY, double posZ);

  //! Roll the grid to enable adding new point cloud
  void Roll(const Eigen::Vector3d& T);

  //! Get points near T
  PointCloud::Ptr Get(const Eigen::Vector3d& T);

  //! Get all points
  PointCloud::Ptr Get();

  //! Add some points to the grid
  void Add(const PointCloud::Ptr& pointcloud);

  //! Remove all points from all voxels
  void Clear();

  //! Reset map (clear voxels, reset position, ...)
  void Reset();

  //! Set min and max keypoints bounds of current frame
  void SetMinMaxPoints(const Eigen::Vector3d& minPoint, const Eigen::Vector3d& maxPoint);

  //! Set grid size and clear all points from voxels
  void SetGridSize(int size);
  GetMacro(GridSize, int)

  SetMacro(VoxelResolution, double)
  GetMacro(VoxelResolution, double)

  SetMacro(LeafSize, double)
  GetMacro(LeafSize, double)

private:

  //! [voxels] Size of the voxel grid: n*n*n voxels
  int GridSize = 50;

  //! [m/voxel] Resolution of a voxel
  double VoxelResolution = 10.;

  //! [voxels] Minimum and maximum keypoints coordinates
  int MinPoint[3], MaxPoint[3];

  //! [m] Size of the leaf used to downsample the pointcloud with a VoxelGrid filter within each voxel
  double LeafSize = 0.2;

  //! VoxelGrid of pointcloud
  std::vector<std::vector<std::vector<PointCloud::Ptr>>> Grid;

  //! [voxel, voxel, voxel] Current position of the center of the VoxelGrid
  int VoxelGridPosition[3] = { 0, 0, 0 };
};

#endif  // ROLLING_GRID_H