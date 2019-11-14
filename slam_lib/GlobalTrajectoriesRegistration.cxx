#include "GlobalTrajectoriesRegistration.h"

//------------------------------------------------------------------------------
bool GlobalTrajectoriesRegistration::ComputeTransformOffset(const std::vector<Transform>& fromPoses,
                                                            const std::vector<Transform>& toPoses,
                                                            Eigen::Isometry3d& offset)
{
  // Convert to PCL pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& pose: fromPoses)
    fromCloud->push_back(pcl::PointXYZ(pose.x, pose.y, pose.z));
  pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& pose: toPoses)
    toCloud->push_back(pcl::PointXYZ(pose.x, pose.y, pose.z));

  // Compute rough transormation to get better initialization
  Eigen::Translation3d translation = this->ComputeRoughTranslationOffset(fromPoses[0], toPoses[0]);
  Eigen::Quaterniond rotation = this->ComputeRoughRotationOffset(fromPoses[0], fromPoses.back(), toPoses[0], toPoses.back());
  Eigen::Isometry3d roughTransform(rotation * translation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr roughOptimCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*fromCloud, *roughOptimCloud, roughTransform.matrix());

  // Run ICP for transform refinement
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
  icp.setMaximumIterations(this->NbrIcpIterations);
  icp.setInputSource(roughOptimCloud);
  icp.setInputTarget(toCloud);
  pcl::PointCloud<pcl::PointXYZ> optimCloud;
  icp.align(optimCloud);
  Eigen::Isometry3d fineTransform(icp.getFinalTransformation());

  // Compose rough and precise transforms
  offset = fineTransform * roughTransform;

  if (this->Verbose)
  {
    std::cout << "ICP has converged:" << icp.hasConverged()
              << "\nICP score: " << icp.getFitnessScore()
              << "\nRough transform:\n" << roughTransform.matrix()
              << "\nICP transform:\n" << fineTransform.matrix()
              << "\nFinal transform:\n" << offset.matrix()
              << std::endl;
  }

  return icp.hasConverged();
}

//------------------------------------------------------------------------------
Eigen::Translation3d GlobalTrajectoriesRegistration::ComputeRoughTranslationOffset(const Transform& fromPose,
                                                                                   const Transform& toPose)
{
  return Eigen::Translation3d(toPose.x - fromPose.x,
                              toPose.y - fromPose.y,
                              toPose.z - fromPose.z);
}

//------------------------------------------------------------------------------
Eigen::Quaterniond GlobalTrajectoriesRegistration::ComputeRoughRotationOffset(const Transform& fromPose1,
                                                                              const Transform& fromPose2,
                                                                              const Transform& toPose1,
                                                                              const Transform& toPose2)
{
  // Get fromPose direction
  Eigen::Vector3d fromDirection(fromPose2.x - fromPose1.x,
                                fromPose2.y - fromPose1.y,
                                fromPose2.z - fromPose1.z);

  // Get approximate GPS trajectory direction
  Eigen::Vector3d toDirection(toPose2.x - toPose1.x,
                              toPose2.y - toPose1.y,
                              toPose2.z - toPose1.z);

  // Compute orientation alignment between two sub-trajectories
  return Eigen::Quaterniond::FromTwoVectors(fromDirection, toDirection);
}