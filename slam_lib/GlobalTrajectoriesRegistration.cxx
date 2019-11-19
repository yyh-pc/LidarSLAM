#include "GlobalTrajectoriesRegistration.h"

namespace
{
  Eigen::Isometry3d OppositeTransform(const Eigen::Isometry3d& a2b)
  {
    Eigen::Isometry3d b2a;
    b2a.translation() = -a2b.translation();
    b2a.linear() = a2b.linear().transpose();
    return b2a;
  }
}

//------------------------------------------------------------------------------
bool GlobalTrajectoriesRegistration::ComputeTransformOffset(const std::vector<Transform>& fromPoses,
                                                            const std::vector<Transform>& toPoses,
                                                            Eigen::Isometry3d& offset)
{
  // TODO Use timestamps to filter out outliers points

  // TODO It is better (faster, more accurate and correct score estimation) to fit a
  // sparser trajectory to a denser one. As a result, if 'toPoses' is sparser
  // than 'fromPoses', swap source and target for ICP.

  // Convert to PCL pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& pose: fromPoses)
    fromCloud->push_back(pcl::PointXYZ(pose.x, pose.y, pose.z));
  pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& pose: toPoses)
    toCloud->push_back(pcl::PointXYZ(pose.x, pose.y, pose.z));

  // Compute rough transformation to get better initialization if needed
  Eigen::Isometry3d roughTransform = Eigen::Isometry3d::Identity();
  if (this->InitWithRoughEstimate)
    this->ComputeRoughTransformOffset(fromPoses, toPoses, roughTransform);

  // Run ICP for transform refinement
  pcl::PointCloud<pcl::PointXYZ> optimCloud;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
  icp.setMaximumIterations(this->NbrIcpIterations);
  icp.setInputSource(fromCloud);
  icp.setInputTarget(toCloud);
  icp.align(optimCloud, roughTransform.matrix());
  offset = icp.getFinalTransformation();

  // DEBUG If requested, impose no roll angle
  if (this->NoRoll)
  {
    // Eigen::Vector3d rpy = offset.rotation().eulerAngles(0, 1, 2);
    // rpy(0) = 0.;
    // offset.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())
    //                                 * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
    //                                 * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d rpy = offset.inverse().linear().eulerAngles(0, 1, 2);
    offset = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * offset;
  }

  // TODO ICP transform display is not correct
  if (this->Verbose)
  {
    std::cout << "ICP has converged:" << icp.hasConverged()
              << "\nICP score: " << icp.getFitnessScore()
              << "\nRough transform:\n" << roughTransform.matrix()
              << "\nICP transform:\n" << (offset * roughTransform.inverse()).matrix()  // CHECK
              << "\nFinal transform:\n" << offset.matrix()
              << std::endl;
  }

  return icp.hasConverged();
}

//------------------------------------------------------------------------------
bool GlobalTrajectoriesRegistration::ComputeRoughTransformOffset(const std::vector<Transform>& fromPoses,
                                                                 const std::vector<Transform>& toPoses,
                                                                 Eigen::Isometry3d& offset)
{
  Eigen::Translation3d translation = ComputeRoughTranslationOffset(fromPoses.back(), toPoses.back());
  Eigen::Quaterniond rotation = ComputeRoughRotationOffset(fromPoses[0], fromPoses.back(),
                                                           toPoses[0], toPoses.back());
  offset = translation * rotation;
  return true;
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