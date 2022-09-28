//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
// Creation date: 2020-12-10
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

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "LidarSlamTestNode.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_slam_test
{

//------------------------------------------------------------------------------
// TOOLS
//------------------------------------------------------------------------------

namespace Utils
{

//------------------------------------------------------------------------------
Eigen::Matrix3d RPYtoRotationMatrix(double roll, double pitch, double yaw)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
}

//------------------------------------------------------------------------------
Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d& rot)
{
  // `rpy = rot.eulerAngles(2, 1, 0).reverse()`             returns angles in range [-PI:PI]x[-PI:PI]x[0:PI].
  // `rpy = Eigen::EulerAnglesZYXd(rot).angles().reverse()` returns angles in range [-PI:PI]x[-PI:PI]x[-PI:PI].
  // But these are bad. For first range, yaw angle cannot be negative : this
  // leads to un-necessary non trivial RPY decomposition, and to unstable
  // optimization result as we are not optimizing around 0.
  // For second ranges, there exist several RPY decomposition for the same
  // rotation (one of them being non-trivial too). Therefore the optimization
  // may also be unstable by oscillating between them.
  // We prefer to output angles in range [-PI:PI]x[-PI/2:PI/2]x[-PI:PI] : we
  // allow negative values to avoid oscillation artefacts, and minimize the
  // pitch angle to fix representation.
  Eigen::Vector3d rpy;
  rpy.x() = std::atan2(rot(2, 1), rot(2, 2));
  rpy.y() = -std::asin(rot(2, 0));
  rpy.z() = std::atan2(rot(1, 0), rot(0, 0));
  return rpy;
}

//------------------------------------------------------------------------------
Eigen::Isometry3d XYZRPYtoIsometry(Eigen::Vector6d pose)
{
  Eigen::Isometry3d transform;
  transform.linear() = RPYtoRotationMatrix(pose(3), pose(4), pose(5));    // Set rotation part
  transform.translation() = Eigen::Vector3d(pose(0), pose(1), pose(2));   // Set translation part
  transform.makeAffine();                                                 // Set the last row to [0 0 0 1]
  return transform;
}

//------------------------------------------------------------------------------
Eigen::Vector6d IsometryToXYZRPY(const Eigen::Isometry3d& transform)
{
  Eigen::Vector6d xyzrpy;
  xyzrpy << transform.translation(), RotationMatrixToRPY(transform.linear());
  return xyzrpy;
}

//------------------------------------------------------------------------------
Eigen::Isometry3d PoseMsgToIsometry(const geometry_msgs::Pose& poseMsg)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond rot(poseMsg.orientation.w,
                         poseMsg.orientation.x,
                         poseMsg.orientation.y,
                         poseMsg.orientation.z);
  transform.rotate(rot);
  transform.translation() = Eigen::Vector3d({poseMsg.position.x,
                                             poseMsg.position.y,
                                             poseMsg.position.z});

  return transform;
}

//------------------------------------------------------------------------------
float Average(float value, float average, unsigned int counter)
{
  return (average * counter + value) / (counter + 1);
}

//------------------------------------------------------------------------------
double Normalize(double value)
{
  return abs(value) < 1e-15 ? 0.f : value;
}

}

//------------------------------------------------------------------------------
// CLASS METHODS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
LidarSlamTestNode::LidarSlamTestNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : Nh(nh)
  , PrivNh(priv_nh)
{
  //  Compare or not the results with a reference
  if (this->PrivNh.getParam("ref_path", this->RefPath) && !this->RefPath.empty())
  {
    ROS_INFO_STREAM("Loading reference...");
    this->LoadRef();
  }
  else
    ROS_INFO_STREAM("No reference data supplied : comparison ignored");

  //  Compare or not the results with a reference
  if (!this->PrivNh.getParam("res_path", this->ResPath) || this->ResPath.empty())
  {
    ROS_WARN_STREAM("No result folder specified : the results will be stored in /tmp if it exists");
    this->ResPath = "/tmp";
  }

  // Clean results if necessary
  std::remove((this->ResPath + "/Poses.csv").c_str());
  std::remove((this->ResPath + "/Evaluators.csv").c_str());

  // Loading parameters
  this->PrivNh.getParam("time_threshold",     this->TimeThreshold);
  this->PrivNh.getParam("position_threshold", this->PositionThreshold);
  this->PrivNh.getParam("angle_threshold",    this->AngleThreshold);
  this->PrivNh.getParam("verbose",            this->Verbose);

  // Init ROS subscriber
  this->PoseListener = nh.subscribe("slam_odom", 1, &LidarSlamTestNode::PoseCallback, this);
  this->ConfidenceListener = nh.subscribe("slam_confidence", 1, &LidarSlamTestNode::ConfidenceCallback, this);

  ROS_INFO_STREAM(BOLD_GREEN("Lidar slam evaluator is ready !"));
}

//------------------------------------------------------------------------------
bool LidarSlamTestNode::CanBeCompared()
{
  return !this->RefPath.empty() && !this->RefPoses.empty();
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::LoadRef()
{
  // Check the file
  std::string path = this->RefPath + "/Poses.csv";
  std::ifstream refPosesFile(path);
  if (refPosesFile.fail())
  {
    ROS_ERROR_STREAM("The poses csv file '" << path << "' was not found : comparison ignored");
    return;
  }

  // Temporal string to store line data
  std::string line;

  // Fill the reference poses vector
  while (getline(refPosesFile, line))
  {
    // Temporal struct to store pose info
    Pose pose;
    int pos; // char position
    // Store timestamp :
    pos = line.find(" ");
    pose.Stamp = std::stod(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Get pose
    for (int i = 0; i < 5; ++i)
    {
      pos = line.find(" ");
      pose.data(i) = std::stod(line.substr(0, pos));
      line.erase(0, pos + 1);
    }
    pose.data(5) = std::stod(line);
    this->RefPoses.push_back(pose);
  }
  refPosesFile.close();
  ROS_INFO_STREAM("Poses loaded!");

  // Fill the reference confidence vector
  std::ifstream refEvaluatorsFile(this->RefPath + "/Evaluators.csv");
  if (refEvaluatorsFile.fail())
  {
    ROS_ERROR_STREAM("The evaluators csv file '" << path << "' was not found : comparison ignored");
    return;
  }

  while (std::getline(refEvaluatorsFile, line))
  {
    // Temporal struct to store evaluator info
    Evaluator eval;
    int pos; // char position
    // Store timestamp :
    pos = line.find(" ");
    eval.Stamp = std::stod(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Store overlap
    pos = line.find(" ");
    eval.Overlap = std::stof(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Store the number of matches
    pos = line.find(" ");
    eval.NbMatches = std::stoi(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Store the computation time
    eval.Duration = std::stof(line);
    this->RefEvaluators.push_back(eval);
  }
  refEvaluatorsFile.close();
  ROS_INFO_STREAM("Evaluators loaded!");
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::PoseCallback(const nav_msgs::Odometry& poseMsg)
{
  std::ofstream resPosesFile(this->ResPath + "/Poses.csv", std::ofstream::app);
  if (resPosesFile.fail())
  {
    ROS_ERROR_STREAM("Could not save pose");
    return;
  }

  // Save the pose in a file
  double time = poseMsg.header.stamp.toSec();
  Eigen::Isometry3d transform = Utils::PoseMsgToIsometry(poseMsg.pose.pose);
  Eigen::Vector6d pose = Utils::IsometryToXYZRPY(transform);

  resPosesFile << std::fixed << std::setprecision(9) << time << " "
               << Utils::Normalize(pose(0)) << " " << Utils::Normalize(pose(1)) << " " << Utils::Normalize(pose(2)) << " "
               << Utils::Normalize(pose(3)) << " " << Utils::Normalize(pose(4)) << " " << Utils::Normalize(pose(5)) << "\n";
  resPosesFile.close();

  // Check if comparison is required
  if (!this->CanBeCompared())
    return;

  if (this->PoseCounter >= this->RefPoses.size())
  {
    ROS_ERROR_STREAM("More poses received than with reference, comparison ignored");
    return;
  }

  // If the current reference frame has not been seen in this new run ->
  // search the current frame in reference
  if (time - this->RefPoses[this->PoseCounter].Stamp > 1e-6)
  {
    while (this->PoseCounter < this->RefEvaluators.size() && time - this->RefPoses[this->PoseCounter].Stamp > 1e-6)
      ++this->PoseCounter;
  }

  if (this->PoseCounter == this->RefEvaluators.size())
  {
    this->OutputTestResult();
    return;
  }

  // If the current frame has not been seen in reference -> return (wait for next frame)
  if (this->RefPoses[this->PoseCounter].Stamp - time > 1e-6)
  {
    ROS_WARN_STREAM("Reference does not contain a frame at "
                     << std::fixed << std::setprecision(9) << time
                     << " (may have been dropped)."
                     << " Check the reference was computed on the same data.");
    return;
  }

  // Compare the pose with reference trajectory
  Eigen::Isometry3d refTransform = Utils::XYZRPYtoIsometry(this->RefPoses[this->PoseCounter].data);
  Eigen::Isometry3d refPrevTransform;
  if (this->PoseCounter >= 1)
     refPrevTransform = Utils::XYZRPYtoIsometry(this->RefPoses[this->PoseCounter - 1].data);
  else
  {
    this->PrevTransform = transform;
    refPrevTransform = refTransform;
  }
  Eigen::Isometry3d diffTransform = (refPrevTransform.inverse() * refTransform).inverse() * (this->PrevTransform.inverse() * transform);
  Eigen::Vector6d diffPose = Utils::IsometryToXYZRPY(diffTransform);
  // Compute angle difference
  float currentDiffAngle = diffPose.tail(3).norm();
  this->DiffAngle = Utils::Average(currentDiffAngle, this->DiffAngle, this->PoseCounter);
  // Compute translation difference
  float currentDiffPosition = diffPose.head(3).norm();
  this->DiffPosition = Utils::Average(currentDiffPosition, this->DiffPosition, this->PoseCounter);

  // Test fails if any pose is too different from its reference pose
  if (currentDiffPosition > this->PositionThreshold || currentDiffAngle * 180.f / M_PI > this->AngleThreshold)
  {
    ROS_ERROR_STREAM("Pose at " << std::fixed << std::setprecision(9) << time << " is not consistent with reference");
    this->Failure = true;
  }

  if (this->Verbose)
  {
    ROS_INFO_STREAM("Pose difference (at " << std::fixed << std::setprecision(9) << time << ") :\n"
                    << "\t" << currentDiffAngle * 180.f / M_PI << " degrees\n"
                    << "\t" << currentDiffPosition << " m");
    ROS_INFO_STREAM("Pose difference average (at " << std::fixed << std::setprecision(9) << time << ") :\n"
                    << "\t" << this->DiffAngle * 180.f / M_PI << " degrees\n"
                    << "\t" << this->DiffPosition << " m");
  }

  diffTransform = refTransform.inverse() * transform;
  diffPose = Utils::IsometryToXYZRPY(diffTransform);
  this->LastPositionDiff = diffPose.head(3).norm();
  this->LastAngleDiff = diffPose.tail(3).norm();

  ++this->PoseCounter;
  this->PrevTransform = transform;

  // At the end of the test data, notify the user about the success or the failure of the test
  // The last frame cannot be dropped so the node should be ended in any case.
  if (this->PoseCounter == this->RefPoses.size() && this->ConfidenceCounter == this->RefPoses.size())
    this->OutputTestResult();
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::ConfidenceCallback(const lidar_slam::Confidence& confidenceMsg)
{
  // Log the confidence values
  double time = confidenceMsg.header.stamp.toSec();
  float overlap = confidenceMsg.overlap;
  float nbMatches = confidenceMsg.nb_matches;
  float computationTime = confidenceMsg.computation_time;
  std::ofstream EvaluatorsFile(this->ResPath + "/Evaluators.csv", std::ofstream::app);
  if (EvaluatorsFile.fail())
  {
    ROS_ERROR_STREAM("Could not save confidence estimators");
    return;
  }

  EvaluatorsFile << std::fixed << std::setprecision(9) << time << " "
                 << overlap << " " << nbMatches << " " << computationTime << "\n";
  EvaluatorsFile.close();

  // Check if comparison is required
  if (!this->CanBeCompared())
    return;

  if (this->ConfidenceCounter >= this->RefEvaluators.size())
  {
    this->OutputTestResult();
    return;
  }

  // If the current reference frame has not been seen in this new run ->
  // search the current frame in reference
  if (time - this->RefEvaluators[this->ConfidenceCounter].Stamp > 1e-6)
  {
    while (this->ConfidenceCounter < this->RefEvaluators.size() && time - this->RefEvaluators[this->ConfidenceCounter].Stamp > 1e-6)
      ++this->ConfidenceCounter;
  }

  if (this->ConfidenceCounter >= this->RefEvaluators.size())
  {
    this->OutputTestResult();
    return;
  }

  // If the current frame has not been seen in reference -> return (wait for next frame)
  // The last frame cannot be dropped so the node should be ended in any case.
  if (this->RefEvaluators[this->ConfidenceCounter].Stamp - time > 1e-6)
  {
    ROS_WARN_STREAM("Reference does not contain a frame at "
                     << std::fixed << std::setprecision(9) << time
                     << " (may have been dropped)."
                     << " Check the reference was computed on the same data");
    return;
  }

  // Compare with reference evaluators
  float diffOverlap   = overlap         - this->RefEvaluators[this->ConfidenceCounter].Overlap;
  float diffNbMatches = nbMatches       - this->RefEvaluators[this->ConfidenceCounter].NbMatches;
  float diffTime      = computationTime - this->RefEvaluators[this->ConfidenceCounter].Duration;
  this->DiffOverlap   = Utils::Average(diffOverlap,   this->DiffOverlap,   this->ConfidenceCounter);
  this->DiffNbMatches = Utils::Average(diffNbMatches, this->DiffNbMatches, this->ConfidenceCounter);
  this->DiffTime      = Utils::Average(diffTime,      this->DiffTime,      this->ConfidenceCounter);

  if (this->Verbose)
  {
    ROS_INFO_STREAM("Confidence difference for pose at " << std::fixed << std::setprecision(9) << time << ": \n"
                    << "\t" << "Overlap difference: "            << 100 * diffOverlap << " %\n"
                    << "\t" << "Number of matches difference : " << diffNbMatches     << " matches\n"
                    << "\t" << "Computation time difference : "  << diffTime          << " s");
  }

  ++this->ConfidenceCounter;

  // At the end of the test data, notify the user about the success or the failure of the test
  // The last frame cannot be dropped so the node should be ended in any case.
  if (this->PoseCounter == this->RefPoses.size() && this->ConfidenceCounter == this->RefPoses.size())
    this->OutputTestResult();
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::OutputTestResult()
{
  // Test fails if the mean computation time is too high
  // compared with the reference processing
  if (this->DiffTime > this->TimeThreshold)
  {
    ROS_ERROR_STREAM("Computation time is too long compared to reference (" << this->DiffTime << "s longer)");
    this->Failure = true;
  }
  if (!this->Failure)
    ROS_INFO_STREAM(BOLD_GREEN("Test successfully passed"));
  else
    ROS_ERROR_STREAM("Test failed");

  ROS_INFO_STREAM("Comparison with reference (averages): ");
  ROS_INFO_STREAM("Overlap difference : "           << 100 * this->DiffOverlap << " %");
  ROS_INFO_STREAM("Number of matches difference : " << this->DiffNbMatches     << " matches");
  ROS_INFO_STREAM("Computation time difference : "  << this->DiffTime          << " s");
  ROS_INFO_STREAM("Trajectory difference : "        << this->DiffAngle         << " degrees and " << this->DiffPosition << " m");
  ROS_INFO_STREAM ("Final drift from reference : "  << this->LastAngleDiff     << " degrees and " << this->LastPositionDiff << " m");

  // Comparison has stopped : Shut the node down
  ros::shutdown();
}

}  // end of namespace lidar_slam_test

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_slam_test");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  lidar_slam_test::LidarSlamTestNode test(n, priv_nh);

  ros::spin();

  return 0;
}
