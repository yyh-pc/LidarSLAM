//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Authors: Arthur Bourbousson (Kitware SAS),
// Creation date: 2022-10-17
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

#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <unsupported/Eigen/Splines>
#include "LidarSlam/State.h"
#include "LidarSlam/Enums.h"
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{
//==============================================================================
//   Quaternion helpers
//==============================================================================
namespace Utils
{
// Make the current quaternion to an angle < 180deg to prev
Eigen::Quaterniond Canonicalized(const Eigen::Quaterniond& prev, const Eigen::Quaterniond& current);

// ---------------------------------------------------------------------------
// Exponential map function for unit quaternion
// Inspired by Sophus and https://splines.readthedocs.io/en/latest/python-module/splines.quaternion.html#
// Using https://splines.readthedocs.io/en/latest/_modules/splines/quaternion.html#UnitQuaternion.ExpMap
Eigen::Quaterniond ExpMap(const Eigen::Vector3d& tangent);

// ---------------------------------------------------------------------------
// Logarithmic map function for unit quaternion
// Inspired by Sophus and https://splines.readthedocs.io/en/latest/python-module/splines.quaternion.html#
// Using https://splines.readthedocs.io/en/latest/_modules/splines/quaternion.html#UnitQuaternion.LogMap
Eigen::Vector3d LogMap(const Eigen::Quaterniond& quat);

// ---------------------------------------------------------------------------
// Slerp with a check for too close values
Eigen::Quaterniond SlerpWithCheck(double t, const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1);

// ---------------------------------------------------------------------------
// De Casteljau algorithm making Spherical Bézier "cubic" curve interpolation
Eigen::Quaterniond DeCasteljau(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1,
                               const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q3,
                               double t);

}// End of Utils namespace

// ---------------------------------------------------------------------------
namespace Interpolation
{

/**
 * @brief IModel
 * Interface class for a basic pose interpolation
 */
class IModel
{
  public:

    // Build the interpolation model with the poses data
    virtual void BuildModel(const std::vector<PoseStamped>& poses) = 0;
    // Interpolate a pose at the time t
    virtual Eigen::Isometry3d operator()(double t) const = 0;
    // Clear the data for interpolation
    virtual void Reset() = 0;
};

// ---------------------------------------------------------------------------
//   Translation Models
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/**
 * @brief Spline
 * Interpolate translations with a global polynomial spline of a certain degree (2, 3...)
 * Use Eigen Spline Library
 */
class Spline : public IModel
{
  public:
    Spline() = delete;
    Spline(const std::vector<PoseStamped>& poses, unsigned int degree){this->BuildModel(poses, degree);}

    // Build the Spline model from new poses
    void BuildModel(const std::vector<PoseStamped>& poses) override;
    void BuildModel(const std::vector<PoseStamped>& poses, unsigned int degree);
    Eigen::Isometry3d operator()(double t) const override;
    void Reset() override {this->SplineModel.reset();}

  private:
     // Eigen spline manager
    std::unique_ptr<Eigen::Spline3d> SplineModel;
    // Bound time values, they are stored to scale the timestamps
    double MinTime, MaxTime;
    unsigned int Degree;

    // Scale time
    double ScaleTime(double t) const {return Utils::Normalize(t, this->MinTime, this->MaxTime);}
    // Scale time values
    Eigen::VectorXd ScaleTimes(const Eigen::VectorXd& times) const {return times.unaryExpr([this](double t) {return this->ScaleTime(t);});}
};

// ---------------------------------------------------------------------------
//   Rotation Models
// ---------------------------------------------------------------------------

/**
 * @brief NSlerp
 * Interpolate N rotations with a SLERP Model between each point and the successive one
 * @warning The points need to be sorted in ascending order of time
 * @warning This class is work in progress, using a simple dichotomy algorithm to find points
 */
class NSlerp : public IModel
{
  public:
    NSlerp() = delete;
    NSlerp(const std::vector<PoseStamped>& poses){this->BuildModel(poses);}
    void BuildModel(const std::vector<PoseStamped>& poses) override;
    Eigen::Isometry3d operator()(double t) const override;
    void Reset() override {Times.clear(); VecRotations.clear();}

  private:
    std::vector<double> Times;
    std::vector<Eigen::UnalignedQuaterniond> VecRotations;
};

// ---------------------------------------------------------------------------
// Non-Uniform Catmull–Rom-Like Rotation Splines
// Using a spherical Bézier curves of cubic "degree"
// Using https://splines.readthedocs.io/en/latest/rotation/catmull-rom-non-uniform.html in C++
// Ctrl points : https://splines.readthedocs.io/en/latest/rotation/catmull-rom-non-uniform.html
// End points : https://splines.readthedocs.io/en/latest/rotation/end-conditions-natural.html
class RotSpline : public IModel
{
  public:
    RotSpline() = delete;
    RotSpline(const std::vector<PoseStamped>& poses);

    void BuildModel(const std::vector<PoseStamped>& poses) override;
    Eigen::Isometry3d operator()(double t) const override;
    void Reset() override {this->Knots.clear(); this->CtrlPoints.clear(); this->Times.clear();}

  private:
    // Rotations through which the curve passes
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> Knots;
    // Control points to constrain the curve
    std::vector<std::pair<Eigen::UnalignedQuaterniond, Eigen::UnalignedQuaterniond>> CtrlPoints;
    // Times of the control knots
    std::vector<double> Times;
};

// ---------------------------------------------------------------------------
//   Trajectory Model
// ---------------------------------------------------------------------------

/**
 * @brief Interpolation between a series of poses
 *        The interpolation combines a rotation model and a translation model
 * @param poses               Data with a pose and a time
 * @param interpolationModel  Model of interpolation choosen
 */
  class Trajectory : public IModel
  {
    using ModelPtr = std::unique_ptr<IModel>;

    public:
      Trajectory() = default;
      Trajectory(Model interpolationModel, const std::vector<PoseStamped>& poses = {});

      void SetModel(Model interpolationModel) {this->InterpolationModel = interpolationModel;}
      Model GetModel() const {return this->InterpolationModel;};

      unsigned int GetNbRequiredData() const {return ModelRequiredNbData.at(this->InterpolationModel);}

      // Build the current interpolation model with new data
      // Spline is built with the model of https://stackoverflow.com/questions/29822041/eigen-spline-interpolation-how-to-get-spline-y-value-at-arbitray-point-x
      void BuildModel(const std::vector<PoseStamped>& poses) override;
      // Compute the interpolation at time t
      Eigen::Isometry3d operator()(double t) const override;
      // Compute the transfo relative between the pose at t1 and the pose at t2
      Eigen::Isometry3d operator()(double t1, double t2) const {return ((*this)(t1).inverse() * (*this)(t2));};
      // Reset interpolation
      void Reset() override;
      // Can the interpolation be used
      bool CanInterpolate(double timeThresh = -1) const;

    private:
      // Interpolation model used
      Model InterpolationModel = LINEAR;

      // Vector of poses used for interpolation
      std::vector<PoseStamped> Poses;

      // Separate models into a translation model and a rotation model
      // Idea from https://arxiv.org/abs/1911.08860
      ModelPtr TranslationPtr;
      ModelPtr RotationPtr;

      double TimeThreshold = 10.;
  };

// ---------------------------------------------------------------------------
//   Interpolation utilities
// ---------------------------------------------------------------------------

// Compute a linear interpolation at time t
// with pose0 and pose1
Eigen::Isometry3d LinearInterpo(const PoseStamped& pose0, const PoseStamped& pose1, double t);

/**
 * @brief Compute a one-time interpolation for transformation at time t
 * @param poses Dataset use for interpolation
 * @param time Time where to interpolate
 * @param model Model of interpolation (see enum Model)
 */
Eigen::Isometry3d Interpolate(const std::vector<PoseStamped>& poses, double time, Model model);

// Use for debugging the PoseStamped vector
void PrintPosesStamped(const std::vector<PoseStamped>& poses);

}  // end of Interpolation namespace
}  // end of LidarSlam namespace
