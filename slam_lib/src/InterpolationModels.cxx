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

#include "LidarSlam/InterpolationModels.h"

namespace LidarSlam
{

namespace Utils
{

Eigen::Quaterniond Canonicalized(const Eigen::Quaterniond& prev, const Eigen::Quaterniond& current)
{
  if (prev.dot(current) <  -1e-6)
    return (Eigen::Quaterniond{current.w() * -1., current.x() * -1., current.y() * -1., current.z() * -1.});

  return Eigen::Quaterniond{current};
}

// ---------------------------------------------------------------------------
Eigen::Quaterniond ExpMap(const Eigen::Vector3d& tangent)
{
  double norm = tangent.norm() / 2;
  if (std::abs(norm) < 1e-6)
    return (Eigen::Quaterniond::Identity());
  double sin_norm = sin(norm) / norm / 2;
  return Eigen::Quaterniond(cos(norm), tangent(0) * sin_norm, tangent(1) * sin_norm, tangent(2) * sin_norm).normalized();
}

// ---------------------------------------------------------------------------
Eigen::Vector3d LogMap(const Eigen::Quaterniond& quat)
{
  if (std::abs(quat.norm() - 1.) > 1e-6)
    return (Eigen::Vector3d::Zero());
  Eigen::AngleAxis<double> angleAxis(quat);
  return angleAxis.angle() * angleAxis.axis();
}

// ---------------------------------------------------------------------------
Eigen::Quaterniond SlerpWithCheck(double t, const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1)
{
  return q0.isApprox(q1)? q0 : q0.slerp(t, q1);
}

// ---------------------------------------------------------------------------
Eigen::Quaterniond DeCasteljau(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1,
                               const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q3,
                               double t)
{
  // First iteration
  Eigen::Quaterniond slerp_0_1 = SlerpWithCheck(t, q0, q1);
  Eigen::Quaterniond slerp_1_2 = SlerpWithCheck(t, q1, q2);
  Eigen::Quaterniond slerp_2_3 = SlerpWithCheck(t, q2, q3);

  // Second iteration
  auto slerp_0_2 = SlerpWithCheck(t, slerp_0_1, slerp_1_2);
  auto slerp_1_3 = SlerpWithCheck(t, slerp_1_2, slerp_2_3);

  // Final iteration
  auto result = SlerpWithCheck(t, slerp_0_2, slerp_1_3);
  return result;
}

}// End of Utils namespace

namespace Interpolation
{
// ---------------------------------------------------------------------------
//   Translation Models
// ---------------------------------------------------------------------------

// -------------------------------- Spline -----------------------------------

Eigen::Isometry3d Spline::operator()(double t) const
{
  return Eigen::Translation3d((*this->SplineModel)(this->ScaleTime(t))) * Eigen::Quaterniond::Identity();
}

// ---------------------------------------------------------------------------
void Spline::BuildModel(const std::vector<PoseStamped>& poses, unsigned int degree)
{
  if (degree == 0u)
    PRINT_WARNING("Unvalid degree for translation interpolation, using degree = 1");
  this->Degree = std::max(degree, 1u);
  this->BuildModel(poses);
}

// ---------------------------------------------------------------------------
void Spline::BuildModel(const std::vector<PoseStamped>& poses)
{
  unsigned int size = std::max(poses.size(), size_t(2));
  Eigen::VectorXd timeVec(size);
  Eigen::MatrixXd knotMat(3, size);

  if (poses.empty())
  {
    PRINT_ERROR("No data for Spline interpolation, perform constant null interpolation");
    knotMat.setZero();
    timeVec << 0, 1;
  }

  // Create data matrix used for interpolation
  for (size_t i = 0; i < poses.size(); ++i)
  {
    timeVec[i] = poses[i].Time;
    knotMat.col(i) = poses[i].Pose.translation();
  }
  if (poses.size() == 1)
  {
    PRINT_WARNING("Only one data for spline interpolation, perform constant translation");
    // Duplicate value to do a constant Spline
    timeVec[1] = timeVec[0] + 1;
    knotMat.col(1) = knotMat.col(0);
  }

  this->MinTime = timeVec[0];
  this->MaxTime = timeVec[size - 1u];

  // Interpolate data using scaled values and max degree possible
  unsigned int effectiveDegree = std::min(size - 1u, this->Degree);
  this->SplineModel = std::make_unique<Eigen::Spline3d>(Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(knotMat,
                                                        effectiveDegree, this->ScaleTimes(timeVec).transpose()));
}

// ---------------------------------------------------------------------------
//   Rotation Models
// ---------------------------------------------------------------------------

// -------------------------------- N-Slerp ------------------------------------

Eigen::Isometry3d NSlerp::operator()(double t) const
{
  size_t inf, supp;

  if (this->Times.size() <= 2 || t <= this->Times.front())
    {inf = 0; supp = 1;}
  else if (t >= this->Times.back())
    {inf = this->Times.size() - 2; supp = this->Times.size() - 1;}
  else
  {
    auto postIt = std::upper_bound(this->Times.begin(), this->Times.end(), t);
    supp = std::distance(this->Times.begin(), postIt);
    inf  = supp - 1;
  }

  // Interpolate the rotation with Slerp
  double time = Utils::Normalize(t, this->Times[inf], this->Times[supp]);
  return static_cast<Eigen::Isometry3d>(this->VecRotations[inf].slerp(time, this->VecRotations[supp]));
}

// ---------------------------------------------------------------------------
void NSlerp::BuildModel(const std::vector<PoseStamped>& poses)
{
  this->Times.clear();
  this->VecRotations.clear();
  if (poses.empty())
  {
    PRINT_WARNING("No data for NSlerp interpolation, interpolation will return identity rotation");
    this->Times = {0., 1.};
    this->VecRotations = {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()};
    return;
  }
  for (const auto& state : poses)
  {
    this->Times.emplace_back(state.Time);
    this->VecRotations.emplace_back(state.Pose.linear());
  }
  if (poses.size() == 1)
  {
    PRINT_WARNING("Only one data for N-Slerp, perform constant rotation");
    this->Times.emplace_back(poses[0].Time + 1.);
    this->VecRotations.emplace_back(poses[0].Pose.linear());
  }
}


// -------------------------------- Rotation Spline ------------------------------------

RotSpline::RotSpline(const std::vector<PoseStamped>& poses)
{
  this->BuildModel(poses);
}

// ---------------------------------------------------------------------------
void RotSpline::BuildModel(const std::vector<PoseStamped>& poses)
{
  this->Knots.clear();
  this->CtrlPoints.clear();
  this->Times.clear();

  // Error if no data for interpolation, RotSpline must have at least 2 rotations
  if (poses.empty())
  {
    PRINT_ERROR("No data for rotation spline, interpolation will return identity rotation");
    this->Times = {0., 1.};
    this->Knots = {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()};
    return;
  }

  // Convert the data from PoseStamped to Sophus rotation
  // Make sure the angle between rotations are less than 180 degrees
  auto prevQuat = Eigen::Quaterniond::Identity();
  for (auto it = poses.begin(); it != poses.end(); ++it)
  {
    Eigen::Quaterniond quat{it->Pose.linear()};
    // canonicalize the quaternions so the angle is always inferior to 180 degrees
    quat = Utils::Canonicalized(prevQuat, quat).normalized();
    prevQuat = quat;
    this->Knots.push_back(quat);
    this->Times.push_back(it->Time);
  }
  // Error if there is no data for interpolation, RotSpline must have at least 2
  if (poses.size() == 1)
  {
    PRINT_WARNING("Only one data for rotation spline, interpolation will return constant rotation");
    this->Times.emplace_back(poses[0].Time + 1.);
    this->Knots.emplace_back(poses[0].Pose.linear());
  }

  if (this->Knots.size() <= 2)
    return;

  // Create Time diff array : ti+1 - ti : delta_i
  std::vector<double> vecDiffTime;
  vecDiffTime.reserve(this->Times.size() - 1);
  for (size_t i = 0; i < this->Times.size() - 1; ++i)
    vecDiffTime.emplace_back(this->Times[i + 1] - this->Times[i]);

  // Create rho array
  // Formula : rho_i = log_mat(qi+1 * qi^-1) / (ti+1 - ti)
  std::vector<Eigen::Vector3d> vecRho;
  vecRho.reserve(this->Knots.size() - 1);
  for (size_t i = 0; i < this->Knots.size() - 1; ++i)
  {
    auto rho = Utils::LogMap(this->Knots[i + 1] * this->Knots[i].conjugate()) / vecDiffTime[i];
    vecRho.emplace_back(rho);
  }

  // Create Omega array
  // Formula : omega_i = ((ti+1 - ti)rho_i_1 + (ti - ti_1)rho_i) / (ti+1 - ti_1)
  std::vector<Eigen::Vector3d> vecOmega;
  vecOmega.reserve(this->Knots.size() - 1);
  for (size_t i = 1; i < this->Knots.size() - 1; ++i)
  {
    auto omega = (vecDiffTime[i] * vecRho[i-1] + vecDiffTime[i-1] * vecRho[i]) /
                 (this->Times[i + 1] - this->Times[i - 1]);
    vecOmega.emplace_back(omega);
  }

  // Create Controls points from Knots points and time
  // Using https://splines.readthedocs.io/en/latest/rotation/catmull-rom-non-uniform.html
  // and   https://splines.readthedocs.io/en/latest/rotation/end-conditions-natural.html
  // Formula : ctr_pt+ = exp(diffTime_i / 3 * omega_i) * qi
  // Formula : ctr_pt- = exp(diffTime_i_1 / 3 * omega_i)^-1 * qi
  for (size_t i = 1; i < this->Knots.size() - 1; ++i)
  {
    Eigen::Quaterniond qi_plus  = Utils::ExpMap(vecDiffTime[i] / 3. * vecOmega[i - 1]) * this->Knots[i];
    Eigen::Quaterniond qi_minus = Utils::ExpMap(vecDiffTime[i - 1] / 3. * vecOmega[i - 1]).conjugate() * this->Knots[i];

    // Start Point : ctr_pt0+ = slerp(q0, ctrl_pt1-, 0.5)
    if (i == 1)
    {
      Eigen::Quaterniond q0_plus = Utils::SlerpWithCheck(0.5, this->Knots.front(), qi_minus);
      q0_plus = Utils::Canonicalized(Eigen::Quaterniond::Identity(), q0_plus);
      this->CtrlPoints.emplace_back(Eigen::Quaterniond::Identity(), q0_plus);
    }
    qi_minus = Utils::Canonicalized(this->Knots[i - 1], qi_minus);
    qi_plus = Utils::Canonicalized(this->Knots[i], qi_plus);
    this->CtrlPoints.emplace_back(qi_minus, qi_plus);

    // End Point : ctr_pt(N-1)- = slerp(q(N-1), ctrl_pt(N-2)+, 0.5)
    if (i == this->Knots.size() - 2)
    {
      Eigen::Quaterniond qN_minus = Utils::SlerpWithCheck(0.5, this->Knots.back(), qi_plus);
      qN_minus = Utils::Canonicalized(this->Knots[i - 1], qN_minus);
      this->CtrlPoints.emplace_back(qN_minus, Eigen::Quaterniond::Identity());
    }
  }
}

// ---------------------------------------------------------------------------
Eigen::Isometry3d RotSpline::operator()(double t) const
{
  // If only 2 elements, perform a slerp
  if (this->Knots.size() == 2)
  {
    if (this->Times[0] == this->Times[1] || this->Knots[0].isApprox(this->Knots[1]))
      return(Eigen::Translation3d::Identity() * this->Knots[0]);
    double t_norm = Utils::Normalize(t, this->Times[1], this->Times[0]);
    Eigen::Quaterniond rot = Utils::SlerpWithCheck(t_norm, this->Knots[0], this->Knots[1]);
    return (Eigen::Translation3d::Identity() * rot);
  }

  // Normal case : quadratic or cubic rotation spline
  size_t firstInd, secondInd;
  if (t <= this->Times.front())
    secondInd = 1;
  else if (t >= this->Times.back())
    secondInd = this->Times.size() - 1;
  else
    secondInd = std::distance(this->Times.begin(),
                              std::upper_bound(this->Times.begin(), this->Times.end(), t));
  firstInd  = secondInd - 1;

  double t_norm = Utils::Normalize(t, this->Times[firstInd], this->Times[secondInd]);
  return (firstInd == secondInd || this->Knots[firstInd].isApprox(this->Knots[secondInd])) ?
          static_cast<Eigen::Isometry3d>(this->Knots[firstInd]) :
          static_cast<Eigen::Isometry3d>(Utils::DeCasteljau(this->Knots[firstInd], this->CtrlPoints[firstInd].second,
                                                             this->CtrlPoints[secondInd].first, this->Knots[secondInd],
                                                             t_norm));
}

// ------------------ Tools Methods ------------------

// ---------------------------------------------------------------------------
//   Trajectory Model
// ---------------------------------------------------------------------------

Trajectory::Trajectory(Model interpolationModel, const std::vector<PoseStamped>& poses) : InterpolationModel(interpolationModel)
{
  if (!poses.empty())
    this->BuildModel(poses);
}

// ---------------------------------------------------------------------------
void Trajectory::BuildModel(const std::vector<PoseStamped>& poses)
{
  // Create poses depending using the required number of data for the model
  size_t requiredNbData = ModelRequiredNbData.at(this->InterpolationModel);
  if (requiredNbData < poses.size())
    this->Poses = {poses.end() - requiredNbData, poses.end()};
  else
    this->Poses = poses;

  if (this->Poses.size() < requiredNbData)
    PRINT_WARNING("Interpolation : insufficient number of knots ("
                  << this->Poses.size() << "), simpler model will be used");
  // Sort by order of time
  std::sort(this->Poses.begin(), this->Poses.end(),
    [](const PoseStamped& a, const PoseStamped& b) {return a.Time < b.Time;});

  // Choose translation model
  switch (this->InterpolationModel)
  {
    case Model::LINEAR:     this->TranslationPtr = std::make_unique<Spline>(this->Poses, 1);  break;
    case Model::QUADRATIC:  this->TranslationPtr = std::make_unique<Spline>(this->Poses, 2);  break;
    case Model::CUBIC:      this->TranslationPtr = std::make_unique<Spline>(this->Poses, 3);  break;
    default: this->TranslationPtr = std::make_unique<Spline>(this->Poses, 1);  break; // Default is linear
  }
  // Choose rotation model
  if (this->InterpolationModel == Model::QUADRATIC ||
      this->InterpolationModel == Model::CUBIC)
    this->RotationPtr = std::make_unique<RotSpline>(this->Poses);
  else
    this->RotationPtr = std::make_unique<NSlerp>(this->Poses);

  // User is warned if interpolation is not usable
  if (std::abs(this->Poses.back().Time - this->Poses.front().Time) < 1e-6)
    PRINT_WARNING("Interpolation times are constant: interpolation cannot be used")
  if (std::abs(this->Poses.back().Time - this->Poses.front().Time) > this->TimeThreshold)
    PRINT_WARNING("Interpolation times are too far: interpolation cannot be used")
}

// ---------------------------------------------------------------------------
Eigen::Isometry3d Trajectory::operator()(double t) const
{
  // Edge cases
  if (this->Poses.size() == 1) // One only pose available
    return (this->Poses.front().Pose);
  else if (t < this->Poses.front().Time) // Time is before the model -> extrapolation
    return LinearInterpo(this->Poses[0], this->Poses[1], t);
  else if (t > this->Poses.back().Time) // Time is after the model -> extrapolation
    return LinearInterpo(this->Poses[this->Poses.size() - 2], this->Poses.back(), t);

  return (*this->TranslationPtr)(t) * (*this->RotationPtr)(t);
}

// ---------------------------------------------------------------------------
void Trajectory::Reset()
{
  this->Poses.clear();
  this->TranslationPtr.reset();
  this->RotationPtr.reset();
}

// ---------------------------------------------------------------------------
bool Trajectory::CanInterpolate(double timeThresh) const
{
  double timeRange = std::abs(this->Poses.back().Time - this->Poses.front().Time);

  return !this->Poses.empty() && this->TranslationPtr && this->RotationPtr &&
         timeRange > 1e-6 && timeRange < std::max(timeThresh, this->TimeThreshold);
}

// ---------------------------------------------------------------------------
//   Interpolation tools
// ---------------------------------------------------------------------------

Eigen::Isometry3d LinearInterpo(const PoseStamped& pose0, const PoseStamped& pose1, double t)
{
  const Eigen::Isometry3d& H0 = pose0.Pose, H1 = pose1.Pose;
  double t0 = pose0.Time, t1 = pose1.Time;

  if(t0 == t1 || H0.isApprox(H1))
    return H1;
  double time = Utils::Normalize(t, t0, t1);
  Eigen::Quaterniond rot(Eigen::Quaterniond(H0.linear()).slerp(time, Eigen::Quaterniond(H1.linear())));
  Eigen::Translation3d trans(H0.translation() + time * (H1.translation() - H0.translation()));
  return trans * rot;
}

// ---------------------------------------------------------------------------
Eigen::Isometry3d Interpolate(const std::vector<PoseStamped>& poses, double time, Model model)
{
  Trajectory interpo(model, poses);
  return interpo(time);
}

// ---------------------------------------------------------------------------
void PrintPosesStamped(const std::vector<PoseStamped>& poses)
{
  for (size_t i = 0; i < poses.size(); ++i)
  {
    std::cout << "Poses[" << i << "] :\n"
                 " translation = [" << poses[i].Pose.translation().transpose()                                        << "] m\n"
                 " rotation    = [" << Utils::Rad2Deg(Utils::RotationMatrixToRPY(poses[i].Pose.linear())).transpose() << "] °\n"
                 " time = " << poses[i].Time << "] s\n";
  }
}

}  // end of Interpolation namespace
}  // end of LidarSlam namespace
