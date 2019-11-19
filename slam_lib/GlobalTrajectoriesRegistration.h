#ifndef GLOBAL_TRAJECTORIES_REGISTRATION_H
#define GLOBAL_TRAJECTORIES_REGISTRATION_H

#include "Transform.h"
#include <pcl/registration/icp.h>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

/**
 * @brief Find the global transform between two trajectories with ICP.
 * 
 * TODO Add optionnal constraint to get global transform parallel to output XY-plane.
 */
class GlobalTrajectoriesRegistration
{
public:

  //----------------------------------------------------------------------------
  GlobalTrajectoriesRegistration() = default;

  SetMacro(NbrIcpIterations, unsigned int)

  SetMacro(InitWithRoughEstimate, bool)

  SetMacro(NoRoll, bool)

  SetMacro(Verbose, bool)

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute global translation/rotation offset between two trajectories.
   * @param[in]  fromPoses The initial trajectory.
   * @param[in]  toPoses   The final trajectory.
   * @param[out] offset    The global transform from 'fromPoses' to 'toPoses'.
   * @return true if offset has been computed correctly, false if error occured.
   */
  bool ComputeTransformOffset(const std::vector<Transform>& fromPoses,
                              const std::vector<Transform>& toPoses,
                              Eigen::Isometry3d& offset);

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute approximate global translation/rotation offset between two
   *        trajectories using only first and last points.
   * @param[in]  fromPoses The initial trajectory.
   * @param[in]  toPoses   The final trajectory.
   * @param[out] offset    The approximate global transform from 'fromPoses' to 'toPoses'.
   * @return true if offset has been computed correctly, false if error occured.
   */
  static bool ComputeRoughTransformOffset(const std::vector<Transform>& fromPoses,
                                          const std::vector<Transform>& toPoses,
                                          Eigen::Isometry3d& offset);

private:

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute translation offset between two poses.
   * @param[in] fromPose The initial point.
   * @param[in] toPose   The final point.
   * @return The translation from 'fromPose' to 'toPose'.
   */
  static Eigen::Translation3d ComputeRoughTranslationOffset(const Transform& fromPose, const Transform& toPose);

  //----------------------------------------------------------------------------
  /*!
   * @brief Compute orientation offset between two trajectories.
   * @param[in] fromPose1 The initial point of the initial trajectory.
   * @param[in] fromPose2 The final point of the initial trajectory.
   * @param[in] toPose1   The initial point of the final trajectory.
   * @param[in] toPose2   The final point of the final trajectory.
   * @return The rotation from (fromPose1, fromPose2) to (toPose1, toPose2).
   */
  static Eigen::Quaterniond ComputeRoughRotationOffset(const Transform& fromPose1, const Transform& fromPose2,
                                                       const Transform& toPose1, const Transform& toPose2);

private:

  unsigned int NbrIcpIterations = 50;  ///< Max number of iterations to do in ICP matching.
  bool InitWithRoughEstimate = true;   ///< Init ICP with a rough estimate of the transform to help convergence.
  bool NoRoll = false;   ///< If true, roll angle (X axis) will be set to 0 in output transform.
  bool Verbose = false;  ///< If true, print some debug info.
};

#endif // GLOBAL_TRAJECTORIES_REGISTRATION_H