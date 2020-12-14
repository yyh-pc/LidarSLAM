//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Nicolas Cadart (Kitware SAS)
// Creation date: 2020-11-10
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

#ifndef LIDAR_SLAM_ENUMS_H
#define LIDAR_SLAM_ENUMS_H

//------------------------------------------------------------------------------
//! Type of a keypoint
enum Keypoint
{
  EDGE  = 0,   ///< edge keypoint (sharp local structure)
  PLANE = 1,   ///< plane keypoint (flat local structure)
  BLOB  = 2,   ///< blob keypoint (spherical local structure)
  nKeypointTypes
};

//------------------------------------------------------------------------------
//! How to deal with undistortion
enum UndistortionMode
{
  //! No undistortion is performed :
  //!  - End scan pose is optimized using rigid registration of raw scan and map.
  //!  - Raw input scan is added to maps.
  NONE = 0,

  //! Minimal undistortion is performed :
  //!  - Begin scan pose is linearly interpolated between previous and current end scan poses.
  //!  - End scan pose is optimized using rigid registration of undistorted scan and map.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  APPROXIMATED = 1,

  //! Ceres-optimized undistortion is performed :
  //!  - Both begin and end scan poses are optimized using registration of undistorted scan and map.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  OPTIMIZED = 2
};

//------------------------------------------------------------------------------
//! How to estimate Ego-Motion (approximate relative motion since last frame)
enum class EgoMotionMode
{
  //! No ego-motion step is performed : relative motion is Identity, new
  //! estimated Tworld is equal to previous Tworld.
  //! Fast, but may lead to unstable and imprecise Localization step if motion
  //! is important.
  NONE = 0,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses.
  //! Fast and precise if motion is roughly constant and continuous.
  MOTION_EXTRAPOLATION = 1,

  //! Estimate Trelative (and therefore Tworld) by globally registering new
  //! frame on previous frame.
  //! Slower and need textured enough environment, but do not rely on
  //! constant motion hypothesis.
  REGISTRATION = 2,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses. Then this estimation is refined by globally
  //! registering new frame on previous frame.
  //! Slower and need textured enough environment, but should be more precise
  //! and rely less on constant motion hypothesis.
  MOTION_EXTRAPOLATION_AND_REGISTRATION = 3
};

#endif // LIDAR_SLAM_ENUMS_H