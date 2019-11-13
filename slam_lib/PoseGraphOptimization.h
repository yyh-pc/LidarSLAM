//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Laurenson Nick
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
//=========================================================================
#ifndef POSE_GRAPH_OPTIMIZATION_H
#define POSE_GRAPH_OPTIMIZATION_H

#include <g2o/core/sparse_optimizer.h>
#include "Transform.h"

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

/**
 * @brief The PoseGraphOptimization class enable to optimize a trajectory given
 * some GPS anchor points.
 */
class PoseGraphOptimization
{
public:
  PoseGraphOptimization();

  //! Pose of the GPS in the referential of the sensor that produce the trajectory to optimize
  void SetGPSCalibration(double x, double y, double z, double rx, double ry, double rz);

  //! Compute optimized Slam trajectory using GPS ground-control points.
  bool Process(const std::vector<Transform>& slamPoses,
               const std::vector<Transform>& gpsPoses,
               const std::vector<std::array<double, 36>>& slamCov,
               const std::vector<std::array<double, 9>>& gpsCov,
               std::vector<Transform>& optimizedSlamPoses);

  SetMacro(SaveG2OFile, bool)

  SetMacro(G2OFileName, std::string)

  SetMacro(NbIteration, int)

  SetMacro(Verbose, bool)

  SetMacro(TimeOffset, double)

private:

  g2o::SparseOptimizer GraphOptimizer;
  bool SaveG2OFile = false;
  std::string G2OFileName = "";
  int NbIteration = 30;
  bool Verbose = false;
  double TimeOffset = 0;
};

#endif // POSE_GRAPH_OPTIMIZATION_H