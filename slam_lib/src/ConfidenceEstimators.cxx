//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Julia Sanchez (Kitware SAS)
// Creation date: 2021-06-01
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

#include <LidarSlam/ConfidenceEstimators.h>

namespace LidarSlam
{

namespace Confidence
{

float LCPEstimator(PointCloud::ConstPtr cloud, const std::map<Keypoint, KDTree>& kdTrees, const std::map<Keypoint, float>& leafSizes, int nbThreads)
{
  float LCP = 0.f;
  int nbPoints = cloud->size();
  if (nbPoints > 0)
  {
    #pragma omp parallel for reduction(+:LCP) num_threads(nbThreads)
    for (int n = 0; n < nbPoints; ++n)
    {
      for (auto& kdTree : kdTrees)
      {
        // Check if a kdtree was filled for this keypoint type
        if (kdTree.second.GetInputCloud()->size() > 0)
        {
          std::vector<int> knnIndices;
          std::vector<float> knnSqDist;
          kdTree.second.KnnSearch(cloud->at(n), 1, knnIndices, knnSqDist);
          float sqLCPThreshold = std::pow(leafSizes.at(kdTree.first), 2);
          if (!knnSqDist.empty())
          {
            // We use a Gaussian like estimation for each point fitted in target leaf space
            // to check the probability that one cloud point has a neighbor in the target
            // Probability = 1 if the two points are superimposed
            // Probability < 0.6 if the distance is g.t. the leaf size
            LCP += std::exp( -knnSqDist[0] / (2.f * sqLCPThreshold) );
            break;
          }
        }
      }
    }
    LCP /= nbPoints;
  }
  return LCP;
}

}

}