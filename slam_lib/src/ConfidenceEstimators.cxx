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
#include <numeric>

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
    // Get only usable keypoint types
    std::vector<Keypoint> kpToUse;
    for (const auto& kdTree : kdTrees)
    {
      if (!kdTree.second.GetInputCloud()->empty())
        kpToUse.push_back(kdTree.first);
    }
    
    std::vector<float> LCPvec(nbPoints, 0.f);
    #pragma omp parallel for num_threads(nbThreads)
    for (int n = 0; n < nbPoints; ++n)
    {
      for (const auto& k : kpToUse)
      {
        std::vector<int> knnIndices;
        std::vector<float> knnSqDist;
        if (kdTrees.at(k).KnnSearch(cloud->at(n), 1, knnIndices, knnSqDist) > 0)
        {
          // We use a Gaussian like estimation for each point fitted in target leaf space
          // to check the probability that one cloud point has a neighbor in the target
          // Probability = 1 if the two points are superimposed
          // Probability < 0.011 if the distance is g.t. the leaf size
          float sqLCPThreshold = std::pow(leafSizes.at(k) / 3.f, 2);
          float currentProba = std::exp( -knnSqDist[0] / (2.f * sqLCPThreshold) );
          if (currentProba > LCPvec[n])
            LCPvec[n] = currentProba;
        }
      }
    }
    LCP = std::accumulate(LCPvec.begin(), LCPvec.end(), 0.f) / nbPoints;
  }
  return LCP;
}

}

}