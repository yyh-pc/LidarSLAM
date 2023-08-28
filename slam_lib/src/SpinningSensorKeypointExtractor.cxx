//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Laurenson Nick (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2018-03-27
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

#include "LidarSlam/Utilities.h"
#include "LidarSlam/SpinningSensorKeypointExtractor.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <random>

namespace LidarSlam
{

namespace
{
//-----------------------------------------------------------------------------
bool LineFitting::FitLineAndCheckConsistency(const SpinningSensorKeypointExtractor::PointCloud& cloud,
                                            const std::vector<int>& indices)
{
  // Check line width
  float lineLength = (cloud[indices.front()].getVector3fMap() - cloud[indices.back()].getVector3fMap()).norm();
  float widthTheshold = std::max(this->MaxLineWidth, lineLength / this->LengthWidthRatio);

  float maxDist = widthTheshold;
  Eigen::Vector3f bestDirection;

  this->Position = Eigen::Vector3f::Zero();
  for (int idx : indices)
    this->Position += cloud[idx].getVector3fMap();
  this->Position /= indices.size();

  // RANSAC
  for (int i = 0; i < int(indices.size()) - 1; ++i)
  {
    // Extract first point
    auto& point1 = cloud[indices[i]].getVector3fMap();
    for (int j = i+1; j < int(indices.size()); ++j)
    {
      // Extract second point
      auto& point2 = cloud[indices[j]].getVector3fMap();

      // Compute line formed by point1 and point2
      this->Direction = (point1 - point2).normalized();

      // Reset score for new points pair
      float currentMaxDist = 0;
      // Compute score : maximum distance of one neighbor to the current line
      for (int idx : indices)
      {
        currentMaxDist = std::max(currentMaxDist, this->DistanceToPoint(cloud[idx].getVector3fMap()));

        // If the current point distance is too high,
        // the current line won't be selected anyway so we
        // can avoid computing next points' distances
        if (currentMaxDist > widthTheshold)
          break;
      }

      // If the current line implies high error for one neighbor
      // the output line is considered as not trustworthy
      if (currentMaxDist > 2.f * widthTheshold)
        return false;

      if (currentMaxDist <= maxDist)
      {
        bestDirection = this->Direction;
        maxDist = currentMaxDist;
      }

    }
  }

  if (maxDist >= widthTheshold - 1e-10)
    return false;

  this->Direction = bestDirection;

  return true;
}

//-----------------------------------------------------------------------------
inline float LineFitting::DistanceToPoint(Eigen::Vector3f const& point) const
{
  return ((point - this->Position).cross(this->Direction)).norm();
}
} // end of anonymous namespace

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::Enable(const std::vector<Keypoint>& kptTypes)
{
  for (auto& en : this->Enabled)
    en.second = false;
  for (auto& k : kptTypes)
    this->Enabled[k] = true;
}

//-----------------------------------------------------------------------------
SpinningSensorKeypointExtractor::PointCloud::Ptr SpinningSensorKeypointExtractor::GetKeypoints(Keypoint k)
{
  if (!this->Enabled.count(k) || !this->Enabled[k])
  {
    PRINT_ERROR("Unable to get keypoints of type " << KeypointTypeNames.at(k));
    return PointCloud::Ptr();
  }

  PointCloud::Ptr keypoints = this->Keypoints.at(k).GetCloud(this->MaxPoints);
  Utils::CopyPointCloudMetadata(*this->Scan, *keypoints);
  return keypoints;
}


//-----------------------------------------------------------------------------
// 对当前帧计算提取关键点
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  // Split whole pointcloud into separate laser ring clouds
  // 将一帧激光雷达按照线束保存，并估算了旋转激光雷达的水平分辨率
  this->ConvertAndSortScanLines();

  // Initialize the features vectors and keypoints
  // 开辟相关的数组、变量
  this->PrepareDataForNextFrame();

  // Compute keypoints scores
  // 计算曲率
  // !:在此之前只需要准备好按照线束存储的点云，即可计算各种类型的曲率
  this->ComputeCurvature();

  // Labelize and extract keypoints
  // Warning : order matters
  // 通过对扫描线上的点进行随机采样，将采样后的点添加到关键点集合中的Blob类型中，用于表示斑点特征
  if (this->Enabled[Keypoint::BLOB])
    this->ComputeBlobs();
  if (this->Enabled[Keypoint::PLANE])
    this->ComputePlanes();
  // 利用深度间隙、空间间隙、以及角度来筛选角点
  if (this->Enabled[Keypoint::EDGE])
    this->ComputeEdges();
  // 利用this->IntensityGap信息筛选
  if (this->Enabled[Keypoint::INTENSITY_EDGE])
    this->ComputeIntensityEdges();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Clear previous scan lines
  for (auto& scanLineCloud: this->ScanLines)
  {
    // Use clear() if pointcloud already exists to avoid re-allocating memory.
    // No worry as ScanLines is never shared with outer scope.
    if (scanLineCloud)
      scanLineCloud->clear();
    else
      scanLineCloud.reset(new PointCloud);
  }

  // Separate pointcloud into different scan lines
  for (const Point& point: *this->Scan)
  {
    // Ensure that there are enough available scan lines
    while (point.laser_id >= this->ScanLines.size())
      this->ScanLines.emplace_back(new PointCloud);

    // Add the current point to its corresponding laser scan
    this->ScanLines[point.laser_id]->push_back(point);
  }

  // Save the number of lasers
  this->NbLaserRings = this->ScanLines.size();

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  // ?:这里对水平分辨率进行了估算，但是并没有看懂怎么实现的
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution)
    this->EstimateAzimuthalResolution();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Initialize the features vectors with the correct length
  this->Angles.resize(this->NbLaserRings);
  this->DepthGap.resize(this->NbLaserRings);
  this->SpaceGap.resize(this->NbLaserRings);
  this->IntensityGap.resize(this->NbLaserRings);
  this->Label.resize(this->NbLaserRings);

  // Initialize the scan lines features vectors with the correct length
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    size_t nbPoint = this->ScanLines[scanLine]->size();
    this->Label[scanLine].assign(nbPoint, KeypointFlags().reset());  // set all flags to 0
    this->Angles[scanLine].assign(nbPoint, -1.);
    this->DepthGap[scanLine].assign(nbPoint, -1.);
    this->SpaceGap[scanLine].assign(nbPoint, -1.);
    this->IntensityGap[scanLine].assign(nbPoint, -1.);
  }

  // Reset voxel grids
  LidarPoint minPt, maxPt;
  pcl::getMinMax3D(*this->Scan, minPt, maxPt);

  for (auto k : KeypointTypes)
  {
    if (this->Keypoints.count(k))
      this->Keypoints[k].Clear();
    if (this->Enabled[k])
      this->Keypoints[k].Init(minPt.getVector3fMap(), maxPt.getVector3fMap(), this->VoxelResolution, this->Scan->size());
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  // Compute useful const values to lighten the loop
  // ?:计算有用的常量值来简化循环，目前还没有完全理解这几个常量的意义
  const float cosMinBeamSurfaceAngle = std::cos(Utils::Deg2Rad(this->MinBeamSurfaceAngle));
  // AzimuthalResolution指的是旋转激光雷达的水平角分辨率，用于计算连续两次发射之间的预期距离
  // 该值设定初始值为0，会在下一帧自动估算
  // 以1.5倍的水平分辨率作为最大的方位角的余弦值作为阈值
  const float cosMaxAzimuth = std::cos(1.5 * this->AzimuthalResolution);
  // 以5个点（5倍）的水平分辨率作为一个空间gap
  const float cosSpaceGapAngle = std::cos(this->EdgeNbGapPoints * this->AzimuthalResolution);
  // 方位角范围控制在[0, 2pi]
  float azimuthMinRad = Utils::Deg2Rad(this->AzimuthMin);
  float azimuthMaxRad = Utils::Deg2Rad(this->AzimuthMax);

  // Rescale angles in [0, 2pi]
  while (azimuthMinRad < 0)
    azimuthMinRad += 2 * M_PI;
  while (azimuthMaxRad < 0)
    azimuthMaxRad += 2 * M_PI;

  // Init random distribution
  // 这行代码创建了一个名为 gen 的伪随机数生成器对象
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  // 用于定义一个在 [0.0, 1.0) 范围内的均匀分布
  std::uniform_real_distribution<> dis(0.0, 1.0);

  // loop over scans lines
  // 按线束循环处理
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    // 按线存储的点云
    const PointCloud& scanLineCloud = *(this->ScanLines[scanLine]);
    // 该线的点云数量
    const int Npts = scanLineCloud.size();

    // if the line is almost empty, skip it
    // 点数过少直接跳过提取
    if (this->IsScanLineAlmostEmpty(Npts))
      continue;

    // Loop over points in the current scan line
    // 对当前线里的点进行遍历
    for (int index = 0; index < Npts; ++index)
    {
      // Random sampling to decrease keypoints extraction
      // computation time
      if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
        continue;

      // Central point
      // 当前点求range作为depth
      const Eigen::Vector3f& centralPoint = scanLineCloud[index].getVector3fMap();
      float centralDepth = centralPoint.norm();

      // Check distance to sensor
      // 距离限制在1.5——200米之间
      if (centralDepth < this->MinDistanceToSensor || centralDepth > this->MaxDistanceToSensor)
        continue;

      // Check azimuth angle
      // ?:检查xy平面centralPoint对应的方位角（以弧度表示）是否在由azimuthMinRad和azimuthMaxRad定义的范围内
      if (std::abs(azimuthMaxRad - azimuthMinRad) < 2 * M_PI - 1e-6)
      {
        // 计算当前点以x轴正方向为基准的方位角，取值范围0——2PI
        float cosAzimuth = centralPoint.x() / std::sqrt(std::pow(centralPoint.x(), 2) + std::pow(centralPoint.y(), 2));
        float azimuth = centralPoint.y() > 0? std::acos(cosAzimuth) : 2*M_PI - std::acos(cosAzimuth);
        if (azimuthMinRad == azimuthMaxRad)
          continue;
        if (azimuthMinRad < azimuthMaxRad &&
            (azimuth < azimuthMinRad ||
             azimuth > azimuthMaxRad))
          continue;

        if (azimuthMinRad > azimuthMaxRad &&
            (azimuth < azimuthMinRad && azimuth > azimuthMaxRad))
          continue;
      }

      // Fill left and right neighbors
      // Those points must be more numerous than MinNeighNb and occupy more space than MinNeighRadius
      /* 这是一个Lambda函数的定义，它用于获取当前点的左侧或右侧近邻点。
      Lambda函数接收一个bool类型的参数right，用于指示是获取右侧近邻点（true）还是左侧近邻点（false）。
      函数返回一个存储近邻点索引的std::vector<int> */
      auto GetNeighbors = [&](bool right) -> std::vector<int>
      {
        // neighbors中存储的是当前线的点云索引
        std::vector<int> neighbors;
        neighbors.reserve(this->MinNeighNb);
        // 左或者右近邻
        int plusOrMinus = right? 1 : -1;
        // 近邻索引偏移量
        int idxNeigh = 1;
        // 近邻点首尾距离
        float lineLength = 0.f;
        // 当近邻点个数超过4同时近邻点首尾距离超过0.1米就停止近邻点搜索
        while ((int(neighbors.size()) < this->MinNeighNb
                || lineLength < this->MinNeighRadius)
                && int(neighbors.size()) < Npts)
        {
          neighbors.emplace_back((index + plusOrMinus * idxNeigh + Npts) % Npts); // +Npts to avoid negative values
          lineLength = (scanLineCloud[neighbors.back()].getVector3fMap() - scanLineCloud[neighbors.front()].getVector3fMap()).norm();
          ++idxNeigh;
        }

        // Sample the neighborhood to limit computation time
        // 对超过近邻点阈值数量的进行间隔采样，防止点数过多
        if (neighbors.size() > this->MinNeighNb)
        {
          int step = neighbors.size() / this->MinNeighNb;
          std::vector<int> newIndices;
          newIndices.reserve(neighbors.size() / step);
          for (int i = 0; i < neighbors.size(); i = i + step)
            newIndices.emplace_back(neighbors[i]);
          neighbors = newIndices;
        }
        return neighbors;
      };

      // 左右紧邻都搜索，相当于搜索当前点前后各4个点，距离当前点0.1米之内的近邻
      std::vector<int> leftNeighbors  = GetNeighbors(false);
      std::vector<int> rightNeighbors = GetNeighbors(true);

      // 最近的左右近邻点构成的向量以及模长
      const auto& rightPt = scanLineCloud[rightNeighbors.front()].getVector3fMap();
      const auto& leftPt = scanLineCloud[leftNeighbors.front()].getVector3fMap();

      const float rightDepth = rightPt.norm();
      const float leftDepth = leftPt.norm();

      // a*b = |a||b|cos
      // 计算近邻点与当前点之间的夹角，abs保证夹角为0-90度
      const float cosAngleRight = std::abs(rightPt.dot(centralPoint) / (rightDepth * centralDepth));
      const float cosAngleLeft = std::abs(leftPt.dot(centralPoint) / (leftDepth * centralDepth));

      // 当前点到左右近邻点构成的向量及模长
      const Eigen::Vector3f diffVecRight = rightPt - centralPoint;
      const Eigen::Vector3f diffVecLeft = leftPt - centralPoint;

      // 向量差的模长
      const float diffRightNorm = diffVecRight.norm();
      const float diffLeftNorm = diffVecLeft.norm();

      // 计算两个diff向量与当前点形成的锐角的余弦值，abs保证了正值
      float cosBeamLineAngleLeft = std::abs(diffVecLeft.dot(centralPoint) / (diffLeftNorm * centralDepth) );
      float cosBeamLineAngleRight = std::abs(diffVecRight.dot(centralPoint) / (diffRightNorm * centralDepth));

      // Step:通过邻域几何属性来筛选角点
      if (this->Enabled[EDGE])
      {
        // Compute space gap

        // Init variables
        // 用于存储右侧和左侧的空间gap。这些gap的初值设置为-1，表示暂时未计算到空间gap。
        float distRight = -1.f;
        float distLeft = -1.f;

        // Compute space gap (if some neighbors were missed)
        if (cosBeamLineAngleRight < cosMinBeamSurfaceAngle && cosAngleRight < cosSpaceGapAngle)
          distRight = diffRightNorm;

        if (cosBeamLineAngleLeft < cosMinBeamSurfaceAngle && cosAngleLeft < cosSpaceGapAngle)
          distLeft = diffLeftNorm;

        // 计算空间gap
        // 左右两侧的空间gap的最大值存储在SpaceGap中
        this->SpaceGap[scanLine][index] = std::max(distLeft, distRight);

        // Stop search for first and last points of the scan line
        // because the discontinuity may alter the other criteria detection
        // 该线的前几个点和后几个点不做计算
        if (index < int(leftNeighbors.size()) || index >= Npts - int(rightNeighbors.size()))
          continue;

        // Compute depth gap

        // Reinit variables
        distRight = -1.f;
        distLeft = -1.f;

        // ?:这里有一个问题，怎么保证计算而来的distRight和distLeft都是正的，这个投影可能是负的
        if (cosAngleRight > cosMaxAzimuth)
          distRight = centralPoint.dot(diffVecRight) / centralDepth;
        if (cosAngleLeft > cosMaxAzimuth)
          distLeft = leftPt.dot(diffVecLeft) / leftDepth;

        // Check right points are consecutive + not on a bended wall
        // If the points lay on a bended wall, previous and next points should be in the same direction
        // nextdiffVecRight.dot(diffVecRight) / diffRightNorm求取出来的就是相邻两个diffVector之间的夹角余弦值
        if (distRight > this->EdgeDepthGapThreshold)
        {
          auto nextdiffVecRight = (scanLineCloud[rightNeighbors[1]].getVector3fMap() - rightPt).normalized();
          // 夹角小于10°
          if ((nextdiffVecRight.dot(diffVecRight) / diffRightNorm) > cosMinBeamSurfaceAngle ||
              (-diffVecLeft.dot(diffVecRight) / (diffRightNorm * diffLeftNorm)) > cosMinBeamSurfaceAngle)
            // 重置dist
            distRight = -1.f;
        }

        // Check left points are consecutive + not on a bended wall
        // If the points lay on a bended wall, previous and next points should be in the same direction
        if (distLeft > this->EdgeDepthGapThreshold)
        {
          auto prevdiffVecLeft = (scanLineCloud[leftNeighbors[1]].getVector3fMap() - leftPt).normalized();
          if ((prevdiffVecLeft.dot(diffVecLeft) / diffLeftNorm > cosMinBeamSurfaceAngle) ||
              (-diffVecRight.dot(diffVecLeft) / (diffRightNorm * diffLeftNorm)) > cosMinBeamSurfaceAngle)
            distLeft = -1.f;
        }

        // 计算深度gap
        this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
      }

      if (cosAngleRight < cosMaxAzimuth || cosAngleLeft < cosMaxAzimuth)
        continue;

      // Fit line on the left and right neighborhoods and
      // skip point if they are not usable
      LineFitting leftLine, rightLine;
      if (!leftLine.FitLineAndCheckConsistency(scanLineCloud, leftNeighbors) ||
          !rightLine.FitLineAndCheckConsistency(scanLineCloud, rightNeighbors))
        continue;

      // 当前点向量和相邻点拟合直线之间的余弦值
      cosBeamLineAngleLeft = std::abs(leftLine.Direction.dot(centralPoint) / centralDepth);
      if (cosBeamLineAngleLeft > cosMinBeamSurfaceAngle)
        continue;

      cosBeamLineAngleRight = std::abs(rightLine.Direction.dot(centralPoint) / centralDepth);
      if (cosBeamLineAngleRight > cosMinBeamSurfaceAngle)
        continue;

      // Step:通过intensity属性来筛选角点
      if (this->Enabled[INTENSITY_EDGE])
      {
        // Compute intensity gap
        if (std::abs(scanLineCloud[rightNeighbors.front()].intensity - scanLineCloud[leftNeighbors.front()].intensity) > this->EdgeIntensityGapThreshold)
        {
          // 计算左右相邻点的强度均值
          // Compute mean intensity on the left
          float meanIntensityLeft = 0;
          for (int indexLeft : leftNeighbors)
            meanIntensityLeft += scanLineCloud[indexLeft].intensity;
          meanIntensityLeft /= leftNeighbors.size();
          // Compute mean intensity on the right
          float meanIntensityRight = 0;
          for (int indexRight : rightNeighbors)
            meanIntensityRight += scanLineCloud[indexRight].intensity;
          meanIntensityRight /= rightNeighbors.size();
          // 将左右强度均值差的绝对值作为当前点的均值
          this->IntensityGap[scanLine][index] = std::abs(meanIntensityLeft - meanIntensityRight);

          // Remove neighbor points to get the best intensity discontinuity locally
          // 和前一个点相比，谁强度小谁被赋值为-1
          if (this->IntensityGap[scanLine][index-1] < this->IntensityGap[scanLine][index])
            this->IntensityGap[scanLine][index-1] = -1;
          else
            this->IntensityGap[scanLine][index] = -1;
        }
      }

      // Check point is not too far from the fitted lines before computing an angle
      // 如果当前点离拟合出的左右直线之间的距离大于一定阈值则跳过
      if (leftLine.DistanceToPoint(centralPoint) > this->MaxDistance || rightLine.DistanceToPoint(centralPoint) > this->MaxDistance)
        continue;

      if (this->Enabled[PLANE] || this->Enabled[EDGE])
      {
        // Compute angles
        // 感觉像将两个拟合直线方向向量的叉乘的模长，也就是夹角的正弦值，作为这个角度
        this->Angles[scanLine][index] = (leftLine.Direction.cross(rightLine.Direction)).norm();
        // Remove previous point from angle inspection if the angle is not maximal locally
        // 夹角要大于60°
        if (this->Enabled[EDGE] && this->Angles[scanLine][index] > this->EdgeSinAngleThreshold)
        {
          // Check previously computed angle to keep only the maximum angle keypoint locally
          for (int indexLeft : leftNeighbors)
          {
            // 如果左邻域点的角度小于当前点的角度，则当前点置-1
            if (this->Angles[scanLine][indexLeft] <= this->Angles[scanLine][index])
              this->Angles[scanLine][indexLeft] = -1;
            else
            {
              this->Angles[scanLine][index] = -1;
              break;
            }
          }
        }
      }
    } // Loop on points
  } // Loop on scanlines
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::AddKptsUsingCriterion (Keypoint k,
                                                             const std::vector<std::vector<float>>& values,
                                                             float threshold,
                                                             bool threshIsMax,
                                                             double weightBasis)
{
  // Loop over the scan lines
  // 按当前帧线束进行遍历
  for (int scanlineIdx = 0; scanlineIdx < static_cast<int>(this->NbLaserRings); ++scanlineIdx)
  {
    const int Npts = this->ScanLines[scanlineIdx]->size();

    // If the line is almost empty, skip it
    // 如果该线点云数量过少（少于9个），则跳过
    if (this->IsScanLineAlmostEmpty(Npts))
      continue;

    // Initialize original index locations
    // Remove indices corresponding to negative values
    // and indices corresponding to values beside threshold
    std::vector<size_t> sortedValuesIndices;
    sortedValuesIndices.reserve(values[scanlineIdx].size());
    // 删除负值对应的索引，筛选出符合阈值的点的索引
    for (int idx = 0; idx < values[scanlineIdx].size(); ++idx)
    {
      if (values[scanlineIdx][idx] > 0 &&
          (threshIsMax  && values[scanlineIdx][idx] < threshold ||
           !threshIsMax && values[scanlineIdx][idx] > threshold))
        sortedValuesIndices.emplace_back(idx);
    }

    // If threshIsMax : ascending order (lowest first)
    // If threshIsMin : descending order (greatest first)
    // 阈值如果是最大值，则按照value值执行升序排序
    Utils::SortIdx(values[scanlineIdx], sortedValuesIndices, threshIsMax, this->MaxPoints);

    // 对排序后的进行遍历
    for (const auto& index: sortedValuesIndices)
    {
      // If the point was already picked, continue
      if (this->Label[scanlineIdx][index][k])
        continue;

      // Check criterion threshold
      bool valueAboveThresh = values[scanlineIdx][index] > threshold;
      // If criterion is not respected, break loop as indices are sorted.
      if ((threshIsMax && valueAboveThresh) ||
          (!threshIsMax && !valueAboveThresh))
        break;

      // The points with the lowest weight have priority for extraction
      float weight = threshIsMax? weightBasis + values[scanlineIdx][index] / threshold
                                : weightBasis - values[scanlineIdx][index] / values[scanlineIdx][0];

      // Indicate the type of the keypoint to debug and to exclude double edges
      this->Label[scanlineIdx][index].set(k);
      // Add keypoint
      this->Keypoints[k].AddPoint(this->ScanLines[scanlineIdx]->at(index), weight);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputePlanes()
{
  this->AddKptsUsingCriterion(Keypoint::PLANE, this->Angles, this->PlaneSinAngleThreshold);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeEdges()
{
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->DepthGap, this->EdgeDepthGapThreshold, false, 1);
  if (this->Keypoints[Keypoint::EDGE].Size() < this->MaxPoints)
    this->AddKptsUsingCriterion(Keypoint::EDGE, this->Angles, this->EdgeSinAngleThreshold, false, 2);
  if (this->Keypoints[Keypoint::EDGE].Size() < this->MaxPoints)
    this->AddKptsUsingCriterion(Keypoint::EDGE, this->SpaceGap, this->EdgeDepthGapThreshold, false, 3);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  this->AddKptsUsingCriterion(Keypoint::INTENSITY_EDGE, this->IntensityGap, this->EdgeIntensityGapThreshold, false);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeBlobs()
{
  // Init random distribution
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (unsigned int scanLine = 0; scanLine < this->NbLaserRings; ++scanLine)
  {
    for (unsigned int index = 0; index < this->ScanLines[scanLine]->size(); ++index)
    {
      // Random sampling to decrease keypoints extraction
      // computation time
      if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
        continue;
      this->Keypoints[Keypoint::BLOB].AddPoint(this->ScanLines[scanLine]->at(index));
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::EstimateAzimuthalResolution()
{
  // Compute horizontal angle values between successive points
  std::vector<float> angles;
  angles.reserve(this->Scan->size());
  // 对每一线的每一个点进行遍历
  for (const PointCloud::Ptr& scanLine : this->ScanLines)
  {
    for (unsigned int index = 1; index < scanLine->size(); ++index)
    {
      // Compute horizontal angle between two measurements
      // WARNING: to be correct, the points need to be in the LIDAR sensor
      // coordinates system, where the sensor is spinning around Z axis.
      // 利用当前点和上一个点来计算两次测量之间的水平角度
      Eigen::Map<const Eigen::Vector2f> p1(scanLine->at(index - 1).data);
      Eigen::Map<const Eigen::Vector2f> p2(scanLine->at(index).data);
      float angle = std::abs(std::acos(p1.dot(p2) / (p1.norm() * p2.norm())));

      // Keep only angles greater than 0 to avoid dual return issues
      if (angle > 1e-4)
        angles.push_back(angle);
    }
  }

  // A minimum number of angles is needed to get a trustable estimator
  // 不足100个点不进行估算
  if (angles.size() < 100)
  {
    PRINT_WARNING("Not enough points to estimate azimuthal resolution");
    return;
  }

  // Estimate azimuthal resolution from these angles
  std::sort(angles.begin(), angles.end());
  unsigned int maxInliersIdx = angles.size();
  float maxAngle = Utils::Deg2Rad(5.);
  float medianAngle = 0.;
  // Iterate until only angles between direct LiDAR beam neighbors remain.
  // The max resolution angle is decreased at each iteration.
  // 每次迭代都会降低最大分辨率角度
  while (maxAngle > 1.8 * medianAngle)
  {
    // 找到数组 angles 中第一个大于 maxAngle 的值，并将其索引保存在 maxInliersIdx 中。此操作基于角度数组已经是排序好的前提
    maxInliersIdx = std::upper_bound(angles.begin(), angles.begin() + maxInliersIdx, maxAngle) - angles.begin();
    // 找到中位角
    medianAngle = angles[maxInliersIdx / 2];
    // 取当前中位角的两倍和之前最大角度的1.8分之1之间的较小值作为新的最大角度
    maxAngle = std::min(medianAngle * 2., maxAngle / 1.8);
  }
  this->AzimuthalResolution = medianAngle;
  std::cout << "LiDAR's azimuthal resolution estimated to " << Utils::Rad2Deg(this->AzimuthalResolution) << "°" << std::endl;
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vector2d)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vector2d, int flag)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      v[i] = vector2d[laserId][indexByScanLine[laserId]][flag];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<float>> map;
  map["sin_angle"]      = get1DVector(this->Angles);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["space_gap"]      = get1DVector(this->SpaceGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["intensity_edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::INTENSITY_EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  return map;
}

} // end of LidarSlam namespace