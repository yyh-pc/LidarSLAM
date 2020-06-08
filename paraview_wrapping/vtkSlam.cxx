//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
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

// LOCAL
#include "vtkSlam.h"
#include "vtkSpinningSensorKeypointExtractor.h"

// VTK
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>

// PCL
#include <pcl/common/transforms.h>

// vtkSlam filter input ports (vtkPolyData and vtkTable)
#define LIDAR_FRAME_INPUT_PORT 0       ///< Current LiDAR frame
#define CALIBRATION_INPUT_PORT 1       ///< LiDAR calibration (vtkTable)
#define INPUT_PORT_COUNT 2

// vtkSlam filter output ports (vtkPolyData)
#define SLAM_FRAME_OUTPUT_PORT 0       ///< Current transformed SLAM frame enriched with debug arrays
#define SLAM_TRAJECTORY_OUTPUT_PORT 1  ///< Trajectory (with position, orientation, covariance and time)
#define EDGE_MAP_OUTPUT_PORT 2         ///< Edge keypoints map
#define PLANE_MAP_OUTPUT_PORT 3        ///< Plane keypoints map
#define BLOB_MAP_OUTPUT_PORT 4         ///< Blob keypoints map
#define EDGE_KEYPOINTS_OUTPUT_PORT 5   ///< Extracted edge keypoints from current frame
#define PLANE_KEYPOINTS_OUTPUT_PORT 6  ///< Extracted plane keypoints from current frame
#define BLOB_KEYPOINTS_OUTPUT_PORT 7   ///< Extracted blob keypoints from current frame
#define OUTPUT_PORT_COUNT 8

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlam)

namespace
{
//-----------------------------------------------------------------------------
template<typename T>
vtkSmartPointer<T> createArray(const std::string& Name, int NumberOfComponents = 1, int NumberOfTuples = 0)
{
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->SetNumberOfComponents(NumberOfComponents);
  array->SetNumberOfTuples(NumberOfTuples);
  array->SetName(Name.c_str());
  return array;
}

//-----------------------------------------------------------------------------
inline double Rad2Deg(double val)
{
  return val / vtkMath::Pi() * 180.;
}

//-----------------------------------------------------------------------------
void PointCloudToPolyData(pcl::PointCloud<Slam::Point>::Ptr pc, vtkPolyData* poly)
{
  const unsigned int nbPoints = pc->size();

  // Set points
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetNumberOfPoints(nbPoints);
  auto intensityArray = createArray<vtkDoubleArray>("intensity", 1, nbPoints);
  for (vtkIdType i = 0; i < nbPoints; ++i)
  {
    const Slam::Point& p = pc->points[i];
    pts->SetPoint(i, p.x, p.y, p.z);
    intensityArray->SetTuple1(i, p.intensity);
    // TODO : add other fields (time, laserId)?
  }
  poly->SetPoints(pts);
  poly->GetPointData()->AddArray(intensityArray);

  // Set cells
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(nbPoints * 2);
  vtkIdType* ids = cells->GetPointer(0);
  for (unsigned int i = 0; i < nbPoints; ++i)
  {
    ids[i * 2] = 1;
    ids[i * 2 + 1] = static_cast<vtkIdType>(i);
  }
  auto cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(nbPoints, cells.GetPointer());
  poly->SetVerts(cellArray);
}

//-----------------------------------------------------------------------------
void PolyDataToPointCloud(vtkPolyData* poly, pcl::PointCloud<Slam::Point>::Ptr pc)
{
  const unsigned int nbPoints = poly->GetNumberOfPoints();

  // Get pointers to arrays
  auto arrayTime = poly->GetPointData()->GetArray("adjustedtime");
  auto arrayLaserId = poly->GetPointData()->GetArray("laser_id");
  auto arrayIntensity = poly->GetPointData()->GetArray("intensity");

  // Loop over points data
  pc->resize(nbPoints);
  pc->header.stamp = arrayTime->GetTuple1(nbPoints - 1); // time in microseconds
  for (vtkIdType i = 0; i < nbPoints; i++)
  {
    Slam::Point& p = pc->points[i];
    double pos[3];
    poly->GetPoint(i, pos);
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];
    p.time = arrayTime->GetTuple1(i) * 1e-6; // time in seconds
    p.laserId = arrayLaserId->GetTuple1(i);
    p.intensity = arrayIntensity->GetTuple1(i);
  }
}

//-----------------------------------------------------------------------------
template<typename T>
std::vector<size_t> sortIdx(const std::vector<T>& v)
{
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });

  return idx;
}
} // end of anonymous namespace

//-----------------------------------------------------------------------------
vtkSlam::vtkSlam()
: SlamAlgo(new Slam)
{
  this->SetNumberOfInputPorts(INPUT_PORT_COUNT);
  this->SetNumberOfOutputPorts(OUTPUT_PORT_COUNT);
  this->Reset();
}

//-----------------------------------------------------------------------------
void vtkSlam::Reset()
{
  this->SlamAlgo->Reset();

  // init the output SLAM trajectory
  this->Trajectory = vtkSmartPointer<vtkPolyData>::New();
  auto pts = vtkSmartPointer<vtkPoints>::New();
  this->Trajectory->SetPoints(pts);
  auto cellArray = vtkSmartPointer<vtkCellArray>::New();
  this->Trajectory->SetLines(cellArray);
  this->Trajectory->GetPointData()->AddArray(createArray<vtkDoubleArray>("Time", 1));
  this->Trajectory->GetPointData()->AddArray(createArray<vtkDoubleArray>("Orientation(Quaternion)", 4));
  this->Trajectory->GetPointData()->AddArray(createArray<vtkDoubleArray>("Orientation(AxisAngle)", 4));
  this->Trajectory->GetPointData()->AddArray(createArray<vtkDoubleArray>("Covariance", 36));

  // add the required array in the trajectory
  if (this->DisplayMode)
  {
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    for (const auto& it : debugInfo)
    {
      this->Trajectory->GetPointData()->AddArray(createArray<vtkDoubleArray>(it.first));
    }
  }

  // Refresh view
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkSlam::RequestData(vtkInformation* vtkNotUsed(request),
                         vtkInformationVector** inputVector,
                         vtkInformationVector* outputVector)
{
  // Get the input
  vtkPolyData* input = vtkPolyData::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
  vtkTable* calib = vtkTable::GetData(inputVector[CALIBRATION_INPUT_PORT], 0);
  std::vector<size_t> laserMapping = GetLaserIdMapping(calib);

  // Conversion vtkPolyData -> PCL pointcloud
  pcl::PointCloud<Slam::Point>::Ptr pc(new pcl::PointCloud<Slam::Point>);
  PolyDataToPointCloud(input, pc);

  // Run SLAM
  this->SlamAlgo->AddFrame(pc, laserMapping);

  // Update Trajectory with new SLAM pose
  this->AddCurrentPoseToTrajectory();

  // ===== SLAM frame and pose =====
  // Output : Current undistorted LiDAR frame in world coordinates
  auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
  slamFrame->ShallowCopy(input);
  Slam::PointCloud::Ptr worldFrame = this->SlamAlgo->GetOutputFrame();
  auto undistortedPoints = vtkSmartPointer<vtkPoints>::New();
  undistortedPoints->SetNumberOfPoints(worldFrame->size());
  for (unsigned int i = 0; i < worldFrame->size(); i++)
  {
    const Slam::Point& p = worldFrame->points[i];
    undistortedPoints->SetPoint(i, p.x, p.y, p.z);
  }
  slamFrame->SetPoints(undistortedPoints);
  // Output : SLAM Trajectory
  auto* slamTrajectory = vtkPolyData::GetData(outputVector, SLAM_TRAJECTORY_OUTPUT_PORT);
  slamTrajectory->ShallowCopy(this->Trajectory);

  // ===== Aggregated Keypoints maps =====
  if (this->OutputKeypointsMaps)
  {
    // Output : Edges points map
    auto* edgeMap = vtkPolyData::GetData(outputVector, EDGE_MAP_OUTPUT_PORT);
    PointCloudToPolyData(this->SlamAlgo->GetEdgesMap(), edgeMap);
    // Output : Planar points map
    auto* planarMap = vtkPolyData::GetData(outputVector, PLANE_MAP_OUTPUT_PORT);
    PointCloudToPolyData(this->SlamAlgo->GetPlanarsMap(), planarMap);
    // Output : Blob points map
    auto* blobMap = vtkPolyData::GetData(outputVector, BLOB_MAP_OUTPUT_PORT);
    PointCloudToPolyData(this->SlamAlgo->GetBlobsMap(), blobMap);
  }

  // ===== Extracted keypoints from current frame =====
  if (this->OutputCurrentKeypoints)
  {
    // Output : Current edge keypoints
    auto* edgePoints = vtkPolyData::GetData(outputVector, EDGE_KEYPOINTS_OUTPUT_PORT);
    PointCloudToPolyData(this->SlamAlgo->GetEdgesKeypoints(this->OutputKeypointsInWorldCoordinates), edgePoints);
    // Output : Current planar keypoints
    auto* planarPoints = vtkPolyData::GetData(outputVector, PLANE_KEYPOINTS_OUTPUT_PORT);
    PointCloudToPolyData(this->SlamAlgo->GetPlanarsKeypoints(this->OutputKeypointsInWorldCoordinates), planarPoints);
    // Output : Current blob keypoints
    auto* blobPoints = vtkPolyData::GetData(outputVector, BLOB_KEYPOINTS_OUTPUT_PORT);
    PointCloudToPolyData(this->SlamAlgo->GetBlobsKeypoints(this->OutputKeypointsInWorldCoordinates), blobPoints);
  }

  // add debug information if displayMode is enabled
  if (this->DisplayMode)
  {
    // Keypoints extraction debug array (curvatures, depth gap, intensity gap...)
    // Info added as PointData array of output0
    auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
    auto keypointsExtractionDebugArray = this->SlamAlgo->GetKeyPointsExtractor()->GetDebugArray();
    for (const auto& it : keypointsExtractionDebugArray)
    {
      auto array = createArray<vtkDoubleArray>(it.first.c_str(), 1, it.second.size());
      // memcpy is a better alternative than looping on all tuples
      std::memcpy(array->GetVoidPointer(0), it.second.data(), sizeof(double) * it.second.size());
      slamFrame->GetPointData()->AddArray(array);
    }

    // General SLAM info (number of keypoints used in ICP and optimization, max variance, ...)
    // Info added as PointData array of output1
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    for (const auto& it : debugInfo)
    {
      slamTrajectory->GetPointData()->GetArray(it.first.c_str())->InsertNextTuple1(it.second);
    }

    // ICP keypoints matching results for ego-motion or mapping steps
    // Info added as PointData array of output5-7
    if (this->OutputCurrentKeypoints)
    {
      auto* edgePoints = vtkPolyData::GetData(outputVector, EDGE_KEYPOINTS_OUTPUT_PORT);
      auto* planarPoints = vtkPolyData::GetData(outputVector, PLANE_KEYPOINTS_OUTPUT_PORT);
      std::unordered_map<std::string, vtkPolyData*> outputMap;
      outputMap["EgoMotion: edges matches"] = edgePoints;
      outputMap["Mapping: edges matches"] = edgePoints;
      outputMap["EgoMotion: planes matches"] = planarPoints;
      outputMap["Mapping: planes matches"] = planarPoints;
      auto debugArray = this->SlamAlgo->GetDebugArray();
      for (const auto& it : outputMap)
      {
        auto array = createArray<vtkDoubleArray>(it.first.c_str(), 1, debugArray[it.first].size());
        // memcpy is a better alternative than looping on all tuples
        std::memcpy(array->GetVoidPointer(0), debugArray[it.first].data(), sizeof(double) * debugArray[it.first].size());
        it.second->GetPointData()->AddArray(array);
      }
    }
  }

  return 1;
}

//-----------------------------------------------------------------------------
void vtkSlam::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Slam parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->SlamAlgo->Get##param() << std::endl;

  PrintParameter(FastSlam)
  PrintParameter(Undistortion)
  PrintParameter(NbThreads)
  PrintParameter(Verbosity)

  PrintParameter(MaxDistanceForICPMatching)

  PrintParameter(EgoMotionLMMaxIter)
  PrintParameter(EgoMotionICPMaxIter)
  PrintParameter(EgoMotionLineDistanceNbrNeighbors)
  PrintParameter(EgoMotionMinimumLineNeighborRejection)
  PrintParameter(EgoMotionMaxLineDistance)
  PrintParameter(EgoMotionLineDistancefactor)
  PrintParameter(EgoMotionPlaneDistanceNbrNeighbors)
  PrintParameter(EgoMotionMaxPlaneDistance)
  PrintParameter(EgoMotionPlaneDistancefactor1)
  PrintParameter(EgoMotionPlaneDistancefactor2)
  PrintParameter(EgoMotionInitLossScale)
  PrintParameter(EgoMotionFinalLossScale)

  PrintParameter(MappingLMMaxIter)
  PrintParameter(MappingICPMaxIter)
  PrintParameter(MappingLineDistanceNbrNeighbors)
  PrintParameter(MappingMinimumLineNeighborRejection)
  PrintParameter(MappingMaxLineDistance)
  PrintParameter(MappingLineMaxDistInlier)
  PrintParameter(MappingLineDistancefactor)
  PrintParameter(MappingPlaneDistanceNbrNeighbors)
  PrintParameter(MappingPlaneDistancefactor1)
  PrintParameter(MappingPlaneDistancefactor2)
  PrintParameter(MappingMaxPlaneDistance)
  PrintParameter(MappingInitLossScale)
  PrintParameter(MappingFinalLossScale)

  this->GetKeyPointsExtractor()->PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
int vtkSlam::FillInputPortInformation(int port, vtkInformation* info)
{
  // Pointcloud data
  if (port == LIDAR_FRAME_INPUT_PORT)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");
    return 1;
  }
  // LiDAR calibration
  if (port == CALIBRATION_INPUT_PORT)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTable");
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
std::vector<size_t> vtkSlam::GetLaserIdMapping(vtkTable* calib)
{
  auto array = vtkDataArray::SafeDownCast(calib->GetColumnByName("verticalCorrection"));
  std::vector<size_t> laserIdMapping;
  if (array)
  {
    std::vector<double> verticalCorrection(array->GetNumberOfTuples());
    for (int i = 0; i < array->GetNumberOfTuples(); ++i)
    {
      verticalCorrection[i] = array->GetTuple1(i);
    }
    laserIdMapping = sortIdx(verticalCorrection);
  }
  else
  {
    vtkErrorMacro(<< "The calibration data has no column named 'verticalCorrection'");
  }
  return laserIdMapping;
}

//-----------------------------------------------------------------------------
void vtkSlam::AddCurrentPoseToTrajectory()
{
  // Get current SLAM pose in WORLD coordinates
  Transform Tworld = this->SlamAlgo->GetWorldTransform();
  Eigen::Isometry3d pose = Tworld.GetIsometry();

  // Add position
  Eigen::Vector3d translation = pose.translation();
  this->Trajectory->GetPoints()->InsertNextPoint(translation.x(), translation.y(), translation.z());

  // Add orientation as quaternion
  Eigen::Quaterniond quaternion(pose.linear());
  double wxyz[] = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  this->Trajectory->GetPointData()->GetArray("Orientation(Quaternion)")->InsertNextTuple(wxyz);

  // Add orientation as axis angle
  Eigen::AngleAxisd angleAxis(pose.linear());
  Eigen::Vector3d axis = angleAxis.axis();
  double xyza[] = {axis.x(), axis.y(), axis.z(), angleAxis.angle()};
  this->Trajectory->GetPointData()->GetArray("Orientation(AxisAngle)")->InsertNextTuple(xyza);

  // Add pose time and covariance
  this->Trajectory->GetPointData()->GetArray("Time")->InsertNextTuple(&Tworld.time);
  this->Trajectory->GetPointData()->GetArray("Covariance")->InsertNextTuple(this->SlamAlgo->GetTransformCovariance().data());

  // Add line linking 2 successive points
  vtkIdType nPoints = this->Trajectory->GetNumberOfPoints();
  if (nPoints >= 2)
  {
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0, nPoints - 2);
    line->GetPointIds()->SetId(1, nPoints - 1);
    this->Trajectory->GetLines()->InsertNextCell(line);
  }
}

// =============================================================================
//   Getters / setters
// =============================================================================

//-----------------------------------------------------------------------------
int vtkSlam::GetEgoMotion()
{
  int egoMotion = static_cast<int>(this->SlamAlgo->GetEgoMotion());
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): returning Ego-Motion of " << egoMotion);
  return egoMotion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetEgoMotion(int mode)
{
  Slam::EgoMotionMode egoMotion = static_cast<Slam::EgoMotionMode>(mode);
  if (egoMotion != Slam::EgoMotionMode::NONE         && egoMotion != Slam::EgoMotionMode::MOTION_EXTRAPOLATION &&
      egoMotion != Slam::EgoMotionMode::REGISTRATION && egoMotion != Slam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION)
  {
    vtkErrorMacro("Invalid ego-motion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting Ego-Motion to " << mode);
  if (this->SlamAlgo->GetEgoMotion() != egoMotion)
  {
    this->SlamAlgo->SetEgoMotion(egoMotion);
    this->Modified();
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetUndistortion()
{
  int undistortion = static_cast<int>(this->SlamAlgo->GetUndistortion());
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): returning Undistortion of " << undistortion);
  return undistortion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetUndistortion(int mode)
{
  Slam::UndistortionMode undistortion = static_cast<Slam::UndistortionMode>(mode);
  if (undistortion != Slam::NONE && undistortion != Slam::APPROXIMATED && undistortion != Slam::OPTIMIZED)
  {
    vtkErrorMacro("Invalid undistortion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting Undistortion to " << mode);
  if (this->SlamAlgo->GetUndistortion() != undistortion)
  {
    this->SlamAlgo->SetUndistortion(undistortion);
    this->Modified();
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarTranslation(double x, double y, double z)
{
  Eigen::Translation3d trans(x, y, z);
  Eigen::Quaterniond quat(this->SlamAlgo->GetBaseToLidarOffset().linear());
  Eigen::Isometry3d baseToLidar(trans * quat);
  this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarRotation(double rx, double ry, double rz)
{
  Eigen::Vector3d trans = this->SlamAlgo->GetBaseToLidarOffset().translation();
  Eigen::Vector3d rpy(rx, ry, rz);
  Eigen::Isometry3d baseToLidar = Transform(trans, rpy * vtkMath::Pi() / 180.).GetIsometry();
  this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor* _arg)
{
  vtkSetObjectBodyMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor, _arg);
  this->SlamAlgo->SetKeyPointsExtractor(this->KeyPointsExtractor->GetExtractor());
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridLeafSizeEdges(double size)
{
  this->SlamAlgo->SetVoxelGridLeafSizeEdges(size);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridLeafSizePlanes(double size)
{
  this->SlamAlgo->SetVoxelGridLeafSizePlanes(size);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridLeafSizeBlobs(double size)
{
  this->SlamAlgo->SetVoxelGridLeafSizeBlobs(size);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridSize(int size)
{
  this->SlamAlgo->SetVoxelGridSize(size);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridResolution(double resolution)
{
  this->SlamAlgo->SetVoxelGridResolution(resolution);
  this->ParametersModificationTime.Modified();
}