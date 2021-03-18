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
#include <vtkFloatArray.h>
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

#define IF_VERBOSE(minVerbosityLevel, command) if (this->SlamAlgo->GetVerbosity() >= (minVerbosityLevel)) { command; }

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlam)

namespace Utils
{
namespace
{
// Import helper functions from LidarSlam
using namespace LidarSlam::Utils;

//-----------------------------------------------------------------------------
template<typename T>
vtkSmartPointer<T> CreateArray(const std::string& Name, int NumberOfComponents = 1, int NumberOfTuples = 0)
{
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->SetNumberOfComponents(NumberOfComponents);
  array->SetNumberOfTuples(NumberOfTuples);
  array->SetName(Name.c_str());
  return array;
}
} // end of anonymous namespace
} // end of Utils namespace

//-----------------------------------------------------------------------------
vtkSlam::vtkSlam()
: SlamAlgo(new LidarSlam::Slam)
{
  this->SetNumberOfInputPorts(INPUT_PORT_COUNT);
  this->SetNumberOfOutputPorts(OUTPUT_PORT_COUNT);
  // If auto-detect mode is disabled, user needs to specify input arrays to use
  this->SetInputArrayToProcess(0, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);
  this->SetInputArrayToProcess(1, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);
  this->SetInputArrayToProcess(2, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);
  this->SetInputArrayToProcess(3, CALIBRATION_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);
  this->Reset();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s)
{
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting VoxelGridLeafSize to " << _arg);
  this->SlamAlgo->SetVoxelGridLeafSize(k, s);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::Reset()
{
  this->SlamAlgo->Reset(true);

  // init the output SLAM trajectory
  this->Trajectory = vtkSmartPointer<vtkPolyData>::New();
  auto pts = vtkSmartPointer<vtkPoints>::New();
  this->Trajectory->SetPoints(pts);
  auto cellArray = vtkSmartPointer<vtkCellArray>::New();
  this->Trajectory->SetLines(cellArray);
  this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Time", 1));
  this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(Quaternion)", 4));
  this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(AxisAngle)", 4));
  this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Covariance", 36));

  // Add the optional arrays to the trajectory
  if (this->AdvancedReturnMode)
  {
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    for (const auto& it : debugInfo)
    {
      this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>(it.first));
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
  IF_VERBOSE(1, Utils::Timer::Init("vtkSlam"));
  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : input conversions"));

  // Get the input
  vtkPolyData* input = vtkPolyData::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
  vtkTable* calib = vtkTable::GetData(inputVector[CALIBRATION_INPUT_PORT], 0);
  this->IdentifyInputArrays(input, calib);
  std::vector<size_t> laserMapping = GetLaserIdMapping(calib);

  // Conversion vtkPolyData -> PCL pointcloud
  LidarSlam::Slam::PointCloud::Ptr pc(new LidarSlam::Slam::PointCloud);
  bool allPointsAreValid = this->PolyDataToPointCloud(input, pc, laserMapping);

  // Run SLAM
  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));
  this->SlamAlgo->AddFrame(pc);
  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : basic output conversions"));

  // Update Trajectory with new SLAM pose
  this->AddCurrentPoseToTrajectory();

  // ===== SLAM frame and pose =====
  // Output : Current undistorted LiDAR frame in world coordinates
  auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
  slamFrame->ShallowCopy(input);
  auto worldFrame = this->SlamAlgo->GetOutputFrame();
  vtkIdType nbPoints = input->GetNumberOfPoints();
  // Modify only points coordinates to keep input arrays
  auto registeredPoints = vtkSmartPointer<vtkPoints>::New();
  registeredPoints->SetNumberOfPoints(nbPoints);
  slamFrame->SetPoints(registeredPoints);
  if (allPointsAreValid)
  {
    for (vtkIdType i = 0; i < nbPoints; i++)
      registeredPoints->SetPoint(i, worldFrame->at(i).data);
  }
  else
  {
    unsigned int validFrameIndex = 0;
    for (vtkIdType i = 0; i < nbPoints; i++)
    {
      // Modify point only if valid
      double pos[3];
      input->GetPoint(i, pos);
      if (pos[0] || pos[1] || pos[2])
      {
        const auto& p = worldFrame->points[validFrameIndex++];
        registeredPoints->SetPoint(i, p.data);
      }
      else
        registeredPoints->SetPoint(i, pos);
    }
  }

  // Output : SLAM Trajectory
  auto* slamTrajectory = vtkPolyData::GetData(outputVector, SLAM_TRAJECTORY_OUTPUT_PORT);
  slamTrajectory->ShallowCopy(this->Trajectory);

  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : basic output conversions"));

  // ===== Aggregated Keypoints maps =====
  if (this->OutputKeypointsMaps)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output keypoints maps"));

    // Cache maps to update them only every MapsUpdateStep frames
    static vtkPolyData* cacheEdgeMap = vtkPolyData::New();
    static vtkPolyData* cachePlanarMap = vtkPolyData::New();
    static vtkPolyData* cacheBlobMap = vtkPolyData::New();
    if ((this->SlamAlgo->GetNbrFrameProcessed() - 1) % this->MapsUpdateStep == 0)
    {
      this->PointCloudToPolyData(this->SlamAlgo->GetMap(LidarSlam::EDGE), cacheEdgeMap);
      this->PointCloudToPolyData(this->SlamAlgo->GetMap(LidarSlam::PLANE), cachePlanarMap);
      this->PointCloudToPolyData(this->SlamAlgo->GetMap(LidarSlam::BLOB), cacheBlobMap);
    }

    // Fill outputs from cache
    // Output : Edge points map
    auto* edgeMap = vtkPolyData::GetData(outputVector, EDGE_MAP_OUTPUT_PORT);
    edgeMap->ShallowCopy(cacheEdgeMap);
    // Output : Planar points map
    auto* planarMap = vtkPolyData::GetData(outputVector, PLANE_MAP_OUTPUT_PORT);
    planarMap->ShallowCopy(cachePlanarMap);
    // Output : Blob points map
    auto* blobMap = vtkPolyData::GetData(outputVector, BLOB_MAP_OUTPUT_PORT);
    blobMap->ShallowCopy(cacheBlobMap);

    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output keypoints maps"));
  }

  // ===== Extracted keypoints from current frame =====
  if (this->OutputCurrentKeypoints)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output current keypoints"));
    // Output : Current edge keypoints
    auto* edgePoints = vtkPolyData::GetData(outputVector, EDGE_KEYPOINTS_OUTPUT_PORT);
    this->PointCloudToPolyData(this->SlamAlgo->GetKeypoints(LidarSlam::EDGE, this->OutputKeypointsInWorldCoordinates), edgePoints);
    // Output : Current planar keypoints
    auto* planarPoints = vtkPolyData::GetData(outputVector, PLANE_KEYPOINTS_OUTPUT_PORT);
    this->PointCloudToPolyData(this->SlamAlgo->GetKeypoints(LidarSlam::PLANE, this->OutputKeypointsInWorldCoordinates), planarPoints);
    // Output : Current blob keypoints
    auto* blobPoints = vtkPolyData::GetData(outputVector, BLOB_KEYPOINTS_OUTPUT_PORT);
    this->PointCloudToPolyData(this->SlamAlgo->GetKeypoints(LidarSlam::BLOB, this->OutputKeypointsInWorldCoordinates), blobPoints);
    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output current keypoints"));
  }

  // Add debug information if advanced return mode is enabled
  if (this->AdvancedReturnMode)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : add advanced return arrays"));

    // Keypoints extraction debug array (curvatures, depth gap, intensity gap...)
    // Arrays added to WORLD transformed frame output
    auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
    auto keypointsExtractionDebugArray = this->SlamAlgo->GetKeyPointsExtractor()->GetDebugArray();
    for (const auto& it : keypointsExtractionDebugArray)
    {
      auto array = Utils::CreateArray<vtkFloatArray>(it.first.c_str(), 1, nbPoints);
      slamFrame->GetPointData()->AddArray(array);

      // Fill array values from debug data
      // memcpy is a better alternative than looping on all tuples
      // but can only be used if the arrays use contiguous storage
      if (allPointsAreValid)
        std::memcpy(array->GetVoidPointer(0), it.second.data(), sizeof(float) * it.second.size());

      // Otherwise, we need to loop over each point and test if it is valid.
      // NOTE: this is slow, but we accept it as this mode is only used for debug purpose.
      else
      {
        unsigned int validFrameIndex = 0;
        for (vtkIdType i = 0; i < nbPoints; i++)
        {
          // Add array value only if point coordinates are non empty
          double pos[3];
          input->GetPoint(i, pos);
          array->SetTuple1(i, (pos[0] || pos[1] || pos[2]) ? it.second[validFrameIndex++] : 0.);
        }
      }
    }

    // General SLAM info (number of keypoints used in ICP and optimization, max variance, ...)
    // Arrays added to trajectory output
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    for (const auto& it : debugInfo)
    {
      slamTrajectory->GetPointData()->GetArray(it.first.c_str())->InsertNextTuple1(it.second);
    }

    // ICP keypoints matching results for ego-motion registration or localization steps
    // Arrays added to keypoints extracted from current frame outputs
    if (this->OutputCurrentKeypoints)
    {
      auto* edgePoints = vtkPolyData::GetData(outputVector, EDGE_KEYPOINTS_OUTPUT_PORT);
      auto* planarPoints = vtkPolyData::GetData(outputVector, PLANE_KEYPOINTS_OUTPUT_PORT);
      std::unordered_map<std::string, vtkPolyData*> outputMap;
      outputMap["EgoMotion: edge matches"] = edgePoints;
      outputMap["EgoMotion: edge weights"] = edgePoints;
      outputMap["Localization: edge matches"] = edgePoints;
      outputMap["Localization: edge weights"] = edgePoints;
      outputMap["EgoMotion: plane matches"] = planarPoints;
      outputMap["EgoMotion: plane weights"] = planarPoints;
      outputMap["Localization: plane matches"] = planarPoints;
      outputMap["Localization: plane weights"] = planarPoints;
      auto debugArray = this->SlamAlgo->GetDebugArray();
      for (const auto& it : outputMap)
      {
        auto array = Utils::CreateArray<vtkDoubleArray>(it.first.c_str(), 1, debugArray[it.first].size());
        // memcpy is a better alternative than looping on all tuples
        std::memcpy(array->GetVoidPointer(0), debugArray[it.first].data(), sizeof(double) * debugArray[it.first].size());
        it.second->GetPointData()->AddArray(array);
      }
    }

    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : add advanced return arrays"));
  }

  IF_VERBOSE(1, Utils::Timer::StopAndDisplay("vtkSlam"));

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
  PrintParameter(EgoMotionInitSaturationDistance)
  PrintParameter(EgoMotionFinalSaturationDistance)

  PrintParameter(LocalizationLMMaxIter)
  PrintParameter(LocalizationICPMaxIter)
  PrintParameter(LocalizationLineDistanceNbrNeighbors)
  PrintParameter(LocalizationMinimumLineNeighborRejection)
  PrintParameter(LocalizationMaxLineDistance)
  PrintParameter(LocalizationLineDistancefactor)
  PrintParameter(LocalizationPlaneDistanceNbrNeighbors)
  PrintParameter(LocalizationPlaneDistancefactor1)
  PrintParameter(LocalizationPlaneDistancefactor2)
  PrintParameter(LocalizationMaxPlaneDistance)
  PrintParameter(LocalizationInitSaturationDistance)
  PrintParameter(LocalizationFinalSaturationDistance)

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
vtkMTimeType vtkSlam::GetMTime()
{
  return std::max(this->Superclass::GetMTime(), this->ParametersModificationTime.GetMTime());
}

// =============================================================================
//   Useful helpers
// =============================================================================

//-----------------------------------------------------------------------------
void vtkSlam::IdentifyInputArrays(vtkPolyData* poly, vtkTable* calib)
{
  // Try to auto-detect LiDAR model by checking available arrays
  if (this->AutoDetectInputArrays)
  {
    // Check if requested arrays exist and set them if they are valid
    auto checkAndSetArrays = [&](const char* time, const char* intensity, const char* laserId, const char* angles)
    {
      bool valid = poly->GetPointData()->HasArray(time) &&
                   poly->GetPointData()->HasArray(intensity) &&
                   poly->GetPointData()->HasArray(laserId) &&
                   calib->GetRowData()->HasArray(angles);
      if (valid)
      {
        this->TimeArrayName = time;
        this->IntensityArrayName = intensity;
        this->LaserIdArrayName = laserId;
        this->VerticalCalibArrayName = angles;
      }
      return valid;
    };

    // Check some keypoints extraction parameters values at SLAM initialization
    #define CheckKEParameter(vendor, parameter, condition) \
      if (this->Trajectory->GetNumberOfPoints() == 0 && \
          !(this->SlamAlgo->GetKeyPointsExtractor()->Get ##parameter() condition)) \
        { vtkWarningMacro(<< "SLAM run with " vendor " data: consider using " #parameter " " #condition); }

    // Test if LiDAR data is Velodyne
    if (checkAndSetArrays("adjustedtime", "intensity", "laser_id", "verticalCorrection"))
    {
      this->TimeToSecondsFactor = 1e-6;
      CheckKEParameter("Velodyne", EdgeIntensityGapThreshold, < 100);
    }

    // Test if LiDAR data is Ouster
    else if (checkAndSetArrays("Raw Timestamp", "Signal Photons", "Channel", "Altitude Angles"))
    {
      this->TimeToSecondsFactor = 1e-9;
      CheckKEParameter("Ouster", EdgeIntensityGapThreshold, >= 100);
      CheckKEParameter("Ouster", NeighborWidth, > 4);
    }

    // Failed to recognize LiDAR vendor
    else
      vtkErrorMacro(<< "Unable to identify LiDAR arrays to use.");
  }

  // Otherwise, user needs to specify which arrays to use
  else
  {
    this->TimeArrayName          = this->GetInputArrayToProcess(0, poly)->GetName();
    this->IntensityArrayName     = this->GetInputArrayToProcess(1, poly)->GetName();
    this->LaserIdArrayName       = this->GetInputArrayToProcess(2, poly)->GetName();
    this->VerticalCalibArrayName = this->GetInputArrayToProcess(3, calib)->GetName();
    this->TimeToSecondsFactor    = this->TimeToSecondsFactorSetting;
  }
}

//-----------------------------------------------------------------------------
std::vector<size_t> vtkSlam::GetLaserIdMapping(vtkTable* calib)
{
  std::vector<size_t> laserIdMapping;
  auto array = vtkDataArray::SafeDownCast(calib->GetColumnByName(this->VerticalCalibArrayName.c_str()));
  if (array)
  {
    std::vector<double> verticalAngle(array->GetNumberOfTuples());
    for (vtkIdType i = 0; i < array->GetNumberOfTuples(); ++i)
      verticalAngle[i] = array->GetTuple1(i);
    auto sortedLaserIds = Utils::SortIdx(verticalAngle);
    laserIdMapping = Utils::SortIdx(sortedLaserIds);
  }
  return laserIdMapping;
}

//-----------------------------------------------------------------------------
void vtkSlam::AddCurrentPoseToTrajectory()
{
  // Get current SLAM pose in WORLD coordinates
  LidarSlam::Transform Tworld = this->SlamAlgo->GetWorldTransform();
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

//-----------------------------------------------------------------------------
bool vtkSlam::PolyDataToPointCloud(vtkPolyData* poly,
                                   LidarSlam::Slam::PointCloud::Ptr pc,
                                   const std::vector<size_t>& laserIdMapping = {}) const
{
  const vtkIdType nbPoints = poly->GetNumberOfPoints();
  const bool useLaserIdMapping = !laserIdMapping.empty();
  
  // Get pointers to arrays
  auto arrayTime = poly->GetPointData()->GetArray(this->TimeArrayName.c_str());
  auto arrayLaserId = poly->GetPointData()->GetArray(this->LaserIdArrayName.c_str());
  auto arrayIntensity = poly->GetPointData()->GetArray(this->IntensityArrayName.c_str());

  // Loop over points data
  pc->reserve(nbPoints);
  double frameEndTime = arrayTime->GetRange()[1];
  pc->header.stamp = frameEndTime * (this->TimeToSecondsFactor * 1e6); // max time in microseconds
  bool allPointsAreValid = true;
  for (vtkIdType i = 0; i < nbPoints; i++)
  {
    // Get point coordinates
    double pos[3];
    poly->GetPoint(i, pos);
    // Check that points coordinates are not null before adding point
    if (pos[0] || pos[1] || pos[2])
    {
      LidarSlam::Slam::Point p;
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];
      p.time = (arrayTime->GetTuple1(i) - frameEndTime) * this->TimeToSecondsFactor; // time in seconds
      p.laser_id = useLaserIdMapping ? laserIdMapping[arrayLaserId->GetTuple1(i)] : arrayLaserId->GetTuple1(i);
      p.intensity = arrayIntensity->GetTuple1(i);
      pc->push_back(p);
    }
    else
      allPointsAreValid = false;
  }

  return allPointsAreValid;
}

//-----------------------------------------------------------------------------
void vtkSlam::PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr pc, vtkPolyData* poly) const
{
  const vtkIdType nbPoints = pc->size();

  // Init and register points
  vtkNew<vtkPoints> pts;
  pts->SetNumberOfPoints(nbPoints);
  poly->SetPoints(pts);
  auto intensityArray = Utils::CreateArray<vtkDoubleArray>(this->IntensityArrayName.c_str(), 1, nbPoints);
  poly->GetPointData()->AddArray(intensityArray);

  // Init and register cells
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(nbPoints * 2);
  vtkNew<vtkCellArray> cellArray;
  cellArray->SetCells(nbPoints, cells);
  poly->SetVerts(cellArray);

  // Fill points and cells values
  for (vtkIdType i = 0; i < nbPoints; ++i)
  {
    // Set point
    const auto& p = pc->points[i];
    pts->SetPoint(i, p.x, p.y, p.z);
    intensityArray->SetTuple1(i, p.intensity);
    // TODO : add other fields (time, laserId)?

    // Set cell
    cells->SetValue(i * 2,     1);
    cells->SetValue(i * 2 + 1, i);
  }
}

// =============================================================================
//   Getters / setters
// =============================================================================

//-----------------------------------------------------------------------------
void vtkSlam::SetAdvancedReturnMode(bool _arg)
{
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting AdvancedReturnMode to " << _arg);
  if (this->AdvancedReturnMode != _arg)
  {
    auto debugInfo = this->SlamAlgo->GetDebugInformation();

    // If AdvancedReturnMode is being activated
    if (_arg)
    {
      // Add new optional arrays to trajectory, and init past values to 0.
      for (const auto& it : debugInfo)
      {
        auto array = Utils::CreateArray<vtkDoubleArray>(it.first, 1, this->Trajectory->GetNumberOfPoints());
        for (vtkIdType i = 0; i < this->Trajectory->GetNumberOfPoints(); i++)
          array->SetTuple1(i, 0.);
        this->Trajectory->GetPointData()->AddArray(array);
      }
    }

    // If AdvancedReturnMode is being disabled
    else
    {
      // Delete optional arrays
      for (const auto& it : debugInfo)
        this->Trajectory->GetPointData()->RemoveArray(it.first.c_str());
    }

    this->AdvancedReturnMode = _arg;
    this->ParametersModificationTime.Modified();
  }
}

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
  LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(mode);
  if (egoMotion != LidarSlam::EgoMotionMode::NONE         &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION &&
      egoMotion != LidarSlam::EgoMotionMode::REGISTRATION &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION)
  {
    vtkErrorMacro("Invalid ego-motion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting Ego-Motion to " << mode);
  if (this->SlamAlgo->GetEgoMotion() != egoMotion)
  {
    this->SlamAlgo->SetEgoMotion(egoMotion);
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
  LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(mode);
  if (undistortion != LidarSlam::UndistortionMode::NONE &&
      undistortion != LidarSlam::UndistortionMode::ONCE &&
      undistortion != LidarSlam::UndistortionMode::REFINED)
  {
    vtkErrorMacro("Invalid undistortion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting Undistortion to " << mode);
  if (this->SlamAlgo->GetUndistortion() != undistortion)
  {
    this->SlamAlgo->SetUndistortion(undistortion);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarTranslation(double x, double y, double z)
{
  Eigen::Isometry3d baseToLidar = this->SlamAlgo->GetBaseToLidarOffset();
  baseToLidar.translation() = Eigen::Vector3d(x, y, z);
  this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarRotation(double rx, double ry, double rz)
{
  Eigen::Isometry3d baseToLidar = this->SlamAlgo->GetBaseToLidarOffset();
  baseToLidar.linear() = Utils::RPYtoRotationMatrix(Utils::Deg2Rad(rx), Utils::Deg2Rad(ry), Utils::Deg2Rad(rz));
  this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor* _arg)
{
  vtkSetObjectBodyMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor, _arg);
  this->SlamAlgo->SetKeyPointsExtractor(this->KeyPointsExtractor->GetExtractor());
  this->ParametersModificationTime.Modified();
}