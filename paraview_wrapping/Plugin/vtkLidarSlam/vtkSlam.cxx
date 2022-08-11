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
#include <vtkDelimitedTextReader.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTable.h>

// PCL
#include <pcl/common/transforms.h>

//Boost
// TODO : replace by std when passing to C++17 minimum
#include <boost/filesystem.hpp>

// vtkSlam filter input ports (vtkPolyData and vtkTable)
#define LIDAR_FRAME_INPUT_PORT 0       ///< Current LiDAR frame
#define CALIBRATION_INPUT_PORT 1       ///< LiDAR calibration (vtkTable)
#define INPUT_PORT_COUNT 2

// vtkSlam filter output ports (vtkPolyData)
#define SLAM_FRAME_OUTPUT_PORT 0       ///< Current transformed SLAM frame enriched with debug arrays
#define SLAM_TRAJECTORY_OUTPUT_PORT 1  ///< Trajectory (with position, orientation, covariance and time)
#define EDGE_MAP_OUTPUT_PORT 2         ///< Edge keypoints map
#define INTENSITY_EDGE_MAP_OUTPUT_PORT 3         ///< intensity edge keypoints map
#define PLANE_MAP_OUTPUT_PORT 4        ///< Plane keypoints map
#define EDGE_KEYPOINTS_OUTPUT_PORT 5   ///< Extracted edge keypoints from current frame
#define INTENSITY_EDGE_KEYPOINTS_OUTPUT_PORT 6         ///< intensity edge keypoints map
#define PLANE_KEYPOINTS_OUTPUT_PORT 7  ///< Extracted plane keypoints from current frame
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
  this->SetInputArrayToProcess(3, CALIBRATION_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_ROWS,   vtkDataSetAttributes::SCALARS);
  this->Reset();

  // Enable overlap computation only if advanced return mode is activated
  this->SlamAlgo->SetOverlapSamplingRatio(this->AdvancedReturnMode ? this->OverlapSamplingRatio : 0.);
  // Enable motion limitation checks if advanced return mode is activated
  this->SlamAlgo->SetTimeWindowDuration(this->AdvancedReturnMode ? this->TimeWindowDuration : 0.);
  // Log the necessary poses if advanced return mode is activated
  this->SlamAlgo->SetLoggingTimeout(this->AdvancedReturnMode ? 1.1 * this->TimeWindowDuration : 0.);
}

//-----------------------------------------------------------------------------
void vtkSlam::Reset()
{
  this->SlamAlgo->Reset(true);

  // Init the SLAM state (map + pose)
  if (!this->InitMapPrefix.empty())
    this->SlamAlgo->LoadMapsFromPCD(this->InitMapPrefix);
  this->SlamAlgo->SetWorldTransformFromGuess(LidarSlam::Utils::XYZRPYtoIsometry(this->InitPose));

  // Init the output SLAM trajectory
  this->ResetTrajectory();

  // Add the optional arrays to the trajectory
  if (this->AdvancedReturnMode)
  {
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    for (const auto& it : debugInfo)
      this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>(it.first));
  }

  // Refill sensor managers
  this->SetSensorData(this->ExtSensorFileName);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::RebuildMaps()
{
  this->SlamAlgo->UpdateMaps(this->ResetMaps);
  PRINT_INFO("Rebuild maps finished.")

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::OptimizeGraph()
{
  const std::list<LidarSlam::LidarState>& initLidarStates = this->SlamAlgo->GetLogStates();
  if (initLidarStates.size() < 2)
    return;
  this->SlamAlgo->OptimizeGraph();
  // Update trajectory from the first timestamp of lidarStates with new poses after PGO.
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  this->ResetTrajectory(lidarStates.front().Time);
  for (auto const& state: lidarStates)
    this->AddPoseToTrajectory(state);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialMap(const std::string& mapsPathPrefix)
{
  this->InitMapPrefix = mapsPathPrefix;
  if (this->InitMapPrefix.empty())
    return;
  if (this->InitMapPrefix.substr(this->InitMapPrefix.find('.') + 1, this->InitMapPrefix.size()) == "pcd")
    vtkErrorMacro(<< "Could not load the initial map : only the prefix path must be supplied (not the complete path)");
  this->SlamAlgo->LoadMapsFromPCD(this->InitMapPrefix);
  this->ParametersModificationTime.Modified();
  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPoseTranslation(double x, double y, double z)
{
  vtkDebugMacro(<< "Setting InitialPoseTranslation to " << x << " " << y << " " << z);
  this->InitPose.x() = x;
  this->InitPose.y() = y;
  this->InitPose.z() = z;
  this->SlamAlgo->SetWorldTransformFromGuess(LidarSlam::Utils::XYZRPYtoIsometry(this->InitPose));
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPoseRotation(double roll, double pitch, double yaw)
{
  vtkDebugMacro(<< "Setting InitialPoseRotation to " << roll << " " << pitch << " " << yaw);
  this->InitPose(3) = roll;
  this->InitPose(4) = pitch;
  this->InitPose(5) = yaw;
  this->SlamAlgo->SetWorldTransformFromGuess(LidarSlam::Utils::XYZRPYtoIsometry(this->InitPose));
  this->ParametersModificationTime.Modified();
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
  // Check if input is a multiblock
  if (!input)
  {
    vtkMultiBlockDataSet* mb = vtkMultiBlockDataSet::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
    // Extract first block if it is a vtkPolyData
    input = vtkPolyData::SafeDownCast(mb->GetBlock(0));
  }
  // If the input could not be cast, return
  if (!input)
  {
    vtkErrorMacro(<< "Unable to cast input into a vtkPolyData");
    return 0;
  }
  vtkTable* calib = vtkTable::GetData(inputVector[CALIBRATION_INPUT_PORT], 0);
  this->IdentifyInputArrays(input, calib);
  std::vector<size_t> laserMapping = GetLaserIdMapping(calib);

  // Conversion vtkPolyData -> PCL pointcloud
  LidarSlam::Slam::PointCloud::Ptr pc(new LidarSlam::Slam::PointCloud);
  bool allPointsAreValid = this->PolyDataToPointCloud(input, pc, laserMapping);

  auto arrayTime = input->GetPointData()->GetArray(this->TimeArrayName.c_str());
  // Get frame first point time in vendor format
  double* range = arrayTime->GetRange();
  double frameFirstPointTime = range[0] * this->TimeToSecondsFactor;
  if (this->SynchronizeOnPacket)
  {
    // Get first frame packet reception time
    vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
    double frameReceptionPOSIXTime = inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
    double absCurrentOffset = std::abs(this->SlamAlgo->GetSensorTimeOffset());
    double potentialOffset = frameFirstPointTime - frameReceptionPOSIXTime;
    // We exclude the first frame cause frameReceptionPOSIXTime can be badly set
    if (this->SlamAlgo->GetNbrFrameProcessed() > 0 && (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset))
      this->SlamAlgo->SetSensorTimeOffset(potentialOffset);
  }

  // Run SLAM
  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));
  this->SlamAlgo->AddFrame(pc);
  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : basic output conversions"));

  // Update Trajectory with new SLAM pose
  this->AddPoseToTrajectory(this->SlamAlgo->GetLastState());

  // ===== SLAM frame and pose =====
  // Output : Current undistorted LiDAR frame in world coordinates
  auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
  slamFrame->ShallowCopy(input);
  auto worldFrame = this->SlamAlgo->GetRegisteredFrame();
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
  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output keypoints maps"));

  // Get the previous outputs
  auto* edgeMap          = vtkPolyData::GetData(outputVector, EDGE_MAP_OUTPUT_PORT);
  auto* planarMap        = vtkPolyData::GetData(outputVector, PLANE_MAP_OUTPUT_PORT);
  auto* intensityEdgeMap = vtkPolyData::GetData(outputVector, INTENSITY_EDGE_MAP_OUTPUT_PORT);

  if ((this->SlamAlgo->GetNbrFrameProcessed() - 1) % this->MapsUpdateStep == 0)
  {
    // Cache maps to update them only every MapsUpdateStep frames
    for (auto k : LidarSlam::KeypointTypes)
      this->CacheMaps[k] = vtkSmartPointer<vtkPolyData>::New();

    // The expected maps can be the whole maps or the submaps
    // If the maps is fixed by the user, the whole map and the submap are equal but the submap is outputed (faster)
    switch (this->OutputKeypointsMaps)
    {
      // Output the whole maps that are available
      case OutputKeypointsMapsMode::FULL_MAPS :
        for (auto k : LidarSlam::KeypointTypes)
        {
          if (this->SlamAlgo->KeypointTypeEnabled(k))
            this->PointCloudToPolyData(this->SlamAlgo->GetMap(k), this->CacheMaps[k]);
        }
        break;

      // Output the submaps that are available
      case OutputKeypointsMapsMode::SUB_MAPS :
        for (auto k : LidarSlam::KeypointTypes)
        {
          if (this->SlamAlgo->KeypointTypeEnabled(k))
            this->PointCloudToPolyData(this->SlamAlgo->GetTargetSubMap(k), this->CacheMaps[k]);
        }
        break;

      // If no map should be outputed, let the maps empty
      case OutputKeypointsMapsMode::NONE :
        break;

      default:
        break;
    }
  }

  // Fill outputs from cache
  edgeMap->ShallowCopy(this->CacheMaps[LidarSlam::EDGE]);
  planarMap->ShallowCopy(this->CacheMaps[LidarSlam::PLANE]);
  intensityEdgeMap->ShallowCopy(this->CacheMaps[LidarSlam::INTENSITY_EDGE]);

  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output keypoints maps"));

  // ===== Extracted keypoints from current frame =====
  if (this->OutputCurrentKeypoints)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output current keypoints"));
    for (auto k : LidarSlam::KeypointTypes)
    {
      int port = EDGE_KEYPOINTS_OUTPUT_PORT + static_cast<int>(k);
      if (port >= OUTPUT_PORT_COUNT)
        continue;
      auto* keyoints = vtkPolyData::GetData(outputVector, port);
      if (this->SlamAlgo->KeypointTypeEnabled(k))
        this->PointCloudToPolyData(this->SlamAlgo->GetKeypoints(k, this->OutputKeypointsInWorldCoordinates), keyoints);
      else
        this->PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr(new LidarSlam::Slam::PointCloud), keyoints);
    }
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
      std::unordered_map<std::string, vtkPolyData*> outputMap;
      if (this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE))
      {
        auto* edgePoints = vtkPolyData::GetData(outputVector, EDGE_KEYPOINTS_OUTPUT_PORT);
        outputMap["EgoMotion: edge matches"]     = edgePoints;
        outputMap["EgoMotion: edge weights"]     = edgePoints;
        outputMap["Localization: edge matches"]  = edgePoints;
        outputMap["Localization: edge weights"]  = edgePoints;
      }
      if (this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE))
      {
        auto* planarPoints = vtkPolyData::GetData(outputVector, PLANE_KEYPOINTS_OUTPUT_PORT);
        outputMap["EgoMotion: plane matches"]    = planarPoints;
        outputMap["EgoMotion: plane weights"]    = planarPoints;
        outputMap["Localization: plane matches"] = planarPoints;
        outputMap["Localization: plane weights"] = planarPoints;

      }

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
void vtkSlam::SetImuGravity(double x, double y, double z)
{
  vtkDebugMacro(<< "Setting ImuGravity to " << x << " " << y << " " << z);
  this->SlamAlgo->SetImuGravity(Eigen::Vector3d({x, y, z}));
  // refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
Eigen::Isometry3d vtkSlam::GetCalibrationMatrix(const std::string& fileName) const
{
  // Look for file
  std::ifstream fin (fileName);
  Eigen::Isometry3d base2Sensor = Eigen::Isometry3d::Identity();
  if (fin.is_open())
  {
    // Parse elements
    int i = 0;
    while (fin.good() && i < 16)
    {
      std::string elementString;
      fin >> elementString;
      try
      {
        base2Sensor.matrix()(i) = std::stof(elementString);
      }
      catch (...)
      {
        vtkErrorMacro(<< "Calibration file not well formed"
                      << " -> calibration is set to identity");
        base2Sensor = Eigen::Isometry3d::Identity();
        break;
      }
      ++i;
    }
    base2Sensor.matrix().transposeInPlace();
  }
  else
  {
    vtkErrorMacro(<< "Could not find calibration file : "
                  << fileName
                  << " -> calibration is set to identity");
  }
  vtkDebugMacro(<< "Calibration set to :\n"
                << base2Sensor.matrix() << std::endl);
  return base2Sensor;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSensorData(const std::string& fileName)
{
  vtkDebugMacro(<< "Setting sensor data from " << fileName);
  this->ExtSensorFileName = fileName;
  // Empty current measurements and reset local sensor params
  this->SlamAlgo->ResetSensors(true);

  if (fileName.empty())
    return;

  vtkNew<vtkDelimitedTextReader> reader;
  reader->SetFileName(fileName.c_str());
  reader->DetectNumericColumnsOn();
  reader->SetHaveHeaders(true);
  reader->SetFieldDelimiterCharacters(" ;,");
  reader->Update();

  // Extract the table.
  vtkTable* csvTable = reader->GetOutput();

  // Check if time exists and extract it
  if (!csvTable->GetRowData()->HasArray("time"))
  {
    vtkErrorMacro(<< "No time found in external sensor file, loading aborted");
    return;
  }
  auto arrayTime = csvTable->GetRowData()->GetArray("time");
  if (arrayTime->GetNumberOfTuples() == 0)
  {
    vtkErrorMacro(<< "No measure found in external sensor file");
    return;
  }
  // Set the maximum number of measurements stored in the SLAM filter
  this->SlamAlgo->SetSensorMaxMeasures(arrayTime->GetNumberOfTuples());

  // Look for a calibration file next to first file
  boost::filesystem::path path(fileName);
  std::string calibFileName = (path.parent_path() / "calibration_external_sensor.mat").string();
  // Set calibration
  Eigen::Isometry3d base2Sensor = this->GetCalibrationMatrix(calibFileName);

  bool extSensorFit = false;

  // Process wheel odometer data
  if (csvTable->GetRowData()->HasArray("odom"))
  {
    // this->SlamAlgo->SetWheelOdomCalibration(base2Sensor); // TODO : use calibration in SLAM process
    auto arrayOdom = csvTable->GetRowData()->GetArray("odom");
    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::WheelOdomMeasurement odomMeasurement;
      odomMeasurement.Time = arrayTime->GetTuple1(i);
      odomMeasurement.Distance = arrayOdom->GetTuple1(i);
      this->SlamAlgo->AddWheelOdomMeasurement(odomMeasurement);
    }
    PRINT_INFO("Odometry data successfully loaded")
    extSensorFit = true;
  }

  // Process IMU data
  #ifdef USE_GTSAM
  if (csvTable->GetRowData()->HasArray("acc_x")
   && csvTable->GetRowData()->HasArray("acc_y")
   && csvTable->GetRowData()->HasArray("acc_z")
   && csvTable->GetRowData()->HasArray("w_x")
   && csvTable->GetRowData()->HasArray("w_y")
   && csvTable->GetRowData()->HasArray("w_z"))
  {
    this->SlamAlgo->SetImuCalibration(base2Sensor);
    auto arrayAccX = csvTable->GetRowData()->GetArray("acc_x");
    auto arrayAccY = csvTable->GetRowData()->GetArray("acc_y");
    auto arrayAccZ = csvTable->GetRowData()->GetArray("acc_z");
    auto arrayVelR = csvTable->GetRowData()->GetArray("w_x");
    auto arrayVelP = csvTable->GetRowData()->GetArray("w_y");
    auto arrayVelY = csvTable->GetRowData()->GetArray("w_z");
    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::ImuMeasurement imuMeasurement;
      imuMeasurement.Time = arrayTime->GetTuple1(i);
      imuMeasurement.Acceleration.x()  = arrayAccX->GetTuple1(i);
      imuMeasurement.Acceleration.y()  = arrayAccY->GetTuple1(i);
      imuMeasurement.Acceleration.z()  = arrayAccZ->GetTuple1(i);
      imuMeasurement.AngleVelocity.x() = arrayVelR->GetTuple1(i);
      imuMeasurement.AngleVelocity.y() = arrayVelP->GetTuple1(i);
      imuMeasurement.AngleVelocity.z() = arrayVelY->GetTuple1(i);
      this->SlamAlgo->AddImuMeasurement(imuMeasurement);
    }
    PRINT_INFO("IMU data successfully loaded");
    extSensorFit = true;
  }
  else if (csvTable->GetRowData()->HasArray("acc_x")
        && csvTable->GetRowData()->HasArray("acc_y")
        && csvTable->GetRowData()->HasArray("acc_z"))
  {
  #else
  if (csvTable->GetRowData()->HasArray("acc_x")
   && csvTable->GetRowData()->HasArray("acc_y")
   && csvTable->GetRowData()->HasArray("acc_z"))
  {
    // this->SlamAlgo->SetGravityCalibration(base2Sensor); // TODO : use calibration in SLAM process
  #endif
    auto arrayAccX = csvTable->GetRowData()->GetArray("acc_x");
    auto arrayAccY = csvTable->GetRowData()->GetArray("acc_y");
    auto arrayAccZ = csvTable->GetRowData()->GetArray("acc_z");
    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::GravityMeasurement gravityMeasurement;
      gravityMeasurement.Time = arrayTime->GetTuple1(i);
      gravityMeasurement.Acceleration.x() = arrayAccX->GetTuple1(i);
      gravityMeasurement.Acceleration.y() = arrayAccY->GetTuple1(i);
      gravityMeasurement.Acceleration.z() = arrayAccZ->GetTuple1(i);
      this->SlamAlgo->AddGravityMeasurement(gravityMeasurement);
    }
    PRINT_INFO("IMU data successfully loaded for gravity integration");
    extSensorFit = true;
  }

  // Process Pose data
  if (csvTable->GetRowData()->HasArray("x")
   && csvTable->GetRowData()->HasArray("y")
   && csvTable->GetRowData()->HasArray("z")
   && csvTable->GetRowData()->HasArray("roll")
   && csvTable->GetRowData()->HasArray("pitch")
   && csvTable->GetRowData()->HasArray("yaw"))
  {
    this->SlamAlgo->SetPoseCalibration(base2Sensor);
    auto arrayX     = csvTable->GetRowData()->GetArray("x"    );
    auto arrayY     = csvTable->GetRowData()->GetArray("y"    );
    auto arrayZ     = csvTable->GetRowData()->GetArray("z"    );
    auto arrayRoll  = csvTable->GetRowData()->GetArray("roll" );
    auto arrayPitch = csvTable->GetRowData()->GetArray("pitch");
    auto arrayYaw   = csvTable->GetRowData()->GetArray("yaw"  );

    for (vtkIdType i = 0; i < arrayTime->GetNumberOfTuples(); ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Time = arrayTime->GetTuple1(i);
      // Derive Isometry
      meas.Pose.linear() = Eigen::Matrix3d(
                           Eigen::AngleAxisd(arrayPitch->GetTuple1(i), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(arrayRoll->GetTuple1(i),  Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(arrayYaw->GetTuple1(i),   Eigen::Vector3d::UnitZ())
                           );
      meas.Pose.translation() = Eigen::Vector3d(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i));
      meas.Pose.makeAffine();
      this->SlamAlgo->AddPoseMeasurement(meas);
    }

    PRINT_INFO("Pose data successfully loaded")
    extSensorFit = true;
  }

  if (!extSensorFit)
    vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No usable data found in the external sensor file");

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetTrajectory(const std::string& fileName)
{
  if (fileName.empty())
    return;
  PRINT_INFO("Set trajectory from file.");
  vtkNew<vtkDelimitedTextReader> reader;
  reader->SetFileName(fileName.c_str());
  reader->DetectNumericColumnsOn();
  reader->SetHaveHeaders(true);
  reader->SetFieldDelimiterCharacters(" ;,");
  reader->Update();

  // Extract the table.
  vtkTable* csvTable = reader->GetOutput();

  // Check if time exists and extract it
  if (!csvTable->GetRowData()->HasArray("Time"))
  {
    vtkWarningMacro(<<"No time information in the trajectory file. Load trajectory failed.");
    return;
  }
  auto arrayTime = csvTable->GetRowData()->GetArray("Time");
  vtkIdType numPose = arrayTime->GetNumberOfTuples();
  if (numPose == 0)
  {
    vtkWarningMacro(<<"No valid data in the trajectory file. Load trajectory failed.");
    return;
  }

  // Initialize a pose manager to store the external trajectory
  // Enable Verbose is useful to know whether the new trajectory is loaded correctly.
  // Set DistanceThreshold and AngleThreshold by the same values used in slam for checking keyframes.
  LidarSlam::ExternalSensors::PoseManager trajectoryManager("new trajectory");
  trajectoryManager.SetVerbose(true);
  trajectoryManager.SetDistanceThreshold(std::max(2., 2*this->GetKfDistanceThreshold()));

  // Set default covariance
  float defaultPositionError = 1e-2; // 1cm
  float defaultAngleError = Utils::Deg2Rad(1.f); // 1°
  Eigen::Matrix6d defaultCovariance = Utils::CreateDefaultCovariance(defaultPositionError, defaultAngleError); //1cm, 1°
  // If 2D mode enabled, supply constant covariance for unevaluated variables
  if (this->GetTwoDMode())
  {
    defaultCovariance(2, 2) = std::pow(defaultPositionError, 2);
    defaultCovariance(3, 3) = std::pow(defaultAngleError,    2);
    defaultCovariance(4, 4) = std::pow(defaultAngleError,    2);
  }
  // Process Covariance data
  std::vector<Eigen::Matrix6d> newCovariances(numPose, defaultCovariance);
  bool hasCovariance = true;
  for (int nCov = 0; nCov < 36; ++nCov)
  {
    hasCovariance = hasCovariance && csvTable->GetRowData()->HasArray(("Covariance:" + std::to_string(nCov)).c_str());
    if (!hasCovariance)
      break;
  }
  if (hasCovariance)
  {
    // If covariance exists, set CovarianceRotation true
    trajectoryManager.SetCovarianceRotation(true);
    // Load covariance matrix
    for (int nCov = 0; nCov < 36; ++nCov)
    {
      auto arrayCovariance =  csvTable->GetRowData()->GetArray(("Covariance:"+std::to_string(nCov)).c_str());
      for (vtkIdType poseIdx = 0; poseIdx < numPose; ++poseIdx)
        newCovariances[poseIdx](nCov) =  arrayCovariance->GetTuple1(poseIdx);
    }
  }

  // Process Pose data
  if (csvTable->GetRowData()->HasArray("x")
   && csvTable->GetRowData()->HasArray("y")
   && csvTable->GetRowData()->HasArray("z")
   && csvTable->GetRowData()->HasArray("roll")
   && csvTable->GetRowData()->HasArray("pitch")
   && csvTable->GetRowData()->HasArray("yaw"))
  {
    auto arrayX     = csvTable->GetRowData()->GetArray("x"    );
    auto arrayY     = csvTable->GetRowData()->GetArray("y"    );
    auto arrayZ     = csvTable->GetRowData()->GetArray("z"    );
    auto arrayRoll  = csvTable->GetRowData()->GetArray("roll" );
    auto arrayPitch = csvTable->GetRowData()->GetArray("pitch");
    auto arrayYaw   = csvTable->GetRowData()->GetArray("yaw"  );

    for (vtkIdType i = 0; i < numPose; ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Pose = Utils::XYZRPYtoIsometry(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i),
                                          arrayRoll->GetTuple1(i), arrayPitch->GetTuple1(i), arrayYaw->GetTuple1(i));
      meas.Time = arrayTime->GetTuple1(i);
      // If covariance data is available, check the validity and add to measurement.
      if(hasCovariance)
      {
        if (!Utils::isCovarianceValid(newCovariances[i]))
          newCovariances[i] = defaultCovariance;
        meas.Covariance = newCovariances[i];
      }
      trajectoryManager.AddMeasurement(meas);
    }
  }
  else if (csvTable->GetRowData()->HasArray("Orientation(AxisAngle):0")
        && csvTable->GetRowData()->HasArray("Orientation(AxisAngle):1")
        && csvTable->GetRowData()->HasArray("Orientation(AxisAngle):2")
        && csvTable->GetRowData()->HasArray("Orientation(AxisAngle):3")
        && csvTable->GetRowData()->HasArray("Points:0"                )
        && csvTable->GetRowData()->HasArray("Points:1"                )
        && csvTable->GetRowData()->HasArray("Points:2"                ))
  {
    auto arrayAxisX = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):0");
    auto arrayAxisY = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):1");
    auto arrayAxisZ = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):2");
    auto arrayAngle = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):3");
    auto arrayX     = csvTable->GetRowData()->GetArray("Points:0"                );
    auto arrayY     = csvTable->GetRowData()->GetArray("Points:1"                );
    auto arrayZ     = csvTable->GetRowData()->GetArray("Points:2"                );
    for (vtkIdType i = 0; i < numPose; ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Pose = Utils::XYZAngleAxistoIsometry(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i),
                                                arrayAngle->GetTuple1(i),
                                                arrayAxisX->GetTuple1(i), arrayAxisY->GetTuple1(i), arrayAxisZ->GetTuple1(i));
      meas.Time = arrayTime->GetTuple1(i);
      // If covariance data is available, check the validity and add to measurement.
      if(hasCovariance)
      {
        if (!Utils::isCovarianceValid(newCovariances[i]))
          newCovariances[i] = defaultCovariance;
        meas.Covariance = newCovariances[i];
      }
      trajectoryManager.AddMeasurement(meas);
    }
  }

  // Reload LogStates in Slam with new trajectory
  this->SlamAlgo->ResetStatePoses(trajectoryManager);
  PRINT_INFO("Trajectory successfully loaded.");

  // Update trajectory from first timestamp of lidarStates
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  this->ResetTrajectory(lidarStates.front().Time);
  for (auto const& state: lidarStates)
    this->AddPoseToTrajectory(state);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSensorTimeSynchronization(int mode)
{
  if (mode > 1)
  {
    vtkErrorMacro(<< "Invalid time synchronization mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting SensorTimeSynchronization to " << mode);
  this->SynchronizeOnPacket = (mode == 0);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Slam parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->SlamAlgo->Get##param() << std::endl;

  PrintParameter(Undistortion)
  PrintParameter(NbThreads)
  PrintParameter(Verbosity)

  for (auto& k : LidarSlam::KeypointTypes)
  {
    if (this->SlamAlgo->KeypointTypeEnabled(k))
      os << LidarSlam::KeypointTypeNames.at(k) << " enabled" << std::endl;
  }

  PrintParameter(EgoMotionICPMaxIter)
  PrintParameter(EgoMotionLMMaxIter)
  PrintParameter(EgoMotionMaxNeighborsDistance)
  PrintParameter(EgoMotionEdgeNbNeighbors)
  PrintParameter(EgoMotionEdgeMinNbNeighbors)
  PrintParameter(EgoMotionEdgeMaxModelError)
  PrintParameter(EgoMotionPlaneNbNeighbors)
  PrintParameter(EgoMotionPlaneMaxModelError)
  PrintParameter(EgoMotionPlanarityThreshold)
  PrintParameter(EgoMotionInitSaturationDistance)
  PrintParameter(EgoMotionFinalSaturationDistance)

  PrintParameter(LocalizationICPMaxIter)
  PrintParameter(LocalizationLMMaxIter)
  PrintParameter(LocalizationMaxNeighborsDistance)
  PrintParameter(LocalizationEdgeNbNeighbors)
  PrintParameter(LocalizationEdgeMinNbNeighbors)
  PrintParameter(LocalizationEdgeMaxModelError)
  PrintParameter(LocalizationPlaneNbNeighbors)
  PrintParameter(LocalizationPlanarityThreshold)
  PrintParameter(LocalizationPlaneMaxModelError)
  PrintParameter(LocalizationBlobNbNeighbors)
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
    info->Set(vtkAlgorithm::INPUT_IS_OPTIONAL(), 1);
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
    // Check if requested lidar scan arrays exist and set them if they are valid
    auto CheckAndSetScanArrays = [&](const char* time, const char* intensity, const char* laserId)
    {
      bool valid = poly->GetPointData()->HasArray(time) &&
                   poly->GetPointData()->HasArray(intensity) &&
                   poly->GetPointData()->HasArray(laserId);
      this->TimeArrayName      = valid ? time      : "";
      this->IntensityArrayName = valid ? intensity : "";
      this->LaserIdArrayName   = valid ? laserId   : "";
      return valid;
    };

    // Check if requested calib array exists and set it if it is valid
    auto CheckAndSetCalibArray = [&](const char* vendor, const char* verticalAngles)
    {
      bool valid = calib && calib->GetRowData() && calib->GetRowData()->HasArray(verticalAngles);
      this->VerticalCalibArrayName = valid ? verticalAngles : "";
      // NOTE: This warning is currently disabled as the calibration is
      // actually totally useless. Indeed, the laser ring ID is only used during
      // keypoints extraction (KE), but since current KE only consider each ring
      // independently from the others, we don't care about the actual ordering
      // or value of these IDs. However, this will need to be restored if the
      // ordering becomes important, for example using a range image / vertex map.
      // if (!valid && this->Trajectory->GetNumberOfPoints() == 0)
      //   vtkWarningMacro(<< "SLAM detected " << vendor << " data but failed to load '"
      //                   << verticalAngles << "' calibration array: laser rings' IDs won't be modified.");
      return valid;
    };

    // Check some keypoints extraction parameters values at SLAM initialization
    #define CheckKEParameter(vendor, parameter, condition) \
      if (this->Trajectory->GetNumberOfPoints() == 0 && \
          !(this->SlamAlgo->GetKeyPointsExtractor()->Get ##parameter() condition)) \
        { vtkWarningMacro(<< "SLAM run with " vendor " data: consider using " #parameter " " #condition); }

    // Test if LiDAR data is Velodyne
    if (CheckAndSetScanArrays("adjustedtime", "intensity", "laser_id"))
    {
      this->TimeToSecondsFactor = 1e-6;
      CheckAndSetCalibArray("Velodyne", "verticalCorrection");
      CheckKEParameter("Velodyne", EdgeIntensityGapThreshold, < 100);
    }

    // Test if LiDAR data is Ouster
    else if (CheckAndSetScanArrays("Raw Timestamp", "Signal Photons", "Channel"))
    {
      this->TimeToSecondsFactor = 1e-9;
      CheckAndSetCalibArray("Ouster", "Altitude Angles");
      CheckKEParameter("Ouster", EdgeIntensityGapThreshold, >= 100);
      CheckKEParameter("Ouster", MinNeighNb, > 4);
    }

    // Test if LiDAR data is Hesai
    else if (CheckAndSetScanArrays("Timestamp", "Intensity", "LaserID"))
    {
      this->TimeToSecondsFactor = 1.;
      CheckKEParameter("Hesai", MinNeighNb, > 4);
    }

    // Failed to recognize LiDAR vendor
    else
      vtkErrorMacro(<< "Unable to identify LiDAR arrays to use.");
  }

  // Otherwise, user needs to specify which arrays to use
  else
  {
    this->TimeToSecondsFactor    = this->TimeToSecondsFactorSetting;
    this->TimeArrayName          = this->GetInputArrayToProcess(0, poly)->GetName();
    this->IntensityArrayName     = this->GetInputArrayToProcess(1, poly)->GetName();
    this->LaserIdArrayName       = this->GetInputArrayToProcess(2, poly)->GetName();
    auto angleCalibArray = calib ? this->GetInputArrayToProcess(3, calib) : nullptr;
    this->VerticalCalibArrayName = angleCalibArray ? angleCalibArray->GetName() : "";
  }
}

//-----------------------------------------------------------------------------
std::vector<size_t> vtkSlam::GetLaserIdMapping(vtkTable* calib)
{
  std::vector<size_t> laserIdMapping;
  auto array = calib ? vtkDataArray::SafeDownCast(calib->GetColumnByName(this->VerticalCalibArrayName.c_str())) : nullptr;
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
void vtkSlam::ResetTrajectory(double startTime)
{
  // By default reset the output SLAM trajectory
  // If startTime is set, reset the trajectory from the startTime
  if (startTime < 0)
  {
    this->Trajectory = vtkSmartPointer<vtkPolyData>::New();
    auto pts = vtkSmartPointer<vtkPoints>::New();
    this->Trajectory->SetPoints(pts);
    auto cellArray = vtkSmartPointer<vtkCellArray>::New();
    this->Trajectory->SetLines(cellArray);
    this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Time", 1));
    this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(Quaternion)", 4));
    this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(AxisAngle)", 4));
    this->Trajectory->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Covariance", 36));
    return;
  }
  // Create a temporary trajectory to save the trajectory before startTime
  vtkNew<vtkPolyData> trajectoryTmp;
  vtkNew<vtkPoints> pts;
  trajectoryTmp->SetPoints(pts);
  vtkNew<vtkCellArray> cellArray;
  trajectoryTmp->SetLines(cellArray);
  trajectoryTmp->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Time", 1));
  trajectoryTmp->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(Quaternion)", 4));
  trajectoryTmp->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(AxisAngle)", 4));
  trajectoryTmp->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Covariance", 36));

  auto arrayTime = this->Trajectory->GetPointData()->GetArray("Time");
  for (vtkIdType idx = 0; idx < arrayTime->GetNumberOfTuples(); ++idx)
  {
    if (*arrayTime->GetTuple(idx) < startTime)
    {
      double *translation = this->Trajectory->GetPoint(idx);
      trajectoryTmp->GetPoints()->InsertNextPoint(translation);

      double *quaternion = this->Trajectory->GetPointData()->GetArray("Orientation(Quaternion)")->GetTuple(idx);
      trajectoryTmp->GetPointData()->GetArray("Orientation(Quaternion)")->InsertNextTuple(quaternion);

      double *angleAxis = this->Trajectory->GetPointData()->GetArray("Orientation(AxisAngle)")->GetTuple(idx);
      trajectoryTmp->GetPointData()->GetArray("Orientation(AxisAngle)")->InsertNextTuple(angleAxis);

      double *time = this->Trajectory->GetPointData()->GetArray("Time")->GetTuple(idx);
      trajectoryTmp->GetPointData()->GetArray("Time")->InsertNextTuple(time);

      double *covariance = this->Trajectory->GetPointData()->GetArray("Covariance")->GetTuple(idx);
      trajectoryTmp->GetPointData()->GetArray("Covariance")->InsertNextTuple(covariance);

      // Add line linking 2 successive points
      vtkIdType nPoints = trajectoryTmp->GetNumberOfPoints();
      if (nPoints >= 2)
      {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, nPoints - 2);
        line->GetPointIds()->SetId(1, nPoints - 1);
        trajectoryTmp->GetLines()->InsertNextCell(line);
      }
    }
    else
      break;
  }

  // Copy temporary trajectory to Trajectory
  this->Trajectory->ShallowCopy(trajectoryTmp);

}

//-----------------------------------------------------------------------------
void vtkSlam::AddPoseToTrajectory(const LidarSlam::LidarState& state)
{
  // Add position
  Eigen::Vector3d translation = state.Isometry.translation();
  this->Trajectory->GetPoints()->InsertNextPoint(translation.x(), translation.y(), translation.z());

  // Add orientation as quaternion
  Eigen::Quaterniond quaternion(state.Isometry.linear());
  double wxyz[] = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  this->Trajectory->GetPointData()->GetArray("Orientation(Quaternion)")->InsertNextTuple(wxyz);

  // Add orientation as axis angle
  Eigen::AngleAxisd angleAxis(state.Isometry.linear());
  Eigen::Vector3d axis = angleAxis.axis();
  double xyza[] = {axis.x(), axis.y(), axis.z(), angleAxis.angle()};
  this->Trajectory->GetPointData()->GetArray("Orientation(AxisAngle)")->InsertNextTuple(xyza);

  // Add pose time and covariance
  this->Trajectory->GetPointData()->GetArray("Time")->InsertNextTuple(&state.Time);
  this->Trajectory->GetPointData()->GetArray("Covariance")->InsertNextTuple(state.Covariance.data());

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
  vtkNew<vtkIdTypeArray> connectivity;
  connectivity->SetNumberOfValues(nbPoints);
  vtkNew<vtkCellArray> cellArray;
  cellArray->SetData(1 , connectivity);
  poly->SetVerts(cellArray);

  // Fill points and cells values
  for (vtkIdType i = 0; i < nbPoints; ++i)
  {
    // Set point
    const auto& p = pc->points[i];
    pts->SetPoint(i, p.x, p.y, p.z);
    intensityArray->SetTuple1(i, p.intensity);
    // TODO : add other fields (time, laserId)?

    connectivity->SetValue(i, i); //TODO can we iota this thing
  }
}

// =============================================================================
//   Getters / setters
// =============================================================================

bool vtkSlam::areEdgesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE);
  if (enabled)
    vtkDebugMacro(<< "Edges are enabled");
  else
    vtkDebugMacro(<< "Edges are disabled");
  return enabled;
}

bool vtkSlam::areIntensityEdgesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::INTENSITY_EDGE);
  if (enabled)
    vtkDebugMacro(<< "Intensity edges are enabled");
  else
    vtkDebugMacro(<< "Intensity edges are disabled");
  return enabled;
}

bool vtkSlam::arePlanesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE);
  if (enabled)
    vtkDebugMacro(<< "Planes are enabled");
  else
    vtkDebugMacro(<< "Planes are disabled");
  return enabled;
}

bool vtkSlam::areBlobsEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB);
  if (enabled)
    vtkDebugMacro(<< "Blobs are enabled");
  else
    vtkDebugMacro(<< "Blobs are disabled");
  return enabled;
}

//-----------------------------------------------------------------------------
void vtkSlam::EnableEdges(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling edges");
  else
  {
    vtkDebugMacro(<< "Disabling edges");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::EDGE, enabled);
}

void vtkSlam::EnableIntensityEdges(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling intensity edges");
  else
  {
    vtkDebugMacro(<< "Disabling intensity edges");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::INTENSITY_EDGE, enabled);
}

void vtkSlam::EnablePlanes(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling planes");
  else
  {
    vtkDebugMacro(<< "Disabling planes");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::PLANE, enabled);
}

void vtkSlam::EnableBlobs(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling blobs");
  else
  {
    vtkDebugMacro(<< "Disabling blobs");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::BLOB, enabled);
}

//-----------------------------------------------------------------------------
void vtkSlam::SetAdvancedReturnMode(bool _arg)
{
  vtkDebugMacro(<< "Setting AdvancedReturnMode to " << _arg);
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
      // Enable overlap computation
      this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
      this->SlamAlgo->SetTimeWindowDuration(this->TimeWindowDuration);
      this->SlamAlgo->SetLoggingTimeout(std::max(this->LoggingTimeout, 1.1 * this->TimeWindowDuration));
    }

    // If AdvancedReturnMode is being disabled
    else
    {
      // Delete optional arrays
      for (const auto& it : debugInfo)
        this->Trajectory->GetPointData()->RemoveArray(it.first.c_str());
      // Disable overlap computation
      this->SlamAlgo->SetOverlapSamplingRatio(0.);
      this->SlamAlgo->SetTimeWindowDuration(0.);
      this->SlamAlgo->SetLoggingTimeout(this->LoggingTimeout);
    }

    this->AdvancedReturnMode = _arg;
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetOutputKeypointsMaps()
{
  int outputMaps = static_cast<int>(this->OutputKeypointsMaps);
  vtkDebugMacro(<< "Returning output keypoints maps mode : " << outputMaps);
  return outputMaps;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetOutputKeypointsMaps(int mode)
{
  OutputKeypointsMapsMode outputMaps = static_cast<OutputKeypointsMapsMode>(mode);
  if (outputMaps != OutputKeypointsMapsMode::NONE      &&
      outputMaps != OutputKeypointsMapsMode::FULL_MAPS &&
      outputMaps != OutputKeypointsMapsMode::SUB_MAPS)
  {
    vtkErrorMacro(<< "Invalid output keypoints maps mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting output keypoints maps mode to " << mode);
  if (this->OutputKeypointsMaps != outputMaps)
  {
    this->OutputKeypointsMaps = outputMaps;
    this->ParametersModificationTime.Modified();
  }
  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
int vtkSlam::GetEgoMotion()
{
  int egoMotion = static_cast<int>(this->SlamAlgo->GetEgoMotion());
  vtkDebugMacro(<< "Returning Ego-Motion of " << egoMotion);
  return egoMotion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetEgoMotion(int mode)
{
  LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(mode);
  if (egoMotion != LidarSlam::EgoMotionMode::NONE                 &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION &&
      egoMotion != LidarSlam::EgoMotionMode::REGISTRATION         &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION &&
      egoMotion != LidarSlam::EgoMotionMode::EXTERNAL &&
      egoMotion != LidarSlam::EgoMotionMode::EXTERNAL_OR_MOTION_EXTRAPOLATION)
  {
    vtkErrorMacro(<< "Invalid ego-motion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting Ego-Motion to " << mode);
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
  vtkDebugMacro(<< "Returning Undistortion of " << undistortion);
  return undistortion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetUndistortion(int mode)
{
  LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(mode);
  if (undistortion != LidarSlam::UndistortionMode::NONE &&
      undistortion != LidarSlam::UndistortionMode::ONCE &&
      undistortion != LidarSlam::UndistortionMode::REFINED &&
      undistortion != LidarSlam::UndistortionMode::EXTERNAL)
  {
    vtkErrorMacro(<< "Invalid undistortion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting Undistortion to " << mode);
  if (this->SlamAlgo->GetUndistortion() != undistortion)
  {
    this->SlamAlgo->SetUndistortion(undistortion);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarTranslation(double x, double y, double z)
{
  vtkDebugMacro(<< "Setting BaseToLidarTranslation to " << x << " " << y << " " << z);
  Eigen::Isometry3d baseToLidar = this->SlamAlgo->GetBaseToLidarOffset();
  baseToLidar.translation() = Eigen::Vector3d(x, y, z);
  this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarRotation(double rx, double ry, double rz)
{
  vtkDebugMacro(<< "Setting BaseToLidarRotation to " << rx << " " << ry << " " << rz);
  Eigen::Isometry3d baseToLidar = this->SlamAlgo->GetBaseToLidarOffset();
  baseToLidar.linear() = Utils::RPYtoRotationMatrix(Utils::Deg2Rad(rx), Utils::Deg2Rad(ry), Utils::Deg2Rad(rz));
  this->SlamAlgo->SetBaseToLidarOffset(baseToLidar);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetBaseToLidarTransform(std::string filename)
{
  Eigen::Isometry3d baseToLidar;
  if (filename.empty())
    baseToLidar = Eigen::Isometry3d::Identity();
  else
    baseToLidar = this->GetCalibrationMatrix(filename);
  vtkDebugMacro(<< "Setting BaseToLidarTransform to \n" << baseToLidar.matrix() << "\n");
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

//-----------------------------------------------------------------------------
unsigned int vtkSlam::GetMapUpdate()
{
  unsigned int mapUpdate = static_cast<unsigned int>(this->SlamAlgo->GetMapUpdate());
  vtkDebugMacro(<< "Returning mapping mode of " << mapUpdate);
  return mapUpdate;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetMapUpdate(unsigned int mode)
{
  LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mode);
  if (mapUpdate != LidarSlam::MappingMode::NONE         &&
      mapUpdate != LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP &&
      mapUpdate != LidarSlam::MappingMode::UPDATE)
  {
    vtkErrorMacro(<< "Invalid mapping mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting mapping mode to " << mode);
  if (this->SlamAlgo->GetMapUpdate() != mapUpdate)
  {
    this->SlamAlgo->SetMapUpdate(mapUpdate);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s)
{
  // The setting of this parameter is only possible if the relative map exists
  // the enabling step will create the map on which the sampling mode parameter can be set
  // The user can call this setter function only if the keypoint type has been enabled (see xml)
  // So, this function must not be called before clicking on enabled
  // However, clicking on Apply call all the setters with default values.
  // Therefore, the on/off state is checked but no warning can be raised
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
    return;

  vtkDebugMacro(<< "Setting VoxelGridLeafSize to " << s);
  this->SlamAlgo->SetVoxelGridLeafSize(k, s);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
double vtkSlam::GetVoxelGridLeafSize(LidarSlam::Keypoint k) const
{
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
  {
    vtkErrorMacro(<< "Cannot get leaf size, " << LidarSlam::KeypointTypeNames.at(k) << " keypoints are not enabled.");
    return -1.;
  }
  double leafSize = this->SlamAlgo->GetVoxelGridLeafSize(k);
  vtkDebugMacro(<< "Returning sampling mode : " << leafSize);
  return leafSize;
}

//-----------------------------------------------------------------------------
int vtkSlam::GetVoxelGridSamplingMode(LidarSlam::Keypoint k) const
{
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
  {
    vtkErrorMacro(<< "Cannot get sampling mode, " << LidarSlam::KeypointTypeNames.at(k) << " keypoints are not enabled.");
    return -1;
  }
  LidarSlam::SamplingMode sampling = this->SlamAlgo->GetVoxelGridSamplingMode(k);
  int sm = static_cast<int>(sampling);
  vtkDebugMacro(<< "Returning sampling mode : " << sm);
  return sm;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridSamplingMode(LidarSlam::Keypoint k, int mode)
{
  // The setting of this parameter is only possible if the relative map exists
  // the enabling step will create the map on which the sampling mode parameter can be set
  // The user can call this setter function only if the keypoint type has been enabled (see xml)
  // So, this function must not be called before clicking on enabled
  // However, clicking on Apply call all the setters with default values.
  // Therefore, the on/off state is checked but no warning can be raised
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
    return;

  LidarSlam::SamplingMode sampling = static_cast<LidarSlam::SamplingMode>(mode);
  if (sampling != LidarSlam::SamplingMode::FIRST         &&
      sampling != LidarSlam::SamplingMode::LAST          &&
      sampling != LidarSlam::SamplingMode::MAX_INTENSITY &&
      sampling != LidarSlam::SamplingMode::CENTER_POINT  &&
      sampling != LidarSlam::SamplingMode::CENTROID)
  {
    vtkErrorMacro(<< "Invalid sampling mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting sampling mode to " << mode);
  if (this->SlamAlgo->GetVoxelGridSamplingMode(k) != sampling)
  {
    this->SlamAlgo->SetVoxelGridSamplingMode(k, sampling);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetOverlapSamplingRatio(double ratio)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< "Setting OverlapSamplingRatio to " << ratio);
  if (this->OverlapSamplingRatio != ratio)
  {
    this->OverlapSamplingRatio = ratio;
    this->ParametersModificationTime.Modified();
  }

  // Forward this parameter change to SLAM if Advanced Return Mode is enabled
  if (this->AdvancedReturnMode)
  {
    this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetAccelerationLimits(float linearAcc, float angularAcc)
{
  vtkDebugMacro(<< "Setting AccelerationLimits to " << linearAcc << " " << angularAcc);
  this->SlamAlgo->SetAccelerationLimits({linearAcc, angularAcc});
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVelocityLimits(float linearVel, float angularVel)
{
  vtkDebugMacro(<< "Setting VelocityLimits to " << linearVel << " " << angularVel);
  this->SlamAlgo->SetVelocityLimits({linearVel, angularVel});
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetTimeWindowDuration(float time)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< "Setting TimeWindowDuration to " << time);
  if (this->TimeWindowDuration != time)
  {
    this->TimeWindowDuration = time;
    this->ParametersModificationTime.Modified();
  }

  // Forward this parameter change to SLAM if Advanced Return Mode is enabled
  if (this->AdvancedReturnMode)
  {
    this->SlamAlgo->SetTimeWindowDuration(this->TimeWindowDuration);
    this->SlamAlgo->SetLoggingTimeout(std::max(this->LoggingTimeout, 1.1 * this->TimeWindowDuration));
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetLoggingTimeout(double loggingTimeout)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting LoggingTimeout to " << loggingTimeout);
  if (this->LoggingTimeout != loggingTimeout)
  {
    this->LoggingTimeout = loggingTimeout;
    this->ParametersModificationTime.Modified();
  }

  // Forward this parameter change to SLAM
  // If Advanced Return mode is enabled, use the max value
  if (this->AdvancedReturnMode)
    this->SlamAlgo->SetLoggingTimeout(std::max(this->LoggingTimeout, 1.1 * this->TimeWindowDuration));
  else
    this->SlamAlgo->SetLoggingTimeout(this->LoggingTimeout);
}

//-----------------------------------------------------------------------------
void vtkSlam::SetUsePoseGraph(bool usePoseGraph)
{
  if (this->UsePoseGraph != usePoseGraph)
  {
    this->UsePoseGraph = usePoseGraph;
    this->ParametersModificationTime.Modified();

    // If UsePoseGraph is enabled, check LoggingTimeout and return a warning if LoggingTimeout is 0
    if (this->UsePoseGraph && this->GetLoggingTimeout() <= 1e-6)
      vtkWarningMacro(<< "Pose graph is required but the logging timeout is null : no pose can be used to build the graph, please increase the logging timeout.");
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetLoopClosureQueryIdx(unsigned int loopClosureQueryIdx)
{
  // Check the input frame index can be found in Logstates
  // If the input query frame index is not in Logstates, replace it by the last frame index stored in Logstates
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  if (lidarStates.empty())
    return;
  if (loopClosureQueryIdx < lidarStates.front().Index || loopClosureQueryIdx > lidarStates.back().Index )
  {
    vtkWarningMacro(<< "The input query frame index is not valid. Please enter a frame index between ["
                    << lidarStates.front().Index << ", " << lidarStates.back().Index << "].\n"
                    << "Otherwise, the query frame index will be replaced by the last stored frame #"
                    << lidarStates.back().Index);
    loopClosureQueryIdx = lidarStates.back().Index;
  }
  vtkDebugMacro("Setting LoopClosureQueryFrameIdx to " << loopClosureQueryIdx);
  if (this->SlamAlgo->GetLoopClosureQueryIdx() != loopClosureQueryIdx)
  {
    this->SlamAlgo->SetLoopClosureQueryIdx(loopClosureQueryIdx);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetLoopClosureRevisitedIdx(unsigned int loopClosureRevisitedIdx)
{
  // Check the input frame index can be found in Logstates
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  if (lidarStates.empty())
    return;
  if (loopClosureRevisitedIdx < lidarStates.front().Index || loopClosureRevisitedIdx > lidarStates.back().Index )
  {
    vtkWarningMacro(<< "The input query frame index is not valid. Please enter a frame index between ["
                    << lidarStates.front().Index << ", " << lidarStates.back().Index << "].");
    return;
  }
  vtkDebugMacro("Setting LoopClosureRevisitedFrameIdx to " << loopClosureRevisitedIdx);
  if (this->SlamAlgo->GetLoopClosureRevisitedIdx() != loopClosureRevisitedIdx)
  {
    this->SlamAlgo->SetLoopClosureRevisitedIdx(loopClosureRevisitedIdx);
    this->ParametersModificationTime.Modified();
  }
}
