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

// This slam algorithm is inspired by the LOAM algorithm:
// J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
// Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// The algorithm is composed of three sequential steps:
//
// - Keypoints extraction: this step consists of extracting keypoints over
// the points clouds. To do that, the laser lines / scans are treated independently.
// The laser lines are projected onto the XY plane and are rescaled depending on
// their vertical angle. Then we compute their curvature and create two classes of
// keypoints. The edges keypoints which correspond to points with a high curvature
// and planar points which correspond to points with a low curvature.
//
// - Ego-Motion: this step consists of recovering the motion of the lidar
// sensor between two frames (two sweeps). The motion is modelized by a constant
// velocity and angular velocity between two frames (i.e null acceleration).
// Hence, we can parameterize the motion by a rotation and translation per sweep / frame
// and interpolate the transformation inside a frame using the timestamp of the points.
// Since the points clouds generated by a lidar are sparse we can't design a
// pairwise match between keypoints of two successive frames. Hence, we decided to use
// a closest-point matching between the keypoints of the current frame
// and the geometric features derived from the keypoints of the previous frame.
// The geometric features are lines or planes and are computed using the edges
// and planar keypoints of the previous frame. Once the matching is done, a keypoint
// of the current frame is matched with a plane / line (depending of the
// nature of the keypoint) from the previous frame. Then, we recover R and T by
// minimizing the function f(R, T) = sum(d(point, line)^2) + sum(d(point, plane)^2).
// Which can be writen f(R, T) = sum((R*X+T-P).t*A*(R*X+T-P)) where:
// - X is a keypoint of the current frame
// - P is a point of the corresponding line / plane
// - A = (n*n.t) with n being the normal of the plane
// - A = (I - n*n.t).t * (I - n*n.t) with n being a director vector of the line
// Since the function f(R, T) is a non-linear mean square error function
// we decided to use the Levenberg-Marquardt algorithm to recover its argmin.
//
// - Localization: This step consists of refining the motion recovered in the Ego-Motion
// step and to add the new frame in the environment map. Thanks to the ego-motion
// recovered at the previous step it is now possible to estimate the new position of
// the sensor in the map. We use this estimation as an initial point (R0, T0) and we
// perform an optimization again using the keypoints of the current frame and the matched
// keypoints of the map (and not only the previous frame this time!). Once the position in the
// map has been refined from the first estimation it is then possible to update the map by
// adding the keypoints of the current frame into the map.
//
// In the following programs, three 3D coordinates system are used :
// - LIDAR {L} : attached to the geometric center of the LiDAR sensor. The
//   coordinates of the received pointclouds are expressed in this system.
//   LIDAR is rigidly linked (static transform) to BASE.
// - BASE  {B} : attached to the origin of the moving body (e.g. vehicle). We
//   are generally interested in tracking an other point of the moving body than
//   the LiDAR's (for example, we prefer to track the GPS antenna pose).
// - WORLD {W} : The world coordinate system {W} coincides with BASE at the
//   initial position. The output trajectory describes BASE origin in WORLD.

#pragma once

#include "LidarSlam/Utilities.h"
#include "LidarSlam/Transform.h"
#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/Enums.h"
#include "LidarSlam/SpinningSensorKeypointExtractor.h"
#include "LidarSlam/KeypointsMatcher.h"
#include "LidarSlam/LocalOptimizer.h"
#include "LidarSlam/MotionModel.h"
#include "LidarSlam/RollingGrid.h"
#include "LidarSlam/PointCloudStorage.h"
#include "LidarSlam/ExternalSensorManagers.h"
#include "LidarSlam/State.h"

#include <Eigen/Geometry>

#include <list>

#ifdef USE_G2O
#include "LidarSlam/PoseGraphOptimizer.h"
#endif  // USE_G2O

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

class Slam
{
public:
  // Needed as Slam has fixed size Eigen vectors as members
  // http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Usefull types
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;
  using KeypointExtractorPtr = std::shared_ptr<SpinningSensorKeypointExtractor>;
  using PCStorage = PointCloudStorage<LidarPoint>;

  // Initialization
  Slam();
  // Reset internal state : maps and trajectory are cleared
  // and current pose is set back to origin.
  // This keeps parameters and sensor data unchanged.
  void Reset(bool resetLog = true);

  // ---------------------------------------------------------------------------
  //   Main SLAM use
  // ---------------------------------------------------------------------------

  // Add a new frame to SLAM process.
  // This will trigger the following sequential steps:
  // - keypoints extraction: extract interesting keypoints to lower problem dimensionality
  // - ego-motion: estimate motion since last pose to init localization step
  // - localization: estimate global pose of current frame in map
  // - maps update: update maps using current registered frame
  void AddFrame(const PointCloud::Ptr& pc) { this->AddFrames({pc}); }

  // Add a set of frames to SLAM process.
  // This will trigger the following sequential steps:
  // - keypoints extraction: extract interesting keypoints from each frame to
  //   lower problem dimensionality, then aggregate them.
  // - ego-motion: estimate motion since last pose to init localization step
  // - localization: estimate global pose of current frame in map
  // - maps update: update maps using current registered frame
  // This first frame will be considered as 'main': its timestamp will be the
  // current pose time, its frame id will be used if no other is specified, ...
  void AddFrames(const std::vector<PointCloud::Ptr>& frames);

  // Get the computed world transform so far, but compensating SLAM computation duration latency.
  Eigen::Isometry3d GetLatencyCompensatedWorldTransform() const;
  // Get keypoints maps
  // If clean is true, the moving objects are removed from map
  PointCloud::Ptr GetMap(Keypoint k, bool clean = false) const;

  // Get target keypoints for current scan
  PointCloud::Ptr GetTargetSubMap(Keypoint k) const;

  // Get extracted and optionally undistorted keypoints from current frame.
  // If worldCoordinates=false, it returns keypoints in BASE coordinates,
  // If worldCoordinates=true, it returns keypoints in WORLD coordinates.
  // NOTE: The requested keypoints are lazy-transformed: if the requested WORLD
  // keypoints are not directly available in case they have not already been
  // internally transformed, this will be done on first call of this method.
  PointCloud::Ptr GetKeypoints(Keypoint k, bool worldCoordinates = false);

  // Get current registered (and optionally undistorted) input points.
  // All frames from all devices are aggregated.
  PointCloud::Ptr GetRegisteredFrame();

  // Get current number of frames already processed
  GetMacro(NbrFrameProcessed, unsigned int)

  // Get general information about ICP and optimization
  std::unordered_map<std::string, double> GetDebugInformation() const;
  // Get information for each keypoint of the current frame (used/rejected keypoints, ...)
  std::unordered_map<std::string, std::vector<double>> GetDebugArray() const;

  // Run pose graph optimization using GPS trajectory to improve SLAM maps and trajectory.
  // Each GPS position must have an associated precision covariance.
  // TODO : run that in a separated thread.
  void RunPoseGraphOptimization(const std::vector<Transform>& gpsPositions,
                                const std::vector<std::array<double, 9>>& gpsCovariances,
                                Eigen::Isometry3d& gpsToSensorOffset,
                                const std::string& g2oFileName = "");

  // Optimize graph containing lidar states with
  // landmarks' constraints as a postprocess
  void OptimizeGraph();

  // Set world transform with an initial guess (usually from GPS after calibration).
  void SetWorldTransformFromGuess(const Eigen::Isometry3d& poseGuess);

  // Save keypoints maps to disk for later use
  void SaveMapsToPCD(const std::string& filePrefix, PCDFormat pcdFormat = PCDFormat::BINARY_COMPRESSED, bool submap = true) const;

  // Load keypoints maps from disk (and reset SLAM maps)
  void LoadMapsFromPCD(const std::string& filePrefix, bool resetMaps = true);

  // ---------------------------------------------------------------------------
  //   General parameters
  // ---------------------------------------------------------------------------

  GetMacro(NbThreads, int)
  void SetNbThreads(int n);

  SetMacro(Verbosity, int)
  GetMacro(Verbosity, int)

  void SetUseBlobs(bool ub) { this->UseKeypoints[BLOB] = ub; }
  bool GetUseBlobs() const { return this->UseKeypoints.at(BLOB); }

  SetMacro(EgoMotion, EgoMotionMode)
  GetMacro(EgoMotion, EgoMotionMode)

  SetMacro(Undistortion, UndistortionMode)
  GetMacro(Undistortion, UndistortionMode)

  void SetLoggingTimeout(double lMax);
  GetMacro(LoggingTimeout, double)

  SetMacro(LoggingStorage, PointCloudStorageType)
  GetMacro(LoggingStorage, PointCloudStorageType)

  LidarState& GetLastState();

  GetMacro(Latency, double);

  // ---------------------------------------------------------------------------
  //   Graph parameters
  // ---------------------------------------------------------------------------

  GetMacro(G2oFileName, std::string)
  SetMacro(G2oFileName, std::string)

  GetMacro(FixFirstVertex, bool)
  SetMacro(FixFirstVertex, bool)

  GetMacro(FixLastVertex, bool)
  SetMacro(FixLastVertex, bool)

  // ---------------------------------------------------------------------------
  //   Coordinates systems parameters
  // ---------------------------------------------------------------------------

  SetMacro(BaseFrameId, std::string const&)
  GetMacro(BaseFrameId, std::string)

  SetMacro(WorldFrameId, std::string const&)
  GetMacro(WorldFrameId, std::string)

  // ---------------------------------------------------------------------------
  //   Keypoints extraction
  // ---------------------------------------------------------------------------

  // Get/Set all keypoints extractors
  std::map<uint8_t, KeypointExtractorPtr> GetKeyPointsExtractors() const;
  void SetKeyPointsExtractors(const std::map<uint8_t, KeypointExtractorPtr>& extractors);

  // Get/Set a specific keypoints extractor
  // NOTE: If no keypoint extractor exists for the requested deviceId, the returned pointer is null.
  KeypointExtractorPtr GetKeyPointsExtractor(uint8_t deviceId = 0) const;
  void SetKeyPointsExtractor(KeypointExtractorPtr extractor, uint8_t deviceId = 0);

  // Get/Set a specific base to Lidar offset
  // NOTE: If no base to lidar offset exists for the requested deviceId, the returned transform is identity.
  Eigen::Isometry3d GetBaseToLidarOffset(uint8_t deviceId = 0) const;
  void SetBaseToLidarOffset(const Eigen::Isometry3d& transform, uint8_t deviceId = 0);

  // ---------------------------------------------------------------------------
  //   Optimization parameters
  // ---------------------------------------------------------------------------

  GetMacro(TwoDMode, bool)
  SetMacro(TwoDMode, bool)

  // Get/Set EgoMotion
  GetMacro(EgoMotionLMMaxIter, unsigned int)
  SetMacro(EgoMotionLMMaxIter, unsigned int)

  GetMacro(EgoMotionICPMaxIter, unsigned int)
  SetMacro(EgoMotionICPMaxIter, unsigned int)

  GetMacro(EgoMotionMaxNeighborsDistance, double)
  SetMacro(EgoMotionMaxNeighborsDistance, double)

  GetMacro(EgoMotionEdgeNbNeighbors, unsigned int)
  SetMacro(EgoMotionEdgeNbNeighbors, unsigned int)

  GetMacro(EgoMotionEdgeMinNbNeighbors, unsigned int)
  SetMacro(EgoMotionEdgeMinNbNeighbors, unsigned int)

  GetMacro(EgoMotionPlaneNbNeighbors, unsigned int)
  SetMacro(EgoMotionPlaneNbNeighbors, unsigned int)

  GetMacro(EgoMotionPlanarityThreshold, double)
  SetMacro(EgoMotionPlanarityThreshold, double)

  GetMacro(EgoMotionEdgeMaxModelError, double)
  SetMacro(EgoMotionEdgeMaxModelError, double)

  GetMacro(EgoMotionPlaneMaxModelError, double)
  SetMacro(EgoMotionPlaneMaxModelError, double)

  GetMacro(EgoMotionInitSaturationDistance, double)
  SetMacro(EgoMotionInitSaturationDistance, double)

  GetMacro(EgoMotionFinalSaturationDistance, double)
  SetMacro(EgoMotionFinalSaturationDistance, double)

  // Get/Set Localization
  GetMacro(LocalizationLMMaxIter, unsigned int)
  SetMacro(LocalizationLMMaxIter, unsigned int)

  GetMacro(LocalizationICPMaxIter, unsigned int)
  SetMacro(LocalizationICPMaxIter, unsigned int)

  GetMacro(LocalizationMaxNeighborsDistance, double)
  SetMacro(LocalizationMaxNeighborsDistance, double)

  GetMacro(LocalizationEdgeNbNeighbors, unsigned int)
  SetMacro(LocalizationEdgeNbNeighbors, unsigned int)

  GetMacro(LocalizationEdgeMinNbNeighbors, unsigned int)
  SetMacro(LocalizationEdgeMinNbNeighbors, unsigned int)

  GetMacro(LocalizationPlaneNbNeighbors, unsigned int)
  SetMacro(LocalizationPlaneNbNeighbors, unsigned int)

  GetMacro(LocalizationPlanarityThreshold, double)
  SetMacro(LocalizationPlanarityThreshold, double)

  GetMacro(LocalizationEdgeMaxModelError, double)
  SetMacro(LocalizationEdgeMaxModelError, double)

  GetMacro(LocalizationPlaneMaxModelError, double)
  SetMacro(LocalizationPlaneMaxModelError, double)

  GetMacro(LocalizationBlobNbNeighbors, unsigned int)
  SetMacro(LocalizationBlobNbNeighbors, unsigned int)

  GetMacro(LocalizationInitSaturationDistance, double)
  SetMacro(LocalizationInitSaturationDistance, double)

  GetMacro(LocalizationFinalSaturationDistance, double)
  SetMacro(LocalizationFinalSaturationDistance, double)

  // External Sensor parameters

  // General
  GetMacro(SensorTimeOffset, double)
  void SetSensorTimeOffset(double timeOffset);

  GetMacro(SensorTimeThreshold, double)
  void SetSensorTimeThreshold(double timeOffset);

  GetMacro(SensorMaxMeasures, unsigned int)
  void SetSensorMaxMeasures(unsigned int max);

  void ClearSensorMeasurements();

  // Odometer
  double GetWheelOdomWeight() const;
  void SetWheelOdomWeight(double weight);

  bool GetWheelOdomRelative() const;
  void SetWheelOdomRelative(bool relative);

  void AddWheelOdomMeasurement(const ExternalSensors::WheelOdomMeasurement& om);

  bool WheelOdomHasData() {return this->WheelOdomManager && this->WheelOdomManager->HasData();}

  // IMU
  double GetGravityWeight() const;
  void SetGravityWeight(double weight);

  void AddGravityMeasurement(const ExternalSensors::GravityMeasurement& gm);

  bool ImuHasData() {return this->ImuManager && this->ImuManager->HasData();}

  // Landmark detector
  GetMacro(LandmarkWeight, double)
  void SetLandmarkWeight(double weight);

  GetMacro(LandmarkSaturationDistance, float)
  void SetLandmarkSaturationDistance(float dist);

  GetMacro(LandmarkPositionOnly, bool)
  void SetLandmarkPositionOnly(bool positionOnly);

  GetMacro(LandmarkCovarianceRotation, bool)
  void SetLandmarkCovarianceRotation(bool rotate);

  GetMacro(LandmarkConstraintLocal, bool)
  SetMacro(LandmarkConstraintLocal, bool)

  void AddLandmarkManager(int id, const Eigen::Vector6d& absolutePose, const Eigen::Matrix6d& absolutePoseCovariance);

  void AddLandmarkMeasurement(const ExternalSensors::LandmarkMeasurement& lm, int id);

  bool LmCanBeUsedLocally();
  bool LmHasData();

  // ---------------------------------------------------------------------------
  //   Key frames and Maps parameters
  // ---------------------------------------------------------------------------

  GetMacro(KfDistanceThreshold, double)
  SetMacro(KfDistanceThreshold, double)

  GetMacro(KfAngleThreshold, double)
  SetMacro(KfAngleThreshold, double)

  GetMacro(MapUpdate, MappingMode)
  SetMacro(MapUpdate, MappingMode)

  double GetVoxelGridDecayingThreshold();
  void SetVoxelGridDecayingThreshold(double decay);

  SamplingMode GetVoxelGridSamplingMode(Keypoint k);
  void SetVoxelGridSamplingMode(Keypoint k, SamplingMode sm);

  // Set RollingGrid Parameters
  void ClearMaps();
  void SetVoxelGridLeafSize(Keypoint k, double size);
  void SetVoxelGridSize(int size);
  void SetVoxelGridResolution(double resolution);
  void SetVoxelGridMinFramesPerVoxel(unsigned int minFrames);

  // ---------------------------------------------------------------------------
  //   Confidence estimation
  // ---------------------------------------------------------------------------

  // Overlap
  GetMacro(OverlapSamplingRatio, float)
  void SetOverlapSamplingRatio(float _arg);

  GetMacro(OverlapEstimation, float)

  // Matches
  GetMacro(TotalMatchedKeypoints, int)

  // Motion constraints
  GetMacro(AccelerationLimits, Eigen::Array2f)
  SetMacro(AccelerationLimits, const Eigen::Array2f&)

  GetMacro(VelocityLimits, Eigen::Array2f)
  SetMacro(VelocityLimits, const Eigen::Array2f&)

  GetMacro(TimeWindowDuration, float)
  SetMacro(TimeWindowDuration, float)

  GetMacro(ComplyMotionLimits, bool)

private:

  // ---------------------------------------------------------------------------
  //   General stuff and flags
  // ---------------------------------------------------------------------------

  // Max number of threads to use for parallel processing
  int NbThreads = 1;

  // Booleans to decide whether to extract the keypoints of the relative type or not
  std::map<Keypoint, bool> UseKeypoints= {{EDGE, true}, {PLANE, true}, {BLOB, false}};

  // How to estimate Ego-Motion (approximate relative motion since last frame).
  // The ego-motion step aims to give a fast and approximate initialization of
  // new frame world pose to ensure faster and more precise convergence in
  // Localization step.
  EgoMotionMode EgoMotion = EgoMotionMode::MOTION_EXTRAPOLATION;

  // How to correct the rolling shutter distortion during frame acquisition.
  // The undistortion should greatly improve the accuracy for smooth motions,
  // but might be unstable for high-frequency motions.
  UndistortionMode Undistortion = UndistortionMode::REFINED;

  // Indicate verbosity level to display more or less information :
  // 0: print errors, warnings or one time info
  // 1: 0 + frame number, total frame processing time
  // 2: 1 + extracted features, used keypoints, localization variance, ego-motion and localization summary
  // 3: 2 + sub-problems processing duration
  // 4: 3 + ceres optimization summary
  // 5: 4 + logging/maps memory usage
  int Verbosity = 0;

  // Maximum duration on which to keep states in memory.
  // This duration must be increased if a pose graph optimization is planned
  // The minimum number of logged states is 2, to be able to handle ego-motion and undistortion,
  // independently of this timeout value
  double LoggingTimeout = 0;

  // Wether to use octree compression during keypoints logging.
  // This reduces about 5 times the memory consumption, but slows down logging (and PGO).
  PointCloudStorageType LoggingStorage = PointCloudStorageType::PCL_CLOUD;

  // Number of frames that have been processed by SLAM (number of poses in trajectory)
  unsigned int NbrFrameProcessed = 0;

  // Timestamp of the current input frame
  double CurrentTime = 0.;
  // ---------------------------------------------------------------------------
  //   Trajectory, transforms and undistortion
  // ---------------------------------------------------------------------------

  // **** COORDINATES SYSTEMS ****

  // Coordinates systems (CS) names to fill in pointclouds or poses headers
  std::string WorldFrameId = "world";  // CS of trajectory and maps
  std::string BaseFrameId = "base";    // CS of current keypoints

  // **** LOCALIZATION ****

  // Global transformation to map the current pointcloud to the previous one
  Eigen::Isometry3d Trelative;

  // Transformation to map the current pointcloud in the world coordinates
  // This pose is the pose of BASE in WORLD coordinates, at the time
  // corresponding to the timestamp in the header of input Lidar scan.
  Eigen::Isometry3d Tworld;
  Eigen::Isometry3d PreviousTworld;

  // Reflect the success of one SLAM iteration computation (used to log or not the state).
  bool Valid = true;
  // Store the keyframe information
  bool IsKeyFrame = true;

  // [s] SLAM computation duration of last processed frame (~Tworld delay)
  // used to compute latency compensated pose
  double Latency;

  // **** UNDISTORTION ****

  // Transform interpolator to estimate the pose of the sensor within a lidar
  // frame, using the BASE poses at the beginning and end of frame.
  // This will be used to undistort the pointcloud and express its points
  // relatively to the same BASE pose at frame header timestamp.
  // This will use the point-wise 'time' field, representing the time offset
  // in seconds to add to the frame header timestamp.
  LinearTransformInterpolator<double> WithinFrameMotion;

  // **** LOGGING ****

  // Log info on each pose
  // It contains :
  //     -The estimated Lidar isometry and its covariance
  //     -The time associated with the pose
  //     -The relative index in the pose graph
  //     -A boolean to store the keyframe info
  //     -The undistorted keypoints (expressed in BASE)
  // The oldest states are forgotten (cf. LoggingTimeout parameter)
  std::list<LidarState> LogStates;

  // ---------------------------------------------------------------------------
  //   Keypoints extraction
  // ---------------------------------------------------------------------------

  // Sequence id of the previous processed frame, used to check frames dropping
  std::map<int, unsigned int> PreviousFramesSeq;

  // Keypoints extractors, 1 for each lidar device
  std::map<uint8_t, KeypointExtractorPtr> KeyPointsExtractors;

  // Static transform to link BASE and LIDAR coordinates systems for each device.
  // It corresponds to the pose of each LIDAR device origin in BASE coordinates.
  // If the transform is not available for a given device, identity will be used.
  std::map<uint8_t, Eigen::UnalignedIsometry3d> BaseToLidarOffsets;

  // ---------------------------------------------------------------------------
  //   Keypoints from current frame
  // ---------------------------------------------------------------------------

  // Current frames (all raw input frames)
  std::vector<PointCloud::Ptr> CurrentFrames;

  // Current aggregated points from all input frames, in WORLD coordinates (with undistortion if enabled)
  PointCloud::Ptr RegisteredFrame;

  // Raw extracted keypoints, in BASE coordinates (no undistortion)
  std::map<Keypoint, PointCloud::Ptr> CurrentRawKeypoints;
  std::map<Keypoint, PointCloud::Ptr> PreviousRawKeypoints;

  // Extracted keypoints, in BASE coordinates (with undistortion if enabled)
  std::map<Keypoint, PointCloud::Ptr> CurrentUndistortedKeypoints;

  // Extracted keypoints, in WORLD coordinates (with undistortion if enabled)
  std::map<Keypoint, PointCloud::Ptr> CurrentWorldKeypoints;

  // ---------------------------------------------------------------------------
  //   Key frames and Maps
  // ---------------------------------------------------------------------------

  // Last keyframe pose
  Eigen::Isometry3d KfLastPose = Eigen::Isometry3d::Identity();

  // Min distance or angle to travel since last keyframe to add a new one
  double KfDistanceThreshold = 0.5;  ///< [m] Min distance to travel since last KF to add a new one
  double KfAngleThreshold = 5.;      ///< [°] Min angle to rotate since last KF to add a new one

  // Number of keyrames
  int KfCounter = 0;

  // How to update the map
  // The map can be updated more or less with new input keypoints
  // from current scanned points depending on the initial map reliability.
  MappingMode MapUpdate = MappingMode::UPDATE;

  // How to downsample the points in the keypoints' maps
  // This mode parameter allows to choose how to select the remaining point in each voxel.
  // It can be taking the first/last acquired point, taking the max intensity point,
  // considering the closest point to the voxel center or averaging the points.
  SamplingMode DownSampling = SamplingMode::MAX_INTENSITY;

  // Keypoints local map
  std::map<Keypoint, std::shared_ptr<RollingGrid>> LocalMaps;

  // ---------------------------------------------------------------------------
  //   Optimization data
  // ---------------------------------------------------------------------------

  //! Matching results
  std::map<Keypoint, KeypointsMatcher::MatchingResults> EgoMotionMatchingResults;
  std::map<Keypoint, KeypointsMatcher::MatchingResults> LocalizationMatchingResults;

  // Optimization results
  // Variance-Covariance matrix that estimates the localization error about the
  // 6-DoF parameters (DoF order : X, Y, Z, rX, rY, rZ)
  LocalOptimizer::RegistrationError LocalizationUncertainty;

  // Odometry manager
  // It computes the residual with a weight, a measurements list and
  // taking account of the acquisition time correspondance
  // The odometry measurements must be filled and cleared from outside this lib
  // using External Sensors interface
  std::shared_ptr<ExternalSensors::WheelOdometryManager> WheelOdomManager;
  // Weight
  double WheelOdomWeight = 0.;
  // relative mode for wheel odometer
  bool WheelOdomRelative = false;

  // IMU manager
  // Compute the residual with a weight, a measurements list and
  // taking account of the acquisition time correspondance
  // The IMU measurements must be filled and cleared from outside this lib
  // using External Sensors interface
  std::shared_ptr<ExternalSensors::ImuManager> ImuManager;
  // Weight
  double ImuWeight = 0.;

  // Landmarks manager
  // Each landmark has its own manager and is identified by its ID.
  // This map can be initialized from outside the lib if the absolute poses of the tags are known
  // If not, it will be filled at each new detection.
  // The managers compute the residuals with a weight, measurements lists and
  // taking account of the acquisition time correspondance
  // The tag measurements must be filled and cleared from outside this lib
  // using External Sensors interface
  std::unordered_map<int, ExternalSensors::LandmarkManager> LandmarksManagers;
  // Variable to store the landmark weight to init correctly the landmark managers
  float LandmarkWeight = 0.f;
  // Boolean to choose whether to compute the reference pose of
  // the tag locally from observations when updating the model (true)
  // or to use the absolute tag pose supplied at init (false)
  bool LandmarkConstraintLocal = false;
  // Saturation distance beyond which the tags are not taken into account in the optimization
  float LandmarkSaturationDistance = 5.f;
  // Boolean to check whether to use the whole tag pose (position + orientation)
  // or only the position to create a constraint in the optimization
  bool LandmarkPositionOnly = true;
  // Boolean to decide whether to rotate the covariance
  // during measures interpolation or not. Covariances are only used
  // in pose graph optimization, not in local optimization
  bool LandmarkCovarianceRotation = true;

  // Time difference between Lidar's measurements and external sensors'
  // not null if they are not expressed relatively to the same time reference
  double SensorTimeOffset = 0.;

  // Maximum time difference (s) between two measurements to
  // allow the measures interpolation and the integration
  // of a sensor constraint in the optimization
  double SensorTimeThreshold = 0.5;

  // Maximum number of sensor measurements stored
  // Above this number, the oldest measurements are forgotten
  unsigned int SensorMaxMeasures = 1e6;

  // ---------------------------------------------------------------------------
  //   Optimization parameters
  // ---------------------------------------------------------------------------

  // Optimize only 2D pose in BASE coordinates.
  // This will only optimize X, Y (ground coordinates) and yaw (rZ).
  // This will hold Z (elevation), rX (roll) and rY (pitch) constant.
  bool TwoDMode = false;

  // Number of outer ICP-optim loop iterations to perform.
  // Each iteration will consist of building ICP matches, then optimizing them.
  unsigned int EgoMotionICPMaxIter = 4;
  unsigned int LocalizationICPMaxIter = 3;

  // Maximum number of iterations of the Levenberg-Marquardt optimizer to solve
  // the ICP problem composed of the built point-to-neighborhood residuals
  unsigned int EgoMotionLMMaxIter = 15;
  unsigned int LocalizationLMMaxIter = 15;

  // Point-to-neighborhood matching parameters.
  // The goal will be to loop over all keypoints, and to build the corresponding
  // point-to-neighborhood residuals that will be optimized later.
  // For each source keypoint, the steps will be:
  // - To extract the N nearest neighbors from the target cloud.
  //   These neighbors should not be too far from the source keypoint.
  // - Assess the neighborhood shape by checking its PCA eigenvalues.
  // - Fit a line/plane/blob model on the neighborhood using PCA.
  // - Assess the model quality by checking its error relatively to the neighborhood.
  // - Build the corresponding point-to-model distance operator
  // If any of this step fails, the matching procedure of the current keypoint aborts.
  // See KeypointsMatcher::Parameters for more details on each parameter.

  // Max distance allowed between a source keypoint and its neighbors in target map.
  // If one of the neighbors is farther, the neighborhood will be rejected.
  double EgoMotionMaxNeighborsDistance = 5.;
  double LocalizationMaxNeighborsDistance = 5.;

  // Edge keypoints matching: point-to-line distance
  unsigned int EgoMotionEdgeNbNeighbors = 8;
  unsigned int EgoMotionEdgeMinNbNeighbors = 3;
  double EgoMotionEdgeMaxModelError = 0.2;
  unsigned int LocalizationEdgeNbNeighbors = 10;
  unsigned int LocalizationEdgeMinNbNeighbors = 4;
  double LocalizationEdgeMaxModelError = 0.2;

  // Plane keypoints matching: point-to-plane distance
  unsigned int EgoMotionPlaneNbNeighbors = 5;
  double EgoMotionPlanarityThreshold = 0.04;
  double EgoMotionPlaneMaxModelError = 0.2;
  unsigned int LocalizationPlaneNbNeighbors = 5;
  double LocalizationPlanarityThreshold = 0.04;
  double LocalizationPlaneMaxModelError = 0.2;

  // Blob keypoints matching: point-to-ellipsoid distance
  unsigned int LocalizationBlobNbNeighbors = 10;

  // Maximum distance (in meters) beyond which the residual errors are
  // saturated to robustify the optimization against outlier constraints.
  // The residuals will be robustified by Tukey loss at scale sqrt(SatDist),
  // leading to ~90% of saturation at SatDist/2, fully saturated at SatDist.
  double EgoMotionInitSaturationDistance = 5. ;
  double EgoMotionFinalSaturationDistance = 1. ;
  double LocalizationInitSaturationDistance = 2.;
  double LocalizationFinalSaturationDistance = 0.5;

  // ---------------------------------------------------------------------------
  //   Graph parameters
  // ---------------------------------------------------------------------------
  // Log info from g2o, if empty, log is not stored
  std::string G2oFileName;

  // Boolean to decide if we want to some vertices of the graph
  bool FixFirstVertex = false;
  bool FixLastVertex = false;

  // ---------------------------------------------------------------------------
  //   Confidence estimation
  // ---------------------------------------------------------------------------

  // Data

  // Overlap estimation of the current registered scan on the keypoints map
  // A valid value lies in range [0-1].
  // It is set to -1 if overlap has not been evaluated (disabled or not enough points).
  float OverlapEstimation = -1.f;

  // Number of matches for processed frame
  unsigned int TotalMatchedKeypoints = 0;

  // Check motion limitations compliance
  bool ComplyMotionLimits = true;

  // Previous computed velocity (for acceleration computation)
  Eigen::Array2f PreviousVelocity;

  // Parameters

  // Extrapolating a pose farther from this time ratio is forbidden and will abort.
  // i.e, if using 2 frames with timestamps t1 and t2, extrapolating at t3 is
  // allowed only if abs((t3 - t2)/(t2 - t1)) < MaxExtrapolationRatio.
  // Otherwise, extrapolation will return the pose at t2.
  double MaxExtrapolationRatio = 3.;

  // Min number of matches to consider the optimization problem usable.
  // Below this threshold, we consider that there are not enough matches to
  // provide good enough optimization results, and registration is aborted.
  unsigned int MinNbMatchedKeypoints = 20;

  // [0-1] Ratio of points from the input cloud to compute overlap on.
  // Downsampling accelerates the overlap computation, but may be less precise.
  // If 0, overlap won't be computed.
  float OverlapSamplingRatio = 0.f;

  // Motion limitations
  // Local velocity thresholds in BASE
  Eigen::Array2f VelocityLimits     = {FLT_MAX, FLT_MAX};
  // Local acceleration thresholds in BASE
  Eigen::Array2f AccelerationLimits = {FLT_MAX, FLT_MAX};

  // Duration on which to estimate the local velocity
  // This window is used to smooth values to get a more accurate velocity estimation
  // If 0, motion limits won't be checked.
  // WARNING : the logging time out must be greater
  // in order to comply with this required value.
  float TimeWindowDuration = 0.f;

  // ---------------------------------------------------------------------------
  //   Main sub-problems and methods
  // ---------------------------------------------------------------------------

  // Check that input frames are correct
  // (empty frame, same timestamp, frame dropping, ...)
  bool CheckFrames(const std::vector<PointCloud::Ptr>& frames);

  // Extract keypoints from input pointclouds,
  // and transform them from LIDAR to BASE coordinate system.
  void ExtractKeypoints();

  // Compute constraints provided by external sensors
  void ComputeSensorConstraints();

  // Estimate the ego motion since last frame.
  // Extrapolate new pose with a constant velocity model and/or
  // refine estimation by registering current frame keypoints on previous frame keypoints.
  void ComputeEgoMotion();

  // Compute the pose of the current frame in world referential by registering
  // current frame keypoints on keypoints from maps
  void Localization();

  // Transform current keypoints to WORLD coordinates,
  // and add points to the maps if we are dealing with a new keyframe.
  void UpdateMapsUsingTworld();

  // Check if the current frame is a keyframe or not
  bool CheckKeyFrame();

  // Log current frame processing results : pose, covariance and keypoints.
  void LogCurrentFrameState(double time);

  // ---------------------------------------------------------------------------
  //   Undistortion helpers
  // ---------------------------------------------------------------------------

  // All points of the current frame have been acquired at different timestamps.
  // The goal is to express them in the same referential, at the timestamp in
  // input scan header. This can be done using estimated egomotion and assuming
  // a constant velocity during a sweep.

  // Extra/Interpolate scan pose using previous motion from PreviousTworld and Tworld.
  // 'time' arg is the time offset in seconds to current frame header.stamp.
  Eigen::Isometry3d InterpolateScanPose(double time);

  // Init undistortion interpolator time bounds based on point-wise time field.
  void InitUndistortion();

  // Update the undistortion interpolator poses bounds,
  // and refine the undistortion of the current keypoints clouds.
  void RefineUndistortion();

  // ---------------------------------------------------------------------------
  //   Confidence estimator helpers
  // ---------------------------------------------------------------------------

  // Estimate the overlap of the current scan onto the keypoints submaps
  void EstimateOverlap();

  // Test if the pose complies with motion limitations
  void CheckMotionLimits();

  // ---------------------------------------------------------------------------
  //   Transformation helpers
  // ---------------------------------------------------------------------------

  // Rigidly transform a pointcloud in a multi-threaded way.
  PointCloud::Ptr TransformPointCloud(PointCloud::ConstPtr cloud,
                                      const Eigen::Isometry3d& tf,
                                      const std::string& frameId = "") const;

  // Aggregate a set of frames from LIDAR to BASE or WORLD coordinates.
  // If worldCoordinates=false, it returns points in BASE coordinates (no undistortion).
  // If worldCoordinates=true, it returns points in WORLD coordinates (optionally undistorted).
  // The LIDAR to BASE offsets specific to each sensor are properly added.
  // The output aggregated points timestamps are corrected to be relative to the 1st frame timestamp.
  // NOTE: If transforming to WORLD coordinates, be sure that Tworld/WithinFrameMotion have been updated
  //       (updated during the Localization step).
  PointCloud::Ptr AggregateFrames(const std::vector<PointCloud::Ptr>& frames, bool worldCoordinates) const;

  // ---------------------------------------------------------------------------
  //   External sensor helpers
  // ---------------------------------------------------------------------------

  void InitWheelOdom();
  void InitImu();
  void InitLandmarkManager(int id);

};

} // end of LidarSlam namespace