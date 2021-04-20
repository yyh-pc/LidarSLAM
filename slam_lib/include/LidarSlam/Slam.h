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
#include "LidarSlam/KDTreePCLAdaptor.h"
#include "LidarSlam/KeypointsMatcher.h"
#include "LidarSlam/LocalOptimizer.h"
#include "LidarSlam/MotionModel.h"
#include "LidarSlam/RollingGrid.h"
#include "LidarSlam/PointCloudStorage.h"
#include "LidarSlam/SensorConstraints.h"

#include <Eigen/Geometry>

#include <deque>

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
  using KDTree = KDTreePCLAdaptor<Point>;
  using KeypointExtractorPtr = std::shared_ptr<SpinningSensorKeypointExtractor>;

  // Initialization
  Slam();
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

  // Get the computed world transform so far (current BASE pose in WORLD coordinates)
  Transform GetWorldTransform() const;
  // Get the computed world transform so far, but compensating SLAM computation duration latency.
  Transform GetLatencyCompensatedWorldTransform() const;
  // Get the covariance of the last localization step (registering the current frame to the last map)
  // DoF order : X, Y, Z, rX, rY, rZ
  std::array<double, 36> GetTransformCovariance() const;

  // Get the whole trajectory and covariances of each step (aggregated WorldTransforms and TransformCovariances).
  // (buffer of temporal length LoggingTimeout)
  std::vector<Transform> GetTrajectory() const;
  std::vector<std::array<double, 36>> GetCovariances() const;

  // Get keypoints maps
  PointCloud::Ptr GetMap(Keypoint k) const;

  // Get extracted and optionally undistorted keypoints from current frame.
  // If worldCoordinates=false, it returns keypoints in BASE coordinates,
  // If worldCoordinates=true, it returns keypoints in WORLD coordinates.
  PointCloud::Ptr GetKeypoints(Keypoint k, bool worldCoordinates = false) const;

  // Get current frame
  PointCloud::Ptr GetOutputFrame();

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

  // Set world transform with an initial guess (usually from GPS after calibration).
  void SetWorldTransformFromGuess(const Transform& poseGuess);

  // Save keypoints maps to disk for later use
  void SaveMapsToPCD(const std::string& filePrefix, PCDFormat pcdFormat = PCDFormat::BINARY_COMPRESSED) const;

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

  SetMacro(LoggingTimeout, double)
  GetMacro(LoggingTimeout, double)

  SetMacro(LoggingStorage, PointCloudStorageType)
  GetMacro(LoggingStorage, PointCloudStorageType)

  SetMacro(UpdateMap, bool)
  GetMacro(UpdateMap, bool)

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

  GetMacro(MaxDistanceForICPMatching, double)
  SetMacro(MaxDistanceForICPMatching, double)

  // Get/Set EgoMotion
  GetMacro(EgoMotionLMMaxIter, unsigned int)
  SetMacro(EgoMotionLMMaxIter, unsigned int)

  GetMacro(EgoMotionICPMaxIter, unsigned int)
  SetMacro(EgoMotionICPMaxIter, unsigned int)

  GetMacro(EgoMotionLineDistanceNbrNeighbors, unsigned int)
  SetMacro(EgoMotionLineDistanceNbrNeighbors, unsigned int)

  GetMacro(EgoMotionMinimumLineNeighborRejection, unsigned int)
  SetMacro(EgoMotionMinimumLineNeighborRejection, unsigned int)

  GetMacro(EgoMotionLineDistancefactor, double)
  SetMacro(EgoMotionLineDistancefactor, double)

  GetMacro(EgoMotionPlaneDistanceNbrNeighbors, unsigned int)
  SetMacro(EgoMotionPlaneDistanceNbrNeighbors, unsigned int)

  GetMacro(EgoMotionPlaneDistancefactor1, double)
  SetMacro(EgoMotionPlaneDistancefactor1, double)

  GetMacro(EgoMotionPlaneDistancefactor2, double)
  SetMacro(EgoMotionPlaneDistancefactor2, double)

  GetMacro(EgoMotionMaxLineDistance, double)
  SetMacro(EgoMotionMaxLineDistance, double)

  GetMacro(EgoMotionMaxPlaneDistance, double)
  SetMacro(EgoMotionMaxPlaneDistance, double)

  GetMacro(EgoMotionInitSaturationDistance, double)
  SetMacro(EgoMotionInitSaturationDistance, double)

  GetMacro(EgoMotionFinalSaturationDistance, double)
  SetMacro(EgoMotionFinalSaturationDistance, double)

  // Get/Set Localization
  GetMacro(LocalizationLMMaxIter, unsigned int)
  SetMacro(LocalizationLMMaxIter, unsigned int)

  GetMacro(LocalizationICPMaxIter, unsigned int)
  SetMacro(LocalizationICPMaxIter, unsigned int)

  GetMacro(LocalizationLineDistanceNbrNeighbors, unsigned int)
  SetMacro(LocalizationLineDistanceNbrNeighbors, unsigned int)

  GetMacro(LocalizationMinimumLineNeighborRejection, unsigned int)
  SetMacro(LocalizationMinimumLineNeighborRejection, unsigned int)

  GetMacro(LocalizationLineDistancefactor, double)
  SetMacro(LocalizationLineDistancefactor, double)

  GetMacro(LocalizationPlaneDistanceNbrNeighbors, unsigned int)
  SetMacro(LocalizationPlaneDistanceNbrNeighbors, unsigned int)

  GetMacro(LocalizationPlaneDistancefactor1, double)
  SetMacro(LocalizationPlaneDistancefactor1, double)

  GetMacro(LocalizationPlaneDistancefactor2, double)
  SetMacro(LocalizationPlaneDistancefactor2, double)

  GetMacro(LocalizationMaxLineDistance, double)
  SetMacro(LocalizationMaxLineDistance, double)

  GetMacro(LocalizationMaxPlaneDistance, double)
  SetMacro(LocalizationMaxPlaneDistance, double)

  GetMacro(LocalizationInitSaturationDistance, double)
  SetMacro(LocalizationInitSaturationDistance, double)

  GetMacro(LocalizationFinalSaturationDistance, double)
  SetMacro(LocalizationFinalSaturationDistance, double)

  void SetOdomWeight(double weight) {this->OdomManager.SetWeight(weight);} 
  double GetOdomWeight() {return this->OdomManager.GetWeight();}

  void SetGravityWeight(double weight) {this->ImuManager.SetWeight(weight);} 
  double GetGravityWeight() {return this->ImuManager.GetWeight();}

  void SetSensorTimeOffset(double timeOffset);
  double GetSensorTimeOffset() {return this->ImuManager.GetTimeOffset();}

  void AddGravityMeasurement(const SensorConstraints::GravityMeasurement& gm);
  void AddOdomMeasurement(const SensorConstraints::WheelOdomMeasurement& om);
  void ClearSensorMeasurements();

  // ---------------------------------------------------------------------------
  //   Key frames and Maps parameters
  // ---------------------------------------------------------------------------

  GetMacro(KfDistanceThreshold, double)
  SetMacro(KfDistanceThreshold, double)

  GetMacro(KfAngleThreshold, double)
  SetMacro(KfAngleThreshold, double)

  // Set RollingGrid Parameters
  void ClearMaps();
  void SetVoxelGridLeafSize(Keypoint k, double size);
  void SetVoxelGridSize(int size);
  void SetVoxelGridResolution(double resolution);

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

  // Optional log of computed pose, localization covariance and keypoints of each
  // processed frame.
  // - A value of 0. will disable logging.
  // - A negative value will log all incoming data, without any timeout.
  // - A positive value will keep only the most recent data, forgetting all
  //   previous data older than LoggingTimeout seconds.
  // WARNING : A big value of LoggingTimeout may lead to an important memory
  //           consumption if SLAM is run for a long time.
  double LoggingTimeout = 0.;

  // Wether to use octree compression during keypoints logging.
  // This reduces about 5 times the memory consumption, but slows down logging (and PGO).
  PointCloudStorageType LoggingStorage = PointCloudStorageType::PCL_CLOUD;

  // Should the keypoints features maps be updated at each step.
  // It is usually set to true, but forbiding maps update can be usefull in case
  // of post-SLAM optimization with GPS and then run localization only in fixed
  // optimized map.
  bool UpdateMap = true;

  // Number of frames that have been processed by SLAM (number of poses in trajectory)
  unsigned int NbrFrameProcessed = 0;

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

  // Computed trajectory of the sensor (the list of past computed poses,
  // covariances and keypoints of each frame).
  std::deque<Transform> LogTrajectory;
  std::deque<std::array<double, 36>> LogCovariances;
  std::map<Keypoint, std::deque<PointCloudStorage<Point>>> LogKeypoints;

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

  // Current frames (all input frames)
  std::vector<PointCloud::Ptr> CurrentFrames;

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

  // keypoints local map
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

  // Boolean to notify the odometry constraint has been successfully computed
  bool OdomEnabled = false;
  // Odometry residual
  CeresTools::Residual OdomResidual;
  // Odometry manager
  // Compute the residual with a weight, a measurements list and
  // taking account of the acquisition time correspondance
  SensorConstraints::WheelOdometryManager OdomManager;

  // Boolean to notify the gravity constraint has been successfully computed
  bool GravityEnabled = false;
  // Gravity residual
  CeresTools::Residual GravityResidual;
  // IMU manager
  // Compute the residual with a weight, a measurements list and
  // taking account of the acquisition time correspondance
  SensorConstraints::ImuManager ImuManager;

  // Time offset to make a correspondance
  // between the frame timestamps and POSIX acquisition times
  // which can be linked with external sensors acquisition times
  // It must be computed as FrameFirstPointTimestamp - FrameReceptionPOSIXTime
  double SensorTimeOffset = 0.;

  // ---------------------------------------------------------------------------
  //   Optimization parameters
  // ---------------------------------------------------------------------------

  // Optimize only 2D pose in BASE coordinates.
  // This will only optimize X, Y (ground coordinates) and yaw (rZ).
  // This will hold Z (elevation), rX (roll) and rY (pitch) constant.
  bool TwoDMode = false;

  // The max distance allowed between a keypoint from the current frame and its
  // neighborhood from the map (or previous frame) to build an ICP match.
  // If the distance is over this limit, no match residual will be built.
  double MaxDistanceForICPMatching = 5.;

  // Maximum number of iteration
  // in the ego motion optimization step
  unsigned int EgoMotionLMMaxIter = 15;

  // Maximum number of iteration
  // in the localization optimization step
  unsigned int LocalizationLMMaxIter = 15;

  // During the Levenberg-Marquardt algoritm
  // keypoints will have to be match with planes
  // and lines of the previous frame. This parameter
  // indicates how many times we want to do the
  // the ICP matching
  unsigned int EgoMotionICPMaxIter = 4;
  unsigned int LocalizationICPMaxIter = 3;

  // When computing the point<->line and point<->plane distance
  // in the ICP, the kNearest edges/planes points of the current
  // points are selected to approximate the line/plane using a PCA
  // If the one of the k-nearest points is too far the neigborhood
  // is rejected. We also make a filter upon the ratio of the eigen
  // values of the variance-covariance matrix of the neighborhood
  // to check if the points are distributed upon a line or a plane
  unsigned int LocalizationLineDistanceNbrNeighbors = 10;
  unsigned int LocalizationMinimumLineNeighborRejection = 4;
  double LocalizationLineDistancefactor = 5.0;

  unsigned int LocalizationPlaneDistanceNbrNeighbors = 5;
  double LocalizationPlaneDistancefactor1 = 35.0;
  double LocalizationPlaneDistancefactor2 = 8.0;

  double LocalizationMaxPlaneDistance = 0.2;
  double LocalizationMaxLineDistance = 0.2;

  unsigned int LocalizationBlobDistanceNbrNeighbors = 25.;  // TODO : set from user interface

  unsigned int EgoMotionLineDistanceNbrNeighbors = 8;
  unsigned int EgoMotionMinimumLineNeighborRejection = 3;
  double EgoMotionLineDistancefactor = 5.;

  unsigned int EgoMotionPlaneDistanceNbrNeighbors = 5;
  double EgoMotionPlaneDistancefactor1 = 35.0;
  double EgoMotionPlaneDistancefactor2 = 8.0;

  double EgoMotionMaxPlaneDistance = 0.2;
  double EgoMotionMaxLineDistance = 0.2;

  double MinNbrMatchedKeypoints = 20.;  // TODO : set from user interface

  // Maximum distance (in meters) beyond which the residual errors are
  // saturated to robustify the optimization against outlier constraints.
  // The residuals will be robustified by Tukey loss at scale sqrt(SatDist),
  // leading to ~90% of saturation at SatDist/2, fully saturated at SatDist.
  double EgoMotionInitSaturationDistance = 5. ;
  double EgoMotionFinalSaturationDistance = 1. ;
  double LocalizationInitSaturationDistance = 2.;
  double LocalizationFinalSaturationDistance = 0.5;

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

  // Log current frame processing results : pose, covariance and keypoints.
  void LogCurrentFrameState(double time, const std::string& frameId);

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
};

} // end of LidarSlam namespace