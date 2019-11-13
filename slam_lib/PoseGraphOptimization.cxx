#include "PoseGraphOptimization.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>

namespace Eigen
{
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
}

namespace
{
  //----------------------------------------------------------------------------
  Eigen::Isometry3d GetRelativeTransform(const Transform& pose1, const Transform& pose2)
  {
    return pose1.GetIsometry().inverse() * pose2.GetIsometry();
  }

  //----------------------------------------------------------------------------
  /*!
   * @brief Find closest SLAM point index matching with a GPS point.
   * @param[in] gpsPose     The GPS point to match.
   * @param[in] slamPoses   The SLAM points to search in.
   * @param[in] maxTimeDiff The maximum time difference allowed between a 
   *                        GPS/SLAM pair of points to be considered matching.
   * @return The index of the closest SLAM point.
   * 
   * NOTE : The assertion of sorted vectors is done (time is increasing through poses).
   */
  int FindClosestSlamPose(const Transform& gpsPose, const std::vector<Transform>& slamPoses, double maxTimeDiff = 0.1)
  {
    double gpsTime = gpsPose.time;
    double timeDiff, prevTimeDiff = std::numeric_limits<double>::max();
    int bestId = -1;
    for (int j = 0; j < slamPoses.size(); ++j)
    {
      timeDiff = std::abs(gpsTime - slamPoses[j].time);

      if (timeDiff <= maxTimeDiff)
      {
        if (timeDiff < prevTimeDiff)
          bestId = j;
        // CHECK As vector is sorted, if time difference is increasing, we already passed through best point.
        // else
        //   return bestId;        
      }

      prevTimeDiff = timeDiff;
    }
    return bestId;
  }

  //----------------------------------------------------------------------------
  Eigen::Translation3d ComputeBarycenter(const std::vector<Transform>& poses)
  {
    Eigen::Translation3d barycenter(0., 0., 0.);
    for (const auto& pose : poses)
    {
      barycenter.x() = barycenter.x() + pose.x;
      barycenter.y() = barycenter.y() + pose.y;
      barycenter.z() = barycenter.z() + pose.z;
    }
    unsigned int nbPose = poses.size();
    barycenter.x() = barycenter.x() / nbPose;
    barycenter.y() = barycenter.y() / nbPose;
    barycenter.z() = barycenter.z() / nbPose;

    return barycenter;
  }

  //----------------------------------------------------------------------------
  Eigen::Translation3d ComputeTranslationOffset(const std::vector<Transform>& slamPoses,
                                                const std::vector<Transform>& gpsPoses)
  {
    // DEBUG
    // Eigen::Translation3d barycenterSlam = ComputeBarycenter(slamPoses);
    // Eigen::Translation3d barycenterGPS = ComputeBarycenter(gpsPoses);
    // Eigen::Translation3d translation(barycenterSlam.x() - barycenterGPS.x(),
    //                                  barycenterSlam.y() - barycenterGPS.y(),
    //                                  barycenterSlam.z() - barycenterGPS.z());
    Eigen::Translation3d translation(slamPoses[0].x - gpsPoses[0].x,
                                     slamPoses[0].y - gpsPoses[0].y,
                                     slamPoses[0].z - gpsPoses[0].z);
    return translation;
  }

  //----------------------------------------------------------------------------
  Eigen::Quaterniond ComputeRotationOffset(const std::vector<Transform>& slamPoses,
                                           const std::vector<Transform>& gpsPoses)
  {
    // Get approximate SLAM trajectory direction
    unsigned int nbSlamPoses = slamPoses.size();
    Eigen::Vector3d slamDirection(slamPoses[nbSlamPoses -1].x - slamPoses[0].x,
                                  slamPoses[nbSlamPoses -1].y - slamPoses[0].y,
                                  slamPoses[nbSlamPoses -1].z - slamPoses[0].z);

    // Get approximate GPS trajectory direction
    unsigned int nbGpsPoses = gpsPoses.size();
    Eigen::Vector3d gpsDirection(gpsPoses[nbGpsPoses -1].x - gpsPoses[0].x,
                                 gpsPoses[nbGpsPoses -1].y - gpsPoses[0].y,
                                 gpsPoses[nbGpsPoses -1].z - gpsPoses[0].z);

    // Compute approximate heading alignment from GPS to SLAM
    Eigen::Quaterniond rotation;
    rotation = Eigen::Quaterniond::FromTwoVectors(gpsDirection, slamDirection);
    return rotation;
  }
}


//------------------------------------------------------------------------------
PoseGraphOptimization::PoseGraphOptimization()
{
  // create optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
  this->GraphOptimizer.setAlgorithm(solver);

  // add GPS Offset parameter
  auto* GPSOffset = new g2o::ParameterSE3Offset;
  GPSOffset->setId(0);
  this->GraphOptimizer.addParameter(GPSOffset);
}

//------------------------------------------------------------------------------
void PoseGraphOptimization::SetGPSCalibration(double x, double y, double z)
{
  auto* GPSOffset = dynamic_cast<g2o::ParameterSE3Offset*>(this->GraphOptimizer.parameter(0));
  if (GPSOffset)
  {
    Eigen::Translation3d t(x, y, z);
    Eigen::Isometry3d Trans(t);
    GPSOffset->setOffset(Trans);
  }
  else
  {
    std::cerr << "ERROR : The first g2o parameter is not an SO3 Offset" << std::endl;
  }
}


//------------------------------------------------------------------------------
bool PoseGraphOptimization::Process(const std::vector<Transform>& slamPoses,
                                    const std::vector<Transform>& gpsPoses,
                                    const std::vector<std::array<double, 36>>& slamCov,
                                    const std::vector<std::array<double, 9>>& gpsCov,
                                    std::vector<Transform>& optimizedSlamPoses)
{
  unsigned int nbSlamPoses = slamPoses.size();
  unsigned int nbGpsPoses = gpsPoses.size();

  // Check input vector sizes (at least 2 elements)
  if ((nbSlamPoses < 2) || (nbGpsPoses < 2))
  {
    std::cerr << "ERROR : SLAM and GPS trajectories must have at least 2 points "
              << "(Got " << nbSlamPoses << " SLAM poses and " << nbGpsPoses << " GPS positions)."
              << std::endl;
    return false;
  }

  // Check timestamps
  double gpsInitTime = gpsPoses[0].time;
  double gpsEndTime = gpsPoses[nbGpsPoses - 1].time;
  double slamInitTime = slamPoses[0].time;
  double slamEndTime = slamPoses[nbSlamPoses - 1].time;
  auto checkInterval = [](double t, double tmin, double tmax) { return tmin < t && t < tmax; };
  if (!checkInterval(gpsInitTime, slamInitTime, slamEndTime) &&
      !checkInterval(slamInitTime, gpsInitTime, gpsEndTime))
  {
    std::cerr << "ERROR : Slam time and GPS time do not match. "
              << "GPS = ("<< gpsInitTime << ", " << gpsEndTime << "); "
              << "SLAM = ("<< slamInitTime << ", " << slamEndTime << "). "
              << "Please indicate the time offset." << std::endl;
    return false;
  }

  // Compute translation between gps and slam data
  Eigen::Translation3d translation = ComputeTranslationOffset(slamPoses, gpsPoses);
  translation = translation.inverse();

  // Compute rotation between gps and slam data
  Eigen::Quaterniond rotation = ComputeRotationOffset(slamPoses, gpsPoses);

  // Apply transformation to GPS position
  std::vector<Transform> transGpsPoses = gpsPoses;  // DEBUG
  // std::vector<Transform> transGpsPoses(nbGpsPoses);
  // for (unsigned int i = 0; i < nbGpsPoses; ++i)
  // {
  //   Eigen::Vector3d initialGpsPose(gpsPoses[i].x, gpsPoses[i].y, gpsPoses[i].z);
  //   Eigen::Vector3d finalGpsPose(translation * initialGpsPose);
  //   transGpsPoses[i].x = finalGpsPose.x();
  //   transGpsPoses[i].y = finalGpsPose.y();
  //   transGpsPoses[i].z = finalGpsPose.z();
  //   transGpsPoses[i].time = gpsPoses[i].time + this->TimeOffset;
  // }

  // Apply transformation to SLAM position
  std::vector<Transform> transSlamPoses = slamPoses;  // DEBUG
  // std::vector<Transform> transSlamPoses(nbSlamPoses);
  // for (unsigned int i = 0; i < nbSlamPoses; ++i)
  // {
  //   Eigen::Vector3d initialSlamPose(slamPoses[i].x, slamPoses[i].y, slamPoses[i].z);
  //   Eigen::Vector3d finalSlamPose(rotation * initialSlamPose);
  //   // Eigen::Vector3d finalSlamPose(rotation * translation * initialSlamPose);
  //   transSlamPoses[i].x = finalSlamPose.x();
  //   transSlamPoses[i].y = finalSlamPose.y();
  //   transSlamPoses[i].z = finalSlamPose.z();
  //   transSlamPoses[i].time = slamPoses[i].time;
  // }

  this->GraphOptimizer.clear();
  int idCount = 0;

  // Handle SLAM data
  for (unsigned int i = 0; i < nbSlamPoses; ++i)
  {
    // Get current SLAM pose
    Eigen::Isometry3d H = transSlamPoses[i].GetIsometry();

    // Add SLAM pose (position + orientation) as a vertex
    g2o::VertexSE3* poseVertex = new g2o::VertexSE3;
    g2o::SE3Quat pose(H.linear(), H.translation());
    poseVertex->setId(++idCount);
    poseVertex->setEstimate(pose);
    poseVertex->setFixed(false);
    this->GraphOptimizer.addVertex(poseVertex);

    // Add edge between 2 consecutive SLAM poses
    if (i > 0)
    {
      // Get edge between two last SLAM poses
      Eigen::Isometry3d H_rel = GetRelativeTransform(transSlamPoses[i-1], transSlamPoses[i]);
      Eigen::Map<Eigen::Matrix6d> covMatrix((double*) slamCov[i].data());

      // Add edge to pose graph
      auto slamEdge = std::make_unique<g2o::EdgeSE3>();
      slamEdge->vertices()[0] = this->GraphOptimizer.vertex(i - 1);
      slamEdge->vertices()[1] = this->GraphOptimizer.vertex(i);
      slamEdge->setMeasurement(H_rel);
      slamEdge->setInformation(covMatrix.inverse());
      this->GraphOptimizer.addEdge(slamEdge.release());
    }
  }

  // Handle GPS data
  int prevFoundId = -1;
  for (unsigned int i = 0; i < nbGpsPoses; ++i)
  {
    // TODO can be optimized in order to not search again through all slam poses.
    int foundId = FindClosestSlamPose(transGpsPoses[i], transSlamPoses);

    // Check matching validity, and ensure that the found slam pose is different
    // from the previous one (to prevent matching a single SLAM point to 2 
    // different GPS points).
    // CHECK if SLAM points are sparser than GPS, the first GPS/SLAM match may not be the best one.
    if ((foundId != -1) && (foundId != prevFoundId))
    {
      prevFoundId = foundId;

       // Get current GPS pose and covariance
      Eigen::Isometry3d H = transGpsPoses[i].GetIsometry();
      Eigen::Map<Eigen::Matrix3d> covMatrix((double*) gpsCov[i].data());

      // Add GPS position as a vertex
      auto poseVertex = std::make_unique<g2o::VertexPointXYZ>();
      poseVertex->setId(++idCount);
      poseVertex->setEstimate(H.translation());
      poseVertex->setFixed(true);
      poseVertex->setMarginalized(true);
      this->GraphOptimizer.addVertex(poseVertex.release());

      // Add an edge between the temporal closest SLAM vertex
      auto gpsEdge = std::make_unique<g2o::EdgeSE3PointXYZ>();
      gpsEdge->vertices()[1] = this->GraphOptimizer.vertex(idCount);
      gpsEdge->vertices()[0] = this->GraphOptimizer.vertex(foundId); // get the temporal closest SLAM vertex
      gpsEdge->setMeasurement(Eigen::Vector3d::Zero());  // CHECK we want to merge this SLAM point to this GPS point
      gpsEdge->setInformation(covMatrix.inverse());
      gpsEdge->setParameterId(0, 0); // tel the edge to use the Lidar/GPS calibration which is define inside the graph
      this->GraphOptimizer.addEdge(gpsEdge.release());
    }
  }

  // Save Graph before optimization
  if (this->SaveG2OFile)
  {
    if (!G2OFileName.empty())
      this->GraphOptimizer.save(G2OFileName.c_str());
    else
      std::cout << "WARNING : Could not save the g2o graph. Please specify a filename" << std::endl;
  }

  // Optimize the graph
  this->GraphOptimizer.initializeOptimization();
  this->GraphOptimizer.optimize(this->NbIteration);

  // Set the output data
  optimizedSlamPoses.clear();
  optimizedSlamPoses.resize(nbSlamPoses);
  for (int i = 0; i < nbSlamPoses; ++i)
  {
    auto* v = this->GraphOptimizer.vertex(i);
    if (v)  // CHECK can fail ?
    {
      // Get optimization result
      double data[7];
      v->getEstimateData(data);
      // Fill new optimized trajectory
      double time = transSlamPoses[i].time;
      Eigen::Translation3d trans(data[0], data[1], data[2]);
      Eigen::Quaterniond rot(data[6], data[3], data[4], data[5]);
      optimizedSlamPoses[i] = Transform(time, trans, rot);
    }
  }

  // Print debug info
  if (this->Verbose)
  {
    std::cout << "\nThe Graph is composed of: \n"
              << "\t " << nbSlamPoses     << "\t slam vertices \n"
              << "\t " << nbSlamPoses - 1 << "\t slam edges \n"
              << "\t " << nbGpsPoses      << "\t GPS vertices \n"
              << "\t " << this->GraphOptimizer.vertices().size() - (nbSlamPoses - 1) << "\t GPS edges \n"
              << std::endl;
  }

  return true;
}