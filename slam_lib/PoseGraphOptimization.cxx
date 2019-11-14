#include "PoseGraphOptimization.h"
#include "GlobalTrajectoriesRegistration.h"

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
}


//------------------------------------------------------------------------------
PoseGraphOptimization::PoseGraphOptimization()
{
  // create optimizer
  // TODO change optimizer
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
void PoseGraphOptimization::SetGPSCalibration(double x, double y, double z, double rx, double ry, double rz)
{
  Eigen::Translation3d t(x, y, z);
  Eigen::Quaterniond r(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
  Eigen::Isometry3d transform(r * t);
  this->SetGPSCalibration(transform);
}

//------------------------------------------------------------------------------
void PoseGraphOptimization::SetGPSCalibration(const Eigen::Isometry3d& sensorToGps)
{
  auto* GPSOffset = dynamic_cast<g2o::ParameterSE3Offset*>(this->GraphOptimizer.parameter(0));
  if (GPSOffset)
    GPSOffset->setOffset(sensorToGps);
  else
    std::cerr << "ERROR : The first g2o parameter is not an SO3 Offset" << std::endl;
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

  // Compute transformation from SLAM to GPS data
  // TODO Impose transform to have no roll
  GlobalTrajectoriesRegistration registration;
  registration.SetVerbose(this->Verbose);
  Eigen::Isometry3d tfSlamToGps;
  registration.ComputeTransformOffset(slamPoses, gpsPoses, tfSlamToGps);

  // Apply transformation to SLAM poses
  std::vector<Transform> transSlamPoses(nbSlamPoses);
  for (unsigned int i = 0; i < nbSlamPoses; ++i)
  {
    // // DEBUG what about rotation part ?!
    // Eigen::Vector3d initialSlamPose(slamPoses[i].x, slamPoses[i].y, slamPoses[i].z);
    // Eigen::Vector3d finalSlamPose(tfSlamToGps * initialSlamPose);
    // transSlamPoses[i].x = finalSlamPose.x();
    // transSlamPoses[i].y = finalSlamPose.y();
    // transSlamPoses[i].z = finalSlamPose.z();
    // transSlamPoses[i].time = slamPoses[i].time + this->TimeOffset;
    // TODO Use this instead
    Eigen::Isometry3d finalSlamPose(tfSlamToGps * slamPoses[i].GetIsometry());
    transSlamPoses[i] = Transform(slamPoses[i].time + this->TimeOffset, finalSlamPose);
  }

  this->GraphOptimizer.clear();
  int idCount = -1;

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
    int foundId = FindClosestSlamPose(gpsPoses[i], transSlamPoses);

    // Check matching validity, and ensure that the found slam pose is different
    // from the previous one (to prevent matching a single SLAM point to 2 
    // different GPS points).
    // CHECK if SLAM points are sparser than GPS, the first GPS/SLAM match may not be the best one.
    if ((foundId != -1) && (foundId != prevFoundId))
    {
      prevFoundId = foundId;

       // Get current GPS pose and covariance
      Eigen::Isometry3d H = gpsPoses[i].GetIsometry();
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

  // Print debug info
  if (this->Verbose)
  {
    std::cout << "\nThe Graph is composed of: \n"
              << "\t " << nbSlamPoses     << "\t SLAM vertices \n"
              << "\t " << nbSlamPoses - 1 << "\t SLAM edges \n"
              << "\t " << this->GraphOptimizer.vertices().size() - nbSlamPoses    << "\t GPS vertices \n"
              << "\t " << this->GraphOptimizer.edges().size() - (nbSlamPoses - 1) << "\t GPS/SLAM edges \n";
  }

  // Optimize the graph
  this->GraphOptimizer.initializeOptimization();
  int iterations = this->GraphOptimizer.optimize(this->NbIteration);

  // Print debug info
  if (this->Verbose)
  {
    std::cout << "Pose graph optimization succeeded in " << iterations << " iterations.\n" << std::endl;
  }

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

  return true;
}