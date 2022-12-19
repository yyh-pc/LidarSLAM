
#include "LidarSlam/InterpolationModels.h"
#include "LidarSlam/State.h"
#include "LidarSlam/Utilities.h"
#include <Eigen/Geometry>
#include <cmath>

// Test a model of interpolation at time t
void test_transfo(LidarSlam::Interpolation::IModel *interpolation, double t)
{
  Eigen::MatrixXd M((*interpolation)(t).matrix());
  std::cout << "Interpolation at t = " << t << " :" << std::endl << M << std::endl;
}

// Create a PoseStamped from a position and a time
LidarSlam::PoseStamped createTranslationState(double x, double y, double z, double t)
{
  Eigen::UnalignedIsometry3d iso = LidarSlam::Utils::XYZRPYtoIsometry(x, y, z, 0, 0, 0);
  LidarSlam::PoseStamped state(iso, t);
  return state;
}

/**
 * @brief Create a Rotation State object from euler angle (x, y ,z) in rad
 */
LidarSlam::PoseStamped createRotationState(double x, double y, double z, double t)
{
  Eigen::UnalignedIsometry3d iso = LidarSlam::Utils::XYZRPYtoIsometry(0, 0, 0, x, y, z);
  LidarSlam::PoseStamped state(iso, t);
  return state;
}

/**
 * @brief Create complete State with translation AND rotation
 * with translation : (px, py, pz) and rotation with euler angle : (rx, ry, rz)
 */
LidarSlam::PoseStamped createFullState(double px, double py, double pz, double rx, double ry, double rz, double t)
{
  Eigen::UnalignedIsometry3d iso = LidarSlam::Utils::XYZRPYtoIsometry(px, py, pz, rx, ry, rz);
  LidarSlam::PoseStamped state(iso, t);
  return state;
}

  // ---------------------------------------------------------------------------
  //   Linear Translation Models
  // ---------------------------------------------------------------------------

// test the LidarSlam::Interpolation::Linear function
void  test_linear_interpolation(void)
{
  static std::string delimiter = "********";
  std::cout << delimiter <<" LINEAR INTERPOLATION TEST " << delimiter << std::endl;
  auto state0 = createTranslationState(1.5, 1.5, 1.5, 1);
  auto state1 = createTranslationState(0, 0, 0, 0);
  auto state2 = createTranslationState(2, 2, 2, 1);
  auto state3 = createTranslationState(3, 3, 3, 2);

  std::cout << delimiter << " invalid data" << delimiter << std::endl;
  std::vector<LidarSlam::PoseStamped> vec1{state0};
  LidarSlam::Interpolation::Linear invalid_lin(vec1);
  test_transfo(&invalid_lin, 0);
  test_transfo(&invalid_lin, 0.5);
  std::cout << delimiter << " try recompute invalid data" << delimiter << std::endl;
  invalid_lin.RecomputeModel(vec1);
  test_transfo(&invalid_lin, 1);
  std::cout << std::endl;

  std::cout << delimiter << " f(t) = (2t, 2t, 2t) translation " << delimiter << std::endl;
  vec1[0] = state1;
  vec1.push_back(state2);
  LidarSlam::Interpolation::Linear normal1_lin(vec1);
  test_transfo(&normal1_lin, 0);
  test_transfo(&normal1_lin, 1);
  test_transfo(&normal1_lin, 3);
  std::cout << std::endl;

  std::cout << delimiter << " f(t) = (t, 0, -2t) translation " << delimiter << std::endl;
  vec1[1] = createTranslationState(1, 0, -2, 1);
  normal1_lin.RecomputeModel(vec1);
  test_transfo(&normal1_lin, 0);
  test_transfo(&normal1_lin, 1);
  test_transfo(&normal1_lin, 3);
  std::cout << std::endl;

  vec1[1] = state2;
  vec1.push_back(state3);
  std::cout << delimiter << " f(t) = (1 + t, 1 + t, 1 + t) translation with 3 data" << delimiter << std::endl;
  normal1_lin.RecomputeModel(vec1);
  test_transfo(&normal1_lin, 0);
  test_transfo(&normal1_lin, 1);
  test_transfo(&normal1_lin, 3);
  std::cout << std::endl;

}

  // ---------------------------------------------------------------------------
  //   Spline Translation Models
  // ---------------------------------------------------------------------------
// test the LidarSlam::Interpolation::Spline function
void test_spline_interpolation(void)
{
  static std::string delimiter = "********";
  std::cout << delimiter <<" SPLINE INTERPOLATION TEST " << delimiter << std::endl;

  std::cout << delimiter << "const spline with one value" << delimiter << std::endl;
  auto state0 = createTranslationState(1, 1, 1, 1);
  std::vector<LidarSlam::PoseStamped> vec0{state0};
  LidarSlam::Interpolation::Spline const0_spline(vec0, 1);
  test_transfo(&const0_spline, 1);
  test_transfo(&const0_spline, 2);

  std::cout << delimiter << "const spline f(t) = (1, 1, 1)" << delimiter << std::endl;
  auto state1 = createTranslationState(1, 1, 1, 1);
  auto state2 = createTranslationState(1, 1, 1, 2);
  std::vector<LidarSlam::PoseStamped> vec01{state1, state2};
  LidarSlam::Interpolation::Spline const1_spline(vec01, 2);
  test_transfo(&const1_spline, 0);
  test_transfo(&const1_spline, 1);
  test_transfo(&const1_spline, 3);
  std::cout << std::endl;

  std::cout << delimiter << " identity spline f(t) = (t, t, t)" << delimiter << std::endl;
  state1 = createTranslationState(0, 0, 0, 0);
  state2 = createTranslationState(1, 1, 1, 1);
  auto state3 = createTranslationState(2, 2, 2, 2);
  std::vector<LidarSlam::PoseStamped> vec1{state1, state2, state3};
  LidarSlam::Interpolation::Spline ident1_spline(vec1, 1);
  test_transfo(&ident1_spline, 0);
  test_transfo(&ident1_spline, 1);
  test_transfo(&ident1_spline, 3);
  std::cout << std::endl;

  std::cout << delimiter << " quadratic spline f(t) = (2, t, t^2)" << delimiter << std::endl;
  vec1[0] = createTranslationState(2, 0, 0, 0);
  vec1[1] = createTranslationState(2, 1, 1, 1);
  vec1[2] = createTranslationState(2, 2, 4, 2);
  LidarSlam::Interpolation::Spline quad1_spline(vec1, 2);
  test_transfo(&quad1_spline, 0);
  test_transfo(&quad1_spline, 1);
  test_transfo(&quad1_spline, 3);
  std::cout << std::endl;

  std::cout << delimiter << " cubic spline f(t) = (2, -t, t^3)" << delimiter << std::endl;
  vec1[1] = createTranslationState(2, -1, 1, 1);
  vec1[2] = createTranslationState(2, -2, 8, 2);
  vec1.push_back(createTranslationState(2, -3, 27, 3));
  LidarSlam::Interpolation::Spline cub1_spline(vec1, 3);
  test_transfo(&cub1_spline, 0);
  test_transfo(&cub1_spline, 1);
  test_transfo(&cub1_spline, 4);
}

  // ---------------------------------------------------------------------------
  //   N-Slerp Rotation Models
  // ---------------------------------------------------------------------------
// test the LidarSlam::Interpolation::NSlerp function
void test_N_Slerp(void)
{
  static std::string delimiter = "********";
  std::cout << delimiter <<" N-SLERP INTERPOLATION TEST " << delimiter << std::endl;

  auto state1(createRotationState(0, 0, 0, 0));
  auto state2(createRotationState(0, 0, 0, 1));
  auto state3(createRotationState(0, M_PI / 4, 0, 2));

  std::cout << delimiter << " N-Slerp with empty values" << delimiter << std::endl;
  LidarSlam::Interpolation::NSlerp null_slerp{std::vector<LidarSlam::PoseStamped>()};
  test_transfo(&null_slerp, 0);
  test_transfo(&null_slerp, 1);
  test_transfo(&null_slerp, 3);
  std::cout << std::endl;

  std::cout << delimiter << " N-slerp with 3 points : 0 to 1: null rotation" << delimiter << std::endl;
  std::vector<LidarSlam::PoseStamped> vec1{state1, state2, state3};
  LidarSlam::Interpolation::NSlerp normal_nslerp1(vec1);
  test_transfo(&normal_nslerp1, 0);
  test_transfo(&normal_nslerp1, 0.5);
  test_transfo(&normal_nslerp1, 1);
  std::cout << delimiter << " N-slerp with 3 points : 1 to 2 : 90° rotation over Y " << delimiter << std::endl;
  test_transfo(&normal_nslerp1, 1.5);
  test_transfo(&normal_nslerp1, 2);
  test_transfo(&normal_nslerp1, 3);
  std::cout << std::endl;
}

  // ---------------------------------------------------------------------------
  //   Trajectory Models
  // ---------------------------------------------------------------------------
// test the LidarSlam::Interpolation::Trajectory function
void  test_trajectory_interpolation(void)
{
  static std::string delimiter = "********";
  std::cout << delimiter <<" COMPLETE TRAJECTORY INTERPOLATION TEST " << delimiter << std::endl;

  std::cout << delimiter << " traj t -> (t, t, t, 0, 0, pi/4 * t) " << delimiter << std::endl;
  auto state1(createFullState(0., 0., 0., 0., 0., 0., 0.));
  auto state2(createFullState(1., 1., 1., 0., 0., M_PI / 4., 1.));
  std::vector<LidarSlam::PoseStamped> vec1{state1, state2};

  LidarSlam::Interpolation::Trajectory traj1(vec1, LidarSlam::Interpolation::Model::LINEAR);
  test_transfo(&traj1, 0);
  test_transfo(&traj1, 0.5);
  test_transfo(&traj1, 1);
  std::cout << std::endl;

  std::cout << delimiter << " traj t -> (t2, t2, t2, 0, 0, pi/4 * t) " << delimiter << std::endl;
  auto state3(createFullState(4., 4., 4., 0., 0., M_PI / 2., 2.));
  vec1.push_back(state3);
  traj1.SetModel(vec1, LidarSlam::Interpolation::Model::QUADRATIC_SPLINE);
  test_transfo(&traj1, 0);
  test_transfo(&traj1, 0.5);
  test_transfo(&traj1, 1);
  std::cout << std::endl;
}

int main(void)
{
  // test_linear_interpolation();
  // test_spline_interpolation();
  test_N_Slerp();
  // test_trajectory_interpolation();

  return 0;
}