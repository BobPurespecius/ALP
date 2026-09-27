// Reproduce the current continuousLocalSfcMaxViolation time-origin error.
// This audit does not change production or corridor geometry.
#include <optimizer/poly_traj_utils.hpp>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdio>

int main()
{
  Eigen::Matrix3d head, tail;
  head << Eigen::Vector3d(0,0,0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  tail << Eigen::Vector3d(0.424264,0,0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  Eigen::MatrixXd inner(3,2);
  inner.col(0) = Eigen::Vector3d(0.141421,0,0);
  inner.col(1) = Eigen::Vector3d(0.282843,0,0);
  Eigen::VectorXd durations = Eigen::VectorXd::Constant(3, 0.54);
  poly_traj::MinJerkOpt mjo;
  mjo.reset(head, tail, 3);
  mjo.generate(inner, durations);
  auto traj = mjo.getTraj();
  const int piece = 1;
  const double piece_start = durations(0);
  const double piece_end = piece_start + durations(1);
  const double world_time = piece_start + 0.10;
  const double direct = traj.getPos(world_time).x();
  const auto coeff = traj[piece].getCoeffMat();
  auto eval = [&](double s) {
    double result = 0.0;
    for (int k=5;k>=0;--k)
      result = result*s + coeff(0,5-k);
    return result;
  };
  const double correct = eval(world_time - piece_start);
  // Production sets t_piece_start=t_piece_end before this subtraction.
  const double production = eval(world_time - piece_end);
  const double corridor_left_wall = 0.14;
  std::printf("piece_start=%.6f piece_end=%.6f world_time=%.6f\n",
              piece_start, piece_end, world_time);
  std::printf("direct_x=%.6f correct_local_x=%.6f production_local_x=%.6f\n",
              direct, correct, production);
  std::printf("corridor_x_min=%.6f direct_violation=%.6f "
              "production_violation=%.6f\n", corridor_left_wall,
              std::max(0.0, corridor_left_wall-direct),
              std::max(0.0, corridor_left_wall-production));
  const bool pass = std::abs(direct-correct) < 1.0e-8 &&
                    direct > corridor_left_wall &&
                    production < corridor_left_wall;
  std::printf("time_origin_bug_reproduced=%d\n", (int)pass);
  return pass ? 0 : 1;
}
