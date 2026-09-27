// Deterministic MINCO timing check using Feedback119's logged three-piece
// short-time case (0.424264 m path, 0.058926 s per piece). The original
// candidate's endpoint PVA and inner points were not logged, so this is a
// same-scale reconstruction, not an exact trajectory replay.
#include <optimizer/poly_traj_utils.hpp>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdio>

struct Rates { double v, a, j; };

static Rates rates(const poly_traj::Trajectory &traj)
{
  Rates result{traj.getMaxVelRate(), traj.getMaxAccRate(), 0.0};
  const double duration = traj.getTotalDuration();
  const double dt = std::max(0.02, std::min(0.05, duration / 100.0));
  for (double t = 0.0; t < duration + 1.0e-6; t += dt)
    result.j = std::max(result.j, traj.getJer(std::min(t, duration)).norm());
  result.j = std::max(result.j, traj.getJer(duration).norm());
  return result;
}

int main()
{
  constexpr int pieces = 3;
  Eigen::Matrix3d head, tail;
  head << Eigen::Vector3d(0.0, 0.0, 1.0),
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  tail << Eigen::Vector3d(0.424264, 0.0, 1.0),
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  Eigen::MatrixXd inner(3, pieces - 1);
  inner.col(0) = Eigen::Vector3d(0.141421, 0.0, 1.0);
  inner.col(1) = Eigen::Vector3d(0.282843, 0.0, 1.0);
  Eigen::VectorXd durations = Eigen::VectorXd::Constant(pieces, 0.058926);
  poly_traj::MinJerkOpt seed;
  seed.reset(head, tail, pieces);
  seed.generate(inner, durations);
  const Rates before = rates(seed.getTraj());
  const double old_duration = seed.getTraj().getTotalDuration();
  const double scale = std::max({1.0, before.v / 3.0,
                                std::sqrt(before.a / 6.0),
                                std::cbrt(before.j / 20.0)});
  const double corridor_start = 0.030, corridor_end = 0.140;
  const double los_world_start = 1790363549.500;
  durations *= scale;
  seed.reset(head, tail, pieces);
  seed.generate(inner, durations);
  const Rates after = rates(seed.getTraj());
  const double new_duration = seed.getTraj().getTotalDuration();
  const bool pass = std::isfinite(scale) && scale > 1.0 &&
      before.j > 1000.0 && after.j < before.j / 100.0 &&
      after.v <= 3.0 * 1.01 && after.a <= 6.0 * 1.01 &&
      after.j <= 20.0 * 1.01 &&
      std::abs(new_duration - old_duration * scale) < 1.0e-9 &&
      std::abs(corridor_start * scale / new_duration -
               corridor_start / old_duration) < 1.0e-9 &&
      std::abs(corridor_end * scale / new_duration -
               corridor_end / old_duration) < 1.0e-9 &&
      los_world_start == 1790363549.500;
  std::printf("before v=%.6f a=%.6f j=%.6f duration=%.6f\n",
              before.v, before.a, before.j, old_duration);
  std::printf("lambda=%.6f after v=%.6f a=%.6f j=%.6f duration=%.6f\n",
              scale, after.v, after.a, after.j, new_duration);
  std::printf("static_window=[%.6f,%.6f] scaled=[%.6f,%.6f] "
              "los_world_start=%.3f result=%s\n",
              corridor_start, corridor_end, corridor_start * scale,
              corridor_end * scale, los_world_start, pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
