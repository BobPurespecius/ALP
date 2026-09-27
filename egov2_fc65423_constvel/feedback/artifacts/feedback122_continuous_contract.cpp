#include <optimizer/poly_traj_optimizer.h>

#include <cassert>
#include <cmath>
#include <iostream>

using ego_planner::LocalSfcPlane;
using ego_planner::PolyTrajOptimizer;

static poly_traj::CoefficientMat coefficients(
    const std::initializer_list<double> ascending)
{
  poly_traj::CoefficientMat matrix = poly_traj::CoefficientMat::Zero();
  int power = 0;
  for (double coefficient : ascending)
    matrix(0, 5 - power++) = coefficient;
  return matrix;
}

static LocalSfcPlane plane(const int piece, const double u0,
                           const double u1, const double clearance = 0.0)
{
  LocalSfcPlane p;
  p.source = LocalSfcPlane::STATIC_COLLISION_CORRIDOR;
  p.normal = Eigen::Vector3d::UnitX();
  p.point = Eigen::Vector3d::Zero();
  p.piece_id = piece;
  p.piece_u_begin = u0;
  p.piece_u_end = u1;
  p.clearance = clearance;
  p.active_start = 0.0;  // stale legacy seconds must have no effect
  p.active_end = 100.0;
  return p;
}

static void near(const double actual, const double expected,
                 const double tolerance = 1.0e-7)
{
  if (!(std::abs(actual - expected) <= tolerance))
  {
    std::cerr << "actual=" << actual << " expected=" << expected << '\n';
    std::abort();
  }
}

int main()
{
  PolyTrajOptimizer optimizer;
  {
    poly_traj::Trajectory path({1.0}, {coefficients({0.0, 1.0})});
    near(optimizer.continuousLocalSfcMaxViolation(path, {plane(0, 0, 1)},
                                                   nullptr), 0.0);
  }
  {
    poly_traj::Trajectory path({1.0}, {coefficients({0.5, -1.0})});
    near(optimizer.continuousLocalSfcMaxViolation(path, {plane(0, 0, 1)},
                                                   nullptr), 0.5);
    near(optimizer.continuousLocalSfcMaxViolation(path, {plane(0, 0.2, 0.4)},
                                                   nullptr), 0.0);
  }
  {
    poly_traj::Trajectory path({1.0},
        {coefficients({0.4, -1.0, 0.0, 0.0, 0.0, 1.0})});
    const double critical = std::pow(0.2, 0.25);
    const double minimum = 0.4 - critical + std::pow(critical, 5);
    near(optimizer.continuousLocalSfcMaxViolation(path, {plane(0, 0, 1)},
                                                   nullptr), -minimum);
  }
  {
    // The second piece has a different duration and a clipped local window.
    // The global time origin of that piece is 0.7, not its end at 2.0.
    poly_traj::Trajectory path({0.7, 1.3},
        {coefficients({10.0}), coefficients({0.8, -1.0})});
    near(optimizer.continuousLocalSfcMaxViolation(path, {plane(1, 0.25, 0.75)},
                                                   nullptr), 0.175);
  }
  {
    // A degree-reduced polynomial with a repeated stationary root.
    poly_traj::Trajectory path({1.0},
        {coefficients({0.25, -1.0, 1.0})});
    near(optimizer.continuousLocalSfcMaxViolation(path,
            {plane(0, 0, 1, 0.1)}, nullptr), 0.1);
  }
  {
    // A quartic with a double root and two simple roots.
    Eigen::Matrix<double, 5, 1> c;
    c << 0.0252, -0.316, 1.31, -2.0, 1.0;
    const auto roots = PolyTrajOptimizer::quarticRealRoots(c, 0.0, 1.0);
    assert(roots.size() == 3);
    near(roots[0], 0.2, 1.0e-5);
    near(roots[1], 0.7, 1.0e-5);
    near(roots[2], 0.9, 1.0e-5);
  }
  std::cout << "continuous corridor contract: PASS\n";
}
