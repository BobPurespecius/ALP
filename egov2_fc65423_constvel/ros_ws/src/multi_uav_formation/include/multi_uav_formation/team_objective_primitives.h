#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace multi_uav_formation
{

struct TeamObjectiveWeights
{
  double k2{8.0};
  double accumulated{1.0};
  double complementarity{1.5};
  double encirclement{0.35};
  double regularization{0.20};
};

struct TeamObjectiveTerms
{
  double k2{0.0};
  double accumulated{0.0};
  double complementarity{0.0};
  double encirclement{0.0};
  double regularization{0.0};

  double weightedK2(const TeamObjectiveWeights &w) const { return w.k2 * k2; }
  double weightedAccumulated(const TeamObjectiveWeights &w) const
  {
    return w.accumulated * accumulated;
  }
  double weightedComplementarity(const TeamObjectiveWeights &w) const
  {
    return w.complementarity * complementarity;
  }
  double weightedEncirclement(const TeamObjectiveWeights &w) const
  {
    return w.encirclement * encirclement;
  }
  double weightedRegularization(const TeamObjectiveWeights &w) const
  {
    return w.regularization * regularization;
  }
  double total(const TeamObjectiveWeights &w) const
  {
    return weightedK2(w) + weightedAccumulated(w) +
           weightedComplementarity(w) + weightedEncirclement(w) +
           weightedRegularization(w);
  }
};

// Normalized quality shared by the cheap topology pass, every continuous
// refinement stage, and the final Team proposal guard.  The first three
// entries are dimensionless [0, 1] qualities (larger is better); the final
// two are soft costs (smaller is better).
struct TeamQualityVector
{
  double q2_ratio{0.0};
  double accumulated_visibility_ratio{0.0};
  double complementarity{0.0};
  double encirclement_cost{std::numeric_limits<double>::infinity()};
  double regularization_cost{std::numeric_limits<double>::infinity()};

  bool allFinite() const
  {
    return std::isfinite(q2_ratio) &&
           std::isfinite(accumulated_visibility_ratio) &&
           std::isfinite(complementarity) &&
           std::isfinite(encirclement_cost) &&
           std::isfinite(regularization_cost);
  }
};

struct TeamQualityTolerance
{
  double q2{1.0e-3};
  double accumulated_visibility{1.0e-3};
  double complementarity{1.0e-3};
};

// One hundredth of one time-sample's normalized mass.  This ties the
// lexicographic equality band to discretization/convergence resolution,
// rather than introducing a task-level allowance for visibility loss.
inline TeamQualityTolerance teamQualityToleranceFromSampling(
    const double horizon, const double sample_dt)
{
  const double normalized_sample = sample_dt / std::max(1.0e-9, horizon);
  const double epsilon = std::max(
      1.0e-5, std::min(1.0e-3, 0.01 * normalized_sample));
  return TeamQualityTolerance{epsilon, epsilon, epsilon};
}

inline bool betterTeamQuality(
    const TeamQualityVector &lhs, const TeamQualityVector &rhs,
    const TeamQualityTolerance &tolerance = {})
{
  if (!lhs.allFinite())
    return false;
  if (!rhs.allFinite())
    return true;
  if (lhs.q2_ratio > rhs.q2_ratio + tolerance.q2)
    return true;
  if (rhs.q2_ratio > lhs.q2_ratio + tolerance.q2)
    return false;
  if (lhs.accumulated_visibility_ratio >
      rhs.accumulated_visibility_ratio +
          tolerance.accumulated_visibility)
    return true;
  if (rhs.accumulated_visibility_ratio >
      lhs.accumulated_visibility_ratio +
          tolerance.accumulated_visibility)
    return false;
  if (lhs.complementarity >
      rhs.complementarity + tolerance.complementarity)
    return true;
  if (rhs.complementarity >
      lhs.complementarity + tolerance.complementarity)
    return false;
  if (lhs.encirclement_cost + 1.0e-9 < rhs.encirclement_cost)
    return true;
  if (rhs.encirclement_cost + 1.0e-9 < lhs.encirclement_cost)
    return false;
  return lhs.regularization_cost + 1.0e-9 < rhs.regularization_cost;
}

inline bool teamPrimaryVisibilityNonWorsening(
    const TeamQualityVector &candidate, const TeamQualityVector &baseline,
    const TeamQualityTolerance &tolerance = {})
{
  if (!candidate.allFinite() || !baseline.allFinite() ||
      candidate.q2_ratio < baseline.q2_ratio - tolerance.q2)
    return false;
  // Accumulated visibility is the second primary criterion whenever Q2 is
  // numerically equivalent.  Stages currently preserve it unconditionally;
  // this explicit branch documents the final proposal contract.
  if (std::abs(candidate.q2_ratio - baseline.q2_ratio) <= tolerance.q2 &&
      candidate.accumulated_visibility_ratio <
          baseline.accumulated_visibility_ratio -
              tolerance.accumulated_visibility)
    return false;
  return true;
}

inline bool teamQualitySatisfiesFloors(
    const TeamQualityVector &candidate, const double q2_floor,
    const double visibility_floor,
    const double complementarity_floor =
        -std::numeric_limits<double>::infinity())
{
  return candidate.allFinite() &&
         candidate.q2_ratio + 1.0e-9 >= q2_floor &&
         candidate.accumulated_visibility_ratio + 1.0e-9 >=
             visibility_floor &&
         candidate.complementarity + 1.0e-9 >= complementarity_floor;
}

inline double wrapTeamAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double teamQ2(const std::array<double, 3> &v)
{
  return v[0] * v[1] + v[0] * v[2] + v[1] * v[2] -
         2.0 * v[0] * v[1] * v[2];
}

inline std::array<double, 3> teamQ2Gradient(
    const std::array<double, 3> &v)
{
  return {{v[1] + v[2] - 2.0 * v[1] * v[2],
           v[0] + v[2] - 2.0 * v[0] * v[2],
           v[0] + v[1] - 2.0 * v[0] * v[1]}};
}

struct PairQualitySample
{
  bool valid{false};
  double angle{0.0};
  double value{0.0};
  double derivative_angle{0.0};
  Eigen::Vector3d gradient_first{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gradient_second{Eigen::Vector3d::Zero()};
};

// The one shared 25 -> 60 degree smoothstep used by cheap tuple ranking and
// the continuous Team objective.  The gradients are with respect to the two
// target-centred relative vectors.
inline PairQualitySample targetCenteredPairQuality(
    const Eigen::Vector3d &first, const Eigen::Vector3d &second,
    const double bad_angle = 25.0 * M_PI / 180.0,
    const double good_angle = 60.0 * M_PI / 180.0)
{
  PairQualitySample result;
  const double n0 = first.norm();
  const double n1 = second.norm();
  if (!first.allFinite() || !second.allFinite() || n0 <= 1.0e-8 ||
      n1 <= 1.0e-8 || !std::isfinite(bad_angle) ||
      !std::isfinite(good_angle) || good_angle <= bad_angle + 1.0e-9)
    return result;
  const double cosine = std::max(-1.0, std::min(1.0,
      first.dot(second) / (n0 * n1)));
  result.angle = std::acos(cosine);
  result.valid = true;
  if (result.angle <= bad_angle)
    return result;
  if (result.angle >= good_angle)
  {
    result.value = 1.0;
    return result;
  }
  const double u = (result.angle - bad_angle) / (good_angle - bad_angle);
  result.value = u * u * (3.0 - 2.0 * u);
  result.derivative_angle = 6.0 * u * (1.0 - u) /
                            (good_angle - bad_angle);
  const double sine = std::sqrt(std::max(1.0e-12, 1.0 - cosine * cosine));
  const Eigen::Vector3d dc_df = second / (n0 * n1) -
      cosine * first / (n0 * n0);
  const Eigen::Vector3d dc_ds = first / (n0 * n1) -
      cosine * second / (n1 * n1);
  result.gradient_first = -result.derivative_angle / sine * dc_df;
  result.gradient_second = -result.derivative_angle / sine * dc_ds;
  return result;
}

inline Eigen::Vector3d targetCenteredCartesian(const Eigen::Vector3d &target,
                                               const double radius,
                                               const double bearing,
                                               const double height)
{
  return target + Eigen::Vector3d(radius * std::cos(bearing),
                                  radius * std::sin(bearing), height);
}

inline bool cartesianToTargetCentered(const Eigen::Vector3d &position,
                                      const Eigen::Vector3d &target,
                                      double &radius, double &bearing,
                                      double &height)
{
  const Eigen::Vector3d relative = position - target;
  if (!position.allFinite() || !target.allFinite())
    return false;
  radius = relative.head<2>().norm();
  bearing = std::atan2(relative.y(), relative.x());
  height = relative.z();
  return std::isfinite(radius) && std::isfinite(bearing) &&
         std::isfinite(height) && radius > 1.0e-6;
}

inline double targetCenteredCheapObjective(
    const double horizon, const double k2_fraction,
    const double mean_visible, const double complementarity_score,
    const double encirclement_cost, const double movement_cost,
    const TeamObjectiveWeights &weights)
{
  TeamObjectiveTerms terms;
  terms.k2 = horizon * (1.0 - std::max(0.0, std::min(1.0, k2_fraction)));
  terms.accumulated = horizon *
      (3.0 - std::max(0.0, std::min(3.0, mean_visible)));
  terms.complementarity = horizon *
      (3.0 - std::max(0.0, std::min(3.0, complementarity_score)));
  terms.encirclement = std::max(0.0, encirclement_cost);
  terms.regularization = std::max(0.0, movement_cost);
  return terms.total(weights);
}

}  // namespace multi_uav_formation
