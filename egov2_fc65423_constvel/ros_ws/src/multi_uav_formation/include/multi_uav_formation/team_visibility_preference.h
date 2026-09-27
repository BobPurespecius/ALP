#pragma once

#include <algorithm>
#include <cmath>

namespace multi_uav_formation
{

// Shared policy used by both the trajectory-level Joint selector and the
// reference-level phi0 diagnostics.  Safety is deliberately outside this
// helper: callers first form an equally-safe executable set, then use this
// policy to protect near-best K=2 coverage while maximizing accumulated
// camera-visible time.  The remaining fields are telemetry only; geometry is
// compared by the caller after this helper reports a visibility tie.
struct TeamVisibilityPreference
{
  double atleast2{0.0};
  double mean_visible_count{0.0};
  double all3{0.0};
  double none{1.0};
  double spread{0.0};
};

inline bool inNearBestK2Set(const TeamVisibilityPreference &value,
                            const double best_k2,
                            const double tolerance)
{
  return value.atleast2 + 1.0e-9 >=
         best_k2 - std::max(0.0, tolerance);
}

inline bool betterTeamVisibilityWithinNearBestK2(
    const TeamVisibilityPreference &lhs,
    const TeamVisibilityPreference &rhs,
    const double best_k2,
    const double k2_near_best_tolerance)
{
  constexpr double eps = 1.0e-9;
  const bool lhs_admissible =
      inNearBestK2Set(lhs, best_k2, k2_near_best_tolerance);
  const bool rhs_admissible =
      inNearBestK2Set(rhs, best_k2, k2_near_best_tolerance);
  if (lhs_admissible != rhs_admissible)
    return lhs_admissible;
  if (std::abs(lhs.mean_visible_count - rhs.mean_visible_count) > eps)
    return lhs.mean_visible_count > rhs.mean_visible_count;
  if (std::abs(lhs.atleast2 - rhs.atleast2) > eps)
    return lhs.atleast2 > rhs.atleast2;
  return false;
}

}  // namespace multi_uav_formation
