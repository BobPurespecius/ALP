#pragma once

#include <cmath>
#include <multi_uav_formation/team_visibility_preference.h>

namespace ego_planner
{

// The one local quality order applied after current-revision hard admission.
// K2 and accumulated camera visibility are decision authority.  The remaining
// fields stay in the report for telemetry only; they must not create another
// admission/ranking layer.
struct LocalVisibilityPreference
{
  double none{1.0};
  double atleast2{0.0};
  double atleast_k{0.0};
  double mean_visible_count{0.0};
  double all3{0.0};
  double min_uav_visibility{0.0};
  double max_loss_duration{0.0};
  double diversity{0.0};
  double min_pairwise_angle_deg{0.0};
  double team_utility{0.0};
};

inline bool betterLocalVisibilityCandidate(
    const LocalVisibilityPreference &lhs,
    const LocalVisibilityPreference &rhs)
{
  const multi_uav_formation::TeamVisibilityPreference lhs_shared{
      lhs.atleast2, lhs.mean_visible_count, lhs.all3, lhs.none, lhs.diversity};
  const multi_uav_formation::TeamVisibilityPreference rhs_shared{
      rhs.atleast2, rhs.mean_visible_count, rhs.all3, rhs.none, rhs.diversity};
  return multi_uav_formation::betterTeamVisibilityWithinNearBestK2(
      lhs_shared, rhs_shared, std::max(lhs.atleast2, rhs.atleast2), 0.0);
}

}  // namespace ego_planner
