#ifndef PLAN_MANAGE_LOCAL_SFC_BOUNDARY_SCAN_H_
#define PLAN_MANAGE_LOCAL_SFC_BOUNDARY_SCAN_H_

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner
{

struct LocalSfcBoundaryScanResult
{
  bool valid_input{false};
  bool guide_occupied{false};
  bool transition_found{false};
  int sample_count{0};
  double lambda_free{std::numeric_limits<double>::quiet_NaN()};
  double lambda_occupied{std::numeric_limits<double>::quiet_NaN()};
  double lambda_boundary{std::numeric_limits<double>::quiet_NaN()};
};

// Scan a finite ray interval from lambda=0 and return only its first
// free-to-occupied transition. The final query is explicitly clamped to
// max_distance so floating-point stepping cannot omit the interval endpoint.
template <typename OccupancyQuery>
LocalSfcBoundaryScanResult scanFirstLocalSfcBoundary(
    const double max_distance, const double scan_step,
    OccupancyQuery occupancy_query)
{
  LocalSfcBoundaryScanResult result;
  if (!std::isfinite(max_distance) || !std::isfinite(scan_step) ||
      max_distance <= 0.0 || scan_step <= 0.0)
    return result;

  result.valid_input = true;
  double previous_lambda = 0.0;
  bool previous_occupied = occupancy_query(previous_lambda);
  result.sample_count = 1;
  result.guide_occupied = previous_occupied;
  if (previous_occupied)
    return result;

  const double endpoint_tolerance =
      1.0e-12 * std::max(1.0, max_distance);
  while (previous_lambda < max_distance - endpoint_tolerance)
  {
    const double current_lambda =
        std::min(max_distance, previous_lambda + scan_step);
    const bool current_occupied = occupancy_query(current_lambda);
    ++result.sample_count;
    if (!previous_occupied && current_occupied)
    {
      result.transition_found = true;
      result.lambda_free = previous_lambda;
      result.lambda_occupied = current_lambda;
      result.lambda_boundary = 0.5 * (previous_lambda + current_lambda);
      return result;
    }
    previous_lambda = current_lambda;
    previous_occupied = current_occupied;
  }

  return result;
}

}  // namespace ego_planner

#endif  // PLAN_MANAGE_LOCAL_SFC_BOUNDARY_SCAN_H_
