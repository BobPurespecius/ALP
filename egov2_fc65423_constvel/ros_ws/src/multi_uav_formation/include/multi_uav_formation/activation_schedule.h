#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace multi_uav_formation
{
// Wall-clock measurements, shared by seed and adaptive hypotheses. Keep a
// conservative cold-start budget until three complete evaluations are available;
// thereafter retain the worst of the last 32, with 25% execution jitter.
class PipelineLatencyEstimate
{
public:
  explicit PipelineLatencyEstimate(double cold_start = 0.35)
      : cold_start_(cold_start) {}
  void observe(double seconds, bool complete = true)
  {
    if (!std::isfinite(seconds) || seconds < 0.0) return;
    recent_.push_back(seconds);
    complete_.push_back(complete);
    if (recent_.size() > 32) { recent_.pop_front(); complete_.pop_front(); }
  }
  std::size_t completeObservations() const
  { return std::count(complete_.begin(), complete_.end(), true); }
  double budget() const
  {
    double peak = recent_.empty() ? 0.0 :
        *std::max_element(recent_.begin(), recent_.end());
    if (completeObservations() < 3) peak = std::max(peak, cold_start_);
    return std::max(0.020, 1.25 * peak);
  }
private:
  double cold_start_;
  std::deque<double> recent_;
  std::deque<bool> complete_;
};

struct ActivationSchedule
{
  bool valid{false};
  double activation{0.0};
  double lead{0.0};
  double required_margin{0.0};
};

inline bool activationReserveAvailable(double activation, double now,
                                       double required)
{
  return std::isfinite(activation) && std::isfinite(now) &&
      std::isfinite(required) && required >= 0.0 && activation - now >= required;
}

// Called AFTER candidate generation and selection, BEFORE source slicing or
// nonlinear evaluation. Completed stages are not charged a second time.
// Never shift an optimized trajectory's world-time origin after validation.
inline ActivationSchedule scheduleTeamActivation(
    double now, double minimum_lead, double prepare_budget,
    double joint_budget, double ack_timeout)
{
  ActivationSchedule result;
  if (!std::isfinite(now) || !std::isfinite(minimum_lead) ||
      !std::isfinite(prepare_budget) || !std::isfinite(joint_budget) ||
      !std::isfinite(ack_timeout) || minimum_lead < 0.0 ||
      prepare_budget < 0.0 || joint_budget < 0.0 || ack_timeout < 0.01)
    return result;
  // Existing ACK timeout + existing minimum commit lead, unchanged.
  result.required_margin = ack_timeout + 0.025;
  result.lead = std::max(minimum_lead, prepare_budget + joint_budget +
                        result.required_margin + 0.025);
  result.activation = now + result.lead;
  // Planner still rejects >1s future activation. Do not clamp an infeasible
  // latency estimate into that window and hope the pipeline finishes.
  result.valid = result.lead <= 0.95 && std::isfinite(result.activation);
  return result;
}
}  // namespace multi_uav_formation
