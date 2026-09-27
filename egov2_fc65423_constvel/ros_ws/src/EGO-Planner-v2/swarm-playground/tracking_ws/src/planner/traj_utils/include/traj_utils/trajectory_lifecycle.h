#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace trajectory_lifecycle {
struct State {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
};
// The execution model is finite polynomial followed by terminal position hold.
// Never extrapolate an expired peer using its terminal velocity.
template<class Trajectory>
State sample(const Trajectory &trajectory, double elapsed) {
  State s;
  const double duration = trajectory.getTotalDuration();
  s.p = trajectory.getPos(std::max(0.0, std::min(elapsed, duration)));
  if (elapsed < duration) {
    s.v = trajectory.getVel(std::max(0.0, elapsed));
    s.a = trajectory.getAcc(std::max(0.0, elapsed));
  }
  return s;
}
struct Tolerances { double p = 0.02, v = 0.05, a = 0.10; };
struct Residual {
  double dp, dv, da;
  bool accepted(const Tolerances &t) const {
    return std::isfinite(dp) && std::isfinite(dv) && std::isfinite(da) &&
        dp <= t.p && dv <= t.v && da <= t.a;
  }
};
inline Residual compare(const State &old_state, const State &new_state) {
  return {(old_state.p-new_state.p).norm(), (old_state.v-new_state.v).norm(),
          (old_state.a-new_state.a).norm()};
}
inline bool currentGeneration(uint64_t result, uint64_t pending,
                              uint64_t captured_active, uint64_t active) {
  return result == pending && captured_active == active;
}
inline bool canStage(int executable_count) { return executable_count > 0; }
inline bool successorDue(double remaining, double planning_budget,
                         double activation_margin, double execution_margin) {
  return remaining <= planning_budget + activation_margin + execution_margin;
}
// Coordination never owns the local rolling planner's progress permission.
inline bool localReplanAllowed(bool /*team_pending*/) { return true; }

// When validated predecessor coverage is exhausted, the same Local Planner
// may restart from authoritative current state.  Executor admission remains a
// lifecycle check: an explicit current-state marker, a full safety certificate
// and a newer generation are all required.
inline bool currentStateRestartHandoff(bool explicitly_current_state,
                                       bool safety_validated,
                                       uint64_t old_generation,
                                       uint64_t new_generation,
                                       double new_activation) {
  return explicitly_current_state && safety_validated &&
      new_generation > old_generation && std::isfinite(new_activation);
}
} // namespace trajectory_lifecycle
