#pragma once
#include <multi_uav_formation/encirclement_geometry.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ego_planner {
struct LocalGeometryMetrics {
  bool valid{false};
  double confidence{1.0};
  double ratio{0.0}, violation{0.0}, maximum{0.0}, minimum{2*M_PI};
};
struct LocalGeometryPolicy {
  double minimum{25*M_PI/180}, maximum{170*M_PI/180};
  double peer_fresh_age{0.45};
  double horizon{1.5}, dt{0.10};
  void validate() const {
    multi_uav_formation::validateEncirclementLimits(minimum,maximum);
    for(double v:{peer_fresh_age,horizon,dt})
      if(!std::isfinite(v) || v<=0) throw std::invalid_argument("Invalid local geometry policy");
  }
};
enum class LocalGeometryState { NORMAL_ENCIRCLEMENT, GEOMETRY_DEGRADED };
inline const char *geometryStateName(LocalGeometryState s) {
  switch(s) {
    case LocalGeometryState::NORMAL_ENCIRCLEMENT:return "NORMAL_ENCIRCLEMENT";
    default:return "GEOMETRY_DEGRADED";
  }
}
inline LocalGeometryState classifyGeometry(const LocalGeometryMetrics &current,
    const LocalGeometryMetrics &next,const LocalGeometryPolicy &p) {
  (void)current;
  (void)p;
  if(!next.valid) return LocalGeometryState::GEOMETRY_DEGRADED;
  return next.ratio >= 0.8 ? LocalGeometryState::NORMAL_ENCIRCLEMENT
                           : LocalGeometryState::GEOMETRY_DEGRADED;
}
inline bool geometryBetter(const LocalGeometryMetrics &a,const LocalGeometryMetrics &b,
    const LocalGeometryMetrics &current,const LocalGeometryPolicy &p,
    double cost_a,double cost_b) {
  // K2/mean-visible are compared by LocalVisibilityPreference before this
  // function.  Geometry and native cost are quality ties only; this helper
  // must not introduce a second visibility authority.
  (void)current;
  (void)p;
  const double ga = a.valid && std::isfinite(a.violation)
      ? a.violation : std::numeric_limits<double>::infinity();
  const double gb = b.valid && std::isfinite(b.violation)
      ? b.violation : std::numeric_limits<double>::infinity();
  if(std::isfinite(ga) != std::isfinite(gb)) return std::isfinite(ga);
  if(std::isfinite(ga) && std::abs(ga-gb)>1e-8) return ga<gb;
  return cost_a<cost_b;
}

// Wall-time decayed maximum: failures count, fast calls cannot erase a tail.
struct ExecutionPlanningBudget {
  double initial{1.10}, minimum{0.20}, maximum{2.50}, factor{1.35}, half_life{60.0};
  double peak{0.0}, last{0.0};
  void validate() const {
    if(!std::isfinite(initial+minimum+maximum+factor+half_life) || minimum<=0 ||
       initial<minimum || maximum<initial || factor<1 || half_life<=0)
      throw std::invalid_argument("Invalid execution planning budget");
  }
  double estimate(double now) const {
    const double raw=peak>0?peak:initial/factor;
    return std::min(maximum,std::max(minimum,factor*raw*
        std::exp2(-std::max(0.0,last>0?now-last:0.0)/half_life)));
  }
  void observe(double elapsed,double now) {
    const double previous=estimate(now)/factor;
    peak=std::max(previous,std::max(0.0,elapsed));last=now;
  }
  bool canStart(double coverage,double now,double activation,double execution) const {
    return std::isfinite(coverage) && coverage>=estimate(now)+activation+execution;
  }
};
inline bool refinementFits(double deadline,double wall_now,double uninterruptible_budget) {
  return wall_now+uninterruptible_budget<deadline;
}

// One clock-domain contract for finite predecessor execution coverage.  The
// validated end is an authority supplied by the full trajectory validators;
// this helper never extends it.  A successor may start only after the real
// handoff/dispatch margin and strictly before that validated end.
struct ValidatedMovingCoverageWindow {
  double now{0.0};
  double validated_end{std::numeric_limits<double>::infinity()};
  double activation_margin{0.0};

  bool valid() const {
    return std::isfinite(now) && std::isfinite(activation_margin) &&
           activation_margin >= 0.0 &&
           (std::isfinite(validated_end) ||
            validated_end == std::numeric_limits<double>::infinity());
  }
  double activationEarliest() const { return now + activation_margin; }
  double planningDeadline() const { return validated_end - activation_margin; }
  double validatedCoverageRemaining() const { return validated_end - now; }
  double planningBudgetRemaining() const { return planningDeadline() - now; }
  bool activationFits(const double activation) const {
    return valid() && std::isfinite(activation) &&
           activation + 1.0e-9 >= activationEarliest() &&
           activation < validated_end - 1.0e-9;
  }
  bool canStillHandoff() const {
    return valid() && activationEarliest() < validated_end - 1.0e-9;
  }
};

// A finite predecessor bounds ordinary planning only while it can still cover
// activation.  Once that boundary is gone, the same Local Planner starts from
// current odometry with its normal measured pipeline budget; there is no
// second recovery-planner authority.
inline double movingPlanningBudget(
    const ValidatedMovingCoverageWindow &window,
    const double estimated_pipeline) {
  if (!std::isfinite(estimated_pipeline) || estimated_pipeline <= 0.0)
    return 0.0;
  if (!window.canStillHandoff())
    return estimated_pipeline;
  return std::max(0.0,
                  std::min(estimated_pipeline,
                           window.planningBudgetRemaining()));
}

// Optional quality work reserves the final uninterruptible interval.  Before
// any executable candidate exists, mandatory supply may use the remaining
// bounded interval down to the actual deadline; optimizer checkpoints still
// enforce that deadline.  This never extends predecessor coverage.
inline bool movingPlanningAttemptAllowed(const double deadline,
                                         const double wall_now,
                                         const double uninterruptible_budget,
                                         const bool executable_found) {
  return executable_found
      ? refinementFits(deadline, wall_now, uninterruptible_budget)
      : std::isfinite(deadline) && wall_now < deadline;
}

// A deadline-aware batch has two phases.  Before a safe candidate exists,
// alternatives are successor supply.  Afterwards the first safe candidate is
// retained as the coverage reserve while remaining alternatives are optional
// quality work.  In both phases the existing wall deadline is the authority:
// finding the first safe candidate is not, by itself, a reason to stop.
inline bool deadlineAwareAlternativeAllowed(const bool deadline_available,
                                            const bool reserve_limited,
                                            const bool executable_found) {
  (void)reserve_limited;
  (void)executable_found;
  return deadline_available;
}

enum class CoverageQualityAction {
  FIND_SAFE_SUPPLY,
  SEARCH_QUALITY_WITH_RESERVED_SAFE,
  COMMIT_RESERVED_SAFE
};

inline CoverageQualityAction coverageQualityAction(
    const bool safe_reserved, const bool deadline_available) {
  if (!safe_reserved)
    return CoverageQualityAction::FIND_SAFE_SUPPLY;
  return deadline_available
      ? CoverageQualityAction::SEARCH_QUALITY_WITH_RESERVED_SAFE
      : CoverageQualityAction::COMMIT_RESERVED_SAFE;
}
} // namespace ego_planner
