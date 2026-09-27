#include <multi_uav_formation/topology_coordinator_core.h>
#include <multi_uav_formation/team_visibility_preference.h>
#include <multi_uav_formation/encirclement_geometry.h>
#include <multi_uav_formation/tracking_visibility_geometry.h>

#include <algorithm>
#include <cmath>

namespace multi_uav_formation
{
namespace
{

constexpr double kEps = 1.0e-9;

double clamp01(const double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double angleDeg(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  const double denom = a.norm() * b.norm();
  if (denom <= 1.0e-9)
    return 0.0;
  const double cosine = std::max(-1.0, std::min(1.0, a.dot(b) / denom));
  return std::acos(cosine) * 180.0 / M_PI;
}

double cross2d(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
  return a.x() * b.y() - a.y() * b.x();
}

bool segmentIntersection(const Eigen::Vector3d &a0, const Eigen::Vector3d &a1,
                         const Eigen::Vector3d &b0, const Eigen::Vector3d &b1,
                         double &alpha, double &beta)
{
  const Eigen::Vector2d p = a0.head<2>();
  const Eigen::Vector2d r = (a1 - a0).head<2>();
  const Eigen::Vector2d q = b0.head<2>();
  const Eigen::Vector2d s = (b1 - b0).head<2>();
  const double denominator = cross2d(r, s);
  if (std::abs(denominator) <= 1.0e-9)
    return false;
  alpha = cross2d(q - p, s) / denominator;
  beta = cross2d(q - p, r) / denominator;
  return alpha >= 0.0 && alpha <= 1.0 && beta >= 0.0 && beta <= 1.0;
}

bool sameKinds(const std::array<int, 3> &lhs, const std::array<int, 3> &rhs)
{
  return lhs == rhs;
}

}  // namespace

TopologyCoordinatorCore::TopologyCoordinatorCore(
    const TopologyCoordinatorParams &params) : params_(params)
{
  validateEncirclementLimits(params.preferred_view_angle_deg*M_PI/180.0,params.theta_gap_max,params.encirclement_ratio_min);
  params_.sample_dt = std::max(0.02, params_.sample_dt);
  params_.swarm_clearance = std::max(0.0, params_.swarm_clearance);
  params_.near_margin = std::max(0.0, params_.near_margin);
  params_.max_bundle_age = std::max(0.01, params_.max_bundle_age);
  params_.max_epoch_skew = std::max(0.0, params_.max_epoch_skew);
  params_.max_start_skew = std::max(0.0, params_.max_start_skew);
  params_.preferred_view_angle_deg =
      std::max(1.0, std::min(180.0, params_.preferred_view_angle_deg));
  params_.encirclement_spread_saturation_angle_deg = std::max(
      1.0, std::min(180.0,
                    params_.encirclement_spread_saturation_angle_deg));
  params_.team_top_k = std::max(1, std::min(3, params_.team_top_k));
}

bool TopologyCoordinatorCore::executable(const TopologyCandidateCore &candidate)
{
  return candidate.success && candidate.static_valid && candidate.dynamics_valid &&
         candidate.swarm_valid && candidate.safety_class == 2 && candidate.duration > 1.0e-6 &&
         static_cast<bool>(candidate.position_at_global_time) &&
         static_cast<bool>(candidate.visible_at_global_time);
}

TopologyCombinationMetrics TopologyCoordinatorCore::evaluate(
    const std::array<const TopologyCandidateCore *, 3> &combo,
    const std::array<TopologyBundleCore, 3> &bundles,
    const double evaluation_start_time,
    const double evaluation_horizon,
    const bool evaluate_crossings) const
{
  TopologyCombinationMetrics metrics;
  const double start = evaluation_start_time;
  const double end = start + evaluation_horizon;
  metrics.evaluation_start_time = start;
  metrics.evaluation_horizon = evaluation_horizon;
  metrics.outer_loop_evaluation = bundles[0].outer_loop_evaluation;
  metrics.min_dynamic_clearance = std::numeric_limits<double>::infinity();
  metrics.minimum_safety_class = std::numeric_limits<int>::max();
  for (int drone = 0; drone < 3; ++drone)
  {
    const auto &candidate = *combo[drone];
    metrics.candidate_ids[drone] = candidate.candidate_id;
    metrics.kinds[drone] = candidate.kind;
    const double visible_horizon = candidate.evaluation_horizon > 1.0e-6
                                       ? candidate.evaluation_horizon
                                       : candidate.duration;
    const double candidate_end = candidate.start_time +
                                 std::min(candidate.duration, visible_horizon);
    if (candidate.start_time > start + 1.0e-6 ||
        candidate_end + 1.0e-6 < end)
      return metrics;
    metrics.min_dynamic_clearance =
        std::min(metrics.min_dynamic_clearance, candidate.dynamic_clearance);
    metrics.minimum_safety_class =
        std::min(metrics.minimum_safety_class, candidate.safety_class);
    metrics.tracking_score += candidate.tracking_score;
    metrics.native_cost += candidate.native_cost;
  }
  const int hypothesis = combo[0]->encirclement_hypothesis_id;
  if (combo[1]->encirclement_hypothesis_id != hypothesis ||
      combo[2]->encirclement_hypothesis_id != hypothesis ||
      combo[1]->encirclement_generation !=
          combo[0]->encirclement_generation ||
      combo[2]->encirclement_generation !=
          combo[0]->encirclement_generation)
    return metrics;
  metrics.encirclement_hypothesis_id = hypothesis;
  metrics.encirclement_phi0 = combo[0]->encirclement_phi0;
  metrics.encirclement_generation = combo[0]->encirclement_generation;
  metrics.tracking_score /= 3.0;
  if (!std::isfinite(start) || !std::isfinite(end) ||
      !std::isfinite(evaluation_horizon) ||
      end <= start + 1.0e-6)
    return metrics;

  const int sample_count =
      std::max(2, static_cast<int>(std::ceil((end - start) / params_.sample_dt)) + 1);
  const double dt = (end - start) / static_cast<double>(sample_count - 1);
  std::array<bool, 3> pair_conflict{{false, false, false}};
  double view_min_sum = 0.0;
  double view_mean_sum = 0.0;
  double diversity_sum = 0.0;
  double complementarity_integral = 0.0;
  std::vector<std::array<Eigen::Vector3d, 3>> sampled_positions;
  sampled_positions.reserve(sample_count);

  std::vector<double> maximum_gaps;
  double k2_run = 0.0, blackout_run = 0.0;
  std::array<double,3> individual_loss_run{{0.0,0.0,0.0}};
  for (int sample = 0; sample < sample_count; ++sample)
  {
    const double t = start + sample * dt;
    std::array<Eigen::Vector3d, 3> positions;
    std::array<bool, 3> visible{{false, false, false}};
    for (int drone = 0; drone < 3; ++drone)
    {
      positions[drone] = combo[drone]->position_at_global_time(t);
      if (!positions[drone].allFinite()) return metrics;
      visible[drone] = combo[drone]->visible_at_global_time
                           ? combo[drone]->visible_at_global_time(t)
                           : false;
    }
    sampled_positions.push_back(positions);

    int visible_count = 0;
    for (const bool value : visible)
      visible_count += value ? 1 : 0;
    if (sample + 1 < sample_count)
    {
      const BinaryCameraInterval camera(visible_count,dt);
      metrics.all3_visibility += camera.all3_time/evaluation_horizon;
      metrics.atleast2_visibility += camera.k2_time/evaluation_horizon;
      metrics.none_visibility += camera.none_time/evaluation_horizon;
      metrics.binary_camera_time += camera.camera_time;
      for(int drone=0;drone<3;++drone) {
        if(visible[drone]) {
          metrics.individual_visibility[drone]+=dt/evaluation_horizon;
          individual_loss_run[drone]=0.0;
        } else {
          individual_loss_run[drone]+=dt;
          metrics.longest_individual_loss[drone]=std::max(
              metrics.longest_individual_loss[drone],individual_loss_run[drone]);
        }
      }
      k2_run = visible_count < 2 ? k2_run+dt : 0.0;
      blackout_run = visible_count == 0 ? blackout_run+dt : 0.0;
      metrics.longest_k2_loss = std::max(metrics.longest_k2_loss,k2_run);
      metrics.longest_blackout = std::max(metrics.longest_blackout,blackout_run);
    }

    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    for (int drone = 0; drone < 3; ++drone)
      target += bundles[drone].target_position +
                bundles[drone].target_velocity * (t - bundles[drone].planning_epoch);
    target /= 3.0;
    std::array<Eigen::Vector3d,3> relative;
    for(int i=0;i<3;++i) relative[i]=positions[i]-target;
    if(sample==0)metrics.initial_relative=relative;
    metrics.final_relative=relative;
    metrics.geometry_cost+=encirclementGeometryCost(relative,params_.preferred_view_angle_deg*M_PI/180.0,params_.theta_gap_max)/sample_count;
    const auto geometry=circularGapGeometry(relative);
    maximum_gaps.push_back(geometry.maximum);
    if(sample+1<sample_count && geometry.eligible(
        params_.preferred_view_angle_deg*M_PI/180.0,params_.theta_gap_max))
      metrics.encirclement_ratio+=dt/evaluation_horizon;
    const double angle01 = angleDeg(positions[0] - target, positions[1] - target);
    const double angle02 = angleDeg(positions[0] - target, positions[2] - target);
    const double angle12 = angleDeg(positions[1] - target, positions[2] - target);
    const double min_angle = std::min(angle01, std::min(angle02, angle12));
    const double mean_angle = (angle01 + angle02 + angle12) / 3.0;
    view_min_sum += min_angle;
    view_mean_sum += mean_angle;
    const double spread_saturation = params_.preferred_view_angle_deg;
    diversity_sum += spreadPreferenceScore(min_angle, spread_saturation);
    if (sample + 1 < sample_count)
    {
      for (int first = 0; first < 3; ++first)
        for (int second = first + 1; second < 3; ++second)
        {
          const PairQualitySample quality = targetCenteredPairQuality(
              relative[first], relative[second]);
          if (quality.valid)
            complementarity_integral += dt *
                static_cast<double>(visible[first]) *
                static_cast<double>(visible[second]) * quality.value;
        }
    }

    int pair_index = 0;
    for (int first = 0; first < 3; ++first)
    {
      for (int second = first + 1; second < 3; ++second, ++pair_index)
      {
        const Eigen::Vector3d relative=positions[first]-positions[second];
        const double distance=std::sqrt(relative.head<2>().squaredNorm()+0.25*relative.z()*relative.z());
        metrics.team_min_separation = std::min(metrics.team_min_separation, distance);
        if (distance < params_.swarm_clearance)
        {
          pair_conflict[pair_index] = true;
          metrics.conflict_duration += dt;
          const double deficit = params_.swarm_clearance - distance;
          metrics.temporal_fix_burden += deficit * deficit * dt;
        }
        if (distance < params_.swarm_clearance + params_.near_margin)
          metrics.near_conflict_duration += dt;
        if (distance < 0.8 * params_.swarm_clearance)
          ++metrics.severe_conflict_samples;
      }
    }
  }

  metrics.gap_max_p50=gapQuantile(maximum_gaps,0.50);
  metrics.gap_max_p95=gapQuantile(maximum_gaps,0.95);
  metrics.gap_max_max=gapQuantile(maximum_gaps,1.0);
  metrics.conflict_pair_count =
      static_cast<int>(pair_conflict[0]) + static_cast<int>(pair_conflict[1]) +
      static_cast<int>(pair_conflict[2]);
  metrics.mean_visible_count = metrics.binary_camera_time/evaluation_horizon;
  metrics.weakest_camera_visibility=*std::min_element(
      metrics.individual_visibility.begin(),metrics.individual_visibility.end());
  metrics.maximum_camera_loss=*std::max_element(
      metrics.longest_individual_loss.begin(),metrics.longest_individual_loss.end());
  metrics.min_pairwise_view_angle_deg = view_min_sum / sample_count;
  metrics.mean_pairwise_view_angle_deg = view_mean_sum / sample_count;
  metrics.diversity_score = diversity_sum / sample_count;
  metrics.complementarity_score = evaluation_horizon > 1.0e-9
      ? complementarity_integral / evaluation_horizon : 0.0;

  if (evaluate_crossings)
  {
    // Crossing is a topology diagnostic, not the safety lattice.  Keep the
    // authoritative 50 ms distance/visibility sampling above, but use a
    // 100 ms polyline here to avoid an O(S^2) diagnostic dominating the
    // coordinator callback under three-planner CPU load.
    const int crossing_stride =
        std::max(1, static_cast<int>(std::ceil(0.10 / dt)));
    for (int first = 0; first < 3; ++first)
    {
      for (int second = first + 1; second < 3; ++second)
      {
        bool spatial_crossing = false;
        bool temporal_crossing = false;
        for (int ia = 0; ia + 1 < sample_count && !temporal_crossing;
             ia += crossing_stride)
        {
          const int ia_next = std::min(sample_count - 1, ia + crossing_stride);
          const double ta0 = start + ia * dt;
          const double ta1 = start + ia_next * dt;
          const Eigen::Vector3d &a0 = sampled_positions[ia][first];
          const Eigen::Vector3d &a1 = sampled_positions[ia_next][first];
          for (int ib = 0; ib + 1 < sample_count; ib += crossing_stride)
          {
            const int ib_next =
                std::min(sample_count - 1, ib + crossing_stride);
            const double tb0 = start + ib * dt;
            const double tb1 = start + ib_next * dt;
            const Eigen::Vector3d &b0 = sampled_positions[ib][second];
            const Eigen::Vector3d &b1 = sampled_positions[ib_next][second];
            double alpha = 0.0;
            double beta = 0.0;
            if (!segmentIntersection(a0, a1, b0, b1, alpha, beta))
              continue;
            spatial_crossing = true;
            const double za = a0.z() + alpha * (a1.z() - a0.z());
            const double zb = b0.z() + beta * (b1.z() - b0.z());
            if (std::abs(za - zb) > params_.swarm_clearance)
              continue;
            const double crossing_ta = ta0 + alpha * (ta1 - ta0);
            const double crossing_tb = tb0 + beta * (tb1 - tb0);
            if (std::abs(crossing_ta - crossing_tb) <=
                params_.crossing_time_tolerance)
            {
              temporal_crossing = true;
              break;
            }
          }
        }
        if (temporal_crossing)
          ++metrics.crossing_pair_count;
        else if (spatial_crossing)
          ++metrics.temporally_separated_crossing_pair_count;
      }
    }
  }

  // The cheap tuple pass and continuous refinement share the same normalized
  // lexicographic quality vector.  Binary candidate traces are merely the
  // low-resolution evaluator here; All3/blackout/weakest/diversity remain
  // telemetry and never enter the optimizer.
  const double cheap_regularization =
      0.001 * std::max(0.0, metrics.native_cost) +
      metrics.temporal_fix_burden;
  metrics.quality.q2_ratio = clamp01(metrics.atleast2_visibility);
  metrics.quality.accumulated_visibility_ratio =
      clamp01(metrics.mean_visible_count / 3.0);
  metrics.quality.complementarity =
      clamp01(metrics.complementarity_score / 3.0);
  metrics.quality.encirclement_cost = std::max(0.0, metrics.geometry_cost);
  metrics.quality.regularization_cost =
      std::max(0.0, cheap_regularization);
  // Retain the old scalar only as a diagnostic for existing messages.  It is
  // no longer an ordering authority.
  metrics.cheap_objective = targetCenteredCheapObjective(
      evaluation_horizon, metrics.atleast2_visibility,
      metrics.mean_visible_count, metrics.complementarity_score,
      metrics.geometry_cost,
      cheap_regularization,
      params_.objective_weights);
  metrics.scalar_utility = -metrics.cheap_objective;
  metrics.valid = std::isfinite(metrics.team_min_separation) &&
                  metrics.conflict_pair_count == 0 && metrics.minimum_safety_class == 2;
  return metrics;
}

bool TopologyCoordinatorCore::better(const TopologyCombinationMetrics &lhs,
                                     const TopologyCombinationMetrics &rhs) const
{
  if (safetyBetter(lhs, rhs))
    return true;
  if (!safetyEquivalent(lhs, rhs))
    return false;
  const TeamQualityTolerance tolerance = teamQualityToleranceFromSampling(
      std::max(1.0e-9, std::min(lhs.evaluation_horizon,
                                rhs.evaluation_horizon)),
      params_.sample_dt);
  if (betterTeamQuality(lhs.quality, rhs.quality, tolerance))
    return true;
  if (betterTeamQuality(rhs.quality, lhs.quality, tolerance))
    return false;
  if (std::abs(lhs.tracking_score - rhs.tracking_score) > kEps)
    return lhs.tracking_score > rhs.tracking_score;
  if (std::abs(lhs.native_cost - rhs.native_cost) > kEps)
    return lhs.native_cost < rhs.native_cost;
  return std::abs(lhs.encirclement_hypothesis_id) <
         std::abs(rhs.encirclement_hypothesis_id);
}

bool TopologyCoordinatorCore::safetyBetter(
    const TopologyCombinationMetrics &lhs,
    const TopologyCombinationMetrics &rhs) const
{
  if (!lhs.valid)
    return false;
  if (!rhs.valid)
    return true;
  if (lhs.minimum_safety_class != rhs.minimum_safety_class)
    return lhs.minimum_safety_class > rhs.minimum_safety_class;
  if (lhs.conflict_pair_count != rhs.conflict_pair_count)
    return lhs.conflict_pair_count < rhs.conflict_pair_count;
  if (lhs.severe_conflict_samples != rhs.severe_conflict_samples)
    return lhs.severe_conflict_samples < rhs.severe_conflict_samples;
  if (std::abs(lhs.conflict_duration - rhs.conflict_duration) > kEps)
    return lhs.conflict_duration < rhs.conflict_duration;
  // Once both combinations are conflict-free, extra separation is a quality
  // tie-break rather than a different safety class.  If every available
  // combination conflicts, retain the established maximize-separation rule.
  if (lhs.conflict_pair_count > 0 &&
      std::abs(lhs.team_min_separation - rhs.team_min_separation) > kEps)
    return lhs.team_min_separation > rhs.team_min_separation;
  return false;
}

bool TopologyCoordinatorCore::safetyEquivalent(
    const TopologyCombinationMetrics &lhs,
    const TopologyCombinationMetrics &rhs) const
{
  return !safetyBetter(lhs, rhs) && !safetyBetter(rhs, lhs);
}

TopologySelectionCore TopologyCoordinatorCore::select(
    const std::array<TopologyBundleCore, 3> &bundles, const double now,
    const TopologyHistory &history) const
{
  TopologySelectionCore selection;
  double min_epoch = std::numeric_limits<double>::infinity();
  double max_epoch = -std::numeric_limits<double>::infinity();
  double min_start = std::numeric_limits<double>::infinity();
  double max_start = -std::numeric_limits<double>::infinity();
  double evaluation_start = -std::numeric_limits<double>::infinity();
  double requested_horizon = std::numeric_limits<double>::infinity();
  double maximum_requested_horizon =
      -std::numeric_limits<double>::infinity();
  double current_available_end = std::numeric_limits<double>::infinity();
  const bool outer_loop_evaluation = bundles[0].outer_loop_evaluation;
  for (int drone = 0; drone < 3; ++drone)
  {
    const auto &bundle = bundles[drone];
    if (bundle.drone_id != drone || bundle.candidates.empty())
    {
      selection.reason = "MISSING_BUNDLE";
      return selection;
    }
    const double age = now - bundle.bundle_stamp;
    if (!std::isfinite(age) || age < -0.10 || age > params_.max_bundle_age)
    {
      selection.reason = "STALE_BUNDLE";
      return selection;
    }
    min_epoch = std::min(min_epoch, bundle.planning_epoch);
    max_epoch = std::max(max_epoch, bundle.planning_epoch);
    if (bundle.outer_loop_evaluation != outer_loop_evaluation)
    {
      selection.reason = "INCOMPATIBLE_OUTER_LOOP_SNAPSHOT";
      return selection;
    }
    const TopologyCandidateCore *local_candidate = nullptr;
    for (const auto &candidate : bundle.candidates)
    {
      if (!executable(candidate))
        continue;
      min_start = std::min(min_start, candidate.start_time);
      max_start = std::max(max_start, candidate.start_time);
      if (candidate.candidate_id == bundle.local_candidate_id &&
          candidate.encirclement_hypothesis_id == bundle.local_hypothesis_id)
        local_candidate = &candidate;
    }
    if (local_candidate == nullptr)
      for (const auto &candidate : bundle.candidates)
        if (executable(candidate)) { local_candidate = &candidate; break; }
    if (local_candidate == nullptr)
    {
      selection.reason = "NO_HARD_SAFE_CANDIDATE";
      return selection;
    }
    const double bundle_evaluation_start =
        std::isfinite(bundle.evaluation_start_time) &&
                bundle.evaluation_start_time > 0.0
            ? bundle.evaluation_start_time
            : local_candidate->start_time;
    const double bundle_requested_horizon =
        std::isfinite(bundle.requested_evaluation_horizon) &&
                bundle.requested_evaluation_horizon > 1.0e-6
            ? bundle.requested_evaluation_horizon
            : local_candidate->duration;
    const double local_visible_horizon =
        local_candidate->evaluation_horizon > 1.0e-6
            ? local_candidate->evaluation_horizon
            : local_candidate->duration;
    evaluation_start = std::max(evaluation_start, bundle_evaluation_start);
    requested_horizon = std::min(requested_horizon,
                                 bundle_requested_horizon);
    maximum_requested_horizon = std::max(maximum_requested_horizon,
                                         bundle_requested_horizon);
    current_available_end = std::min(
        current_available_end,
        local_candidate->start_time +
            std::min(local_candidate->duration, local_visible_horizon));
  }
  if (max_epoch - min_epoch > params_.max_epoch_skew)
  {
    selection.reason = "INCOMPATIBLE_PLANNING_EPOCH";
    return selection;
  }
  if (!std::isfinite(min_start) || max_start - min_start > params_.max_start_skew)
  {
    selection.reason = "INCOMPATIBLE_TRAJECTORY_START";
    return selection;
  }
  if (!std::isfinite(requested_horizon) ||
      !std::isfinite(maximum_requested_horizon) ||
      maximum_requested_horizon - requested_horizon > 1.0e-6)
  {
    selection.reason = "INCOMPATIBLE_EVALUATION_HORIZON";
    return selection;
  }
  // 本轮修复 3：H_eval 不再固定为配置值，而是收敛到三机真实的公共已验证前缀。
  // 之前用固定的 H_configured（1.5 s）与各机实际 validated coverage（实测
  // 0.70~0.79 s）比较，判据 current_available_end < evaluation_start + H 永远
  // 成立，于是 Joint 结构性不可用（team reference 可用率 0/1627），本应负责
  // 团队角度修正的通道长期失效。现在：
  //   H_common = current_available_end - evaluation_start   （三机取 min 之后）
  //   H_eval   = min(H_configured, H_common)
  // Joint 仍然只是后台 refinement：不延长 Local 轨迹、不降低安全验证、
  // 不阻塞 Local，也不恢复固定 1.5 s 门限。
  const double configured_horizon = requested_horizon;
  const double common_horizon =
      std::isfinite(current_available_end) && std::isfinite(evaluation_start)
          ? (current_available_end - evaluation_start)
          : -1.0;
  selection.configured_evaluation_horizon = configured_horizon;
  selection.common_validated_horizon = common_horizon;
  if (!std::isfinite(evaluation_start) || !std::isfinite(configured_horizon) ||
      configured_horizon <= 1.0e-6)
  {
    selection.reason = "NO_COMMON_EXECUTION_PREFIX";
    return selection;
  }
  // 公共前缀还必须覆盖协调器的最小可用窗口，否则退化为无意义的一小段。
  constexpr double kMinimumJointEvaluationHorizon = 0.20;
  if (!std::isfinite(common_horizon) ||
      common_horizon < kMinimumJointEvaluationHorizon)
  {
    selection.reason = "CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE";
    return selection;
  }
  const double evaluation_horizon =
      std::min(configured_horizon, common_horizon);
  selection.effective_evaluation_horizon = evaluation_horizon;
  selection.horizon_clamped_to_common_prefix =
      evaluation_horizon < configured_horizon - 1.0e-9;

  std::vector<TopologyCombinationMetrics> evaluated;
  for (const auto &first : bundles[0].candidates)
  {
    if (!executable(first))
      continue;
    for (const auto &second : bundles[1].candidates)
    {
      if (!executable(second))
        continue;
      for (const auto &third : bundles[2].candidates)
      {
        if (!executable(third))
          continue;
        if (first.encirclement_hypothesis_id !=
                second.encirclement_hypothesis_id ||
            first.encirclement_hypothesis_id !=
                third.encirclement_hypothesis_id ||
            first.encirclement_generation !=
                second.encirclement_generation ||
            first.encirclement_generation !=
                third.encirclement_generation)
        {
          ++selection.cross_hypothesis_combination_count;
          continue;
        }
        const std::array<const TopologyCandidateCore *, 3> combo{{&first, &second, &third}};
        TopologyCombinationMetrics metrics = evaluate(
            combo, bundles, evaluation_start, evaluation_horizon, false);
        if (metrics.valid)
          evaluated.push_back(metrics);
      }
    }
  }
  selection.combination_count = static_cast<int>(evaluated.size());
  if (evaluated.empty())
  {
    selection.reason = "NO_EXECUTABLE_COMBINATION";
    return selection;
  }

  // Safety alone determines eligibility. Incomplete recovery is a preference,
  // never a reason to discard every safe team combination.
  evaluated.erase(std::remove_if(evaluated.begin(),evaluated.end(),[&](const TopologyCombinationMetrics &m) {
    return !m.valid || m.conflict_pair_count>0;
  }),evaluated.end());
  if(evaluated.empty())
  {
    selection.reason="NO_HARD_SAFE_COMBINATION";
    return selection;
  }

  // Coverage protects the current hard-safe local combination. If it is
  // unavailable, use the best-K2 hard-safe candidate as a recovery reference.
  const TopologyCombinationMetrics *reference = nullptr;
  const TopologyCombinationMetrics *best_visibility_reference = &evaluated.front();
  for (const auto &metrics : evaluated)
  {
    if (better(metrics, *best_visibility_reference))
      best_visibility_reference = &metrics;
    if (metrics.encirclement_hypothesis_id == bundles[0].local_hypothesis_id &&
        metrics.candidate_ids[0] == bundles[0].local_candidate_id &&
        metrics.candidate_ids[1] == bundles[1].local_candidate_id &&
        metrics.candidate_ids[2] == bundles[2].local_candidate_id) reference=&metrics;
    selection.best_k2=std::max(selection.best_k2,metrics.atleast2_visibility);
  }
  if (!reference) reference=best_visibility_reference;
  selection.local=*reference;
  selection.joint=*reference;
  for (const auto &metrics : evaluated)
  {
    // K2/blackout are ranking terms; never exclude a hard-safe recovery.
    if (better(metrics,selection.joint)) selection.joint=metrics;
  }
  std::stable_sort(evaluated.begin(), evaluated.end(),
      [this](const TopologyCombinationMetrics &lhs,
             const TopologyCombinationMetrics &rhs) {
        return better(lhs, rhs);
      });
  const size_t retained = std::min(
      evaluated.size(), static_cast<size_t>(params_.team_top_k));
  selection.top_k.assign(evaluated.begin(), evaluated.begin() + retained);
  selection.coordination_available = true;
  selection.selected_lost_k2_from_best = std::max(
      0.0, selection.best_k2 - selection.joint.atleast2_visibility);
  if (selection.reason.empty())
    selection.reason = selection.joint.conflict_pair_count == 0
                           ? "HARD_SAFE_VISIBILITY_FIRST_GEOMETRY_TIE"
                           : "ALL_COMBINATIONS_CONFLICT_MAXIMIZE_SEPARATION";
  const auto add_crossing_diagnostics = [&](TopologyCombinationMetrics &metrics) {
    std::array<const TopologyCandidateCore *, 3> combo{{nullptr, nullptr, nullptr}};
    for (int drone = 0; drone < 3; ++drone)
    {
      for (const auto &candidate : bundles[drone].candidates)
      {
        if (candidate.candidate_id == metrics.candidate_ids[drone])
        {
          combo[drone] = &candidate;
          break;
        }
      }
      if (combo[drone] == nullptr)
        return;
    }
    metrics = evaluate(combo, bundles, evaluation_start,
                       evaluation_horizon, true);
  };
  add_crossing_diagnostics(selection.joint);
  if (selection.local.candidate_ids == selection.joint.candidate_ids)
    selection.local = selection.joint;
  else
    add_crossing_diagnostics(selection.local);
  selection.differed_from_local =
      selection.joint.candidate_ids != selection.local.candidate_ids;
  selection.improved_min_separation =
      selection.joint.team_min_separation > selection.local.team_min_separation + kEps;
  selection.reduced_conflict =
      selection.joint.conflict_pair_count < selection.local.conflict_pair_count ||
      selection.joint.conflict_duration < selection.local.conflict_duration - kEps;
  selection.improved_k2 =
      selection.joint.atleast2_visibility > selection.local.atleast2_visibility + kEps;
  selection.improved_diversity =
      selection.joint.diversity_score > selection.local.diversity_score + kEps;
  selection.worsened_safety =
      selection.joint.conflict_pair_count > selection.local.conflict_pair_count ||
      selection.joint.severe_conflict_samples >
          selection.local.severe_conflict_samples ||
      selection.joint.team_min_separation + 0.05 <
          selection.local.team_min_separation;
  selection.topology_switched =
      history.valid &&
      (selection.joint.encirclement_hypothesis_id !=
           history.encirclement_hypothesis_id ||
       !sameKinds(selection.joint.kinds, history.kinds));
  selection.rapid_reversal =
      selection.topology_switched && history.previous_valid &&
      selection.joint.encirclement_hypothesis_id ==
          history.previous_encirclement_hypothesis_id &&
      sameKinds(selection.joint.kinds, history.previous_kinds) &&
      now - history.last_switch_time <= params_.rapid_reversal_window;
  return selection;
}

}  // namespace multi_uav_formation
