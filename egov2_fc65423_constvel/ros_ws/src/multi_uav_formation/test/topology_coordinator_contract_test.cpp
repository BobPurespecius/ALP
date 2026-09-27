#include <multi_uav_formation/topology_coordinator_core.h>
#include <multi_uav_formation/team_visibility_preference.h>

#include <cmath>
#include <iostream>

using multi_uav_formation::TopologyBundleCore;
using multi_uav_formation::TopologyCandidateCore;
using multi_uav_formation::TopologyCoordinatorCore;
using multi_uav_formation::TopologyCoordinatorParams;
using multi_uav_formation::TopologyHistory;

namespace
{

int failures = 0;

// These historical fixtures deliberately isolate safety/visibility/hysteresis
// using arbitrary straight lines. Full default geometry is exercised by
// encirclement_geometry_contract_test with both eligible and ineligible teams.
TopologyCoordinatorParams visibilityOnlyFixtureParams()
{
  TopologyCoordinatorParams p;
  p.encirclement_ratio_min=0.0;
  return p;
}
std::array<TopologyBundleCore, 3> baseBundles();

void expect(const bool condition, const std::string &name)
{
  if (!condition)
  {
    std::cerr << "FAIL " << name << '\n';
    ++failures;
  }
}

TopologyCandidateCore candidate(const int drone, const int id, const int kind,
                                const Eigen::Vector3d &origin,
                                const Eigen::Vector3d &velocity,
                                const bool visible = true,
                                const double dynamic_clearance = 2.0,
                                const double tracking_score = 1.0,
                                const int hypothesis_id = 0,
                                const std::uint64_t encirclement_generation = 0)
{
  TopologyCandidateCore value;
  value.drone_id = drone;
  value.candidate_id = id;
  value.encirclement_hypothesis_id = hypothesis_id;
  value.encirclement_phi0 = hypothesis_id * M_PI / 12.0;
  value.encirclement_generation = encirclement_generation;
  value.kind = kind;
  value.safety_class = 2;
  value.success = true;
  value.static_valid = true;
  value.dynamics_valid = true;
  value.swarm_valid = true;
  value.start_time = 10.0;
  value.duration = 2.0;
  value.evaluation_horizon = 2.0;
  value.dynamic_clearance = dynamic_clearance;
  value.tracking_score = tracking_score;
  value.native_cost = 1.0;
  value.position_at_global_time = [origin, velocity](const double global_time) {
    return origin + velocity * (global_time - 10.0);
  };
  value.visible_at_global_time = [visible](const double) { return visible; };
  return value;
}

void testHypothesisIsolationAndTrajectoryVisibility()
{
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].local_hypothesis_id = 0;
    for (int hypothesis = -1; hypothesis <= 1; ++hypothesis)
    {
      const bool actual_candidate_visible = hypothesis == 1;
      const int id = 100 * drone + hypothesis + 2;
      bundles[drone].candidates.push_back(candidate(
          drone, id, 0,
          Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
          Eigen::Vector3d::Zero(), actual_candidate_visible, 2.0, 1.0,
          hypothesis, 77));
      if (hypothesis == 0)
        bundles[drone].local_candidate_id = id;
    }
  }
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.coordination_available, "hypothesis_coordination_available");
  expect(result.combination_count == 3, "same_hypothesis_only_three");
  expect(result.cross_hypothesis_combination_count == 24,
         "cross_hypothesis_24_rejected");
  expect(result.joint.encirclement_hypothesis_id == 1,
         "trajectory_visibility_selects_positive_hypothesis");
  expect(result.joint.atleast2_visibility > 0.99,
         "trajectory_visibility_not_reference_proxy");
}

std::array<TopologyBundleCore, 3> baseBundles()
{
  std::array<TopologyBundleCore, 3> bundles;
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].drone_id = drone;
    bundles[drone].planning_generation = 100 + drone;
    bundles[drone].planning_epoch = 10.0 + 0.01 * drone;
    bundles[drone].bundle_stamp = 10.05 + 0.01 * drone;
    bundles[drone].evaluation_start_time = 10.0;
    bundles[drone].requested_evaluation_horizon = 1.5;
    bundles[drone].target_position = Eigen::Vector3d(0.0, 0.0, 0.0);
    bundles[drone].target_velocity = Eigen::Vector3d::Zero();
  }
  return bundles;
}

void testFixedFormationHorizon()
{
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].requested_evaluation_horizon = 1.5;
    TopologyCandidateCore prefix_good = candidate(
        drone, 100 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 0, 91);
    prefix_good.visible_at_global_time = [](const double time) {
      return time <= 11.5 + 1.0e-9;
    };
    TopologyCandidateCore whole_good = candidate(
        drone, 200 + drone, 0,
        Eigen::Vector3d(2.5, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 1, 91);
    whole_good.visible_at_global_time = [](const double time) {
      return time > 11.5 + 1.0e-9;
    };
    bundles[drone].candidates = {prefix_good, whole_good};
    bundles[drone].local_candidate_id = prefix_good.candidate_id;
    bundles[drone].local_hypothesis_id = 0;
  }
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.coordination_available, "prefix_coordination_available");
  expect(std::abs(result.joint.evaluation_horizon - 1.5) < 1.0e-9,
         "fixed_1p5_formation_horizon_used");
  expect(result.joint.encirclement_hypothesis_id == 0,
         "prefix_visibility_beats_better_whole_trajectory");
}

void testShortAlternativeCannotShrinkCommonPrefix()
{
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].requested_evaluation_horizon = 1.5;
    TopologyCandidateCore current = candidate(
        drone, 300 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), false, 2.0, 1.0, 0, 92);
    TopologyCandidateCore short_alternative = candidate(
        drone, 400 + drone, 0,
        Eigen::Vector3d(2.5, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 1, 92);
    short_alternative.duration = 0.3;
    short_alternative.evaluation_horizon = 0.3;
    bundles[drone].candidates = {current, short_alternative};
    bundles[drone].local_candidate_id = current.candidate_id;
    bundles[drone].local_hypothesis_id = 0;
  }
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.coordination_available, "short_alternative_available");
  expect(std::abs(result.joint.evaluation_horizon - 1.5) < 1.0e-9,
         "short_alternative_did_not_shrink_fixed_horizon");
  expect(result.joint.encirclement_hypothesis_id == 0,
         "short_alternative_excluded");
}

void testPhi0SampleAndHold()
{
  TopologyCoordinatorParams params=visibilityOnlyFixtureParams();
  TopologyCoordinatorCore core(params);
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].outer_loop_evaluation = true;
    TopologyCandidateCore current = candidate(
        drone, 500 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 0, 93);
    TopologyCandidateCore weak_alternative = candidate(
        drone, 600 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 1, 93);
    weak_alternative.native_cost = 0.0;
    bundles[drone].candidates = {current, weak_alternative};
    bundles[drone].local_candidate_id = current.candidate_id;
    bundles[drone].local_hypothesis_id = 0;
  }
  const auto held = core.select(bundles, 10.1);
  expect(held.coordination_available, "phi0_hold_coordination_available");
  expect(held.joint.encirclement_hypothesis_id == 1,
         "camera_time_tie_uses_native_cost_without_phi0_restore");
  expect(held.reason == "HARD_SAFE_VISIBILITY_FIRST_GEOMETRY_TIE",
         "adaptive_camera_time_policy_reason");

  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].candidates[0].visible_at_global_time =
        [](const double) { return false; };
  }
  const auto improved = core.select(bundles, 10.1);
  expect(improved.joint.encirclement_hypothesis_id == 1,
         "phi0_switches_for_meaningful_fov_gain");
}

void testEnumeration()
{
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    for (int kind = 0; kind < 3; ++kind)
      bundles[drone].candidates.push_back(candidate(
          drone, 10 * drone + kind, kind,
          Eigen::Vector3d(2.0 + drone, 3.0 * drone + 0.1 * kind, 1.0),
          Eigen::Vector3d::Zero()));
    bundles[drone].local_candidate_id = 10 * drone;
  }
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.coordination_available, "enumeration_available");
  expect(result.combination_count == 27, "enumeration_27");
}

void testWorldSpaceNotLabels()
{
  auto same_plus = baseBundles();
  same_plus[0].candidates.push_back(candidate(
      0, 1, 1, Eigen::Vector3d(0.0, -2.0, 1.0), Eigen::Vector3d(1.0, 0.0, 0.0)));
  same_plus[1].candidates.push_back(candidate(
      1, 2, 1, Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 0.0)));
  same_plus[2].candidates.push_back(candidate(
      2, 3, 0, Eigen::Vector3d(0.0, 2.0, 1.0), Eigen::Vector3d(1.0, 0.0, 0.0)));
  for (int drone = 0; drone < 3; ++drone)
    same_plus[drone].local_candidate_id = drone + 1;
  const auto safe = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(same_plus, 10.1);
  expect(safe.joint.conflict_pair_count == 0, "same_plus_world_safe");

  auto different_labels = baseBundles();
  different_labels[0].candidates.push_back(candidate(
      0, 1, 1, Eigen::Vector3d(-1.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 0.0)));
  different_labels[1].candidates.push_back(candidate(
      1, 2, 2, Eigen::Vector3d(1.0, 0.0, 1.0), Eigen::Vector3d(-1.0, 0.0, 0.0)));
  different_labels[2].candidates.push_back(candidate(
      2, 3, 0, Eigen::Vector3d(0.0, 3.0, 1.0), Eigen::Vector3d::Zero()));
  for (int drone = 0; drone < 3; ++drone)
    different_labels[drone].local_candidate_id = drone + 1;
  const auto unsafe = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(different_labels, 10.1);
  expect(!unsafe.coordination_available,
         "different_labels_world_conflict_rejected");
}

void testSafetyBeforeVisibility()
{
  auto bundles = baseBundles();
  bundles[0].candidates.push_back(candidate(
      0, 10, 0, Eigen::Vector3d(-1.0, 0.0, 1.0),
      Eigen::Vector3d(1.0, 0.0, 0.0), true));
  bundles[0].candidates.push_back(candidate(
      0, 11, 1, Eigen::Vector3d(-1.0, -1.5, 1.0),
      Eigen::Vector3d(1.0, 0.0, 0.0), false));
  bundles[1].candidates.push_back(candidate(
      1, 20, 0, Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0), true));
  bundles[2].candidates.push_back(candidate(
      2, 30, 0, Eigen::Vector3d(0.0, 2.0, 1.0),
      Eigen::Vector3d::Zero(), true));
  bundles[0].local_candidate_id = 10;
  bundles[1].local_candidate_id = 20;
  bundles[2].local_candidate_id = 30;
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.joint.candidate_ids[0] == 11, "safety_beats_all3");
  expect(result.joint.conflict_pair_count == 0, "safe_combo_no_conflict");
  expect(result.joint.atleast2_visibility > 0.99, "safe_combo_keeps_k2");
}

void testKOfNAndDiversity()
{
  auto bundles = baseBundles();
  bundles[0].candidates.push_back(candidate(
      0, 10, 0, Eigen::Vector3d(2.0, 0.05, 1.0), Eigen::Vector3d::Zero(), true,
      1.0));
  bundles[0].candidates.push_back(candidate(
      0, 11, 1, Eigen::Vector3d(-2.0, 0.0, 1.0), Eigen::Vector3d::Zero(), false,
      2.0));
  bundles[1].candidates.push_back(candidate(
      1, 20, 0, Eigen::Vector3d(2.0, 0.0, 1.0), Eigen::Vector3d::Zero(), true,
      2.0));
  bundles[2].candidates.push_back(candidate(
      2, 30, 0, Eigen::Vector3d(0.0, 2.0, 1.0), Eigen::Vector3d::Zero(), true,
      2.0));
  bundles[0].local_candidate_id = 10;
  bundles[1].local_candidate_id = 20;
  bundles[2].local_candidate_id = 30;
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.joint.candidate_ids[0] == 11, "k2_allows_lower_all3");
  expect(result.joint.atleast2_visibility > 0.99, "k2_preserved");
  expect(result.joint.all3_visibility == 0.0,
         "all3_not_hard_requirement");
  expect(result.joint.diversity_score > 0.99,
         "safe_recovery_diversity_above_lower_bound");
}

void testStaleFallback()
{
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].candidates.push_back(candidate(
        drone, drone, 0, Eigen::Vector3d(0.0, 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero()));
    bundles[drone].local_candidate_id = drone;
  }
  bundles[1].bundle_stamp = 9.0;
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(!result.coordination_available, "stale_nonblocking_fallback");
  expect(result.reason == "STALE_BUNDLE", "stale_reason");
}

void testSingleExecutableCombination()
{
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].candidates.push_back(candidate(
        drone, 1400 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 0, 97));
    bundles[drone].local_candidate_id = 1400 + drone;
    bundles[drone].local_hypothesis_id = 0;
  }
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.coordination_available, "single_combination_available");
  expect(result.combination_count == 1, "single_combination_count_one");
  expect(result.joint.candidate_ids ==
             std::array<int, 3>{{1400, 1401, 1402}},
         "single_combination_selected");
}

void testHysteresis()
{
  TopologyCoordinatorParams params=visibilityOnlyFixtureParams();
  TopologyCoordinatorCore core(params);
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].candidates.push_back(candidate(
        drone, 10 * drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0), Eigen::Vector3d::Zero(),
        true, 2.0, 1.0));
    bundles[drone].candidates.push_back(candidate(
        drone, 10 * drone + 1, 1,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0), Eigen::Vector3d::Zero(),
        true, 2.0, 1.02));
    bundles[drone].local_candidate_id = 10 * drone + 1;
  }
  TopologyHistory history;
  history.valid = true;
  history.kinds = {{0, 0, 0}};
  const auto weak = core.select(bundles, 10.1, history);
  expect(weak.joint.kinds != history.kinds,
         "single_comparator_has_no_second_hysteresis_gate");

  bundles[0].candidates[0] = candidate(
      0, 0, 0, Eigen::Vector3d(-1.0, 0.0, 1.0),
      Eigen::Vector3d(1.0, 0.0, 0.0));
  bundles[1].candidates[0] = candidate(
      1, 10, 0, Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0));
  const auto safety = core.select(bundles, 10.1, history);
  expect(safety.joint.kinds != history.kinds,
         "hysteresis_allows_immediate_safety_switch");
}

void testUnavailable()
{
  auto bundles = baseBundles();
  bundles[0].candidates.push_back(candidate(
      0, 1, 0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(!result.coordination_available, "coordinator_unavailable_fallback");
}

void testVisibilityRanksBeforeSaturatedSpread()
{
  TopologyCoordinatorParams params=visibilityOnlyFixtureParams();
  params.encirclement_spread_saturation_angle_deg = 60.0;
  TopologyCoordinatorCore core(params);
  auto bundles = baseBundles();
  const std::array<double, 3> exact_angles{{0.0, 120.0, 240.0}};
  const std::array<double, 3> visible_angles{{0.0, 75.0, 210.0}};
  const double visible_radius =
      std::sqrt(3.0) / std::sin(37.5 * M_PI / 180.0);
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].outer_loop_evaluation = true;
    const double exact = exact_angles[drone] * M_PI / 180.0;
    const double visible = visible_angles[drone] * M_PI / 180.0;
    TopologyCandidateCore exact_120 = candidate(
        drone, 800 + drone, 0,
        2.0 * Eigen::Vector3d(std::cos(exact), std::sin(exact), 0.0),
        Eigen::Vector3d::Zero(), false, 2.0, 1.0, 0, 94);
    TopologyCandidateCore visible_75 = candidate(
        drone, 900 + drone, 0,
        visible_radius *
            Eigen::Vector3d(std::cos(visible), std::sin(visible), 0.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 1, 94);
    bundles[drone].candidates = {exact_120, visible_75};
    bundles[drone].local_candidate_id = exact_120.candidate_id;
    bundles[drone].local_hypothesis_id = 0;
  }
  const auto result = core.select(bundles, 10.1);
  expect(result.coordination_available,
         "visibility_before_spread_coordination_available");
  expect(result.joint.encirclement_hypothesis_id == 1,
         "visible_75_beats_invisible_exact_120");
  expect(std::abs(result.joint.diversity_score - 1.0) < 1.0e-12,
         "seventy_five_degree_spread_saturated");
}

void testNearBestK2VisibilityPolicy()
{
  using multi_uav_formation::TeamVisibilityPreference;
  using multi_uav_formation::betterTeamVisibilityWithinNearBestK2;
  const double tolerance = 0.01;
  const TeamVisibilityPreference a{1.00, 2.60, 0.60, 0.00, 0.80};
  const TeamVisibilityPreference near_b{0.995, 2.90, 0.80, 0.005, 0.70};
  expect(betterTeamVisibilityWithinNearBestK2(
             near_b, a, 1.0, tolerance),
         "near_best_k2_mean_visible_changes_winner");

  const TeamVisibilityPreference outside_b{0.97, 2.95, 0.90, 0.00, 1.00};
  expect(!betterTeamVisibilityWithinNearBestK2(
             outside_b, a, 1.0, tolerance),
         "k2_tolerance_protects_coverage");

  const TeamVisibilityPreference same_k2_high_mean{
      1.00, 2.80, 0.50, 0.00, 0.60};
  expect(betterTeamVisibilityWithinNearBestK2(
             same_k2_high_mean, a, 1.0, tolerance),
         "same_k2_higher_mean_visible_wins");

  TeamVisibilityPreference high_all3 = a;
  high_all3.all3 = 0.70;
  expect(!betterTeamVisibilityWithinNearBestK2(
             high_all3, a, 1.0, tolerance) &&
         !betterTeamVisibilityWithinNearBestK2(
             a, high_all3, 1.0, tolerance),
         "same_mean_all3_is_telemetry_only");

  TeamVisibilityPreference prettier_spread = a;
  prettier_spread.mean_visible_count = 2.50;
  prettier_spread.spread = 1.0;
  expect(!betterTeamVisibilityWithinNearBestK2(
             prettier_spread, a, 1.0, tolerance),
         "spread_cannot_beat_mean_visible");
}

void testSafetyClassBeforeMeanVisible()
{
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    TopologyCandidateCore safe = candidate(
        drone, 1000 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), false, 2.0, 1.0, 0, 95);
    TopologyCandidateCore improved_only = candidate(
        drone, 1100 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 1, 95);
    improved_only.safety_class = 1;
    bundles[drone].candidates = {safe, improved_only};
    bundles[drone].local_candidate_id = improved_only.candidate_id;
    bundles[drone].local_hypothesis_id = 1;
  }
  const auto result = TopologyCoordinatorCore(visibilityOnlyFixtureParams()).select(bundles, 10.1);
  expect(result.joint.encirclement_hypothesis_id == 0,
         "absolute_safe_beats_higher_visibility_improved_only");
}

void testOuterNearBestK2MeanVisibleSwitch()
{
  TopologyCoordinatorParams params=visibilityOnlyFixtureParams();
  params.sample_dt = 0.02;
  TopologyCoordinatorCore core(params);
  auto bundles = baseBundles();
  for (int drone = 0; drone < 3; ++drone)
  {
    bundles[drone].outer_loop_evaluation = true;
    bundles[drone].requested_evaluation_horizon = 2.0;
    TopologyCandidateCore current = candidate(
        drone, 1200 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 0, 96);
    current.visible_at_global_time = [drone](const double time) {
      const int sample = static_cast<int>(std::lround((time - 10.0) / 0.02));
      return drone < 2 || sample < 50;
    };
    TopologyCandidateCore alternative = candidate(
        drone, 1300 + drone, 0,
        Eigen::Vector3d(2.0, -2.0 + 2.0 * drone, 1.0),
        Eigen::Vector3d::Zero(), true, 2.0, 1.0, 1, 96);
    alternative.visible_at_global_time = [drone](const double time) {
      const int sample = static_cast<int>(std::lround((time - 10.0) / 0.02));
      return sample < 100 || drone == 2;
    };
    bundles[drone].candidates = {current, alternative};
    bundles[drone].local_candidate_id = current.candidate_id;
    bundles[drone].local_hypothesis_id = 0;
  }
  const auto result = core.select(bundles, 10.1);
  expect(result.joint.encirclement_hypothesis_id == 1,
         "outer_near_best_k2_mean_visible_gain_switches");
  expect(result.selected_lost_k2_from_best <= 0.01 + 1.0e-9,
         "selected_k2_loss_within_tolerance");
}

}  // namespace

int main()
{
  testEnumeration();
  testWorldSpaceNotLabels();
  testSafetyBeforeVisibility();
  testKOfNAndDiversity();
  testStaleFallback();
  testSingleExecutableCombination();
  testHysteresis();
  testUnavailable();
  testHypothesisIsolationAndTrajectoryVisibility();
  testFixedFormationHorizon();
  testShortAlternativeCannotShrinkCommonPrefix();
  testPhi0SampleAndHold();
  testVisibilityRanksBeforeSaturatedSpread();
  testNearBestK2VisibilityPolicy();
  testSafetyClassBeforeMeanVisible();
  testOuterNearBestK2MeanVisibleSwitch();
  if (failures == 0)
  {
    std::cout << "JOINT_TOPOLOGY_COORDINATOR_CONTRACT=PASS tests=16 "
                 "cross_hypothesis=0 trajectory_visibility=1 "
                 "fixed_horizon_1p5=1 sample_and_hold=1 "
                 "visibility_before_saturated_spread=1 "
                 "near_best_k2_mean_visible=1 safety_class_first=1 "
                 "outer_joint_policy_consistent=1 single_combination=1\n";
    return 0;
  }
  std::cerr << "JOINT_TOPOLOGY_COORDINATOR_CONTRACT=FAIL failures=" << failures
            << '\n';
  return 1;
}
