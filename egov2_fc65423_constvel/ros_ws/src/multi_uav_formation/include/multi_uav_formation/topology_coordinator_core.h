#pragma once

#include <Eigen/Core>
#include <multi_uav_formation/encirclement_geometry.h>
#include <multi_uav_formation/team_objective_primitives.h>

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace multi_uav_formation
{

struct TopologyCandidateCore
{
  int drone_id{-1};
  int candidate_id{-1};
  int encirclement_hypothesis_id{0};
  double encirclement_phi0{0.0};
  std::uint64_t encirclement_generation{0};
  int kind{0};
  int safety_class{0};
  bool success{false};
  bool static_valid{false};
  bool dynamics_valid{false};
  bool swarm_valid{false};
  std::uint8_t conflict_reason_mask{0};
  bool los_conflict_valid{false};
  int los_obstacle_identity{-1};
  double los_conflict_time{-1.0};
  double start_time{0.0};
  double duration{0.0};
  double evaluation_horizon{0.0};
  double dynamic_clearance{-1.0};
  double tracking_score{0.0};
  double native_cost{0.0};
  std::function<Eigen::Vector3d(double)> position_at_global_time;
  std::function<bool(double)> visible_at_global_time;
};

struct TopologyBundleCore
{
  int drone_id{-1};
  std::uint64_t planning_generation{0};
  double planning_epoch{0.0};
  double bundle_stamp{0.0};
  double evaluation_start_time{0.0};
  double requested_evaluation_horizon{0.0};
  bool outer_loop_evaluation{false};
  int local_candidate_id{-1};
  int local_hypothesis_id{0};
  int local_kind{0};
  Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_velocity{Eigen::Vector3d::Zero()};
  std::vector<TopologyCandidateCore> candidates;
};

struct TopologyCoordinatorParams
{
  double swarm_clearance{0.50};
  double near_margin{0.08};
  double sample_dt{0.05};
  double max_bundle_age{0.65};
  double max_epoch_skew{0.65};
  double max_start_skew{0.65};
  double preferred_view_angle_deg{25.0};
  double theta_gap_max{170.0 * M_PI / 180.0};
  double encirclement_ratio_min{0.8};
  double encirclement_spread_saturation_angle_deg{60.0};
  double k2_protection_tolerance{0.02};
  double rapid_reversal_window{1.0};
  double crossing_time_tolerance{0.25};
  int team_top_k{3};
  TeamObjectiveWeights objective_weights;
};

struct TopologyCombinationMetrics
{
  bool valid{false};
  int minimum_safety_class{0};
  std::array<int, 3> candidate_ids{{-1, -1, -1}};
  std::array<int, 3> kinds{{0, 0, 0}};
  int encirclement_hypothesis_id{0};
  double encirclement_phi0{0.0};
  std::uint64_t encirclement_generation{0};
  double team_min_separation{std::numeric_limits<double>::infinity()};
  int conflict_pair_count{0};
  int crossing_pair_count{0};
  int temporally_separated_crossing_pair_count{0};
  double conflict_duration{0.0};
  double near_conflict_duration{0.0};
  int severe_conflict_samples{0};
  double temporal_fix_burden{0.0};
  double atleast2_visibility{0.0};
  double all3_visibility{0.0};
  double none_visibility{0.0};
  double mean_visible_count{0.0};
  double binary_camera_time{0.0};
  std::array<double,3> individual_visibility{{0.0,0.0,0.0}};
  std::array<double,3> longest_individual_loss{{0.0,0.0,0.0}};
  double weakest_camera_visibility{0.0};
  double maximum_camera_loss{0.0};
  double encirclement_ratio{0.0};
  double geometry_cost{0.0};
  std::array<Eigen::Vector3d,3> initial_relative, final_relative;
  double gap_max_p50{0.0}, gap_max_p95{0.0}, gap_max_max{0.0};
  double longest_k2_loss{0.0};
  double longest_blackout{0.0};
  double min_pairwise_view_angle_deg{0.0};
  double mean_pairwise_view_angle_deg{0.0};
  double diversity_score{0.0};
  double complementarity_score{0.0};
  TeamQualityVector quality;
  double cheap_objective{std::numeric_limits<double>::infinity()};
  double min_dynamic_clearance{-1.0};
  double tracking_score{0.0};
  double native_cost{0.0};
  double scalar_utility{-std::numeric_limits<double>::infinity()};
  double evaluation_start_time{0.0};
  double evaluation_horizon{0.0};
  bool outer_loop_evaluation{false};
};

struct TopologyHistory
{
  bool valid{false};
  int encirclement_hypothesis_id{0};
  std::array<int, 3> kinds{{0, 0, 0}};
  bool previous_valid{false};
  int previous_encirclement_hypothesis_id{0};
  std::array<int, 3> previous_kinds{{0, 0, 0}};
  double last_switch_time{-std::numeric_limits<double>::infinity()};
};

struct TopologySelectionCore
{
  bool coordination_available{false};
  std::string reason;
  int combination_count{0};
  int cross_hypothesis_combination_count{0};
  TopologyCombinationMetrics joint;
  TopologyCombinationMetrics local;
  bool differed_from_local{false};
  bool improved_min_separation{false};
  bool reduced_conflict{false};
  bool improved_k2{false};
  bool improved_diversity{false};
  bool worsened_safety{false};
  bool topology_switched{false};
  bool rapid_reversal{false};
  double best_k2{0.0};
  double selected_lost_k2_from_best{0.0};
  /* 本轮修复 3：评价视野被收敛到三机真实公共已验证前缀的审计字段。 */
  double configured_evaluation_horizon{0.0};
  double common_validated_horizon{0.0};
  double effective_evaluation_horizon{0.0};
  bool horizon_clamped_to_common_prefix{false};
  std::vector<TopologyCombinationMetrics> top_k;
};

class TopologyCoordinatorCore
{
public:
  explicit TopologyCoordinatorCore(const TopologyCoordinatorParams &params = {});

  TopologySelectionCore select(const std::array<TopologyBundleCore, 3> &bundles,
                               double now,
                               const TopologyHistory &history = {}) const;

  static bool executable(const TopologyCandidateCore &candidate);
private:
  TopologyCoordinatorParams params_;

  TopologyCombinationMetrics evaluate(
      const std::array<const TopologyCandidateCore *, 3> &combo,
      const std::array<TopologyBundleCore, 3> &bundles,
      double evaluation_start_time, double evaluation_horizon,
      bool evaluate_crossings = false) const;
  bool better(const TopologyCombinationMetrics &lhs,
              const TopologyCombinationMetrics &rhs) const;
  bool safetyBetter(const TopologyCombinationMetrics &lhs,
                    const TopologyCombinationMetrics &rhs) const;
  bool safetyEquivalent(const TopologyCombinationMetrics &lhs,
                        const TopologyCombinationMetrics &rhs) const;
};

}  // namespace multi_uav_formation
