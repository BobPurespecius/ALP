#pragma once

#include <multi_uav_formation/team_objective_primitives.h>
#include <multi_uav_formation/tracking_visibility_geometry.h>

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace multi_uav_formation
{

struct TeamReferenceKnot
{
  double world_time{0.0};
  double radius{0.0};
  double bearing{0.0};
  double height{0.0};
};

struct TeamReferenceMember
{
  int drone_id{-1};
  int source_candidate_id{-1};
  int topology_kind{0};
  double desired_radius{0.0};
  double desired_bearing{0.0};
  double fixed_height{0.0};
  double reference_phase_shift{0.0};
  bool initial_yaw_valid{false};
  double initial_yaw{0.0};
  double initial_yaw_rate{0.0};
  std::vector<TeamReferenceKnot> knots;
};

struct TeamTargetCenteredInput
{
  double evaluation_start_world{0.0};
  double prediction_horizon{2.0};
  std::uint64_t target_prediction_revision{0};
  std::array<TeamReferenceMember, 3> members;
};

struct TeamTargetCenteredParams
{
  double prediction_horizon{2.0};
  double sample_dt{0.10};
  int reference_knot_count{4};
  int top_k{3};
  int max_iterations{10};
  double timeout_seconds{0.045};
  double radius_trust{0.30};
  double bearing_trust{20.0 * M_PI / 180.0};
  double phase_trust{0.45};
  double initial_step{0.08};
  double minimum_step{1.0e-4};
  double radius_deadband{0.15};
  double bearing_deadband{10.0 * M_PI / 180.0};
  double radius_scale{0.60};
  double bearing_scale{30.0 * M_PI / 180.0};
  double regularization_radius_scale{0.25};
  double regularization_bearing_scale{15.0 * M_PI / 180.0};
  double regularization_time_scale{0.30};
  double pair_bad_angle{25.0 * M_PI / 180.0};
  double pair_good_angle{60.0 * M_PI / 180.0};
  // Stage 3 is the only place where these two lower-priority objectives are
  // combined.  Visibility and complementarity floors are enforced by hard
  // filters, not by these coefficients.
  double stage3_regularization_lambda{0.20};
  double quality_guard_barrier{1.0};
  TeamObjectiveWeights weights;
};

enum class TeamOptimizationStage
{
  LOCAL_BASELINE = 0,
  VISIBILITY_Q2 = 1,
  VISIBILITY_ACC = 2,
  COMPLEMENTARITY = 3,
  ENCIRCLEMENT_REGULARIZATION = 4
};

struct TeamTargetCenteredEnvironment
{
  std::function<bool()> cancelled;
  std::function<Eigen::Vector3d(double)> target_at_world_time;
  std::function<ContinuousVisibilitySample(
      int, const Eigen::Vector3d &, const Eigen::Vector3d &, double, double)>
      visibility;
};

struct TeamTargetCenteredMetrics
{
  bool valid{false};
  int sample_count{0};
  TeamObjectiveTerms raw;
  TeamObjectiveTerms weighted;
  TeamQualityVector quality;
  double objective{0.0};
  double k2_fraction{0.0};
  double accumulated_visibility{0.0};
  double complementarity_score{0.0};
  double pair_angle_p50{0.0};
  double pair_angle_p90{0.0};
  double radius_error_p50{0.0};
  double radius_error_p90{0.0};
  double bearing_error_p50{0.0};
  double bearing_error_p90{0.0};
  double height_error_p90{0.0};
  double height_error_max{0.0};
  double simultaneous_los_loss_duration{0.0};
};

struct TeamTargetCenteredResult
{
  bool success{false};
  std::string reason;
  int iterations{0};
  double solve_time_ms{0.0};
  TeamTargetCenteredInput schedule;
  TeamTargetCenteredMetrics before;
  TeamTargetCenteredMetrics after;
  TeamTargetCenteredMetrics stage1_q2;
  TeamTargetCenteredMetrics stage1_acc;
  TeamTargetCenteredMetrics stage2_comp;
  TeamTargetCenteredMetrics stage3_enc;
  TeamQualityTolerance tolerance;
  double q_floor{0.0};
  double visibility_floor{0.0};
  double complementarity_floor{0.0};
  double stage_q2_latency_ms{0.0};
  double stage_acc_latency_ms{0.0};
  double stage_comp_latency_ms{0.0};
  double stage_enc_latency_ms{0.0};
  TeamOptimizationStage final_stage{TeamOptimizationStage::LOCAL_BASELINE};
  bool primary_visibility_rejected{false};
  bool complementarity_rejected{false};
  bool stage1_fallback{false};
  bool stage2_fallback{false};
  bool stage3_fallback{false};
  std::array<double, 3> reference_phase_shifts{{0.0, 0.0, 0.0}};
};

const char *teamOptimizationStageName(TeamOptimizationStage stage);

class TeamTargetCenteredOptimizer
{
public:
  explicit TeamTargetCenteredOptimizer(
      const TeamTargetCenteredParams &params = {});

  TeamTargetCenteredResult optimize(
      const TeamTargetCenteredInput &input,
      const TeamTargetCenteredEnvironment &environment) const;

  TeamTargetCenteredMetrics evaluate(
      const TeamTargetCenteredInput &schedule,
      const TeamTargetCenteredInput &reference,
      const TeamTargetCenteredEnvironment &environment,
      Eigen::VectorXd *gradient = nullptr) const;

  static Eigen::Vector3d referencePosition(
      const TeamReferenceMember &member,
      const Eigen::Vector3d &target_at_world_time,
      double world_time);

  static bool validScheduleIdentity(const TeamTargetCenteredInput &schedule);

  const TeamTargetCenteredParams &params() const { return params_; }

private:
  TeamTargetCenteredParams params_;

  int variableCount(const TeamTargetCenteredInput &input) const;
  Eigen::VectorXd encode(const TeamTargetCenteredInput &schedule,
                         const TeamTargetCenteredInput &reference) const;
  TeamTargetCenteredInput decode(const Eigen::VectorXd &variables,
                                 const TeamTargetCenteredInput &reference) const;
};

}  // namespace multi_uav_formation
