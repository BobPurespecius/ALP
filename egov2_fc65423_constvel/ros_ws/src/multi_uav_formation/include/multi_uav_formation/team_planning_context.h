#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace multi_uav_formation
{

// Stable identity of a prediction producer/model contract.  Rolling sample
// time belongs to target_snapshot_epoch and must not be folded into this
// identity: three asynchronous consumers of one ROS source can legitimately
// hold adjacent samples while still sharing the same prediction authority.
inline std::uint64_t stablePredictionSourceIdentity(
    const std::string &topic, const std::string &frame,
    const std::string &model)
{
  std::uint64_t identity = 1469598103934665603ULL;
  const auto mix = [&identity](const std::string &value) {
    for (const unsigned char character : value)
    {
      identity ^= static_cast<std::uint64_t>(character);
      identity *= 1099511628211ULL;
    }
    identity ^= 0xffULL;
    identity *= 1099511628211ULL;
  };
  mix(topic);
  mix(frame);
  mix(model);
  return identity == 0 ? 1 : identity;
}

struct PlanningWorldSnapshot
{
  std::uint64_t target_snapshot_identity{0};
  double target_snapshot_epoch{0.0};
  std::uint64_t dynamic_prediction_identity{0};
  double dynamic_prediction_valid_from{0.0};
  double dynamic_prediction_valid_to{0.0};
  std::uint64_t static_map_revision{0};
  std::uint64_t visibility_model_version{0};

  bool complete() const
  {
    return target_snapshot_identity != 0 &&
           dynamic_prediction_identity != 0 &&
           static_map_revision != 0 && visibility_model_version != 0 &&
           std::isfinite(target_snapshot_epoch) &&
           std::isfinite(dynamic_prediction_valid_from) &&
           std::isfinite(dynamic_prediction_valid_to) &&
           dynamic_prediction_valid_to >= dynamic_prediction_valid_from;
  }
};

// Identity-only contract for one planner's contribution to an early-joint
// snapshot.  Geometry and polynomials stay in TopologyCandidateBundle; this
// structure keeps the authority/freshness rules deterministic and testable.
struct TeamPlanningMemberContext
{
  int drone_id{-1};
  std::uint64_t planning_generation{0};
  std::uint64_t expected_execution_generation{0};
  int expected_trajectory_id{-1};
  double bundle_stamp{0.0};
  double target_prediction_epoch{0.0};
  double dynamic_prediction_epoch{0.0};
  std::uint64_t target_snapshot_identity{0};
  double target_snapshot_epoch{0.0};
  std::uint64_t dynamic_prediction_identity{0};
  double dynamic_prediction_valid_from{0.0};
  double dynamic_prediction_valid_to{0.0};
  std::uint64_t static_map_revision{0};
  std::uint64_t visibility_model_version{0};
  double earliest_activation{0.0};
  double validated_end{0.0};
  std::array<double, 3> boundary_position{{0.0, 0.0, 0.0}};
  std::array<double, 3> boundary_velocity{{0.0, 0.0, 0.0}};
  std::array<double, 3> boundary_acceleration{{0.0, 0.0, 0.0}};
};

struct TeamPlanningContext
{
  bool valid{false};
  std::string reason;
  std::uint64_t team_generation{0};
  double snapshot_time{0.0};
  double prediction_epoch{0.0};
  std::uint64_t target_snapshot_identity{0};
  std::uint64_t dynamic_prediction_identity{0};
  std::uint64_t static_map_revision{0};
  std::uint64_t visibility_model_version{0};
  double common_activation{0.0};
  double planning_horizon{0.0};
  double wall_deadline{0.0};
  std::array<TeamPlanningMemberContext, 3> members;
};

inline TeamPlanningContext makeTeamPlanningContext(
    const std::array<TeamPlanningMemberContext, 3> &members,
    const std::uint64_t team_generation, const double now,
    const double activation, const double horizon, const double wall_deadline,
    const double maximum_epoch_skew, const double maximum_bundle_age)
{
  TeamPlanningContext context;
  context.team_generation = team_generation;
  context.snapshot_time = now;
  context.common_activation = activation;
  context.planning_horizon = horizon;
  context.wall_deadline = wall_deadline;
  context.members = members;
  if (!std::isfinite(now) || !std::isfinite(activation) ||
      !std::isfinite(horizon) || horizon <= 1.0e-6 ||
      !std::isfinite(wall_deadline) || wall_deadline <= now)
  {
    context.reason = "INVALID_TEAM_TIME";
    return context;
  }
  double min_target_epoch = std::numeric_limits<double>::infinity();
  double max_target_epoch = -std::numeric_limits<double>::infinity();
  for (int drone = 0; drone < 3; ++drone)
  {
    const auto &member = members[drone];
    if (member.drone_id != drone || member.planning_generation == 0 ||
        member.expected_execution_generation == 0 ||
        member.expected_trajectory_id < 0)
    {
      context.reason = "INVALID_MEMBER_IDENTITY";
      return context;
    }
    if (!std::isfinite(member.bundle_stamp) ||
        now - member.bundle_stamp < -0.10 ||
        now - member.bundle_stamp > maximum_bundle_age)
    {
      context.reason = "STALE_MEMBER_BUNDLE";
      return context;
    }
    if (member.target_snapshot_identity == 0 ||
        member.dynamic_prediction_identity == 0 ||
        !std::isfinite(member.target_snapshot_epoch) ||
        !std::isfinite(member.dynamic_prediction_valid_from) ||
        !std::isfinite(member.dynamic_prediction_valid_to) ||
        member.dynamic_prediction_valid_from > activation + 1.0e-9 ||
        member.dynamic_prediction_valid_to + 1.0e-9 < activation + horizon)
    {
      context.reason = "PREDICTION_COVERAGE_INVALID";
      return context;
    }
    if (drone > 0 &&
        (member.target_snapshot_identity !=
             members[0].target_snapshot_identity ||
         member.dynamic_prediction_identity !=
             members[0].dynamic_prediction_identity))
    {
      context.reason = "PREDICTION_IDENTITY_MISMATCH";
      return context;
    }
    min_target_epoch = std::min(min_target_epoch,
                                member.target_snapshot_epoch);
    max_target_epoch = std::max(max_target_epoch,
                                member.target_snapshot_epoch);
    if (member.static_map_revision == 0 ||
        member.visibility_model_version == 0 ||
        (drone > 0 &&
         (member.static_map_revision != members[0].static_map_revision ||
          member.visibility_model_version !=
              members[0].visibility_model_version)))
    {
      context.reason = "MODEL_IDENTITY_MISMATCH";
      return context;
    }
    if (!std::isfinite(member.earliest_activation) ||
        activation + 1.0e-9 < member.earliest_activation ||
        !std::isfinite(member.validated_end) ||
        activation >= member.validated_end - 1.0e-6)
    {
      context.reason = "NO_COMMON_ACTIVATION";
      return context;
    }
    for (int axis = 0; axis < 3; ++axis)
      if (!std::isfinite(member.boundary_position[axis]) ||
          !std::isfinite(member.boundary_velocity[axis]) ||
          !std::isfinite(member.boundary_acceleration[axis]))
      {
        context.reason = "INVALID_BOUNDARY_PVA";
        return context;
      }
  }
  if (max_target_epoch - min_target_epoch > maximum_epoch_skew)
  {
    context.reason = "TARGET_SNAPSHOT_EPOCH_SKEW";
    return context;
  }
  context.prediction_epoch = max_target_epoch;
  context.target_snapshot_identity = members[0].target_snapshot_identity;
  context.dynamic_prediction_identity = members[0].dynamic_prediction_identity;
  context.static_map_revision = members[0].static_map_revision;
  context.visibility_model_version = members[0].visibility_model_version;
  context.valid = true;
  context.reason = "VALID";
  return context;
}

inline bool teamPlanningContextStillCurrent(
    const TeamPlanningContext &context,
    const std::array<std::uint64_t, 3> &planning_generations,
    const std::array<std::uint64_t, 3> &execution_generations,
    const std::array<int, 3> &trajectory_ids,
    const double now)
{
  if (!context.valid || !std::isfinite(now) ||
      now >= context.common_activation)
    return false;
  for (int drone = 0; drone < 3; ++drone)
    if (planning_generations[drone] !=
            context.members[drone].planning_generation ||
        execution_generations[drone] !=
            context.members[drone].expected_execution_generation ||
        trajectory_ids[drone] !=
            context.members[drone].expected_trajectory_id)
      return false;
  return true;
}

struct TeamTopologySeedDescriptor
{
  int drone_id{-1};
  int candidate_id{-1};
  int hypothesis_id{0};
  std::uint64_t hypothesis_generation{0};
  int kind{0};
  bool constructible{false};
  double predicted_camera_time{0.0};
  double dynamic_clearance{-1.0};
  double transition_burden{0.0};
};

struct TeamTopologyTuple
{
  std::array<int, 3> candidate_ids{{-1, -1, -1}};
  std::array<int, 3> kinds{{0, 0, 0}};
  int hypothesis_id{0};
  std::uint64_t hypothesis_generation{0};
  double predicted_camera_time{0.0};
  double minimum_dynamic_clearance{-1.0};
  double transition_burden{0.0};
};

// Enumerate every constructible N/L/R tuple for one shared hypothesis.  The
// result is only a cheap, deterministic order for budget-driven optimization;
// it grants no execution or safety authority.
inline std::vector<TeamTopologyTuple> enumerateTeamTopologyTuples(
    const std::array<std::vector<TeamTopologySeedDescriptor>, 3> &seeds,
    const int shared_hypothesis,
    const std::uint64_t shared_hypothesis_generation)
{
  std::vector<TeamTopologyTuple> tuples;
  for (const auto &first : seeds[0])
    for (const auto &second : seeds[1])
      for (const auto &third : seeds[2])
      {
        const std::array<const TeamTopologySeedDescriptor *, 3> values{{
            &first, &second, &third}};
        bool compatible = true;
        TeamTopologyTuple tuple;
        tuple.hypothesis_id = shared_hypothesis;
        tuple.hypothesis_generation = shared_hypothesis_generation;
        tuple.minimum_dynamic_clearance =
            std::numeric_limits<double>::infinity();
        for (int drone = 0; drone < 3; ++drone)
        {
          const auto &seed = *values[drone];
          compatible = compatible && seed.constructible &&
              seed.drone_id == drone &&
              seed.hypothesis_id == shared_hypothesis &&
              seed.hypothesis_generation == shared_hypothesis_generation;
          tuple.candidate_ids[drone] = seed.candidate_id;
          tuple.kinds[drone] = seed.kind;
          tuple.predicted_camera_time += seed.predicted_camera_time;
          tuple.transition_burden += seed.transition_burden;
          if (std::isfinite(seed.dynamic_clearance) &&
              seed.dynamic_clearance >= 0.0)
            tuple.minimum_dynamic_clearance = std::min(
                tuple.minimum_dynamic_clearance, seed.dynamic_clearance);
        }
        if (compatible)
          tuples.push_back(tuple);
      }
  std::stable_sort(tuples.begin(), tuples.end(),
      [](const TeamTopologyTuple &lhs, const TeamTopologyTuple &rhs) {
        if (std::abs(lhs.predicted_camera_time - rhs.predicted_camera_time) >
            1.0e-9)
          return lhs.predicted_camera_time > rhs.predicted_camera_time;
        if (std::abs(lhs.transition_burden - rhs.transition_burden) > 1.0e-9)
          return lhs.transition_burden < rhs.transition_burden;
        if (std::abs(lhs.minimum_dynamic_clearance -
                     rhs.minimum_dynamic_clearance) > 1.0e-9)
          return lhs.minimum_dynamic_clearance >
                 rhs.minimum_dynamic_clearance;
        return lhs.candidate_ids < rhs.candidate_ids;
      });
  return tuples;
}

inline bool teamBenefitRequiresIndividualSacrifice(
    const std::array<double, 3> &individual_best_values,
    const std::array<double, 3> &selected_individual_values,
    const double local_team_camera_time,
    const double selected_team_camera_time,
    const double numerical_epsilon)
{
  bool sacrifice = false;
  for (int drone = 0; drone < 3; ++drone)
    sacrifice = sacrifice || selected_individual_values[drone] +
        numerical_epsilon < individual_best_values[drone];
  return sacrifice && selected_team_camera_time >
      local_team_camera_time + numerical_epsilon;
}

}  // namespace multi_uav_formation
