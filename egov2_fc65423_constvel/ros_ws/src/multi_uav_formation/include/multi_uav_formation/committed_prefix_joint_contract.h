#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optimizer/poly_traj_utils.hpp>

namespace multi_uav_formation
{

struct CommittedPrefixMember
{
  int drone_id{-1};
  std::uint64_t owner_revision{0};
  int owner_trajectory_id{-1};
  bool safety_validated{false};
  double committed_start{0.0};
  double validated_end{0.0};
};

struct CommittedPrefixSnapshot
{
  bool valid{false};
  double captured_at{0.0};
  double frontier{0.0};
  std::array<CommittedPrefixMember, 3> members;
};

inline CommittedPrefixSnapshot discoverCommittedFutureFrontier(
    const std::array<CommittedPrefixMember, 3> &members,
    const double now, const double preparation_budget,
    const double joint_budget, const double ack_budget,
    const double activation_margin)
{
  CommittedPrefixSnapshot snapshot;
  snapshot.captured_at = now;
  snapshot.members = members;
  if (!std::isfinite(now) || !std::isfinite(preparation_budget) ||
      !std::isfinite(joint_budget) || !std::isfinite(ack_budget) ||
      !std::isfinite(activation_margin) || preparation_budget < 0.0 ||
      joint_budget < 0.0 || ack_budget < 0.0 || activation_margin < 0.0)
    return snapshot;
  double frontier = now + preparation_budget + joint_budget + ack_budget +
                    activation_margin;
  double common_end = std::numeric_limits<double>::infinity();
  for (int drone = 0; drone < 3; ++drone)
  {
    const auto &member = members[drone];
    if (member.drone_id != drone || member.owner_revision == 0 ||
        member.owner_trajectory_id < 0 || !member.safety_validated ||
        !std::isfinite(member.committed_start) ||
        !std::isfinite(member.validated_end) ||
        member.validated_end <= member.committed_start)
      return snapshot;
    frontier = std::max(frontier, member.committed_start);
    common_end = std::min(common_end, member.validated_end);
  }
  if (!std::isfinite(frontier) || !std::isfinite(common_end) ||
      frontier >= common_end - activation_margin)
    return snapshot;
  snapshot.frontier = frontier;
  snapshot.valid = true;
  return snapshot;
}

inline bool predictionWindowCovers(const double valid_from,
                                   const double valid_to,
                                   const double frontier,
                                   const double horizon)
{
  return std::isfinite(valid_from) && std::isfinite(valid_to) &&
         std::isfinite(frontier) && std::isfinite(horizon) && horizon > 0.0 &&
         valid_from <= frontier + 1.0e-9 &&
         valid_to + 1.0e-9 >= frontier + horizon;
}

inline bool committedFrontierStillCurrent(
    const CommittedPrefixSnapshot &captured,
    const std::array<CommittedPrefixMember, 3> &current)
{
  if (!captured.valid)
    return false;
  for (int drone = 0; drone < 3; ++drone)
  {
    const auto &before = captured.members[drone];
    const auto &after = current[drone];
    if (after.drone_id != drone || !after.safety_validated ||
        after.owner_revision != before.owner_revision ||
        after.owner_trajectory_id != before.owner_trajectory_id ||
        after.committed_start != before.committed_start ||
        after.validated_end + 1.0e-9 < captured.frontier)
      return false;
  }
  return true;
}

inline bool reheadCommittedFutureTailSeed(
    const poly_traj::Trajectory &source, const Eigen::Vector3d &head_p,
    const Eigen::Vector3d &head_v, const Eigen::Vector3d &head_a,
    const double horizon, poly_traj::MinJerkOpt &optimizer)
{
  const int piece_count = source.getPieceNum();
  const double source_duration = source.getTotalDuration();
  if (piece_count <= 0 || !head_p.allFinite() || !head_v.allFinite() ||
      !head_a.allFinite() || !std::isfinite(horizon) || horizon <= 1.0e-6 ||
      !std::isfinite(source_duration) || source_duration <= 1.0e-6)
    return false;
  Eigen::VectorXd durations = source.getDurations();
  durations *= horizon / source_duration;
  Eigen::MatrixXd inner(3, std::max(0, piece_count - 1));
  double cumulative = 0.0;
  const Eigen::VectorXd source_durations = source.getDurations();
  for (int piece = 0; piece < piece_count - 1; ++piece)
  {
    cumulative += source_durations(piece);
    inner.col(piece) = source.getPos(cumulative);
  }
  Eigen::Matrix3d head_state;
  Eigen::Matrix3d tail_state;
  head_state << head_p, head_v, head_a;
  tail_state << source.getPos(source_duration), source.getVel(source_duration),
      source.getAcc(source_duration);
  optimizer.reset(head_state, tail_state, piece_count);
  optimizer.generate(inner, durations);
  const auto trajectory = optimizer.getTraj();
  return trajectory.getPieceNum() == piece_count &&
         std::isfinite(trajectory.getTotalDuration()) &&
         (trajectory.getPos(0.0) - head_p).norm() <= 1.0e-9 &&
         (trajectory.getVel(0.0) - head_v).norm() <= 1.0e-9 &&
         (trajectory.getAcc(0.0) - head_a).norm() <= 1.0e-9;
}

struct LocalRollingTrace
{
  std::uint64_t candidate_id{0};
  std::uint64_t commit_id{0};
  std::uint64_t activation_id{0};
  double activation_time{0.0};
  double validated_end{0.0};
  int hold_count{0};
  int starvation_count{0};
  int end_before_next_count{0};
  int unvalidated_count{0};
};

inline bool sameLocalRollingTrace(const LocalRollingTrace &off,
                                  const LocalRollingTrace &joint_failed)
{
  return off.candidate_id == joint_failed.candidate_id &&
         off.commit_id == joint_failed.commit_id &&
         off.activation_id == joint_failed.activation_id &&
         off.activation_time == joint_failed.activation_time &&
         off.validated_end == joint_failed.validated_end &&
         off.hold_count == joint_failed.hold_count &&
         off.starvation_count == joint_failed.starvation_count &&
         off.end_before_next_count == joint_failed.end_before_next_count &&
         off.unvalidated_count == joint_failed.unvalidated_count;
}

}  // namespace multi_uav_formation
