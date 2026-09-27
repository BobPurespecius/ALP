#ifndef MULTI_UAV_FORMATION_TEAM_SOLUTION_COMMIT_CONTRACT_H
#define MULTI_UAV_FORMATION_TEAM_SOLUTION_COMMIT_CONTRACT_H

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace multi_uav_formation
{

enum class TeamAckAction
{
  IGNORE,
  RECORD,
  COMMIT,
  ABORT
};

enum class PostCommitTeamAction
{
  KEEP_COMMITTED_TEAM_TAIL,
  REVOKE_TEAM_COMMIT
};

// A negative ACK can cross the COMMIT publication boundary. Once COMMIT has
// been published, ordinary pre-commit generation/quality notifications are
// stale and must not turn a team schedule into a partial activation. Only an
// explicit hard-safety revoke remains authoritative.
inline bool isPostCommitHardSafetyRevoke(const std::string &reason)
{
  static const std::string prefix = "TEAM_COMMIT_REVOKE:";
  return reason.compare(0, prefix.size(), prefix) == 0;
}

inline PostCommitTeamAction postCommitAction(
    const bool commit_received, const bool hard_safety_invalidated)
{
  if (!commit_received)
    return PostCommitTeamAction::KEEP_COMMITTED_TEAM_TAIL;
  return hard_safety_invalidated
      ? PostCommitTeamAction::REVOKE_TEAM_COMMIT
      : PostCommitTeamAction::KEEP_COMMITTED_TEAM_TAIL;
}

inline bool validTeamActivationCardinality(const int activated_members)
{
  return activated_members == 0 || activated_members == 3;
}

struct TeamAckContractInput
{
  bool active{false};
  bool committed{false};
  std::uint64_t team_solution_id{0};
  std::uint64_t coordination_generation{0};
  std::array<std::uint64_t, 3> expected_planning_generation{{0, 0, 0}};
  std::array<int, 3> expected_candidate_id{{-1, -1, -1}};
  std::array<bool, 3> already_acked{{false, false, false}};
  double activation_time{0.0};

  std::uint64_t ack_team_solution_id{0};
  std::uint64_t ack_coordination_generation{0};
  int ack_drone_id{-1};
  std::uint64_t ack_planning_generation{0};
  int ack_candidate_id{-1};
  bool ack_accepted{false};
  std::string ack_reason;
  double now{0.0};
  double minimum_commit_lead{0.025};
};

struct TeamAckContractResult
{
  TeamAckAction action{TeamAckAction::IGNORE};
  std::array<bool, 3> acked{{false, false, false}};
  std::string reason;
};

inline bool postAckCasReady(const bool all_acked, const double all_acked_time,
                            const double now, const double settle_time)
{
  return all_acked && std::isfinite(all_acked_time) && std::isfinite(now) &&
      std::isfinite(settle_time) && settle_time >= 0.0 &&
      now >= all_acked_time + settle_time;
}

inline TeamAckContractResult evaluateTeamAckContract(
    const TeamAckContractInput &input)
{
  TeamAckContractResult result;
  result.acked = input.already_acked;
  if (!input.active || input.committed ||
      input.ack_team_solution_id != input.team_solution_id ||
      input.ack_coordination_generation != input.coordination_generation)
  {
    result.reason = "NOT_CURRENT_PROPOSAL";
    return result;
  }
  if (input.ack_drone_id < 0 || input.ack_drone_id >= 3)
  {
    result.action = TeamAckAction::ABORT;
    result.reason = "INVALID_ACK_DRONE_ID";
    return result;
  }
  const int drone = input.ack_drone_id;
  if (input.ack_planning_generation !=
          input.expected_planning_generation[drone] ||
      input.ack_candidate_id != input.expected_candidate_id[drone])
  {
    result.action = TeamAckAction::ABORT;
    result.reason = "ACK_IDENTITY_MISMATCH_UAV" + std::to_string(drone);
    return result;
  }
  if (!input.ack_accepted)
  {
    result.action = TeamAckAction::ABORT;
    result.reason = "UAV" + std::to_string(drone) + "_REJECT:" +
                    input.ack_reason;
    return result;
  }
  result.acked[drone] = true;
  if (!result.acked[0] || !result.acked[1] || !result.acked[2])
  {
    result.action = TeamAckAction::RECORD;
    result.reason = "WAITING_FOR_TEAM";
    return result;
  }
  if (input.activation_time - input.now < input.minimum_commit_lead)
  {
    result.action = TeamAckAction::ABORT;
    result.reason = "ACTIVATION_LEAD_EXHAUSTED";
    return result;
  }
  result.action = TeamAckAction::COMMIT;
  result.reason = "ALL_THREE_ACKED";
  return result;
}

}  // namespace multi_uav_formation

#endif  // MULTI_UAV_FORMATION_TEAM_SOLUTION_COMMIT_CONTRACT_H
