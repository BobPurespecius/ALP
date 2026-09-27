#include <multi_uav_formation/team_solution_commit_contract.h>

#include <iostream>
#include <cmath>
#include <limits>
#include <multi_uav_formation/activation_schedule.h>

int main()
{
  using namespace multi_uav_formation;
  bool ok = !isPostCommitHardSafetyRevoke(
      "LOCAL_GENERATION_INVALIDATED:LOCAL_SUCCESSOR_SUPERSEDED_GENERATION");
  ok = ok && isPostCommitHardSafetyRevoke(
      "TEAM_COMMIT_REVOKE:STATIC_INVALIDATED");
  std::cout << "[team-commit-contract] post_commit_nack_classification="
            << ok << '\n';
  using multi_uav_formation::TeamAckAction;
  using multi_uav_formation::TeamAckContractInput;
  using multi_uav_formation::evaluateTeamAckContract;
  PipelineLatencyEstimate joint(0.35), prepare(0.020);
  joint.observe(0.310); // exceeds the entire old fixed activation lead
  prepare.observe(0.035);
  const auto adaptive = scheduleTeamActivation(10.0, 0.24, prepare.budget(), joint.budget(), 0.060);
  const auto seed = scheduleTeamActivation(10.0, 0.24, prepare.budget(), joint.budget(), 0.060);
  ok = ok && adaptive.valid && adaptive.lead > 0.24 &&
      std::abs(adaptive.required_margin-0.085)<1e-12 &&
      adaptive.activation == seed.activation &&
      activationReserveAvailable(adaptive.activation, 10.0+0.310+0.035, 0.085);
  ok = ok && !activationReserveAvailable(10.240, 10.310+0.035, 0.085);
  const double original = joint.budget();
  joint.observe(0.600);
  ok = ok && joint.budget() > original;
  joint.observe(1.1);
  ok = ok && !scheduleTeamActivation(10.0, 0.24, prepare.budget(), joint.budget(), 0.060).valid;
  ok = ok && !activationReserveAvailable(10.2, 10.19, 0.085) &&
      !activationReserveAvailable(std::numeric_limits<double>::quiet_NaN(), 10.0, 0.085);
  PipelineLatencyEstimate recovery(0.35);
  // Replay optimized-runtime measurements: fast early failures must not
  // calibrate the solver, but three full evaluations release the cold budget.
  for (int i=0;i<8;++i) recovery.observe(0.00003, false);
  ok = ok && recovery.completeObservations()==0 && recovery.budget()>=0.4375;
  recovery.observe(0.004188);
  recovery.observe(0.008688);
  ok = ok && recovery.budget()>=0.4375;
  recovery.observe(0.003329);
  const auto calibrated = scheduleTeamActivation(10.0,0.24,0.020,recovery.budget(),0.060);
  ok = ok && recovery.completeObservations()==3 && calibrated.valid &&
      std::abs(calibrated.lead-0.24)<1e-12 &&
      activationReserveAvailable(calibrated.activation,10.020+0.008688,0.085);
  // The observed 496 ms old bundle fits the unchanged planner window after
  // calibration; no stale timeout or identity rule has been relaxed.
  ok = ok && 0.496+0.020+recovery.budget()+0.060 < 0.98;
  recovery.observe(0.600, false);
  ok = ok && recovery.budget()>=0.750;
  for (int i=0;i<40;++i) recovery.observe(0.010);
  ok = ok && recovery.budget() >= 0.020 && recovery.budget() < original;
  std::cout << "[activation-lifecycle] slow_pipeline_rescheduled=" << ok
      << " planned_lead_ms=" << adaptive.lead*1000.0
      << " proposal_remaining_ms=" << (adaptive.lead-0.345)*1000.0
      << " required_margin_ms=" << adaptive.required_margin*1000.0
      << " seed_adaptive_same_contract=1 impossible_schedule_rejected=1\n";
  TeamAckContractInput input;
  input.active = true;
  input.team_solution_id = 42;
  input.coordination_generation = 9;
  input.expected_planning_generation = {{100, 101, 102}};
  input.expected_candidate_id = {{3, 7, 11}};
  input.activation_time = 10.2;
  input.now = 10.0;
  input.ack_team_solution_id = 42;
  input.ack_coordination_generation = 9;

  for (int drone = 0; drone < 3; ++drone)
  {
    input.ack_drone_id = drone;
    input.ack_planning_generation =
        input.expected_planning_generation[drone];
    input.ack_candidate_id = input.expected_candidate_id[drone];
    input.ack_accepted = true;
    const auto result = evaluateTeamAckContract(input);
    const TeamAckAction expected =
        drone == 2 ? TeamAckAction::COMMIT : TeamAckAction::RECORD;
    ok = ok && result.action == expected;
    input.already_acked = result.acked;
  }
  std::cout << "[team-commit-contract] all_three_ack_commit=" << ok << '\n';
  ok = ok && !postAckCasReady(true, 10.0, 10.019, 0.02) &&
      postAckCasReady(true, 10.0, 10.020, 0.02) &&
      !postAckCasReady(false, 10.0, 10.100, 0.02);
  std::cout << "[team-commit-contract] post_ack_cas_settle=" << ok << '\n';

  TeamAckContractInput stale = input;
  stale.already_acked = {{false, false, false}};
  stale.ack_team_solution_id = 41;
  ok = ok && evaluateTeamAckContract(stale).action == TeamAckAction::IGNORE;

  TeamAckContractInput mismatched = input;
  mismatched.already_acked = {{false, false, false}};
  mismatched.ack_drone_id = 1;
  mismatched.ack_planning_generation = 999;
  ok = ok &&
       evaluateTeamAckContract(mismatched).action == TeamAckAction::ABORT;

  TeamAckContractInput rejected = input;
  rejected.already_acked = {{false, false, false}};
  rejected.ack_drone_id = 0;
  rejected.ack_planning_generation = 100;
  rejected.ack_candidate_id = 3;
  rejected.ack_accepted = false;
  rejected.ack_reason = "STATIC_RECHECK_FAILED";
  ok = ok &&
       evaluateTeamAckContract(rejected).action == TeamAckAction::ABORT;

  TeamAckContractInput late = input;
  late.already_acked = {{true, true, false}};
  late.ack_drone_id = 2;
  late.ack_planning_generation = 102;
  late.ack_candidate_id = 11;
  late.ack_accepted = true;
  late.now = 10.19;
  ok = ok && evaluateTeamAckContract(late).action == TeamAckAction::ABORT;

  std::cout << "[team-commit-contract] stale_ignore_identity_abort=" << ok
            << " reject_abort_late_abort=" << ok << '\n';
  std::cout << (ok ? "TEAM_SOLUTION_COMMIT_CONTRACT_PASS\n"
                   : "TEAM_SOLUTION_COMMIT_CONTRACT_FAIL\n");
  return ok ? 0 : 1;
}
