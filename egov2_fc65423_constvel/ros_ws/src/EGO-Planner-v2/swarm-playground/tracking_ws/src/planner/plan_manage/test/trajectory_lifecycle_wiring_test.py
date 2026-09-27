#!/usr/bin/env python3
"""Execution-entry contracts complement the numerical lifecycle tests."""
from pathlib import Path
import unittest
P=Path(__file__).resolve().parents[1]
SRC=P.parents[5]
def body(text,signature):
 b=text.index('{',text.index(signature));i=b+1;d=1
 while d:
  d+=(text[i]=='{')-(text[i]=='}');i+=1
 return text[b:i]
class LifecycleWiring(unittest.TestCase):
 @classmethod
 def setUpClass(c):
  c.pm=(P/'src/planner_manager.cpp').read_text();c.fsm=(P/'src/ego_replan_fsm.cpp').read_text();c.ts=(P/'src/traj_server.cpp').read_text()
  c.co=(SRC/'multi_uav_formation/src/multi_uav_topology_coordinator.cpp').read_text()
 def test_pending_transaction_does_not_freeze_normal_fsm(self):
  pre=body(self.fsm,'void EGOReplanFSM::execFSMCallback').split('switch (exec_state_)')[0]
  self.assertNotIn('TopologyProcessStatus::PENDING)',pre)
  replan=body(self.fsm,'case REPLAN_TRAJ:')
  self.assertNotIn('topologyCoordinationPending()',replan)
  call=body(self.fsm,'EGOReplanFSM::ReplanOutcome EGOReplanFSM::callReboundReplan')
  self.assertIn('localActivationPending()',call)
  self.assertIn('action=PLAN_ANYWAY',call)
 def test_local_reserve_precedes_bundle_and_has_no_execution_authority(self):
  b=body(self.fsm,'EGOReplanFSM::ReplanOutcome EGOReplanFSM::callReboundReplan')
  self.assertLess(b.index('finalizeCapturedCandidates('),b.index('stageCapturedTopologyCoordination('))
  self.assertIn('local_first_safe_direct',b)
  self.assertIn('local_gate_by_joint=0',b)
 def test_transaction_boundary_survives_callback_queue_latency(self):
  stage=body(self.pm,'EGOPlannerManager::stageCapturedTopologyCoordination(')
  commit=body(self.pm,'bool EGOPlannerManager::setLocalTrajFromOpt')
  quality=body(self.pm,'bool EGOPlannerManager::backgroundTeamQualityAdmissible(')
  self.assertIn('pending_topology_.active',stage)
  self.assertIn('boundary_source_revision',stage)
  self.assertIn('boundary_source_trajectory_id',stage)
  self.assertIn('invalidatePendingTopology("LOCAL_SUCCESSOR_SUPERSEDED_GENERATION")',commit)
  self.assertNotIn('return false;\n    poly_traj::Trajectory',commit.split('Local rolling execution')[1][:500])
  self.assertIn('execution_safe_until_',quality)
  self.assertIn('planningToCommitMedian()',quality)
  self.assertIn('std::max(team_coordination_timeout_, team_solution_max_age_)',stage)
 def test_zero_joint_seed_no_publish(self):
  b=body(self.pm,'EGOPlannerManager::stageCapturedTopologyCoordination(')
  self.assertLess(b.index('joint_seed_count <= 0'),b.index('pending_topology_.active = true'))
  branch=body(b,'if (joint_seed_count <= 0')
  self.assertIn('pending_topology_ = PendingTopologyState()',branch);self.assertIn('return false',branch)
 def test_generation_negative_ends_pending(self):
  cb=body(self.pm,'void EGOPlannerManager::topologyResultCallback')
  negative=body(cb,'if (!message->valid)')
  self.assertIn('planning_generations[i] == pending_topology_.planning_generation',negative)
  self.assertIn('latest_topology_result_',negative)
  self.assertIn('JOINT_DISCARD_LOCAL_UNCHANGED',negative)
  self.assertIn('publishTerminalNegative("NO_EXECUTABLE_COMBINATION")',self.co)
 def test_actual_active_not_source_only_gate(self):
  callback=body(self.pm,'void EGOPlannerManager::teamTrajectorySolutionCallback')
  for s in ('prefixTrajectory(expected_source_suffix', 'teamProposalBoundaryMatches(proposed_traj, expected_source_suffix','checkActiveHandoff(proposed_traj','currentGeneration('):self.assertIn(s,callback)
  boundary=body(self.pm,'bool EGOPlannerManager::teamProposalBoundaryMatches(')
  for s in ('scheduled_predecessor_','predecessor.traj.getPos(elapsed)','predecessor.traj.getVel(elapsed)','predecessor.traj.getAcc(elapsed)','source_suffix.getPos(source_duration)','source_suffix.getVel(source_duration)','source_suffix.getAcc(source_duration)'):
   self.assertIn(s,boundary)
  self.assertEqual(boundary.count('<=1e-5'),6)
  self.assertIn('!require_source_tail',boundary)
  commit=body(self.pm,'bool EGOPlannerManager::setLocalTrajFromOpt')
  self.assertLess(commit.index('checkActiveHandoff'),commit.index('traj_.setLocalTraj'))
  self.assertLess(commit.index('validateExecutionTrajectory'),commit.index('traj_.setLocalTraj'))
 def test_future_head_before_all_safety_and_sfc(self):
  finalize=body(self.pm,'EGOPlannerManager::finalizeCapturedCandidates(')
  for gate in ('validateRetimedLocalSfc','checkActiveHandoff','validateExecutionTrajectory'):
   self.assertLess(finalize.index('prepareLocalHandoff'),finalize.index(gate))
  self.assertNotIn('retryTrajectory',finalize)
 def test_expiry_and_invalid_active_retry_moving_planning(self):
  self.assertIn('lifecycleSuccessorDue()',self.fsm)
  self.assertNotIn('validatedBrakeSuccessor',self.pm)
  self.assertIn('changeFSMExecState(REPLAN_TRAJ, "MOVING_ROLLING_RETRY")',self.fsm)
  # 本轮修复（rolling/terminal 语义）：finalize 的最终 fallback 不再硬编码
  # terminal 全时长语义，改为由被校验轨迹自身携带的语义决定。
  self.assertIn('validatePreviousRemainingTrajectory(ros::Time::now().toSec(),',
                self.pm)
  self.assertIn('active_execution_touch_goal_,reason,', self.pm)
  self.assertNotIn('validatePreviousRemainingTrajectory(now,true', self.pm)
 def test_target_updates_cannot_wait_for_local_or_zero_moving_tail(self):
  self.assertNotIn('while (exec_state_ != EXEC_TRAJ)',self.fsm)
  self.assertIn('object_v_ = twist_v.allFinite()',self.fsm)
  self.assertNotIn('object_v_ = enable_fov_tracking_',self.fsm)
  self.assertIn('cooperative_goal_velocity_ : (satrt_tracking_ ? object_v_',self.fsm)
 def test_executor_gate_and_shared_end_model(self):
  self.assertIn('executionHandoffGate(*scheduled_traj_,scheduled_source_,"ACTIVATION")',self.ts)
  self.assertIn('CURRENT_STATE_RESTART_VALIDATED',self.pm)
  self.assertIn('currentStateRestartHandoff(',self.ts)
  self.assertIn('event=EXECUTOR_HANDOFF_ACCEPTED',self.ts)
  self.assertIn('authority=" << (current_state_restart ? "CURRENT_STATE"',self.ts)
  self.assertIn('trajectory_lifecycle::sample(*traj_,t_cur).p',self.ts)
  self.assertIn('trajectory_lifecycle::sample(other.traj, other_t).p',self.pm)
 def test_scheduled_predecessor_order_and_prefix_validation(self):
  self.assertIn('trajectory_lifecycle::sample(scheduled_predecessor_.traj',self.pm)
  commit=body(self.pm,'bool EGOPlannerManager::setLocalTrajFromOpt')
  self.assertIn('invalidatePendingTopology("LOCAL_SUCCESSOR_SUPERSEDED_GENERATION")',commit)
  self.assertNotIn('pending_team_trajectory_.accepted',commit)
  self.assertIn('validateCommittedPrefixUntil(activation,safety_reason)',self.pm)
 def test_no_timeout_restarts_original_local_head(self):
  pending=body(self.pm,'EGOPlannerManager::processPendingTopologyCoordination()')
  self.assertNotIn('finalizeTopologyCandidate(',pending)
  self.assertIn('JOINT_FAILURE_LOCAL_NOOP',pending)
  self.assertIn('pending_topology_.active_generation != active_traj_generation_',pending)
 def test_execution_reserve_precedes_every_planning_batch(self):
  b=body(self.fsm,'EGOReplanFSM::ReplanOutcome EGOReplanFSM::callReboundReplan')
  self.assertLess(b.index('ensureExecutionCoverage()'),b.index('prepareFutureActivation'))
  self.assertNotIn('CoverageDecision::DEFERRED',b)
  self.assertIn('~BatchObservation()',b)
  self.assertIn('optionalRefinementAllowed()',self.pm)
 def test_geometry_after_rehead_and_full_safety(self):
  b=body(self.pm,'EGOPlannerManager::finalizeCapturedCandidates(')
  # prepareLocalHandoff owns the current-revision head regeneration. There is
  # no post-check repair authority: one final preflight accepts or rejects it.
  self.assertLess(b.index('prepareLocalHandoff('),b.index('validateRetimedLocalSfc('))
  self.assertLess(b.index('validateRetimedLocalSfc('),b.index('validateExecutionTrajectory(prepared.getTraj()'))
  self.assertNotIn('retryTrajectory',b)
  self.assertNotIn('continued.getTraj()',b)
  self.assertLess(b.index('validateExecutionTrajectory('),b.index('geometryBetter('))
  self.assertIn('if(!safe)',b)
  self.assertIn('logExecutionGeometry("PERSISTENCE_REVALIDATION")',b)
  self.assertNotIn('geometryRecoveryReplanDue()',self.fsm)
 def test_coverage_uses_only_finite_moving_execution(self):
  b=body(self.pm,'void EGOPlannerManager::ensureExecutionCoverage')
  self.assertNotIn('setLocalTrajFromOpt',b)
  self.assertIn('execution_reserve_limited_=true',b)
  self.assertIn('MOVING_SUCCESSOR_STARVATION',b)
  prefix=body(self.pm,'bool EGOPlannerManager::validateActivePrefixUntil')
  self.assertIn('MOVING_SUFFIX_EXHAUSTED',prefix)
  self.assertNotIn('CoefficientMat hold',prefix)
  committed=body(self.pm,'bool EGOPlannerManager::validateCommittedPrefixUntil')
  self.assertIn('scheduled_predecessor_',committed)
  self.assertIn('traj_.local_traj',committed)
  self.assertGreaterEqual(committed.count('validate_interval'),3)
  self.assertIn('COMMITTED_SUCCESSOR_BOUNDARY_MISMATCH',committed)
  self.assertIn('validateExecutionTrajectory(segment, begin, false, reason)',committed)
 def test_expired_predecessor_ends_old_authority_and_reuses_local_planner(self):
  coverage=body(self.pm,'void EGOPlannerManager::ensureExecutionCoverage')
  for token in ('PREDECESSOR_HANDOFF_DEADLINE_EXPIRED',
                'event=ENTER',
                'movingPlanningBudget(',
                'execution_budget_.estimate(wall)'):
   self.assertIn(token,coverage)
  future=body(self.pm,'void EGOPlannerManager::prepareFutureActivation')
  self.assertIn('current_state_p_=p',future)
  self.assertIn('currentStateAt(local_activation_time_)',future)
  handoff=body(self.pm,'bool EGOPlannerManager::checkActiveHandoff')
  self.assertIn('currentStateAt(activation)',handoff)
  commit=body(self.pm,'bool EGOPlannerManager::setLocalTrajFromOpt')
  self.assertIn('!current_state_commit && !validateCommittedPrefixUntil',commit)
  self.assertLess(commit.index('checkActiveHandoff'),commit.index('validateExecutionTrajectory'))
  for gate in ('checkTrajectoryDynamics','checkTrajectoryStaticSafety',
               'evaluateDynamicRisk','checkTrajectorySwarmSafety'):
   self.assertIn(gate,body(self.pm,'bool EGOPlannerManager::validateExecutionTrajectory'))
  self.assertIn('event=SUCCESS',commit)
  self.assertIn('event=EXIT',commit)
 def test_optional_quality_and_mandatory_supply_have_distinct_entry_checks(self):
  fsm=body(self.fsm,'EGOReplanFSM::ReplanOutcome EGOReplanFSM::callReboundReplan')
  self.assertIn('ensureExecutionCoverage()',fsm)
  replan=body(self.pm,'bool EGOPlannerManager::reboundReplan')
  self.assertIn('mandatoryPlanningAttemptAllowed()',replan)
  self.assertIn('if(!optional_allowed && !mandatory_allowed)',replan)
 def test_quadrature_deadline_does_not_accept_partial_cost(self):
  opt=(P.parent/'traj_opt/src/poly_traj_optimizer.cpp').read_text()
  b=body(opt,'void PolyTrajOptimizer::addDirectionalVisibilityGradCost2CT(')
  self.assertIn('if((sample & 63)==0) executionCheckpoint()',b)
  callback=body(opt,'double PolyTrajOptimizer::costFunctionCallback(')
  self.assertNotIn('addStaticLosGradCost2CT(',callback)
  self.assertIn('catch(const ExecutionDeadlineExceeded &)',callback)
  self.assertIn('std::numeric_limits<double>::infinity()',callback)
  gateway=body(opt,'bool PolyTrajOptimizer::optimizeTrajectory(')
  self.assertIn('jerkOpt_.generate(initInnerPts,initT)',gateway)
  self.assertIn('partial_cost_accepted=0',gateway)
  self.assertIn('return false;',gateway)
 def test_joint_activation_budget_is_not_double_charged(self):
  coordinator=self.co
  self.assertIn('joint_budget + schedule.required_margin)',coordinator)
  self.assertNotIn(
      'joint_budget + schedule.required_margin + prepare_budget',coordinator)
if __name__=='__main__':unittest.main()
