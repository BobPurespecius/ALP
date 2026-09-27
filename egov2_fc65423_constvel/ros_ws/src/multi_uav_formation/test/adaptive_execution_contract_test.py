#!/usr/bin/env python3
"""Cross-package wiring contracts; numerical behavior is tested in C++.

These checks deliberately cover the execution entry points, not log strings.
They complement, rather than replace, nonlinear trajectory validation.
"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

SRC = Path(__file__).resolve().parents[2]
PLANNER = SRC / 'EGO-Planner-v2/swarm-playground/tracking_ws/src/planner'


def body(text, signature):
    begin = text.index('{', text.index(signature))
    depth = 1
    end = begin + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin:end]


class AdaptiveExecutionContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manager = (PLANNER / 'plan_manage/src/planner_manager.cpp').read_text()
        cls.fsm = (PLANNER / 'plan_manage/src/ego_replan_fsm.cpp').read_text()
        cls.server = (PLANNER / 'plan_manage/src/traj_server.cpp').read_text()

    def test_every_commit_passes_nonlinear_safety_gateway(self):
        commit = body(self.manager, 'bool EGOPlannerManager::setLocalTrajFromOpt(')
        self.assertLess(commit.index('validateExecutionTrajectory('), commit.index('traj_.setLocalTraj('))
        gateway = body(self.manager, 'bool EGOPlannerManager::validateExecutionTrajectory(')
        for check in ('checkTrajectoryDynamics(', 'checkTrajectoryStaticSafety(trajectory, touch_goal',
                      'evaluateDynamicRisk(trajectory,activation,touch_goal)',
                      'checkTrajectorySwarmSafety(', 'TEAM_SWARM_FAIL'):
            self.assertIn(check, gateway)

    def test_failed_static_dynamic_or_retiming_cannot_reenter_fallback(self):
        finalize = body(self.manager, 'EGOPlannerManager::finalizeTopologyCandidate(')
        self.assertIn('if (candidate.safety_class != CandidateSafetyClass::ABSOLUTE_SAFE)\n      return retain_previous', finalize)
        self.assertIn('? TopologyProcessStatus::RETAINED_PREVIOUS : TopologyProcessStatus::FAILED;', finalize)
        self.assertIn('validateRetimedLocalSfc(candidate,committed.getTraj())', finalize)
        self.assertIn('returnstatus==TopologyProcessStatus::COMMITTED||status==TopologyProcessStatus::RETAINED_PREVIOUS;', ''.join(self.manager.split()))

    def test_persistence_revalidates_actual_remaining_suffix(self):
        validate = body(self.manager, 'bool EGOPlannerManager::validatePreviousRemainingTrajectory(')
        self.assertIn('prediction_epoch - traj_.local_traj.start_time', validate)
        self.assertIn('sliceTrajectory(', validate)
        self.assertIn('validateExecutionTrajectory(remaining,prediction_epoch,touch_goal,reason)', validate)

    def test_joint_commit_uses_ack_payload_and_team_activation(self):
        callback = body(self.manager, 'void EGOPlannerManager::teamTrajectorySolutionCallback(')
        self.assertIn('ACK_PAYLOAD_CHANGED', callback)
        self.assertIn('message->yaw_samples_uav2==accepted.yaw_samples_uav2', callback)
        self.assertIn('team.activation_time, &team.solution)', self.manager)

    def test_planning_failure_cannot_publish_stop_trajectory(self):
        self.assertNotIn('validatedBrakeSuccessor', self.manager)
        self.assertNotIn('VALIDATED_BRAKE', self.manager)
        self.assertNotIn('EGOPlannerManager::EmergencyStop', self.manager)
        self.assertNotIn('callEmergencyStop', self.fsm)
        self.assertIn('changeFSMExecState(REPLAN_TRAJ, "SAFETY")',self.fsm)

    def test_one_soft_reference_and_local_first_safe_precede_team_quality(self):
        self.assertIn('std::vector<EGOPlannerManager::CandidateSetOutput> sets;', self.fsm)
        self.assertIn('sets.push_back(std::move(output));', self.fsm)
        self.assertLess(self.fsm.index('finalizeCapturedCandidates('),
                        self.fsm.index('stageCapturedTopologyCoordination('))
        self.assertNotIn('reserve_for_team_transaction', self.manager)
        view = (SRC / 'multi_uav_formation/src/cooperative_viewpoint_manager.cpp').read_text()
        self.assertIn('one thing only: the cooperative soft encirclement', view)
        self.assertIn('reference_pubs_[index].publish(reference)', view)
        self.assertNotIn('EGOPlannerManager', view)
        self.assertNotIn('AdaptiveViewpointGenerator', view)

    def test_source_identity_is_executed_and_scheduled_atomically(self):
        self.assertIn('!msg->safety_validated || msg->generation == 0', self.server)
        self.assertIn('scheduled_source_ = *msg', self.server)
        self.assertIn('active_source_ = scheduled_source_', self.server)
        self.assertIn('execution_adoption_pub.publish(active_source_)', self.server)
        self.assertIn('planner_manager_->annotateExecutionSource(poly_msg)', self.fsm)

    def test_rolling_geometry_recovery_reaches_trajectory_prefix(self):
        opt = (SRC / 'EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp').read_text()
        tracking = body(opt, 'bool PolyTrajOptimizer::trackingGradCostP(')
        for contract in ('encirclementGeometryCost(', 'trajectory_lifecycle::sample(peer.traj,elapsed)',
                         'gradp+=encirclement_local_gap_weight_',
                         'grad_prev_t+=encirclement_local_gap_weight_*explicit_time'):
            self.assertIn(contract, tracking)
        self.assertIn('encirclement_tracking_active_', tracking)

    def test_tracking_has_one_fixed_soft_geometry_regularizer_and_no_elastic_mode(self):
        optimizer = (PLANNER / 'traj_opt/src/poly_traj_optimizer.cpp').read_text()
        active = body(optimizer, 'if (encirclement_tracking_active_)')
        self.assertIn('observation_radius_band_', active)
        self.assertIn('observation_height_band_', active)
        self.assertIn('encirclement_base_angular_slack_', active)
        self.assertNotIn('ElasticRiskState', optimizer)
        self.assertNotIn('enable_elastic_tracking_region_', optimizer)

    def test_nonblocking_relative_recovery_wiring(self):
        finalize = body(self.manager, 'EGOPlannerManager::finalizeCapturedCandidates(')
        self.assertNotIn('PRESERVE_SAFE_ENCIRCLEMENT', finalize)
        self.assertLess(finalize.index('validateExecutionTrajectory('),finalize.index('geometryBetter('))
        self.assertIn('if(best)',finalize)
        self.assertIn('finalizeTopologyCandidate(*best',finalize)
        predictor=body(self.manager,'EGOPlannerManager::evaluateLocalGeometry(')
        self.assertIn('peerGeometryConfidence',predictor)
        self.assertNotIn('wait',predictor.replace('never waited for',''))
        joint=(SRC/'multi_uav_formation/src/team_target_centered_optimizer.cpp').read_text()
        self.assertIn('TeamTargetCenteredOptimizer::optimize(', joint)
        self.assertIn('TeamOptimizationStage::VISIBILITY_Q2', joint)
        self.assertIn('TeamOptimizationStage::VISIBILITY_ACC', joint)
        self.assertNotIn('measurableBenefit(', joint)
        self.assertNotIn('relativeEncirclementAcceptance(', joint)
        self.assertNotIn('encirclementAcceptance(', joint)
        selector=(SRC/'multi_uav_formation/src/topology_coordinator_core.cpp').read_text()
        self.assertNotIn('m.encirclement_ratio+1e-10<',selector)
        self.assertNotIn('if (!protected_coverage(metrics)) continue;',selector)

    def test_launch_contract(self):
        root = ET.parse(SRC / 'multi_uav_formation/launch/native_egov2_rviz.launch').getroot()
        args = {n.attrib['name']: n.attrib.get('default') for n in root.findall('arg')}
        self.assertEqual(int(args['team_reference_knot_count']), 4)
        self.assertEqual(int(args['team_top_k']), 3)
        self.assertGreater(float(args['team_objective_k2_weight']),
                           float(args['team_objective_acc_weight']))
        self.assertGreater(float(args['team_bearing_trust']), 0.0)
        self.assertGreater(float(args['team_phase_trust']), 0.0)
        self.assertNotIn('team_position_trust_radius', args)
        self.assertNotIn('team_time_trust_ratio', args)
        self.assertNotIn('team_k2_continuity_weight', args)
        self.assertNotIn('team_blackout_weight', args)
        self.assertNotIn('adaptive_far_horizon', args)


if __name__ == '__main__':
    unittest.main(verbosity=2)
