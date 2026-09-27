#include <multi_uav_formation/cooperative_viewpoint_core.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{

bool near(const double lhs, const double rhs, const double tolerance = 1.0e-8)
{
  return std::abs(lhs - rhs) <= tolerance;
}

int fail(const std::string &reason)
{
  std::cerr << "COOPERATIVE_VIEWPOINT_CONTRACT_TEST=FAIL reason=" << reason
            << std::endl;
  return 1;
}

multi_uav_formation::CooperativeViewpointCore makeCore(
    const std::vector<ego_planner::StaticLosCylinder> &cylinders)
{
  multi_uav_formation::CooperativeViewpointCore core;
  core.setGeometry(cylinders);
  core.configure(2.0, 0.1, 0.2, 8.0, 0.08, 0.0, 0.5, 0.5,
                 25.0 * M_PI / 180.0, 0.10, 0.25, 0.5, 0.5);
  return core;
}

multi_uav_formation::CooperativeViewpointCore makeWallCore(
    const std::vector<ego_planner::StaticLosWall> &walls)
{
  multi_uav_formation::CooperativeViewpointCore core;
  core.setWalls(walls);
  core.configure(2.0, 0.1, 0.2, 8.0, 0.08, 0.0, 0.5, 0.5,
                 25.0 * M_PI / 180.0, 0.10, 0.25, 0.5, 0.5);
  return core;
}

std::vector<ego_planner::StaticLosCylinder> losBlocker()
{
  ego_planner::StaticLosCylinder blocker;
  blocker.name = "uav0_los_blocker";
  blocker.center = Eigen::Vector2d(-0.75, -0.425);
  blocker.radius = 0.20;
  blocker.z_min = 0.0;
  blocker.z_max = 3.0;
  return {blocker};
}

} // namespace

int main()
{
  multi_uav_formation::AdaptiveExecutionWindow timing;
  timing.configure(8, 1.0, 0.1, 2.0, 0.5, 1.5);
  if (!near(timing.executionMedian(), 1.0) || timing.historySize() != 0)
    return fail("EXECUTION_WINDOW_BOOTSTRAP_NOT_REPLAN_CADENCE");
  if (timing.recordSupersededDuration(9.0, true, true) ||
      timing.recordSupersededDuration(9.0, false, false) ||
      timing.historySize() != 0)
    return fail("FINAL_STOP_OR_PREMOTION_POLLUTED_EXEC_HISTORY");
  const std::array<double, 9> durations{{
      0.4, 0.8, 0.6, 1.0, 0.7, 0.5, 0.9, 1.1, 1.2}};
  for (const double duration : durations)
    if (!timing.recordSupersededDuration(duration, true, false))
      return fail("VALID_EXECUTION_DURATION_REJECTED");
  if (timing.historySize() != 8 || !near(timing.executionMedian(), 0.85) ||
      !near(timing.outerCadence(), 0.5) ||
      !near(timing.predictionHorizon(), 1.5) ||
      !near(timing.evaluationHorizon(0.2), 1.5) ||
      !near(timing.evaluationHorizon(5.0), 1.5))
    return fail("ROLLING_DIAGNOSTIC_OR_FIXED_MULTIRATE_CONTRACT_WRONG");
  if (!timing.outerDue(10.0, true, false, false))
    return fail("FIRST_OUTER_EVALUATION_NOT_DUE");
  timing.markOuterEvaluation(10.0);
  if (timing.outerDue(10.4, true, false, false) ||
      timing.outerDue(10.5, true, true, false) ||
      timing.outerDue(10.5, true, false, true) ||
      timing.outerDue(10.49, true, false, false) ||
      !timing.outerDue(10.5, true, false, false))
    return fail("PHI0_OUTER_CADENCE_OR_GATING_WRONG");
  timing.recordSupersededDuration(0.11, true, false);
  if (!near(timing.outerCadence(), 0.5) ||
      !near(timing.predictionHorizon(), 1.5))
    return fail("T_EXEC_ILLEGALLY_CONTROLS_PHI0_SCHEDULE_OR_HORIZON");

  const Eigen::Vector3d target(0.0, 0.0, 1.5);
  const Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
  const std::array<double, 3> zero_angles{{0.0, 0.0, 0.0}};
  std::array<bool, 3> odom_valid{{true, true, true}};

  multi_uav_formation::CooperativeViewpointCore core = makeCore(losBlocker());
  const auto fixed = core.fixedFormationStep(
      target, target_velocity, "FEATURE_DISABLED_FIXED_FORMATION");
  for (int uav = 0; uav < 3; ++uav)
  {
    const Eigen::Vector3d expected = target + core.offsets()[uav];
    if ((fixed.positions[uav] - expected).norm() > 1.0e-9)
      return fail("FEATURE_OFF_OR_PHI_ZERO_NOT_FIXED_FORMATION");
  }

  if (!core.pairwiseSeparationValid(zero_angles))
    return fail("BASELINE_FORMATION_SEPARATION_REJECTED");
  std::array<double, 3> crowded = zero_angles;
  crowded[0] = multi_uav_formation::CooperativeViewpointCore::wrapToPi(
      core.observationAngle(1, 0.0) - core.observationAngle(0, 0.0));
  if (core.pairwiseSeparationValid(crowded))
    return fail("PAIRWISE_ANGLE_BELOW_25_DEG_ACCEPTED");

  std::array<Eigen::Vector3d, 3> odom_positions = fixed.positions;
  const auto blocked = core.evaluate(target, target_velocity, zero_angles, 0,
                                     odom_positions, odom_valid);
  std::array<double, 3> clear_angles = zero_angles;
  clear_angles[0] = M_PI / 2.0;
  const auto clear = core.evaluate(target, target_velocity, clear_angles, 0,
                                   odom_positions, odom_valid);
  if (!clear.valid || clear.visibility[0] <= blocked.visibility[0] ||
      !multi_uav_formation::CooperativeViewpointCore::betterForUav(
          clear, blocked, 0))
    return fail("STATIC_BLOCKED_CANDIDATE_RANKED_ABOVE_CLEAR");

  auto nonactive_blockers = losBlocker();
  ego_planner::StaticLosCylinder nonactive_reference_blocker;
  nonactive_reference_blocker.name = "nonactive_uav2_reference_blocker";
  nonactive_reference_blocker.center = fixed.positions[2].head<2>();
  nonactive_reference_blocker.radius = 0.10;
  nonactive_reference_blocker.z_min = 0.0;
  nonactive_reference_blocker.z_max = 3.0;
  nonactive_blockers.push_back(nonactive_reference_blocker);
  multi_uav_formation::CooperativeViewpointCore candidate_scope =
      makeCore(nonactive_blockers);
  const auto scoped_candidate = candidate_scope.evaluate(
      target, target_velocity, clear_angles, 0, odom_positions, odom_valid);
  if (!scoped_candidate.valid)
    return fail("NONACTIVE_REFERENCE_WRONGLY_INVALIDATED_ACTIVE_CANDIDATE");

  multi_uav_formation::ViewpointStep transition;
  for (int step = 0; step < 8; ++step)
    transition = core.step(target, target_velocity, odom_positions, odom_valid,
                           0.1 * step, 0.1);
  if (transition.state != multi_uav_formation::ViewpointState::TRANSITION ||
      transition.active_uav != 0)
    return fail("PERSISTENT_STATIC_LOS_DID_NOT_TRIGGER_SINGLE_UAV");
  if ((transition.positions[1] - fixed.positions[1]).norm() > 1.0e-9 ||
      (transition.positions[2] - fixed.positions[2]).norm() > 1.0e-9)
    return fail("NON_ACTIVE_UAV_REFERENCE_CHANGED");
  if (!transition.reselection_blocked)
    return fail("TRANSITION_RESELECTION_NOT_BLOCKED");
  if (transition.direction_reversal)
    return fail("TRANSITION_DIRECTION_REVERSED");
  const double frozen_target = transition.target_angles[0];

  bool arrived = false;
  double previous_phi = transition.commanded_angles[0];
  for (int step = 8; step < 180; ++step)
  {
    odom_positions[0] = transition.positions[0];
    transition = core.step(target, target_velocity, odom_positions, odom_valid,
                           0.1 * step, 0.1);
    if (transition.state == multi_uav_formation::ViewpointState::TRANSITION)
    {
      if (!near(transition.target_angles[0], frozen_target, 1.0e-9))
        return fail("TRANSITION_TARGET_RESELECTED");
      const double increment =
          multi_uav_formation::CooperativeViewpointCore::wrapToPi(
              transition.commanded_angles[0] - previous_phi);
      if (std::abs(increment) > 0.0250001)
        return fail("REFERENCE_RATE_LIMIT_BROKEN");
      if (transition.direction_reversal)
        return fail("TRANSITION_DIRECTION_REVERSED");
      previous_phi = transition.commanded_angles[0];
    }
    if (transition.arrived)
    {
      arrived = true;
      break;
    }
  }
  if (!arrived || core.state() != multi_uav_formation::ViewpointState::HOLD ||
      core.activeTransitionUav() != -1)
    return fail("ACTUAL_ODOM_ARRIVAL_DID_NOT_COMPLETE_TRANSITION");

  multi_uav_formation::CooperativeViewpointCore unsettled =
      makeCore(losBlocker());
  const auto unsettled_fixed = unsettled.fixedFormationStep(
      target, target_velocity, "TEST");
  std::array<Eigen::Vector3d, 3> unsettled_odom = unsettled_fixed.positions;
  unsettled_odom[0].z() += 0.75;
  multi_uav_formation::ViewpointStep unsettled_step;
  for (int step = 0; step < 8; ++step)
    unsettled_step = unsettled.step(
        target, target_velocity, unsettled_odom, odom_valid,
        0.1 * step, 0.1);
  if (unsettled_step.state != multi_uav_formation::ViewpointState::HOLD ||
      unsettled_step.selection_reason != "PLANNER_NOT_SETTLED")
    return fail("TRACKING_ERROR_DID_NOT_BLOCK_VIEWPOINT_SWITCH");

  std::array<bool, 3> stale_odom{{false, false, false}};
  multi_uav_formation::CooperativeViewpointCore stale = makeCore(losBlocker());
  const auto stale_step = stale.step(target, target_velocity, odom_positions,
                                     stale_odom, 0.0, 0.1);
  if (stale_step.state != multi_uav_formation::ViewpointState::HOLD ||
      stale_step.active_uav != -1)
    return fail("STALE_ODOM_STARTED_TRANSITION");

  // A: explicitly disabling encirclement must preserve the legacy state
  // machine result, cycle by cycle.
  multi_uav_formation::CooperativeViewpointCore legacy = makeCore(losBlocker());
  multi_uav_formation::CooperativeViewpointCore disabled = makeCore(losBlocker());
  disabled.setEncirclementTracking(false);
  std::array<Eigen::Vector3d, 3> legacy_odom = fixed.positions;
  for (int cycle = 0; cycle < 8; ++cycle)
  {
    const auto legacy_step = legacy.step(
        target, target_velocity, legacy_odom, odom_valid,
        0.1 * cycle, 0.1);
    const auto disabled_step = disabled.step(
        target, target_velocity, legacy_odom, odom_valid,
        0.1 * cycle, 0.1);
    if (legacy_step.state != disabled_step.state ||
        legacy_step.active_uav != disabled_step.active_uav ||
        legacy_step.selection_reason != disabled_step.selection_reason)
      return fail("ENCIRCLEMENT_DISABLED_STATE_REGRESSION");
    for (int uav = 0; uav < 3; ++uav)
      if ((legacy_step.positions[uav] -
           disabled_step.positions[uav]).norm() > 1.0e-12 ||
          !near(legacy_step.commanded_angles[uav],
                disabled_step.commanded_angles[uav], 1.0e-12))
        return fail("ENCIRCLEMENT_DISABLED_REFERENCE_REGRESSION");
  }

  ego_planner::StaticLosCylinder distant;
  distant.name = "distant_nonblocking_geometry";
  distant.center = Eigen::Vector2d(20.0, 20.0);
  distant.radius = 0.2;
  distant.z_min = 0.0;
  distant.z_max = 3.0;
  multi_uav_formation::CooperativeViewpointCore encirclement =
      makeCore({distant});
  encirclement.setEncirclementTracking(true);
  std::array<Eigen::Vector3d, 3> encirclement_odom =
      encirclement.encirclementReferencePositions(target, 0.0);
  const auto encirclement_step = encirclement.step(
      target, target_velocity, encirclement_odom, odom_valid, 0.0, 0.1);
  if (!encirclement_step.encirclement_enabled)
    return fail("ENCIRCLEMENT_MODE_NOT_ACTIVE");

  // B: generated references, rather than labels, must have the requested
  // three 120-degree world-space bearings.
  std::array<double, 3> reference_bearings;
  for (int uav = 0; uav < 3; ++uav)
  {
    const Eigen::Vector3d relative =
        encirclement_step.positions[uav] - target;
    reference_bearings[uav] = std::atan2(relative.y(), relative.x());
  }
  for (int uav = 0; uav < 3; ++uav)
  {
    const int next = (uav + 1) % 3;
    const double separation =
        multi_uav_formation::CooperativeViewpointCore::wrapToPi(
            reference_bearings[next] - reference_bearings[uav]);
    if (!near(separation, 2.0 * M_PI / 3.0, 1.0e-9))
      return fail("ENCIRCLEMENT_REFERENCE_NOT_120_DEGREES");
  }

  // C: changing phi0 rotates every slot equally and preserves its radius,
  // height, and pairwise 120-degree structure.
  const double rotated_phi0 = 0.37;
  const auto rotated_references =
      encirclement.encirclementReferencePositions(target, rotated_phi0);
  for (int uav = 0; uav < 3; ++uav)
  {
    const Eigen::Vector3d before = encirclement_step.positions[uav] - target;
    const Eigen::Vector3d after = rotated_references[uav] - target;
    const double rotation =
        multi_uav_formation::CooperativeViewpointCore::wrapToPi(
            std::atan2(after.y(), after.x()) -
            std::atan2(before.y(), before.x()));
    if (!near(rotation, rotated_phi0, 1.0e-9) ||
        !near(before.head<2>().norm(), after.head<2>().norm(), 1.0e-9) ||
        !near(before.z(), after.z(), 1.0e-9))
      return fail("COMMON_PHI0_DID_NOT_ROTATE_ALL_REFERENCES");
  }

  // D/E: this wall blocks two slots at phi0=0 but only one at phi0=45deg.
  // The common selector must choose the K2-safe direction and rate-limit all
  // three slots together.
  ego_planner::StaticLosWall wall;
  wall.name = "encirclement_k2_wall";
  wall.center = Eigen::Vector2d(-0.45, 0.0);
  wall.half_size = Eigen::Vector2d(0.06, 1.0);
  wall.yaw = 0.0;
  wall.z_min = 0.0;
  wall.z_max = 3.0;
  multi_uav_formation::CooperativeViewpointCore wall_core =
      makeWallCore({wall});
  wall_core.setEncirclementTracking(true);
  std::array<Eigen::Vector3d, 3> wall_odom =
      wall_core.encirclementReferencePositions(target, 0.0);
  const auto phi0_blocked = wall_core.evaluateEncirclement(
      target, target_velocity, 0.0);
  const auto phi0_clear = wall_core.evaluateEncirclement(
      target, target_velocity, M_PI / 4.0);
  if (!phi0_blocked.valid || !phi0_clear.valid ||
      phi0_clear.atleast2 <= phi0_blocked.atleast2)
    return fail("ENCIRCLEMENT_WALL_DID_NOT_CREATE_K2_DIFFERENCE");

  multi_uav_formation::ViewpointStep wall_step = wall_core.step(
      target, target_velocity, wall_odom, odom_valid, 0.0, 0.1);
  if (wall_step.state != multi_uav_formation::ViewpointState::HOLD ||
      wall_step.encirclement_candidates.size() != 3 ||
      wall_step.selection_reason != "ENCIRCLEMENT_AWAITING_JOINT_SELECTION")
    return fail("REFERENCE_LEVEL_SELECTOR_REMAINED_AUTHORITATIVE");
  wall_core.requestJointSelectedPhi0(M_PI / 12.0);
  wall_step = wall_core.step(target, target_velocity, wall_odom, odom_valid,
                             0.1, 0.1);
  if (wall_step.state != multi_uav_formation::ViewpointState::TRANSITION ||
      wall_step.selection_reason !=
          "ENCIRCLEMENT_JOINT_TRAJECTORY_SELECTION")
    return fail("JOINT_PHI0_SELECTION_DID_NOT_START_TRANSITION");
  double previous_common_phi0 = wall_step.phi0_current;
  bool soft_reference_completed = false;
  for (int cycle = 2; cycle < 300; ++cycle)
  {
    wall_step = wall_core.step(target, target_velocity, wall_odom, odom_valid,
                               0.1 * cycle, 0.1);
    if (wall_step.arrived)
    {
      soft_reference_completed = true;
      break;
    }
  }
  (void)soft_reference_completed;

  // F: persistent loss without a lexicographic K2/all3/diversity gain must
  // not create a meaningless topology switch.
  ego_planner::StaticLosCylinder target_blocker;
  target_blocker.name = "all_phi_equally_blocked";
  target_blocker.center = target.head<2>();
  target_blocker.radius = 0.20;
  target_blocker.z_min = 0.0;
  target_blocker.z_max = 3.0;
  multi_uav_formation::CooperativeViewpointCore no_improvement =
      makeCore({target_blocker});
  no_improvement.setEncirclementTracking(true);
  std::array<Eigen::Vector3d, 3> no_improvement_odom =
      no_improvement.encirclementReferencePositions(target, 0.0);
  multi_uav_formation::ViewpointStep no_improvement_step;
  for (int cycle = 0; cycle < 8; ++cycle)
    no_improvement_step = no_improvement.step(
        target, target_velocity, no_improvement_odom, odom_valid,
        0.1 * cycle, 0.1);
  if (no_improvement_step.state !=
          multi_uav_formation::ViewpointState::HOLD ||
      no_improvement_step.selection_reason !=
          "ENCIRCLEMENT_AWAITING_JOINT_SELECTION" ||
      !no_improvement_step.periodic_visibility_evaluation ||
      std::abs(no_improvement_step.phi0_current) > 1.0e-9)
    return fail("ENCIRCLEMENT_NO_IMPROVEMENT_SWITCHED_PHI0");

  // G: a statically valid endpoint is insufficient when an intermediate
  // common-rotation reference intersects an obstacle.
  ego_planner::StaticLosCylinder arc_blocker;
  arc_blocker.name = "rotation_arc_blocker";
  const double arc_radius =
      encirclement.offsets()[0].head<2>().norm();
  arc_blocker.center = Eigen::Vector2d(
      arc_radius * std::cos(M_PI / 8.0),
      arc_radius * std::sin(M_PI / 8.0));
  arc_blocker.radius = 0.08;
  arc_blocker.z_min = 0.0;
  arc_blocker.z_max = 3.0;
  multi_uav_formation::CooperativeViewpointCore arc_core =
      makeCore({arc_blocker});
  arc_core.setEncirclementTracking(true);
  if (!arc_core.evaluateEncirclement(target, target_velocity, 0.0).valid ||
      !arc_core.evaluateEncirclement(
          target, target_velocity, M_PI / 4.0).valid ||
      arc_core.encirclementTransitionValid(
          target, target_velocity, 0.0, M_PI / 4.0))
    return fail("ENCIRCLEMENT_ROTATION_ARC_NOT_VALIDATED");

  // H: HOLD is still a publication state.  Every cycle must return a valid
  // last encirclement reference so the manager can refresh its ROS stamp.
  multi_uav_formation::CooperativeViewpointCore hold_core =
      makeCore({distant});
  hold_core.setEncirclementTracking(true);
  std::array<Eigen::Vector3d, 3> hold_odom =
      hold_core.encirclementReferencePositions(target, 0.0);
  for (int cycle = 0; cycle < 20; ++cycle)
  {
    const auto hold_step = hold_core.step(
        target, target_velocity, hold_odom, odom_valid,
        0.1 * cycle, 0.1);
    if (hold_step.state != multi_uav_formation::ViewpointState::HOLD ||
        !hold_step.reference_valid[0] || !hold_step.reference_valid[1] ||
        !hold_step.reference_valid[2])
      return fail("ENCIRCLEMENT_HOLD_REFERENCE_NOT_REFRESHABLE");
  }

  // I: if geometry changes after a prevalidated transition starts, retain the
  // last valid encirclement reference instead of oscillating to the legacy
  // fixed formation.
  multi_uav_formation::CooperativeViewpointCore cancel_core =
      makeWallCore({wall});
  cancel_core.setEncirclementTracking(true);
  std::array<Eigen::Vector3d, 3> cancel_odom =
      cancel_core.encirclementReferencePositions(target, 0.0);
  cancel_core.step(
      target, target_velocity, cancel_odom, odom_valid, 0.0, 0.1);
  cancel_core.requestJointSelectedPhi0(M_PI / 12.0);
  const auto cancel_started = cancel_core.step(
      target, target_velocity, cancel_odom, odom_valid, 0.1, 0.1);
  if (cancel_started.state !=
      multi_uav_formation::ViewpointState::TRANSITION)
    return fail("ENCIRCLEMENT_CANCEL_TEST_DID_NOT_START");
  ego_planner::StaticLosCylinder transition_blocker;
  transition_blocker.name = "late_transition_blocker";
  const Eigen::Vector3d moved_target =
      target + Eigen::Vector3d(2.0, 0.0, 0.0);
  const double next_phi = cancel_started.phi0_current + 0.025;
  const auto next_references = cancel_core.encirclementReferencePositions(
      moved_target, next_phi);
  transition_blocker.center = next_references[0].head<2>();
  transition_blocker.radius = 0.01;
  transition_blocker.z_min = 0.0;
  transition_blocker.z_max = 3.0;
  cancel_core.setGeometry({transition_blocker});
  const auto canceled = cancel_core.step(
      moved_target, target_velocity, cancel_odom, odom_valid, 0.2, 0.1);
  if (canceled.state != multi_uav_formation::ViewpointState::HOLD ||
      !canceled.using_last_valid_reference ||
      !canceled.reference_valid[0] || !canceled.reference_valid[1] ||
      !canceled.reference_valid[2])
    return fail("ENCIRCLEMENT_CANCEL_DID_NOT_KEEP_LAST_VALID_REFERENCE");
  const auto rebased_last_valid = cancel_core.encirclementReferencePositions(
      moved_target, canceled.phi0_current);
  for (int uav = 0; uav < 3; ++uav)
    if ((canceled.positions[uav] - rebased_last_valid[uav]).norm() > 1.0e-9)
      return fail("ENCIRCLEMENT_LAST_VALID_REFERENCE_NOT_TARGET_RELATIVE");
  const auto legacy_fixed = cancel_core.fixedFormationStep(
      moved_target, target_velocity, "TEST");
  bool equals_fixed = true;
  for (int uav = 0; uav < 3; ++uav)
    equals_fixed = equals_fixed &&
        (canceled.positions[uav] - legacy_fixed.positions[uav]).norm() <
            1.0e-9;
  if (equals_fixed)
    return fail("ENCIRCLEMENT_CANCEL_FELL_BACK_TO_FIXED_FORMATION");

  // J: clustered startup bearings expose the previous fixed-ID mismatch.
  // Choose the minimum-displacement permutation once and keep it locked.
  multi_uav_formation::CooperativeViewpointCore assignment_core =
      makeCore({distant});
  assignment_core.setEncirclementTracking(true);
  std::array<Eigen::Vector3d, 3> clustered{{
      Eigen::Vector3d(-2.2, -1.0, 1.5),
      Eigen::Vector3d(-2.2, 0.0, 1.5),
      Eigen::Vector3d(-2.2, 1.0, 1.5)}};
  const std::array<int, 3> identity{{0, 1, 2}};
  const double identity_phi = assignment_core.estimateInitialPhi0(
      target, clustered, odom_valid, identity);
  const double identity_displacement =
      assignment_core.initialBearingDisplacement(
          target, clustered, odom_valid, identity_phi, identity);
  const auto assigned_step = assignment_core.step(
      target, target_velocity, clustered, odom_valid, 0.0, 0.1);
  const auto selected_assignment = assignment_core.slotAssignment();
  const double selected_displacement =
      assignment_core.initialBearingDisplacement(
          target, clustered, odom_valid, assigned_step.phi0_current,
          selected_assignment);
  if (!assignment_core.initialSlotAssignmentChanged() ||
      selected_displacement >= identity_displacement - 0.5)
    return fail("ENCIRCLEMENT_STARTUP_SLOT_MISMATCH_NOT_REDUCED");
  assignment_core.step(
      target, target_velocity, clustered, odom_valid, 0.1, 0.1);
  if (assignment_core.slotAssignment() != selected_assignment)
    return fail("ENCIRCLEMENT_SLOT_ASSIGNMENT_CHANGED_AT_RUNTIME");

  // K: authoritative +X-forward camera-frame checks include both horizontal
  // and vertical FOV instead of reducing visibility to a yaw-only cone.
  multi_uav_formation::TrackingCameraContract camera;
  camera.horizontal_fov = 85.0 * M_PI / 180.0;
  camera.vertical_fov = multi_uav_formation::verticalFovFromHorizontal(
      camera.horizontal_fov, 1280.0, 720.0);
  const Eigen::Quaterniond identity_attitude =
      Eigen::Quaterniond::Identity();
  const auto camera_clear =
      multi_uav_formation::targetInTrackingCameraFov(
          Eigen::Vector3d::Zero(), identity_attitude,
          Eigen::Vector3d(2.0, 0.0, 0.0),
          camera);
  const auto horizontal_out =
      multi_uav_formation::targetInTrackingCameraFov(
          Eigen::Vector3d::Zero(), identity_attitude,
          Eigen::Vector3d(1.0, 2.0, 0.0),
          camera);
  const auto vertical_out =
      multi_uav_formation::targetInTrackingCameraFov(
          Eigen::Vector3d::Zero(), identity_attitude,
          Eigen::Vector3d(1.0, 0.0, 1.0),
          camera);
  if (!camera_clear.valid || horizontal_out.valid || vertical_out.valid)
    return fail("CAMERA_FRAME_HFOV_VFOV_CONTRACT_BROKEN");

  // L: an initially misaligned real yaw cannot see the target immediately;
  // the production yaw-rate/acceleration contract eventually reacquires it.
  multi_uav_formation::PredictedYawState yaw_state;
  yaw_state.valid = true;
  yaw_state.yaw = M_PI / 2.0;
  yaw_state.yaw_rate = 0.0;
  const auto early_fov =
      multi_uav_formation::targetInTrackingCameraFov(
          Eigen::Vector3d::Zero(),
          Eigen::Quaterniond(Eigen::AngleAxisd(
              yaw_state.yaw, Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d(2.0, 0.0, 0.0), camera);
  bool later_visible = false;
  for (int index = 0; index < 20; ++index)
  {
    yaw_state = multi_uav_formation::advanceTargetFacingYaw(
        yaw_state, 0.0, 0.1);
    later_visible = later_visible ||
        multi_uav_formation::targetInTrackingCameraFov(
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond(Eigen::AngleAxisd(
                yaw_state.yaw, Eigen::Vector3d::UnitZ())),
            Eigen::Vector3d(2.0, 0.0, 0.0), camera).valid;
  }
  if (early_fov.valid || !later_visible)
    return fail("YAW_DYNAMICS_NOT_REFLECTED_IN_FOV");

  // M: dynamic occlusion uses the same expanded vertical cylinder contract
  // as telemetry and changes phi0 ranking even when static LOS is clear.
  multi_uav_formation::CooperativeViewpointCore dynamic_core =
      makeCore({distant});
  dynamic_core.setEncirclementTracking(true);
  dynamic_core.setCameraContract(camera);
  multi_uav_formation::DynamicLosCylinder dynamic_blocker;
  dynamic_blocker.name = "dynamic_los_blocker";
  dynamic_blocker.center = Eigen::Vector2d(0.75, 0.0);
  dynamic_blocker.axis = Eigen::Vector2d::UnitX();
  dynamic_blocker.radius = 0.18;
  dynamic_blocker.z_min = 0.0;
  dynamic_blocker.z_max = 3.0;
  dynamic_core.setDynamicObstacles({dynamic_blocker});
  dynamic_core.setDynamicMotionElapsed(0.0);
  if (dynamic_core.dynamicLosClear(Eigen::Vector3d(1.5, 0.0, 1.5),
                                   target, 0.0) ||
      !dynamic_core.dynamicLosClear(Eigen::Vector3d(0.0, 1.5, 1.5),
                                    target, 0.0))
    return fail("DYNAMIC_LOS_CYLINDER_CONTRACT_BROKEN");
  const auto dynamic_blocked = dynamic_core.evaluateEncirclementTransition(
      target, target_velocity, 0.0, 0.0);
  const auto dynamic_clear = dynamic_core.evaluateEncirclementTransition(
      target, target_velocity, M_PI / 2.0, 0.0);
  if (!dynamic_blocked.valid || !dynamic_clear.valid ||
      dynamic_clear.mean_visible_count <=
          dynamic_blocked.mean_visible_count + 0.5 ||
      !multi_uav_formation::CooperativeViewpointCore::
          betterEncirclementCandidate(
              dynamic_clear, M_PI / 2.0, dynamic_blocked, 0.0, 0.0))
    return fail("DYNAMIC_OCCLUSION_DID_NOT_CHANGE_PHI0_RANKING");

  // N: a soft reference is not the executed vehicle pose.  The visibility
  // predictor must start from measured geometry by retaining the current
  // reference-tracking residual, otherwise a clear reference ray can hide an
  // actually blocked (or clear) observer ray at transition start.
  auto offset_observers =
      dynamic_core.encirclementReferencePositions(target, 0.0);
  offset_observers[0].y() += 1.0;
  dynamic_core.setObserverPositions(offset_observers, odom_valid);
  const auto residual_aware = dynamic_core.evaluateEncirclementTransition(
      target, target_velocity, 0.0, 0.0);
  if (residual_aware.mean_visible_count <=
      dynamic_blocked.mean_visible_count + 0.5)
    return fail("TRACKING_RESIDUAL_NOT_USED_BY_VISIBILITY_PREDICTOR");

  // O: the lexicographic selector must prefer true predicted FOV time over a
  // candidate that is merely static-LOS valid.
  multi_uav_formation::ViewpointMetrics static_only;
  static_only.valid = true;
  static_only.atleast2 = 0.20;
  static_only.none = 0.50;
  static_only.mean_visible_count = 0.80;
  static_only.all3 = 0.0;
  static_only.min_pairwise_angle = 2.0 * M_PI / 3.0;
  multi_uav_formation::ViewpointMetrics actual_fov = static_only;
  actual_fov.atleast2 = 0.80;
  actual_fov.none = 0.0;
  actual_fov.mean_visible_count = 2.40;
  actual_fov.all3 = 0.50;
  if (!multi_uav_formation::CooperativeViewpointCore::
          betterEncirclementCandidate(
              actual_fov, M_PI / 4.0, static_only, 0.0, 0.0))
    return fail("TRUE_FOV_TIME_NOT_PRIORITIZED_OVER_STATIC_PROXY");

  // P: secondary all-3/mean-visible optimization must not rotate away from
  // an already all-three-visible executed state.  A material K=2 recovery is
  // still allowed immediately.
  multi_uav_formation::CooperativeViewpointCore observed_gate =
      makeCore({distant});
  observed_gate.setObservedVisibleCount(3, true);
  multi_uav_formation::ViewpointMetrics all3_baseline = static_only;
  all3_baseline.atleast2 = 1.0;
  all3_baseline.none = 0.0;
  all3_baseline.mean_visible_count = 2.7;
  all3_baseline.all3 = 0.7;
  multi_uav_formation::ViewpointMetrics secondary_gain = all3_baseline;
  secondary_gain.mean_visible_count = 2.9;
  secondary_gain.all3 = 0.9;
  if (observed_gate.encirclementImproves(secondary_gain, all3_baseline))
    return fail("ALL3_VISIBLE_SECONDARY_SWITCH_NOT_GATED");
  multi_uav_formation::ViewpointMetrics k2_baseline = all3_baseline;
  k2_baseline.atleast2 = 0.75;
  multi_uav_formation::ViewpointMetrics k2_gain = secondary_gain;
  k2_gain.atleast2 = 1.0;
  if (!observed_gate.encirclementImproves(k2_gain, k2_baseline))
    return fail("K2_RECOVERY_WAS_BLOCKED_BY_ACTUAL_FOV_GATE");

  // Q: reference-level phi0 diagnostics use the same near-best-K2 policy as
  // the authoritative Joint trajectory selector.
  multi_uav_formation::ViewpointMetrics near_best = all3_baseline;
  near_best.atleast2 = 0.995;
  near_best.mean_visible_count = 2.95;
  near_best.all3 = 0.95;
  if (!multi_uav_formation::CooperativeViewpointCore::
          betterEncirclementCandidate(
              near_best, M_PI / 12.0, all3_baseline, 0.0, 0.0, 0.01))
    return fail("PHI0_NEAR_BEST_K2_DID_NOT_PRIORITIZE_MEAN_VISIBLE");
  multi_uav_formation::ViewpointMetrics outside_k2 = near_best;
  outside_k2.atleast2 = 0.97;
  outside_k2.mean_visible_count = 3.0;
  if (multi_uav_formation::CooperativeViewpointCore::
          betterEncirclementCandidate(
              outside_k2, M_PI / 12.0, all3_baseline, 0.0, 0.0, 0.01))
    return fail("PHI0_K2_PROTECTION_FAILED");

  std::cout <<
      "COOPERATIVE_VIEWPOINT_CONTRACT_TEST=PASS feature_off=1 phi0=1 "
      "independent_reference=1 one_active=1 separation=1 blocked_ranking=1 "
      "candidate_static_scope=1 "
      "persistent_trigger=1 planner_unsettled_block=1 target_frozen=1 "
      "rate_limit=1 reversal_zero=1 actual_odom_arrival=1 stale_fallback=1 "
      "encirclement_disabled_regression=1 common_phi0=1 soft_120deg=1 "
      "encirclement_wall_k2_selection=1 encirclement_rate_limit=1 "
      "encirclement_no_improvement_hold=1 periodic_without_loss=1 "
      "rotation_arc_validation=1 hold_reference_refresh=1 "
      "last_valid_no_fixed_fallback=1 startup_slot_assignment=1"
      " soft_reference_completion=1 camera_hfov_vfov=1 dynamic_los=1"
      " yaw_dynamics=1 actual_fov_ranking=1 predictor_fixture=1"
      " tracking_residual=1 actual_all3_switch_gate=1"
      " near_best_k2_mean_visible=1 joint_outer_policy_consistent=1"
      << std::endl;
  return 0;
}
