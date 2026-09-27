#include <plan_manage/planner_manager.h>

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

using ego_planner::EGOPlannerManager;

namespace
{
int failures = 0;

void expect(const bool value, const std::string &name)
{
  std::cout << (value ? "PASS " : "FAIL ") << name << '\n';
  failures += !value;
}

poly_traj::MinJerkOpt stationary(const Eigen::Vector3d &position,
                                 const double duration)
{
  Eigen::Matrix3d head = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d tail = Eigen::Matrix3d::Zero();
  head.col(0) = position;
  tail.col(0) = position;
  poly_traj::MinJerkOpt optimizer;
  optimizer.reset(head, tail, 2);
  Eigen::MatrixXd middle(3, 1);
  middle.col(0) = position;
  optimizer.generate(middle, Eigen::Vector2d::Constant(0.5 * duration));
  return optimizer;
}

std::string fixtureScene()
{
  std::string source(__FILE__);
  const size_t slash = source.find_last_of('/');
  return source.substr(0, slash + 1) + "visibility_topology_fixture_scene.json";
}

void configure(ros::NodeHandle &node, const int drone_id)
{
  node.setParam("manager/max_vel", 3.0);
  node.setParam("manager/max_acc", 6.0);
  node.setParam("manager/max_jer", 20.0);
  node.setParam("manager/polyTraj_piece_length", 1.0);
  node.setParam("manager/drone_id", drone_id);
  node.setParam("manager/enable_risk_triggered_candidates", true);
  node.setParam("manager/enable_side_local_astar_repair", true);
  node.setParam("manager/enable_candidate_region_constraint", true);
  node.setParam("manager/risk_sample_dt", 0.05);
  node.setParam("manager/visibility_sample_dt", 0.05);

  node.setParam("optimization/max_vel", 3.0);
  node.setParam("optimization/max_acc", 6.0);
  node.setParam("optimization/max_jer", 20.0);
  node.setParam("optimization/constraint_points_perPiece", 5);
  node.setParam("optimization/weight_obstacle", 10000.0);
  node.setParam("optimization/weight_obstacle_soft", 5000.0);
  node.setParam("optimization/weight_swarm", 10000.0);
  node.setParam("optimization/weight_tracking", 100.0);
  node.setParam("optimization/weight_feasibility", 10000.0);
  node.setParam("optimization/weight_sqrvariance", 10000.0);
  node.setParam("optimization/weight_time", 10.0);
  node.setParam("optimization/obstacle_clearance", 0.10);
  node.setParam("optimization/obstacle_clearance_soft", 0.50);
  node.setParam("optimization/swarm_clearance", 0.50);
  node.setParam("optimization/moving_obj_clearance", 0.50);
  node.setParam("optimization/moving_obj_lambda", 10000.0);
  node.setParam("optimization/enable_encirclement_tracking", true);
  node.setParam("optimization/weight_visibility", 20.0);
  node.setParam("optimization/static_los_margin", 0.08);
  node.setParam("optimization/static_los_scene_file", fixtureScene());
  node.setParam("optimization/use_fov_costs", false);
  node.setParam("optimization/use_fov_tracking", false);

  node.setParam("prediction/obj_num", 1);
  node.setParam("prediction/use_time_aware_moving_obj_cost", true);

  node.setParam("grid_map/resolution", 0.1);
  node.setParam("grid_map/map_size_x", 12.0);
  node.setParam("grid_map/map_size_y", 12.0);
  node.setParam("grid_map/map_size_z", 4.0);
  node.setParam("grid_map/ground_height", 0.0);
  node.setParam("grid_map/virtual_ceil_height", 4.0);
  node.setParam("grid_map/obstacles_inflation", 0.0);
}

void seedKnownFree(EGOPlannerManager &manager)
{
  for (double x = -5.0; x <= 5.0; x += 0.1)
    for (double y = -5.0; y <= 5.0; y += 0.1)
      for (double z = 0.2; z <= 2.5; z += 0.1)
        manager.grid_map_->setFree(Eigen::Vector3d(x, y, z));
}

void publishFarPrediction(ros::NodeHandle &root, const double epoch)
{
  ros::Publisher prediction =
      root.advertise<nav_msgs::Path>("/dynamic/prediction_0", 1, true);
  nav_msgs::Path path;
  for (const double dt : {0.0, 10.0})
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(epoch + dt);
    pose.pose.position.x = 20.0;
    pose.pose.position.y = 20.0;
    pose.pose.position.z = 1.2;
    path.poses.push_back(pose);
  }
  prediction.publish(path);
  for (int i = 0; i < 20; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
}

void publishConstantPrediction(ros::NodeHandle &root, const double epoch,
                                const Eigen::Vector3d &center)
{
  ros::Publisher prediction =
      root.advertise<nav_msgs::Path>("/dynamic/prediction_0", 1, true);
  nav_msgs::Path path;
  for (const double dt : {0.0, 10.0})
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(epoch + dt);
    pose.pose.position.x = center.x();
    pose.pose.position.y = center.y();
    pose.pose.position.z = center.z();
    path.poses.push_back(pose);
  }
  prediction.publish(path);
  for (int i = 0; i < 20; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
}

struct CaseResult
{
  bool generated{false};
  bool trigger{false};
  bool plus{false};
  bool minus{false};
  bool side_executable{false};
  bool astar_rescue_executable{false};
  bool committed{false};
  EGOPlannerManager::CandidateKind selected{
      EGOPlannerManager::CandidateKind::NOMINAL};
  double predicted_loss{0.0};
  double trigger_time{0.0};
  double activation_time{0.0};
};

struct MixedConflictResult
{
  bool generated{false};
  unsigned int reason_mask{EGOPlannerManager::CONFLICT_NONE};
  int primary_id{-1};
  int body_id{-1};
  int los_id{-1};
  EGOPlannerManager::ConflictObstacleMotion body_motion{
      EGOPlannerManager::ConflictObstacleMotion::UNKNOWN};
  EGOPlannerManager::ConflictObstacleMotion los_motion{
      EGOPlannerManager::ConflictObstacleMotion::UNKNOWN};
  double body_time{-1.0};
  double los_time{-1.0};
  double side_anchor{-1.0};
};

CaseResult runMirrorCase(ros::NodeHandle &root, const int index,
                         const double observer_x,
                         const bool block_simple_minus = false)
{
  ros::NodeHandle node("/visibility_topology/uav" + std::to_string(index));
  configure(node, index);
  std::unique_ptr<EGOPlannerManager> manager(new EGOPlannerManager);
  manager->initPlanModules(
      node, ego_planner::PlanningVisualization::Ptr(
                new ego_planner::PlanningVisualization(node)));
  seedKnownFree(*manager);

  // The path-frame minus-side simple profile for the x=-2 case passes
  // through this compact block, while a same-side channel remains around
  // its outside.  The observation-frame seed may fail first and legitimately
  // fall back to the path frame; the block therefore exercises the actual
  // body-path collision -> same-side A* -> local-SFC chain rather than the
  // observation half-space itself.
  if (block_simple_minus)
    for (double x = -2.20; x <= -1.80; x += 0.1)
      for (double y = -2.35; y <= -2.10; y += 0.1)
        for (double z = 0.3; z <= 2.2; z += 0.1)
          manager->grid_map_->setOccupied(Eigen::Vector3d(x, y, z));
  const double now = ros::Time::now().toSec();
  const Eigen::Vector3d observer(observer_x, -2.0, 1.2);
  manager->traj_.local_traj.traj = stationary(observer, 10.0).getTraj();
  manager->traj_.local_traj.traj_id = 41 + index;
  manager->traj_.local_traj.drone_id = index;
  manager->traj_.local_traj.start_time = now;
  manager->traj_.local_traj.duration = 10.0;
  manager->traj_.local_traj.end_time = now + 10.0;
  publishFarPrediction(root, now);

  const Eigen::Vector3d target(0.0, -2.0, 1.2);
  const Eigen::Vector3d target_velocity(0.0, 1.0, 0.0);
  const Eigen::Vector3d offset(observer_x, 0.0, 0.0);
  EGOPlannerManager::CandidateSetOutput output;
  CaseResult result;
  result.trigger_time = ros::Time::now().toSec();
  result.predicted_loss = result.trigger_time + 1.55;
  result.generated = manager->reboundReplan(
      observer, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      target + offset, target_velocity, target, target_velocity,
      Eigen::Quaterniond::Identity(), offset, false, false, false, true,
      0.5, &output);
  result.trigger = output.nominal_conflict.valid &&
      output.nominal_conflict.reason_mask != EGOPlannerManager::CONFLICT_NONE;
  for (const auto &candidate : output.candidates)
  {
    const auto trajectory = candidate.min_jerk_opt.getTraj();
    if (candidate.kind != EGOPlannerManager::CandidateKind::NOMINAL)
    {
      std::cout << "ASTAR_FIXTURE_TRACE blocked=" << block_simple_minus
                << " kind="
                << static_cast<int>(candidate.kind)
                << " success=" << candidate.success
                << " astar=" << candidate.astar_used
                << " handoff=" << candidate.local_sfc_handoff_valid
                << " planes=" << candidate.local_sfc_planes.size();
      if (trajectory.getPieceNum() > 0)
      {
        std::cout << " duration=" << trajectory.getTotalDuration();
        for (const double phase : {0.0, 0.25, 0.5, 0.75, 1.0})
          std::cout << " p" << phase << "="
                    << trajectory.getPos(phase * trajectory.getTotalDuration()).transpose();
      }
      std::cout << " astar=" << candidate.astar_used
                << " sfc=" << candidate.local_sfc_planes.size() << '\n';
    }
    result.plus = result.plus ||
                  candidate.kind == EGOPlannerManager::CandidateKind::SIDE_PLUS;
    result.minus = result.minus ||
                   candidate.kind == EGOPlannerManager::CandidateKind::SIDE_MINUS;
    result.side_executable = result.side_executable ||
        (candidate.kind != EGOPlannerManager::CandidateKind::NOMINAL &&
         candidate.success && candidate.existing_checks_passed);
    result.astar_rescue_executable = result.astar_rescue_executable ||
        (candidate.kind != EGOPlannerManager::CandidateKind::NOMINAL &&
         candidate.astar_used && candidate.local_sfc_handoff_valid &&
         !candidate.local_sfc_planes.empty());
  }
  std::vector<EGOPlannerManager::CandidateSetOutput> sets;
  sets.push_back(std::move(output));
  int selected_set = -1;
  const auto status = manager->finalizeCapturedCandidates(
      sets, true, 0.5, selected_set);
  result.committed =
      status == EGOPlannerManager::TopologyProcessStatus::COMMITTED;
  result.activation_time = manager->traj_.local_traj.start_time;
  if (selected_set >= 0)
    for (const auto &candidate : sets[selected_set].candidates)
      if (candidate.candidate_id == sets[selected_set].local_candidate_id)
        result.selected = candidate.kind;
  std::cout << "VISIBILITY_TOPOLOGY_TRACE case=" << index
            << " observer_x=" << observer_x
            << " trigger=" << result.trigger
            << " plus=" << result.plus << " minus=" << result.minus
            << " side_executable=" << result.side_executable
            << " astar_rescue=" << result.astar_rescue_executable
            << " selected=" << static_cast<int>(result.selected)
            << " trigger_lead=" << result.predicted_loss - result.trigger_time
            << " activation_lead=" << result.predicted_loss - result.activation_time
            << '\n';
  return result;
}

MixedConflictResult runMixedBodyLosCase(ros::NodeHandle &root)
{
  ros::NodeHandle node("/visibility_topology/mixed_body_los");
  configure(node, 0);
  std::unique_ptr<EGOPlannerManager> manager(new EGOPlannerManager);
  manager->initPlanModules(
      node, ego_planner::PlanningVisualization::Ptr(
                new ego_planner::PlanningVisualization(node)));
  seedKnownFree(*manager);

  const double now = ros::Time::now().toSec();
  const Eigen::Vector3d observer(-2.0, -2.0, 1.2);
  manager->traj_.local_traj.traj = stationary(observer, 10.0).getTraj();
  manager->traj_.local_traj.traj_id = 71;
  manager->traj_.local_traj.drone_id = 0;
  manager->traj_.local_traj.start_time = now;
  manager->traj_.local_traj.duration = 10.0;
  manager->traj_.local_traj.end_time = now + 10.0;

  // The static fixture supplies a LOS witness while this independent
  // prediction crosses the nominal body path.  The two identities must stay
  // separate in the production descriptor.
  publishConstantPrediction(root, now, Eigen::Vector3d(-2.0, -1.35, 1.2));
  const Eigen::Vector3d target(0.0, -2.0, 1.2);
  const Eigen::Vector3d target_velocity(0.0, 1.0, 0.0);
  const Eigen::Vector3d offset(observer.x(), 0.0, 0.0);
  EGOPlannerManager::CandidateSetOutput output;
  MixedConflictResult result;
  result.generated = manager->reboundReplan(
      observer, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      target + offset, target_velocity, target, target_velocity,
      Eigen::Quaterniond::Identity(), offset, false, false, false, true,
      0.5, &output);
  result.reason_mask = output.nominal_conflict.reason_mask;
  result.primary_id = output.nominal_conflict.obstacle_identity;
  result.body_id = output.nominal_conflict.body_obstacle_identity;
  result.los_id = output.nominal_conflict.los_obstacle_identity;
  result.body_motion = output.nominal_conflict.body_obstacle_motion;
  result.los_motion = output.nominal_conflict.los_obstacle_motion;
  result.body_time = output.nominal_conflict.body_conflict_time;
  result.los_time = output.nominal_conflict.los_conflict_time;
  result.side_anchor = output.nominal_risk.conflict_time;
  std::cout << "MIXED_BODY_LOS_TRACE generated=" << result.generated
            << " reason_mask=" << result.reason_mask
            << " primary_id=" << result.primary_id
            << " body_id=" << result.body_id
            << " los_id=" << result.los_id
            << " body_motion=" << static_cast<int>(result.body_motion)
            << " los_motion=" << static_cast<int>(result.los_motion)
            << " body_time=" << result.body_time
            << " los_time=" << result.los_time
            << " side_anchor=" << result.side_anchor << '\n';
  return result;
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visibility_topology_production_test");
  ros::NodeHandle root;
  const CaseResult left = runMirrorCase(root, 0, -2.0);
  const CaseResult right = runMirrorCase(root, 0, 2.0);
  const MixedConflictResult mixed = runMixedBodyLosCase(root);

  expect(left.generated && right.generated,
         "production_candidate_batches_generated");
  expect(left.trigger && right.trigger,
         "shared_visibility_risk_triggers_before_binary_loss");
  expect(left.plus && left.minus && right.plus && right.minus,
         "both_topologies_dispatched_in_original_and_mirror");
  expect(left.side_executable && right.side_executable,
         "target_side_candidates_complete_full_executable_chain");
  expect(left.committed && right.committed,
         "selected_candidate_activated_before_predicted_loss");
  expect(left.predicted_loss > left.activation_time &&
             right.predicted_loss > right.activation_time,
         "activation_has_positive_visibility_loss_lead");
  expect(mixed.generated &&
             (mixed.reason_mask & EGOPlannerManager::CONFLICT_BODY_SAFETY) != 0 &&
             (mixed.reason_mask & EGOPlannerManager::CONFLICT_LOS_OCCLUSION) != 0,
         "mixed_body_and_los_reaches_shared_production_descriptor");
  expect(mixed.body_id >= 0 && mixed.los_id >= 0 &&
             mixed.body_motion == EGOPlannerManager::ConflictObstacleMotion::DYNAMIC &&
             mixed.los_motion == EGOPlannerManager::ConflictObstacleMotion::STATIC,
         "mixed_body_and_los_preserves_independent_blocker_identities");
  expect(mixed.body_time >= 0.0 && mixed.los_time >= 0.0 &&
             std::abs(mixed.side_anchor - mixed.body_time) < 1.0e-6,
         "mixed_body_and_los_keeps_body_safety_side_anchor");
  return failures == 0 ? 0 : 1;
}
