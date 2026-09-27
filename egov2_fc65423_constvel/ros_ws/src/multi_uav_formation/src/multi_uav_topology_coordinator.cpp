#include <multi_uav_formation/topology_coordinator_core.h>
#include <multi_uav_formation/team_target_centered_optimizer.h>
#include <multi_uav_formation/realized_team_validator.h>
#include <multi_uav_formation/team_planning_context.h>
#include <multi_uav_formation/committed_prefix_joint_contract.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/plan_container.hpp>
#include <multi_uav_formation/activation_schedule.h>
#include <map>
#include <multi_uav_formation/tracking_visibility_geometry.h>

#include <optimizer/poly_traj_utils.hpp>
#include <plan_env/obj_predictor.h>
#include <plan_env/static_los_geometry.h>
#include <ros/ros.h>
#include <traj_utils/TeamTrajectoryAck.h>
#include <traj_utils/TeamTrajectorySolution.h>
#include <traj_utils/TeamReferenceSchedule.h>
#include <traj_utils/CooperativeTaskReference.h>
#include <traj_utils/TaskReferenceAdjustment.h>
#include <traj_utils/TopologyEscalation.h>
#include <traj_utils/TopologyCandidateBundle.h>
#include <traj_utils/ablation_runtime.h>
#include <traj_utils/TopologyCoordination.h>

#include <array>
#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>

namespace multi_uav_formation
{
namespace
{

bool reconstructTrajectory(const traj_utils::MINCOTraj &message,
                           poly_traj::Trajectory &trajectory)
{
  if (message.order != 5 || message.duration.empty() ||
      message.duration.size() != message.inner_x.size() + 1 ||
      message.inner_x.size() != message.inner_y.size() ||
      message.inner_x.size() != message.inner_z.size())
    return false;
  const int piece_count = static_cast<int>(message.duration.size());
  Eigen::Matrix<double, 3, 3> head_state;
  Eigen::Matrix<double, 3, 3> tail_state;
  head_state << message.start_p[0], message.start_v[0], message.start_a[0],
      message.start_p[1], message.start_v[1], message.start_a[1],
      message.start_p[2], message.start_v[2], message.start_a[2];
  tail_state << message.end_p[0], message.end_v[0], message.end_a[0],
      message.end_p[1], message.end_v[1], message.end_a[1],
      message.end_p[2], message.end_v[2], message.end_a[2];
  Eigen::MatrixXd inner_points(3, piece_count - 1);
  Eigen::VectorXd durations(piece_count);
  for (int index = 0; index < piece_count - 1; ++index)
    inner_points.col(index) << message.inner_x[index], message.inner_y[index],
        message.inner_z[index];
  for (int index = 0; index < piece_count; ++index)
  {
    durations(index) = message.duration[index];
    if (!std::isfinite(durations(index)) || durations(index) <= 1.0e-6)
      return false;
  }
  poly_traj::MinJerkOpt optimizer;
  optimizer.reset(head_state, tail_state, piece_count);
  optimizer.generate(inner_points, durations);
  trajectory = optimizer.getTraj();
  return trajectory.getPieceNum() == piece_count &&
         std::isfinite(trajectory.getTotalDuration());
}

bool reconstructOptimizer(const traj_utils::MINCOTraj &message,
                          poly_traj::MinJerkOpt &optimizer)
{
  poly_traj::Trajectory trajectory;
  if (!reconstructTrajectory(message, trajectory))
    return false;
  const int piece_count = trajectory.getPieceNum();
  Eigen::Matrix3d head_state;
  Eigen::Matrix3d tail_state;
  head_state << trajectory.getJuncPos(0), trajectory.getJuncVel(0),
      trajectory.getJuncAcc(0);
  tail_state << trajectory.getJuncPos(piece_count),
      trajectory.getJuncVel(piece_count), trajectory.getJuncAcc(piece_count);
  optimizer.reset(head_state, tail_state, piece_count);
  optimizer.generate(
      trajectory.getPositions().block(0, 1, 3, piece_count - 1),
      trajectory.getDurations());
  return optimizer.getTraj().getPieceNum() == piece_count;
}

bool optimizerFromTrajectory(const poly_traj::Trajectory &trajectory,
                             poly_traj::MinJerkOpt &optimizer)
{
  const int piece_count = trajectory.getPieceNum();
  if (piece_count <= 0 || !std::isfinite(trajectory.getTotalDuration()))
    return false;
  Eigen::Matrix3d head_state;
  Eigen::Matrix3d tail_state;
  head_state << trajectory.getJuncPos(0), trajectory.getJuncVel(0),
      trajectory.getJuncAcc(0);
  tail_state << trajectory.getJuncPos(piece_count),
      trajectory.getJuncVel(piece_count), trajectory.getJuncAcc(piece_count);
  optimizer.reset(head_state, tail_state, piece_count);
  optimizer.generate(
      trajectory.getPositions().block(0, 1, 3, piece_count - 1),
      trajectory.getDurations());
  return optimizer.getTraj().getPieceNum() == piece_count;
}

double sampleYawTrace(const std::vector<double> &times,
                      const std::vector<double> &values,
                      const double time)
{
  if (times.empty() || times.size() != values.size())
    return 0.0;
  const auto upper = std::lower_bound(times.begin(), times.end(), time);
  if (upper == times.begin())
    return values.front();
  if (upper == times.end())
    return values.back();
  const size_t high = static_cast<size_t>(upper - times.begin());
  const size_t low = high - 1;
  const double span = times[high] - times[low];
  if (span <= 1.0e-9)
    return values[low];
  double delta = values[high] - values[low];
  while (delta > M_PI)
    delta -= 2.0 * M_PI;
  while (delta < -M_PI)
    delta += 2.0 * M_PI;
  return values[low] + (time - times[low]) / span * delta;
}

void trajectoryToMINCOMessage(const poly_traj::Trajectory &trajectory,
                              const int drone_id, const int trajectory_id,
                              const double start_time,
                              traj_utils::MINCOTraj &message)
{
  const int piece_count = trajectory.getPieceNum();
  message.drone_id = drone_id;
  message.traj_id = trajectory_id;
  message.start_time = ros::Time(start_time);
  message.order = 5;
  message.duration.resize(piece_count);
  const Eigen::VectorXd durations = trajectory.getDurations();
  Eigen::Vector3d value = trajectory.getPos(0.0);
  for (int axis = 0; axis < 3; ++axis)
    message.start_p[axis] = value(axis);
  value = trajectory.getVel(0.0);
  for (int axis = 0; axis < 3; ++axis)
    message.start_v[axis] = value(axis);
  value = trajectory.getAcc(0.0);
  for (int axis = 0; axis < 3; ++axis)
    message.start_a[axis] = value(axis);
  const double duration = trajectory.getTotalDuration();
  value = trajectory.getPos(duration);
  for (int axis = 0; axis < 3; ++axis)
    message.end_p[axis] = value(axis);
  value = trajectory.getVel(duration);
  for (int axis = 0; axis < 3; ++axis)
    message.end_v[axis] = value(axis);
  value = trajectory.getAcc(duration);
  for (int axis = 0; axis < 3; ++axis)
    message.end_a[axis] = value(axis);
  message.inner_x.resize(std::max(0, piece_count - 1));
  message.inner_y.resize(std::max(0, piece_count - 1));
  message.inner_z.resize(std::max(0, piece_count - 1));
  const Eigen::MatrixXd positions = trajectory.getPositions();
  for (int piece = 0; piece < piece_count - 1; ++piece)
  {
    message.inner_x[piece] = positions(0, piece + 1);
    message.inner_y[piece] = positions(1, piece + 1);
    message.inner_z[piece] = positions(2, piece + 1);
  }
  for (int piece = 0; piece < piece_count; ++piece)
    message.duration[piece] = durations(piece);
}

struct SmoothScore
{
  bool valid{false};
  double value{0.0};
  double derivative{0.0};
};

// A C1 geometry score: zero at/below inner, one at/above outer.  The
// derivative is with respect to the supplied clearance and therefore retains
// the units of that clearance (metres or radians).
SmoothScore smoothIncreasingScore(const double clearance,
                                  const double inner,
                                  const double outer)
{
  SmoothScore result;
  if (!std::isfinite(clearance) || !std::isfinite(inner) ||
      !std::isfinite(outer) || outer <= inner + 1.0e-12)
    return result;
  result.valid = true;
  if (clearance <= inner)
    return result;
  if (clearance >= outer)
  {
    result.value = 1.0;
    return result;
  }
  const double u = (clearance - inner) / (outer - inner);
  result.value = u * u * (3.0 - 2.0 * u);
  result.derivative = 6.0 * u * (1.0 - u) / (outer - inner);
  return result;
}

void multiplyScore(const SmoothScore &factor,
                   const Eigen::Vector3d &factor_gradient,
                   double &value, Eigen::Vector3d &gradient, bool &valid)
{
  if (!factor.valid || !factor_gradient.allFinite())
  {
    valid = false;
    return;
  }
  gradient = gradient * factor.value + value * factor.derivative *
                                         factor_gradient;
  value *= factor.value;
}

const char *kindName(const int kind)
{
  switch (kind)
  {
  case traj_utils::TopologyCandidate::KIND_SIDE_PLUS:
    return "SIDE_PLUS";
  case traj_utils::TopologyCandidate::KIND_SIDE_MINUS:
    return "SIDE_MINUS";
  default:
    return "NOMINAL";
  }
}

}  // namespace

class MultiUavTopologyCoordinator
{
public:
  explicit MultiUavTopologyCoordinator(ros::NodeHandle &node) : node_(node)
  {
    ablation_config_ = traj_utils::loadAblationConfig(
        "topology_coordinator", &formal_ablation_mode_);
    TopologyCoordinatorParams params;
    node_.param("swarm_clearance", params.swarm_clearance, 0.50);
    node_.param("near_margin", params.near_margin, 0.08);
    node_.param("sample_dt", params.sample_dt, 0.05);
    node_.param("max_bundle_age", params.max_bundle_age, 0.65);
    max_bundle_age_ = std::max(0.01, params.max_bundle_age);
    node_.param("max_epoch_skew", params.max_epoch_skew, 0.65);
    node_.param("max_start_skew", params.max_start_skew, 0.65);
    node_.param("preferred_view_angle_deg", params.preferred_view_angle_deg, 25.0);
    node_.param("encirclement_spread_saturation_angle_deg",
                params.encirclement_spread_saturation_angle_deg, 60.0);
    node_.param("binary_k2_protection_tolerance", params.k2_protection_tolerance, 0.02);
    node_.param("rapid_reversal_window", params.rapid_reversal_window, 1.0);
    node_.param("crossing_time_tolerance", params.crossing_time_tolerance, 0.25);
    node_.param("team_top_k", params.team_top_k, 3);
    node_.param("team_objective_k2_weight", params.objective_weights.k2, 8.0);
    node_.param("team_objective_acc_weight",
                params.objective_weights.accumulated, 1.0);
    node_.param("team_objective_comp_weight",
                params.objective_weights.complementarity, 1.5);
    node_.param("team_objective_enc_weight",
                params.objective_weights.encirclement, 0.35);
    node_.param("team_objective_reg_weight",
                params.objective_weights.regularization, 0.20);
    node_.param("planner_acceptance_timeout", planner_acceptance_timeout_, 0.20);
    double team_solution_max_age = 0.30;
    node_.param("team_solution_max_age", team_solution_max_age, 0.30);
    node_.param("result_publish_margin", result_publish_margin_, 0.02);
    planner_acceptance_timeout_ = std::max(0.02, planner_acceptance_timeout_);
    result_publish_margin_ = std::max(
        0.0, std::min(result_publish_margin_, 0.5 * planner_acceptance_timeout_));
    double gap_max_deg=170.0;
    node_.param("/encirclement_geometry/max_circular_gap_deg",gap_max_deg,170.0);
    node_.param("/encirclement_geometry/min_angular_separation_deg",params.preferred_view_angle_deg,25.0);
    node_.param("/encirclement_geometry/ratio_min",params.encirclement_ratio_min,0.8);
    params.theta_gap_max=gap_max_deg*M_PI/180.0;
    core_.reset(new TopologyCoordinatorCore(params));

    node_.param("enable_team_visibility_optimizer",
                enable_team_visibility_optimizer_, false);
    node_.param("enable_joint_pt_optimization",
                enable_joint_pt_optimization_, false);
    if (formal_ablation_mode_ &&
        (enable_team_visibility_optimizer_ != ablation_config_.team_pt ||
         enable_joint_pt_optimization_ != ablation_config_.team_pt))
      throw std::invalid_argument(
          "Formal ablation Team P/T launch state does not match AblationConfig");
    node_.param("enable_predicted_attitude_fov",
                enable_predicted_attitude_fov_, false);
    node_.param("team_prediction_horizon",
                target_centered_params_.prediction_horizon, 2.0);
    node_.param("team_reference_sample_dt",
                target_centered_params_.sample_dt, 0.10);
    node_.param("team_reference_knot_count",
                target_centered_params_.reference_knot_count, 4);
    node_.param("team_top_k", target_centered_params_.top_k, 3);
    node_.param("team_target_centered_max_iterations",
                target_centered_params_.max_iterations, 10);
    node_.param("team_target_centered_timeout",
                target_centered_params_.timeout_seconds, 0.045);
    node_.param("team_radius_trust",
                target_centered_params_.radius_trust, 0.30);
    node_.param("team_bearing_trust",
                target_centered_params_.bearing_trust,
                20.0 * M_PI / 180.0);
    node_.param("team_phase_trust",
                target_centered_params_.phase_trust, 0.45);
    node_.param("team_reference_realization_timeout",
                team_reference_realization_timeout_, 0.25);
    node_.param("team_objective_k2_weight",
                target_centered_params_.weights.k2, 8.0);
    node_.param("team_objective_acc_weight",
                target_centered_params_.weights.accumulated, 1.0);
    node_.param("team_objective_comp_weight",
                target_centered_params_.weights.complementarity, 1.5);
    node_.param("team_objective_enc_weight",
                target_centered_params_.weights.encirclement, 0.35);
    node_.param("team_objective_reg_weight",
                target_centered_params_.weights.regularization, 0.20);
    team_swarm_clearance_ = params.swarm_clearance;
    ROS_INFO("[encirclement-contract] max_gap_deg=%.3f min_gap_deg=%.3f "
             "ratio_min=%.3f",
             gap_max_deg, params.preferred_view_angle_deg,
             params.encirclement_ratio_min);
    node_.param("team_activation_lead", team_activation_lead_, 0.24);
    node_.param("team_ack_timeout", team_ack_timeout_, 0.06);
    team_activation_lead_ = std::max(0.05, team_activation_lead_);
    team_ack_timeout_ = std::max(0.01, std::min(
        team_ack_timeout_, 0.75 * team_activation_lead_));

    node_.param("visibility_min_target_distance",
                tracking_camera_.min_range, 0.20);
    node_.param("visibility_max_target_distance",
                tracking_camera_.max_range, 8.0);
    node_.param("visibility_occlusion_margin",
                visibility_occlusion_margin_, 0.08);
    node_.param("visibility_range_smoothing", visibility_range_smoothing_,
                0.25);
    node_.param("visibility_los_smoothing", visibility_los_smoothing_, 0.15);
    node_.param("predictive_relay_horizon", relay_horizon_, 2.0);
    node_.param("predictive_relay_sample_dt", relay_sample_dt_, 0.10);
    node_.param("predictive_relay_trigger_margin",
                relay_trigger_margin_, 0.30);
    node_.param("predictive_relay_required_margin",
                relay_required_margin_, 0.20);
    node_.param("predictive_relay_release_margin",
                relay_release_margin_, 0.45);
    node_.param("predictive_relay_overlap", relay_overlap_, 0.50);
    node_.param("predictive_relay_stable_time", relay_stable_time_, 0.20);
    node_.param("predictive_relay_contract_timeout",
                relay_contract_timeout_, 1.0);
    node_.param("/alp/team_k3_repair_enabled", team_k3_repair_enabled_, false);
    node_.param("/alp/k3_local_escalation_enabled", k3_local_autonomy_enabled_, false);
    relay_horizon_ = std::max(0.20, relay_horizon_);
    relay_sample_dt_ = std::max(0.02, relay_sample_dt_);
    relay_overlap_ = std::max(relay_sample_dt_, relay_overlap_);
    relay_stable_time_ = std::max(0.0, relay_stable_time_);
    relay_contract_timeout_ = std::max(0.10, relay_contract_timeout_);
    double camera_hfov_deg = 85.0;
    double camera_width = 1280.0;
    double camera_height = 720.0;
    double camera_x = 0.0, camera_y = 0.0, camera_z = 0.0;
    double camera_roll = 0.0, camera_pitch = 0.0, camera_yaw = 0.0;
    node_.param("tracking_camera_hfov_deg", camera_hfov_deg, 85.0);
    node_.param("tracking_camera_image_width", camera_width, 1280.0);
    node_.param("tracking_camera_image_height", camera_height, 720.0);
    node_.param("tracking_camera_extrinsic_x", camera_x, 0.0);
    node_.param("tracking_camera_extrinsic_y", camera_y, 0.0);
    node_.param("tracking_camera_extrinsic_z", camera_z, 0.0);
    node_.param("tracking_camera_extrinsic_roll", camera_roll, 0.0);
    node_.param("tracking_camera_extrinsic_pitch", camera_pitch, 0.0);
    node_.param("tracking_camera_extrinsic_yaw", camera_yaw, 0.0);
    tracking_camera_.horizontal_fov = camera_hfov_deg * M_PI / 180.0;
    tracking_camera_.vertical_fov = verticalFovFromHorizontal(
        tracking_camera_.horizontal_fov, camera_width, camera_height);
    tracking_camera_.translation_body =
        Eigen::Vector3d(camera_x, camera_y, camera_z);
    tracking_camera_.body_from_camera =
        Eigen::AngleAxisd(camera_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(camera_pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(camera_roll, Eigen::Vector3d::UnitX());

    const bool requested_team_optimizer =
        ablation_config_.team_pt && enable_team_visibility_optimizer_ &&
        enable_joint_pt_optimization_;
    if (requested_team_optimizer)
    {
      // Match the planner's pending-generation lifetime: ordinary team
      // coordination budget plus the proposal transport window.  Identity,
      // common-time, and executable-horizon checks remain unchanged.
      planner_acceptance_timeout_ += std::max(0.0, team_solution_max_age);
    }
    std::string scene_file;
    node_.param("scene_file", scene_file, std::string());
    std::string scene_reason = "DISABLED";
    if (requested_team_optimizer)
    {
      team_static_geometry_ready_ =
          static_geometry_.loadScene(scene_file, scene_reason);
      int moving_object_count = 0;
      node_.param("prediction/obj_num", moving_object_count, 0);
      if (moving_object_count > 0)
      {
        object_predictor_.reset(new fast_planner::ObjPredictor(node_));
        object_predictor_->init();
      }
      target_centered_optimizer_.reset(new TeamTargetCenteredOptimizer(
          target_centered_params_));
      team_runtime_ready_ = team_static_geometry_ready_ &&
                            static_cast<bool>(target_centered_optimizer_);
    }

    for (int drone = 0; drone < 3; ++drone)
    {
      const std::string topic =
          "/topology_coordination/uav" + std::to_string(drone) +
          "/candidate_bundle";
      bundle_subscribers_[drone] = node_.subscribe<traj_utils::TopologyCandidateBundle>(
          topic, 4,
          boost::bind(&MultiUavTopologyCoordinator::bundleCallback, this, _1,
                      drone),
          ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    }
    for(int drone=0;drone<3;++drone)
      execution_subscribers_[drone]=node_.subscribe<traj_utils::PolyTraj>(
          "/drone_"+std::to_string(drone)+"_planning/trajectory",20,
          [this](const traj_utils::PolyTrajConstPtr &m){ executionCallback(m); });
    actual_execution_subscriber_=node_.subscribe<traj_utils::PolyTraj>("/trajectory_execution/activated",30,
          [this](const traj_utils::PolyTrajConstPtr &m){ executionCallback(m); });
    result_publisher_ = node_.advertise<traj_utils::TopologyCoordination>(
        "/topology_coordination/result", 8);
    team_solution_publisher_ =
        node_.advertise<traj_utils::TeamTrajectorySolution>(
            "/topology_coordination/team_solution", 8, true);
    team_reference_publisher_ =
        node_.advertise<traj_utils::TeamReferenceSchedule>(
            "/topology_coordination/team_reference_schedule", 8, true);
    team_ack_subscriber_ = node_.subscribe<traj_utils::TeamTrajectoryAck>(
        "/topology_coordination/team_ack", 12,
        &MultiUavTopologyCoordinator::teamAckCallback, this,
        ros::TransportHints().tcpNoDelay());
    // 阶段 B：任务几何唯一 authority 是 cooperative_viewpoint_manager。
    // Team 的 desired_radius/bearing/height 基线一律来自该消息。
    task_reference_subscriber_ = node_.subscribe<traj_utils::CooperativeTaskReference>(
        "/cooperative_task_reference", 1,
        &MultiUavTopologyCoordinator::taskReferenceCallback, this,
        ros::TransportHints().tcpNoDelay());
    // Team -> Cooperative 的任务级 phi0 建议（realized triple 被接受后发出）。
    task_adjustment_publisher_ = node_.advertise<traj_utils::TaskReferenceAdjustment>(
        "/cooperative_task_adjustment", 4, true);
    // 阶段 G:topology escalation 广播(same-topology SCP 失败 -> N/L/R)。
    topology_escalation_publisher_ =
        node_.advertise<traj_utils::TopologyEscalation>(
            "/topology_coordination/topology_escalation", 4, true);
    team_commit_timer_ = node_.createTimer(
        ros::Duration(0.01),
        &MultiUavTopologyCoordinator::teamCommitTimerCallback, this);
    ROS_INFO("[topology-coordinator-config] enabled=1 N=3 max_combinations=27 "
             "sample_dt=%.3f swarm_clearance=%.3f near_margin=%.3f "
             "max_bundle_age=%.3f max_epoch_skew=%.3f max_start_skew=%.3f "
             "world_space_interaction=1 side_label_physical_direction=0 "
             "blocking=0",
             params.sample_dt, params.swarm_clearance, params.near_margin,
             params.max_bundle_age, params.max_epoch_skew, params.max_start_skew);
    ROS_INFO("[team-target-centered-config] enabled=%d joint_pt=%d "
             "runtime_ready=%d scene=%s scene_status=%s horizon=%.3f "
             "knots=%d top_k=%d max_iter=%d timeout=%.4f "
             "activation_lead=%.3f ack_timeout=%.3f "
             "execution=LOCAL_REALIZE_THEN_2PC joint_yaw=absent",
             static_cast<int>(enable_team_visibility_optimizer_),
             static_cast<int>(enable_joint_pt_optimization_),
             static_cast<int>(team_runtime_ready_), scene_file.c_str(),
             scene_reason.c_str(), target_centered_params_.prediction_horizon,
             target_centered_params_.reference_knot_count,
             target_centered_params_.top_k,
             target_centered_params_.max_iterations,
             target_centered_params_.timeout_seconds,
             team_activation_lead_, team_ack_timeout_);
    ROS_INFO("[team-target-centered-config] variables=R/PHI/T height_fixed=1 "
             "top_k=%d knots=%d H_pred=%.3f H_exec=LOCAL_VALIDATED_PREFIX "
             "objectives=J_K2,J_ACC,J_COMP,J_ENC,J_REG "
             "joint_yaw=OFF executable_authority=LOCAL_ONLY",
             target_centered_params_.top_k,
             target_centered_params_.reference_knot_count,
             target_centered_params_.prediction_horizon);
    ROS_INFO("[PREDICTIVE_VISIBILITY_RELAY_CONFIG] horizon=%.3f dt=%.3f "
             "trigger=%.3f required=%.3f release=%.3f overlap=%.3f "
             "stable=%.3f timeout=%.3f",
             relay_horizon_, relay_sample_dt_, relay_trigger_margin_,
             relay_required_margin_, relay_release_margin_, relay_overlap_,
             relay_stable_time_, relay_contract_timeout_);
    // loadAblationConfig() runs at the very beginning of construction.  On a
    // fast launch rosout can miss that first line, which makes the runner's
    // fail-closed reconciliation reject a correct process.  Repeat the same
    // resolved truth only after all publishers/subscribers are installed.
    ROS_INFO("[ablation-config] component=topology_coordinator mode=%s formal=%d "
             "local_visibility=%d team_pt=%d los_topology=%d "
             "dynamic_body_topology=%d target_facing_astar=%d "
             "safe_seed_retention=%d local_gap=%d encirclement=%d "
             "cooperative_reference=%d physical_safety=%d joint_yaw=%d",
             traj_utils::toString(ablation_config_.mode),
             static_cast<int>(formal_ablation_mode_),
             static_cast<int>(ablation_config_.local_visibility),
             static_cast<int>(ablation_config_.team_pt),
             static_cast<int>(ablation_config_.los_topology),
             static_cast<int>(ablation_config_.dynamic_body_topology),
             static_cast<int>(ablation_config_.target_facing_astar),
             static_cast<int>(ablation_config_.safe_seed_retention),
             static_cast<int>(ablation_config_.local_gap),
             static_cast<int>(ablation_config_.encirclement),
             static_cast<int>(ablation_config_.cooperative_reference),
             static_cast<int>(ablation_config_.physical_safety),
             static_cast<int>(ablation_config_.joint_yaw));
  }

private:
  struct RelayAssessment
  {
    bool valid{false};
    double min_m2{std::numeric_limits<double>::infinity()};
    double min_world_time{0.0};
    ContinuousVisibilitySample::Limiter limiter{
        ContinuousVisibilitySample::LIMITER_NONE};
    std::array<double, 3> activation_margins{{0.0, 0.0, 0.0}};
    int outgoing{-1};
    int incoming{-1};
    int stable{-1};
    // Feedback098: the UAV whose margin actually produced M2 at the critical
    // world time, plus the per-UAV margins at that same instant.  Propagated
    // explicitly so no downstream stage has to re-derive it.
    int limiting_uav{-1};
    double limiting_margin{0.0};
    std::array<double, 3> critical_margins{{0.0, 0.0, 0.0}};
    std::uint64_t forecast_created_world_time_ns{0};
    // Feedback100: M2 only DETECTS the risk; it no longer decides the single
    // UAV that gets repaired.  The assessment records the FIRST threshold
    // crossing (t_cross) and the end of that first contiguous crossing run
    // (t_recover); the contract window is anchored there instead of on the
    // argmin, so the intervention starts as soon as the future crossing is
    // visible.  t_star (= min_world_time) is kept for severity diagnostics.
    bool crossing_found{false};
    bool crossing_closed{false};
    double t_cross{0.0};
    double t_recover{0.0};
    std::array<double, 3> cross_margins{{0.0, 0.0, 0.0}};
    // Repair target set: the two channels whose restoration can raise M2
    // (ranked[1] and ranked[2]).  ranked[0] is the stable observer and is
    // already above the requirement.
    int repair_candidate_count{0};
    std::array<int, 2> repair_candidates{{-1, -1}};
    std::array<double, 2> repair_candidate_margins{{0.0, 0.0}};
    int repair_priority_uav{-1};
  };

  struct K3Assessment
  {
    bool valid{false};
    bool found{false};
    int target{-1};
    double t_cross{0.0};
    double t_recover{0.0};
    double t_star{0.0};
    double min_m3{std::numeric_limits<double>::infinity()};
    double m2_at_star{0.0};
    std::array<double, 3> margins{{0.0, 0.0, 0.0}};
    int baseline_k3_samples{0};
    int baseline_k2_samples{0};
    int critical_samples{0};
    // K3 Local escalation detect evidence: the common-window sample total
    // behind the K2/K3 fractions and the target's limiting component at t_star
    // (ContinuousVisibilitySample::Limiter).  Copied into the contract so the
    // later escalation broadcast stays event-bound.
    int baseline_total_samples{0};
    int t_star_limiter{0};
  };

  struct PersistentHandoffContract
  {
    bool active{false};
    std::uint8_t repair_k{2};
    std::uint64_t id{0};
    std::uint64_t task_reference_generation{0};
    std::uint64_t target_revision{0};
    std::uint64_t dynamic_revision{0};
    std::uint64_t static_revision{0};
    std::uint64_t visibility_revision{0};
    // Immutable audit identity captured when the relay contract is created.
    std::array<std::uint64_t, 3> baseline_owner_revisions{{0, 0, 0}};
    std::array<int, 3> baseline_owner_trajectory_ids{{0, 0, 0}};
    // Mutable direct-successor frontier.  A persistent handoff requirement is
    // allowed to follow the already validated Local lineage across rolling
    // commits; an unrelated/rebased trajectory still makes it stale.
    std::array<std::uint64_t, 3> expected_lineage_revisions{{0, 0, 0}};
    std::array<int, 3> expected_lineage_trajectory_ids{{0, 0, 0}};
    int outgoing{-1};
    int incoming{-1};
    int stable{-1};
    double activation{0.0};
    double acquire_by{0.0};
    double preserve_until{0.0};
    double created{0.0};
    double expires{0.0};
    double required_overlap{0.0};
    double required_margin{0.0};
    double m2_before{0.0};
    double m2_min{0.0};
    double m2_min_time{0.0};
    // Feedback098: the UAV that actually produced M2 at the critical time.
    // Feedback100: retained as a DIAGNOSTIC ("who currently defines M2"); it is
    // no longer the single repair target.
    int limiting_uav{-1};
    double limiting_margin{0.0};
    // Feedback100: intervention interval (first threshold crossing run) and
    // the repair target set that replaces the single limiting UAV.
    double t_cross{0.0};
    double t_recover{0.0};
    double t_star{0.0};
    std::array<double, 3> forecast_margins{{0.0, 0.0, 0.0}};
    int repair_candidate_count{0};
    std::array<int, 2> repair_candidates{{-1, -1}};
    std::array<double, 2> repair_candidate_margins{{0.0, 0.0}};
    int repair_priority_uav{-1};
    // Feedback100 pipeline anchor: the wall-clock instant the earliest
    // crossing was observed and this contract was created.
    double detect_wall{0.0};
    // K3 Local escalation binding evidence (K3 contracts only).  k2/k3 are
    // binary fractions over the fused assessment window; detect_limiter is the
    // target's limiting component at t_star.  The escalation broadcast
    // forwards them so the Planner can bind the early SIDE authority to the
    // same event instead of a bare (target, window) pair.
    std::uint8_t detect_limiter{0};
    double detect_k2{0.0};
    double detect_k3{0.0};
  };
  void executionCallback(const traj_utils::PolyTrajConstPtr &m) {
    if(!m || m->drone_id<0 || m->drone_id>=3 || !m->safety_validated || m->order!=5 ||
        m->duration.empty() || m->coef_x.size()!=6*m->duration.size() ||
        m->coef_y.size()!=m->coef_x.size() || m->coef_z.size()!=m->coef_x.size())return;
    const int i=m->drone_id;if(m->generation<=execution_generations_[i])return;
    ego_planner::LocalTrajData next;next.drone_id=i;next.traj_id=m->traj_id;next.start_time=m->start_time.toSec();
    for(size_t j=0;j<m->duration.size();++j) {
      if(!std::isfinite(m->duration[j]) || m->duration[j]<=0)return;
      poly_traj::CoefficientMat c;
      for(int k=0;k<6;++k)c.col(k)<<m->coef_x[6*j+k],m->coef_y[6*j+k],m->coef_z[6*j+k];
      if(!c.allFinite())return;next.traj.emplace_back(m->duration[j],c);
    }
    next.duration=next.traj.getTotalDuration();
    next.optimized_yaw_valid=m->optimized_yaw_valid;
    next.optimized_yaw_sample_times=m->yaw_sample_times;next.optimized_yaw_samples=m->yaw_samples;
    if(execution_[i].traj.getPieceNum()>0) {
      next.predecessor=std::make_shared<ego_planner::LocalTrajData>(execution_[i]);next.predecessor->predecessor.reset();
    }
    if (active_handoff_contract_.active && !m->team_prepare_only)
    {
      const bool frontier_is_current =
          active_handoff_contract_.expected_lineage_revisions[i] ==
              execution_generations_[i] &&
          active_handoff_contract_.expected_lineage_trajectory_ids[i] ==
              execution_[i].traj_id;
      const bool direct_successor = frontier_is_current &&
          m->expected_predecessor_generation == execution_generations_[i] &&
          m->expected_predecessor_id == execution_[i].traj_id;
      if (direct_successor)
      {
        const std::uint64_t old_generation = execution_generations_[i];
        const int old_trajectory_id = execution_[i].traj_id;
        active_handoff_contract_.expected_lineage_revisions[i] = m->generation;
        active_handoff_contract_.expected_lineage_trajectory_ids[i] = m->traj_id;
        ROS_INFO("[HANDOFF_CONTRACT_LINEAGE_ADVANCE] id=%lu drone=%d "
                 "old=%lu/%d new=%lu/%d lineage=%s",
                 static_cast<unsigned long>(active_handoff_contract_.id), i,
                 static_cast<unsigned long>(old_generation), old_trajectory_id,
                 static_cast<unsigned long>(m->generation), m->traj_id,
                 m->lineage_class.c_str());
      }
      else if (frontier_is_current)
      {
        ROS_INFO("[HANDOFF_CONTRACT_LINEAGE_DIVERGED] id=%lu drone=%d "
                 "frontier=%lu/%d predecessor=%lu/%d new=%lu/%d",
                 static_cast<unsigned long>(active_handoff_contract_.id), i,
                 static_cast<unsigned long>(execution_generations_[i]),
                 execution_[i].traj_id,
                 static_cast<unsigned long>(m->expected_predecessor_generation),
                 m->expected_predecessor_id,
                 static_cast<unsigned long>(m->generation), m->traj_id);
      }
    }
    execution_[i]=next;execution_generations_[i]=m->generation;
    execution_team_reference_ids_[i] = m->team_reference_id;
    ROS_INFO("[team-reference-feedback] drone=%d reference_id=%lu generation=%lu trajectory_id=%d activation=%.9f source=%s reason=%s",
        i,static_cast<unsigned long>(m->team_reference_id),static_cast<unsigned long>(m->generation),m->traj_id,
        m->start_time.toSec(),m->trajectory_source.c_str(),m->team_reference_reason.c_str());
    // Early-joint bundles intentionally arrive before the independently
    // committed local fallback.  Re-evaluate when the expected execution
    // identity becomes authoritative instead of consuming the snapshot as
    // stale or incomplete.
    evaluateSnapshot();
  }

  bool executionYawAt(const int drone, const double world_time,
                      double &yaw) const
  {
    if (drone < 0 || drone >= 3 || !std::isfinite(world_time))
      return false;
    const auto &local = execution_[drone];
    const double local_time = world_time - local.start_time;
    if (local.optimized_yaw_valid &&
        !local.optimized_yaw_sample_times.empty() &&
        local.optimized_yaw_sample_times.size() ==
            local.optimized_yaw_samples.size())
    {
      yaw = sampleYawTrace(local.optimized_yaw_sample_times,
                           local.optimized_yaw_samples, local_time);
      return std::isfinite(yaw);
    }

    // The candidate trace is the producer's frozen prediction of the same
    // target-facing executor state.  It is a legitimate fallback only when
    // its identity is the Local candidate named by this bundle.
    const auto &bundle = bundle_messages_[drone];
    for (const auto &candidate : bundle.candidates)
    {
      if (candidate.candidate_id != bundle.local_selected_candidate_id ||
          candidate.predicted_yaw_sample_offsets.empty() ||
          candidate.predicted_yaw_sample_offsets.size() !=
              candidate.predicted_yaw_samples.size())
        continue;
      yaw = sampleYawTrace(
          candidate.predicted_yaw_sample_offsets,
          candidate.predicted_yaw_samples,
          world_time - candidate.trajectory_start_time.toSec());
      return std::isfinite(yaw);
    }
    return false;
  }

  // Linear interpolation of one UAV trace at an absolute world time.
  static bool sampleForecastMargin(
      const traj_utils::VisibilityForecastTrace &trace, const double world_time,
      double &margin, int &limiter)
  {
    const size_t count = trace.world_times.size();
    if (!trace.valid || count == 0 || count != trace.margins.size() ||
        count != trace.limiters.size())
      return false;
    if (world_time < trace.world_times.front() - 1.0e-9 ||
        world_time > trace.world_times.back() + 1.0e-9)
      return false;
    size_t upper = 0;
    while (upper < count && trace.world_times[upper] < world_time - 1.0e-9)
      ++upper;
    if (upper == 0)
    {
      margin = trace.margins[0];
      limiter = trace.limiters[0];
      return true;
    }
    if (upper >= count)
    {
      margin = trace.margins[count - 1];
      limiter = trace.limiters[count - 1];
      return true;
    }
    const double t0 = trace.world_times[upper - 1];
    const double t1 = trace.world_times[upper];
    const double m0 = trace.margins[upper - 1];
    const double m1 = trace.margins[upper];
    if (!std::isfinite(t0) || !std::isfinite(t1) || t1 - t0 <= 1.0e-9)
    {
      margin = m1;
      limiter = trace.limiters[upper];
      return true;
    }
    const double alpha = (world_time - t0) / (t1 - t0);
    margin = m0 + alpha * (m1 - m0);
    limiter = (alpha < 0.5) ? trace.limiters[upper - 1] : trace.limiters[upper];
    return true;
  }

  static bool sampleForecastBinary(
      const traj_utils::VisibilityForecastTrace &trace,
      const double world_time, bool &visible)
  {
    if (!trace.valid || trace.k3_world_times.empty() ||
        trace.k3_world_times.size() != trace.k3_binary_visible.size())
      return false;
    for (size_t index = 0; index < trace.k3_world_times.size(); ++index)
      if (std::abs(trace.k3_world_times[index].toSec() - world_time) < 1.0e-5)
      {
        visible = trace.k3_binary_visible[index];
        return true;
      }
    return false;
  }

  K3Assessment assessK3Relay(const double activation, const double horizon)
  {
    K3Assessment result;
    const double dt = relay_sample_dt_;
    double begin = activation;
    double end = activation + std::min(relay_horizon_, horizon);
    for (int drone = 0; drone < 3; ++drone)
    {
      const auto &trace = bundle_messages_[drone].visibility_forecast;
      if (!trace.valid || trace.k3_world_times.empty() ||
          trace.k3_world_times.size() != trace.k3_binary_visible.size())
        return result;
      begin = std::max(begin, trace.k3_world_times.front().toSec());
      end = std::min(end, trace.k3_world_times.back().toSec());
    }
    if (end < begin + dt - 1.0e-6)
      return result;
    const long long first_index = static_cast<long long>(
        std::ceil((begin - 1.0e-6) / dt));
    const long long last_index = static_cast<long long>(
        std::floor((end + 1.0e-6) / dt));
    if (last_index < first_index)
      return result;
    bool interval_closed = false;
    for (long long index = first_index; index <= last_index; ++index)
    {
      const double world_time = static_cast<double>(index) * dt;
      std::array<double, 3> margins{{0.0, 0.0, 0.0}};
      std::array<bool, 3> visible{{false, false, false}};
      std::array<int, 3> sample_limiters{{0, 0, 0}};
      int count = 0;
      for (int drone = 0; drone < 3; ++drone)
      {
        int limiter = 0;
        const auto &trace = bundle_messages_[drone].visibility_forecast;
        if (!sampleForecastBinary(trace, world_time, visible[drone]) ||
            !sampleForecastMargin(trace, world_time, margins[drone], limiter) ||
            !std::isfinite(margins[drone]))
          return K3Assessment();
        sample_limiters[drone] = limiter;
        count += visible[drone] ? 1 : 0;
      }
      ++result.baseline_total_samples;
      result.baseline_k2_samples += count >= 2 ? 1 : 0;
      result.baseline_k3_samples += count == 3 ? 1 : 0;
      const int invisible = count == 2
          ? (visible[0] ? (visible[1] ? 2 : 1) : 0) : -1;
      if (!interval_closed && invisible >= 0)
      {
        if (!result.found)
        {
          result.found = true;
          result.target = invisible;
          result.t_cross = world_time;
        }
        if (invisible == result.target)
        {
          result.t_recover = world_time;
          ++result.critical_samples;
          const double m3 = std::min(margins[0],
                                     std::min(margins[1], margins[2]));
          if (m3 < result.min_m3)
          {
            result.min_m3 = m3;
            result.t_star = world_time;
            result.margins = margins;
            result.t_star_limiter =
                sample_limiters[result.target >= 0 && result.target < 3
                                    ? result.target : 0];
            std::array<double, 3> ranked = margins;
            std::sort(ranked.begin(), ranked.end(), std::greater<double>());
            result.m2_at_star = ranked[1];
          }
        }
        else
          interval_closed = true;
      }
      else if (result.found)
        interval_closed = true;
    }
    result.valid = true;
    return result;
  }

  // Feedback098 — M2 is fused from the three Planner-side forecasts.
  //
  // The Coordinator no longer extrapolates execution_[drone].traj: those
  // polynomials are replaced by rolling Local commits every few hundred
  // milliseconds, so extrapolating them up to relay_horizon_ described a
  // future that never happens (Feedback097: coordinator M2 = -1.33 where the
  // realized geometry was margin = 1.0).  It now only fuses the traces the
  // Planner itself produced for its own current Local baseline.
  RelayAssessment assessPredictiveRelay(const double activation,
                                        const double horizon,
                                        const double now_world_time)
  {
    RelayAssessment result;
    if (!std::isfinite(activation) || !std::isfinite(horizon) ||
        horizon < relay_sample_dt_)
      return result;
    const double evaluated_horizon = std::min(relay_horizon_, horizon);

    // Common world-time interval of the three forecasts, additionally bounded
    // by the requested evaluation window.  Never extended with old data.
    const double window_begin = activation;
    const double window_end = activation + evaluated_horizon;
    double t_begin = window_begin;
    double t_end = window_end;
    std::uint64_t newest_forecast = 0;
    for (int drone = 0; drone < 3; ++drone)
    {
      const auto &trace = bundle_messages_[drone].visibility_forecast;
      if (!trace.valid)
      {
        ROS_WARN_THROTTLE(1.0,
            "[RELAY_FORECAST_INSUFFICIENT] drone=%d reason=NO_VALID_FORECAST "
            "activation=%.9f", drone, activation);
        return result;
      }
      t_begin = std::max(t_begin, trace.start_world_time.toSec());
      t_end = std::min(t_end, trace.end_world_time.toSec());
      newest_forecast = std::max(
          newest_forecast,
          static_cast<std::uint64_t>(
              trace.created_world_time.toSec() * 1.0e9));
    }
    if (!(t_end > t_begin + relay_sample_dt_) ||
        t_end - t_begin < relay_sample_dt_)
    {
      ROS_WARN_THROTTLE(1.0,
          "[RELAY_FORECAST_INSUFFICIENT] activation=%.9f common_window="
          "[%.9f,%.9f] required_dt=%.3f reason=INSUFFICIENT_COMMON_WINDOW",
          activation, t_begin, t_end, relay_sample_dt_);
      return result;
    }

    const int sample_count = std::max(
        2, static_cast<int>(std::ceil((t_end - t_begin) / relay_sample_dt_)));
    for (int sample_index = 0; sample_index <= sample_count; ++sample_index)
    {
      const double alpha = static_cast<double>(sample_index) /
                           static_cast<double>(sample_count);
      const double world_time = t_begin + alpha * (t_end - t_begin);
      std::array<std::pair<double, int>, 3> ranked;
      std::array<double, 3> margins{{0.0, 0.0, 0.0}};
      std::array<int, 3> limiters{{0, 0, 0}};
      for (int drone = 0; drone < 3; ++drone)
      {
        double margin = 0.0;
        int limiter = 0;
        if (!sampleForecastMargin(bundle_messages_[drone].visibility_forecast,
                                  world_time, margin, limiter))
        {
          ROS_WARN_THROTTLE(1.0,
              "[RELAY_FORECAST_INSUFFICIENT] drone=%d world_time=%.9f "
              "reason=SAMPLE_UNAVAILABLE", drone, world_time);
          return result;
        }
        margins[drone] = margin;
        limiters[drone] = limiter;
        ranked[drone] = std::make_pair(margin, drone);
      }
      std::sort(ranked.begin(), ranked.end(),
                [](const std::pair<double, int> &lhs,
                   const std::pair<double, int> &rhs) {
                  if (std::abs(lhs.first - rhs.first) > 1.0e-12)
                    return lhs.first > rhs.first;
                  return lhs.second < rhs.second;
                });
      if (sample_index == 0)
      {
        for (int drone = 0; drone < 3; ++drone)
          result.activation_margins[drone] = margins[drone];
        result.stable = ranked[0].second;
        result.outgoing = ranked[1].second;
        result.incoming = ranked[2].second;
      }
      // M2 is the second-largest margin at THIS world time; the UAV that
      // produced it is recorded explicitly.
      if (ranked[1].first < result.min_m2)
      {
        result.min_m2 = ranked[1].first;
        result.min_world_time = world_time;
        result.limiting_uav = ranked[1].second;
        result.limiting_margin = ranked[1].first;
        result.limiter = static_cast<ContinuousVisibilitySample::Limiter>(
            limiters[ranked[1].second]);
        for (int drone = 0; drone < 3; ++drone)
          result.critical_margins[drone] = margins[drone];
      }
      // Feedback100 earliest-crossing detection.  The FIRST contiguous run of
      // samples with M2 < trigger defines the intervention interval; later
      // crossings never extend it.
      const bool crossing = ranked[1].first < relay_trigger_margin_;
      if (crossing)
      {
        if (!result.crossing_found)
        {
          result.crossing_found = true;
          result.t_cross = world_time;
          result.t_recover = world_time;
          result.repair_candidate_count = 2;
          result.repair_candidates[0] = ranked[1].second;
          result.repair_candidates[1] = ranked[2].second;
          result.repair_candidate_margins[0] = ranked[1].first;
          result.repair_candidate_margins[1] = ranked[2].first;
          // The repair priority is the deficient channel that needs the
          // SMALLEST improvement to reach the requirement, i.e. the highest
          // margin among the two candidates.  This is read off the published
          // forecast so every node derives the same order without a protocol.
          result.repair_priority_uav = ranked[1].second;
          for (int drone = 0; drone < 3; ++drone)
            result.cross_margins[drone] = margins[drone];
        }
        else if (!result.crossing_closed)
        {
          result.t_recover = world_time;
        }
      }
      else if (result.crossing_found && !result.crossing_closed)
      {
        result.crossing_closed = true;
      }
    }
    result.forecast_created_world_time_ns = newest_forecast;
    result.valid = std::isfinite(result.min_m2) && result.outgoing >= 0 &&
                   result.incoming >= 0 && result.stable >= 0 &&
                   result.limiting_uav >= 0;
    ROS_INFO("[RELAY_FORECAST_M2] valid=%d activation=%.9f window=[%.9f,%.9f] "
             "m2=%.6f critical_world_time=%.9f limiting_uav=%d "
             "limiting_margin=%.6f limiter=%s "
             "m_uav0=%.6f m_uav1=%.6f m_uav2=%.6f "
             "stable=%d outgoing=%d incoming=%d samples=%d "
             "crossing=%d t_cross=%.9f t_recover=%.9f t_star=%.9f "
             "repair_priority_uav=%d",
             static_cast<int>(result.valid), activation, t_begin, t_end,
             result.min_m2, result.min_world_time, result.limiting_uav,
             result.limiting_margin,
             visibilityMarginLimiterName(result.limiter),
             result.critical_margins[0], result.critical_margins[1],
             result.critical_margins[2], result.stable, result.outgoing,
             result.incoming, sample_count + 1,
             static_cast<int>(result.crossing_found), result.t_cross,
             result.t_recover, result.min_world_time,
             result.repair_priority_uav);
    return result;
  }

  bool sameRelayLineage(const PersistentHandoffContract &contract,
                        const std::uint64_t task_generation,
                        const std::uint64_t target_revision,
                        const std::uint64_t dynamic_revision,
                        const std::uint64_t map_revision,
                        const std::uint64_t visibility_revision) const
  {
    // Cooperative task references roll continuously.  A newer generation
    // does not invalidate an already issued world-time visibility obligation;
    // only a reset behind the captured generation or a changed world/model
    // identity does.  Executable ownership follows only an explicit direct
    // predecessor chain maintained by executionCallback().
    bool same = contract.active &&
        task_generation >= contract.task_reference_generation &&
        contract.target_revision == target_revision &&
        contract.dynamic_revision == dynamic_revision &&
        contract.static_revision == map_revision &&
        contract.visibility_revision == visibility_revision;
    for (int drone = 0; drone < 3; ++drone)
      same = same && contract.expected_lineage_revisions[drone] ==
                         execution_generations_[drone] &&
                    contract.expected_lineage_trajectory_ids[drone] ==
                         execution_[drone].traj_id;
    return same;
  }

  bool updateRelayContract(const RelayAssessment &assessment,
                           const K3Assessment &k3,
                           const double activation,
                           const double execution_horizon,
                           const std::uint64_t task_generation,
                           const std::uint64_t target_revision,
                           const std::uint64_t dynamic_revision,
                           const std::uint64_t map_revision,
                           const std::uint64_t visibility_revision)
  {
    const double now = ros::Time::now().toSec();
    ROS_INFO("[TEAM_M2_MIN] valid=%d activation=%.9f horizon=%.3f "
             "m2=%.6f world_time=%.9f limiter=%s trigger=%.6f",
             static_cast<int>(assessment.valid), activation,
             std::min(relay_horizon_, execution_horizon), assessment.min_m2,
             assessment.min_world_time,
             visibilityMarginLimiterName(assessment.limiter),
             relay_trigger_margin_);

    if (active_handoff_contract_.active)
    {
      const auto &contract = active_handoff_contract_;
      const double window_tolerance = 0.5 * relay_sample_dt_ + 1.0e-6;
      const bool k3_window_fresh =
          contract.acquire_by >= activation - 1.0e-6 &&
          contract.preserve_until <= activation +
              std::min(relay_horizon_, execution_horizon) + 1.0e-6 &&
          k3.t_cross >= contract.acquire_by - window_tolerance &&
          k3.t_recover <= contract.preserve_until + window_tolerance;
      if (active_handoff_contract_.repair_k == 3 &&
          (!assessment.valid ||
           assessment.min_m2 < relay_trigger_margin_ ||
           !k3.valid || !k3.found ||
           k3.target != active_handoff_contract_.repair_priority_uav ||
           !k3_window_fresh))
      {
        ROS_INFO("[RELAY_K3_CLEARED] contract_id=%lu reason=%s",
                 static_cast<unsigned long>(active_handoff_contract_.id),
                 assessment.valid &&
                     assessment.min_m2 < relay_trigger_margin_
                     ? "M2_PREEMPTION"
                     : (k3.valid && k3.found &&
                        k3.target == contract.repair_priority_uav &&
                        !k3_window_fresh)
                           ? "K3_WINDOW_CHANGED"
                           : "K3_RISK_GONE_OR_IDENTITY_CHANGED");
        active_handoff_contract_ = PersistentHandoffContract();
      }
    }
    if (active_handoff_contract_.active)
    {
      const bool lineage_fresh = sameRelayLineage(
          active_handoff_contract_, task_generation, target_revision,
          dynamic_revision, map_revision, visibility_revision);
      if (!lineage_fresh || now > active_handoff_contract_.expires)
      {
        ROS_INFO("[HANDOFF_CONTRACT_STALE] id=%lu lineage_fresh=%d "
                 "now=%.9f expires=%.9f task=%lu->%lu "
                 "target=%lu->%lu dynamic=%lu->%lu static=%lu->%lu "
                 "visibility=%lu->%lu",
                 static_cast<unsigned long>(active_handoff_contract_.id),
                 static_cast<int>(lineage_fresh), now,
                 active_handoff_contract_.expires,
                 static_cast<unsigned long>(
                     active_handoff_contract_.task_reference_generation),
                 static_cast<unsigned long>(task_generation),
                 static_cast<unsigned long>(
                     active_handoff_contract_.target_revision),
                 static_cast<unsigned long>(target_revision),
                 static_cast<unsigned long>(
                     active_handoff_contract_.dynamic_revision),
                 static_cast<unsigned long>(dynamic_revision),
                 static_cast<unsigned long>(
                     active_handoff_contract_.static_revision),
                 static_cast<unsigned long>(map_revision),
                 static_cast<unsigned long>(
                     active_handoff_contract_.visibility_revision),
                 static_cast<unsigned long>(visibility_revision));
        active_handoff_contract_ = PersistentHandoffContract();
      }
      else if (active_handoff_contract_.repair_k == 2 && assessment.valid &&
               assessment.min_m2 >= relay_release_margin_)
      {
        ROS_INFO("[HANDOFF_CONTRACT_RELEASE] id=%lu m2=%.6f release=%.6f",
                 static_cast<unsigned long>(active_handoff_contract_.id),
                 assessment.min_m2, relay_release_margin_);
        active_handoff_contract_ = PersistentHandoffContract();
      }
      else
      {
        ROS_INFO("[HANDOFF_CONTRACT_REUSED] id=%lu outgoing=%d incoming=%d "
                 "stable=%d acquire=%.9f preserve=%.9f",
                 static_cast<unsigned long>(active_handoff_contract_.id),
                 active_handoff_contract_.outgoing,
                 active_handoff_contract_.incoming,
                 active_handoff_contract_.stable,
                 active_handoff_contract_.acquire_by,
                 active_handoff_contract_.preserve_until);
        return true;
      }
    }

    if (!assessment.valid || assessment.min_m2 >= relay_trigger_margin_)
      return false;
    const double horizon_end = activation +
        std::min(relay_horizon_, execution_horizon);
    // Feedback100: anchor the obligation on the FIRST threshold crossing
    // instead of on the argmin.  severity_time (t_star) stays in the contract
    // for diagnostics only.
    const double cross_begin = assessment.crossing_found
        ? assessment.t_cross : assessment.min_world_time;
    const double cross_end = assessment.crossing_found
        ? std::max(assessment.t_recover, cross_begin)
        : assessment.min_world_time;
    double acquire = std::max(activation, cross_begin);
    double preserve = std::min(horizon_end, cross_end);
    // Keep the historical minimum obligation length; the window is only ever
    // extended forward, never shrunk below the observed problem interval.
    if (preserve - acquire + 1.0e-9 < relay_overlap_)
      preserve = std::min(horizon_end, acquire + relay_overlap_);
    if (preserve - acquire + 1.0e-9 < relay_overlap_ ||
        preserve <= activation + relay_sample_dt_)
    {
      ROS_INFO("[HANDOFF_CONTRACT_NOOP] reason=INSUFFICIENT_WORLD_TIME "
               "activation=%.9f acquire=%.9f preserve=%.9f overlap=%.3f",
               activation, acquire, preserve, relay_overlap_);
      return false;
    }

    PersistentHandoffContract contract;
    contract.active = true;
    contract.repair_k = 2;
    contract.id = ++handoff_contract_serial_;
    contract.task_reference_generation = task_generation;
    contract.target_revision = target_revision;
    contract.dynamic_revision = dynamic_revision;
    contract.static_revision = map_revision;
    contract.visibility_revision = visibility_revision;
    for (int drone = 0; drone < 3; ++drone)
    {
      contract.baseline_owner_revisions[drone] = execution_generations_[drone];
      contract.baseline_owner_trajectory_ids[drone] = execution_[drone].traj_id;
      contract.expected_lineage_revisions[drone] = execution_generations_[drone];
      contract.expected_lineage_trajectory_ids[drone] =
          execution_[drone].traj_id;
    }
    contract.outgoing = assessment.outgoing;
    contract.incoming = assessment.incoming;
    contract.stable = assessment.stable;
    contract.activation = activation;
    contract.acquire_by = acquire;
    contract.preserve_until = preserve;
    contract.created = now;
    contract.expires = preserve + relay_contract_timeout_;
    contract.required_overlap = relay_overlap_;
    contract.required_margin = relay_required_margin_;
    contract.m2_before = assessment.activation_margins[assessment.outgoing];
    contract.m2_min = assessment.min_m2;
    contract.m2_min_time = assessment.min_world_time;
    contract.limiting_uav = assessment.limiting_uav;
    contract.limiting_margin = assessment.limiting_margin;
    contract.t_cross = cross_begin;
    contract.t_recover = cross_end;
    contract.t_star = assessment.min_world_time;
    for (int drone = 0; drone < 3; ++drone)
      contract.forecast_margins[drone] = assessment.critical_margins[drone];
    contract.repair_candidate_count = assessment.repair_candidate_count;
    contract.repair_candidates = assessment.repair_candidates;
    contract.repair_candidate_margins = assessment.repair_candidate_margins;
    contract.repair_priority_uav = assessment.repair_priority_uav;
    contract.detect_wall = ros::WallTime::now().toSec();
    active_handoff_contract_ = contract;
    ROS_INFO("[HANDOFF_CONTRACT_CREATED] id=%lu outgoing=%d incoming=%d "
             "stable=%d acquire=%.9f preserve=%.9f overlap=%.3f "
             "margin=%.3f m2_before=%.6f m2_min=%.6f m2_time=%.9f "
             "limiting_uav=%d limiting_margin=%.6f m_uav0=%.6f m_uav1=%.6f "
             "m_uav2=%.6f",
             static_cast<unsigned long>(contract.id), contract.outgoing,
             contract.incoming, contract.stable, contract.acquire_by,
             contract.preserve_until, contract.required_overlap,
             contract.required_margin, contract.m2_before, contract.m2_min,
             contract.m2_min_time, contract.limiting_uav,
             contract.limiting_margin, assessment.critical_margins[0],
             assessment.critical_margins[1], assessment.critical_margins[2]);
    // Feedback100: M2_LIMITING_UAV only says who currently defines M2.
    // TEAM_REPAIR_CANDIDATES is the set of channels whose restoration can
    // actually raise M2, and TEAM_REPAIR_SELECTED_UAV is who was chosen.
    ROS_INFO("[TEAM_REPAIR_OPTIONS] contract_id=%lu "
             "critical_t_cross=%.9f critical_t_recover=%.9f "
             "critical_t_star=%.9f "
             "stable_uav=%d M2_limiting_uav=%d m2_min=%.6f required=%.6f "
             "trigger=%.6f candidate_count=%d "
             "candidate_1=%d candidate_1_margin=%.6f "
             "candidate_2=%d candidate_2_margin=%.6f "
             "repair_priority_uav=%d",
             static_cast<unsigned long>(contract.id), contract.t_cross,
             contract.t_recover, contract.t_star, contract.stable,
             contract.limiting_uav, contract.m2_min, contract.required_margin,
             relay_trigger_margin_, contract.repair_candidate_count,
             contract.repair_candidates[0],
             contract.repair_candidate_margins[0],
             contract.repair_candidates[1],
             contract.repair_candidate_margins[1],
             contract.repair_priority_uav);
    return true;
  }

  bool updateK3Contract(const K3Assessment &k3,
                        const RelayAssessment &m2,
                        const double activation,
                        const double execution_horizon,
                        const std::uint64_t task_generation,
                        const std::uint64_t target_revision,
                        const std::uint64_t dynamic_revision,
                        const std::uint64_t map_revision,
                        const std::uint64_t visibility_revision)
  {
    if (!team_k3_repair_enabled_ || active_handoff_contract_.active ||
        !m2.valid || m2.min_m2 < relay_trigger_margin_ ||
        !k3.valid || !k3.found || k3.target < 0)
      return false;
    // K3 Local autonomy(第三轮 authority 清理):/alp/k3_local_escalation_enabled
    // 开启时,K3 合同只保留 [RELAY_K3_EVENT] 检测遥测,不再安装 active contract、
    // 不再驱动 Team proposal/SCP。K3 recovery 的开始与执行完全归 Local。
    if (k3_local_autonomy_enabled_)
    {
      ROS_INFO("[RELAY_K3_EVENT] contract_id=0 snapshot_id=0 "
               "target_uav=%d t_cross=%.9f t_recover=%.9f t_star=%.9f "
               "mode=TELEMETRY_ONLY reason=LOCAL_AUTONOMOUS_K3 "
               "executable_authority=LOCAL_ONLY",
               k3.target, k3.t_cross, k3.t_recover, k3.t_star);
      return false;
    }
    const double horizon_end = activation +
        std::min(relay_horizon_, execution_horizon);
    const double acquire = std::max(activation, k3.t_cross);
    // K3 repairs the detected binary loss interval.  The M2 relay's 0.5 s
    // make-before-break overlap is a different obligation and can introduce
    // additional, much deeper LOS losses after this K3 interval.  A singleton
    // binary sample owns one existing relay sample cell.
    const double preserve = std::min(horizon_end,
        std::max(k3.t_recover, acquire + relay_sample_dt_));
    if (preserve <= acquire + 1.0e-6)
      return false;
    PersistentHandoffContract contract;
    contract.active = true;
    contract.repair_k = 3;
    contract.id = ++handoff_contract_serial_;
    contract.task_reference_generation = task_generation;
    contract.target_revision = target_revision;
    contract.dynamic_revision = dynamic_revision;
    contract.static_revision = map_revision;
    contract.visibility_revision = visibility_revision;
    for (int drone = 0; drone < 3; ++drone)
    {
      contract.baseline_owner_revisions[drone] = execution_generations_[drone];
      contract.baseline_owner_trajectory_ids[drone] = execution_[drone].traj_id;
      contract.expected_lineage_revisions[drone] = execution_generations_[drone];
      contract.expected_lineage_trajectory_ids[drone] = execution_[drone].traj_id;
      contract.forecast_margins[drone] = k3.margins[drone];
    }
    // Existing relay fields remain explicit and valid; only the named target
    // receives a K3 SCP.  The M2 make-before-break obligation is not applied
    // to this mode by the final validator.
    contract.incoming = k3.target;
    for (int drone = 0; drone < 3; ++drone)
      if (drone != k3.target)
      {
        if (contract.stable < 0) contract.stable = drone;
        else contract.outgoing = drone;
      }
    contract.activation = activation;
    contract.acquire_by = acquire;
    contract.preserve_until = preserve;
    contract.created = ros::Time::now().toSec();
    contract.expires = preserve + relay_contract_timeout_;
    contract.required_overlap = preserve - acquire;
    contract.required_margin = relay_required_margin_;
    contract.m2_before = m2.activation_margins[m2.outgoing];
    contract.m2_min = m2.min_m2;
    contract.m2_min_time = m2.min_world_time;
    contract.limiting_uav = k3.target;
    contract.limiting_margin = k3.margins[k3.target];
    contract.t_cross = k3.t_cross;
    contract.t_recover = k3.t_recover;
    contract.t_star = k3.t_star;
    contract.repair_candidate_count = 1;
    contract.repair_candidates[0] = k3.target;
    contract.repair_candidate_margins[0] = k3.margins[k3.target];
    contract.repair_priority_uav = k3.target;
    contract.detect_wall = ros::WallTime::now().toSec();
    contract.detect_limiter =
        static_cast<std::uint8_t>(std::max(0, k3.t_star_limiter));
    contract.detect_k2 =
        k3.baseline_total_samples > 0
            ? static_cast<double>(k3.baseline_k2_samples) /
                  static_cast<double>(k3.baseline_total_samples)
            : 0.0;
    contract.detect_k3 =
        k3.baseline_total_samples > 0
            ? static_cast<double>(k3.baseline_k3_samples) /
                  static_cast<double>(k3.baseline_total_samples)
            : 0.0;
    active_handoff_contract_ = contract;
    ROS_INFO("[RELAY_K3_EVENT] contract_id=%lu snapshot_id=%lu "
             "target_uav=%d t_cross=%.9f t_recover=%.9f t_star=%.9f "
             "margins=[%.6f,%.6f,%.6f] M2=%.6f M3=%.6f "
             "baseline_K3_samples=%d baseline_K2_samples=%d "
             "critical_samples=%d",
             static_cast<unsigned long>(contract.id),
             static_cast<unsigned long>(m2.forecast_created_world_time_ns),
             k3.target, k3.t_cross, k3.t_recover, k3.t_star,
             k3.margins[0], k3.margins[1], k3.margins[2],
             k3.m2_at_star, k3.min_m3, k3.baseline_k3_samples,
             k3.baseline_k2_samples, k3.critical_samples);
    return true;
  }

  TopologyBundleCore toCore(
      const traj_utils::TopologyCandidateBundle &message,
      const bool early_joint_seed = false) const
  {
    TopologyBundleCore bundle;
    bundle.drone_id = message.drone_id;
    bundle.planning_generation = message.planning_generation;
    bundle.planning_epoch = message.planning_epoch.toSec();
    bundle.bundle_stamp = message.bundle_stamp.toSec();
    bundle.evaluation_start_time = message.evaluation_start_time.toSec();
    bundle.requested_evaluation_horizon =
        message.requested_evaluation_horizon;
    bundle.outer_loop_evaluation = message.outer_loop_evaluation;
    bundle.local_candidate_id = message.local_selected_candidate_id;
    bundle.local_hypothesis_id = message.local_selected_hypothesis_id;
    bundle.local_kind = message.local_selected_kind;
    bundle.target_position << message.target_position.x,
        message.target_position.y, message.target_position.z;
    bundle.target_velocity << message.target_velocity.x,
        message.target_velocity.y, message.target_velocity.z;
    for (const auto &candidate_message : message.candidates)
    {
      poly_traj::Trajectory trajectory;
      const traj_utils::MINCOTraj &trajectory_message = early_joint_seed
          ? candidate_message.joint_seed : candidate_message.trajectory;
      if (early_joint_seed &&
          (!candidate_message.joint_seed_valid ||
           !candidate_message.joint_seed_static_constructible ||
           !candidate_message.joint_seed_local_sfc_valid))
        continue;
      if (!reconstructTrajectory(trajectory_message, trajectory))
        continue;
      const auto trajectory_ptr =
          std::make_shared<poly_traj::Trajectory>(trajectory);
      const auto offsets = early_joint_seed
          ? candidate_message.joint_seed_visibility_sample_offsets
          : candidate_message.visibility_sample_offsets;
      const auto states = early_joint_seed
          ? candidate_message.joint_seed_self_visibility_samples
          : candidate_message.self_visibility_samples;
      const bool visibility_valid = early_joint_seed
          ? candidate_message.joint_seed_binary_visibility_valid
          : candidate_message.binary_visibility_valid;
      if (!visibility_valid || offsets.size()<2 || offsets.size()!=states.size()) continue;
      bool trace_valid=std::abs(offsets.front())<=1e-6;
      for(size_t k=0;k<offsets.size();++k)
        trace_valid=trace_valid && std::isfinite(offsets[k]) && states[k]<=1 && (k==0 || offsets[k]>offsets[k-1]);
      if(!trace_valid) continue;
      TopologyCandidateCore candidate;
      candidate.drone_id = message.drone_id;
      candidate.candidate_id = candidate_message.candidate_id;
      candidate.encirclement_hypothesis_id =
          candidate_message.encirclement_hypothesis_id;
      candidate.encirclement_phi0 = candidate_message.encirclement_phi0;
      candidate.encirclement_generation =
          candidate_message.encirclement_generation;
      candidate.kind = candidate_message.candidate_kind;
      // The core's executable bit is used here only as a cheap tuple
      // construction predicate.  A JOINT_SEED is never an executable
      // trajectory: full dynamics/static/dynamic/swarm/Local-SFC checks are
      // performed by the optimizer and again independently by all planners.
      candidate.safety_class = early_joint_seed
          ? traj_utils::TopologyCandidate::SAFETY_ABSOLUTE_SAFE
          : candidate_message.safety_class;
      candidate.success = early_joint_seed ? true : candidate_message.success;
      candidate.static_valid = early_joint_seed
          ? candidate_message.joint_seed_static_constructible
          : candidate_message.static_valid;
      candidate.dynamics_valid = early_joint_seed ? true
                                                   : candidate_message.dynamics_valid;
      candidate.swarm_valid = early_joint_seed ? true
                                                : candidate_message.swarm_valid;
      candidate.conflict_reason_mask = candidate_message.conflict_reason_mask;
      candidate.los_conflict_valid = candidate_message.los_conflict_valid;
      candidate.los_obstacle_identity =
          candidate_message.los_obstacle_identity;
      candidate.los_conflict_time = candidate_message.los_conflict_time.toSec();
      if (candidate.los_conflict_valid)
        ROS_INFO("[los-provenance-import] drone=%d candidate=%d reason_mask=%u "
                 "los_blocker=%d los_time=%.6f mode=%s",
                 message.drone_id, candidate.candidate_id,
                 static_cast<unsigned int>(candidate.conflict_reason_mask),
                 candidate.los_obstacle_identity, candidate.los_conflict_time,
                 early_joint_seed ? "JOINT_SEED" : "TRAJECTORY");
      candidate.start_time = early_joint_seed
          ? candidate_message.joint_seed_start_time.toSec()
          : candidate_message.trajectory_start_time.toSec();
      candidate.duration = trajectory.getTotalDuration();
      candidate.evaluation_horizon = std::min(candidate_message.evaluation_horizon,offsets.back());
      candidate.dynamic_clearance = early_joint_seed
          ? candidate_message.joint_seed_dynamic_clearance
          : candidate_message.dynamic_clearance;
      candidate.tracking_score = candidate_message.tracking_score;
      candidate.native_cost = candidate_message.native_cost;
      const double start_time = candidate.start_time;
      const double duration = candidate.duration;
      candidate.position_at_global_time =
          [trajectory_ptr, start_time, duration](const double global_time) {
            const double local_time =
                std::max(0.0, std::min(duration, global_time - start_time));
            return trajectory_ptr->getPos(local_time);
          };
      candidate.visible_at_global_time =
          [offsets, states, start_time](const double global_time) {
            if (offsets.empty() || offsets.size() != states.size())
              return false;
            const double local_time = global_time - start_time;
            const auto iterator =
                std::lower_bound(offsets.begin(), offsets.end(), local_time);
            size_t index = 0;
            if (iterator == offsets.end())
              index = offsets.size() - 1;
            else if (iterator == offsets.begin())
              index = 0;
            else
            {
              const size_t upper = static_cast<size_t>(iterator - offsets.begin());
              index = std::abs(offsets[upper] - local_time) <
                              std::abs(offsets[upper - 1] - local_time)
                          ? upper
                          : upper - 1;
            }
            return states[index] != 0;
          };
      bundle.candidates.push_back(candidate);
    }
    return bundle;
  }

  void bundleCallback(const traj_utils::TopologyCandidateBundleConstPtr &message,
                      const int expected_drone)
  {
    if (!message || message->drone_id != expected_drone)
    {
      ROS_WARN("[topology-bundle-reject] expected_drone=%d received_drone=%d "
               "reason=DRONE_ID_MISMATCH",
               expected_drone, message ? message->drone_id : -1);
      return;
    }
    if (message->planning_generation <= last_evaluated_generation_[expected_drone])
      return; // This identity already received a terminal decision/proposal.
    const double now = ros::Time::now().toSec();
    std::uint64_t encirclement_generation = 0;
    if (!message->candidates.empty())
      encirclement_generation =
          message->candidates.front().encirclement_generation;
    if (message->outer_loop_evaluation)
    {
      if (!outer_snapshot_active_ ||
          encirclement_generation != outer_snapshot_generation_)
      {
        outer_snapshot_active_ = true;
        outer_snapshot_generation_ = encirclement_generation;
        outer_snapshot_started_ = now;
        bundle_valid_ = {{false, false, false}};
      }
    }
    else if (outer_snapshot_active_)
    {
      if (now - outer_snapshot_started_ <= max_bundle_age_)
      {
        ROS_INFO_THROTTLE(
            0.2,
            "[topology-bundle-recv] drone=%d generation=%lu "
            "outer_snapshot_generation=%lu action=INNER_LOCAL_FALLBACK "
            "reason=PRESERVE_PENDING_OUTER_SNAPSHOT",
            expected_drone,
            static_cast<unsigned long>(message->planning_generation),
            static_cast<unsigned long>(outer_snapshot_generation_));
        return;
      }
      outer_snapshot_active_ = false;
      bundle_valid_ = {{false, false, false}};
    }

    bundles_[expected_drone] = toCore(*message);
    early_seed_bundles_[expected_drone] = toCore(*message, true);
    bundle_messages_[expected_drone] = *message;
    bundle_valid_[expected_drone] = true;
    ROS_INFO("[topology-bundle-recv] drone=%d generation=%lu candidate_count=%zu "
             "planning_epoch=%.9f bundle_stamp=%.9f outer_loop=%d "
             "T_EVAL_START=%.9f H_EVAL_REQUESTED=%.6f",
             expected_drone,
             static_cast<unsigned long>(message->planning_generation),
             bundles_[expected_drone].candidates.size(),
             message->planning_epoch.toSec(), message->bundle_stamp.toSec(),
             static_cast<int>(message->outer_loop_evaluation),
             message->evaluation_start_time.toSec(),
             message->requested_evaluation_horizon);
    evaluateSnapshot();
  }

  bool hasCompleteSnapshot() const
  {
    for (int drone = 0; drone < 3; ++drone)
      if (!bundle_valid_[drone])
        return false;
    return true;
  }

  void consumeCurrentSnapshot()
  {
    for (int drone = 0; drone < 3; ++drone)
    {
      last_evaluated_generation_[drone] = bundles_[drone].planning_generation;
      bundle_valid_[drone] = false;
    }
    outer_snapshot_active_ = false;
  }

  const traj_utils::TopologyCandidate *selectedCandidateMessage(
      const int drone, const TopologySelectionCore &selection) const
  {
    for (const auto &candidate : bundle_messages_[drone].candidates)
      if (candidate.candidate_id == selection.joint.candidate_ids[drone] &&
          candidate.encirclement_hypothesis_id ==
              selection.joint.encirclement_hypothesis_id &&
          candidate.encirclement_generation ==
              selection.joint.encirclement_generation)
        return &candidate;
    return nullptr;
  }

  Eigen::Vector3d targetAtWorldTime(const double world_time) const
  {
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    for (int drone = 0; drone < 3; ++drone)
    {
      const auto &bundle = bundle_messages_[drone];
      const Eigen::Vector3d position(bundle.target_position.x,
                                     bundle.target_position.y,
                                     bundle.target_position.z);
      const Eigen::Vector3d velocity(bundle.target_velocity.x,
                                     bundle.target_velocity.y,
                                     bundle.target_velocity.z);
      target += position + velocity *
          (world_time - bundle.target_snapshot_epoch.toSec());
    }
    return target / 3.0;
  }

  ContinuousVisibilitySample continuousVisibility(
      const int drone, const Eigen::Vector3d &observer,
      const Eigen::Vector3d &target, const double world_time,
      const double predicted_yaw) const
  {
    (void)drone;
    ContinuousVisibilitySample result;
    if (!observer.allFinite() || !target.allFinite() ||
        !std::isfinite(world_time) || !std::isfinite(predicted_yaw))
      return result;
    const Eigen::Vector3d delta = observer - target;
    const double range = delta.norm();
    if (!std::isfinite(range) || range <= 1.0e-9)
      return result;
    double value = 1.0;
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    bool valid = true;
    bool binary_valid = true;
    bool binary_los_clear = true;
    const Eigen::Vector3d range_gradient = delta / range;
    std::array<VisibilityRiskComponent, 4> margin_components;
    margin_components[0].limiter =
        ContinuousVisibilitySample::LIMITER_STATIC;
    margin_components[1].limiter =
        ContinuousVisibilitySample::LIMITER_DYNAMIC_LOS;
    margin_components[2].limiter =
        ContinuousVisibilitySample::LIMITER_FOV;
    margin_components[3].limiter =
        ContinuousVisibilitySample::LIMITER_RANGE;
    margin_components[1].valid = true;  // no moving object means zero risk

    const DirectionalRiskResult near_range_risk = directionalClearanceRisk(
        range - tracking_camera_.min_range, 0.0,
        std::max(1.0e-3, visibility_range_smoothing_));
    const DirectionalRiskResult far_range_risk = directionalClearanceRisk(
        tracking_camera_.max_range - range, 0.0,
        std::max(1.0e-3, visibility_range_smoothing_));
    if (near_range_risk.valid && far_range_risk.valid)
    {
      const bool near_limits = near_range_risk.value >= far_range_risk.value;
      const DirectionalRiskResult &risk =
          near_limits ? near_range_risk : far_range_risk;
      margin_components[3].valid = true;
      margin_components[3].risk = risk.value;
      const Eigen::Vector3d clearance_gradient =
          near_limits ? Eigen::Vector3d(range_gradient)
                      : Eigen::Vector3d(-range_gradient);
      margin_components[3].gradient_position =
          risk.derivative_clearance * clearance_gradient;
    }
    multiplyScore(smoothIncreasingScore(
                      range - tracking_camera_.min_range, 0.0,
                      visibility_range_smoothing_),
                  range_gradient, value, gradient, valid);
    multiplyScore(smoothIncreasingScore(
                      tracking_camera_.max_range - range, 0.0,
                      visibility_range_smoothing_),
                  -range_gradient, value, gradient, valid);

    double static_clearance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d static_gradient = Eigen::Vector3d::Zero();
    if (static_geometry_.querySegmentClearance(
            observer, target, static_clearance, &static_gradient, nullptr,
            nullptr))
    {
      binary_los_clear = binary_los_clear &&
                         static_clearance >= visibility_occlusion_margin_;
      multiplyScore(smoothIncreasingScore(
                        static_clearance, visibility_occlusion_margin_,
                        visibility_occlusion_margin_ +
                            visibility_los_smoothing_),
                    static_gradient, value, gradient, valid);
      const DirectionalRiskResult risk = directionalClearanceRisk(
          static_clearance, visibility_occlusion_margin_,
          visibility_occlusion_margin_ + visibility_los_smoothing_);
      if (risk.valid)
      {
        margin_components[0].valid = true;
        margin_components[0].risk = risk.value;
        margin_components[0].gradient_position =
            risk.derivative_clearance * static_gradient;
      }
    }
    else
      valid = binary_valid = false;

    if (object_predictor_)
    {
      for (int object = 0; object < object_predictor_->getObjNums(); ++object)
      {
        if (!object_predictor_->hasPrediction(object))
        {
          valid = false;
          binary_valid = false;
          margin_components[1].valid = false;
          break;
        }
        const Eigen::Vector3d center =
            object_predictor_->evaluateConstVel(object, world_time);
        const Eigen::Vector3d scale = object_predictor_->getObjScale(object);
        if (!center.allFinite() || !scale.allFinite())
        {
          valid = false;
          binary_valid = false;
          margin_components[1].valid = false;
          break;
        }
        const double radius = 0.5 * std::max(scale.x(), scale.y());
        const SegmentCylinderClearanceGradient clearance =
            segmentVerticalCylinderClearanceGradient(
                observer, target, center, radius, scale.z());
        if (!clearance.valid)
        {
          valid = false;
          binary_valid = false;
          margin_components[1].valid = false;
          break;
        }
        if (std::isfinite(clearance.clearance))
        {
          binary_los_clear = binary_los_clear &&
                             clearance.clearance >= visibility_occlusion_margin_;
          multiplyScore(smoothIncreasingScore(
                            clearance.clearance,
                            visibility_occlusion_margin_,
                            visibility_occlusion_margin_ +
                                visibility_los_smoothing_),
                        clearance.gradient_start, value, gradient, valid);
          const DirectionalRiskResult risk = directionalClearanceRisk(
              clearance.clearance, visibility_occlusion_margin_,
              visibility_occlusion_margin_ + visibility_los_smoothing_);
          if (risk.valid &&
              risk.value > margin_components[1].risk + 1.0e-12)
          {
            margin_components[1].risk = risk.value;
            margin_components[1].gradient_position =
                risk.derivative_clearance * clearance.gradient_start;
          }
        }
      }
    }

    const CameraFovDirectionalRisk fov =
        trackingCameraFovDirectionalRisk(observer, predicted_yaw, target,
                                         tracking_camera_);
    if (!fov.valid)
      valid = false;
    else
    {
      margin_components[2].valid = true;
      margin_components[2].risk = fov.risk;
      margin_components[2].gradient_position = fov.gradient_body_position;
      margin_components[2].gradient_yaw = fov.gradient_yaw;
      SmoothScore fov_score;
      fov_score.valid = true;
      fov_score.value = std::exp(-std::max(0.0, fov.risk));
      fov_score.derivative = -fov_score.value;
      const double visibility_before_fov = value;
      result.gradient_yaw = visibility_before_fov *
                            fov_score.derivative * fov.gradient_yaw;
      multiplyScore(fov_score, fov.gradient_body_position, value, gradient,
                    valid);
    }
    result.valid = valid && std::isfinite(value) && gradient.allFinite();
    result.value = std::max(0.0, std::min(1.0, value));
    result.gradient_position = gradient;
    const Eigen::Quaterniond world_from_body(
        Eigen::AngleAxisd(predicted_yaw, Eigen::Vector3d::UnitZ()));
    const CameraFovResult binary_fov = targetInTrackingCameraFov(
        observer, world_from_body, target, tracking_camera_);
    result.binary_valid = binary_valid &&
        std::isfinite(binary_fov.range) &&
        std::isfinite(binary_fov.horizontal_angle) &&
        std::isfinite(binary_fov.vertical_angle);
    result.binary_visible = result.binary_valid && binary_los_clear &&
                            binary_fov.valid;
    composeVisibilityMargin(margin_components, result);
    return result;
  }

  bool attemptTargetCenteredTeamReference(
      const TopologySelectionCore &selection,
      const traj_utils::TopologyCoordination &legacy)
  {
    if (!enable_team_visibility_optimizer_ || !enable_joint_pt_optimization_ ||
        !ablation_config_.team_pt || !team_runtime_ready_ ||
        !target_centered_optimizer_ || selection.top_k.empty())
      return false;

    traj_utils::incrementAblationCounter(
        traj_utils::AblationCounter::TEAM_PT_ATTEMPT,
        "topology_coordinator", ablation_config_);
    const double now = ros::Time::now().toSec();
    const double total_opt_budget = target_centered_params_.timeout_seconds *
        static_cast<double>(selection.top_k.size());
    const double prepare_budget = preparation_latency_.budget();
    std::array<CommittedPrefixMember, 3> committed;
    for (int drone = 0; drone < 3; ++drone)
    {
      committed[drone].drone_id = drone;
      committed[drone].owner_revision = execution_generations_[drone];
      committed[drone].owner_trajectory_id = execution_[drone].traj_id;
      committed[drone].safety_validated =
          bundle_messages_[drone].committed_safety_validated &&
          execution_[drone].traj.getPieceNum() > 0;
      committed[drone].committed_start = execution_[drone].start_time;
      committed[drone].validated_end = std::min(
          bundle_messages_[drone].validated_end.toSec(),
          execution_[drone].start_time + execution_[drone].duration);
    }
    const CommittedPrefixSnapshot prefix = discoverCommittedFutureFrontier(
        committed, now, prepare_budget, total_opt_budget,
        team_reference_realization_timeout_, team_activation_lead_);
    if (!prefix.valid)
    {
      ROS_INFO("[TEAM_REFERENCE_NOOP] reason=NO_COMMON_LOCAL_FRONTIER");
      return false;
    }
    const double activation = prefix.frontier;
    double execution_horizon = std::numeric_limits<double>::infinity();
    for (const auto &member : committed)
      execution_horizon = std::min(
          execution_horizon, member.validated_end - activation);
    if (!std::isfinite(execution_horizon) || execution_horizon < 0.20)
    {
      ROS_INFO("[TEAM_REFERENCE_NOOP] reason=LOCAL_EXECUTION_HORIZON_TOO_SHORT "
               "H_exec=%.6f", execution_horizon);
      return false;
    }

    const std::uint64_t target_revision =
        bundle_messages_[0].target_snapshot_identity;
    const std::uint64_t dynamic_revision =
        bundle_messages_[0].dynamic_prediction_identity;
    const std::uint64_t map_revision = bundle_messages_[0].static_map_revision;
    const std::uint64_t visibility_version =
        bundle_messages_[0].visibility_model_version;
    const std::uint64_t coordinator_visibility_version =
        trackingCameraContractVersion(tracking_camera_);
    for (int drone = 0; drone < 3; ++drone)
    {
      const auto &bundle = bundle_messages_[drone];
      if (target_revision == 0 ||
          bundle.target_snapshot_identity != target_revision ||
          bundle.dynamic_prediction_identity != dynamic_revision ||
          bundle.static_map_revision != map_revision ||
          bundle.visibility_model_version != visibility_version ||
          bundle.visibility_model_version !=
              coordinator_visibility_version ||
          bundle.dynamic_prediction_valid_from.toSec() > activation + 1.0e-9 ||
          bundle.dynamic_prediction_valid_to.toSec() + 1.0e-9 <
              activation + target_centered_params_.prediction_horizon)
      {
        ROS_INFO("[TEAM_REFERENCE_NOOP] reason=PREDICTION_CONTEXT_NOT_SHARED "
                 "drone=%d target_revision=%lu dynamic_revision=%lu",
                 drone, static_cast<unsigned long>(target_revision),
                 static_cast<unsigned long>(dynamic_revision));
        return false;
      }
    }

    TeamTargetCenteredEnvironment environment;
    environment.target_at_world_time = [this](const double world_time) {
      return targetAtWorldTime(world_time);
    };
    environment.visibility = [this](
        const int drone, const Eigen::Vector3d &observer,
        const Eigen::Vector3d &target, const double world_time,
        const double yaw) {
      return continuousVisibility(drone, observer, target, world_time, yaw);
    };

    // 阶段 B：任务几何唯一 authority = cooperative_viewpoint_manager 发布的
    // CooperativeTaskReference。没有新鲜任务参考时 Team 本轮不做优化
    // （Local first-safe commit 不受影响）；Team 绝不自带一套任务几何。
    traj_utils::CooperativeTaskReference task_reference;
    if (!currentTaskReference(task_reference))
    {
      ROS_INFO_THROTTLE(
          2.0, "[TEAM_REFERENCE_NOOP] reason=TASK_REFERENCE_MISSING_OR_STALE");
      return false;
    }
    const std::uint64_t task_reference_generation = task_reference.generation;
    const RelayAssessment relay_assessment = assessPredictiveRelay(
        activation, execution_horizon, ros::Time::now().toSec());
    const K3Assessment k3_assessment =
        team_k3_repair_enabled_ && relay_assessment.valid &&
                relay_assessment.min_m2 >= relay_trigger_margin_
            ? assessK3Relay(activation, execution_horizon)
            : K3Assessment();
    bool handoff_contract_active = updateRelayContract(
        relay_assessment, k3_assessment, activation, execution_horizon,
        task_reference_generation, target_revision, dynamic_revision,
        map_revision, visibility_version);
    if (!handoff_contract_active)
      handoff_contract_active = updateK3Contract(
          k3_assessment, relay_assessment, activation, execution_horizon,
          task_reference_generation, target_revision, dynamic_revision,
          map_revision, visibility_version);

    TeamTargetCenteredResult best;
    TopologyCombinationMetrics best_tuple;
    bool found = false;
    const ros::WallTime optimization_started = ros::WallTime::now();
    const int retained = std::min(
        static_cast<int>(selection.top_k.size()), target_centered_params_.top_k);
    ROS_INFO("[TEAM_TUPLE_TOPK] enumerated=%d retained=%d configured=%d",
             selection.combination_count, retained,
             target_centered_params_.top_k);
    for (int tuple_index = 0; tuple_index < retained; ++tuple_index)
    {
      const TopologyCombinationMetrics &tuple = selection.top_k[tuple_index];
      TeamTargetCenteredInput input;
      input.evaluation_start_world = activation;
      input.prediction_horizon = target_centered_params_.prediction_horizon;
      input.target_prediction_revision = target_revision;
      bool input_valid = true;
      for (int drone = 0; drone < 3 && input_valid; ++drone)
      {
        const traj_utils::TopologyCandidate *candidate = nullptr;
        for (const auto &value : bundle_messages_[drone].candidates)
          if (value.candidate_id == tuple.candidate_ids[drone] &&
              value.candidate_kind == tuple.kinds[drone] &&
              value.encirclement_hypothesis_id ==
                  tuple.encirclement_hypothesis_id &&
              value.encirclement_generation == tuple.encirclement_generation)
          {
            candidate = &value;
            break;
          }
        poly_traj::Trajectory seed;
        if (!candidate || !candidate->joint_seed_valid ||
            !candidate->joint_seed_static_constructible ||
            !candidate->joint_seed_local_sfc_valid ||
            !reconstructTrajectory(candidate->joint_seed, seed))
        {
          input_valid = false;
          break;
        }
        auto &member = input.members[drone];
        member.drone_id = drone;
        member.source_candidate_id = candidate->candidate_id;
        member.topology_kind = candidate->candidate_kind;
        if (!candidate->predicted_yaw_sample_offsets.empty() &&
            candidate->predicted_yaw_sample_offsets.size() ==
                candidate->predicted_yaw_samples.size())
        {
          const double yaw_time = activation -
                                  candidate->trajectory_start_time.toSec();
          const double rate_dt = 0.02;
          member.initial_yaw = sampleYawTrace(
              candidate->predicted_yaw_sample_offsets,
              candidate->predicted_yaw_samples, yaw_time);
          const double next_yaw = sampleYawTrace(
              candidate->predicted_yaw_sample_offsets,
              candidate->predicted_yaw_samples, yaw_time + rate_dt);
          double delta = next_yaw - member.initial_yaw;
          while (delta > M_PI) delta -= 2.0 * M_PI;
          while (delta < -M_PI) delta += 2.0 * M_PI;
          member.initial_yaw_rate = delta / rate_dt;
          member.initial_yaw_valid =
              std::isfinite(member.initial_yaw) &&
              std::isfinite(member.initial_yaw_rate);
        }
        const double seed_start = candidate->joint_seed_start_time.toSec();
        const double seed_duration = seed.getTotalDuration();
        const int knots = target_centered_params_.reference_knot_count;
        for (int knot = 0; knot < knots; ++knot)
        {
          TeamReferenceKnot value;
          value.world_time = activation + input.prediction_horizon *
              static_cast<double>(knot) / static_cast<double>(knots - 1);
          const double seed_time = std::max(
              0.0, std::min(seed_duration, value.world_time - seed_start));
          const Eigen::Vector3d target = targetAtWorldTime(value.world_time);
          if (!cartesianToTargetCentered(seed.getPos(seed_time), target,
                                         value.radius, value.bearing,
                                         value.height))
          {
            input_valid = false;
            break;
          }
          member.knots.push_back(value);
        }
        if (!input_valid)
          break;
        // 阶段 B：baseline 语义 = r_i(t) = r_i*(t) + Δr_i(t)。
        // r*/φ*/h* 全部来自 CooperativeTaskReference（已经过 slot 映射）；
        // candidate seed 只作为 knots 的数值初始化，绝不能反向定义任务期望。
        member.desired_radius = task_reference.desired_radius[drone];
        member.desired_bearing = task_reference.desired_bearing[drone];
        member.fixed_height = task_reference.desired_height[drone];
        for (auto &knot : member.knots)
          knot.height = member.fixed_height;
      }
      if (!input_valid ||
          !TeamTargetCenteredOptimizer::validScheduleIdentity(input))
        continue;

      TeamTargetCenteredParams per_tuple_params = target_centered_params_;
      per_tuple_params.timeout_seconds = std::max(
          0.005, total_opt_budget / static_cast<double>(retained));
      TeamTargetCenteredOptimizer per_tuple_optimizer(per_tuple_params);
      TeamTargetCenteredResult optimized =
          per_tuple_optimizer.optimize(input, environment);
      ROS_INFO("[TEAM_TARGET_CENTERED_OPT] tuple=%d/%d success=%d "
               "candidate_ids=%d,%d,%d kinds=%d,%d,%d "
               "objective_before=%.9f objective_after=%.9f "
               "J_K2=%.9f J_ACC=%.9f J_COMP=%.9f J_ENC=%.9f J_REG=%.9f "
               "weighted=%.9f,%.9f,%.9f,%.9f,%.9f "
               "reference_phase_shift=%.6f,%.6f,%.6f "
               "simultaneous_loss=%.6f->%.6f "
               "solve_ms=%.3f reason=%s",
               tuple_index + 1, retained, static_cast<int>(optimized.success),
               tuple.candidate_ids[0], tuple.candidate_ids[1],
               tuple.candidate_ids[2], tuple.kinds[0], tuple.kinds[1],
               tuple.kinds[2], optimized.before.objective,
               optimized.after.objective, optimized.after.raw.k2,
               optimized.after.raw.accumulated,
               optimized.after.raw.complementarity,
               optimized.after.raw.encirclement,
               optimized.after.raw.regularization,
               optimized.after.weighted.k2,
               optimized.after.weighted.accumulated,
               optimized.after.weighted.complementarity,
               optimized.after.weighted.encirclement,
               optimized.after.weighted.regularization,
               optimized.reference_phase_shifts[0],
               optimized.reference_phase_shifts[1],
               optimized.reference_phase_shifts[2],
               optimized.before.simultaneous_los_loss_duration,
               optimized.after.simultaneous_los_loss_duration,
               optimized.solve_time_ms, optimized.reason.c_str());
      if (optimized.primary_visibility_rejected)
        ++primary_visibility_reject_count_;
      if (optimized.complementarity_rejected)
        ++complementarity_reject_count_;
      if (optimized.stage1_fallback)
        ++stage1_fallback_count_;
      if (optimized.stage2_fallback)
        ++stage2_fallback_count_;
      if (optimized.stage3_fallback)
        ++stage3_fallback_count_;
      ROS_INFO("[team-hierarchy] scope=tuple tuple=%d/%d "
               "stage1_q2_base=%.9f stage1_q2_best=%.9f "
               "stage1_acc_base=%.9f stage1_acc_best=%.9f "
               "q_floor=%.9f v_floor=%.9f "
               "stage2_comp_base=%.9f stage2_comp_best=%.9f "
               "c_floor=%.9f stage3_enc_before=%.9f "
               "stage3_enc_after=%.9f stage3_reg_before=%.9f "
               "stage3_reg_after=%.9f final_stage=%s final_reason=%s "
               "latency_q2_ms=%.3f latency_acc_ms=%.3f "
               "latency_comp_ms=%.3f latency_enc_ms=%.3f total_ms=%.3f",
               tuple_index + 1, retained,
               optimized.before.quality.q2_ratio,
               optimized.stage1_q2.quality.q2_ratio,
               optimized.before.quality.accumulated_visibility_ratio,
               optimized.stage1_acc.quality.accumulated_visibility_ratio,
               optimized.q_floor, optimized.visibility_floor,
               optimized.before.quality.complementarity,
               optimized.stage2_comp.quality.complementarity,
               optimized.complementarity_floor,
               optimized.stage2_comp.quality.encirclement_cost,
               optimized.stage3_enc.quality.encirclement_cost,
               optimized.stage2_comp.quality.regularization_cost,
               optimized.stage3_enc.quality.regularization_cost,
               teamOptimizationStageName(optimized.final_stage),
               optimized.reason.c_str(), optimized.stage_q2_latency_ms,
               optimized.stage_acc_latency_ms,
               optimized.stage_comp_latency_ms,
               optimized.stage_enc_latency_ms, optimized.solve_time_ms);
      if (optimized.success &&
          (!found || betterTeamQuality(
              optimized.after.quality, best.after.quality,
              optimized.tolerance)))
      {
        found = true;
        best = optimized;
        best_tuple = tuple;
      }
    }
    const double total_opt_ms =
        (ros::WallTime::now() - optimization_started).toSec() * 1000.0;
    if (!found)
    {
      ROS_INFO("[TEAM_REFERENCE_NOOP] reason=NO_TOPK_OBJECTIVE_IMPROVEMENT "
               "top_k=%d total_opt_ms=%.3f", retained, total_opt_ms);
      return false;
    }

    // Defense at the publication boundary: even a numerically successful
    // lower stage cannot turn into a Team proposal if primary visibility is
    // worse than this tuple's own Local source baseline.
    if (!teamPrimaryVisibilityNonWorsening(
            best.after.quality, best.before.quality, best.tolerance))
    {
      ++primary_visibility_reject_count_;
      ROS_INFO("[TEAM_QUALITY_REJECT] reason=PRIMARY_VISIBILITY_WORSENED "
               "q2_base=%.9f q2_team=%.9f acc_base=%.9f acc_team=%.9f",
               best.before.quality.q2_ratio, best.after.quality.q2_ratio,
               best.before.quality.accumulated_visibility_ratio,
               best.after.quality.accumulated_visibility_ratio);
      return false;
    }
    switch (best.final_stage)
    {
    case TeamOptimizationStage::VISIBILITY_Q2:
      ++final_stage_q2_count_;
      break;
    case TeamOptimizationStage::VISIBILITY_ACC:
      ++final_stage_acc_count_;
      break;
    case TeamOptimizationStage::COMPLEMENTARITY:
      ++final_stage_comp_count_;
      break;
    case TeamOptimizationStage::ENCIRCLEMENT_REGULARIZATION:
      ++final_stage_enc_count_;
      break;
    default:
      break;
    }
    ROS_INFO("[team-hierarchy] scope=selected tuple=%d,%d,%d "
             "stage1_q2_base=%.9f stage1_q2_best=%.9f "
             "stage1_acc_base=%.9f stage1_acc_best=%.9f "
             "q_floor=%.9f v_floor=%.9f "
             "stage2_comp_base=%.9f stage2_comp_best=%.9f "
             "c_floor=%.9f stage3_enc_before=%.9f "
             "stage3_enc_after=%.9f stage3_reg_before=%.9f "
             "stage3_reg_after=%.9f final_stage=%s final_reason=%s",
             best_tuple.candidate_ids[0], best_tuple.candidate_ids[1],
             best_tuple.candidate_ids[2],
             best.before.quality.q2_ratio,
             best.stage1_q2.quality.q2_ratio,
             best.before.quality.accumulated_visibility_ratio,
             best.stage1_acc.quality.accumulated_visibility_ratio,
             best.q_floor, best.visibility_floor,
             best.before.quality.complementarity,
             best.stage2_comp.quality.complementarity,
             best.complementarity_floor,
             best.stage2_comp.quality.encirclement_cost,
             best.stage3_enc.quality.encirclement_cost,
             best.stage2_comp.quality.regularization_cost,
             best.stage3_enc.quality.regularization_cost,
             teamOptimizationStageName(best.final_stage),
             best.reason.c_str());
    ROS_INFO("[TEAM_HIERARCHY_COUNTERS] "
             "PRIMARY_VISIBILITY_REJECT_COUNT=%lu "
             "COMPLEMENTARITY_REJECT_COUNT=%lu "
             "STAGE1_FALLBACK_COUNT=%lu STAGE2_FALLBACK_COUNT=%lu "
             "STAGE3_FALLBACK_COUNT=%lu FINAL_STAGE_Q2_COUNT=%lu "
             "FINAL_STAGE_ACC_COUNT=%lu FINAL_STAGE_COMP_COUNT=%lu "
             "FINAL_STAGE_ENC_COUNT=%lu",
             static_cast<unsigned long>(primary_visibility_reject_count_),
             static_cast<unsigned long>(complementarity_reject_count_),
             static_cast<unsigned long>(stage1_fallback_count_),
             static_cast<unsigned long>(stage2_fallback_count_),
             static_cast<unsigned long>(stage3_fallback_count_),
             static_cast<unsigned long>(final_stage_q2_count_),
             static_cast<unsigned long>(final_stage_acc_count_),
             static_cast<unsigned long>(final_stage_comp_count_),
             static_cast<unsigned long>(final_stage_enc_count_));

    traj_utils::TeamReferenceSchedule schedule;
    schedule.state = traj_utils::TeamReferenceSchedule::STATE_PROPOSAL;
    schedule.valid = true;
    schedule.team_reference_id = ++team_solution_serial_;
    // 阶段 B：绑定产生本 proposal 的 CooperativeTaskReference.generation。
    schedule.task_reference_generation = task_reference_generation;
    schedule.coordination_generation = legacy.coordination_generation;
    schedule.target_prediction_revision = target_revision;
    // Identity names the common target prediction producer/model.  Epoch is
    // the newest member sample admitted by the bounded-skew context.
    double shared_target_epoch = 0.0;
    for (int drone = 0; drone < 3; ++drone)
      shared_target_epoch = std::max(
          shared_target_epoch,
          bundle_messages_[drone].target_snapshot_epoch.toSec());
    schedule.target_snapshot_epoch = ros::Time(shared_target_epoch);
    schedule.dynamic_prediction_identity = dynamic_revision;
    double dynamic_from = 0.0;
    double dynamic_to = std::numeric_limits<double>::infinity();
    for (int drone = 0; drone < 3; ++drone)
    {
      dynamic_from = std::max(
          dynamic_from,
          bundle_messages_[drone].dynamic_prediction_valid_from.toSec());
      dynamic_to = std::min(
          dynamic_to,
          bundle_messages_[drone].dynamic_prediction_valid_to.toSec());
    }
    schedule.dynamic_prediction_valid_from = ros::Time(dynamic_from);
    schedule.dynamic_prediction_valid_to = ros::Time(dynamic_to);
    schedule.static_map_revision = map_revision;
    schedule.visibility_model_version = visibility_version;
    schedule.hypothesis_id = best_tuple.encirclement_hypothesis_id;
    schedule.encirclement_generation = best_tuple.encirclement_generation;
    schedule.snapshot_stamp = ros::Time::now();
    schedule.evaluation_start_world = ros::Time(activation);
    schedule.activation_time = ros::Time(activation);
    schedule.prediction_horizon = best.schedule.prediction_horizon;
    schedule.execution_horizon = execution_horizon;
    schedule.knots_per_member = static_cast<uint32_t>(
        best.schedule.members[0].knots.size());
    schedule.reason = "TARGET_CENTERED_TOPK_REFERENCE_PENDING_LOCAL_REALIZATION";
    schedule.handoff_contract_active = handoff_contract_active;
    schedule.handoff_repair_k = handoff_contract_active
        ? active_handoff_contract_.repair_k : 0;
    if (handoff_contract_active)
    {
      const auto &contract = active_handoff_contract_;
      // Feedback098: name the UAV that produced M2 and the critical instant so
      // the Planner re-checks and refines exactly that UAV.
      schedule.handoff_limiting_uav =
          static_cast<int16_t>(contract.limiting_uav);
      schedule.handoff_limiting_margin = contract.limiting_margin;
      // Feedback100 repair target set.  M2 remains a diagnostic; the repair
      // candidates are the two channels whose restoration can raise M2.
      schedule.handoff_repair_candidates.clear();
      schedule.handoff_repair_candidate_margins.clear();
      for (int slot = 0; slot < contract.repair_candidate_count; ++slot)
      {
        if (contract.repair_candidates[slot] < 0)
          continue;
        schedule.handoff_repair_candidates.push_back(
            static_cast<int16_t>(contract.repair_candidates[slot]));
        schedule.handoff_repair_candidate_margins.push_back(
            contract.repair_candidate_margins[slot]);
      }
      schedule.handoff_repair_priority_uav =
          static_cast<int16_t>(contract.repair_priority_uav);
      schedule.handoff_forecast_margins.clear();
      for (int drone = 0; drone < 3; ++drone)
        schedule.handoff_forecast_margins.push_back(
            contract.forecast_margins[drone]);
      schedule.handoff_t_cross = ros::Time(contract.t_cross);
      schedule.handoff_t_recover = ros::Time(contract.t_recover);
      schedule.handoff_t_star = ros::Time(contract.t_star);
      // Feedback100 pipeline anchors.  detect_wall is the moment this
      // assessment (and therefore the earliest crossing) was available;
      // request_wall is the moment the repair request left the Coordinator.
      schedule.handoff_detect_wall = contract.detect_wall;
      schedule.handoff_request_wall = ros::WallTime::now().toSec();
      schedule.handoff_critical_world_time =
          ros::Time(contract.repair_k == 3
                        ? contract.t_star : contract.m2_min_time);
      schedule.handoff_forecast_created_world_time =
          ros::Time(static_cast<double>(
              relay_assessment.forecast_created_world_time_ns) * 1.0e-9);
      if (contract.repair_k == 2) ROS_INFO("[RELAY_M2_LIMITING_UAV] contract_id=%lu limiting_uav=%d "
               "critical_world_time=%.9f limiting_margin=%.6f "
               "M2_LIMITING_UAV=%d TEAM_REPAIR_CANDIDATES=[%d,%d] "
               "TEAM_REPAIR_PRIORITY_UAV=%d "
               "outgoing=%d incoming=%d stable=%d",
               static_cast<unsigned long>(contract.id), contract.limiting_uav,
               contract.m2_min_time, contract.limiting_margin,
               contract.limiting_uav, contract.repair_candidates[0],
               contract.repair_candidates[1], contract.repair_priority_uav,
               contract.outgoing, contract.incoming, contract.stable);
      schedule.handoff_contract_id = contract.id;
      schedule.handoff_world_revision = contract.target_revision;
      schedule.handoff_outgoing_uav = contract.outgoing;
      schedule.handoff_incoming_uav = contract.incoming;
      schedule.handoff_stable_observer_uav = contract.stable;
      schedule.handoff_acquire_by_world_time = ros::Time(contract.acquire_by);
      schedule.handoff_preserve_until_world_time =
          ros::Time(contract.preserve_until);
      schedule.handoff_required_overlap = contract.required_overlap;
      schedule.handoff_required_visibility_margin = contract.required_margin;
      schedule.handoff_detect_limiter = contract.detect_limiter;
      schedule.handoff_detect_k2 = contract.detect_k2;
      schedule.handoff_detect_k3 = contract.detect_k3;
      schedule.handoff_contract_created_time = ros::Time(contract.created);
      schedule.handoff_contract_expire_time = ros::Time(contract.expires);
      schedule.handoff_m2_before = contract.m2_before;
      schedule.handoff_m2_min = contract.m2_min;
      schedule.handoff_m2_min_time = ros::Time(contract.m2_min_time);
      for (int drone = 0; drone < 3; ++drone)
      {
        schedule.handoff_baseline_owner_revisions.push_back(
            contract.baseline_owner_revisions[drone]);
        schedule.handoff_baseline_owner_trajectory_ids.push_back(
            contract.baseline_owner_trajectory_ids[drone]);
        schedule.handoff_expected_lineage_revisions.push_back(
            contract.expected_lineage_revisions[drone]);
        schedule.handoff_expected_lineage_trajectory_ids.push_back(
            contract.expected_lineage_trajectory_ids[drone]);
      }
      ROS_INFO("[HANDOFF_CONTRACT_ATTACHED] reference_id=%lu contract_id=%lu "
               "TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF=0",
               static_cast<unsigned long>(schedule.team_reference_id),
               static_cast<unsigned long>(contract.id));
    }
    schedule.objective_before = best.before.objective;
    schedule.objective_after = best.after.objective;
    schedule.j_k2_raw = best.after.raw.k2;
    schedule.j_acc_raw = best.after.raw.accumulated;
    schedule.j_comp_raw = best.after.raw.complementarity;
    schedule.j_enc_raw = best.after.raw.encirclement;
    schedule.j_reg_raw = best.after.raw.regularization;
    schedule.j_k2_weighted = best.after.weighted.k2;
    schedule.j_acc_weighted = best.after.weighted.accumulated;
    schedule.j_comp_weighted = best.after.weighted.complementarity;
    schedule.j_enc_weighted = best.after.weighted.encirclement;
    schedule.j_reg_weighted = best.after.weighted.regularization;
    schedule.complementarity_before = best.before.complementarity_score;
    schedule.complementarity_after = best.after.complementarity_score;
    schedule.pair_angle_p50 = best.after.pair_angle_p50;
    schedule.pair_angle_p90 = best.after.pair_angle_p90;
    schedule.simultaneous_los_loss_before =
        best.before.simultaneous_los_loss_duration;
    schedule.simultaneous_los_loss_after =
        best.after.simultaneous_los_loss_duration;
    schedule.optimization_iterations = best.iterations;
    schedule.optimization_time_ms = total_opt_ms;
    for (int drone = 0; drone < 3; ++drone)
    {
      schedule.drone_ids.push_back(drone);
      schedule.planning_generations.push_back(
          bundle_messages_[drone].planning_generation);
      schedule.source_candidate_ids.push_back(
          best.schedule.members[drone].source_candidate_id);
      schedule.topology_kinds.push_back(
          best.schedule.members[drone].topology_kind);
      schedule.frontier_owner_revisions.push_back(
          execution_generations_[drone]);
      schedule.frontier_owner_trajectory_ids.push_back(
          execution_[drone].traj_id);
      // TeamReferenceSchedule keeps the historical ROS field name for wire
      // compatibility.  Its value is a reference phase shift; it never
      // changes Local MINCO piece durations.
      schedule.time_shifts.push_back(
          handoff_contract_active
              ? 0.0
              : best.schedule.members[drone].reference_phase_shift);
      for (const auto &knot : best.schedule.members[drone].knots)
      {
        schedule.knot_world_times.push_back(knot.world_time);
        schedule.knot_radii.push_back(knot.radius);
        schedule.knot_bearings.push_back(knot.bearing);
        schedule.knot_heights.push_back(knot.height);
      }
    }

    pending_team_solution_ = PendingTeamSolution();
    pending_team_solution_.active = true;
    pending_team_solution_.schedule = schedule;
    pending_team_solution_.ack_deadline = ros::Time::now().toSec() +
                                          team_reference_realization_timeout_;
    team_reference_publisher_.publish(schedule);
    ROS_INFO("[TEAM_REFERENCE_PROPOSED] reference_id=%lu generation=%lu "
             "topology=%s,%s,%s activation=%.9f H_pred=%.3f H_exec=%.3f "
             "target_revision=%lu executable_authority=LOCAL_ONLY",
             static_cast<unsigned long>(schedule.team_reference_id),
             static_cast<unsigned long>(schedule.coordination_generation),
             kindName(schedule.topology_kinds[0]),
             kindName(schedule.topology_kinds[1]),
             kindName(schedule.topology_kinds[2]), activation,
             schedule.prediction_horizon, schedule.execution_horizon,
             static_cast<unsigned long>(schedule.target_prediction_revision));
    return true;
  }

  void abortTeamProposal(const std::string &reason)
  {
    // 阶段 G:same-topology refinement 失败(PT-SCP 判不可行/信任耗尽)时,
    // 把 (target, window) 作为 topology escalation 广播回 planner —— 该窗口
    // 的遮挡由 N/L/R 解,而不是同一轮里无限重试同一 SCP。仅由 solver 失败
    // 触发;CAS/identity/传输类失败不代表拓扑证据,不触发。每个 contract
    // 只广播一次。
    const bool scp_failure = reason.find("TEAM_SCP_FAILED") != std::string::npos;
    const bool local_preflight_failure =
        reason.find("LOCAL_REALIZATION_FAILED:LOCAL_HARD_PREFLIGHT_FAILED:") == 0;
    const bool local_realization_failure =
        reason == "LOCAL_REALIZATION_FAILED:LOCAL_MINCO_REALIZATION_FAILED" ||
        reason.find("LOCAL_REALIZATION_FAILED:LOCAL_CONSTRAINT_INITIALIZATION_FAILED:") == 0 ||
        reason == "LOCAL_REALIZATION_FAILED:BOUND_LOCAL_TOPOLOGY_NOT_CONSTRUCTIBLE";
    const bool realization_failure =
        local_preflight_failure || local_realization_failure;
    if (pending_team_solution_.active && (scp_failure || realization_failure))
    {
      const auto &schedule = pending_team_solution_.schedule;
      const bool k3 = schedule.handoff_repair_k == 3;
      const char *reject_reason = "NONE";
      const double now = ros::Time::now().toSec();
      if (k3)
      {
        traj_utils::CooperativeTaskReference current_reference;
        const bool reference_current = currentTaskReference(current_reference);
        const auto &contract = active_handoff_contract_;
        const bool contract_matches = contract.active && contract.repair_k == 3 &&
            contract.id == schedule.handoff_contract_id &&
            contract.repair_priority_uav == schedule.handoff_incoming_uav &&
            std::abs(contract.acquire_by -
                     schedule.handoff_acquire_by_world_time.toSec()) <= 1.0e-6 &&
            std::abs(contract.preserve_until -
                     schedule.handoff_preserve_until_world_time.toSec()) <= 1.0e-6 &&
            contract.required_margin == schedule.handoff_required_visibility_margin &&
            contract.detect_limiter == schedule.handoff_detect_limiter &&
            contract.target_revision == schedule.handoff_world_revision;
        bool bundles_current = true;
        for (int drone = 0; drone < 3; ++drone)
        {
          const auto &bundle = bundle_messages_[drone];
          bundles_current = bundles_current &&
              now - bundle.bundle_stamp.toSec() <= max_bundle_age_ &&
              bundle.target_snapshot_identity == contract.target_revision &&
              bundle.dynamic_prediction_identity == contract.dynamic_revision &&
              bundle.static_map_revision == contract.static_revision &&
              bundle.visibility_model_version == contract.visibility_revision;
        }
        const bool lineage_current = reference_current && contract_matches &&
            sameRelayLineage(contract, current_reference.generation,
                             contract.target_revision, contract.dynamic_revision,
                             contract.static_revision, contract.visibility_revision);
        const RelayAssessment m2 = lineage_current && bundles_current
            ? assessPredictiveRelay(std::max(now, schedule.activation_time.toSec()),
                                    schedule.execution_horizon, now)
            : RelayAssessment();
        if (!contract_matches) reject_reason = "CONTRACT_IDENTITY_MISMATCH";
        else if (now > contract.expires ||
                 now > schedule.handoff_contract_expire_time.toSec())
          reject_reason = "CONTRACT_EXPIRED";
        else if (!reference_current || !bundles_current || !lineage_current)
          reject_reason = "WORLD_OR_LINEAGE_STALE";
        else if (!m2.valid) reject_reason = "M2_CONTEXT_UNAVAILABLE";
        else if (m2.min_m2 < relay_trigger_margin_)
          reject_reason = "M2_PREEMPTION";
        else if (contract.repair_priority_uav < 0 ||
                 contract.repair_priority_uav >= 3 ||
                 !(contract.preserve_until > contract.acquire_by) ||
                 !std::isfinite(contract.required_margin) ||
                 contract.target_revision == 0 ||
                 contract.detect_limiter == 0)
          reject_reason = "K3_EVIDENCE_INVALID";
      }
      const bool eligible = k3 ? std::string(reject_reason) == "NONE"
                               : scp_failure;
      bool published = false;
      if (eligible && schedule.handoff_contract_id != 0 &&
          schedule.handoff_contract_id != last_escalated_contract_id_ &&
          schedule.handoff_incoming_uav >= 0 &&
          schedule.handoff_incoming_uav < 3 &&
          !schedule.handoff_acquire_by_world_time.isZero())
      {
        last_escalated_contract_id_ = schedule.handoff_contract_id;
        traj_utils::TopologyEscalation escalation;
        escalation.header.stamp = ros::Time::now();
        escalation.escalation_id = ++topology_escalation_serial_;
        escalation.handoff_contract_id = schedule.handoff_contract_id;
        escalation.task_reference_generation =
            schedule.task_reference_generation;
        escalation.target_drone = schedule.handoff_incoming_uav;
        escalation.repair_k = schedule.handoff_repair_k;
        escalation.window_begin_world = schedule.handoff_acquire_by_world_time;
        escalation.window_end_world =
            schedule.handoff_preserve_until_world_time;
        escalation.required_margin =
            schedule.handoff_required_visibility_margin;
        // K3 事件绑定证据:contract 创建时刻的检测结论原样转写,Planner 用它
        // 把 early SIDE authority 绑定到同一事件而不是裸 (target, window)。
        escalation.detect_limiter = schedule.handoff_detect_limiter;
        escalation.margin_at_detect = schedule.handoff_limiting_margin;
        escalation.k2_at_detect = schedule.handoff_detect_k2;
        escalation.k3_at_detect = schedule.handoff_detect_k3;
        escalation.target_prediction_revision =
            schedule.handoff_world_revision;
        escalation.reason = reason;
        topology_escalation_publisher_.publish(escalation);
        published = true;
        ROS_WARN(
            "[TOPOLOGY_ESCALATION] contract_id=%lu escalation_id=%lu "
            "target=%d repair_k=%d window=[%.3f, %.3f] reason=%s "
            "action=NEXT_CYCLE_SIDE_DISPATCH",
            static_cast<unsigned long>(schedule.handoff_contract_id),
            static_cast<unsigned long>(escalation.escalation_id),
            escalation.target_drone, escalation.repair_k,
            escalation.window_begin_world.toSec(),
            escalation.window_end_world.toSec(), reason.c_str());
      }
      if (k3)
      {
        const char *source = scp_failure ? "TEAM_SCP_FAILED"
            : local_preflight_failure ? "LOCAL_HARD_PREFLIGHT_FAILED"
                                      : "LOCAL_REALIZATION_FAILED";
        ROS_WARN("[K3_ESCALATION_SOURCE] contract_id=%lu target_uav=%d "
                 "source=%s team_abort_reason=%s escalation_published=%d "
                 "reject_reason=%s",
                 static_cast<unsigned long>(schedule.handoff_contract_id),
                 schedule.handoff_incoming_uav, source, reason.c_str(),
                 static_cast<int>(published), published ? "NONE" :
                     (eligible ? "ALREADY_ESCALATED_OR_WINDOW_INVALID" : reject_reason));
      }
    }
    if (!pending_team_solution_.active)
      return;
    traj_utils::TeamReferenceSchedule abort =
        pending_team_solution_.schedule;
    abort.state = traj_utils::TeamReferenceSchedule::STATE_ABORT;
    abort.valid = false;
    abort.snapshot_stamp = ros::Time::now();
    abort.reason = reason;
    team_reference_publisher_.publish(abort);
    if (pending_team_solution_.prepared ||
        pending_team_solution_.committed)
    {
      traj_utils::TeamTrajectorySolution execution_abort =
          pending_team_solution_.proposal;
      execution_abort.state =
          traj_utils::TeamTrajectorySolution::STATE_ABORT;
      execution_abort.valid = false;
      execution_abort.snapshot_stamp = ros::Time::now();
      execution_abort.reason = reason;
      team_solution_publisher_.publish(execution_abort);
    }
    ROS_INFO("[TEAM_REFERENCE_NOOP] reference_id=%lu reason=%s "
             "local_commit_changed=0",
             static_cast<unsigned long>(abort.team_reference_id),
             reason.c_str());
    pending_team_solution_ = PendingTeamSolution();
  }

  bool commitPendingTeamProposal(const double now)
  {
    if (!pending_team_solution_.active || pending_team_solution_.committed ||
        !pending_team_solution_.all_acked)
      return false;
    const double pending_activation =
        pending_team_solution_.schedule.activation_time.toSec();
    if (pending_activation - now < 0.025)
    {
      abortTeamProposal("ACTIVATION_LEAD_EXHAUSTED_AFTER_3ACK");
      return false;
    }
    const std::vector<uint64_t> &owner_revisions =
        pending_team_solution_.schedule.frontier_owner_revisions;
    const std::vector<int32_t> &owner_trajectory_ids =
        pending_team_solution_.schedule.frontier_owner_trajectory_ids;
    bool commit_cas_fresh = owner_revisions.size() == 3 &&
                            owner_trajectory_ids.size() == 3;
    for (int drone = 0; drone < 3; ++drone)
      commit_cas_fresh = commit_cas_fresh &&
          execution_generations_[drone] ==
              owner_revisions[drone] &&
          execution_[drone].traj_id ==
              owner_trajectory_ids[drone];
    if (!commit_cas_fresh)
    {
      abortTeamProposal("REFERENCE_CAS_STALE_AFTER_3READY");
      return false;
    }
    const auto &schedule = pending_team_solution_.schedule;
      // 阶段 F：REALIZED TRIPLE VALIDATION。reference optimizer 的 before/after
      // 只是 REFERENCE_METRIC telemetry；只有三条 Local realized polynomial 的
      // 联合复评 PASS 才允许生成 execution transaction。Validator 只评价、
      // 不修改轨迹。
      multi_uav_formation::RealizedTeamValidationInput validation_input;
      validation_input.team_reference_id = schedule.team_reference_id;
      validation_input.task_reference_generation =
          schedule.task_reference_generation;
      validation_input.target_prediction_revision =
          schedule.target_prediction_revision;
      validation_input.dynamic_prediction_identity =
          schedule.dynamic_prediction_identity;
      validation_input.static_map_revision = schedule.static_map_revision;
      validation_input.visibility_model_version =
          schedule.visibility_model_version;
      validation_input.activation_time = schedule.activation_time.toSec();
      validation_input.execution_horizon = schedule.execution_horizon;
      validation_input.handoff_contract_active =
          schedule.handoff_contract_active && schedule.handoff_repair_k == 2;
      validation_input.repair_k = schedule.handoff_repair_k;
      validation_input.repair_target_uav =
          schedule.handoff_repair_priority_uav;
      validation_input.critical_begin_world_time =
          schedule.handoff_t_cross.toSec();
      validation_input.critical_end_world_time =
          schedule.handoff_t_recover.toSec();
      validation_input.handoff_contract_id = schedule.handoff_contract_id;
      validation_input.outgoing_uav = schedule.handoff_outgoing_uav;
      validation_input.incoming_uav = schedule.handoff_incoming_uav;
      validation_input.stable_observer_uav =
          schedule.handoff_stable_observer_uav;
      validation_input.acquire_by_world_time =
          schedule.handoff_acquire_by_world_time.toSec();
      validation_input.preserve_until_world_time =
          schedule.handoff_preserve_until_world_time.toSec();
      validation_input.required_overlap =
          schedule.handoff_required_overlap;
      validation_input.required_visibility_margin =
          schedule.handoff_required_visibility_margin;
      for (int drone = 0; drone < 3; ++drone)
      {
        poly_traj::Trajectory realized;
        const bool ok =
            reconstructTrajectory(
                pending_team_solution_.realized_trajectories[drone],
                realized) &&
            realized.getPieceNum() > 0;
        auto &member = validation_input.realized[drone];
        member.valid = ok;
        member.trajectory = realized;
        member.start_time = ok
            ? pending_team_solution_.realized_trajectories[drone]
                  .start_time.toSec()
            : 0.0;
        if (schedule.source_candidate_ids.size() == 3)
        {
          for (const auto &candidate : bundle_messages_[drone].candidates)
          {
            if (candidate.candidate_id !=
                    schedule.source_candidate_ids[drone])
              continue;
            if (!candidate.predicted_yaw_sample_offsets.empty() &&
                candidate.predicted_yaw_sample_offsets.size() ==
                    candidate.predicted_yaw_samples.size())
            {
              const double yaw_time = schedule.activation_time.toSec() -
                                      candidate.trajectory_start_time.toSec();
              const double rate_dt = 0.02;
              member.initial_yaw = sampleYawTrace(
                  candidate.predicted_yaw_sample_offsets,
                  candidate.predicted_yaw_samples, yaw_time);
              const double next_yaw = sampleYawTrace(
                  candidate.predicted_yaw_sample_offsets,
                  candidate.predicted_yaw_samples, yaw_time + rate_dt);
              double delta = next_yaw - member.initial_yaw;
              while (delta > M_PI) delta -= 2.0 * M_PI;
              while (delta < -M_PI) delta += 2.0 * M_PI;
              member.initial_yaw_rate = delta / rate_dt;
              member.initial_yaw_valid =
                  std::isfinite(member.initial_yaw) &&
                  std::isfinite(member.initial_yaw_rate);
            }

            break;
          }
        }
        validation_input.side_contracts[drone] =
            pending_team_solution_.realized_side_contracts[drone];
      }
      for (int drone = 0; drone < 3; ++drone)
      {
        const auto &local = execution_[drone];
        const bool ok = local.traj_id > 0 && local.traj.getPieceNum() > 0;
        auto &baseline = validation_input.baseline[drone];
        baseline.valid = ok;
        baseline.trajectory = local.traj;
        baseline.start_time = ok ? local.start_time : 0.0;
        baseline.initial_yaw_valid =
            validation_input.realized[drone].initial_yaw_valid;
        baseline.initial_yaw = validation_input.realized[drone].initial_yaw;
        baseline.initial_yaw_rate =
            validation_input.realized[drone].initial_yaw_rate;
      }
      traj_utils::CooperativeTaskReference task_reference_for_slots;
      if (currentTaskReference(task_reference_for_slots))
        for (int drone = 0; drone < 3; ++drone)
          validation_input.slot_for_uav[drone] =
              task_reference_for_slots.slot_for_uav[drone];

      multi_uav_formation::RealizedTeamValidator::Params validator_params;
      validator_params.swarm_min_separation = team_swarm_clearance_;
      const auto validation = realized_team_validator_.validate(
          validation_input, validator_params,
          [this](int drone, const Eigen::Vector3d &observer,
                 const Eigen::Vector3d &target, double world_time,
                 double yaw) {
            return continuousVisibility(drone, observer, target, world_time,
                                        yaw);
          },
          [this](double world_time) { return targetAtWorldTime(world_time); });
      if (schedule.handoff_repair_k == 3)
        ROS_INFO("[TEAM_K3_VALIDATION] team_solution_id=%lu "
                 "critical_K3_before=%d critical_K3_after=%d "
                 "total_K3_before=%d total_K3_after=%d "
                 "K2_regression_samples=%d validator_pass=%d reason=%s",
                 static_cast<unsigned long>(schedule.team_reference_id),
                 validation.critical_k3_before,
                 validation.critical_k3_after,
                 validation.total_k3_before,
                 validation.total_k3_after,
                 validation.k2_regression_samples,
                 static_cast<int>(validation.pass),
                 validation.reason.c_str());
      ROS_INFO(
          "[REALIZED_TEAM_VALIDATION] reference_id=%lu decision=%s "
          "reason=%s "
          "realized_q2=%.6f realized_acc=%.6f realized_redundancy=%.6f "
          "realized_enc=%.6f realized_min_pair=%.3f "
          "realized_m2=%.6f m2_time=%.9f handoff_ok=%d side_ok=%d "
          "overlap=%.3f "
          "baseline_q2=%.6f baseline_acc=%.6f baseline_redundancy=%.6f "
          "baseline_enc=%.6f baseline_min_pair=%.3f",
          static_cast<unsigned long>(schedule.team_reference_id),
          validation.pass ? "PASS" : "REJECT",
          validation.reason.c_str(),
          validation.realized.q2, validation.realized.accumulated,
          validation.realized.redundancy, validation.realized.encirclement,
          validation.realized.min_pairwise_distance,
          validation.realized.min_m2,
          validation.realized.min_m2_world_time,
          static_cast<int>(validation.handoff_ok),
          static_cast<int>(validation.side_topology_ok),
          validation.realized_overlap,
          validation.baseline.q2, validation.baseline.accumulated,
          validation.baseline.redundancy, validation.baseline.encirclement,
          validation.baseline.min_pairwise_distance);
      if (!validation.pass)
      {
        abortTeamProposal(std::string("REALIZED_TEAM_VALIDATION_") +
                          validation.reason);
        return false;
      }
      last_realized_validation_ = validation;

      traj_utils::TeamTrajectorySolution prepare;
      prepare.state = traj_utils::TeamTrajectorySolution::STATE_PREPARE;
      prepare.valid = true;
      prepare.team_solution_id = schedule.team_reference_id;
      prepare.task_reference_generation = schedule.task_reference_generation;
      prepare.transaction_id = schedule.team_reference_id;
      prepare.coordination_generation = schedule.coordination_generation;
      // Historical wire flag consumed by Local's proposal decoder.  In the
      // current chain it means "payload is the certified Local realization";
      // the coordinator never produced this polynomial.
      prepare.early_joint_primary = true;
      prepare.team_context_generation = schedule.coordination_generation;
      prepare.team_context_snapshot = schedule.snapshot_stamp;
      prepare.prediction_epoch = schedule.evaluation_start_world;
      prepare.target_snapshot_identity = schedule.target_prediction_revision;
      prepare.target_snapshot_epoch = schedule.target_snapshot_epoch;
      prepare.dynamic_prediction_identity = schedule.dynamic_prediction_identity;
      prepare.dynamic_prediction_valid_from =
          schedule.dynamic_prediction_valid_from;
      prepare.dynamic_prediction_valid_to =
          schedule.dynamic_prediction_valid_to;
      prepare.frontier_owner_revisions = schedule.frontier_owner_revisions;
      prepare.frontier_owner_trajectory_ids =
          schedule.frontier_owner_trajectory_ids;
      prepare.static_map_revision = schedule.static_map_revision;
      prepare.visibility_model_version = schedule.visibility_model_version;
      prepare.hypothesis_id = schedule.hypothesis_id;
      prepare.encirclement_generation = schedule.encirclement_generation;
      prepare.snapshot_stamp = ros::Time::now();
      prepare.activation_time = schedule.activation_time;
      prepare.evaluation_horizon = schedule.execution_horizon;
      prepare.reason = "REALIZED_TEAM_CERTIFIED_EXECUTION_PREPARE";
      prepare.drone_ids = schedule.drone_ids;
      prepare.planning_generations = schedule.planning_generations;
      prepare.source_candidate_ids = schedule.source_candidate_ids;
      prepare.source_candidate_kinds = schedule.topology_kinds;
      prepare.objective_before = schedule.objective_before;
      prepare.objective_after = schedule.objective_after;
      // 阶段 F（任务书第十六节）：execution transaction 里的 before/after 一律
      // 写 REALIZED_METRIC（baseline = 同 activation/horizon 的 committed Local
      // triple；after = realized triple）。schedule.j_*_raw 只是
      // REFERENCE_METRIC，仅保留在 TeamReferenceSchedule telemetry 里。
      prepare.k2_before = 1.0 - last_realized_validation_.baseline.q2;
      prepare.k2_after = 1.0 - last_realized_validation_.realized.q2;
      prepare.accumulated_visibility_cost_before =
          1.0 - last_realized_validation_.baseline.accumulated;
      prepare.accumulated_visibility_cost_after =
          1.0 - last_realized_validation_.realized.accumulated;
      prepare.deviation_cost_before =
          last_realized_validation_.baseline.redundancy +
          last_realized_validation_.baseline.encirclement;
      prepare.deviation_cost_after =
          last_realized_validation_.realized.redundancy +
          last_realized_validation_.realized.encirclement;
      ROS_INFO(
          "[REFERENCE_METRIC] reference_id=%lu j_k2_raw=%.6f j_acc_raw=%.6f "
          "j_comp_raw=%.6f j_enc_raw=%.6f",
          static_cast<unsigned long>(schedule.team_reference_id),
          schedule.j_k2_raw, schedule.j_acc_raw, schedule.j_comp_raw,
          schedule.j_enc_raw);
      ROS_INFO(
          "[REALIZED_METRIC] reference_id=%lu k2_cost=%.6f->%.6f "
          "acc_cost=%.6f->%.6f geometry=%.6f->%.6f "
          "min_pair=%.3f",
          static_cast<unsigned long>(schedule.team_reference_id),
          prepare.k2_before, prepare.k2_after,
          prepare.accumulated_visibility_cost_before,
          prepare.accumulated_visibility_cost_after,
          prepare.deviation_cost_before, prepare.deviation_cost_after,
          last_realized_validation_.realized.min_pairwise_distance);
      prepare.position_change_norm = 0.0;
      prepare.time_change_norm = 0.0;
      for (const double shift : schedule.time_shifts)
        prepare.time_change_norm += shift * shift;
      prepare.time_change_norm = std::sqrt(prepare.time_change_norm);
      prepare.yaw_change_norm = 0.0;
      prepare.optimized_yaw_valid = false;
      prepare.optimized_yaw_executed = false;
      prepare.optimization_iterations = schedule.optimization_iterations;
      prepare.optimization_time_ms = schedule.optimization_time_ms;
      for (int drone = 0; drone < 3; ++drone)
        prepare.trajectories.push_back(
            pending_team_solution_.realized_trajectories[drone]);
      pending_team_solution_.proposal = prepare;
      pending_team_solution_.prepared = true;
      pending_team_solution_.executor_ready = {{false, false, false}};
      pending_team_solution_.executor_all_ready = false;
      pending_team_solution_.executor_ready_deadline = std::min(
          schedule.activation_time.toSec() - 0.025,
          now + std::max(0.02, team_ack_timeout_));
      pending_team_solution_.last_commit_publish = now;
      team_solution_publisher_.publish(prepare);
      if (schedule.handoff_repair_k == 3)
        ROS_INFO("[TEAM_K3_ADOPTED] team_solution_id=%lu target_uav=%d "
                 "activation=%.9f critical=[%.9f,%.9f]",
                 static_cast<unsigned long>(schedule.team_reference_id),
                 static_cast<int>(schedule.handoff_repair_priority_uav),
                 schedule.activation_time.toSec(),
                 schedule.handoff_t_cross.toSec(),
                 schedule.handoff_t_recover.toSec());
      ROS_INFO("[TEAM_REALIZATION_3READY] reference_id=%lu accepted=1,1,1",
          static_cast<unsigned long>(schedule.team_reference_id));
      ROS_INFO("[TEAM_EXECUTION_PREPARE] reference_id=%lu activation=%.9f "
               "source=THREE_LOCAL_REALIZATIONS authority=LOCAL_ONLY",
          static_cast<unsigned long>(schedule.team_reference_id),
          schedule.activation_time.toSec());
      // 阶段 B / 任务书第六节：Team realized triple 被接受后，向 Cooperative
      // 发出任务级 phi0 建议。Cooperative manager 仍通过自己的 rate-limited
      // transition 决定是否接受 —— Team 不是任务几何 authority。
      {
        traj_utils::TaskReferenceAdjustment adjustment;
        adjustment.header.stamp = ros::Time::now();
        adjustment.reference_generation = schedule.task_reference_generation;
        adjustment.team_transaction_id = prepare.transaction_id;
        auto wrap_pi = [](double angle) {
          while (angle > M_PI) angle -= 2.0 * M_PI;
          while (angle < -M_PI) angle += 2.0 * M_PI;
          return angle;
        };
        traj_utils::CooperativeTaskReference task_reference;
        const bool have_reference = currentTaskReference(task_reference);
        double phi_sum = 0.0;
        const int knots = std::max(1, static_cast<int>(schedule.knots_per_member));
        for (int drone = 0; drone < 3; ++drone)
        {
          const double final_bearing =
              schedule.knot_bearings[drone * knots + knots - 1];
          const double slot_offset = have_reference
              ? 2.0 * M_PI * static_cast<double>(task_reference.slot_for_uav[drone]) / 3.0
              : 2.0 * M_PI * static_cast<double>(drone) / 3.0;
          phi_sum += wrap_pi(final_bearing - slot_offset);
        }
        adjustment.recommended_phi0 = wrap_pi(phi_sum / 3.0);
        adjustment.reason = "REALIZED_TRIPLE_ACCEPTED";
        task_adjustment_publisher_.publish(adjustment);
        ROS_INFO(
            "[TASK_REFERENCE_ADJUSTMENT] recommended_phi0=%.6f "
            "reference_generation=%lu team_transaction_id=%lu",
            adjustment.recommended_phi0,
            static_cast<unsigned long>(adjustment.reference_generation),
            static_cast<unsigned long>(adjustment.team_transaction_id));
      }
    return true;
  }

  bool commitPreparedTeamExecution(const double now)
  {
    if (!pending_team_solution_.active ||
        !pending_team_solution_.prepared ||
        pending_team_solution_.committed ||
        !pending_team_solution_.executor_all_ready)
      return false;
    if (pending_team_solution_.proposal.activation_time.toSec() - now < 0.025)
    {
      abortTeamProposal("EXECUTION_COMMIT_LEAD_EXHAUSTED");
      return false;
    }
    const auto &owner_revisions =
        pending_team_solution_.proposal.frontier_owner_revisions;
    const auto &owner_trajectory_ids =
        pending_team_solution_.proposal.frontier_owner_trajectory_ids;
    bool cas_fresh = owner_revisions.size() == 3 &&
                     owner_trajectory_ids.size() == 3;
    for (int drone = 0; drone < 3; ++drone)
      cas_fresh = cas_fresh &&
          execution_generations_[drone] == owner_revisions[drone] &&
          execution_[drone].traj_id == owner_trajectory_ids[drone];
    if (!cas_fresh)
    {
      abortTeamProposal("EXECUTION_COMMIT_CAS_STALE");
      return false;
    }
    pending_team_solution_.proposal.state =
        traj_utils::TeamTrajectorySolution::STATE_COMMIT;
    pending_team_solution_.proposal.snapshot_stamp = ros::Time::now();
    pending_team_solution_.proposal.reason =
        "EXECUTOR_3READY_ATOMIC_COMMIT";
    pending_team_solution_.committed = true;
    pending_team_solution_.last_commit_publish = now;
    team_solution_publisher_.publish(pending_team_solution_.proposal);
    ROS_INFO("[TEAM_EXECUTION_COMMIT] reference_id=%lu activation=%.9f "
             "executor_ready=1,1,1",
        static_cast<unsigned long>(
            pending_team_solution_.proposal.team_solution_id),
        pending_team_solution_.proposal.activation_time.toSec());
    return true;
  }

  void taskReferenceCallback(
      const traj_utils::CooperativeTaskReferenceConstPtr &message)
  {
    if (!message || !message->valid)
      return;
    latest_task_reference_ = *message;
    latest_task_reference_stamp_ = ros::Time::now();
    has_task_reference_ = true;
    ROS_INFO_THROTTLE(
        5.0,
        "[task-reference] received generation=%lu phi0=%.6f "
        "slots=%d,%d,%d bearings=%.3f,%.3f,%.3f radii=%.2f,%.2f,%.2f",
        static_cast<unsigned long>(message->generation), message->phi0,
        message->slot_for_uav[0], message->slot_for_uav[1],
        message->slot_for_uav[2], message->desired_bearing[0],
        message->desired_bearing[1], message->desired_bearing[2],
        message->desired_radius[0], message->desired_radius[1],
        message->desired_radius[2]);
  }

  // 任务参考有效性：valid 且新鲜（manager 以 rate_hz_ 持续发布）。
  bool currentTaskReference(traj_utils::CooperativeTaskReference &out) const
  {
    if (!has_task_reference_)
      return false;
    if ((ros::Time::now() - latest_task_reference_stamp_).toSec() > 1.5)
      return false;
    out = latest_task_reference_;
    return true;
  }

  void teamAckCallback(
      const traj_utils::TeamTrajectoryAckConstPtr &message)
  {
    if (!message)
      return;
    if (pending_team_solution_.active &&
        message->team_solution_id ==
            pending_team_solution_.schedule.team_reference_id &&
        message->coordination_generation ==
            pending_team_solution_.schedule.coordination_generation &&
        message->phase ==
            traj_utils::TeamTrajectoryAck::PHASE_EXECUTION_REVOKE)
    {
      abortTeamProposal(std::string("EXECUTION_REVOKE:") + message->reason);
      return;
    }
    if (pending_team_solution_.active &&
        pending_team_solution_.prepared &&
        message->phase ==
            traj_utils::TeamTrajectoryAck::PHASE_EXECUTOR_READY)
    {
      const auto &proposal = pending_team_solution_.proposal;
      if (message->team_solution_id != proposal.team_solution_id ||
          message->coordination_generation !=
              proposal.coordination_generation ||
          message->drone_id < 0 || message->drone_id >= 3)
        return;
      const int drone = message->drone_id;
      const bool identity_matches =
          proposal.planning_generations.size() == 3 &&
          proposal.source_candidate_ids.size() == 3 &&
          message->planning_generation ==
              proposal.planning_generations[drone] &&
          message->source_candidate_id ==
              proposal.source_candidate_ids[drone] &&
          message->target_prediction_revision ==
              proposal.target_snapshot_identity &&
          message->target_snapshot_epoch == proposal.target_snapshot_epoch &&
          message->dynamic_prediction_identity ==
              proposal.dynamic_prediction_identity &&
          message->static_map_revision == proposal.static_map_revision &&
          message->visibility_model_version ==
              proposal.visibility_model_version &&
          message->activation_time == proposal.activation_time;
      if (!message->accepted || !identity_matches)
      {
        abortTeamProposal(message->accepted
            ? "EXECUTOR_READY_IDENTITY_MISMATCH"
            : std::string("EXECUTOR_PREPARE_FAILED:") + message->reason);
        return;
      }
      pending_team_solution_.executor_ready[drone] = true;
      pending_team_solution_.executor_all_ready =
          pending_team_solution_.executor_ready[0] &&
          pending_team_solution_.executor_ready[1] &&
          pending_team_solution_.executor_ready[2];
      ROS_INFO("[TEAM_EXECUTOR_READY_RECV] reference_id=%lu drone=%d "
               "ready=%d,%d,%d",
          static_cast<unsigned long>(proposal.team_solution_id), drone,
          static_cast<int>(pending_team_solution_.executor_ready[0]),
          static_cast<int>(pending_team_solution_.executor_ready[1]),
          static_cast<int>(pending_team_solution_.executor_ready[2]));
      if (pending_team_solution_.executor_all_ready)
        pending_team_solution_.executor_all_ready_time =
            ros::Time::now().toSec();
      return;
    }
    if (message->phase != traj_utils::TeamTrajectoryAck::PHASE_REALIZATION)
      return;
    if (pending_team_solution_.active)
    {
      const auto &schedule = pending_team_solution_.schedule;
      if (message->team_reference_id != schedule.team_reference_id ||
          message->team_solution_id != schedule.team_reference_id ||
          message->coordination_generation !=
              schedule.coordination_generation ||
          message->drone_id < 0 || message->drone_id >= 3)
        return;
      const int drone = message->drone_id;
      const bool identity_matches =
          schedule.planning_generations.size() == 3 &&
          schedule.source_candidate_ids.size() == 3 &&
          schedule.topology_kinds.size() == 3 &&
          schedule.frontier_owner_revisions.size() == 3 &&
          schedule.frontier_owner_trajectory_ids.size() == 3 &&
          message->planning_generation ==
              schedule.planning_generations[drone] &&
          message->source_candidate_id ==
              schedule.source_candidate_ids[drone] &&
          message->target_prediction_revision ==
              schedule.target_prediction_revision &&
          message->target_snapshot_epoch == schedule.target_snapshot_epoch &&
          message->dynamic_prediction_identity ==
              schedule.dynamic_prediction_identity &&
          message->static_map_revision == schedule.static_map_revision &&
          message->visibility_model_version ==
              schedule.visibility_model_version &&
          message->evaluation_start_world ==
              schedule.evaluation_start_world &&
          message->activation_time == schedule.activation_time &&
          std::abs(message->execution_horizon -
                   schedule.execution_horizon) <= 1.0e-6 &&
          message->topology_kind == schedule.topology_kinds[drone] &&
          message->frontier_owner_revision ==
              schedule.frontier_owner_revisions[drone] &&
          message->frontier_owner_trajectory_id ==
              schedule.frontier_owner_trajectory_ids[drone];
      const bool topology_is_side =
          schedule.topology_kinds[drone] ==
              traj_utils::TopologyCandidate::KIND_SIDE_PLUS ||
          schedule.topology_kinds[drone] ==
              traj_utils::TopologyCandidate::KIND_SIDE_MINUS;
      const int expected_side_sign =
          schedule.topology_kinds[drone] ==
                  traj_utils::TopologyCandidate::KIND_SIDE_PLUS
              ? 1
              : (schedule.topology_kinds[drone] ==
                         traj_utils::TopologyCandidate::KIND_SIDE_MINUS
                     ? -1
                     : 0);
      const Eigen::Vector3d side_origin(
          message->side_contract_origin.x,
          message->side_contract_origin.y,
          message->side_contract_origin.z);
      const Eigen::Vector3d side_direction(
          message->side_contract_direction.x,
          message->side_contract_direction.y,
          message->side_contract_direction.z);
      const bool side_contract_matches = topology_is_side
          ? message->side_contract_active && side_origin.allFinite() &&
                side_direction.allFinite() &&
                side_direction.norm() > 1.0e-6 &&
                message->side_contract_sign == expected_side_sign &&
                std::isfinite(message->side_contract_offset) &&
                message->side_contract_offset > 0.0 &&
                std::isfinite(message->side_contract_conflict_progress) &&
                message->side_contract_conflict_progress >= 0.0 &&
                message->side_contract_conflict_progress <= 1.0 &&
                std::isfinite(message->side_contract_guidance_window) &&
                message->side_contract_guidance_window > 0.0 &&
                message->side_contract_guidance_window <= 1.0
          : !message->side_contract_active;
      if (!message->accepted || !message->realization_valid ||
          !identity_matches || !side_contract_matches)
      {
        abortTeamProposal(message->accepted
            ? "LOCAL_REALIZATION_IDENTITY_MISMATCH"
            : std::string("LOCAL_REALIZATION_FAILED:") + message->reason);
        return;
      }
      poly_traj::Trajectory realized;
      if (!reconstructTrajectory(message->realized_trajectory, realized) ||
          std::abs(message->realized_trajectory.start_time.toSec() -
                   schedule.activation_time.toSec()) > 1.0e-6 ||
          realized.getTotalDuration() + 1.0e-6 < schedule.execution_horizon)
      {
        abortTeamProposal("MALFORMED_LOCAL_REALIZATION");
        return;
      }
      if (pending_team_solution_.acked[drone])
        return;
      poly_traj::Trajectory side_seed;
      if (topology_is_side &&
          (!reconstructTrajectory(message->side_contract_seed_trajectory,
                                  side_seed) ||
           std::abs(message->side_contract_seed_trajectory.start_time.toSec() -
                    schedule.activation_time.toSec()) > 1.0e-6 ||
           side_seed.getTotalDuration() <= 1.0e-6))
      {
        abortTeamProposal("MALFORMED_SIDE_SEED_CERTIFICATE");
        return;
      }
      pending_team_solution_.acked[drone] = true;
      pending_team_solution_.realized_trajectories[drone] =
          message->realized_trajectory;
      auto &side = pending_team_solution_.realized_side_contracts[drone];
      side.active = message->side_contract_active;
      if (side.active)
      {
        side.origin = side_origin;
        side.direction = side_direction;
        side.side_sign = message->side_contract_sign;
        side.offset = message->side_contract_offset;
        side.conflict_progress = message->side_contract_conflict_progress;
        side.guidance_window = message->side_contract_guidance_window;
        side.seed_trajectory = side_seed;
      }
      pending_team_solution_.realization_latency_ms[drone] =
          message->realization_latency_ms;
      ROS_INFO("[TEAM_REFERENCE_REALIZATION_SUCCESS] reference_id=%lu "
               "drone=%d topology=%s latency_ms=%.3f",
          static_cast<unsigned long>(schedule.team_reference_id), drone,
          kindName(schedule.topology_kinds[drone]),
          message->realization_latency_ms);
      pending_team_solution_.all_acked =
          pending_team_solution_.acked[0] &&
          pending_team_solution_.acked[1] &&
          pending_team_solution_.acked[2];
      if (pending_team_solution_.all_acked)
      {
        pending_team_solution_.all_acked_time = ros::Time::now().toSec();
        ROS_INFO("[TEAM_REFERENCE_REALIZATION_SUCCESS] reference_id=%lu "
                 "all_three=1 latency_ms=%.3f,%.3f,%.3f",
            static_cast<unsigned long>(schedule.team_reference_id),
            pending_team_solution_.realization_latency_ms[0],
            pending_team_solution_.realization_latency_ms[1],
            pending_team_solution_.realization_latency_ms[2]);
      }
      return;
    }
  }

  void teamCommitTimerCallback(const ros::TimerEvent &)
  {
    if (!pending_team_solution_.active)
      return;
    const double now = ros::Time::now().toSec();
    if (!pending_team_solution_.committed)
    {
      if (pending_team_solution_.prepared)
      {
        if (pending_team_solution_.executor_all_ready &&
            now - pending_team_solution_.executor_all_ready_time >= 0.01)
        {
          commitPreparedTeamExecution(now);
          return;
        }
        if (now > pending_team_solution_.executor_ready_deadline)
          abortTeamProposal("EXECUTOR_READY_TIMEOUT");
        return;
      }
      if (pending_team_solution_.all_acked &&
          now - pending_team_solution_.all_acked_time >= 0.02)
      {
        commitPendingTeamProposal(now);
        return;
      }
      if (now > pending_team_solution_.ack_deadline)
        abortTeamProposal("ACK_TIMEOUT");
      return;
    }
    const double activation =
        pending_team_solution_.proposal.activation_time.toSec();
    if (now < activation &&
        now - pending_team_solution_.last_commit_publish >= 0.02)
    {
      pending_team_solution_.proposal.snapshot_stamp = ros::Time::now();
      team_solution_publisher_.publish(pending_team_solution_.proposal);
      pending_team_solution_.last_commit_publish = now;
    }
    if (now >= activation + 0.05)
    {
      if (pending_team_solution_.schedule.handoff_repair_k == 3 &&
          pending_team_solution_.committed)
      {
        const auto &schedule = pending_team_solution_.schedule;
        const bool all_activated =
            execution_team_reference_ids_[0] == schedule.team_reference_id &&
            execution_team_reference_ids_[1] == schedule.team_reference_id &&
            execution_team_reference_ids_[2] == schedule.team_reference_id;
        if (all_activated)
          ROS_INFO("[TEAM_K3_ACTIVATED] team_solution_id=%lu target_uav=%d "
                   "trajectory_id=%d,%d,%d activation=%.9f "
                   "critical_window=[%.9f,%.9f]",
                   static_cast<unsigned long>(schedule.team_reference_id),
                   static_cast<int>(schedule.handoff_repair_priority_uav),
                   execution_[0].traj_id, execution_[1].traj_id,
                   execution_[2].traj_id, activation,
                   schedule.handoff_t_cross.toSec(),
                   schedule.handoff_t_recover.toSec());
        else
          ROS_WARN("[TEAM_K3_ACTIVATION_MISSED] team_solution_id=%lu "
                   "observed_reference_ids=%lu,%lu,%lu",
                   static_cast<unsigned long>(schedule.team_reference_id),
                   static_cast<unsigned long>(execution_team_reference_ids_[0]),
                   static_cast<unsigned long>(execution_team_reference_ids_[1]),
                   static_cast<unsigned long>(execution_team_reference_ids_[2]));
      }
      if (pending_team_solution_.schedule.handoff_contract_active &&
          active_handoff_contract_.active &&
          pending_team_solution_.schedule.handoff_contract_id ==
              active_handoff_contract_.id)
      {
        ROS_INFO("[HANDOFF_CONTRACT_COMPLETED] id=%lu reference_id=%lu "
                 "realized_overlap=%.6f realized_m2=%.6f",
                 static_cast<unsigned long>(active_handoff_contract_.id),
                 static_cast<unsigned long>(
                     pending_team_solution_.schedule.team_reference_id),
                 last_realized_validation_.realized_overlap,
                 last_realized_validation_.realized.min_m2);
        active_handoff_contract_ = PersistentHandoffContract();
      }
      pending_team_solution_ = PendingTeamSolution();
    }
  }

  void publishTerminalNegative(const std::string &reason)
  {
    traj_utils::TopologyCoordination result;
    result.valid=false;
    result.snapshot_stamp=ros::Time::now();
    result.coordination_generation=++coordination_generation_;
    result.selection_reason=reason;
    for(int drone=0;drone<3;++drone)
    {
      result.drone_ids.push_back(drone);
      result.planning_generations.push_back(bundles_[drone].planning_generation);
      result.selected_candidate_ids.push_back(-1);
      result.selected_candidate_kinds.push_back(0);
    }
    result_publisher_.publish(result);
    ROS_INFO("[topology-terminal-negative] reason=%s generations=%lu,%lu,%lu",reason.c_str(),
        static_cast<unsigned long>(bundles_[0].planning_generation),
        static_cast<unsigned long>(bundles_[1].planning_generation),
        static_cast<unsigned long>(bundles_[2].planning_generation));
  }

  void evaluateSnapshot()
  {
    // Assemble the latest unconsumed pending identities. A terminal result
    // releases those identities on the planners, so recycling even one of
    // them produces proposals that can no longer collect three ACKs.
    // Only this asynchronous coordinator collects bundles; local never waits.
    if (!hasCompleteSnapshot() || pending_team_solution_.active)
      return;
    const double now = ros::Time::now().toSec();
    const ros::WallTime evaluation_start = ros::WallTime::now();

    // Each bundle names an independently committed, safety-validated local
    // trajectory. Planning generations and suggested activation slots may
    // differ; the coordinator discovers a common future frontier below.
    bool waiting_for_boundary_observation=false;
    bool boundary_superseded=false;
    for (int drone=0;drone<3;++drone)
    {
      const auto &wire=bundle_messages_[drone];
      const bool exact=execution_generations_[drone] ==
              wire.expected_execution_generation &&
          execution_[drone].traj_id == wire.expected_trajectory_id;
      waiting_for_boundary_observation = waiting_for_boundary_observation ||
          (!exact && execution_generations_[drone] <=
              wire.expected_execution_generation);
      boundary_superseded = boundary_superseded ||
          wire.transaction_id == 0 ||
          wire.boundary_source_revision !=
              wire.expected_execution_generation ||
          wire.boundary_source_trajectory_id != wire.expected_trajectory_id ||
          execution_generations_[drone] > wire.expected_execution_generation ||
          (execution_generations_[drone] == wire.expected_execution_generation &&
           execution_[drone].traj_id != wire.expected_trajectory_id);
    }
    if (boundary_superseded)
    {
      ROS_WARN("[TEAM_CONTEXT_REJECT] reason=TRANSACTION_BOUNDARY_SUPERSEDED "
               "expected_generations=%lu,%lu,%lu actual_generations=%lu,%lu,%lu "
               "expected_ids=%d,%d,%d actual_ids=%d,%d,%d",
          static_cast<unsigned long>(bundle_messages_[0].expected_execution_generation),
          static_cast<unsigned long>(bundle_messages_[1].expected_execution_generation),
          static_cast<unsigned long>(bundle_messages_[2].expected_execution_generation),
          static_cast<unsigned long>(execution_generations_[0]),
          static_cast<unsigned long>(execution_generations_[1]),
          static_cast<unsigned long>(execution_generations_[2]),
          bundle_messages_[0].expected_trajectory_id,
          bundle_messages_[1].expected_trajectory_id,
          bundle_messages_[2].expected_trajectory_id,
          execution_[0].traj_id,execution_[1].traj_id,execution_[2].traj_id);
      ROS_INFO("[team-reference-input] event=REFERENCE_INPUT_STALE "
               "reason=COMMITTED_OWNER_SUPERSEDED action=DISCARD_ONLY");
      consumeCurrentSnapshot();
      return;
    }
    if (waiting_for_boundary_observation)
    {
      double oldest_age=0.0;
      for(int drone=0;drone<3;++drone)
        oldest_age=std::max(oldest_age,now-bundles_[drone].bundle_stamp);
      if (oldest_age <= max_bundle_age_)
      {
        ROS_INFO_THROTTLE(0.1,"[TEAM_CONTEXT_WAIT] reason=BOUNDARY_OBSERVATION_NOT_YET_ALIGNED "
            "age=%.6f expected_generations=%lu,%lu,%lu actual_generations=%lu,%lu,%lu",
            oldest_age,
            static_cast<unsigned long>(bundle_messages_[0].expected_execution_generation),
            static_cast<unsigned long>(bundle_messages_[1].expected_execution_generation),
            static_cast<unsigned long>(bundle_messages_[2].expected_execution_generation),
            static_cast<unsigned long>(execution_generations_[0]),
            static_cast<unsigned long>(execution_generations_[1]),
            static_cast<unsigned long>(execution_generations_[2]));
        return;
      }
      ROS_INFO("[team-reference-input] event=REFERENCE_INPUT_STALE "
               "reason=COMMITTED_OWNER_NOT_OBSERVED action=DISCARD_ONLY");
      consumeCurrentSnapshot();
      return;
    }

    // The only production Team path: enumerate at most 27 Local N/L/R seeds,
    // retain the shared-objective Top-K, optimize a low-dimensional
    // target-centred reference, then ask all three Local Planners to realize
    // it.  No polynomial is produced or published here.
    const TopologySelectionCore early_reference =
        core_->select(early_seed_bundles_, now, history_);
    if (early_reference.coordination_available)
    {
      traj_utils::TopologyCoordination reference_context;
      reference_context.valid = true;
      reference_context.coordination_generation = ++coordination_generation_;
      reference_context.snapshot_stamp = ros::Time(now);
      reference_context.selection_reason =
          "TARGET_CENTERED_TEAM_REFERENCE_TOPK";
      reference_context.selected_encirclement_hypothesis_id =
          early_reference.joint.encirclement_hypothesis_id;
      reference_context.selected_encirclement_generation =
          early_reference.joint.encirclement_generation;
      for (int drone = 0; drone < 3; ++drone)
      {
        reference_context.drone_ids.push_back(drone);
        reference_context.planning_generations.push_back(
            bundles_[drone].planning_generation);
        reference_context.selected_candidate_ids.push_back(
            early_reference.joint.candidate_ids[drone]);
        reference_context.selected_candidate_kinds.push_back(
            early_reference.joint.kinds[drone]);
        reference_context.local_candidate_ids.push_back(
            bundle_messages_[drone].local_selected_candidate_id);
        reference_context.local_candidate_kinds.push_back(
            bundle_messages_[drone].local_selected_kind);
      }
      attemptTargetCenteredTeamReference(early_reference, reference_context);
    }
    else
    {
      ROS_INFO("[TEAM_REFERENCE_NOOP] reason=%s local_commit_changed=0",
          early_reference.reason.c_str());
    }
    consumeCurrentSnapshot();
    return;

  }

  ros::NodeHandle node_;
  traj_utils::AblationConfig ablation_config_;
  bool formal_ablation_mode_{false};
  std::array<ros::Subscriber, 3> bundle_subscribers_;
  ros::Publisher result_publisher_;
  std::array<TopologyBundleCore,3> early_seed_bundles_;
  std::array<ego_planner::LocalTrajData,3> execution_;
  std::array<std::uint64_t,3> execution_generations_{};
  std::array<ros::Subscriber,3> execution_subscribers_;
  ros::Subscriber actual_execution_subscriber_;
  ros::Publisher team_solution_publisher_;
  ros::Publisher team_reference_publisher_;
  ros::Subscriber team_ack_subscriber_;
  ros::Subscriber task_reference_subscriber_;
  ros::Publisher task_adjustment_publisher_;
  ros::Publisher topology_escalation_publisher_;
  std::uint64_t last_escalated_contract_id_{0};
  std::uint64_t topology_escalation_serial_{0};
  ros::Timer team_commit_timer_;
  std::unique_ptr<TopologyCoordinatorCore> core_;
  std::unique_ptr<TeamTargetCenteredOptimizer> target_centered_optimizer_;
  multi_uav_formation::RealizedTeamValidator realized_team_validator_;
  multi_uav_formation::RealizedTeamValidationResult
      last_realized_validation_;
  fast_planner::ObjPredictor::Ptr object_predictor_;
  ego_planner::StaticLosGeometry static_geometry_;
  TrackingCameraContract tracking_camera_;
  TeamTargetCenteredParams target_centered_params_;
  double team_swarm_clearance_{0.50};
  std::array<TopologyBundleCore, 3> bundles_;
  std::array<traj_utils::TopologyCandidateBundle, 3> bundle_messages_;
  std::array<bool, 3> bundle_valid_{{false, false, false}};
  std::array<std::uint64_t, 3> last_evaluated_generation_{{0, 0, 0}};
  TopologyHistory history_;
  std::uint64_t coordination_generation_{0};
  bool outer_snapshot_active_{false};
  std::uint64_t outer_snapshot_generation_{0};
  double outer_snapshot_started_{0.0};
  double max_bundle_age_{0.30};
  double planner_acceptance_timeout_{0.20};
  double result_publish_margin_{0.02};
  bool enable_team_visibility_optimizer_{false};
  bool enable_joint_pt_optimization_{false};
  bool enable_predicted_attitude_fov_{false};
  bool team_static_geometry_ready_{false};
  bool team_runtime_ready_{false};
  double visibility_occlusion_margin_{0.08};
  // 阶段 B：最近一次 CooperativeTaskReference（任务几何唯一 authority）。
  traj_utils::CooperativeTaskReference latest_task_reference_;
  ros::Time latest_task_reference_stamp_;
  bool has_task_reference_{false};
  double visibility_range_smoothing_{0.25};
  double visibility_los_smoothing_{0.15};
  double relay_horizon_{2.0};
  double relay_sample_dt_{0.10};
  double relay_trigger_margin_{0.30};
  double relay_required_margin_{0.20};
  double relay_release_margin_{0.45};
  double relay_overlap_{0.50};
  double relay_stable_time_{0.20};
  double relay_contract_timeout_{1.0};
  bool team_k3_repair_enabled_{false};
  // K3 Local autonomy mirror:当 Local 侧启用自主 K3 recovery 时,Team K3 合同
  // 只做检测遥测(与 planner 的 manager/k3_local_escalation_enabled 同一开关)。
  bool k3_local_autonomy_enabled_{false};
  std::array<std::uint64_t, 3> execution_team_reference_ids_{{0, 0, 0}};
  std::uint64_t handoff_contract_serial_{0};
  PersistentHandoffContract active_handoff_contract_;
  PipelineLatencyEstimate preparation_latency_{0.020};
  double team_activation_lead_{0.24};
  double team_ack_timeout_{0.06};
  double team_reference_realization_timeout_{0.25};
  std::uint64_t team_solution_serial_{0};
  std::uint64_t primary_visibility_reject_count_{0};
  std::uint64_t complementarity_reject_count_{0};
  std::uint64_t stage1_fallback_count_{0};
  std::uint64_t stage2_fallback_count_{0};
  std::uint64_t stage3_fallback_count_{0};
  std::uint64_t final_stage_q2_count_{0};
  std::uint64_t final_stage_acc_count_{0};
  std::uint64_t final_stage_comp_count_{0};
  std::uint64_t final_stage_enc_count_{0};

  struct PendingTeamSolution
  {
    bool active{false};
    bool prepared{false};
    bool committed{false};
    bool all_acked{false};
    std::array<bool, 3> acked{{false, false, false}};
    std::array<bool, 3> executor_ready{{false, false, false}};
    bool executor_all_ready{false};
    double ack_deadline{0.0};
    double all_acked_time{0.0};
    double executor_ready_deadline{0.0};
    double executor_all_ready_time{0.0};
    double last_commit_publish{0.0};
    traj_utils::TeamTrajectorySolution proposal;
    traj_utils::TeamReferenceSchedule schedule;
    std::array<traj_utils::MINCOTraj, 3> realized_trajectories;
    std::array<RealizedTeamSideContract, 3> realized_side_contracts;
    std::array<double, 3> realization_latency_ms{{0.0, 0.0, 0.0}};
  } pending_team_solution_;
};

}  // namespace multi_uav_formation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_uav_topology_coordinator");
  ros::NodeHandle node("~");
  multi_uav_formation::MultiUavTopologyCoordinator coordinator(node);
  ros::spin();
  return 0;
}
