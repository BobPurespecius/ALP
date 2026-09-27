#ifndef MULTI_UAV_FORMATION_COOPERATIVE_VIEWPOINT_CORE_H
#define MULTI_UAV_FORMATION_COOPERATIVE_VIEWPOINT_CORE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <multi_uav_formation/tracking_visibility_geometry.h>
#include <multi_uav_formation/team_visibility_preference.h>
#include <plan_env/static_los_geometry.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace multi_uav_formation
{

// Small, deterministic state holder shared by the ROS manager and its
// contract test.  Real committed-to-superseded durations remain useful
// diagnostics, but formation orientation is a slower sample-and-hold state:
// its cadence and prediction horizon are deliberately independent of an
// individual receding-horizon trajectory's execution lifetime.
class AdaptiveExecutionWindow
{
public:
  void configure(const std::size_t history_limit,
                 const double bootstrap_horizon,
                 const double minimum_horizon,
                 const double maximum_horizon,
                 const double outer_period = 0.5,
                 const double prediction_horizon = 1.5)
  {
    history_limit_ = std::max<std::size_t>(1, history_limit);
    minimum_horizon_ = std::max(1.0e-3, minimum_horizon);
    maximum_horizon_ = std::max(minimum_horizon_, maximum_horizon);
    bootstrap_horizon_ = clampHorizon(bootstrap_horizon);
    outer_period_ = std::max(1.0e-3, outer_period);
    prediction_horizon_ = std::max(1.0e-3, prediction_horizon);
    while (execution_durations_.size() > history_limit_)
      execution_durations_.pop_front();
  }

  bool recordSupersededDuration(const double duration,
                                const bool target_motion_active,
                                const bool final_stop)
  {
    if (!target_motion_active || final_stop || !std::isfinite(duration) ||
        duration <= 1.0e-6)
      return false;
    execution_durations_.push_back(duration);
    while (execution_durations_.size() > history_limit_)
      execution_durations_.pop_front();
    return true;
  }

  std::size_t historySize() const { return execution_durations_.size(); }

  double executionMedian() const
  {
    if (execution_durations_.empty())
      return bootstrap_horizon_;
    std::vector<double> values(execution_durations_.begin(),
                               execution_durations_.end());
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    const double median = values.size() % 2
                              ? values[middle]
                              : 0.5 * (values[middle - 1] + values[middle]);
    return clampHorizon(median);
  }

  double evaluationHorizon(const double /* candidate_remaining */) const
  {
    return prediction_horizon_;
  }

  double outerCadence() const { return outer_period_; }
  double predictionHorizon() const { return prediction_horizon_; }

  bool outerDue(const double now, const bool mission_started,
                const bool transition_active,
                const bool selection_pending) const
  {
    if (!mission_started || transition_active || selection_pending ||
        !std::isfinite(now))
      return false;
    return !std::isfinite(last_outer_evaluation_time_) ||
           now - last_outer_evaluation_time_ + 1.0e-9 >= outerCadence();
  }

  void markOuterEvaluation(const double now)
  {
    if (std::isfinite(now))
      last_outer_evaluation_time_ = now;
  }

  double lastOuterEvaluationTime() const
  {
    return last_outer_evaluation_time_;
  }

private:
  double clampHorizon(const double value) const
  {
    const double finite = std::isfinite(value) ? value : minimum_horizon_;
    return std::max(minimum_horizon_, std::min(maximum_horizon_, finite));
  }

  std::size_t history_limit_{8};
  double bootstrap_horizon_{1.0};
  double minimum_horizon_{0.1};
  double maximum_horizon_{2.0};
  double outer_period_{0.5};
  double prediction_horizon_{1.5};
  std::deque<double> execution_durations_;
  double last_outer_evaluation_time_{
      std::numeric_limits<double>::quiet_NaN()};
};

enum class ViewpointState
{
  HOLD,
  TRANSITION
};

inline const char *viewpointStateName(const ViewpointState state)
{
  return state == ViewpointState::TRANSITION ? "TRANSITION" : "HOLD";
}

struct ViewpointMetrics
{
  bool valid{false};
  std::array<double, 3> visibility{{0.0, 0.0, 0.0}};
  double all3{0.0};
  double atleast2{0.0};
  double none{1.0};
  double mean_visible_count{0.0};
  double min_los_clearance{-std::numeric_limits<double>::infinity()};
  double movement_distance{std::numeric_limits<double>::infinity()};
  double min_pairwise_angle{0.0};
  int sample_count{0};
};

struct EncirclementCandidateDiagnostic
{
  double phi0{std::numeric_limits<double>::quiet_NaN()};
  double signed_rotation{std::numeric_limits<double>::quiet_NaN()};
  bool arc_valid{false};
  ViewpointMetrics metrics;
};

struct ViewpointStep
{
  ViewpointState state{ViewpointState::HOLD};
  int active_uav{-1};
  int evaluated_uav{-1};
  int valid_candidate_count{0};
  std::array<bool, 3> reference_valid{{false, false, false}};
  std::array<double, 3> commanded_angles{{0.0, 0.0, 0.0}};
  std::array<double, 3> target_angles{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular_rates{{0.0, 0.0, 0.0}};
  std::array<double, 3> actual_los_clearance{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  std::array<double, 3> blocked_duration{{0.0, 0.0, 0.0}};
  std::array<double, 3> tracking_error{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  std::array<Eigen::Vector3d, 3> positions;
  std::array<Eigen::Vector3d, 3> velocities;
  double selected_candidate_phi{std::numeric_limits<double>::quiet_NaN()};
  double candidate_visibility{std::numeric_limits<double>::quiet_NaN()};
  double candidate_all3{std::numeric_limits<double>::quiet_NaN()};
  double candidate_none{std::numeric_limits<double>::quiet_NaN()};
  double candidate_mean_visible{std::numeric_limits<double>::quiet_NaN()};
  double candidate_diversity{std::numeric_limits<double>::quiet_NaN()};
  double current_visibility{std::numeric_limits<double>::quiet_NaN()};
  double current_all3{std::numeric_limits<double>::quiet_NaN()};
  double current_none{std::numeric_limits<double>::quiet_NaN()};
  double current_mean_visible{std::numeric_limits<double>::quiet_NaN()};
  double transition_elapsed{0.0};
  double reference_error{std::numeric_limits<double>::quiet_NaN()};
  bool arrived{false};
  bool reselection_blocked{false};
  bool direction_reversal{false};
  bool encirclement_enabled{false};
  bool periodic_visibility_evaluation{false};
  bool dynamic_los_evaluated{false};
  bool camera_fov_evaluated{false};
  bool yaw_dynamics_evaluated{false};
  bool persistent_static_los_loss{false};
  bool using_last_valid_reference{false};
  bool initial_slot_assignment_changed{false};
  double phi0_current{std::numeric_limits<double>::quiet_NaN()};
  double phi0_selected{std::numeric_limits<double>::quiet_NaN()};
  double phi0_delta{std::numeric_limits<double>::quiet_NaN()};
  std::string selection_reason{"UNINITIALIZED"};
  ViewpointMetrics metrics;
  std::array<int, 3> slot_assignment{{0, 1, 2}};
  std::vector<EncirclementCandidateDiagnostic> encirclement_candidates;
};

class CooperativeViewpointCore
{
public:
  CooperativeViewpointCore()
  {
    offsets_[0] = Eigen::Vector3d(-1.5, -0.85, 0.0);
    offsets_[1] = Eigen::Vector3d(-1.7, 0.0, 0.0);
    offsets_[2] = Eigen::Vector3d(-1.5, 0.85, 0.0);
  }

  bool loadScene(const std::string &scene_file, std::string &reason)
  {
    if (!geometry_.loadScene(scene_file, reason))
      return false;
    dynamic_cylinders_.clear();
    try
    {
      boost::property_tree::ptree root;
      boost::property_tree::read_json(scene_file, root);
      const double default_height =
          root.get<double>("egoPlanner.obstacleHeight", 3.0);
      const auto obstacles = root.get_child_optional("movingObstacleData");
      if (obstacles)
      {
        for (const auto &entry : *obstacles)
        {
          const auto &node = entry.second;
          const auto center_node = node.get_child_optional("centerENU");
          if (!center_node)
            continue;
          std::vector<double> center_values;
          for (const auto &value : *center_node)
            center_values.push_back(value.second.get_value<double>());
          if (center_values.size() < 2)
            continue;
          std::vector<double> axis_values;
          const auto axis_node = node.get_child_optional("axisENU");
          if (axis_node)
            for (const auto &value : *axis_node)
              axis_values.push_back(value.second.get_value<double>());

          DynamicLosCylinder obstacle;
          obstacle.name = node.get<std::string>("modelName", entry.first);
          obstacle.center = Eigen::Vector2d(center_values[0], center_values[1]);
          obstacle.axis = axis_values.size() >= 2
              ? Eigen::Vector2d(axis_values[0], axis_values[1])
              : Eigen::Vector2d::UnitX();
          if (obstacle.axis.norm() <= 1.0e-6)
            obstacle.axis = Eigen::Vector2d::UnitX();
          else
            obstacle.axis.normalize();
          obstacle.amplitude = node.get<double>("amplitude", 0.0);
          obstacle.period = node.get<double>("period", 1.0);
          obstacle.phase = node.get<double>("phase", 0.0);
          obstacle.radius = node.get<double>("radius", 0.5);
          obstacle.z_min = node.get<double>("zMin", 0.0);
          obstacle.z_max = obstacle.z_min +
              node.get<double>("height", default_height);
          if (obstacle.center.allFinite() && obstacle.axis.allFinite() &&
              std::isfinite(obstacle.amplitude) &&
              std::isfinite(obstacle.period) && obstacle.period > 0.0 &&
              std::isfinite(obstacle.phase) &&
              std::isfinite(obstacle.radius) && obstacle.radius > 0.0 &&
              std::isfinite(obstacle.z_min) &&
              std::isfinite(obstacle.z_max) &&
              obstacle.z_max > obstacle.z_min)
            dynamic_cylinders_.push_back(obstacle);
        }
      }
    }
    catch (const std::exception &error)
    {
      reason = std::string("DYNAMIC_JSON_PARSE_ERROR:") + error.what();
      dynamic_cylinders_.clear();
      return false;
    }
    reason = "OK";
    return true;
  }

  void setGeometry(const std::vector<ego_planner::StaticLosCylinder> &cylinders)
  {
    geometry_.setCylinders(cylinders);
  }

  void setWalls(const std::vector<ego_planner::StaticLosWall> &walls)
  {
    geometry_.setWalls(walls);
  }

  void setDynamicObstacles(
      const std::vector<DynamicLosCylinder> &dynamic_cylinders)
  {
    dynamic_cylinders_ = dynamic_cylinders;
  }

  void setDynamicMotionElapsed(const double elapsed)
  {
    dynamic_motion_elapsed_ = std::isfinite(elapsed)
        ? std::max(0.0, elapsed) : 0.0;
  }

  void setCameraContract(const TrackingCameraContract &camera)
  {
    camera_ = camera;
    camera_.min_range = std::max(0.0, camera_.min_range);
    camera_.max_range = std::max(camera_.min_range, camera_.max_range);
  }

  void setObserverYawStates(
      const std::array<PredictedYawState, 3> &observer_yaw_states)
  {
    observer_yaw_states_ = observer_yaw_states;
  }

  void setObserverPositions(
      const std::array<Eigen::Vector3d, 3> &observer_positions,
      const std::array<bool, 3> &observer_position_valid)
  {
    observer_positions_ = observer_positions;
    observer_position_valid_ = observer_position_valid;
  }

  void setObservedVisibleCount(const int visible_count, const bool valid)
  {
    observed_visible_count_valid_ = valid;
    observed_visible_count_ = std::max(0, std::min(3, visible_count));
    if (!valid)
      observed_all3_deficit_duration_ = 0.0;
  }

  const TrackingCameraContract &cameraContract() const { return camera_; }
  const ego_planner::StaticLosGeometry &staticGeometry() const { return geometry_; }
  const std::vector<DynamicLosCylinder> &dynamicObstacles() const { return dynamic_cylinders_; }
  size_t dynamicObstacleCount() const { return dynamic_cylinders_.size(); }

  void setOffsets(const std::array<Eigen::Vector3d, 3> &offsets)
  {
    offsets_ = offsets;
  }

  const std::array<Eigen::Vector3d, 3> &offsets() const { return offsets_; }

  void setEncirclementTracking(const bool enabled)
  {
    if (encirclement_enabled_ == enabled)
      return;
    encirclement_enabled_ = enabled;
    initialized_ = false;
    state_ = ViewpointState::HOLD;
    active_uav_ = -1;
    arrival_hold_accumulator_ = 0.0;
    transition_direction_ = 0;
    transition_remaining_angle_ = 0.0;
    last_transition_end_time_ =
        std::numeric_limits<double>::quiet_NaN();
    blocked_duration_ = {{0.0, 0.0, 0.0}};
    slot_for_uav_ = {{0, 1, 2}};
    initial_slot_assignment_changed_ = false;
    have_last_valid_reference_ = false;
  }

  bool encirclementTrackingEnabled() const { return encirclement_enabled_; }

  // The trajectory-level coordinator is the sole runtime authority for a
  // new common angle.  The core only executes the request through its
  // existing rate-limited, statically revalidated transition contract.
  void requestJointSelectedPhi0(const double phi0)
  {
    if (encirclement_enabled_ && std::isfinite(phi0))
    {
      joint_selected_phi0_ = wrapToPi(phi0);
      joint_selection_pending_ = true;
    }
  }

  void configure(const double horizon, const double sample_dt,
                 const double min_range, const double max_range,
                 const double los_margin, const double reference_clearance,
                 const double visibility_bad_hold,
                 const double trigger_tracking_error,
                 const double theta_sep_min,
                 const double min_visibility_improvement,
                 const double max_angular_rate,
                 const double arrive_distance,
                 const double arrive_hold,
                 const double k2_near_best_tolerance = 0.01)
  {
    horizon_ = std::max(0.0, horizon);
    sample_dt_ = std::max(0.02, sample_dt);
    min_range_ = std::max(0.0, min_range);
    max_range_ = std::max(min_range_, max_range);
    camera_.min_range = min_range_;
    camera_.max_range = max_range_;
    los_margin_ = std::max(0.0, los_margin);
    reference_clearance_ = std::max(0.0, reference_clearance);
    visibility_bad_hold_ = std::max(0.0, visibility_bad_hold);
    trigger_tracking_error_ = std::max(0.0, trigger_tracking_error);
    theta_sep_min_ = std::max(0.0, theta_sep_min);
    min_visibility_improvement_ = std::max(0.0, min_visibility_improvement);
    max_angular_rate_ = std::max(0.0, max_angular_rate);
    arrive_distance_ = std::max(0.0, arrive_distance);
    arrive_hold_ = std::max(0.0, arrive_hold);
    k2_near_best_tolerance_ =
        std::max(0.0, std::min(1.0, k2_near_best_tolerance));
  }

  static double wrapToPi(double angle)
  {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle <= -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  static double angleDistance(const double first, const double second)
  {
    return std::abs(wrapToPi(first - second));
  }

  Eigen::Vector3d rotatedOffset(const int uav, const double angle) const
  {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Eigen::Vector3d result = offsets_[uav];
    result.x() = c * offsets_[uav].x() - s * offsets_[uav].y();
    result.y() = s * offsets_[uav].x() + c * offsets_[uav].y();
    return result;
  }

  std::array<Eigen::Vector3d, 3> referencePositions(
      const Eigen::Vector3d &target_position,
      const std::array<double, 3> &angles) const
  {
    std::array<Eigen::Vector3d, 3> result;
    for (int uav = 0; uav < 3; ++uav)
      result[uav] = target_position + rotatedOffset(uav, angles[uav]);
    return result;
  }

  std::array<Eigen::Vector3d, 3> referencePositions(
      const Eigen::Vector3d &target_position, const double common_angle) const
  {
    return referencePositions(target_position,
                              {{common_angle, common_angle, common_angle}});
  }

  std::array<double, 3> encirclementAngles(const double phi0) const
  {
    return encirclementAngles(phi0, slot_for_uav_);
  }

  std::array<double, 3> encirclementAngles(
      const double phi0, const std::array<int, 3> &slot_for_uav) const
  {
    std::array<double, 3> result;
    for (int uav = 0; uav < 3; ++uav)
      result[uav] = wrapToPi(
          phi0 + 2.0 * M_PI * slot_for_uav[uav] / 3.0);
    return result;
  }

  std::array<Eigen::Vector3d, 3> encirclementReferencePositions(
      const Eigen::Vector3d &target_position, const double phi0) const
  {
    return encirclementReferencePositions(
        target_position, phi0, slot_for_uav_);
  }

  std::array<Eigen::Vector3d, 3> encirclementReferencePositions(
      const Eigen::Vector3d &target_position, const double phi0,
      const std::array<int, 3> &slot_for_uav) const
  {
    std::array<Eigen::Vector3d, 3> result;
    const auto bearings = encirclementAngles(phi0, slot_for_uav);
    for (int uav = 0; uav < 3; ++uav)
    {
      const double radius = offsets_[uav].head<2>().norm();
      result[uav] = target_position + Eigen::Vector3d(
          radius * std::cos(bearings[uav]),
          radius * std::sin(bearings[uav]), offsets_[uav].z());
    }
    return result;
  }

  double estimateInitialPhi0(
      const Eigen::Vector3d &target_position,
      const std::array<Eigen::Vector3d, 3> &odom_positions,
      const std::array<bool, 3> &odom_valid) const
  {
    return estimateInitialPhi0(
        target_position, odom_positions, odom_valid, slot_for_uav_);
  }

  double estimateInitialPhi0(
      const Eigen::Vector3d &target_position,
      const std::array<Eigen::Vector3d, 3> &odom_positions,
      const std::array<bool, 3> &odom_valid,
      const std::array<int, 3> &slot_for_uav) const
  {
    const bool all_odom_valid = odom_valid[0] && odom_valid[1] &&
                                odom_valid[2] &&
                                odom_positions[0].allFinite() &&
                                odom_positions[1].allFinite() &&
                                odom_positions[2].allFinite();
    double cosine_sum = 0.0;
    double sine_sum = 0.0;
    double first_estimate = 0.0;
    for (int uav = 0; uav < 3; ++uav)
    {
      Eigen::Vector2d relative;
      if (all_odom_valid)
        relative = (odom_positions[uav] - target_position).head<2>();
      else
        relative = offsets_[uav].head<2>();
      const double bearing = relative.squaredNorm() > 1.0e-12
          ? std::atan2(relative.y(), relative.x())
          : 2.0 * M_PI * uav / 3.0;
      const double estimate = wrapToPi(
          bearing - 2.0 * M_PI * slot_for_uav[uav] / 3.0);
      if (uav == 0)
        first_estimate = estimate;
      cosine_sum += std::cos(estimate);
      sine_sum += std::sin(estimate);
    }
    if (std::hypot(cosine_sum, sine_sum) <= 1.0e-9)
      return first_estimate;
    return std::atan2(sine_sum, cosine_sum);
  }

  double observationAngle(const int uav, const double phi) const
  {
    const Eigen::Vector3d offset = rotatedOffset(uav, phi);
    return std::atan2(offset.y(), offset.x());
  }

  double minimumPairwiseAngle(const std::array<double, 3> &angles) const
  {
    double minimum = std::numeric_limits<double>::infinity();
    for (int first = 0; first < 3; ++first)
      for (int second = first + 1; second < 3; ++second)
        minimum = std::min(minimum, angleDistance(
            observationAngle(first, angles[first]),
            observationAngle(second, angles[second])));
    return minimum;
  }

  bool pairwiseSeparationValid(const std::array<double, 3> &angles) const
  {
    return minimumPairwiseAngle(angles) + 1.0e-9 >= theta_sep_min_;
  }

  ViewpointMetrics evaluate(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const std::array<double, 3> &angles,
      const int adjusted_uav,
      const std::array<Eigen::Vector3d, 3> &odom_positions,
      const std::array<bool, 3> &odom_valid) const
  {
    ViewpointMetrics result;
    if (!geometry_.valid() || !target_position.allFinite() ||
        !target_velocity.allFinite() || !pairwiseSeparationValid(angles))
      return result;

    result.min_pairwise_angle = minimumPairwiseAngle(angles);
    const int steps = std::max(
        1, static_cast<int>(std::ceil(horizon_ / sample_dt_)));
    std::array<int, 3> visible_count{{0, 0, 0}};
    int all3_count = 0;
    int atleast2_count = 0;
    int none_count = 0;
    double min_clearance = std::numeric_limits<double>::infinity();
    bool references_static_free = true;

    for (int step = 0; step <= steps; ++step)
    {
      const double sample_time = std::min(horizon_, step * sample_dt_);
      const Eigen::Vector3d target =
          target_position + target_velocity * sample_time;
      const auto references = referencePositions(target, angles);
      int visible_uavs = 0;
      for (int uav = 0; uav < 3; ++uav)
      {
        double point_clearance = std::numeric_limits<double>::infinity();
        const bool point_query_valid = geometry_.querySegmentClearance(
            references[uav], references[uav], point_clearance);
        if (!point_query_valid ||
            ((adjusted_uav < 0 || adjusted_uav == uav) &&
             point_clearance < reference_clearance_))
          references_static_free = false;

        double los_clearance = std::numeric_limits<double>::infinity();
        if (!geometry_.querySegmentClearance(references[uav], target,
                                             los_clearance))
          references_static_free = false;
        if (adjusted_uav < 0 || adjusted_uav == uav)
          min_clearance = std::min(min_clearance, los_clearance);
        const double distance = (references[uav] - target).norm();
        const bool visible = std::isfinite(distance) &&
                             distance >= min_range_ &&
                             distance <= max_range_ &&
                             std::isfinite(los_clearance) &&
                             los_clearance >= los_margin_;
        if (visible)
        {
          ++visible_count[uav];
          ++visible_uavs;
        }
      }
      if (visible_uavs == 3)
        ++all3_count;
      if (visible_uavs >= 2)
        ++atleast2_count;
      if (visible_uavs == 0)
        ++none_count;
      ++result.sample_count;
    }

    if (result.sample_count <= 0)
      return result;
    for (int uav = 0; uav < 3; ++uav)
      result.visibility[uav] =
          static_cast<double>(visible_count[uav]) / result.sample_count;
    result.mean_visible_count = result.visibility[0] + result.visibility[1] +
                                result.visibility[2];
    result.all3 = static_cast<double>(all3_count) / result.sample_count;
    result.atleast2 = static_cast<double>(atleast2_count) / result.sample_count;
    result.none = static_cast<double>(none_count) / result.sample_count;
    result.min_los_clearance = min_clearance;
    result.movement_distance = 0.0;
    if (adjusted_uav >= 0)
    {
      if (!odom_valid[adjusted_uav] ||
          !odom_positions[adjusted_uav].allFinite())
        return result;
      result.movement_distance =
          (referencePositions(target_position, angles)[adjusted_uav] -
           odom_positions[adjusted_uav]).norm();
    }
    result.valid = references_static_free &&
                   std::isfinite(result.min_los_clearance) &&
                   std::isfinite(result.movement_distance) &&
                   std::isfinite(result.min_pairwise_angle);
    return result;
  }

  ViewpointMetrics evaluateEncirclement(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const double phi0) const
  {
    return evaluateEncirclement(target_position, target_velocity, phi0,
                                slot_for_uav_);
  }

  ViewpointMetrics evaluateEncirclement(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const double phi0,
      const std::array<int, 3> &slot_for_uav) const
  {
    ViewpointMetrics result;
    if (!geometry_.valid() || !target_position.allFinite() ||
        !target_velocity.allFinite())
      return result;

    const auto bearings = encirclementAngles(phi0, slot_for_uav);
    result.min_pairwise_angle = std::numeric_limits<double>::infinity();
    for (int first = 0; first < 3; ++first)
      for (int second = first + 1; second < 3; ++second)
        result.min_pairwise_angle = std::min(
            result.min_pairwise_angle,
            angleDistance(bearings[first], bearings[second]));
    if (result.min_pairwise_angle + 1.0e-9 < theta_sep_min_)
      return result;

    const int steps = std::max(
        1, static_cast<int>(std::ceil(horizon_ / sample_dt_)));
    std::array<int, 3> visible_count{{0, 0, 0}};
    int all3_count = 0;
    int atleast2_count = 0;
    int none_count = 0;
    double min_clearance = std::numeric_limits<double>::infinity();
    bool references_static_free = true;

    for (int step = 0; step <= steps; ++step)
    {
      const double sample_time = std::min(horizon_, step * sample_dt_);
      const Eigen::Vector3d target =
          target_position + target_velocity * sample_time;
      const auto references = encirclementReferencePositions(
          target, phi0, slot_for_uav);
      int visible_uavs = 0;
      for (int uav = 0; uav < 3; ++uav)
      {
        double point_clearance = std::numeric_limits<double>::infinity();
        if (!geometry_.querySegmentClearance(
                references[uav], references[uav], point_clearance) ||
            point_clearance < reference_clearance_)
          references_static_free = false;

        double los_clearance = std::numeric_limits<double>::infinity();
        if (!geometry_.querySegmentClearance(references[uav], target,
                                             los_clearance))
          references_static_free = false;
        min_clearance = std::min(min_clearance, los_clearance);
        const double distance = (references[uav] - target).norm();
        const bool visible = std::isfinite(distance) &&
                             distance >= min_range_ &&
                             distance <= max_range_ &&
                             std::isfinite(los_clearance) &&
                             los_clearance >= los_margin_;
        if (visible)
        {
          ++visible_count[uav];
          ++visible_uavs;
        }
      }
      if (visible_uavs == 3)
        ++all3_count;
      if (visible_uavs >= 2)
        ++atleast2_count;
      if (visible_uavs == 0)
        ++none_count;
      ++result.sample_count;
    }

    if (result.sample_count <= 0)
      return result;
    for (int uav = 0; uav < 3; ++uav)
      result.visibility[uav] =
          static_cast<double>(visible_count[uav]) / result.sample_count;
    result.mean_visible_count = result.visibility[0] + result.visibility[1] +
                                result.visibility[2];
    result.all3 = static_cast<double>(all3_count) / result.sample_count;
    result.atleast2 = static_cast<double>(atleast2_count) /
                      result.sample_count;
    result.none = static_cast<double>(none_count) / result.sample_count;
    result.min_los_clearance = min_clearance;
    result.movement_distance = 0.0;
    result.valid = references_static_free &&
                   std::isfinite(result.min_los_clearance) &&
                   std::isfinite(result.min_pairwise_angle);
    return result;
  }

  bool dynamicLosClear(const Eigen::Vector3d &observer,
                       const Eigen::Vector3d &target,
                       const double elapsed) const
  {
    for (const auto &obstacle : dynamic_cylinders_)
      if (segmentIntersectsVerticalCylinder(
              observer, target, obstacle.centerAt(elapsed),
              obstacle.radius + los_margin_, obstacle.z_min, obstacle.z_max))
        return false;
    return true;
  }

  // Predict visibility over the rate-limited rotation itself.  A signed
  // rotation of zero evaluates the current topology.  Visibility uses the
  // same range/static-LOS/dynamic-LOS/camera-frame contract as execution
  // telemetry, while candidate validity remains the static feasibility of the
  // soft reference positions.
  ViewpointMetrics evaluateEncirclementTransition(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const double start_phi0, const double signed_rotation) const
  {
    ViewpointMetrics result;
    if (!geometry_.valid() || !target_position.allFinite() ||
        !target_velocity.allFinite() || !std::isfinite(start_phi0) ||
        !std::isfinite(signed_rotation))
      return result;

    const auto initial_angles = encirclementAngles(start_phi0);
    const auto initial_references = encirclementReferencePositions(
        target_position, start_phi0);
    std::array<Eigen::Vector3d, 3> tracking_offsets{{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()}};
    for (int uav = 0; uav < 3; ++uav)
      if (observer_position_valid_[uav] &&
          observer_positions_[uav].allFinite())
        tracking_offsets[uav] =
            observer_positions_[uav] - initial_references[uav];
    result.min_pairwise_angle = std::numeric_limits<double>::infinity();
    for (int first = 0; first < 3; ++first)
      for (int second = first + 1; second < 3; ++second)
        result.min_pairwise_angle = std::min(
            result.min_pairwise_angle,
            angleDistance(initial_angles[first], initial_angles[second]));
    if (result.min_pairwise_angle + 1.0e-9 < theta_sep_min_)
      return result;

    const double transition_duration = max_angular_rate_ > 1.0e-9
        ? std::abs(signed_rotation) / max_angular_rate_ : 0.0;
    // A finite visibility horizon must not truncate a slow reference
    // rotation before its endpoint.  The full rate-limited arc is always
    // represented; a stationary/current candidate still uses the configured
    // horizon.
    const double evaluation_duration = std::max(horizon_, transition_duration);
    const int steps = std::max(
        1, static_cast<int>(std::ceil(evaluation_duration / sample_dt_)));
    std::array<int, 3> visible_count{{0, 0, 0}};
    int all3_count = 0;
    int atleast2_count = 0;
    int none_count = 0;
    int visible_total = 0;
    double min_clearance = std::numeric_limits<double>::infinity();
    bool references_static_free = true;
    std::array<PredictedYawState, 3> yaw_states = observer_yaw_states_;

    for (int step = 0; step <= steps; ++step)
    {
      const double sample_time = std::min(
          evaluation_duration, step * sample_dt_);
      const double progress = std::copysign(
          std::min(std::abs(signed_rotation),
                   max_angular_rate_ * sample_time),
          signed_rotation);
      const double phi0 = wrapToPi(start_phi0 + progress);
      const Eigen::Vector3d target =
          target_position + target_velocity * sample_time;
      const auto references = encirclementReferencePositions(target, phi0);
      int visible_uavs = 0;
      for (int uav = 0; uav < 3; ++uav)
      {
        // The common-phi layer publishes a soft reference; the vehicle does
        // not teleport onto it when a rotation begins.  Preserve the current
        // measured reference-tracking residual across the short prediction
        // window.  This makes t=0 exactly match executed geometry and avoids
        // scoring a collision-free reference ray while the real vehicle is
        // still on the other side of an occluder.
        const Eigen::Vector3d predicted_observer =
            references[uav] + tracking_offsets[uav];
        const Eigen::Vector3d relative = target - predicted_observer;
        const double target_bearing =
            std::atan2(relative.y(), relative.x());
        if (!yaw_states[uav].valid)
        {
          yaw_states[uav].valid = true;
          yaw_states[uav].yaw = target_bearing;
          yaw_states[uav].yaw_rate = 0.0;
        }
        else if (step > 0)
          yaw_states[uav] = advanceTargetFacingYaw(
              yaw_states[uav], target_bearing, sample_dt_);

        double point_clearance = std::numeric_limits<double>::infinity();
        if (!geometry_.querySegmentClearance(
                references[uav], references[uav], point_clearance) ||
            point_clearance < reference_clearance_)
          references_static_free = false;

        double static_clearance = std::numeric_limits<double>::infinity();
        if (!geometry_.querySegmentClearance(
                predicted_observer, target, static_clearance))
          references_static_free = false;
        min_clearance = std::min(min_clearance, static_clearance);
        const bool static_los_clear = std::isfinite(static_clearance) &&
                                      static_clearance >= los_margin_;
        const bool dynamic_los_clear = dynamicLosClear(
            predicted_observer, target,
            dynamic_motion_elapsed_ + sample_time);
        const Eigen::Quaterniond world_from_body(
            Eigen::AngleAxisd(yaw_states[uav].yaw,
                              Eigen::Vector3d::UnitZ()));
        const CameraFovResult fov = targetInTrackingCameraFov(
            predicted_observer, world_from_body, target, camera_);
        const bool visible = static_los_clear && dynamic_los_clear &&
                             fov.valid;
        if (visible)
        {
          ++visible_count[uav];
          ++visible_uavs;
          ++visible_total;
        }
      }
      if (visible_uavs == 3)
        ++all3_count;
      if (visible_uavs >= 2)
        ++atleast2_count;
      if (visible_uavs == 0)
        ++none_count;
      ++result.sample_count;
    }

    if (result.sample_count <= 0)
      return result;
    for (int uav = 0; uav < 3; ++uav)
      result.visibility[uav] =
          static_cast<double>(visible_count[uav]) / result.sample_count;
    result.all3 = static_cast<double>(all3_count) / result.sample_count;
    result.atleast2 = static_cast<double>(atleast2_count) /
                      result.sample_count;
    result.none = static_cast<double>(none_count) / result.sample_count;
    result.mean_visible_count = static_cast<double>(visible_total) /
                                result.sample_count;
    result.min_los_clearance = min_clearance;
    result.movement_distance = std::abs(signed_rotation);
    result.valid = references_static_free &&
                   std::isfinite(result.min_los_clearance) &&
                   std::isfinite(result.min_pairwise_angle);
    return result;
  }

  static bool betterEncirclementCandidate(
      const ViewpointMetrics &lhs, const double lhs_phi0,
      const ViewpointMetrics &rhs, const double rhs_phi0,
      const double current_phi0,
      const double k2_near_best_tolerance = 0.01)
  {
    const double eps = 1.0e-9;
    if (lhs.valid != rhs.valid)
      return lhs.valid;
    if (!lhs.valid)
      return false;
    const double lhs_spread = spreadPreferenceScore(
        lhs.min_pairwise_angle * 180.0 / M_PI, 60.0);
    const double rhs_spread = spreadPreferenceScore(
        rhs.min_pairwise_angle * 180.0 / M_PI, 60.0);
    const double best_k2 = std::max(lhs.atleast2, rhs.atleast2);
    const TeamVisibilityPreference lhs_visibility{
        lhs.atleast2, lhs.mean_visible_count, lhs.all3, lhs.none, lhs_spread};
    const TeamVisibilityPreference rhs_visibility{
        rhs.atleast2, rhs.mean_visible_count, rhs.all3, rhs.none, rhs_spread};
    if (betterTeamVisibilityWithinNearBestK2(
            lhs_visibility, rhs_visibility, best_k2,
            k2_near_best_tolerance))
      return true;
    if (betterTeamVisibilityWithinNearBestK2(
            rhs_visibility, lhs_visibility, best_k2,
            k2_near_best_tolerance))
      return false;
    return angleDistance(lhs_phi0, current_phi0) <
           angleDistance(rhs_phi0, current_phi0) - eps;
  }

  static bool betterForUav(const ViewpointMetrics &lhs,
                           const ViewpointMetrics &rhs,
                           const int adjusted_uav)
  {
    (void)adjusted_uav;
    if (lhs.valid != rhs.valid)
      return lhs.valid;
    if (!lhs.valid)
      return false;
    const TeamVisibilityPreference lhs_visibility{
        lhs.atleast2, lhs.mean_visible_count, lhs.all3, lhs.none, 0.0};
    const TeamVisibilityPreference rhs_visibility{
        rhs.atleast2, rhs.mean_visible_count, rhs.all3, rhs.none, 0.0};
    const double best_k2 = std::max(lhs.atleast2, rhs.atleast2);
    if (betterTeamVisibilityWithinNearBestK2(
            lhs_visibility, rhs_visibility, best_k2, 0.0))
      return true;
    if (betterTeamVisibilityWithinNearBestK2(
            rhs_visibility, lhs_visibility, best_k2, 0.0))
      return false;
    return lhs.movement_distance < rhs.movement_distance - 1.0e-9;
  }

  bool singleUavTransitionValid(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const std::array<double, 3> &base_angles,
      const int active_uav,
      const double target_angle) const
  {
    const double delta = wrapToPi(target_angle - base_angles[active_uav]);
    const int angle_steps = std::max(
        1, static_cast<int>(std::ceil(std::abs(delta) / (M_PI / 16.0))));
    for (int angle_step = 0; angle_step <= angle_steps; ++angle_step)
    {
      const double fraction =
          static_cast<double>(angle_step) / angle_steps;
      std::array<double, 3> angles = base_angles;
      angles[active_uav] = wrapToPi(
          base_angles[active_uav] + fraction * delta);
      if (!pairwiseSeparationValid(angles))
        return false;
      for (int time_index = 0; time_index < 3; ++time_index)
      {
        const double sample_time = 0.5 * time_index * horizon_;
        const Eigen::Vector3d target =
            target_position + target_velocity * sample_time;
        const Eigen::Vector3d reference =
            target + rotatedOffset(active_uav, angles[active_uav]);
        double clearance = std::numeric_limits<double>::infinity();
        if (!geometry_.querySegmentClearance(reference, reference, clearance) ||
            clearance < reference_clearance_)
          return false;
      }
    }
    return true;
  }

  ViewpointStep fixedFormationStep(const Eigen::Vector3d &target_position,
                                   const Eigen::Vector3d &target_velocity,
                                   const std::string &reason) const
  {
    ViewpointStep result;
    result.selection_reason = reason;
    result.commanded_angles = {{0.0, 0.0, 0.0}};
    result.target_angles = result.commanded_angles;
    result.positions = referencePositions(target_position, result.commanded_angles);
    for (int uav = 0; uav < 3; ++uav)
    {
      result.velocities[uav] = target_velocity;
      result.reference_valid[uav] = target_position.allFinite() &&
                                    target_velocity.allFinite();
    }
    return result;
  }

  double initialBearingDisplacement(
      const Eigen::Vector3d &target_position,
      const std::array<Eigen::Vector3d, 3> &odom_positions,
      const std::array<bool, 3> &odom_valid,
      const double phi0,
      const std::array<int, 3> &slot_for_uav) const
  {
    double result = 0.0;
    const auto angles = encirclementAngles(phi0, slot_for_uav);
    for (int uav = 0; uav < 3; ++uav)
    {
      Eigen::Vector2d relative = offsets_[uav].head<2>();
      if (odom_valid[uav] && odom_positions[uav].allFinite())
        relative = (odom_positions[uav] - target_position).head<2>();
      const double actual_bearing = relative.squaredNorm() > 1.0e-12
          ? std::atan2(relative.y(), relative.x()) : angles[uav];
      result += angleDistance(actual_bearing, angles[uav]);
    }
    return result;
  }

  void initializeEncirclement(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const std::array<Eigen::Vector3d, 3> &odom_positions,
      const std::array<bool, 3> &odom_valid)
  {
    // Slot identity is selected exactly once.  Runtime permutation would make
    // vehicles exchange sides and create the very crossings that this soft
    // reference layer is intended to avoid.
    static const std::array<std::array<int, 3>, 6> permutations{{
        {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
        {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    double best_displacement = std::numeric_limits<double>::infinity();
    bool best_valid = false;
    double best_phi0 = 0.0;
    std::array<int, 3> best_assignment{{0, 1, 2}};
    for (const auto &assignment : permutations)
    {
      const double phi0 = estimateInitialPhi0(
          target_position, odom_positions, odom_valid, assignment);
      const ViewpointMetrics metrics = evaluateEncirclement(
          target_position, target_velocity, phi0, assignment);
      const double displacement = initialBearingDisplacement(
          target_position, odom_positions, odom_valid, phi0, assignment);
      if ((metrics.valid && !best_valid) ||
          (metrics.valid == best_valid &&
           displacement < best_displacement - 1.0e-9))
      {
        best_valid = metrics.valid;
        best_displacement = displacement;
        best_phi0 = phi0;
        best_assignment = assignment;
      }
    }
    slot_for_uav_ = best_assignment;
    initial_slot_assignment_changed_ =
        slot_for_uav_ != std::array<int, 3>{{0, 1, 2}};
    commanded_common_phi0_ = best_phi0;
    target_common_phi0_ = commanded_common_phi0_;
    last_valid_common_phi0_ = commanded_common_phi0_;
    commanded_angles_ = encirclementAngles(commanded_common_phi0_);
    target_angles_ = commanded_angles_;
    last_valid_angles_ = commanded_angles_;
  }

  bool encirclementReferencesInstantlyValid(
      const Eigen::Vector3d &target_position, const double phi0) const
  {
    if (!geometry_.valid() || !target_position.allFinite())
      return false;
    const auto references = encirclementReferencePositions(
        target_position, phi0);
    for (int uav = 0; uav < 3; ++uav)
    {
      double clearance = std::numeric_limits<double>::infinity();
      if (!geometry_.querySegmentClearance(
              references[uav], references[uav], clearance) ||
          clearance < reference_clearance_)
        return false;
    }
    return true;
  }

  bool encirclementTransitionValid(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const double start_phi0, const double goal_phi0) const
  {
    return encirclementTransitionValidSigned(
        target_position, target_velocity, start_phi0,
        wrapToPi(goal_phi0 - start_phi0));
  }

  bool encirclementTransitionValidSigned(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const double start_phi0, const double signed_delta) const
  {
    if (!std::isfinite(signed_delta))
      return false;
    const int angle_steps = std::max(
        1, static_cast<int>(std::ceil(
               std::abs(signed_delta) / (M_PI / 16.0))));
    const double transition_duration = max_angular_rate_ > 1.0e-9
        ? std::abs(signed_delta) / max_angular_rate_ : horizon_;
    for (int angle_step = 0; angle_step <= angle_steps; ++angle_step)
    {
      const double fraction = static_cast<double>(angle_step) / angle_steps;
      const double phi0 = wrapToPi(
          start_phi0 + fraction * signed_delta);
      // Pair rotation progress with the target prediction time.  Requiring
      // every intermediate angle to be valid at every future target position
      // falsely rejects a proactive escape as soon as the current angle has a
      // future obstruction.
      const double sample_time = fraction * transition_duration;
      const Eigen::Vector3d target =
          target_position + target_velocity * sample_time;
      const auto references = encirclementReferencePositions(target, phi0);
      for (int uav = 0; uav < 3; ++uav)
      {
        double clearance = std::numeric_limits<double>::infinity();
        if (!geometry_.querySegmentClearance(
                references[uav], references[uav], clearance) ||
            clearance < reference_clearance_)
          return false;
      }
    }
    return true;
  }

  bool selectFeasibleRotation(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const double start_phi0, const double goal_phi0,
      double &signed_rotation) const
  {
    const double shortest = wrapToPi(goal_phi0 - start_phi0);
    if (std::abs(shortest) <= 1.0e-9)
    {
      signed_rotation = 0.0;
      return true;
    }
    if (encirclementTransitionValidSigned(
            target_position, target_velocity, start_phi0, shortest))
    {
      signed_rotation = shortest;
      return true;
    }
    const double reverse = shortest > 0.0
        ? shortest - 2.0 * M_PI : shortest + 2.0 * M_PI;
    if (encirclementTransitionValidSigned(
            target_position, target_velocity, start_phi0, reverse))
    {
      signed_rotation = reverse;
      return true;
    }
    signed_rotation = shortest;
    return false;
  }

  bool encirclementImproves(const ViewpointMetrics &candidate,
                            const ViewpointMetrics &baseline) const
  {
    const double eps = 1.0e-9;
    if (!candidate.valid)
      return false;
    const double best_k2 = std::max(candidate.atleast2, baseline.atleast2);
    const bool candidate_near_best = inNearBestK2Set(
        {candidate.atleast2, candidate.mean_visible_count, candidate.all3,
         candidate.none, 0.0}, best_k2, k2_near_best_tolerance_);
    const bool baseline_near_best = inNearBestK2Set(
        {baseline.atleast2, baseline.mean_visible_count, baseline.all3,
         baseline.none, 0.0}, best_k2, k2_near_best_tolerance_);
    if (candidate_near_best && !baseline_near_best)
      return true;
    // A mean-visible/all-3-only gain is secondary to K=2.  Do not begin a
    // proactive formation rotation while execution telemetry still reports
    // all three cameras visible: planner tracking and obstacle detours are
    // not instantaneous and can turn that unnecessary rotation into a real
    // short FOV outage.  Reuse the established visibility_bad_hold contract
    // rather than introducing another switch parameter.  K=2/none recovery
    // above remains proactive and immediate.
    const bool actual_secondary_gain_ready =
        !observed_visible_count_valid_ ||
        observed_all3_deficit_duration_ + eps >= visibility_bad_hold_;
    if (candidate_near_best && baseline_near_best &&
        candidate.mean_visible_count >=
            baseline.mean_visible_count + min_visibility_improvement_ - eps &&
        actual_secondary_gain_ready)
      return true;
    if (candidate_near_best && baseline_near_best &&
        std::abs(candidate.mean_visible_count -
                 baseline.mean_visible_count) <= eps &&
        candidate.all3 >=
            baseline.all3 + min_visibility_improvement_ - eps &&
        actual_secondary_gain_ready)
      return true;
    if (candidate_near_best && baseline_near_best &&
        std::abs(candidate.mean_visible_count -
                 baseline.mean_visible_count) <= eps &&
        std::abs(candidate.all3 - baseline.all3) <= eps &&
        candidate.none <=
            baseline.none - min_visibility_improvement_ + eps &&
        actual_secondary_gain_ready)
      return true;
    if (!baseline.valid && actual_secondary_gain_ready)
      return true;
    // Common phi0 rotation preserves the 120-degree geometry exactly.
    // Diversity remains a lexicographic tie-breaker, but numerical noise in
    // that invariant is never sufficient reason to rotate the formation.
    return false;
  }

  ViewpointStep encirclementStep(
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const std::array<Eigen::Vector3d, 3> &odom_positions,
      const std::array<bool, 3> &odom_valid,
      const double now, const double dt)
  {
    ViewpointStep result;
    result.encirclement_enabled = true;
    const double safe_dt = std::max(0.0, std::min(0.5, dt));
    if (observed_visible_count_valid_ && observed_visible_count_ < 3)
      observed_all3_deficit_duration_ += safe_dt;
    else
      observed_all3_deficit_duration_ = 0.0;
    if (!geometry_.valid() || !target_position.allFinite() ||
        !target_velocity.allFinite())
    {
      cancelTransitionToLastValid();
      result.state = ViewpointState::HOLD;
      result.commanded_angles = last_valid_angles_;
      result.target_angles = last_valid_angles_;
      result.slot_assignment = slot_for_uav_;
      result.initial_slot_assignment_changed =
          initial_slot_assignment_changed_;
      if (have_last_valid_reference_)
      {
        result.positions = last_valid_positions_;
        result.velocities = last_valid_velocities_;
        result.reference_valid = {{true, true, true}};
        result.using_last_valid_reference = true;
      }
      else
      {
        for (int uav = 0; uav < 3; ++uav)
        {
          result.positions[uav].setZero();
          result.velocities[uav].setZero();
        }
      }
      result.phi0_current = commanded_common_phi0_;
      result.phi0_selected = target_common_phi0_;
      result.phi0_delta = 0.0;
      result.selection_reason = have_last_valid_reference_
          ? "ENCIRCLEMENT_INPUT_INVALID_LAST_VALID_REFERENCE_HOLD"
          : "ENCIRCLEMENT_INPUT_OR_GEOMETRY_INVALID";
      return result;
    }

    if (!initialized_)
    {
      initialized_ = true;
      initializeEncirclement(target_position, target_velocity,
                             odom_positions, odom_valid);
    }
    result.slot_assignment = slot_for_uav_;
    result.initial_slot_assignment_changed =
        initial_slot_assignment_changed_;

    // Snapshot measured positions for the execution-aware visibility
    // predictor.  Candidate validity remains based on the prescribed soft
    // reference arc; only range/LOS/FOV scoring uses this measured residual.
    observer_positions_ = odom_positions;
    observer_position_valid_ = odom_valid;

    const auto current_references = encirclementReferencePositions(
        target_position, commanded_common_phi0_);
    for (int uav = 0; uav < 3; ++uav)
    {
      if (!odom_valid[uav] || !odom_positions[uav].allFinite())
      {
        blocked_duration_[uav] = 0.0;
        continue;
      }
      double clearance = std::numeric_limits<double>::infinity();
      if (!geometry_.querySegmentClearance(odom_positions[uav],
                                           target_position, clearance))
      {
        cancelTransitionToLastValid();
        result.state = ViewpointState::HOLD;
        result.commanded_angles = last_valid_angles_;
        result.target_angles = last_valid_angles_;
        result.positions = have_last_valid_reference_
            ? last_valid_positions_ : current_references;
        result.velocities = have_last_valid_reference_
            ? last_valid_velocities_
            : std::array<Eigen::Vector3d, 3>{{
                  target_velocity, target_velocity, target_velocity}};
        result.reference_valid = have_last_valid_reference_
            ? std::array<bool, 3>{{true, true, true}}
            : std::array<bool, 3>{{false, false, false}};
        result.using_last_valid_reference = have_last_valid_reference_;
        result.phi0_current = commanded_common_phi0_;
        result.phi0_selected = target_common_phi0_;
        result.phi0_delta = 0.0;
        result.selection_reason = have_last_valid_reference_
            ? "ENCIRCLEMENT_GEOMETRY_QUERY_LAST_VALID_REFERENCE_HOLD"
            : "ENCIRCLEMENT_GEOMETRY_QUERY_FAILED";
        return result;
      }
      result.actual_los_clearance[uav] = clearance;
      result.tracking_error[uav] =
          (odom_positions[uav] - current_references[uav]).norm();
      if (clearance < los_margin_)
        blocked_duration_[uav] += safe_dt;
      else
        blocked_duration_[uav] = 0.0;
    }

    const ViewpointMetrics current_metrics = evaluateEncirclementTransition(
        target_position, target_velocity, commanded_common_phi0_, 0.0);
    result.current_visibility = current_metrics.atleast2;
    result.current_all3 = current_metrics.all3;
    result.current_none = current_metrics.none;
    result.current_mean_visible = current_metrics.mean_visible_count;
    result.dynamic_los_evaluated = true;
    result.camera_fov_evaluated = true;
    result.yaw_dynamics_evaluated = true;
    bool canceled_this_cycle = false;
    if (state_ == ViewpointState::HOLD)
    {
      active_uav_ = -1;
      bool persistent_loss = false;
      bool planner_unsettled = false;
      for (int uav = 0; uav < 3; ++uav)
      {
        if (!odom_valid[uav] || !odom_positions[uav].allFinite())
        {
          planner_unsettled = true;
          continue;
        }
        if (result.tracking_error[uav] > trigger_tracking_error_)
          planner_unsettled = true;
        if (blocked_duration_[uav] + 1.0e-9 >= visibility_bad_hold_ &&
            result.tracking_error[uav] <= trigger_tracking_error_)
          persistent_loss = true;
      }
      result.persistent_static_los_loss = persistent_loss;
      result.periodic_visibility_evaluation = true;

      ViewpointMetrics best;
      double best_phi0 = commanded_common_phi0_;
      double best_rotation = 0.0;
      bool have_best = false;
      const std::array<double, 3> candidate_phi0s{{
          commanded_common_phi0_,
          wrapToPi(commanded_common_phi0_ - M_PI / 12.0),
          wrapToPi(commanded_common_phi0_ + M_PI / 12.0)}};

      for (const double candidate_phi0 : candidate_phi0s)
      {
        EncirclementCandidateDiagnostic diagnostic;
        diagnostic.phi0 = candidate_phi0;
        const bool transition_valid = selectFeasibleRotation(
            target_position, target_velocity, commanded_common_phi0_,
            candidate_phi0, diagnostic.signed_rotation);
        diagnostic.metrics = evaluateEncirclementTransition(
            target_position, target_velocity, commanded_common_phi0_,
            diagnostic.signed_rotation);
        diagnostic.arc_valid = transition_valid && diagnostic.metrics.valid;
        result.encirclement_candidates.push_back(diagnostic);
        if (!diagnostic.arc_valid)
          continue;
        ++result.valid_candidate_count;
        if (!have_best || betterEncirclementCandidate(
                diagnostic.metrics, candidate_phi0, best, best_phi0,
                commanded_common_phi0_, k2_near_best_tolerance_))
        {
          best = diagnostic.metrics;
          best_phi0 = candidate_phi0;
          best_rotation = diagnostic.signed_rotation;
          have_best = true;
        }
      }
      result.selected_candidate_phi = best_phi0;
      result.phi0_selected = commanded_common_phi0_;
      result.phi0_delta = 0.0;
      result.candidate_visibility = best.atleast2;
      result.candidate_all3 = best.all3;
      result.candidate_none = best.none;
      result.candidate_mean_visible = best.mean_visible_count;
      result.candidate_diversity = spreadPreferenceScore(
          best.min_pairwise_angle * 180.0 / M_PI, 60.0);
      // Reference-level visibility remains diagnostic/precheck only.  It
      // must never choose phi0; only a validated Joint result may request a
      // transition.
      if (joint_selection_pending_ &&
          angleDistance(joint_selected_phi0_, commanded_common_phi0_) > 1.0e-6)
      {
        double joint_rotation = 0.0;
        if (selectFeasibleRotation(target_position, target_velocity,
                                   commanded_common_phi0_,
                                   joint_selected_phi0_, joint_rotation))
        {
          state_ = ViewpointState::TRANSITION;
          target_common_phi0_ = joint_selected_phi0_;
          target_angles_ = encirclementAngles(target_common_phi0_);
          transition_start_common_phi0_ = commanded_common_phi0_;
          transition_start_time_ = now;
          arrival_hold_accumulator_ = 0.0;
          transition_remaining_angle_ = joint_rotation;
          transition_direction_ = joint_rotation >= 0.0 ? 1 : -1;
          result.phi0_selected = joint_selected_phi0_;
          result.phi0_delta = joint_rotation;
          result.selection_reason = "ENCIRCLEMENT_JOINT_TRAJECTORY_SELECTION";
        }
        else
          result.selection_reason = "ENCIRCLEMENT_JOINT_ARC_INVALID";
        joint_selection_pending_ = false;
      }
      else if (joint_selection_pending_)
      {
        joint_selection_pending_ = false;
        result.selection_reason = "ENCIRCLEMENT_JOINT_HOLD_CURRENT";
      }
      else if (!have_best)
        result.selection_reason = "ENCIRCLEMENT_NO_VALID_CANDIDATE";
      else
        result.selection_reason =
            planner_unsettled ? "ENCIRCLEMENT_HYPOTHESES_PLANNER_UNSETTLED" :
                                "ENCIRCLEMENT_AWAITING_JOINT_SELECTION";
    }
    else
    {
      result.selection_reason = "ENCIRCLEMENT_TRANSITION_TARGET_FROZEN";
      result.reselection_blocked = true;
    }

    if (state_ == ViewpointState::TRANSITION)
    {
      result.reselection_blocked = true;
      if (std::abs(transition_remaining_angle_) > 1.0e-9 &&
          ((transition_remaining_angle_ > 0.0 ? 1 : -1) !=
           transition_direction_))
        result.direction_reversal = true;
      const double increment = transition_direction_ * std::min(
          max_angular_rate_ * safe_dt,
          std::abs(transition_remaining_angle_));
      const double next_phi0 = wrapToPi(
          commanded_common_phi0_ + increment);
      const Eigen::Vector3d next_target =
          target_position + target_velocity * safe_dt;
      if (!encirclementReferencesInstantlyValid(next_target, next_phi0))
      {
        cancelTransitionToLastValid();
        canceled_this_cycle = true;
        last_transition_end_time_ = now;
        result.selection_reason =
            "ENCIRCLEMENT_TRANSITION_ARC_REVALIDATION_REJECTED";
        result.angular_rates = {{0.0, 0.0, 0.0}};
      }
      else
      {
        commanded_common_phi0_ = next_phi0;
        transition_remaining_angle_ -= increment;
        if (std::abs(transition_remaining_angle_) <= 1.0e-9)
          transition_remaining_angle_ = 0.0;
        commanded_angles_ = encirclementAngles(commanded_common_phi0_);
        for (int uav = 0; uav < 3; ++uav)
          result.angular_rates[uav] = safe_dt > 1.0e-9
              ? increment / safe_dt : 0.0;
      }

      if (state_ == ViewpointState::TRANSITION &&
          !encirclementReferencesInstantlyValid(
              target_position, commanded_common_phi0_))
      {
        cancelTransitionToLastValid();
        canceled_this_cycle = true;
        last_transition_end_time_ = now;
        result.selection_reason =
            "ENCIRCLEMENT_REFERENCE_INVALID_FALLBACK";
        result.angular_rates = {{0.0, 0.0, 0.0}};
      }
      else if (state_ == ViewpointState::TRANSITION)
      {
        last_valid_common_phi0_ = commanded_common_phi0_;
        last_valid_angles_ = commanded_angles_;
        result.transition_elapsed = std::max(
            0.0, now - transition_start_time_);
        const auto references = encirclementReferencePositions(
            target_position, commanded_common_phi0_);
        double max_reference_error = 0.0;
        for (int uav = 0; uav < 3; ++uav)
        {
          if (!odom_valid[uav])
            continue;
          const double error_norm =
              (odom_positions[uav] - references[uav]).norm();
          max_reference_error = std::max(max_reference_error, error_norm);
        }
        result.reference_error = max_reference_error;
        const bool angle_arrived =
            std::abs(transition_remaining_angle_) <= 1.0e-3;
        // This state machine commands a soft reference, not a hard formation
        // controller.  Its rotation is complete when the rate-limited common
        // phi0 reaches the selected value.  Actual odometry convergence is
        // retained above as reference_error and gates the next selection in
        // HOLD; it must not keep an already-finished reference transition
        // open until later scene geometry invalidates it.
        if (angle_arrived)
        {
          result.arrived = true;
          blocked_duration_ = {{0.0, 0.0, 0.0}};
          state_ = ViewpointState::HOLD;
          last_transition_end_time_ = now;
          arrival_hold_accumulator_ = 0.0;
          result.selection_reason = "ENCIRCLEMENT_TRANSITION_ARRIVED";
        }
      }
    }

    commanded_angles_ = encirclementAngles(commanded_common_phi0_);
    target_angles_ = encirclementAngles(target_common_phi0_);
    result.state = state_;
    result.active_uav = -1;
    result.commanded_angles = commanded_angles_;
    result.target_angles = target_angles_;
    result.blocked_duration = blocked_duration_;
    result.positions = encirclementReferencePositions(
        target_position, commanded_common_phi0_);
    for (int uav = 0; uav < 3; ++uav)
    {
      const Eigen::Vector3d radial = result.positions[uav] - target_position;
      result.velocities[uav] = target_velocity +
          result.angular_rates[uav] *
              Eigen::Vector3d::UnitZ().cross(radial);
      result.reference_valid[uav] = !canceled_this_cycle;
    }
    result.metrics = evaluateEncirclementTransition(
        target_position, target_velocity, commanded_common_phi0_, 0.0);
    const bool instant_reference_valid = encirclementReferencesInstantlyValid(
        target_position, commanded_common_phi0_);
    if (instant_reference_valid)
    {
      // A rejected transition restores the last valid *angular topology*.
      // Re-anchor that topology to the current moving target instead of
      // replaying an old absolute position.  The latter can turn a harmless
      // one-cycle arc rejection into a multi-metre planner-goal jump.
      last_valid_common_phi0_ = commanded_common_phi0_;
      last_valid_angles_ = commanded_angles_;
      last_valid_positions_ = result.positions;
      last_valid_velocities_ = result.velocities;
      have_last_valid_reference_ = true;
      result.reference_valid = {{true, true, true}};
      result.using_last_valid_reference = canceled_this_cycle;
    }
    else if (have_last_valid_reference_)
    {
      cancelTransitionToLastValid();
      result.state = state_;
      result.commanded_angles = last_valid_angles_;
      result.target_angles = last_valid_angles_;
      result.positions = last_valid_positions_;
      result.velocities = last_valid_velocities_;
      result.angular_rates = {{0.0, 0.0, 0.0}};
      result.reference_valid = {{true, true, true}};
      result.using_last_valid_reference = true;
      if (!canceled_this_cycle)
        result.selection_reason =
            "ENCIRCLEMENT_LAST_VALID_REFERENCE_HOLD";
    }
    else
      result.reference_valid = {{false, false, false}};
    result.phi0_current = commanded_common_phi0_;
    if (!std::isfinite(result.phi0_selected))
      result.phi0_selected = target_common_phi0_;
    if (state_ == ViewpointState::TRANSITION)
      result.phi0_delta = transition_remaining_angle_;
    else if (!std::isfinite(result.phi0_delta))
      result.phi0_delta = wrapToPi(
          result.phi0_selected - result.phi0_current);
    if (!std::isfinite(result.selected_candidate_phi))
      result.selected_candidate_phi = result.phi0_selected;
    if (!std::isfinite(result.candidate_visibility))
    {
      const ViewpointMetrics target_metrics = evaluateEncirclementTransition(
          target_position, target_velocity, commanded_common_phi0_,
          state_ == ViewpointState::TRANSITION
              ? transition_remaining_angle_
              : wrapToPi(target_common_phi0_ - commanded_common_phi0_));
      result.candidate_visibility = target_metrics.atleast2;
      result.candidate_all3 = target_metrics.all3;
      result.candidate_none = target_metrics.none;
      result.candidate_mean_visible = target_metrics.mean_visible_count;
      result.candidate_diversity = spreadPreferenceScore(
          target_metrics.min_pairwise_angle * 180.0 / M_PI, 60.0);
    }
    return result;
  }

  ViewpointStep step(const Eigen::Vector3d &target_position,
                     const Eigen::Vector3d &target_velocity,
                     const std::array<Eigen::Vector3d, 3> &odom_positions,
                     const std::array<bool, 3> &odom_valid,
                     const double now, const double dt)
  {
    if (encirclement_enabled_)
      return encirclementStep(target_position, target_velocity,
                              odom_positions, odom_valid, now, dt);
    ViewpointStep result;
    const double safe_dt = std::max(0.0, std::min(0.5, dt));
    if (!initialized_)
    {
      initialized_ = true;
      commanded_angles_ = {{0.0, 0.0, 0.0}};
      target_angles_ = commanded_angles_;
      last_valid_angles_ = commanded_angles_;
    }

    if (!geometry_.valid() || !target_position.allFinite() ||
        !target_velocity.allFinite())
    {
      cancelTransitionToLastValid();
      result = fixedFormationStep(target_position, target_velocity,
                                  "INPUT_OR_GEOMETRY_INVALID_FALLBACK");
      result.reference_valid = {{false, false, false}};
      return result;
    }

    const auto current_references =
        referencePositions(target_position, commanded_angles_);
    for (int uav = 0; uav < 3; ++uav)
    {
      if (!odom_valid[uav] || !odom_positions[uav].allFinite())
      {
        blocked_duration_[uav] = 0.0;
        continue;
      }
      double clearance = std::numeric_limits<double>::infinity();
      if (!geometry_.querySegmentClearance(odom_positions[uav], target_position,
                                           clearance))
      {
        cancelTransitionToLastValid();
        result = fixedFormationStep(target_position, target_velocity,
                                    "STATIC_GEOMETRY_QUERY_FAILED_FALLBACK");
        result.reference_valid = {{false, false, false}};
        return result;
      }
      result.actual_los_clearance[uav] = clearance;
      result.tracking_error[uav] =
          (odom_positions[uav] - current_references[uav]).norm();
      if (clearance < los_margin_)
        blocked_duration_[uav] += safe_dt;
      else
        blocked_duration_[uav] = 0.0;
    }

    std::array<bool, 3> reference_odom_valid{{true, true, true}};
    ViewpointMetrics current_metrics = evaluate(
        target_position, target_velocity, commanded_angles_, -1,
        odom_positions, reference_odom_valid);

    bool canceled_this_cycle = false;
    if (state_ == ViewpointState::HOLD)
    {
      active_uav_ = -1;
      int selected_uav = -1;
      bool planner_unsettled = false;
      for (int uav = 0; uav < 3; ++uav)
      {
        if (!odom_valid[uav] ||
            blocked_duration_[uav] + 1.0e-9 < visibility_bad_hold_)
          continue;
        if (result.tracking_error[uav] > trigger_tracking_error_)
        {
          planner_unsettled = true;
          continue;
        }
        if (selected_uav < 0 ||
            blocked_duration_[uav] > blocked_duration_[selected_uav] + 1.0e-9 ||
            (std::abs(blocked_duration_[uav] -
                      blocked_duration_[selected_uav]) <= 1.0e-9 &&
             current_metrics.visibility[uav] <
                 current_metrics.visibility[selected_uav] - 1.0e-9))
          selected_uav = uav;
      }

      if (selected_uav >= 0)
      {
        const ViewpointMetrics baseline = evaluate(
            target_position, target_velocity, commanded_angles_, selected_uav,
            odom_positions, odom_valid);
        ViewpointMetrics best;
        double best_angle = commanded_angles_[selected_uav];
        for (int candidate = 0; candidate < 8; ++candidate)
        {
          const double angle = candidate * M_PI / 4.0;
          std::array<double, 3> candidate_angles = commanded_angles_;
          candidate_angles[selected_uav] = angle;
          const ViewpointMetrics metrics = evaluate(
              target_position, target_velocity, candidate_angles, selected_uav,
              odom_positions, odom_valid);
          if (metrics.valid &&
              singleUavTransitionValid(target_position, target_velocity,
                                       commanded_angles_, selected_uav, angle))
          {
            ++result.valid_candidate_count;
            if (betterForUav(metrics, best, selected_uav))
            {
              best = metrics;
              best_angle = wrapToPi(angle);
            }
          }
        }
        result.evaluated_uav = selected_uav;
        result.current_visibility = baseline.visibility[selected_uav];
        result.candidate_visibility = best.visibility[selected_uav];
        result.selected_candidate_phi = best_angle;
        const double improvement = best.valid && baseline.valid ?
            best.visibility[selected_uav] - baseline.visibility[selected_uav] :
            -std::numeric_limits<double>::infinity();
        if (best.valid && baseline.valid &&
            improvement + 1.0e-9 >= min_visibility_improvement_ &&
            angleDistance(best_angle, commanded_angles_[selected_uav]) > 1.0e-6)
        {
          state_ = ViewpointState::TRANSITION;
          active_uav_ = selected_uav;
          target_angles_[selected_uav] = best_angle;
          transition_start_angle_ = commanded_angles_[selected_uav];
          transition_start_time_ = now;
          arrival_hold_accumulator_ = 0.0;
          const double delta = wrapToPi(best_angle - transition_start_angle_);
          transition_direction_ = delta >= 0.0 ? 1 : -1;
          result.selection_reason = "PERSISTENT_STATIC_LOS_IMPROVEMENT";
        }
        else if (!best.valid)
        {
          result.selection_reason = "NO_VALID_CANDIDATE";
        }
        else
        {
          result.selection_reason = "IMPROVEMENT_BELOW_THRESHOLD";
        }
      }
      else if (planner_unsettled)
      {
        result.selection_reason = "PLANNER_NOT_SETTLED";
      }
      else
      {
        result.selection_reason = "HOLD_NO_PERSISTENT_STATIC_LOS_LOSS";
      }
    }
    else
    {
      result.selection_reason = "TRANSITION_TARGET_FROZEN";
      result.reselection_blocked = true;
    }

    if (state_ == ViewpointState::TRANSITION)
    {
      result.active_uav = active_uav_;
      result.reselection_blocked = true;
      const int active = active_uav_;
      const double error = wrapToPi(target_angles_[active] -
                                    commanded_angles_[active]);
      if (std::abs(error) > 1.0e-9 &&
          ((error > 0.0 ? 1 : -1) != transition_direction_))
      {
        result.direction_reversal = true;
      }
      const double increment = transition_direction_ *
          std::min(max_angular_rate_ * safe_dt, std::abs(error));
      commanded_angles_[active] = wrapToPi(
          commanded_angles_[active] + increment);
      result.angular_rates[active] = safe_dt > 1.0e-9 ?
          increment / safe_dt : 0.0;

      std::array<bool, 3> current_valid{{true, true, true}};
      const ViewpointMetrics transition_metrics = evaluate(
          target_position, target_velocity, commanded_angles_, active,
          odom_positions, odom_valid);
      if (!transition_metrics.valid)
      {
        cancelTransitionToLastValid();
        canceled_this_cycle = true;
        result.selection_reason = "REFERENCE_INVALID_FALLBACK";
        result.angular_rates = {{0.0, 0.0, 0.0}};
      }
      else
      {
        last_valid_angles_[active] = commanded_angles_[active];
        result.transition_elapsed = std::max(0.0, now - transition_start_time_);
        const auto references =
            referencePositions(target_position, commanded_angles_);
        if (odom_valid[active])
          result.reference_error =
              (odom_positions[active] - references[active]).norm();
        const bool angle_arrived = angleDistance(
            commanded_angles_[active], target_angles_[active]) <= 1.0e-3;
        const bool position_arrived = odom_valid[active] &&
            result.reference_error <= arrive_distance_;
        if (angle_arrived && position_arrived)
          arrival_hold_accumulator_ += safe_dt;
        else
          arrival_hold_accumulator_ = 0.0;
        if (arrival_hold_accumulator_ + 1.0e-9 >= arrive_hold_)
        {
          result.arrived = true;
          blocked_duration_[active] = 0.0;
          state_ = ViewpointState::HOLD;
          active_uav_ = -1;
          arrival_hold_accumulator_ = 0.0;
          result.selection_reason = "TRANSITION_ARRIVED";
        }
      }
    }

    result.state = state_;
    result.active_uav = active_uav_;
    result.commanded_angles = commanded_angles_;
    result.target_angles = target_angles_;
    result.blocked_duration = blocked_duration_;
    result.positions = referencePositions(target_position, commanded_angles_);
    for (int uav = 0; uav < 3; ++uav)
    {
      result.velocities[uav] = target_velocity +
          result.angular_rates[uav] * Eigen::Vector3d::UnitZ().cross(
              rotatedOffset(uav, commanded_angles_[uav]));
      result.reference_valid[uav] = !canceled_this_cycle;
    }
    result.metrics = evaluate(target_position, target_velocity,
                              commanded_angles_, active_uav_,
                              odom_positions, odom_valid);
    if (!result.metrics.valid)
      result.reference_valid = {{false, false, false}};
    return result;
  }

  ViewpointState state() const { return state_; }
  int activeTransitionUav() const { return active_uav_; }
  const std::array<double, 3> &commandedAngles() const
  {
    return commanded_angles_;
  }
  const std::array<double, 3> &targetAngles() const { return target_angles_; }
  const std::array<int, 3> &slotAssignment() const
  {
    return slot_for_uav_;
  }
  bool initialSlotAssignmentChanged() const
  {
    return initial_slot_assignment_changed_;
  }
  const std::array<double, 3> &blockedDurations() const
  {
    return blocked_duration_;
  }

private:
  void cancelTransitionToLastValid()
  {
    commanded_angles_ = last_valid_angles_;
    target_angles_ = commanded_angles_;
    commanded_common_phi0_ = last_valid_common_phi0_;
    target_common_phi0_ = commanded_common_phi0_;
    state_ = ViewpointState::HOLD;
    active_uav_ = -1;
    transition_direction_ = 0;
    transition_remaining_angle_ = 0.0;
    arrival_hold_accumulator_ = 0.0;
  }

  ego_planner::StaticLosGeometry geometry_;
  std::vector<DynamicLosCylinder> dynamic_cylinders_;
  TrackingCameraContract camera_;
  std::array<PredictedYawState, 3> observer_yaw_states_;
  std::array<Eigen::Vector3d, 3> observer_positions_{{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()}};
  std::array<bool, 3> observer_position_valid_{{false, false, false}};
  bool observed_visible_count_valid_{false};
  int observed_visible_count_{0};
  double observed_all3_deficit_duration_{0.0};
  double dynamic_motion_elapsed_{0.0};
  std::array<Eigen::Vector3d, 3> offsets_;
  double horizon_{2.0};
  double sample_dt_{0.1};
  double min_range_{0.2};
  double max_range_{8.0};
  double los_margin_{0.08};
  double reference_clearance_{0.0};
  double visibility_bad_hold_{0.5};
  double trigger_tracking_error_{0.5};
  double theta_sep_min_{25.0 * M_PI / 180.0};
  double min_visibility_improvement_{0.10};
  double k2_near_best_tolerance_{0.01};
  double max_angular_rate_{0.25};
  double arrive_distance_{0.5};
  double arrive_hold_{0.5};
  double transition_reselection_cooldown_{1.0};
  bool encirclement_enabled_{false};
  bool initialized_{false};
  ViewpointState state_{ViewpointState::HOLD};
  int active_uav_{-1};
  std::array<double, 3> commanded_angles_{{0.0, 0.0, 0.0}};
  std::array<double, 3> target_angles_{{0.0, 0.0, 0.0}};
  std::array<double, 3> last_valid_angles_{{0.0, 0.0, 0.0}};
  std::array<double, 3> blocked_duration_{{0.0, 0.0, 0.0}};
  double transition_start_angle_{0.0};
  double transition_start_time_{0.0};
  double arrival_hold_accumulator_{0.0};
  int transition_direction_{0};
  double transition_remaining_angle_{0.0};
  double last_transition_end_time_{
      std::numeric_limits<double>::quiet_NaN()};
  double commanded_common_phi0_{0.0};
  double target_common_phi0_{0.0};
  double last_valid_common_phi0_{0.0};
  double transition_start_common_phi0_{0.0};
  std::array<int, 3> slot_for_uav_{{0, 1, 2}};
  bool initial_slot_assignment_changed_{false};
  bool have_last_valid_reference_{false};
  bool joint_selection_pending_{false};
  double joint_selected_phi0_{0.0};
  std::array<Eigen::Vector3d, 3> last_valid_positions_{{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()}};
  std::array<Eigen::Vector3d, 3> last_valid_velocities_{{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()}};
};

} // namespace multi_uav_formation

#endif
