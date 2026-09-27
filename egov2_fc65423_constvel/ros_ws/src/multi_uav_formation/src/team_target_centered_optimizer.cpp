#include <multi_uav_formation/team_target_centered_optimizer.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace multi_uav_formation
{
namespace
{

double quantile(std::vector<double> values, const double q)
{
  if (values.empty())
    return 0.0;
  std::sort(values.begin(), values.end());
  const double index = std::max(0.0, std::min(1.0, q)) *
                       static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(index));
  const size_t upper = static_cast<size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lower);
  return (1.0 - alpha) * values[lower] + alpha * values[upper];
}

struct InterpolatedReference
{
  bool valid{false};
  size_t low{0};
  size_t high{0};
  double low_weight{1.0};
  double high_weight{0.0};
  double radius{0.0};
  double bearing{0.0};
  double height{0.0};
  double radius_rate{0.0};
  double bearing_rate{0.0};
};

InterpolatedReference interpolate(const TeamReferenceMember &member,
                                  const double world_time)
{
  InterpolatedReference result;
  if (member.knots.empty() || !std::isfinite(world_time) ||
      !std::isfinite(member.reference_phase_shift))
    return result;
  const double phase_time = world_time - member.reference_phase_shift;
  if (phase_time <= member.knots.front().world_time)
  {
    result.low = result.high = 0;
    result.radius = member.knots.front().radius;
    result.bearing = member.knots.front().bearing;
    result.height = member.knots.front().height;
    result.valid = true;
    return result;
  }
  if (phase_time >= member.knots.back().world_time)
  {
    result.low = result.high = member.knots.size() - 1;
    result.radius = member.knots.back().radius;
    result.bearing = member.knots.back().bearing;
    result.height = member.knots.back().height;
    result.valid = true;
    return result;
  }
  const auto upper = std::upper_bound(
      member.knots.begin(), member.knots.end(), phase_time,
      [](const double value, const TeamReferenceKnot &knot) {
        return value < knot.world_time;
      });
  result.high = static_cast<size_t>(upper - member.knots.begin());
  result.low = result.high - 1;
  const auto &a = member.knots[result.low];
  const auto &b = member.knots[result.high];
  const double span = b.world_time - a.world_time;
  if (!std::isfinite(span) || span <= 1.0e-9)
    return InterpolatedReference();
  result.high_weight = (phase_time - a.world_time) / span;
  result.low_weight = 1.0 - result.high_weight;
  const double bearing_delta = wrapTeamAngle(b.bearing - a.bearing);
  result.radius = result.low_weight * a.radius + result.high_weight * b.radius;
  result.bearing = a.bearing + result.high_weight * bearing_delta;
  result.height = result.low_weight * a.height + result.high_weight * b.height;
  result.radius_rate = (b.radius - a.radius) / span;
  result.bearing_rate = bearing_delta / span;
  result.valid = std::isfinite(result.radius) &&
                 std::isfinite(result.bearing) &&
                 std::isfinite(result.height);
  return result;
}

double deadbandResidual(const double value, const double deadband)
{
  if (value > deadband)
    return value - deadband;
  if (value < -deadband)
    return value + deadband;
  return 0.0;
}

struct SmoothGuard
{
  double penalty{0.0};
  double derivative_deficit{0.0};
};

// C1 positive-part approximation with exactly zero value/gradient inside the
// admissible region.  The returned penalty is normalized by the numerical
// quality tolerance, so it guides the line search without defining the final
// hierarchy; the post-stage hard filter remains authoritative.
SmoothGuard qualityGuard(const double deficit, const double tolerance,
                         const double barrier)
{
  SmoothGuard guard;
  if (deficit <= 0.0)
    return guard;
  const double width = std::max(1.0e-9, tolerance);
  double positive = 0.0;
  double derivative = 0.0;
  if (deficit < width)
  {
    positive = 0.5 * deficit * deficit / width;
    derivative = deficit / width;
  }
  else
  {
    positive = deficit - 0.5 * width;
    derivative = 1.0;
  }
  const double normalized = positive / width;
  guard.penalty = barrier * normalized * normalized;
  guard.derivative_deficit =
      2.0 * barrier * normalized * derivative / width;
  return guard;
}

double stageObjective(const TeamTargetCenteredMetrics &metrics,
                      const TeamOptimizationStage stage,
                      const TeamTargetCenteredParams &params,
                      const TeamQualityTolerance &tolerance,
                      const double q_floor,
                      const double visibility_floor,
                      const double complementarity_floor)
{
  if (!metrics.valid || !metrics.quality.allFinite())
    return std::numeric_limits<double>::infinity();
  const auto q_guard = qualityGuard(
      q_floor - metrics.quality.q2_ratio, tolerance.q2,
      params.quality_guard_barrier);
  const auto visibility_guard = qualityGuard(
      visibility_floor - metrics.quality.accumulated_visibility_ratio,
      tolerance.accumulated_visibility, params.quality_guard_barrier);
  const auto comp_guard = qualityGuard(
      complementarity_floor - metrics.quality.complementarity,
      tolerance.complementarity, params.quality_guard_barrier);
  constexpr double kNumericalRegularization = 1.0e-8;
  switch (stage)
  {
  case TeamOptimizationStage::VISIBILITY_Q2:
    return 1.0 - metrics.quality.q2_ratio +
           kNumericalRegularization * metrics.quality.regularization_cost;
  case TeamOptimizationStage::VISIBILITY_ACC:
    return 1.0 - metrics.quality.accumulated_visibility_ratio +
           q_guard.penalty +
           kNumericalRegularization * metrics.quality.regularization_cost;
  case TeamOptimizationStage::COMPLEMENTARITY:
    return 1.0 - metrics.quality.complementarity + q_guard.penalty +
           visibility_guard.penalty +
           kNumericalRegularization * metrics.quality.regularization_cost;
  case TeamOptimizationStage::ENCIRCLEMENT_REGULARIZATION:
    return metrics.quality.encirclement_cost +
           params.stage3_regularization_lambda *
               metrics.quality.regularization_cost +
           q_guard.penalty + visibility_guard.penalty + comp_guard.penalty;
  default:
    return std::numeric_limits<double>::infinity();
  }
}

TeamObjectiveWeights stageGradientWeights(
    const TeamTargetCenteredMetrics &metrics,
    const TeamOptimizationStage stage,
    const TeamTargetCenteredParams &params,
    const TeamQualityTolerance &tolerance,
    const double horizon,
    const double q_floor,
    const double visibility_floor,
    const double complementarity_floor)
{
  TeamObjectiveWeights weights;
  weights.k2 = 0.0;
  weights.accumulated = 0.0;
  weights.complementarity = 0.0;
  weights.encirclement = 0.0;
  weights.regularization = 0.0;
  const double safe_horizon = std::max(1.0e-9, horizon);
  const auto q_guard = qualityGuard(
      q_floor - metrics.quality.q2_ratio, tolerance.q2,
      params.quality_guard_barrier);
  const auto visibility_guard = qualityGuard(
      visibility_floor - metrics.quality.accumulated_visibility_ratio,
      tolerance.accumulated_visibility, params.quality_guard_barrier);
  const auto comp_guard = qualityGuard(
      complementarity_floor - metrics.quality.complementarity,
      tolerance.complementarity, params.quality_guard_barrier);
  constexpr double kNumericalRegularization = 1.0e-8;
  switch (stage)
  {
  case TeamOptimizationStage::VISIBILITY_Q2:
    weights.k2 = 1.0 / safe_horizon;
    weights.regularization = kNumericalRegularization;
    break;
  case TeamOptimizationStage::VISIBILITY_ACC:
    weights.accumulated = 1.0 / (3.0 * safe_horizon);
    weights.k2 = q_guard.derivative_deficit / safe_horizon;
    weights.regularization = kNumericalRegularization;
    break;
  case TeamOptimizationStage::COMPLEMENTARITY:
    weights.complementarity = 1.0 / (3.0 * safe_horizon);
    weights.k2 = q_guard.derivative_deficit / safe_horizon;
    weights.accumulated = visibility_guard.derivative_deficit /
                          (3.0 * safe_horizon);
    weights.regularization = kNumericalRegularization;
    break;
  case TeamOptimizationStage::ENCIRCLEMENT_REGULARIZATION:
    weights.encirclement = 1.0;
    weights.regularization = params.stage3_regularization_lambda;
    weights.k2 = q_guard.derivative_deficit / safe_horizon;
    weights.accumulated = visibility_guard.derivative_deficit /
                          (3.0 * safe_horizon);
    weights.complementarity = comp_guard.derivative_deficit /
                              (3.0 * safe_horizon);
    break;
  default:
    break;
  }
  return weights;
}

struct StageRun
{
  bool valid{false};
  bool timed_out{false};
  bool cancelled{false};
  int iterations{0};
  double latency_ms{0.0};
  TeamTargetCenteredInput schedule;
  TeamTargetCenteredMetrics metrics;
};

}  // namespace

const char *teamOptimizationStageName(const TeamOptimizationStage stage)
{
  switch (stage)
  {
  case TeamOptimizationStage::LOCAL_BASELINE:
    return "LOCAL_BASELINE";
  case TeamOptimizationStage::VISIBILITY_Q2:
    return "VISIBILITY_Q2";
  case TeamOptimizationStage::VISIBILITY_ACC:
    return "VISIBILITY_ACC";
  case TeamOptimizationStage::COMPLEMENTARITY:
    return "COMPLEMENTARITY";
  case TeamOptimizationStage::ENCIRCLEMENT_REGULARIZATION:
    return "ENCIRCLEMENT_REGULARIZATION";
  }
  return "UNKNOWN";
}

TeamTargetCenteredOptimizer::TeamTargetCenteredOptimizer(
    const TeamTargetCenteredParams &params) : params_(params)
{
  params_.prediction_horizon = std::max(0.2, params_.prediction_horizon);
  params_.sample_dt = std::max(0.02, params_.sample_dt);
  params_.reference_knot_count = std::max(2, params_.reference_knot_count);
  params_.top_k = std::max(1, std::min(3, params_.top_k));
  params_.max_iterations = std::max(1, params_.max_iterations);
  params_.timeout_seconds = std::max(0.001, params_.timeout_seconds);
  params_.radius_trust = std::max(0.0, params_.radius_trust);
  params_.bearing_trust = std::max(0.0, params_.bearing_trust);
  params_.phase_trust = std::max(0.0, params_.phase_trust);
}

bool TeamTargetCenteredOptimizer::validScheduleIdentity(
    const TeamTargetCenteredInput &schedule)
{
  if (!std::isfinite(schedule.evaluation_start_world) ||
      !std::isfinite(schedule.prediction_horizon) ||
      schedule.prediction_horizon <= 1.0e-6 ||
      schedule.target_prediction_revision == 0)
    return false;
  for (int drone = 0; drone < 3; ++drone)
  {
    const auto &member = schedule.members[drone];
    if (member.drone_id != drone || member.source_candidate_id < 0 ||
        member.topology_kind < 0 || member.topology_kind > 2 ||
        member.knots.size() < 2 ||
        !std::isfinite(member.reference_phase_shift) ||
        !member.initial_yaw_valid || !std::isfinite(member.initial_yaw) ||
        !std::isfinite(member.initial_yaw_rate))
      return false;
    for (size_t knot = 0; knot < member.knots.size(); ++knot)
    {
      const auto &value = member.knots[knot];
      if (!std::isfinite(value.world_time) || !std::isfinite(value.radius) ||
          !std::isfinite(value.bearing) || !std::isfinite(value.height) ||
          value.radius <= 1.0e-6 ||
          (knot > 0 && value.world_time <=
                           member.knots[knot - 1].world_time + 1.0e-9))
        return false;
    }
  }
  return true;
}

Eigen::Vector3d TeamTargetCenteredOptimizer::referencePosition(
    const TeamReferenceMember &member,
    const Eigen::Vector3d &target_at_world_time,
    const double world_time)
{
  const InterpolatedReference value = interpolate(member, world_time);
  if (!value.valid || !target_at_world_time.allFinite())
    return Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
  return targetCenteredCartesian(target_at_world_time, value.radius,
                                 value.bearing, value.height);
}

int TeamTargetCenteredOptimizer::variableCount(
    const TeamTargetCenteredInput &input) const
{
  int count = 3;
  for (const auto &member : input.members)
    count += 2 * static_cast<int>(member.knots.size());
  return count;
}

Eigen::VectorXd TeamTargetCenteredOptimizer::encode(
    const TeamTargetCenteredInput &schedule,
    const TeamTargetCenteredInput &reference) const
{
  Eigen::VectorXd variables(variableCount(reference));
  int cursor = 0;
  for (int drone = 0; drone < 3; ++drone)
  {
    for (size_t knot = 0; knot < reference.members[drone].knots.size(); ++knot)
    {
      variables(cursor++) = schedule.members[drone].knots[knot].radius -
                            reference.members[drone].knots[knot].radius;
      variables(cursor++) = wrapTeamAngle(
          schedule.members[drone].knots[knot].bearing -
          reference.members[drone].knots[knot].bearing);
    }
  }
  for (int drone = 0; drone < 3; ++drone)
    variables(cursor++) = schedule.members[drone].reference_phase_shift;
  return variables;
}

TeamTargetCenteredInput TeamTargetCenteredOptimizer::decode(
    const Eigen::VectorXd &variables,
    const TeamTargetCenteredInput &reference) const
{
  TeamTargetCenteredInput schedule = reference;
  if (variables.size() != variableCount(reference))
    return TeamTargetCenteredInput();
  int cursor = 0;
  for (int drone = 0; drone < 3; ++drone)
  {
    for (size_t knot = 0; knot < reference.members[drone].knots.size(); ++knot)
    {
      schedule.members[drone].knots[knot].radius = std::max(
          0.05, reference.members[drone].knots[knot].radius +
                    std::max(-params_.radius_trust,
                             std::min(params_.radius_trust,
                                      variables(cursor++))));
      schedule.members[drone].knots[knot].bearing =
          reference.members[drone].knots[knot].bearing +
          std::max(-params_.bearing_trust,
                   std::min(params_.bearing_trust, variables(cursor++)));
    }
  }
  for (int drone = 0; drone < 3; ++drone)
    schedule.members[drone].reference_phase_shift = std::max(
        -params_.phase_trust,
        std::min(params_.phase_trust, variables(cursor++)));
  return schedule;
}

TeamTargetCenteredMetrics TeamTargetCenteredOptimizer::evaluate(
    const TeamTargetCenteredInput &schedule,
    const TeamTargetCenteredInput &reference,
    const TeamTargetCenteredEnvironment &environment,
    Eigen::VectorXd *gradient) const
{
  TeamTargetCenteredMetrics metrics;
  if (!validScheduleIdentity(schedule) || !validScheduleIdentity(reference) ||
      !environment.target_at_world_time || !environment.visibility)
    return metrics;
  const int dimension = variableCount(reference);
  if (gradient)
    gradient->setZero(dimension);
  std::array<int, 3> variable_offset{{0, 0, 0}};
  int cursor = 0;
  for (int drone = 0; drone < 3; ++drone)
  {
    variable_offset[drone] = cursor;
    cursor += 2 * static_cast<int>(reference.members[drone].knots.size());
  }
  const int time_offset = cursor;
  const int sample_count = std::max(
      2, static_cast<int>(std::ceil(schedule.prediction_horizon /
                                   params_.sample_dt)) + 1);
  const double dt = schedule.prediction_horizon /
                    static_cast<double>(sample_count - 1);
  std::vector<double> pair_angles;
  std::vector<double> radius_errors;
  std::vector<double> bearing_errors;
  std::vector<double> height_errors;
  double q2_integral = 0.0;
  double visibility_integral = 0.0;
  double complementarity_integral = 0.0;
  std::array<PredictedYawState, 3> yaw_states;
  for (int drone = 0; drone < 3; ++drone)
  {
    yaw_states[drone].valid = schedule.members[drone].initial_yaw_valid;
    yaw_states[drone].yaw = schedule.members[drone].initial_yaw;
    yaw_states[drone].yaw_rate = schedule.members[drone].initial_yaw_rate;
  }

  for (int sample = 0; sample < sample_count; ++sample)
  {
    if (environment.cancelled && environment.cancelled())
      return TeamTargetCenteredMetrics();
    const double world_time = schedule.evaluation_start_world + sample * dt;
    const double sample_weight = (sample == 0 || sample + 1 == sample_count)
        ? 0.5 * dt : dt;
    const Eigen::Vector3d target =
        environment.target_at_world_time(world_time);
    if (!target.allFinite())
      return TeamTargetCenteredMetrics();
    std::array<InterpolatedReference, 3> refs;
    std::array<Eigen::Vector3d, 3> positions;
    std::array<Eigen::Vector3d, 3> relative;
    std::array<ContinuousVisibilitySample, 3> visibility;
    std::array<double, 3> v{{0.0, 0.0, 0.0}};
    std::array<Eigen::Vector3d, 3> position_gradient{
        {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()}};
    std::array<double, 3> radius_gradient{{0.0, 0.0, 0.0}};
    std::array<double, 3> bearing_gradient{{0.0, 0.0, 0.0}};
    for (int drone = 0; drone < 3; ++drone)
    {
      refs[drone] = interpolate(schedule.members[drone], world_time);
      if (!refs[drone].valid)
        return TeamTargetCenteredMetrics();
      positions[drone] = targetCenteredCartesian(
          target, refs[drone].radius, refs[drone].bearing,
          schedule.members[drone].fixed_height);
      relative[drone] = positions[drone] - target;
      if (sample > 0)
      {
        const Eigen::Vector3d target_delta = target - positions[drone];
        if (target_delta.head<2>().norm() > 1.0e-6)
          yaw_states[drone] = advanceTargetFacingYaw(
              yaw_states[drone],
              std::atan2(target_delta.y(), target_delta.x()), dt,
              2.0 * M_PI, 5.0 * M_PI);
      }
      visibility[drone] = environment.visibility(
          drone, positions[drone], target, world_time,
          yaw_states[drone].yaw);
      if (!visibility[drone].valid)
        return TeamTargetCenteredMetrics();
      v[drone] = std::max(0.0, std::min(1.0, visibility[drone].value));
      radius_errors.push_back(std::abs(
          refs[drone].radius - schedule.members[drone].desired_radius));
      bearing_errors.push_back(std::abs(wrapTeamAngle(
          refs[drone].bearing - schedule.members[drone].desired_bearing)));
      height_errors.push_back(std::abs(
          refs[drone].height - schedule.members[drone].fixed_height));
    }

    const double q2_value = teamQ2(v);
    const auto q2_gradient = teamQ2Gradient(v);
    metrics.raw.k2 += sample_weight * (1.0 - q2_value);
    metrics.raw.accumulated += sample_weight *
        (3.0 - v[0] - v[1] - v[2]);
    q2_integral += sample_weight * q2_value;
    visibility_integral += sample_weight * (v[0] + v[1] + v[2]);
    std::array<double, 3> visibility_coeff{{0.0, 0.0, 0.0}};
    for (int drone = 0; drone < 3; ++drone)
      visibility_coeff[drone] = sample_weight *
          (-params_.weights.k2 * q2_gradient[drone] -
           params_.weights.accumulated);

    double comp_sum = 0.0;
    for (int first = 0; first < 3; ++first)
    {
      for (int second = first + 1; second < 3; ++second)
      {
        const PairQualitySample quality = targetCenteredPairQuality(
            relative[first], relative[second], params_.pair_bad_angle,
            params_.pair_good_angle);
        if (!quality.valid)
          return TeamTargetCenteredMetrics();
        pair_angles.push_back(quality.angle);
        const double pair_value = v[first] * v[second] * quality.value;
        comp_sum += pair_value;
        visibility_coeff[first] -= sample_weight *
            params_.weights.complementarity * v[second] * quality.value;
        visibility_coeff[second] -= sample_weight *
            params_.weights.complementarity * v[first] * quality.value;
        position_gradient[first] -= sample_weight *
            params_.weights.complementarity * v[first] * v[second] *
            quality.gradient_first;
        position_gradient[second] -= sample_weight *
            params_.weights.complementarity * v[first] * v[second] *
            quality.gradient_second;
      }
    }
    metrics.raw.complementarity += sample_weight * (3.0 - comp_sum);
    complementarity_integral += sample_weight * comp_sum;
    int invisible = 0;
    for (int drone = 0; drone < 3; ++drone)
    {
      position_gradient[drone] += visibility_coeff[drone] *
                                  visibility[drone].gradient_position;
      invisible += v[drone] < 0.5 ? 1 : 0;
      const double radius_error = deadbandResidual(
          refs[drone].radius - schedule.members[drone].desired_radius,
          params_.radius_deadband) / params_.radius_scale;
      const double bearing_error = deadbandResidual(
          wrapTeamAngle(refs[drone].bearing -
                        schedule.members[drone].desired_bearing),
          params_.bearing_deadband) / params_.bearing_scale;
      metrics.raw.encirclement += sample_weight *
          (radius_error * radius_error + bearing_error * bearing_error);
      radius_gradient[drone] += sample_weight * params_.weights.encirclement *
          2.0 * radius_error / params_.radius_scale;
      bearing_gradient[drone] += sample_weight * params_.weights.encirclement *
          2.0 * bearing_error / params_.bearing_scale;
    }
    if (sample + 1 < sample_count && invisible >= 2)
      metrics.simultaneous_los_loss_duration += dt;

    if (gradient)
    {
      for (int drone = 0; drone < 3; ++drone)
      {
        const Eigen::Vector3d dp_dr(std::cos(refs[drone].bearing),
                                    std::sin(refs[drone].bearing), 0.0);
        const Eigen::Vector3d dp_dphi(
            -refs[drone].radius * std::sin(refs[drone].bearing),
             refs[drone].radius * std::cos(refs[drone].bearing), 0.0);
        const double dcost_dr = position_gradient[drone].dot(dp_dr) +
                                radius_gradient[drone];
        const double dcost_dphi = position_gradient[drone].dot(dp_dphi) +
                                  bearing_gradient[drone];
        const int base = variable_offset[drone];
        (*gradient)(base + 2 * static_cast<int>(refs[drone].low)) +=
            refs[drone].low_weight * dcost_dr;
        (*gradient)(base + 2 * static_cast<int>(refs[drone].low) + 1) +=
            refs[drone].low_weight * dcost_dphi;
        if (refs[drone].high != refs[drone].low)
        {
          (*gradient)(base + 2 * static_cast<int>(refs[drone].high)) +=
              refs[drone].high_weight * dcost_dr;
          (*gradient)(base + 2 * static_cast<int>(refs[drone].high) + 1) +=
              refs[drone].high_weight * dcost_dphi;
        }
        const Eigen::Vector3d dp_dt = dp_dr * refs[drone].radius_rate +
                                      dp_dphi * refs[drone].bearing_rate;
        (*gradient)(time_offset + drone) -=
            position_gradient[drone].dot(dp_dt) +
            radius_gradient[drone] * refs[drone].radius_rate +
            bearing_gradient[drone] * refs[drone].bearing_rate;
      }
    }
  }

  const Eigen::VectorXd variables = encode(schedule, reference);
  cursor = 0;
  for (int drone = 0; drone < 3; ++drone)
  {
    for (size_t knot = 0; knot < reference.members[drone].knots.size(); ++knot)
    {
      const double dr = variables(cursor) /
                        params_.regularization_radius_scale;
      metrics.raw.regularization += dr * dr;
      if (gradient)
        (*gradient)(cursor) += params_.weights.regularization * 2.0 * dr /
                               params_.regularization_radius_scale;
      ++cursor;
      const double dp = variables(cursor) /
                        params_.regularization_bearing_scale;
      metrics.raw.regularization += dp * dp;
      if (gradient)
        (*gradient)(cursor) += params_.weights.regularization * 2.0 * dp /
                               params_.regularization_bearing_scale;
      ++cursor;
    }
  }
  for (int drone = 0; drone < 3; ++drone)
  {
    const double dtau = variables(cursor) /
                        params_.regularization_time_scale;
    metrics.raw.regularization += dtau * dtau;
    if (gradient)
      (*gradient)(cursor) += params_.weights.regularization * 2.0 * dtau /
                             params_.regularization_time_scale;
    ++cursor;
  }
  metrics.weighted.k2 = metrics.raw.k2 * params_.weights.k2;
  metrics.weighted.accumulated = metrics.raw.accumulated *
                                 params_.weights.accumulated;
  metrics.weighted.complementarity = metrics.raw.complementarity *
                                     params_.weights.complementarity;
  metrics.weighted.encirclement = metrics.raw.encirclement *
                                  params_.weights.encirclement;
  metrics.weighted.regularization = metrics.raw.regularization *
                                    params_.weights.regularization;
  metrics.objective = metrics.raw.total(params_.weights);
  metrics.sample_count = sample_count;
  const double horizon = std::max(1.0e-9, schedule.prediction_horizon);
  metrics.k2_fraction = std::max(
      0.0, std::min(1.0, q2_integral / horizon));
  metrics.accumulated_visibility = std::max(
      0.0, std::min(1.0, visibility_integral / (3.0 * horizon)));
  metrics.complementarity_score = std::max(
      0.0, std::min(1.0, complementarity_integral / (3.0 * horizon)));
  metrics.quality.q2_ratio = metrics.k2_fraction;
  metrics.quality.accumulated_visibility_ratio =
      metrics.accumulated_visibility;
  metrics.quality.complementarity = metrics.complementarity_score;
  metrics.quality.encirclement_cost = metrics.raw.encirclement;
  metrics.quality.regularization_cost = metrics.raw.regularization;
  metrics.pair_angle_p50 = quantile(pair_angles, 0.50);
  metrics.pair_angle_p90 = quantile(pair_angles, 0.90);
  metrics.radius_error_p50 = quantile(radius_errors, 0.50);
  metrics.radius_error_p90 = quantile(radius_errors, 0.90);
  metrics.bearing_error_p50 = quantile(bearing_errors, 0.50);
  metrics.bearing_error_p90 = quantile(bearing_errors, 0.90);
  metrics.height_error_p90 = quantile(height_errors, 0.90);
  metrics.height_error_max = quantile(height_errors, 1.0);
  metrics.valid = std::isfinite(metrics.objective) &&
                  metrics.quality.allFinite() &&
                  (!gradient || gradient->allFinite());
  return metrics;
}

TeamTargetCenteredResult TeamTargetCenteredOptimizer::optimize(
    const TeamTargetCenteredInput &input,
    const TeamTargetCenteredEnvironment &environment) const
{
  TeamTargetCenteredResult result;
  result.schedule = input;
  if (!validScheduleIdentity(input))
  {
    result.reason = "INVALID_TARGET_CENTERED_INPUT";
    return result;
  }
  using Clock = std::chrono::steady_clock;
  const auto wall_start = Clock::now();
  const auto wall_deadline = wall_start + std::chrono::duration_cast<
      Clock::duration>(std::chrono::duration<double>(params_.timeout_seconds));
  result.tolerance = teamQualityToleranceFromSampling(
      input.prediction_horizon, params_.sample_dt);
  result.before = evaluate(input, input, environment);
  result.after = result.before;
  if (!result.before.valid)
  {
    result.reason = "INVALID_TARGET_CENTERED_METRICS";
    return result;
  }

  const auto stage_deadline = [&](const int remaining_stages) {
    const auto now = Clock::now();
    if (now >= wall_deadline)
      return now;
    return now + (wall_deadline - now) /
                     std::max(1, remaining_stages);
  };

  const auto run_stage = [&](
      const TeamTargetCenteredInput &start,
      const TeamOptimizationStage stage,
      const double q_floor,
      const double visibility_floor,
      const double complementarity_floor,
      const Clock::time_point deadline) {
    StageRun run;
    const auto stage_start = Clock::now();
    run.schedule = start;
    run.metrics = evaluate(start, input, environment);
    if (!run.metrics.valid)
    {
      run.latency_ms = 1000.0 * std::chrono::duration<double>(
          Clock::now() - stage_start).count();
      return run;
    }
    if (environment.cancelled && environment.cancelled())
    {
      run.cancelled = true;
      run.latency_ms = 1000.0 * std::chrono::duration<double>(
          Clock::now() - stage_start).count();
      return run;
    }
    if (Clock::now() >= deadline)
    {
      run.timed_out = true;
      run.latency_ms = 1000.0 * std::chrono::duration<double>(
          Clock::now() - stage_start).count();
      return run;
    }

    Eigen::VectorXd variables = encode(start, input);
    Eigen::VectorXd gradient;
    const auto compute_gradient = [&](
        const TeamTargetCenteredInput &schedule,
        const TeamTargetCenteredMetrics &metrics,
        Eigen::VectorXd &output) {
      TeamTargetCenteredParams gradient_params = params_;
      gradient_params.weights = stageGradientWeights(
          metrics, stage, params_, result.tolerance,
          input.prediction_horizon, q_floor, visibility_floor,
          complementarity_floor);
      const TeamTargetCenteredOptimizer gradient_evaluator(gradient_params);
      const TeamTargetCenteredMetrics evaluated = gradient_evaluator.evaluate(
          schedule, input, environment, &output);
      return evaluated.valid && output.allFinite();
    };
    if (!compute_gradient(run.schedule, run.metrics, gradient))
    {
      run.latency_ms = 1000.0 * std::chrono::duration<double>(
          Clock::now() - stage_start).count();
      return run;
    }
    double best_score = stageObjective(
        run.metrics, stage, params_, result.tolerance, q_floor,
        visibility_floor, complementarity_floor);
    for (int iteration = 0; iteration < params_.max_iterations; ++iteration)
    {
      if (environment.cancelled && environment.cancelled())
      {
        run.cancelled = true;
        break;
      }
      if (Clock::now() >= deadline)
      {
        run.timed_out = true;
        break;
      }
      if (!gradient.allFinite() || gradient.norm() <= 1.0e-9)
        break;
      const double scale = std::max(
          1.0, gradient.lpNorm<Eigen::Infinity>());
      const Eigen::VectorXd direction = -gradient / scale;
      double step = params_.initial_step;
      bool accepted = false;
      while (step >= params_.minimum_step)
      {
        if (Clock::now() >= deadline)
        {
          run.timed_out = true;
          break;
        }
        const TeamTargetCenteredInput trial = decode(
            variables + step * direction, input);
        const TeamTargetCenteredMetrics trial_metrics = evaluate(
            trial, input, environment);
        const double trial_score = stageObjective(
            trial_metrics, stage, params_, result.tolerance, q_floor,
            visibility_floor, complementarity_floor);
        if (trial_metrics.valid &&
            trial_score < best_score - 1.0e-12)
        {
          Eigen::VectorXd trial_gradient;
          if (!compute_gradient(trial, trial_metrics, trial_gradient))
            break;
          variables = encode(trial, input);
          gradient = trial_gradient;
          run.schedule = trial;
          run.metrics = trial_metrics;
          best_score = trial_score;
          accepted = true;
          ++run.iterations;
          break;
        }
        step *= 0.5;
      }
      if (run.timed_out || !accepted)
        break;
    }
    run.valid = run.metrics.valid && !run.cancelled && !run.timed_out;
    run.latency_ms = 1000.0 * std::chrono::duration<double>(
        Clock::now() - stage_start).count();
    return run;
  };

  TeamTargetCenteredInput accepted_schedule = input;
  TeamTargetCenteredMetrics accepted_metrics = result.before;
  result.final_stage = TeamOptimizationStage::LOCAL_BASELINE;

  const StageRun stage_q2 = run_stage(
      accepted_schedule, TeamOptimizationStage::VISIBILITY_Q2,
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(), stage_deadline(4));
  result.stage_q2_latency_ms = stage_q2.latency_ms;
  result.iterations += stage_q2.iterations;
  if (!stage_q2.valid)
  {
    result.stage1_fallback = true;
    result.stage1_q2 = result.before;
    result.stage1_acc = result.before;
    result.stage2_comp = result.before;
    result.stage3_enc = result.before;
    result.reason = stage_q2.timed_out ? "STAGE1_Q2_TIMEOUT_LOCAL_BASELINE"
                                       : "STAGE1_Q2_FAILED_LOCAL_BASELINE";
  }
  else
  {
    accepted_schedule = stage_q2.schedule;
    accepted_metrics = stage_q2.metrics;
    result.stage1_q2 = stage_q2.metrics;
    result.final_stage = TeamOptimizationStage::VISIBILITY_Q2;
    result.q_floor = std::max(
        result.before.quality.q2_ratio,
        stage_q2.metrics.quality.q2_ratio - result.tolerance.q2);

    const StageRun stage_acc = run_stage(
        accepted_schedule, TeamOptimizationStage::VISIBILITY_ACC,
        result.q_floor, -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), stage_deadline(3));
    result.stage_acc_latency_ms = stage_acc.latency_ms;
    result.iterations += stage_acc.iterations;
    const bool stage_acc_legal = stage_acc.valid &&
        stage_acc.metrics.quality.q2_ratio + 1.0e-9 >= result.q_floor;
    if (!stage_acc_legal)
    {
      result.stage1_fallback = true;
      result.primary_visibility_rejected = stage_acc.valid;
      result.stage1_acc = accepted_metrics;
      result.stage2_comp = accepted_metrics;
      result.stage3_enc = accepted_metrics;
      result.reason = stage_acc.timed_out
          ? "STAGE1_ACC_TIMEOUT_USE_Q2"
          : (stage_acc.valid ? "STAGE1_ACC_Q2_FLOOR_REJECT_USE_Q2"
                             : "STAGE1_ACC_FAILED_USE_Q2");
    }
    else
    {
      accepted_schedule = stage_acc.schedule;
      accepted_metrics = stage_acc.metrics;
      result.stage1_acc = stage_acc.metrics;
      result.final_stage = TeamOptimizationStage::VISIBILITY_ACC;
      result.q_floor = std::max(
          result.before.quality.q2_ratio,
          accepted_metrics.quality.q2_ratio - result.tolerance.q2);
      result.visibility_floor = std::max(
          result.before.quality.accumulated_visibility_ratio,
          accepted_metrics.quality.accumulated_visibility_ratio -
              result.tolerance.accumulated_visibility);

      const StageRun stage_comp = run_stage(
          accepted_schedule, TeamOptimizationStage::COMPLEMENTARITY,
          result.q_floor, result.visibility_floor,
          -std::numeric_limits<double>::infinity(), stage_deadline(2));
      result.stage_comp_latency_ms = stage_comp.latency_ms;
      result.iterations += stage_comp.iterations;
      const bool stage_comp_legal = stage_comp.valid &&
          teamQualitySatisfiesFloors(
              stage_comp.metrics.quality, result.q_floor,
              result.visibility_floor);
      if (!stage_comp_legal)
      {
        result.stage2_fallback = true;
        result.primary_visibility_rejected = stage_comp.valid;
        result.stage2_comp = accepted_metrics;
        result.stage3_enc = accepted_metrics;
        result.reason = stage_comp.timed_out
            ? "STAGE2_TIMEOUT_USE_STAGE1"
            : (stage_comp.valid
                   ? "STAGE2_VISIBILITY_FLOOR_REJECT_USE_STAGE1"
                   : "STAGE2_FAILED_USE_STAGE1");
      }
      else
      {
        accepted_schedule = stage_comp.schedule;
        accepted_metrics = stage_comp.metrics;
        result.stage2_comp = stage_comp.metrics;
        result.final_stage = TeamOptimizationStage::COMPLEMENTARITY;
        result.complementarity_floor = std::max(
            result.before.quality.complementarity,
            accepted_metrics.quality.complementarity -
                result.tolerance.complementarity);

        const StageRun stage_enc = run_stage(
            accepted_schedule,
            TeamOptimizationStage::ENCIRCLEMENT_REGULARIZATION,
            result.q_floor, result.visibility_floor,
            result.complementarity_floor, stage_deadline(1));
        result.stage_enc_latency_ms = stage_enc.latency_ms;
        result.iterations += stage_enc.iterations;
        const bool stage_enc_visibility_legal = stage_enc.valid &&
            teamQualitySatisfiesFloors(
                stage_enc.metrics.quality, result.q_floor,
                result.visibility_floor);
        const bool stage_enc_comp_legal = stage_enc_visibility_legal &&
            stage_enc.metrics.quality.complementarity + 1.0e-9 >=
                result.complementarity_floor;
        if (!stage_enc_visibility_legal || !stage_enc_comp_legal)
        {
          result.stage3_fallback = true;
          result.primary_visibility_rejected =
              result.primary_visibility_rejected ||
              (stage_enc.valid && !stage_enc_visibility_legal);
          result.complementarity_rejected =
              stage_enc.valid && stage_enc_visibility_legal &&
              !stage_enc_comp_legal;
          result.stage3_enc = accepted_metrics;
          result.reason = stage_enc.timed_out
              ? "STAGE3_TIMEOUT_USE_STAGE2"
              : (!stage_enc_visibility_legal
                     ? "STAGE3_VISIBILITY_FLOOR_REJECT_USE_STAGE2"
                     : "STAGE3_COMPLEMENTARITY_FLOOR_REJECT_USE_STAGE2");
        }
        else
        {
          accepted_schedule = stage_enc.schedule;
          accepted_metrics = stage_enc.metrics;
          result.stage3_enc = stage_enc.metrics;
          result.final_stage =
              TeamOptimizationStage::ENCIRCLEMENT_REGULARIZATION;
          result.reason = "HIERARCHICAL_REFINEMENT_COMPLETE";
        }
      }
    }
  }

  result.schedule = accepted_schedule;
  result.after = evaluate(accepted_schedule, input, environment);
  const bool primary_non_worsening = teamPrimaryVisibilityNonWorsening(
      result.after.quality, result.before.quality, result.tolerance);
  if (!primary_non_worsening)
  {
    result.primary_visibility_rejected = true;
    result.success = false;
    result.reason = "PRIMARY_VISIBILITY_WORSENED";
  }
  else if (!betterTeamQuality(
               result.after.quality, result.before.quality,
               result.tolerance))
  {
    result.success = false;
    result.reason = "NO_LEXICOGRAPHIC_TEAM_IMPROVEMENT";
  }
  else
  {
    result.success = true;
  }
  for (int drone = 0; drone < 3; ++drone)
    result.reference_phase_shifts[drone] =
        accepted_schedule.members[drone].reference_phase_shift;
  result.solve_time_ms = 1000.0 * std::chrono::duration<double>(
      Clock::now() - wall_start).count();
  return result;
}

}  // namespace multi_uav_formation
