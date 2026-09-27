#include <multi_uav_formation/realized_team_validator.h>

#include <algorithm>
#include <cmath>

namespace multi_uav_formation
{
namespace
{
double wrapPi(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace

RealizedTeamMetrics RealizedTeamValidator::evaluateMetrics(
    const std::array<RealizedTeamMember, 3> &members,
    const std::array<int, 3> &slot_for_uav, double activation,
    double horizon, const Params &params, const VisibilityFn &visibility,
    const TargetFn &target_at)
{
  RealizedTeamMetrics metrics;
  const double dt = std::max(1.0e-3, params.sample_dt);
  const int samples = std::max(2, static_cast<int>(std::ceil(horizon / dt)));
  double q2_sum = 0.0, acc_sum = 0.0, redundancy_sum = 0.0,
         encirclement_sum = 0.0;
  int counted = 0;

  // yaw rollout 状态：activation 时刻必须来自 Local 的预测执行状态。
  std::array<PredictedYawState, 3> yaw_states;
  std::array<Eigen::Vector3d, 3> start_positions;
  bool positions_ok = true;
  for (int drone = 0; drone < 3; ++drone)
  {
    if (!members[drone].valid || members[drone].trajectory.getPieceNum() <= 0)
    {
      positions_ok = false;
      break;
    }
    start_positions[drone] = members[drone].trajectory.getPos(
        std::max(0.0, std::min(members[drone].trajectory.getTotalDuration(),
                               activation - members[drone].start_time)));
    if (!members[drone].initial_yaw_valid ||
        !std::isfinite(members[drone].initial_yaw) ||
        !std::isfinite(members[drone].initial_yaw_rate))
    {
      positions_ok = false;
      break;
    }
    yaw_states[drone].valid = true;
    yaw_states[drone].yaw = members[drone].initial_yaw;
    yaw_states[drone].yaw_rate = members[drone].initial_yaw_rate;
  }
  if (!positions_ok)
    return metrics;

  for (int sample = 0; sample <= samples; ++sample)
  {
    const double t = activation + std::min(horizon, sample * dt);
    const Eigen::Vector3d target = target_at(t);
    std::array<Eigen::Vector3d, 3> positions;
    std::array<double, 3> visibility_values;
    std::array<double, 3> visibility_margins;
    bool all_valid = true;
    bool binary_valid = true;
    int binary_visible_count = 0;
    for (int drone = 0; drone < 3; ++drone)
    {
      const double tau = std::max(
          0.0, std::min(members[drone].trajectory.getTotalDuration(),
                        t - members[drone].start_time));
      positions[drone] = members[drone].trajectory.getPos(tau);
      if (sample > 0)
      {
        const Eigen::Vector3d delta = target - positions[drone];
        if (delta.head<2>().norm() > 1.0e-6)
          yaw_states[drone] = advanceTargetFacingYaw(
              yaw_states[drone], std::atan2(delta.y(), delta.x()), dt,
              params.yaw_rate_limit, params.yaw_accel_limit);
      }
      const double yaw = yaw_states[drone].yaw;
      const ContinuousVisibilitySample sample_value =
          visibility(drone, positions[drone], target, t, yaw);
      if (!sample_value.valid || !sample_value.margin_valid ||
          !std::isfinite(sample_value.value) ||
          !std::isfinite(sample_value.margin))
      {
        all_valid = false;
        break;
      }
      visibility_values[drone] =
          std::max(0.0, std::min(1.0, sample_value.value));
      binary_valid = binary_valid && sample_value.binary_valid;
      binary_visible_count += sample_value.binary_visible ? 1 : 0;
      metrics.binary_visibility_valid =
          metrics.binary_visibility_valid && sample_value.binary_valid &&
          sample_value.binary_visible;
      visibility_margins[drone] = sample_value.margin;
    }
    metrics.binary_visible_counts.push_back(
        all_valid && binary_valid ? binary_visible_count : -1);
    if (!all_valid)
    {
      metrics.q2_samples.push_back(
          std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    const double v0 = visibility_values[0], v1 = visibility_values[1],
                 v2 = visibility_values[2];
    std::array<double, 3> margins = visibility_margins;
    std::sort(margins.begin(), margins.end(), std::greater<double>());
    if (margins[1] < metrics.min_m2)
    {
      metrics.min_m2 = margins[1];
      metrics.min_m2_world_time = t;
    }
    // 与 TeamTargetCenteredOptimizer 相同的容斥式。
    const double q2 = v0 * v1 + v0 * v2 + v1 * v2 - 2.0 * v0 * v1 * v2;
    metrics.q2_samples.push_back(q2);
    q2_sum += q2;
    acc_sum += (v0 + v1 + v2) / 3.0;
    // redundancy / encirclement：复用 encirclement_geometry.h 的共享实现。
    const std::array<Eigen::Vector3d, 3> relative{
        positions[0] - target, positions[1] - target, positions[2] - target};
    const MultiviewQuality quality = multiviewQuality(relative);
    redundancy_sum += 1.0 - quality.quality;
    encirclement_sum += encirclementGeometryCost(
        relative, /*minimum_angle*/ 25.0 * M_PI / 180.0,
        /*maximum_gap*/ 170.0 * M_PI / 180.0);
    ++counted;

    // 两两真实间距（Team interaction 指标）。
    for (int i = 0; i < 3; ++i)
      for (int j = i + 1; j < 3; ++j)
      {
        const double distance =
            (positions[i] - positions[j]).norm();
        metrics.min_pairwise_distance =
            std::min(metrics.min_pairwise_distance, distance);
        metrics.max_pairwise_distance =
            std::max(metrics.max_pairwise_distance, distance);
      }
  }

  if (counted <= 0)
    return metrics;
  metrics.q2 = q2_sum / counted;
  metrics.accumulated = acc_sum / counted;
  metrics.redundancy = redundancy_sum / counted;
  metrics.encirclement = encirclement_sum / counted;
  metrics.valid = true;
  return metrics;
}

RealizedTeamValidationResult RealizedTeamValidator::validate(
    const RealizedTeamValidationInput &input, const Params &params,
    const VisibilityFn &visibility, const TargetFn &target_at) const
{
  RealizedTeamValidationResult result;

  // ---- 1) identity：同一 reference / task reference / snapshot / activation。
  result.identity_ok = input.team_reference_id != 0 &&
                       input.activation_time > 0.0 &&
                       input.execution_horizon > 0.0 &&
                       input.task_reference_generation != 0 &&
                       input.target_prediction_revision != 0 &&
                       input.dynamic_prediction_identity != 0 &&
                       input.static_map_revision != 0 &&
                       input.visibility_model_version != 0;
  if (!result.identity_ok)
  {
    result.reason = "IDENTITY_INCOMPLETE";
    return result;
  }

  // ---- 2) Local certificates：三条 realized 必须全部 realization_valid。
  result.certificates_ok = true;
  for (int drone = 0; drone < 3; ++drone)
    result.certificates_ok = result.certificates_ok &&
                             input.realized[drone].valid &&
                             input.realized[drone].trajectory.getPieceNum() > 0;
  if (!result.certificates_ok)
  {
    result.reason = "LOCAL_REALIZATION_CERTIFICATE_MISSING";
    return result;
  }

  // Validate against the exact frozen seed that defined the SCP hard rows.
  result.side_topology_ok = true;
  for (int drone = 0; drone < 3 && result.side_topology_ok; ++drone)
  {
    const auto &side = input.side_contracts[drone];
    if (!side.active)
      continue;
    if (!side.origin.allFinite() || !side.direction.allFinite() ||
        side.direction.norm() < 1.0e-6 || side.side_sign == 0 ||
        !std::isfinite(side.offset) || side.offset <= 0.0 ||
        side.seed_trajectory.getPieceNum() <= 0 ||
        !std::isfinite(side.seed_trajectory.getTotalDuration()) ||
        side.seed_trajectory.getTotalDuration() <= 1.0e-6)
    {
      result.side_topology_ok = false;
      break;
    }
    const Eigen::Vector3d direction = side.direction.normalized();
    const auto &trajectory = input.realized[drone].trajectory;
    constexpr int kSideSamples = 41;
    for (int sample = 0; sample < kSideSamples; ++sample)
    {
      const double progress = static_cast<double>(sample) /
                              static_cast<double>(kSideSamples - 1);
      if (std::abs(progress - side.conflict_progress) >=
          std::max(1.0e-3, std::min(1.0, side.guidance_window)))
        continue;
      const Eigen::Vector3d seed_position = side.seed_trajectory.getPos(
          progress * side.seed_trajectory.getTotalDuration());
      if (!seed_position.allFinite())
      {
        result.side_topology_ok = false;
        break;
      }
      const double signed_seed = side.side_sign *
          (seed_position - side.origin).dot(direction);
      const double lower = std::max(
          0.0, signed_seed - std::max(0.10, 0.25 * std::abs(signed_seed)));
      if (lower <= 1.0e-9)
        continue;
      const double lateral = side.side_sign *
          (trajectory.getPos(progress * trajectory.getTotalDuration()) -
           side.origin).dot(direction);
      if (lateral < lower - 1.0e-6)
      {
        result.side_topology_ok = false;
        break;
      }
    }
  }
  if (!result.side_topology_ok)
  {
    result.reason = "REALIZED_SIDE_TOPOLOGY_VIOLATION";
    return result;
  }

  // ---- 3/4) realized metrics（真实 polynomial 采样）。
  result.realized = evaluateMetrics(
      input.realized, input.slot_for_uav, input.activation_time,
      input.execution_horizon, params, visibility, target_at);
  if (!result.realized.valid)
  {
    result.reason = "REALIZED_METRICS_NOT_EVALUABLE";
    return result;
  }

  // ---- Team interaction：既有 swarm_clearance 阈值。
  result.interaction_ok =
      result.realized.min_pairwise_distance >= params.swarm_min_separation;
  if (!result.interaction_ok)
  {
    result.reason = "REALIZED_TEAM_INTERACTION_VIOLATION";
    return result;
  }

  // ---- Predictive visibility relay：真实 polynomial 上的 make-before-break。
  // The scalar quality comparison below cannot prove a relay contract: it may
  // average away a short two-camera outage.  Re-sample the exact realized
  // trajectories and require the incoming camera to acquire no later than the
  // contract deadline while outgoing remains valid through preserve_until.
  result.handoff_ok = !input.handoff_contract_active;
  if (input.handoff_contract_active)
  {
    const bool roles_valid = input.handoff_contract_id != 0 &&
        input.outgoing_uav >= 0 && input.outgoing_uav < 3 &&
        input.incoming_uav >= 0 && input.incoming_uav < 3 &&
        input.stable_observer_uav >= 0 && input.stable_observer_uav < 3 &&
        input.outgoing_uav != input.incoming_uav &&
        input.outgoing_uav != input.stable_observer_uav &&
        input.incoming_uav != input.stable_observer_uav &&
        input.acquire_by_world_time >= input.activation_time - 1.0e-9 &&
        input.preserve_until_world_time >=
            input.acquire_by_world_time + input.required_overlap - 1.0e-9;
    if (!roles_valid)
    {
      result.reason = "HANDOFF_CONTRACT_INVALID";
      return result;
    }

    std::array<PredictedYawState, 3> yaw_states;
    for (int drone = 0; drone < 3; ++drone)
    {
      yaw_states[drone].valid = input.realized[drone].initial_yaw_valid;
      yaw_states[drone].yaw = input.realized[drone].initial_yaw;
      yaw_states[drone].yaw_rate = input.realized[drone].initial_yaw_rate;
    }
    const double end = std::min(
        input.activation_time + input.execution_horizon,
        input.preserve_until_world_time);
    const double dt = std::max(1.0e-3, params.sample_dt);
    bool incoming_acquired = false;
    bool outgoing_preserved = true;
    bool stable_preserved = true;
    double overlap = 0.0;
    for (double world_time = input.activation_time;
         world_time <= end + 1.0e-9; world_time += dt)
    {
      const double t = std::min(world_time, end);
      const Eigen::Vector3d target = target_at(t);
      std::array<ContinuousVisibilitySample, 3> samples;
      bool sample_ok = true;
      for (int drone = 0; drone < 3; ++drone)
      {
        const auto &member = input.realized[drone];
        const double tau = std::max(
            0.0, std::min(member.trajectory.getTotalDuration(),
                          t - member.start_time));
        const Eigen::Vector3d position = member.trajectory.getPos(tau);
        if (t > input.activation_time + 1.0e-9)
        {
          const Eigen::Vector3d delta = target - position;
          if (delta.head<2>().norm() > 1.0e-6)
            yaw_states[drone] = advanceTargetFacingYaw(
                yaw_states[drone], std::atan2(delta.y(), delta.x()), dt,
                params.yaw_rate_limit, params.yaw_accel_limit);
        }
        samples[drone] = visibility(
            drone, position, target, t, yaw_states[drone].yaw);
        sample_ok = sample_ok && samples[drone].valid &&
                    samples[drone].margin_valid &&
                    samples[drone].binary_valid;
      }
      if (!sample_ok)
      {
        outgoing_preserved = stable_preserved = false;
        continue;
      }
      const auto visible = [&](const int drone) {
        return samples[drone].binary_visible &&
               samples[drone].margin + 1.0e-9 >=
                   input.required_visibility_margin;
      };
      if (t <= input.acquire_by_world_time + 0.5 * dt &&
          visible(input.incoming_uav))
        incoming_acquired = true;
      if (t <= input.preserve_until_world_time + 1.0e-9)
      {
        outgoing_preserved = outgoing_preserved &&
                             visible(input.outgoing_uav);
        stable_preserved = stable_preserved &&
                           visible(input.stable_observer_uav);
        if (visible(input.incoming_uav) && visible(input.outgoing_uav))
          overlap += dt;
      }
      if (t >= end - 1.0e-9)
        break;
    }
    result.realized_overlap = std::min(
        overlap, std::max(0.0, end - input.activation_time));
    result.handoff_ok = incoming_acquired && outgoing_preserved &&
                        stable_preserved &&
                        result.realized.min_m2 + 1.0e-9 >=
                            input.required_visibility_margin &&
                        result.realized_overlap + 0.5 * dt >=
                            input.required_overlap;
    if (!result.handoff_ok)
    {
      result.reason = "HANDOFF_MAKE_BEFORE_BREAK_VIOLATION";
      return result;
    }
  }

  // ---- 5) baseline comparison：同一 activation / horizon 的 committed
  // Local triple。baseline 缺失（例如冷启动）时无法证明收益 → 拒绝。
  bool baseline_available = true;
  for (int drone = 0; drone < 3; ++drone)
    baseline_available = baseline_available && input.baseline[drone].valid &&
                         input.baseline[drone].trajectory.getPieceNum() > 0;
  if (!baseline_available)
  {
    result.reason = "LOCAL_BASELINE_UNAVAILABLE";
    return result;
  }
  result.baseline = evaluateMetrics(
      input.baseline, input.slot_for_uav, input.activation_time,
      input.execution_horizon, params, visibility, target_at);
  if (!result.baseline.valid)
  {
    result.reason = "BASELINE_METRICS_NOT_EVALUABLE";
    return result;
  }

  if (input.repair_k == 3)
  {
    const double dt = std::max(1.0e-3, params.sample_dt);
    const size_t expected = static_cast<size_t>(
        std::max(2, static_cast<int>(std::ceil(input.execution_horizon / dt)))
        + 1);
    if (input.repair_target_uav < 0 || input.repair_target_uav >= 3 ||
        !std::isfinite(input.critical_begin_world_time) ||
        !std::isfinite(input.critical_end_world_time) ||
        input.critical_end_world_time < input.critical_begin_world_time ||
        result.realized.binary_visible_counts.size() != expected ||
        result.baseline.binary_visible_counts.size() != expected)
    {
      result.reason = "K3_VALIDATION_INPUT_INVALID";
      return result;
    }
    int critical_samples = 0;
    for (size_t index = 0; index < expected; ++index)
    {
      const int before = result.baseline.binary_visible_counts[index];
      const int after = result.realized.binary_visible_counts[index];
      if (before < 0 || after < 0)
      {
        result.reason = "K3_BINARY_VISIBILITY_UNAVAILABLE";
        return result;
      }
      result.total_k3_before += before == 3 ? 1 : 0;
      result.total_k3_after += after == 3 ? 1 : 0;
      result.k2_regression_samples += before >= 2 && after < 2 ? 1 : 0;
      const double t = input.activation_time + std::min(
          input.execution_horizon, static_cast<double>(index) * dt);
      if (t >= input.critical_begin_world_time - 0.5 * dt &&
          t <= input.critical_end_world_time + 0.5 * dt)
      {
        ++critical_samples;
        result.critical_k3_before += before == 3 ? 1 : 0;
        result.critical_k3_after += after == 3 ? 1 : 0;
      }
    }
    if (critical_samples == 0)
      result.reason = "K3_CRITICAL_WINDOW_NOT_SAMPLED";
    else if (result.critical_k3_after <= result.critical_k3_before)
      result.reason = "K3_CRITICAL_GAIN_MISSING";
    else if (result.k2_regression_samples > 0)
      result.reason = "K3_K2_REGRESSION";
    else if (result.total_k3_after < result.total_k3_before)
      result.reason = "K3_TOTAL_VISIBILITY_WORSENED";
    if (!result.reason.empty())
      return result;
  }

  // 字典序：先 Q2 floor，再 ACC，再 geometry（redundancy/encirclement）。
  // repair_k==3 的 Q2 底座改为逐采样判据。原因：临界窗的定义就是另两机
  // binary 可见（count==2），容斥式 Q2 = v1*v2 + v0*(v1+v2-2*v1*v2) 在
  // v1=v2=1 时对第三机 v0 的偏导为 0 —— K3 修复的收益对全时域均值口径
  // 结构性不可见，而修复在其它采样点的微小代价会被全额计入
  // （run 20260925_021921：均值仅降 1.19e-4 即被 1e-4 容差拒绝，逐采样
  // 最大降幅约 2.5e-6）。逐采样底座保留 floor 的本意：任何由 >=2 机承载
  // 的采样，其连续 Q2 都不允许被修复显著毁掉；同时不再惩罚对度量不可见
  // 的 K3 增益。这不是放宽 epsilon——容差仍为 params.q2_epsilon。
  // M2（repair_k==2）与普通 realization 路径保持原均值口径不变。
  if (input.repair_k == 3)
  {
    const size_t aligned =
        std::min(result.realized.q2_samples.size(),
                 result.baseline.q2_samples.size());
    for (size_t index = 0; index < aligned; ++index)
    {
      const int baseline_count = index < result.baseline.binary_visible_counts.size()
          ? result.baseline.binary_visible_counts[index] : -1;
      const double baseline_q2 = result.baseline.q2_samples[index];
      const double realized_q2 = result.realized.q2_samples[index];
      if (baseline_count < 2 ||
          !std::isfinite(baseline_q2) || !std::isfinite(realized_q2))
        continue;  // 非双机承载或 authority 无效的采样不参与底座
      if (realized_q2 < baseline_q2 - params.q2_epsilon)
      {
        result.reason = "PRIMARY_VISIBILITY_WORSENED";
        return result;
      }
    }
  }
  else if (result.realized.q2 < result.baseline.q2 - params.q2_epsilon)
  {
    result.reason = "PRIMARY_VISIBILITY_WORSENED";
    return result;
  }
  if (result.realized.accumulated <
      result.baseline.accumulated - params.acc_epsilon)
  {
    result.reason = "ACCUMULATED_VISIBILITY_WORSENED";
    return result;
  }
  const double realized_geometry = result.realized.redundancy +
                                   result.realized.encirclement;
  const double baseline_geometry = result.baseline.redundancy +
                                   result.baseline.encirclement;
  if (realized_geometry > baseline_geometry + params.q2_epsilon)
  {
    result.reason = "NO_LEXICOGRAPHIC_TEAM_IMPROVEMENT";
    return result;
  }

  result.pass = true;
  result.reason = "REALIZED_TEAM_BENEFICIAL";
  return result;
}

}  // namespace multi_uav_formation
