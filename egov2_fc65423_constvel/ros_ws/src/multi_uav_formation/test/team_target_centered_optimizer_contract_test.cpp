#include <multi_uav_formation/team_target_centered_optimizer.h>

#include <cmath>
#include <iostream>

using namespace multi_uav_formation;

namespace
{

TeamTargetCenteredInput makeInput()
{
  TeamTargetCenteredInput input;
  input.evaluation_start_world = 100.0;
  input.prediction_horizon = 2.0;
  input.target_prediction_revision = 7;
  for (int drone = 0; drone < 3; ++drone)
  {
    auto &member = input.members[drone];
    member.drone_id = drone;
    member.source_candidate_id = 10 + drone;
    member.topology_kind = drone;
    member.desired_radius = 2.0;
    member.desired_bearing = drone * 2.0 * M_PI / 3.0;
    member.fixed_height = 1.0;
    for (int knot = 0; knot < 4; ++knot)
    {
      TeamReferenceKnot value;
      value.world_time = 100.0 + 2.0 * knot / 3.0;
      value.radius = 2.0;
      if (drone == 0)
        value.bearing = -0.55 + 1.10 * knot / 3.0;
      else if (drone == 1)
        value.bearing = 0.95 + 1.10 * knot / 3.0;
      else
        value.bearing = 4.0;
      value.height = 1.0;
      member.knots.push_back(value);
    }
  }
  return input;
}

TeamTargetCenteredEnvironment controlledEnvironment()
{
  TeamTargetCenteredEnvironment environment;
  environment.target_at_world_time = [](double) {
    return Eigen::Vector3d::Zero();
  };
  environment.visibility = [](
      int drone, const Eigen::Vector3d &observer, const Eigen::Vector3d &,
      double, double) {
    ContinuousVisibilitySample sample;
    sample.valid = observer.allFinite();
    sample.binary_valid = sample.valid;
    if (drone == 2)
    {
      sample.value = 1.0;
      sample.binary_visible = true;
      return sample;
    }
    // Slightly offset the second loss so the simultaneous-loss plateau has a
    // deterministic descent direction; without Team timing the two windows
    // still overlap substantially.
    const double center = drone == 0 ? 0.0 : 1.46;
    const double angle = std::atan2(observer.y(), observer.x());
    const double delta = wrapTeamAngle(angle - center);
    const double sigma = 0.16;
    const double occlusion = std::exp(-delta * delta / (sigma * sigma));
    sample.value = 1.0 - occlusion;
    const double radius2 = observer.head<2>().squaredNorm();
    if (radius2 > 1.0e-9)
    {
      const double derivative_angle =
          2.0 * delta / (sigma * sigma) * occlusion;
      sample.gradient_position.x() = derivative_angle *
                                     (-observer.y() / radius2);
      sample.gradient_position.y() = derivative_angle *
                                      (observer.x() / radius2);
    }
    sample.binary_visible = sample.value >= 0.5;
    return sample;
  };
  return environment;
}

bool near(double a, double b, double tolerance)
{
  return std::abs(a - b) <= tolerance *
      std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

}  // namespace

int main()
{
  const Eigen::Vector3d target(1.0, -2.0, 0.5);
  const Eigen::Vector3d position = targetCenteredCartesian(
      target, 2.3, 0.7, 1.1);
  double radius = 0.0, bearing = 0.0, height = 0.0;
  if (!cartesianToTargetCentered(position, target, radius, bearing, height) ||
      !near(radius, 2.3, 1.0e-10) ||
      !near(wrapTeamAngle(bearing - 0.7), 0.0, 1.0e-10) ||
      !near(height, 1.1, 1.0e-10))
  {
    std::cerr << "TARGET_CENTERED_MAPPING_FAILED\n";
    return 1;
  }

  TeamTargetCenteredParams params;
  params.max_iterations = 16;
  params.timeout_seconds = 0.2;
  params.radius_trust = 0.0;
  params.bearing_trust = 0.0;
  params.phase_trust = 0.45;
  params.weights.encirclement = 0.0;
  params.weights.regularization = 0.01;
  TeamTargetCenteredOptimizer optimizer(params);
  TeamTargetCenteredInput input = makeInput();
  const auto environment = controlledEnvironment();
  Eigen::VectorXd gradient;
  const auto baseline = optimizer.evaluate(input, input, environment, &gradient);
  if (!baseline.valid || gradient.size() != 27)
  {
    std::cerr << "WORLD_TIME_SYNCHRONIZATION_CONTRACT_FAILED\n";
    return 2;
  }
  if (!baseline.quality.allFinite() ||
      baseline.quality.q2_ratio < 0.0 ||
      baseline.quality.q2_ratio > 1.0 ||
      baseline.quality.accumulated_visibility_ratio < 0.0 ||
      baseline.quality.accumulated_visibility_ratio > 1.0 ||
      baseline.quality.complementarity < 0.0 ||
      baseline.quality.complementarity > 1.0)
  {
    std::cerr << "TEAM_QUALITY_VECTOR_NORMALIZATION_FAILED\n";
    return 7;
  }

  const TeamQualityTolerance quality_tolerance =
      teamQualityToleranceFromSampling(input.prediction_horizon,
                                       params.sample_dt);
  TeamQualityVector encirclement_preferred;
  encirclement_preferred.q2_ratio = 1.0 - 2.0 * quality_tolerance.q2;
  encirclement_preferred.accumulated_visibility_ratio = 1.0;
  encirclement_preferred.complementarity = 1.0;
  encirclement_preferred.encirclement_cost = 0.0;
  encirclement_preferred.regularization_cost = 0.0;
  TeamQualityVector visibility_preferred = encirclement_preferred;
  visibility_preferred.q2_ratio = 1.0;
  visibility_preferred.encirclement_cost = 1.0e9;
  visibility_preferred.regularization_cost = 1.0e9;
  if (!betterTeamQuality(visibility_preferred, encirclement_preferred,
                         quality_tolerance) ||
      betterTeamQuality(encirclement_preferred, visibility_preferred,
                        quality_tolerance))
  {
    std::cerr << "HIERARCHICAL_VISIBILITY_PRIORITY_TEST_FAILED\n";
    return 8;
  }

  TeamQualityVector close_angle = visibility_preferred;
  close_angle.complementarity = 0.20;
  close_angle.encirclement_cost = 0.0;
  close_angle.regularization_cost = 0.0;
  TeamQualityVector complementary = close_angle;
  complementary.complementarity = 0.90;
  complementary.encirclement_cost = 1.0e9;
  complementary.regularization_cost = 1.0e9;
  if (!betterTeamQuality(complementary, close_angle, quality_tolerance) ||
      betterTeamQuality(close_angle, complementary, quality_tolerance))
  {
    std::cerr << "HIERARCHICAL_COMPLEMENTARITY_PRIORITY_TEST_FAILED\n";
    return 9;
  }

  TeamQualityVector below_visibility_floor = complementary;
  below_visibility_floor.q2_ratio = 0.98;
  TeamQualityVector below_comp_floor = complementary;
  below_comp_floor.complementarity = 0.50;
  if (teamQualitySatisfiesFloors(
          below_visibility_floor, 0.99, 0.99, 0.40) ||
      teamQualitySatisfiesFloors(
          below_comp_floor, 0.99, 0.99, 0.80) ||
      teamPrimaryVisibilityNonWorsening(
          below_visibility_floor, visibility_preferred,
          quality_tolerance))
  {
    std::cerr << "HIERARCHICAL_STAGE_FLOOR_OR_FINAL_GUARD_FAILED\n";
    return 10;
  }

  // T finite difference: the last three variables are the per-UAV shifts.
  const double epsilon = 1.0e-5;
  for (int drone = 0; drone < 2; ++drone)
  {
    TeamTargetCenteredInput plus = input;
    TeamTargetCenteredInput minus = input;
    plus.members[drone].reference_phase_shift += epsilon;
    minus.members[drone].reference_phase_shift -= epsilon;
    const double numeric =
        (optimizer.evaluate(plus, input, environment).objective -
         optimizer.evaluate(minus, input, environment).objective) /
        (2.0 * epsilon);
    if (!near(numeric, gradient(24 + drone), 2.0e-3))
    {
      std::cerr << "TEAM_TIME_GRADIENT_FAILED drone=" << drone
                << " analytic=" << gradient(24 + drone)
                << " numeric=" << numeric << '\n';
      return 3;
    }
  }

  // Complementarity/phi finite difference with visibility fixed at one.
  TeamTargetCenteredEnvironment visible_environment = environment;
  visible_environment.visibility = [](
      int, const Eigen::Vector3d &, const Eigen::Vector3d &, double, double) {
    ContinuousVisibilitySample sample;
    sample.valid = sample.binary_valid = sample.binary_visible = true;
    sample.value = 1.0;
    return sample;
  };
  Eigen::VectorXd comp_gradient;
  optimizer.evaluate(input, input, visible_environment, &comp_gradient);
  TeamTargetCenteredInput phi_plus = input;
  TeamTargetCenteredInput phi_minus = input;
  phi_plus.members[0].knots[1].bearing += epsilon;
  phi_minus.members[0].knots[1].bearing -= epsilon;
  const double numeric_phi =
      (optimizer.evaluate(phi_plus, input, visible_environment).objective -
       optimizer.evaluate(phi_minus, input, visible_environment).objective) /
      (2.0 * epsilon);
  if (!near(numeric_phi, comp_gradient(3), 2.0e-3))
  {
    std::cerr << "TEAM_COMPLEMENTARITY_PHI_GRADIENT_FAILED analytic="
              << comp_gradient(3) << " numeric=" << numeric_phi << '\n';
    return 4;
  }

  const TeamTargetCenteredResult result = optimizer.optimize(input, environment);
  const double maximum_shift = std::max(
      std::abs(result.reference_phase_shifts[0]),
      std::abs(result.reference_phase_shifts[1]));
  if (!result.success || maximum_shift < 1.0e-4 ||
      result.after.simultaneous_los_loss_duration >=
          result.before.simultaneous_los_loss_duration - 1.0e-6)
  {
    std::cerr << "TEMPORAL_DECONFLICTION_FAILED success=" << result.success
              << " shift=" << maximum_shift
              << " tau_grad=" << gradient(24) << ',' << gradient(25)
              << " before=" << result.before.simultaneous_los_loss_duration
              << " after=" << result.after.simultaneous_los_loss_duration
              << " reason=" << result.reason << '\n';
    return 5;
  }
  if (!teamPrimaryVisibilityNonWorsening(
          result.after.quality, result.before.quality, result.tolerance))
  {
    std::cerr << "FINAL_TEAM_NON_WORSENING_GUARD_FAILED\n";
    return 11;
  }

  if (!TeamTargetCenteredOptimizer::validScheduleIdentity(result.schedule) ||
      result.schedule.target_prediction_revision !=
          input.target_prediction_revision ||
      result.schedule.members[0].topology_kind !=
          input.members[0].topology_kind)
  {
    std::cerr << "TEAM_REFERENCE_IDENTITY_CONTRACT_FAILED\n";
    return 6;
  }

  std::cout << "TARGET_CENTERED_MAPPING_PASS\n"
            << "TEAM_QUALITY_VECTOR_NORMALIZATION_PASS\n"
            << "HIERARCHICAL_VISIBILITY_PRIORITY_TEST_PASS\n"
            << "HIERARCHICAL_COMPLEMENTARITY_PRIORITY_TEST_PASS\n"
            << "STAGE2_VISIBILITY_FLOOR_PASS\n"
            << "STAGE3_VISIBILITY_COMPLEMENTARITY_FLOOR_PASS\n"
            << "FINAL_TEAM_NON_WORSENING_GUARD_PASS\n"
            << "J_COMP_PHI_FINITE_DIFFERENCE_PASS\n"
            << "J_K2_J_ACC_J_COMP_TIME_FINITE_DIFFERENCE_PASS\n"
            << "WORLD_TIME_SYNCHRONIZATION_PASS\n"
            << "TEAM_REFERENCE_REVISION_TOPOLOGY_IDENTITY_PASS\n"
            << "SIMULTANEOUS_LOS_LOSS_DURATION_BEFORE="
            << result.before.simultaneous_los_loss_duration << '\n'
            << "SIMULTANEOUS_LOS_LOSS_DURATION_AFTER="
            << result.after.simultaneous_los_loss_duration << '\n'
            << "TEAM_REFERENCE_PHASE_SHIFT="
            << result.reference_phase_shifts[0] << ','
            << result.reference_phase_shifts[1] << ','
            << result.reference_phase_shifts[2] << '\n';
  return 0;
}
