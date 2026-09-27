#ifndef MULTI_UAV_FORMATION_TRACKING_VISIBILITY_GEOMETRY_H
#define MULTI_UAV_FORMATION_TRACKING_VISIBILITY_GEOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace multi_uav_formation
{

struct TrackingCameraContract
{
  // The native benchmark's tracking camera is the body-forward synthetic
  // camera used by visibility telemetry.  body_from_camera describes the
  // fixed camera orientation; +X is forward, +Y left, and +Z up.
  double horizontal_fov{85.0 * M_PI / 180.0};
  double vertical_fov{54.53644224813771 * M_PI / 180.0};
  double min_range{0.2};
  double max_range{8.0};
  Eigen::Vector3d translation_body{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond body_from_camera{Eigen::Quaterniond::Identity()};
};

// Shared differentiable/binary visibility result used by the target-centred
// reference optimizer and by realized-team evaluation.  It is a camera/LOS
// contract value, not an executable-trajectory optimizer type.
struct ContinuousVisibilitySample
{
  bool valid{false};
  double value{0.0};
  Eigen::Vector3d gradient_position{Eigen::Vector3d::Zero()};
  double gradient_yaw{0.0};
  bool binary_valid{false};
  bool binary_visible{false};

  // Predictive Visibility Relay authority.  The scalar is deliberately
  // different from `value`: value remains the historical multiplicative
  // quality score used by the target-centred preference optimizer, whereas
  // margin is the signed hard-contract quantity
  //
  //   margin = 1 - 2 * max(r_static, r_dynamic_los, r_fov, r_range).
  //
  // A positive value therefore means that every component remains on the
  // reliable side of its risk midpoint.  Only differentiable components are
  // represented here; binary camera/LOS truth stays in binary_visible and is
  // checked during exact nonlinear validation.
  enum Limiter : std::uint8_t
  {
    LIMITER_NONE = 0,
    LIMITER_STATIC = 1,
    LIMITER_DYNAMIC_LOS = 2,
    LIMITER_FOV = 3,
    LIMITER_RANGE = 4
  };
  bool margin_valid{false};
  double margin{-std::numeric_limits<double>::infinity()};
  Eigen::Vector3d gradient_margin_position{Eigen::Vector3d::Zero()};
  double gradient_margin_yaw{0.0};
  Limiter limiter{LIMITER_NONE};
  double static_risk{0.0};
  double dynamic_los_risk{0.0};
  double fov_risk{0.0};
  double range_risk{0.0};
};

struct VisibilityRiskComponent
{
  bool valid{false};
  double risk{0.0};
  Eigen::Vector3d gradient_position{Eigen::Vector3d::Zero()};
  double gradient_yaw{0.0};
  ContinuousVisibilitySample::Limiter limiter{
      ContinuousVisibilitySample::LIMITER_NONE};
};

// Single composition rule shared by coordinator prediction, Local Team-SCP
// rows, and realized validation.  Ties are stable in STATIC -> DYNAMIC -> FOV
// -> RANGE order so telemetry and the selected subgradient are deterministic.
inline bool composeVisibilityMargin(
    const std::array<VisibilityRiskComponent, 4> &components,
    ContinuousVisibilitySample &sample)
{
  const VisibilityRiskComponent *limiting = nullptr;
  for (const auto &component : components)
  {
    if (!component.valid || !std::isfinite(component.risk) ||
        !component.gradient_position.allFinite() ||
        !std::isfinite(component.gradient_yaw))
      return false;
    if (limiting == nullptr || component.risk > limiting->risk + 1.0e-12)
      limiting = &component;
  }
  if (limiting == nullptr)
    return false;
  sample.static_risk = components[0].risk;
  sample.dynamic_los_risk = components[1].risk;
  sample.fov_risk = components[2].risk;
  sample.range_risk = components[3].risk;
  sample.margin = 1.0 - 2.0 * limiting->risk;
  sample.gradient_margin_position =
      -2.0 * limiting->gradient_position;
  sample.gradient_margin_yaw = -2.0 * limiting->gradient_yaw;
  sample.limiter = limiting->limiter;
  sample.margin_valid = std::isfinite(sample.margin) &&
                        sample.gradient_margin_position.allFinite() &&
                        std::isfinite(sample.gradient_margin_yaw);
  return sample.margin_valid;
}

inline const char *visibilityMarginLimiterName(
    const ContinuousVisibilitySample::Limiter limiter)
{
  switch (limiter)
  {
  case ContinuousVisibilitySample::LIMITER_STATIC: return "STATIC";
  case ContinuousVisibilitySample::LIMITER_DYNAMIC_LOS: return "DYNAMIC_LOS";
  case ContinuousVisibilitySample::LIMITER_FOV: return "FOV";
  case ContinuousVisibilitySample::LIMITER_RANGE: return "RANGE";
  default: return "NONE";
  }
}

inline std::uint64_t trackingCameraContractVersion(
    const TrackingCameraContract &camera)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](const double value) {
    const std::int64_t quantized = static_cast<std::int64_t>(
        std::llround(value * 1.0e9));
    hash ^= static_cast<std::uint64_t>(quantized);
    hash *= 1099511628211ULL;
  };
  mix(camera.horizontal_fov);
  mix(camera.vertical_fov);
  mix(camera.min_range);
  mix(camera.max_range);
  for (int axis = 0; axis < 3; ++axis)
    mix(camera.translation_body(axis));
  mix(camera.body_from_camera.w());
  mix(camera.body_from_camera.x());
  mix(camera.body_from_camera.y());
  mix(camera.body_from_camera.z());
  return hash == 0 ? 1 : hash;
}

struct DynamicLosCylinder
{
  std::string name;
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  Eigen::Vector2d axis{Eigen::Vector2d::UnitX()};
  double amplitude{0.0};
  double period{1.0};
  double phase{0.0};
  double radius{0.0};
  double z_min{0.0};
  double z_max{0.0};

  Eigen::Vector2d centerAt(const double elapsed) const
  {
    const double omega = 2.0 * M_PI / std::max(period, 1.0e-6);
    return center + axis * amplitude * std::sin(omega * elapsed + phase);
  }
};

struct PredictedYawState
{
  bool valid{false};
  double yaw{0.0};
  double yaw_rate{0.0};
};

struct CameraFovResult
{
  bool valid{false};
  bool in_front{false};
  double range{std::numeric_limits<double>::quiet_NaN()};
  double horizontal_angle{std::numeric_limits<double>::quiet_NaN()};
  double vertical_angle{std::numeric_limits<double>::quiet_NaN()};
};

// A differentiable live visibility penalty amplitude.  The frozen Elastic
// risk remains clamped to [0, 1].  Only the live directional objective uses
// the deep-risk continuation below the inner clearance.  Its slope grows
// monotonically from zero and approaches -deep_kappa / width, so deeply
// blocked rays retain an escape direction instead of losing their gradient.
struct DirectionalRiskResult
{
  bool valid{false};
  double value{0.0};
  double derivative_clearance{0.0};
};

inline DirectionalRiskResult directionalClearanceRisk(
    const double clearance, const double inner_clearance,
    const double outer_clearance, const double deep_kappa = 1.0,
    const double deep_beta = 2.0)
{
  DirectionalRiskResult result;
  if (!std::isfinite(clearance) || !std::isfinite(inner_clearance) ||
      !std::isfinite(outer_clearance) || !std::isfinite(deep_kappa) ||
      !std::isfinite(deep_beta) || deep_kappa < 0.0 || deep_beta <= 0.0 ||
      outer_clearance <= inner_clearance + 1.0e-12)
    return result;
  result.valid = true;
  const double width = outer_clearance - inner_clearance;
  if (clearance >= outer_clearance)
    return result;
  if (clearance <= inner_clearance)
  {
    const double d = (inner_clearance - clearance) / width;
    // expm1 keeps both R and dR/dc accurate as d -> 0.
    const double one_minus_attenuation = -std::expm1(-deep_beta * d);
    result.value = 1.0 + deep_kappa *
                             (d - one_minus_attenuation / deep_beta);
    result.derivative_clearance =
        -deep_kappa * one_minus_attenuation / width;
    return result;
  }
  const double u = (outer_clearance - clearance) / width;
  result.value = u * u * (3.0 - 2.0 * u);
  result.derivative_clearance = -6.0 * u * (1.0 - u) / width;
  return result;
}

struct SegmentCylinderClearanceGradient
{
  bool valid{false};
  double clearance{std::numeric_limits<double>::infinity()};
  double segment_parameter{0.0};
  Eigen::Vector3d gradient_start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gradient_end{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gradient_center{Eigen::Vector3d::Zero()};
};

struct CameraFovDirectionalRisk
{
  bool valid{false};
  double range{std::numeric_limits<double>::quiet_NaN()};
  double horizontal_angle{std::numeric_limits<double>::quiet_NaN()};
  double vertical_angle{std::numeric_limits<double>::quiet_NaN()};
  double horizontal_margin{std::numeric_limits<double>::quiet_NaN()};
  double vertical_margin{std::numeric_limits<double>::quiet_NaN()};
  double horizontal_risk{0.0};
  double vertical_risk{0.0};
  double risk{0.0};
  Eigen::Vector3d gradient_body_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gradient_target_position{Eigen::Vector3d::Zero()};
  double gradient_yaw{0.0};
};

// Shared smooth activation used by both trajectory-level visibility and the
// frozen Elastic risk profile.  A clearance at/below inner_clearance is fully
// risky, while a clearance at/above outer_clearance is inactive.
inline double smoothClearanceRisk(const double clearance,
                                  const double inner_clearance,
                                  const double outer_clearance)
{
  if (!std::isfinite(clearance) || !std::isfinite(inner_clearance) ||
      !std::isfinite(outer_clearance) ||
      outer_clearance <= inner_clearance + 1.0e-12)
    return 0.0;
  const double u = std::max(
      0.0, std::min(1.0, (outer_clearance - clearance) /
                             (outer_clearance - inner_clearance)));
  return u * u * (3.0 - 2.0 * u);
}

inline double verticalFovFromHorizontal(const double horizontal_fov,
                                        const double image_width,
                                        const double image_height)
{
  if (!std::isfinite(horizontal_fov) || !std::isfinite(image_width) ||
      !std::isfinite(image_height) || image_width <= 0.0 ||
      image_height <= 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  return 2.0 * std::atan(
      std::tan(0.5 * horizontal_fov) * image_height / image_width);
}

inline CameraFovResult targetInTrackingCameraFov(
    const Eigen::Vector3d &body_position,
    const Eigen::Quaterniond &world_from_body_raw,
    const Eigen::Vector3d &target_position,
    const TrackingCameraContract &camera)
{
  CameraFovResult result;
  if (!body_position.allFinite() || !target_position.allFinite() ||
      !world_from_body_raw.coeffs().allFinite() ||
      !camera.translation_body.allFinite() ||
      !camera.body_from_camera.coeffs().allFinite() ||
      !std::isfinite(camera.horizontal_fov) ||
      !std::isfinite(camera.vertical_fov) ||
      !std::isfinite(camera.min_range) || !std::isfinite(camera.max_range))
    return result;

  Eigen::Quaterniond world_from_body = world_from_body_raw;
  Eigen::Quaterniond body_from_camera = camera.body_from_camera;
  if (world_from_body.norm() <= 1.0e-12 ||
      body_from_camera.norm() <= 1.0e-12)
    return result;
  world_from_body.normalize();
  body_from_camera.normalize();
  const Eigen::Vector3d camera_position =
      body_position + world_from_body * camera.translation_body;
  const Eigen::Quaterniond world_from_camera =
      world_from_body * body_from_camera;
  const Eigen::Vector3d relative_camera =
      world_from_camera.conjugate() * (target_position - camera_position);
  result.range = relative_camera.norm();
  if (!relative_camera.allFinite() || !std::isfinite(result.range) ||
      result.range <= 1.0e-12)
    return result;
  result.in_front = relative_camera.x() > 0.0;
  result.horizontal_angle = std::atan2(relative_camera.y(),
                                       relative_camera.x());
  result.vertical_angle = std::atan2(
      relative_camera.z(), relative_camera.head<2>().norm());
  result.valid = result.in_front && result.range >= camera.min_range &&
                 result.range <= camera.max_range &&
                 std::abs(result.horizontal_angle) <=
                     0.5 * camera.horizontal_fov + 1.0e-12 &&
                 std::abs(result.vertical_angle) <=
                     0.5 * camera.vertical_fov + 1.0e-12;
  return result;
}

// Soft angular FOV risk under exactly the same camera frame convention as
// targetInTrackingCameraFov().  Range is intentionally not part of this term:
// Elastic already owns radial tracking, while this risk only releases a
// visually poor bearing.  The outer quarter of either half-FOV is a smooth
// transition band; outside the hard FOV is fully risky.
inline double trackingCameraFovRisk(
    const Eigen::Vector3d &body_position,
    const Eigen::Quaterniond &world_from_body,
    const Eigen::Vector3d &target_position,
    const TrackingCameraContract &camera)
{
  const CameraFovResult fov = targetInTrackingCameraFov(
      body_position, world_from_body, target_position, camera);
  if (!fov.in_front || !std::isfinite(fov.horizontal_angle) ||
      !std::isfinite(fov.vertical_angle) ||
      !std::isfinite(camera.horizontal_fov) ||
      !std::isfinite(camera.vertical_fov) ||
      camera.horizontal_fov <= 1.0e-9 || camera.vertical_fov <= 1.0e-9)
    return 1.0;

  const double horizontal_half = 0.5 * camera.horizontal_fov;
  const double vertical_half = 0.5 * camera.vertical_fov;
  const double horizontal_margin =
      horizontal_half - std::abs(fov.horizontal_angle);
  const double vertical_margin =
      vertical_half - std::abs(fov.vertical_angle);
  const double horizontal_risk = smoothClearanceRisk(
      horizontal_margin, 0.0, 0.25 * horizontal_half);
  const double vertical_risk = smoothClearanceRisk(
      vertical_margin, 0.0, 0.25 * vertical_half);
  return std::max(horizontal_risk, vertical_risk);
}

// Live FOV penalty and analytic derivatives with yaw held as the established
// predicted-yaw state (yaw is not an optimization variable).  The angular
// transition band matches trackingCameraFovRisk().  Outside the hard FOV the
// live amplitude continues quadratically, preserving a directional gradient.
inline CameraFovDirectionalRisk trackingCameraFovDirectionalRisk(
    const Eigen::Vector3d &body_position, const double predicted_yaw,
    const Eigen::Vector3d &target_position,
    const TrackingCameraContract &camera)
{
  CameraFovDirectionalRisk result;
  if (!body_position.allFinite() || !target_position.allFinite() ||
      !std::isfinite(predicted_yaw) ||
      !camera.body_from_camera.coeffs().allFinite() ||
      !std::isfinite(camera.horizontal_fov) ||
      !std::isfinite(camera.vertical_fov) ||
      !std::isfinite(camera.min_range) ||
      !std::isfinite(camera.max_range) ||
      camera.body_from_camera.norm() <= 1.0e-12 ||
      camera.horizontal_fov <= 1.0e-9 || camera.vertical_fov <= 1.0e-9 ||
      camera.min_range < 0.0 || camera.max_range <= camera.min_range)
    return result;

  Eigen::Quaterniond body_from_camera = camera.body_from_camera;
  body_from_camera.normalize();
  const Eigen::Matrix3d camera_from_world =
      (Eigen::Quaterniond(Eigen::AngleAxisd(
           predicted_yaw, Eigen::Vector3d::UnitZ())) *
       body_from_camera)
          .conjugate()
          .toRotationMatrix();
  const Eigen::Vector3d relative =
      camera_from_world * (target_position - body_position);
  const double horizontal_sq = relative.head<2>().squaredNorm();
  const double horizontal_norm = std::sqrt(horizontal_sq);
  const double range_sq = horizontal_sq + relative.z() * relative.z();
  const double range = std::sqrt(range_sq);
  // FOV and range are separate visibility components.  In particular, being
  // outside the camera's reliable range does not make azimuth/elevation
  // undefined: RANGE owns that signed margin while FOV must keep providing
  // its bearing derivative.  Only true coincidence or an almost vertical
  // bearing is singular for the angular Jacobian.
  if (!relative.allFinite() || !std::isfinite(range) ||
      horizontal_norm <= 1.0e-9 || range_sq <= 1.0e-12)
    return result;

  const double horizontal_angle = std::atan2(relative.y(), relative.x());
  const double vertical_angle = std::atan2(relative.z(), horizontal_norm);
  const double horizontal_half = 0.5 * camera.horizontal_fov;
  const double vertical_half = 0.5 * camera.vertical_fov;
  const double horizontal_margin =
      horizontal_half - std::abs(horizontal_angle);
  const double vertical_margin = vertical_half - std::abs(vertical_angle);
  const DirectionalRiskResult horizontal = directionalClearanceRisk(
      horizontal_margin, 0.0,
      0.25 * horizontal_half);
  const DirectionalRiskResult vertical = directionalClearanceRisk(
      vertical_margin, 0.0,
      0.25 * vertical_half);
  if (!horizontal.valid || !vertical.valid)
    return result;

  Eigen::Vector3d grad_horizontal_angle_camera(
      -relative.y() / horizontal_sq, relative.x() / horizontal_sq, 0.0);
  Eigen::Vector3d grad_vertical_angle_camera;
  grad_vertical_angle_camera <<
                                      -relative.z() * relative.x() /
                                     (range_sq * horizontal_norm),
      -relative.z() * relative.y() / (range_sq * horizontal_norm),
      horizontal_norm / range_sq;
  const double horizontal_sign = horizontal_angle >= 0.0 ? 1.0 : -1.0;
  const double vertical_sign = vertical_angle >= 0.0 ? 1.0 : -1.0;
  const Eigen::Vector3d grad_horizontal_relative =
      horizontal.derivative_clearance * (-horizontal_sign) *
      grad_horizontal_angle_camera;
  const Eigen::Vector3d grad_vertical_relative =
      vertical.derivative_clearance * (-vertical_sign) *
      grad_vertical_angle_camera;

  result.valid = true;
  result.range = range;
  result.horizontal_angle = horizontal_angle;
  result.vertical_angle = vertical_angle;
  result.horizontal_margin = horizontal_margin;
  result.vertical_margin = vertical_margin;
  result.horizontal_risk = horizontal.value;
  result.vertical_risk = vertical.value;
  const bool horizontal_active = horizontal.value >= vertical.value;
  result.risk = horizontal_active ? horizontal.value : vertical.value;
  const Eigen::Vector3d grad_relative = horizontal_active
                                            ? grad_horizontal_relative
                                            : grad_vertical_relative;
  result.gradient_target_position =
      camera_from_world.transpose() * grad_relative;
  result.gradient_body_position = -result.gradient_target_position;
  // Horizontal bearing is bearing_world-yaw; vertical bearing is invariant
  // under a pure world-Z yaw.
  result.gradient_yaw = horizontal_active
                            ? -horizontal.derivative_clearance *
                                  (-horizontal_sign)
                            : 0.0;
  return result;
}

// Minimum horizontal clearance between a 3-D segment and a finite vertical
// cylinder over the segment portion that overlaps the cylinder's height.
// This is the clearance counterpart of segmentIntersectsVerticalCylinder()
// and uses the same center/height convention as production moving markers.
inline bool segmentVerticalCylinderClearance(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const Eigen::Vector3d &center, const double radius,
    const double height, double &clearance)
{
  clearance = std::numeric_limits<double>::infinity();
  if (!start.allFinite() || !end.allFinite() || !center.allFinite() ||
      !std::isfinite(radius) || !std::isfinite(height) ||
      radius < 0.0 || height <= 0.0)
    return false;

  const Eigen::Vector3d delta = end - start;
  const double z_min = center.z() - 0.5 * height;
  const double z_max = center.z() + 0.5 * height;
  double first = 0.0;
  double last = 1.0;
  if (std::abs(delta.z()) <= 1.0e-12)
  {
    if (start.z() < z_min || start.z() > z_max)
      return true;
  }
  else
  {
    const double first_crossing = (z_min - start.z()) / delta.z();
    const double second_crossing = (z_max - start.z()) / delta.z();
    first = std::max(0.0, std::min(first_crossing, second_crossing));
    last = std::min(1.0, std::max(first_crossing, second_crossing));
    if (first > last)
      return true;
  }

  const Eigen::Vector2d start_relative = start.head<2>() - center.head<2>();
  const Eigen::Vector2d delta_xy = delta.head<2>();
  double closest_parameter = first;
  if (delta_xy.squaredNorm() > 1.0e-12)
  {
    closest_parameter = -start_relative.dot(delta_xy) /
                        delta_xy.squaredNorm();
    closest_parameter = std::max(first, std::min(last, closest_parameter));
  }
  clearance =
      (start_relative + closest_parameter * delta_xy).norm() - radius;
  return std::isfinite(clearance);
}

inline SegmentCylinderClearanceGradient
segmentVerticalCylinderClearanceGradient(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const Eigen::Vector3d &center, const double radius,
    const double height)
{
  SegmentCylinderClearanceGradient result;
  if (!start.allFinite() || !end.allFinite() || !center.allFinite() ||
      !std::isfinite(radius) || !std::isfinite(height) || radius < 0.0 ||
      height <= 0.0)
    return result;

  const Eigen::Vector3d delta = end - start;
  const double z_min = center.z() - 0.5 * height;
  const double z_max = center.z() + 0.5 * height;
  double first = 0.0;
  double last = 1.0;
  if (std::abs(delta.z()) <= 1.0e-12)
  {
    if (start.z() < z_min || start.z() > z_max)
    {
      result.valid = true;
      return result;
    }
  }
  else
  {
    const double first_crossing = (z_min - start.z()) / delta.z();
    const double second_crossing = (z_max - start.z()) / delta.z();
    first = std::max(0.0, std::min(first_crossing, second_crossing));
    last = std::min(1.0, std::max(first_crossing, second_crossing));
    if (first > last)
    {
      result.valid = true;
      return result;
    }
  }

  const Eigen::Vector2d start_relative =
      start.head<2>() - center.head<2>();
  const Eigen::Vector2d delta_xy = delta.head<2>();
  double parameter = first;
  if (delta_xy.squaredNorm() > 1.0e-12)
  {
    parameter = -start_relative.dot(delta_xy) / delta_xy.squaredNorm();
    parameter = std::max(first, std::min(last, parameter));
  }
  const Eigen::Vector2d radial =
      start_relative + parameter * delta_xy;
  const double radial_norm = radial.norm();
  if (radial_norm <= 1.0e-9)
    return result; // no unique direction: fail open instead of inventing one
  const Eigen::Vector2d normal = radial / radial_norm;
  result.valid = true;
  result.clearance = radial_norm - radius;
  result.segment_parameter = parameter;
  result.gradient_start.head<2>() = (1.0 - parameter) * normal;
  result.gradient_end.head<2>() = parameter * normal;
  result.gradient_center.head<2>() = -normal;
  return result;
}

// Encirclement only needs enough angular spread to preserve multiple
// viewpoints.  Once the minimum pairwise angle reaches the preference, larger
// angles (90, 120, ...) receive no additional reward.
inline double spreadPreferenceScore(const double minimum_pairwise_angle_deg,
                                    const double saturation_angle_deg = 60.0)
{
  if (!std::isfinite(minimum_pairwise_angle_deg) ||
      !std::isfinite(saturation_angle_deg) || saturation_angle_deg <= 0.0)
    return 0.0;
  return std::max(0.0, std::min(
      1.0, minimum_pairwise_angle_deg / saturation_angle_deg));
}

inline bool segmentIntersectsVerticalCylinder(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const Eigen::Vector2d &center, const double radius,
    const double z_min, const double z_max)
{
  if (!start.allFinite() || !end.allFinite() || !center.allFinite() ||
      !std::isfinite(radius) || !std::isfinite(z_min) ||
      !std::isfinite(z_max) || radius < 0.0 || z_max < z_min)
    return true;
  const Eigen::Vector3d delta = end - start;
  const Eigen::Vector2d start_relative = start.head<2>() - center;
  const double radial_a = delta.head<2>().squaredNorm();
  const double radial_b = 2.0 * start_relative.dot(delta.head<2>());
  const double radial_c = start_relative.squaredNorm() - radius * radius;
  double radial_first = 0.0;
  double radial_last = 1.0;
  if (radial_a <= 1.0e-12)
  {
    if (radial_c > 0.0)
      return false;
  }
  else
  {
    const double discriminant =
        radial_b * radial_b - 4.0 * radial_a * radial_c;
    if (discriminant < 0.0)
      return false;
    const double root = std::sqrt(std::max(0.0, discriminant));
    radial_first = std::max(
        0.0, (-radial_b - root) / (2.0 * radial_a));
    radial_last = std::min(
        1.0, (-radial_b + root) / (2.0 * radial_a));
    if (radial_first > radial_last)
      return false;
  }

  double height_first = 0.0;
  double height_last = 1.0;
  if (std::abs(delta.z()) <= 1.0e-12)
  {
    if (start.z() < z_min || start.z() > z_max)
      return false;
  }
  else
  {
    const double first = (z_min - start.z()) / delta.z();
    const double second = (z_max - start.z()) / delta.z();
    height_first = std::max(0.0, std::min(first, second));
    height_last = std::min(1.0, std::max(first, second));
    if (height_first > height_last)
      return false;
  }
  return std::max(radial_first, height_first) <=
         std::min(radial_last, height_last) + 1.0e-12;
}

inline PredictedYawState advanceTargetFacingYaw(
    const PredictedYawState &input, const double desired_yaw,
    const double dt, const double max_yaw_rate = 2.0 * M_PI,
    const double max_yaw_acceleration = 5.0 * M_PI)
{
  PredictedYawState result = input;
  if (!input.valid || !std::isfinite(input.yaw) ||
      !std::isfinite(input.yaw_rate) || !std::isfinite(desired_yaw) ||
      !std::isfinite(dt) || dt <= 1.0e-6)
    return result;
  double difference = desired_yaw - input.yaw;
  while (difference > M_PI)
    difference -= 2.0 * M_PI;
  while (difference < -M_PI)
    difference += 2.0 * M_PI;
  const double sign = difference >= 0.0 ? 1.0 : -1.0;
  const double rate_limit = sign * max_yaw_rate;
  const double acceleration_limit = sign * max_yaw_acceleration;
  double max_step = 0.0;
  double endpoint_rate = input.yaw_rate;
  if (std::abs(input.yaw_rate + dt * acceleration_limit) <= max_yaw_rate)
  {
    max_step = input.yaw_rate * dt +
               0.5 * acceleration_limit * dt * dt;
    endpoint_rate = input.yaw_rate + dt * acceleration_limit;
  }
  else
  {
    const double acceleration_time =
        (rate_limit - input.yaw_rate) / acceleration_limit;
    // Integrate the existing rate as well as its acceleration increment.
    // At input.yaw_rate == rate_limit the old increment-only expression was
    // zero, freezing the heading forever while still reporting a nonzero rate.
    max_step = input.yaw_rate * dt + ((dt - acceleration_time) + dt) *
               (rate_limit - input.yaw_rate) / 2.0;
    endpoint_rate = rate_limit;
  }
  const bool target_reached =
      std::abs(difference) <= std::abs(max_step);
  if (!target_reached)
    difference = max_step;
  result.yaw = input.yaw + difference;
  while (result.yaw > M_PI)
    result.yaw -= 2.0 * M_PI;
  while (result.yaw < -M_PI)
    result.yaw += 2.0 * M_PI;
  // This state is fed into the next prediction step and is also used as the
  // analytic d(yaw)/dt term by the visibility objective.  The previous
  // difference/dt value was an average rate; during acceleration it was only
  // half the endpoint derivative and made the FOV duration gradient wrong.
  result.yaw_rate = target_reached ? 0.0 : endpoint_rate;
  return result;
}

} // namespace multi_uav_formation

#endif
