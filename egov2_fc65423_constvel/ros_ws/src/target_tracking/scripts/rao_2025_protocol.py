#!/usr/bin/env python3
"""Validate and publish the immutable Rao 2025 experiment protocol."""

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile

import yaml


SCHEMA_VERSION = "rao_2025_protocol/v1"
PROFILES = ("simulation", "comparison")


class ProtocolError(ValueError):
    pass


def _require_equal(actual, expected, label):
    if actual != expected:
        raise ProtocolError("{} must be {!r}, got {!r}".format(label, expected, actual))


def _require_finite_positive(value, label):
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
        raise ProtocolError("{} must be a finite positive number".format(label))


def _require_positive_integer(value, label):
    if type(value) is not int or value <= 0:
        raise ProtocolError("{} must be a positive integer".format(label))


def load_protocol(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ProtocolError("protocol root must be a mapping")
    return data


def validate_protocol(data, profile):
    if profile not in PROFILES:
        raise ProtocolError("profile must be one of: {}".format(", ".join(PROFILES)))
    _require_equal(data.get("schema_version"), SCHEMA_VERSION, "schema_version")

    paper = data.get("paper", {})
    algorithm = paper.get("algorithm", {})
    common = paper.get("common_parameters", {})
    profiles = paper.get("profiles", {})
    reproduction = data.get("reproduction", {})
    timing = reproduction.get("timing", {})
    replanning = reproduction.get("replanning", {})
    metrics = reproduction.get("metrics", {})
    scene_generation = reproduction.get("scene_generation", {})
    target = reproduction.get("target", {})

    _require_equal(algorithm.get("uav_count"), 3, "paper.algorithm.uav_count")
    _require_equal(algorithm.get("target_prediction"), "constant_velocity_ekf", "paper.algorithm.target_prediction")
    _require_equal(algorithm.get("target_motion_model"), "bicycle", "paper.algorithm.target_motion_model")
    _require_equal(algorithm.get("target_path_planner"), "kinodynamic_hybrid_a_star", "paper.algorithm.target_path_planner")
    _require_equal(algorithm.get("corridor"), "ellipsoid_dilation_convex_polyhedra", "paper.algorithm.corridor")
    _require_equal(algorithm.get("trajectory"), "minco_position_and_yaw", "paper.algorithm.trajectory")
    _require_equal(algorithm.get("distributed_per_uav"), True, "paper.algorithm.distributed_per_uav")

    expected_common = {
        "initial_uav_positions_m": [[0.75, 1.4, 1.0], [-1.5, 0.0, 1.0], [0.75, -1.4, 1.0]],
        "camera_fov_deg": [85.0, 72.0],
        "uav_max_velocity_mps": 3.0,
        "uav_max_acceleration_mps2": 6.0,
        "uav_max_jerk_mps3": 20.0,
        "target_max_velocity_mps": 2.2,
        "cube_width_range_m": [0.3, 0.8],
        "cube_height_range_m": [2.0, 3.0],
        "annulus_diameter_range_m": [0.5, 0.7],
        "annulus_tilt_rad": [-0.5, 0.5],
    }
    for key, expected in expected_common.items():
        _require_equal(common.get(key), expected, "paper.common_parameters.{}".format(key))

    expected_profiles = {
        "simulation": ([65.0, 30.0, 4.0], 140, "4.1"),
        "comparison": ([50.0, 30.0, 5.0], 155, "4.2"),
    }
    for name, (size, count, section) in expected_profiles.items():
        current = profiles.get(name, {})
        _require_equal(current.get("scene_size_m"), size, "paper.profiles.{}.scene_size_m".format(name))
        _require_equal(current.get("obstacle_count"), count, "paper.profiles.{}.obstacle_count".format(name))
        _require_equal(current.get("paper_section"), section, "paper.profiles.{}.paper_section".format(name))

    reference = profiles.get("comparison", {}).get("reference_visibility_percent", {})
    _require_equal(reference.get("proposed"),
                   {"visible_3": 57.6, "visible_2": 32.2, "visible_1": 10.2, "visible_0": 0.0},
                   "paper.profiles.comparison.reference_visibility_percent.proposed")
    _require_equal(reference.get("zhou"),
                   {"visible_3": 10.8, "visible_2": 8.2, "visible_1": 67.2, "visible_0": 13.8},
                   "paper.profiles.comparison.reference_visibility_percent.zhou")

    _require_equal(reproduction.get("seed_derivation"), "sha256_31bit_v1", "reproduction.seed_derivation")
    if not isinstance(reproduction.get("master_seed"), int):
        raise ProtocolError("reproduction.master_seed must be an integer")
    streams = reproduction.get("deterministic_streams")
    _require_equal(streams, ["scene", "target_planner", "sensor_uav1", "sensor_uav2", "sensor_uav3"],
                   "reproduction.deterministic_streams")

    for key in ("control_rate_hz", "prediction_interval_s", "evaluation_duration_s"):
        _require_finite_positive(timing.get(key), "reproduction.timing.{}".format(key))
    _require_positive_integer(timing.get("prediction_count"), "reproduction.timing.prediction_count")
    if timing.get("warmup_s") != 0.0:
        raise ProtocolError("reproduction.timing.warmup_s must be 0.0 for the declared all-sample metric protocol")

    _require_equal(replanning.get("trigger"), "fixed_period", "reproduction.replanning.trigger")
    _require_finite_positive(replanning.get("period_s"), "reproduction.replanning.period_s")
    _require_equal(replanning.get("concurrent_plans_per_uav"), 1, "reproduction.replanning.concurrent_plans_per_uav")
    _require_equal(replanning.get("tick_while_busy"), "skip", "reproduction.replanning.tick_while_busy")
    _require_equal(replanning.get("success_action"), "replace_at_next_control_tick", "reproduction.replanning.success_action")
    _require_equal(replanning.get("failure_action"), "continue_last_valid_trajectory_until_expiry_then_hold",
                   "reproduction.replanning.failure_action")
    _require_equal(replanning.get("target_update_triggers_immediate_replan"), False,
                   "reproduction.replanning.target_update_triggers_immediate_replan")
    _require_equal(replanning.get("execution_safety_projection"), False,
                   "reproduction.replanning.execution_safety_projection")

    _require_equal(metrics.get("sample_rate_hz"), timing.get("control_rate_hz"),
                   "reproduction.metrics.sample_rate_hz")
    _require_equal(metrics.get("visibility_source"), "executed_pose_sensor_geometry_before_sharing",
                   "reproduction.metrics.visibility_source")
    _require_equal(metrics.get("visibility_buckets"), ["visible_3", "visible_2", "visible_1", "visible_0"],
                   "reproduction.metrics.visibility_buckets")
    _require_equal(metrics.get("visibility_denominator"), "all_evaluation_samples",
                   "reproduction.metrics.visibility_denominator")
    for name, reference_values in reference.items():
        if not isinstance(reference_values, dict) or abs(sum(reference_values.values()) - 100.0) > 1e-9:
            raise ProtocolError("comparison reference percentages for {} must sum to 100".format(name))

    _require_finite_positive(scene_generation.get("annulus_fraction"),
                             "reproduction.scene_generation.annulus_fraction")
    if scene_generation["annulus_fraction"] >= 1.0:
        raise ProtocolError("reproduction.scene_generation.annulus_fraction must be below 1")
    _require_finite_positive(scene_generation.get("annulus_wall_thickness_m"),
                             "reproduction.scene_generation.annulus_wall_thickness_m")
    _require_equal(scene_generation.get("annulus_model"), "vertical_ring_axis_in_xy",
                   "reproduction.scene_generation.annulus_model")
    _require_finite_positive(scene_generation.get("annulus_center_margin_m"),
                             "reproduction.scene_generation.annulus_center_margin_m")
    _require_finite_positive(target.get("altitude_m"), "reproduction.target.altitude_m")
    _require_finite_positive(target.get("wheelbase_m"), "reproduction.target.wheelbase_m")
    _require_finite_positive(target.get("collision_clearance_m"),
                             "reproduction.target.collision_clearance_m")
    _require_finite_positive(target.get("goal_tolerance_m"), "reproduction.target.goal_tolerance_m")
    _require_finite_positive(target.get("action_dt_s"), "reproduction.target.action_dt_s")
    _require_positive_integer(target.get("max_expansions"), "reproduction.target.max_expansions")
    _require_finite_positive(target.get("grid_resolution_m"), "reproduction.target.grid_resolution_m")
    _require_finite_positive(target.get("yaw_resolution_deg"), "reproduction.target.yaw_resolution_deg")
    _require_finite_positive(target.get("speed_resolution_mps"), "reproduction.target.speed_resolution_mps")
    _require_finite_positive(target.get("steering_cost_weight"), "reproduction.target.steering_cost_weight")
    _require_finite_positive(target.get("acceleration_cost_weight"),
                             "reproduction.target.acceleration_cost_weight")
    control_dt = 1.0 / float(timing["control_rate_hz"])
    primitive_steps = float(target["action_dt_s"]) / control_dt
    if abs(primitive_steps - round(primitive_steps)) > 1e-9:
        raise ProtocolError("reproduction.target.action_dt_s must be an integer multiple of the control period")
    if len(target.get("steering_set_rad", [])) < 3 or len(target.get("acceleration_set_mps2", [])) < 3:
        raise ProtocolError("reproduction.target action sets must each contain at least three values")
    for name in PROFILES:
        initial = target.get("initial_state", {}).get(name)
        waypoints = target.get("waypoints", {}).get(name)
        if not isinstance(initial, list) or len(initial) != 8:
            raise ProtocolError("reproduction.target.initial_state.{} must contain 8 values".format(name))
        if not isinstance(waypoints, list) or len(waypoints) < 1 or any(len(point) != 2 for point in waypoints):
            raise ProtocolError("reproduction.target.waypoints.{} must contain 2-D points".format(name))
    return True


def derive_seed(master_seed, profile, stream):
    material = "{}:{}:{}".format(master_seed, profile, stream).encode("ascii")
    return int(hashlib.sha256(material).hexdigest()[:8], 16) & 0x7FFFFFFF


def build_manifest(data, profile, config_path):
    validate_protocol(data, profile)
    with open(config_path, "rb") as stream:
        config_sha256 = hashlib.sha256(stream.read()).hexdigest()
    reproduction = data["reproduction"]
    paper = data["paper"]
    streams = reproduction["deterministic_streams"]
    manifest = {
        "schema_version": data["schema_version"],
        "profile": profile,
        "paper_section": paper["profiles"][profile]["paper_section"],
        "config_path": os.path.abspath(config_path),
        "config_sha256": config_sha256,
        "paper_algorithm": paper["algorithm"],
        "paper_common_parameters": paper["common_parameters"],
        "paper_scene": paper["profiles"][profile],
        "paper_unreported_parameters": paper["unreported_parameters"],
        "reproduction_timing": reproduction["timing"],
        "scene_generation": reproduction["scene_generation"],
        "target_generation": reproduction["target"],
        "replanning_contract": reproduction["replanning"],
        "metric_protocol": reproduction["metrics"],
        "master_seed": reproduction["master_seed"],
        "derived_seeds": {
            stream: derive_seed(reproduction["master_seed"], profile, stream) for stream in streams
        },
    }
    return manifest


def validate_manifest(manifest):
    if not isinstance(manifest, dict):
        raise ProtocolError("manifest root must be a mapping")
    _require_equal(manifest.get("schema_version"), SCHEMA_VERSION, "manifest.schema_version")
    profile = manifest.get("profile")
    if profile not in PROFILES:
        raise ProtocolError("manifest.profile must be one of: {}".format(", ".join(PROFILES)))
    config_sha256 = manifest.get("config_sha256")
    if not isinstance(config_sha256, str) or len(config_sha256) != 64:
        raise ProtocolError("manifest.config_sha256 must be a SHA-256 digest")
    scene = manifest.get("paper_scene", {})
    expected = {"simulation": ([65.0, 30.0, 4.0], 140), "comparison": ([50.0, 30.0, 5.0], 155)}
    _require_equal(scene.get("scene_size_m"), expected[profile][0], "manifest.paper_scene.scene_size_m")
    _require_equal(scene.get("obstacle_count"), expected[profile][1], "manifest.paper_scene.obstacle_count")
    for key in ("paper_common_parameters", "scene_generation", "target_generation", "reproduction_timing"):
        if not isinstance(manifest.get(key), dict):
            raise ProtocolError("manifest.{} must be a mapping".format(key))
    return True


def write_manifest(path, manifest):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix="rao_manifest_", suffix=".json", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    except Exception:
        if os.path.exists(temporary):
            os.unlink(temporary)
        raise


class VisibilityAccumulator:
    """Table 1 visibility buckets sampled from executed sensor geometry."""

    def __init__(self, uav_count=3):
        self.uav_count = uav_count
        self.samples = 0
        self.counts = [0] * (uav_count + 1)

    def add(self, visible):
        if len(visible) != self.uav_count or any(type(value) is not bool for value in visible):
            raise ProtocolError("visibility sample must contain exactly {} booleans".format(self.uav_count))
        self.counts[sum(visible)] += 1
        self.samples += 1

    def percentages(self):
        if self.samples == 0:
            return {"visible_{}".format(i): 0.0 for i in range(self.uav_count, -1, -1)}
        return {
            "visible_{}".format(i): 100.0 * self.counts[i] / self.samples
            for i in range(self.uav_count, -1, -1)
        }


def run_ros_node(config_path, profile, manifest_path):
    import rospy
    from std_msgs.msg import Bool, String

    manifest = build_manifest(load_protocol(config_path), profile, config_path)
    validate_manifest(manifest)
    write_manifest(manifest_path, manifest)
    manifest_json = json.dumps(manifest, sort_keys=True)
    manifest_pub = rospy.Publisher("~manifest", String, queue_size=1, latch=True)
    ready_pub = rospy.Publisher("~ready", Bool, queue_size=1, latch=True)
    manifest_pub.publish(String(data=manifest_json))
    ready_pub.publish(Bool(data=True))
    rospy.set_param("~selected_profile", profile)
    rospy.set_param("~config_sha256", manifest["config_sha256"])
    rospy.loginfo("Rao protocol ready: profile=%s section=%s scene=%s obstacles=%d manifest=%s",
                  profile, manifest["paper_section"], manifest["paper_scene"]["scene_size_m"],
                  manifest["paper_scene"]["obstacle_count"], manifest_path)
    rospy.spin()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--profile", choices=PROFILES, default="simulation")
    parser.add_argument("--manifest", default="/tmp/rao_2025_manifest.json")
    parser.add_argument("--check", action="store_true", help="validate, write the manifest, then exit")
    # roslaunch appends remapping arguments such as __name:= and __log:=.
    cli_args = [argument for argument in sys.argv[1:] if ":=" not in argument]
    args = parser.parse_args(cli_args)
    try:
        if args.check:
            manifest = build_manifest(load_protocol(args.config), args.profile, args.config)
            write_manifest(args.manifest, manifest)
            print(json.dumps({
                "status": "valid",
                "profile": args.profile,
                "paper_section": manifest["paper_section"],
                "scene_size_m": manifest["paper_scene"]["scene_size_m"],
                "obstacle_count": manifest["paper_scene"]["obstacle_count"],
                "config_sha256": manifest["config_sha256"],
                "manifest": os.path.abspath(args.manifest),
            }, sort_keys=True))
            return
        import rospy
        rospy.init_node("rao_2025_protocol")
        run_ros_node(args.config, args.profile, args.manifest)
    except (OSError, ProtocolError, yaml.YAMLError) as exc:
        raise SystemExit("Rao protocol validation failed: {}".format(exc))


if __name__ == "__main__":
    main()
