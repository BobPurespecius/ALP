#!/usr/bin/env python3
"""Rao et al. 2025 algorithm-level reproduction harness.

The node is deliberately independent of PX4/Gazebo/EGO-Planner.  It implements
the paper's simulation-side pipeline: bicycle target + CV EKF, FOV sharing,
kinodynamic beam search, polyhedral safe corridors, and a sampled MINCO-like
quintic trajectory optimization with L-BFGS-B penalties.
"""

import csv
import math
import os
import random
import threading
from dataclasses import dataclass

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.optimize import minimize
except ImportError:  # pragma: no cover - the supplied container includes scipy.
    minimize = None


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def norm(v):
    return float(np.linalg.norm(v))


def hinge3(x):
    return max(float(x), 0.0) ** 3


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def rot2(yaw, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]], dtype=float)


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float
    z_min: float
    z_max: float
    annulus: bool = False


@dataclass
class Corridor:
    center: np.ndarray
    half: np.ndarray

    def violation(self, p):
        p = np.asarray(p)
        return np.maximum(np.abs(p - self.center) - self.half, 0.0)


@dataclass
class BeamNode:
    p: np.ndarray
    v: np.ndarray
    path: list
    cost: float


class TargetBicycle:
    """The kinematic target model from Eq. (46) of the paper."""

    def __init__(self, get):
        self.wheelbase = float(get("~target/wheelbase", 2.5))
        self.max_speed = float(get("~target/max_speed", 2.2))
        self.steer_amp = float(get("~target/steering_amplitude", 0.34))
        self.steer_freq = float(get("~target/steering_frequency", 0.055))
        self.acc_amp = float(get("~target/acceleration_amplitude", 0.45))
        self.acc_freq = float(get("~target/acceleration_frequency", 0.041))
        self.z = float(get("~target/altitude", 1.0))
        self.bound_x = float(get("~scene/size_x", 65.0)) * 0.5 - 1.0
        self.bound_y = float(get("~scene/size_y", 30.0)) * 0.5 - 1.0
        self.p = np.array(get("~target/initial_position", [0.0, 0.0, 1.0]), dtype=float)
        self.yaw = float(get("~target/initial_yaw", 0.0))
        self.speed = float(get("~target/initial_speed", 1.5))
        self.t = 0.0
        self.steering = 0.0
        self.kappa = 0.0

    def step(self, dt):
        steering = self.steer_amp * math.sin(self.steer_freq * self.t) + 0.08 * math.sin(0.17 * self.t)
        acceleration = self.acc_amp * math.sin(self.acc_freq * self.t)
        self.steering = clamp(steering, -0.49, 0.49)
        self.speed = clamp(self.speed + acceleration * dt, 0.25, self.max_speed)
        self.kappa = math.tan(self.steering) / self.wheelbase
        self.p[0] += self.speed * math.cos(self.yaw) * dt
        self.p[1] += self.speed * math.sin(self.yaw) * dt
        if abs(self.p[0]) > self.bound_x:
            self.p[0] = clamp(self.p[0], -self.bound_x, self.bound_x)
            self.yaw = angle_diff(math.pi - self.yaw, 0.0)
        if abs(self.p[1]) > self.bound_y:
            self.p[1] = clamp(self.p[1], -self.bound_y, self.bound_y)
            self.yaw = angle_diff(-self.yaw, 0.0)
        self.yaw = angle_diff(self.yaw + self.speed * self.kappa * dt, 0.0)
        self.t += dt
        return self.state()

    def state(self):
        v = np.array([self.speed * math.cos(self.yaw), self.speed * math.sin(self.yaw), 0.0])
        return np.array(self.p), v, self.yaw


class ConstantVelocityEKF:
    """4-state CV EKF used by the paper for target prediction."""

    def __init__(self, get, initial):
        self.x = np.array([initial[0], initial[1], 0.0, 0.0], dtype=float)
        self.P = np.diag([
            float(get("~ekf/initial_position_variance", 0.25)),
            float(get("~ekf/initial_position_variance", 0.25)),
            float(get("~ekf/initial_velocity_variance", 1.0)),
            float(get("~ekf/initial_velocity_variance", 1.0)),
        ])
        self.qp = float(get("~ekf/process_noise_position", 0.04))
        self.qv = float(get("~ekf/process_noise_velocity", 0.30))
        self.r = float(get("~ekf/measurement_noise", 0.08)) ** 2

    def predict(self, dt):
        F = np.array([[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        Q = np.diag([self.qp * dt * dt, self.qp * dt * dt, self.qv * dt, self.qv * dt])
        self.x = F.dot(self.x)
        self.P = F.dot(self.P).dot(F.T) + Q

    def update(self, measurement):
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        z = np.asarray(measurement[:2], dtype=float)
        innovation = z - H.dot(self.x)
        S = H.dot(self.P).dot(H.T) + self.r * np.eye(2)
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(innovation)
        self.P = (np.eye(4) - K.dot(H)).dot(self.P)

    def position_velocity(self):
        return np.array([self.x[0], self.x[1], 1.0]), np.array([self.x[2], self.x[3], 0.0])

    def future(self, dt, count):
        p, v = self.position_velocity()
        return [(p + v * dt * (i + 1), v.copy(), math.atan2(v[1], v[0]) if norm(v[:2]) > 1e-4 else 0.0)
                for i in range(count)]


class QuinticTrajectory:
    """Piecewise quintic (MINCO-compatible boundary representation)."""

    def __init__(self, points, durations):
        self.points = [np.asarray(p, dtype=float) for p in points]
        self.durations = np.maximum(np.asarray(durations, dtype=float), 0.08)
        self.coeffs = self._fit()

    @staticmethod
    def _segment(p0, v0, a0, p1, v1, a1, T):
        c = np.zeros((6, 3), dtype=float)
        c[0], c[1], c[2] = p0, v0, a0 / 2.0
        M = np.array([[T ** 3, T ** 4, T ** 5], [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]], dtype=float)
        rhs = np.vstack((p1 - (c[0] + c[1] * T + c[2] * T ** 2),
                         v1 - (c[1] + 2 * c[2] * T), a1 - 2 * c[2]))
        c[3:6] = np.linalg.solve(M, rhs)
        return c

    def _fit(self):
        velocities = []
        for i in range(len(self.points)):
            if i == 0:
                velocities.append((self.points[1] - self.points[0]) / self.durations[0])
            elif i == len(self.points) - 1:
                velocities.append((self.points[-1] - self.points[-2]) / self.durations[-1])
            else:
                velocities.append((self.points[i + 1] - self.points[i - 1]) /
                                   (self.durations[i - 1] + self.durations[i]))
        coeffs = []
        for i, T in enumerate(self.durations):
            coeffs.append(self._segment(self.points[i], velocities[i], np.zeros(3), self.points[i + 1],
                                         velocities[i + 1], np.zeros(3), float(T)))
        return coeffs

    def evaluate(self, t):
        t = clamp(float(t), 0.0, float(np.sum(self.durations)))
        absolute = 0.0
        for i, T in enumerate(self.durations):
            if t <= absolute + T or i == len(self.durations) - 1:
                tau = t - absolute
                c = self.coeffs[i]
                p = sum(c[k] * tau ** k for k in range(6))
                v = sum(k * c[k] * tau ** (k - 1) for k in range(1, 6))
                a = sum(k * (k - 1) * c[k] * tau ** (k - 2) for k in range(2, 6))
                j = sum(k * (k - 1) * (k - 2) * c[k] * tau ** (k - 3) for k in range(3, 6))
                return p, v, a, j
            absolute += T
        return self.points[-1], np.zeros(3), np.zeros(3), np.zeros(3)


class Rao2025Full:
    def __init__(self):
        get = rospy.get_param
        self.get = get
        self.frame_id = get("~frame_id", "world")
        self.rate_hz = float(get("~rate", 20.0))
        self.plan_rate = float(get("~planner_rate", 2.0))
        self.horizon = float(get("~planning_horizon", 2.4))
        self.pred_dt = float(get("~prediction_dt", 0.3))
        self.pred_steps = int(get("~prediction_steps", 8))
        self.duration = float(get("~duration", 0.0))
        self.csv_path = get("~csv_path", "/tmp/rao_2025_reproduction.csv")
        self.ns = get("~topics/namespace", "/rao_2025").rstrip("/")
        self.scene_x = float(get("~scene/size_x", 65.0))
        self.scene_y = float(get("~scene/size_y", 30.0))
        self.scene_z = float(get("~scene/size_z", 4.0))
        self.vehicle_radius = float(get("~scene/vehicle_radius", 0.35))
        self.obstacles = self._generate_obstacles()
        self.obstacle_xy = np.array([[o.x, o.y] for o in self.obstacles], dtype=float)
        self.obstacle_radius = np.array([o.radius for o in self.obstacles], dtype=float)
        self.obstacle_z_min = np.array([o.z_min for o in self.obstacles], dtype=float)
        self.obstacle_z_max = np.array([o.z_max for o in self.obstacles], dtype=float)
        self.uav_max_speed = float(get("~uav/max_speed", 3.0))
        self.uav_max_acc = float(get("~uav/max_acceleration", 6.0))
        self.uav_max_jerk = float(get("~uav/max_jerk", 20.0))
        self.uav_max_yaw_rate = float(get("~uav/max_yaw_rate", 2.5))
        self.pair_clearance = float(get("~uav/pair_clearance", 1.0))
        self.execution_follow_gain = float(get("~uav/execution_follow_gain", 0.35))
        self.tracking_distance = float(get("~uav/tracking_distance", 2.8))
        self.relative = [np.array(x, dtype=float) for x in get("~uav/relative_positions", [[-2.4, 1.8, 0], [-2.8, 0, 0], [-2.4, -1.8, 0]])]
        self.front_dt = float(get("~front_end/primitive_dt", self.pred_dt))
        self.beam_width = int(get("~front_end/beam_width", 36))
        self.acc_levels = [float(x) for x in get("~front_end/acceleration_levels", [-1.0, 0.0, 1.0])]
        # The paper's 3-D input space is represented by a 2-D 3x3 lattice plus
        # two pure vertical primitives. This keeps the online search bounded.
        self.primitive_controls = [np.array([ax, ay, 0.0], dtype=float)
                                   for ax in self.acc_levels for ay in self.acc_levels]
        self.primitive_controls += [np.array([0.0, 0.0, az], dtype=float) for az in self.acc_levels if abs(az) > 1e-6]
        self.lambda_time = float(get("~front_end/lambda_time", 0.15))
        self.front_visibility_weight = float(get("~front_end/visibility_weight", 3.0))
        self.max_corridor_length = float(get("~front_end/max_corridor_length", 4.0))
        self.backend_iterations = int(get("~back_end/max_iterations", 8))
        self.backend_samples = int(get("~back_end/sample_count", 5))
        self.weights = {name: float(get("~back_end/" + name, default)) for name, default in {
            "time_weight": 0.05, "jerk_weight": 0.015, "formation_weight": 7.0, "distance_weight": 3.0,
            "angular_weight": 1.2, "yaw_weight": 0.35, "fov_weight": 45.0, "los_weight": 6000.0, "obstacle_weight": 80.0,
            "feasibility_weight": 8.0, "reciprocal_weight": 60.0, "corridor_weight": 70.0}.items()}
        self.hfov = math.radians(float(get("~camera/horizontal_fov_deg", 85.0)) / 2.0)
        self.vfov = math.radians(float(get("~camera/vertical_fov_deg", 72.0)) / 2.0)
        self.min_distance = float(get("~camera/min_distance", 0.5))
        self.max_distance = float(get("~camera/max_distance", 20.0))
        self.los_step = float(get("~camera/los_sample_step", 0.12))
        self.visibility_clearance = float(get("~camera/visibility_clearance", 0.03))
        self.detect_probability = float(get("~camera/detection_probability", 1.0))
        initial = np.array(get("~target/initial_position", [0, 0, 1]), dtype=float)
        self.target = TargetBicycle(get)
        self.ekf = ConstantVelocityEKF(get, initial)
        self.uav = [np.array(p, dtype=float) for p in get("~uav/initial_positions", [[.75, 1.4, 1], [-1.5, 0, 1], [.75, -1.4, 1]])]
        self.uav_vel = [np.zeros(3) for _ in range(3)]
        self.uav_yaw = [0.0, 0.0, 0.0]
        self.plans = [None, None, None]
        self.corridors = [[], [], []]
        self.next_plan = 0.0
        self.plan_clock = 0.0
        self.plan_lock = threading.Lock()
        self.plan_thread = None
        self.plan_result = None
        self.last_plan_stats = (0, 0, 0.0)
        self.last_visible = [False, False, False]
        self.visibility_failures = {"range": 0, "fov": 0, "los": 0}
        self.last_source = -1
        self.last_measurement_time = -float("inf")
        self.visible_counts = [0, 0, 0, 0]
        self.samples = 0
        self.collision_count = 0
        self.pair_collision_count = 0
        self.sum_ekf_error = 0.0
        self.sum_track_error = 0.0
        self.min_pair_distance = float("inf")
        self.csv_file = None
        self.csv_writer = None
        self._open_csv()
        self.target_pub = rospy.Publisher(self.ns + "/target_true", PoseStamped, queue_size=2)
        self.shared_pub = rospy.Publisher(self.ns + "/target_shared", PoseStamped, queue_size=2)
        self.target_path_pub = rospy.Publisher(self.ns + "/target_path", Path, queue_size=1)
        self.shared_path_pub = rospy.Publisher(self.ns + "/target_shared_path", Path, queue_size=1)
        self.uav_path_pubs = [rospy.Publisher(self.ns + "/uav%d/path" % (i + 1), Path, queue_size=1) for i in range(3)]
        self.plan_pubs = [rospy.Publisher(self.ns + "/uav%d/planned_path" % (i + 1), Path, queue_size=1) for i in range(3)]
        self.marker_pub = rospy.Publisher(self.ns + "/markers", MarkerArray, queue_size=1)
        self.metrics_pub = rospy.Publisher(self.ns + "/metrics", Float32MultiArray, queue_size=2)
        self.target_path, self.shared_path = Path(), Path()
        self.uav_paths = [Path(), Path(), Path()]
        rospy.on_shutdown(self._close_csv)

    def _open_csv(self):
        parent = os.path.dirname(self.csv_path)
        if parent and not os.path.isdir(parent):
            try:
                os.makedirs(parent)
            except OSError:
                pass
        try:
            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["time_s", "visible_count", "visible_uav1", "visible_uav2", "visible_uav3",
                                       "sharing_source", "shared_target_available", "ekf_error_m", "mean_tracking_error_m", "min_pair_distance_m",
                                       "obstacle_collision", "uav_collision", "front_end_nodes", "corridor_count", "backend_cost"])
        except IOError as exc:
            rospy.logwarn("Cannot write CSV %s: %s", self.csv_path, exc)

    def _close_csv(self):
        if self.csv_file:
            self.csv_file.close()

    def _generate_obstacles(self):
        rng = random.Random(int(self.get("~random_seed", 2025)))
        count = int(self.get("~scene/obstacle_count", 140))
        wmin, wmax = float(self.get("~scene/cube_width_min", .3)), float(self.get("~scene/cube_width_max", .8))
        hmin, hmax = float(self.get("~scene/cube_height_min", 2.0)), float(self.get("~scene/cube_height_max", 3.0))
        amin, amax = float(self.get("~scene/annulus_diameter_min", .5)) / 2, float(self.get("~scene/annulus_diameter_max", .7)) / 2
        annulus_prob = float(self.get("~scene/annulus_probability", .18))
        obstacles = []
        for _ in range(count):
            x = rng.uniform(-self.scene_x / 2 + 1.0, self.scene_x / 2 - 1.0)
            y = rng.uniform(-self.scene_y / 2 + 1.0, self.scene_y / 2 - 1.0)
            annulus = rng.random() < annulus_prob
            radius = rng.uniform(amin, amax) if annulus else rng.uniform(wmin, wmax) / 2.0
            obstacles.append(Obstacle(x, y, radius, 0.0, rng.uniform(hmin, hmax), annulus))
        return obstacles

    def _line_clearance(self, a, b, inflation):
        a, b = np.asarray(a), np.asarray(b)
        d = b - a
        dxy2 = float(d[0] * d[0] + d[1] * d[1])
        vertical = (self.obstacle_z_min - inflation <= max(a[2], b[2])) & (self.obstacle_z_max + inflation >= min(a[2], b[2]))
        if not np.any(vertical):
            return float("inf")
        centers = self.obstacle_xy[vertical]
        radii = self.obstacle_radius[vertical]
        if dxy2 < 1e-10:
            closest = np.broadcast_to(a[:2], centers.shape)
        else:
            u = np.clip((centers - a[:2]).dot(d[:2]) / dxy2, 0.0, 1.0)
            closest = a[:2] + u[:, None] * d[:2]
        distances = np.linalg.norm(centers - closest, axis=1)
        # Annular obstacles contain free space inside the ring.  Store the
        # outer radius in ``radius`` and use a conservative 0.10 m wall for
        # the inner boundary when evaluating a ray or swept segment.
        annular = np.array([o.annulus for o in self.obstacles], dtype=bool)[vertical]
        clearance = distances - radii - inflation
        if np.any(annular):
            inner = np.maximum(radii - 0.10, 0.0)
            ring_hit = annular & (distances + inflation >= inner)
            clearance = np.where(annular & ~ring_hit, np.inf, clearance)
        return float(np.min(clearance))

    def _line_free(self, a, b, inflation=None):
        if inflation is None:
            inflation = self.vehicle_radius
        return self._line_clearance(a, b, inflation) >= 0.0

    def _point_collision(self, p):
        p = np.asarray(p)
        vertical = (self.obstacle_z_min - self.vehicle_radius <= p[2]) & (self.obstacle_z_max + self.vehicle_radius >= p[2])
        if not np.any(vertical):
            return False
        distances = np.linalg.norm(self.obstacle_xy[vertical] - p[:2], axis=1)
        obstacles = [o for o in self.obstacles if o.z_min - self.vehicle_radius <= p[2] <= o.z_max + self.vehicle_radius]
        hit = distances <= self.obstacle_radius[vertical] + self.vehicle_radius
        for idx, obstacle in enumerate(obstacles):
            if obstacle.annulus:
                hit[idx] = hit[idx] and distances[idx] + self.vehicle_radius >= max(obstacle.radius - 0.10, 0.0)
        return bool(np.any(hit))

    def _visible(self, uav, target, yaw):
        rel = np.asarray(target) - np.asarray(uav)
        d = norm(rel)
        if d < self.min_distance or d > self.max_distance:
            self.visibility_failures["range"] += 1
            return False
        horizontal = abs(angle_diff(math.atan2(rel[1], rel[0]), yaw))
        vertical = abs(math.atan2(rel[2], max(1e-6, math.hypot(rel[0], rel[1]))))
        if horizontal > self.hfov or vertical > self.vfov:
            self.visibility_failures["fov"] += 1
            return False
        if not self._line_free(uav, target, self.visibility_clearance):
            self.visibility_failures["los"] += 1
            return False
        return True

    def _target_prediction(self):
        return self.ekf.future(self.pred_dt, self.pred_steps)

    def _heuristic(self, p, v, goal, goal_v):
        # Pontryagin-inspired minimum-jerk heuristic (Eqs. 5-8).
        delta = np.asarray(goal) - p
        speed = max(0.5, norm(v))
        T = clamp(norm(delta) / speed, self.front_dt, self.horizon)
        return norm(delta) + 0.25 * norm(np.asarray(goal_v) - v) + self.lambda_time * T

    def _visible_tracking_goal(self, target_p, target_yaw, relative):
        desired = target_p + rot2(target_yaw, relative)
        if self._line_free(desired, target_p, self.visibility_clearance):
            return desired
        radius = max(0.8, math.hypot(relative[0], relative[1]))
        base_angle = math.atan2(relative[1], relative[0]) + target_yaw
        # W_n: choose the nearest unobstructed formation slot around the
        # predicted target when the nominal relative point is occluded.
        for offset_deg in (20, -20, 40, -40, 60, -60, 90, -90, 120, -120, 160, -160, 180):
            angle = base_angle + math.radians(offset_deg)
            candidate = np.array([target_p[0] + radius * math.cos(angle),
                                  target_p[1] + radius * math.sin(angle),
                                  target_p[2] + relative[2]], dtype=float)
            if (abs(candidate[0]) < self.scene_x / 2 and abs(candidate[1]) < self.scene_y / 2 and
                    self.vehicle_radius < candidate[2] < self.scene_z - self.vehicle_radius and
                    self._line_free(candidate, target_p, self.visibility_clearance)):
                return candidate
        return desired

    def _primitive_search(self, start_p, start_v, predictions, relative):
        beam = [BeamNode(np.asarray(start_p).copy(), np.asarray(start_v).copy(), [np.asarray(start_p).copy()], 0.0)]
        total_nodes = 0
        for depth, (target_p, target_v, target_yaw) in enumerate(predictions):
            desired = self._visible_tracking_goal(target_p, target_yaw, relative)
            next_beam = []
            for node in beam:
                for control in self.primitive_controls:
                    acc = self.uav_max_acc * control
                    v = node.v + acc * self.front_dt
                    vnorm = norm(v)
                    if vnorm > self.uav_max_speed:
                        v *= self.uav_max_speed / vnorm
                    p = node.p + node.v * self.front_dt + 0.5 * acc * self.front_dt ** 2
                    if not (-self.scene_x / 2 < p[0] < self.scene_x / 2 and -self.scene_y / 2 < p[1] < self.scene_y / 2 and 0.2 < p[2] < self.scene_z):
                        continue
                    if not self._line_free(node.p, p):
                        continue
                    cost = node.cost + (norm(acc) ** 2 + self.lambda_time) * self.front_dt
                    cost += self._heuristic(p, v, desired, target_v)
                    los_clearance = self._line_clearance(p, target_p, self.visibility_clearance)
                    cost += self.front_visibility_weight * hinge3(-los_clearance)
                    if norm(v[:2]) > 1.0e-5:
                        bearing = math.atan2(target_p[1] - p[1], target_p[0] - p[0])
                        excess = max(abs(angle_diff(bearing, math.atan2(v[1], v[0]))) - self.hfov, 0.0)
                        cost += self.front_visibility_weight * excess * excess
                    next_beam.append(BeamNode(p, v, node.path + [p.copy()], cost))
                    total_nodes += 1
            next_beam.sort(key=lambda n: n.cost)
            beam = next_beam[:self.beam_width]
            if not beam:
                return [np.asarray(start_p).copy()], np.asarray(start_v).copy(), total_nodes
        terminal = self._visible_tracking_goal(predictions[-1][0], predictions[-1][2], relative)
        best = min(beam, key=lambda n: n.cost + self._heuristic(n.p, n.v, terminal, predictions[-1][1]))
        return best.path, best.v, total_nodes

    def _build_corridors(self, path):
        corridors = []
        max_len = self.max_corridor_length
        for i in range(max(1, len(path) - 1)):
            a = np.asarray(path[i])
            b = np.asarray(path[min(i + 1, len(path) - 1)])
            center = 0.5 * (a + b)
            half = np.maximum(np.abs(b - a) * 0.5 + self.vehicle_radius + 0.15, 0.45)
            half = np.minimum(half, np.array([max_len, max_len, 1.4]))
            for _ in range(12):
                bad = False
                for o in self.obstacles:
                    xy_overlap = math.hypot(max(abs(o.x - center[0]) - half[0], 0.0), max(abs(o.y - center[1]) - half[1], 0.0)) <= o.radius + self.vehicle_radius
                    z_overlap = abs((o.z_min + o.z_max) * 0.5 - center[2]) <= half[2] + (o.z_max - o.z_min) * 0.5
                    if xy_overlap and z_overlap:
                        bad = True
                        axis = int(np.argmax(half[:2]))
                        half[axis] *= 0.72
                        break
                if not bad:
                    break
            corridors.append(Corridor(center, half))
        return corridors

    def _sample_trajectory(self, traj):
        total = float(np.sum(traj.durations))
        return [traj.evaluate(total * i / max(1, self.backend_samples - 1)) for i in range(self.backend_samples)]

    def _backend_optimize(self, start, seed_path, predictions, relative, corridors, other_trajs):
        if len(seed_path) > 5:
            seed_path = [seed_path[int(round(i * (len(seed_path) - 1) / 4.0))] for i in range(5)]
        points = [np.asarray(seed_path[min(i, len(seed_path) - 1)], dtype=float) for i in range(1, len(seed_path))]
        if len(points) < 2:
            points = [np.asarray(start, dtype=float), np.asarray(seed_path[-1], dtype=float)]
        start = np.asarray(start, dtype=float)
        end = points[-1].copy()
        interior_count = max(0, len(points) - 2)
        x0 = np.concatenate([p for p in points[1:-1]]) if interior_count else np.zeros(0)
        durations = np.full(len(points) - 1, self.horizon / max(1, len(points) - 1))

        def make_traj(x):
            interior = [x[3 * i:3 * i + 3] for i in range(interior_count)]
            return QuinticTrajectory([start] + interior + [end], durations)

        def objective(x):
            traj = make_traj(x)
            vals = self._sample_trajectory(traj)
            cost = self.weights["time_weight"] * float(np.sum(durations))
            for idx, (p, v, a, j) in enumerate(vals):
                t = self.horizon * idx / max(1, len(vals) - 1)
                pred_idx = min(len(predictions) - 1, int(t / self.pred_dt))
                target_p, target_v, target_yaw = predictions[pred_idx]
                desired = self._visible_tracking_goal(target_p, target_yaw, relative)
                cost += self.weights["formation_weight"] * norm(p - desired) ** 2
                dist2 = norm(p - target_p) ** 2
                cost += self.weights["distance_weight"] * (hinge3(self.tracking_distance ** 2 - dist2) + hinge3(dist2 - (self.tracking_distance + 1.5) ** 2))
                cost += self.weights["jerk_weight"] * norm(j) ** 2
                cost += self.weights["feasibility_weight"] * (hinge3(norm(v) ** 2 - self.uav_max_speed ** 2) + hinge3(norm(a) ** 2 - self.uav_max_acc ** 2) + hinge3(norm(j) ** 2 - self.uav_max_jerk ** 2))
                vertical = (self.obstacle_z_min - self.vehicle_radius <= p[2]) & (self.obstacle_z_max + self.vehicle_radius >= p[2])
                if np.any(vertical):
                    delta = self.obstacle_xy[vertical] - p[:2]
                    violation = (self.obstacle_radius[vertical] + self.vehicle_radius) ** 2 - np.sum(delta * delta, axis=1)
                    annular = np.array([o.annulus for o in self.obstacles], dtype=bool)[vertical]
                    inner = np.maximum(self.obstacle_radius[vertical] - 0.10 - self.vehicle_radius, 0.0)
                    inside_hole = annular & (np.sum(delta * delta, axis=1) <= inner ** 2)
                    violation = np.where(inside_hole, 0.0, violation)
                    cost += self.weights["obstacle_weight"] * float(np.sum(np.maximum(violation, 0.0) ** 3))
                yaw = math.atan2(v[1], v[0]) if norm(v[:2]) > 1e-5 else target_yaw
                cost += self.weights["yaw_weight"] * angle_diff(yaw, math.atan2(v[1], v[0]) if norm(v[:2]) > 1e-5 else yaw) ** 2
                bearing = math.atan2(target_p[1] - p[1], target_p[0] - p[0])
                fov_excess = max(abs(angle_diff(bearing, yaw)) - self.hfov, 0.0)
                cost += self.weights["fov_weight"] * hinge3(fov_excess)
                los_clearance = self._line_clearance(p, target_p, self.visibility_clearance)
                cost += self.weights["los_weight"] * hinge3(-los_clearance)
                corridor = corridors[min(len(corridors) - 1, idx * len(corridors) // max(1, len(vals)))]
                cost += self.weights["corridor_weight"] * norm(corridor.violation(p)) ** 2
                for other in other_trajs:
                    op = other.evaluate(clamp(t, 0.0, float(np.sum(other.durations))))[0]
                    cost += self.weights["reciprocal_weight"] * hinge3(self.pair_clearance ** 2 - norm(p - op) ** 2)
            # Angular formation penalty, evaluated against the latest peer states.
            for other in other_trajs:
                cost += self.weights["angular_weight"] * hinge3(0.8 - norm(end - other.points[-1]))
            return float(cost)

        if minimize is not None and x0.size:
            bounds = [(-self.scene_x / 2 + .5, self.scene_x / 2 - .5), (-self.scene_y / 2 + .5, self.scene_y / 2 - .5), (0.35, self.scene_z - .2)] * interior_count
            result = minimize(objective, x0, method="L-BFGS-B", bounds=bounds,
                              options={"maxiter": self.backend_iterations, "ftol": 1e-3, "maxls": 8})
            x = result.x if np.all(np.isfinite(result.x)) else x0
            return make_traj(x), objective(x)
        return make_traj(x0), objective(x0)

    def _plan(self, uav_state=None, velocity_state=None, predictions=None):
        uav_state = self.uav if uav_state is None else uav_state
        velocity_state = self.uav_vel if velocity_state is None else velocity_state
        predictions = self._target_prediction() if predictions is None else predictions
        planned = []
        total_nodes = 0
        total_corridors = 0
        costs = []
        for i in range(3):
            path, _, nodes = self._primitive_search(uav_state[i], velocity_state[i], predictions, self.relative[i])
            corridors = self._build_corridors(path)
            traj, cost = self._backend_optimize(uav_state[i], path, predictions, self.relative[i], corridors, planned)
            planned.append(traj)
            total_nodes += nodes
            total_corridors += len(corridors)
            costs.append(cost)
        return planned, [self._build_corridors(self._trajectory_points(x)) for x in planned], (total_nodes, total_corridors, float(np.mean(costs)))

    def _trajectory_points(self, trajectory):
        total = float(np.sum(trajectory.durations))
        return [trajectory.evaluate(total * i / max(1, len(trajectory.durations)))[0]
                for i in range(len(trajectory.durations) + 1)]

    def _start_plan(self, t):
        with self.plan_lock:
            if self.plan_thread is not None and self.plan_thread.is_alive():
                return
            snapshot_uav = [p.copy() for p in self.uav]
            snapshot_vel = [v.copy() for v in self.uav_vel]
            predictions = [(p.copy(), v.copy(), yaw) for p, v, yaw in self._target_prediction()]
            target_base, _ = self.ekf.position_velocity()
            self.plan_thread = threading.Thread(target=self._plan_worker,
                                                args=(snapshot_uav, snapshot_vel, predictions, target_base), daemon=True)
            self.plan_thread.start()

    def _plan_worker(self, uav_state, velocity_state, predictions, target_base):
        try:
            planned, corridors, stats = self._plan(uav_state, velocity_state, predictions)
            with self.plan_lock:
                self.plan_result = (planned, corridors, stats, uav_state, target_base)
        except Exception as exc:
            rospy.logwarn("Rao planner worker failed: %s", exc)

    def _accept_plan(self, t):
        with self.plan_lock:
            if self.plan_result is None:
                return False
            plans, _, self.last_plan_stats, planned_starts, target_base = self.plan_result
            self.plan_result = None
            # The optimizer runs on a snapshot. Re-anchor the accepted plan to
            # the current UAV state so a delayed worker result cannot teleport
            # the executed vehicle or invalidate the rolling-horizon start.
            self.plans = []
            self.corridors = []
            current_target, _ = self.ekf.position_velocity()
            target_shift = current_target - target_base
            for i, plan in enumerate(plans):
                points = [self.uav[i].copy()] + [p.copy() + target_shift for p in plan.points[1:]]
                if len(points) < 2:
                    points.append(planned_starts[i].copy())
                anchored = QuinticTrajectory(points, plan.durations)
                self.plans.append(anchored)
                self.corridors.append(self._build_corridors(self._trajectory_points(anchored)))
            self.plan_clock = t
            return True

    def _pose(self, point, stamp, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [float(x) for x in point]
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return msg

    def _append(self, path, point, stamp, limit=1000):
        path.header.stamp, path.header.frame_id = stamp, self.frame_id
        path.poses.append(self._pose(point, stamp))
        path.poses = path.poses[-limit:]

    def _marker(self, marker_id, marker_type, scale, color, point=None):
        m = Marker()
        m.header.frame_id, m.ns, m.id, m.type, m.action = self.frame_id, "rao_2025", marker_id, marker_type, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color = ColorRGBA(*color)
        if point is not None:
            m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(x) for x in point]
        return m

    def _publish_markers(self, target, shared, visible, stamp):
        arr = MarkerArray()
        for i, o in enumerate(self.obstacles):
            m = self._marker(i, Marker.CYLINDER, (2 * o.radius, 2 * o.radius, o.z_max), (0.35, .40, .48, .8), (o.x, o.y, o.z_max / 2))
            m.header.stamp = stamp
            arr.markers.append(m)
        for i, p in enumerate(self.uav):
            color = (.15, .85, .3, 1) if visible[i] else (.95, .52, .1, 1)
            m = self._marker(1000 + i, Marker.SPHERE, (.6, .6, .6), color, p)
            m.header.stamp = stamp
            arr.markers.append(m)
            line = self._marker(1100 + i, Marker.LINE_STRIP, (.05, .05, .05), color)
            line.header.stamp, line.points = stamp, [Point(*p), Point(*target)]
            arr.markers.append(line)
            for j, c in enumerate(self.corridors[i]):
                # Wireframe box: a visual representation of G_c p - b_c <= 0.
                lo, hi = c.center - c.half, c.center + c.half
                corners = [np.array([x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
                edges = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
                box = self._marker(2000 + i * 100 + j, Marker.LINE_LIST, (.025, .025, .025), (.2, .75, .95, .25))
                box.header.stamp = stamp
                for a, b in edges:
                    box.points.extend([Point(*corners[a]), Point(*corners[b])])
                arr.markers.append(box)
        arr.markers.append(self._marker(1200, Marker.SPHERE, (.7, .7, .7), (.95, .1, .1, 1), target))
        arr.markers.append(self._marker(1201, Marker.SPHERE, (.5, .5, .5), (.1, .6, .95, 1), shared))
        self.marker_pub.publish(arr)

    def _move_uavs(self, now, dt):
        for i, plan in enumerate(self.plans):
            if plan is None:
                continue
            p, v, a, _ = plan.evaluate(min(self.horizon, now - self.plan_clock))
            speed = norm(v)
            if speed > self.uav_max_speed:
                v *= self.uav_max_speed / speed
            # Keep execution safe while allowing progress when a finite-
            # penalty optimizer leaves a small residual obstacle violation.
            p = self._project_to_free(p, i, check_pairs=False)
            self.uav[i] = p
            self.uav_vel[i] = v
            velocity_yaw = math.atan2(v[1], v[0]) if norm(v[:2]) > 1e-4 else self.uav_yaw[i]
            # The paper fixes the camera-to-body transform and penalizes yaw
            # mismatch against the velocity direction (Eq. 34).  Do not aim
            # the camera directly at the target, since that overestimates FOV
            # visibility compared with the published setup.
            self.uav_yaw[i] = angle_diff(self.uav_yaw[i] + clamp(angle_diff(velocity_yaw, self.uav_yaw[i]),
                                                                 -self.uav_max_yaw_rate * dt, self.uav_max_yaw_rate * dt), 0.0)
        # Resolve reciprocal clearance symmetrically after all three proposed
        # positions are available. Sequential projection alone can miss the
        # case where UAV 1 moves into the previous position of UAV 2.
        for _ in range(6):
            resolved = True
            for i, j in ((0, 1), (0, 2), (1, 2)):
                delta = self.uav[j] - self.uav[i]
                d = norm(delta)
                if d >= self.pair_clearance:
                    continue
                resolved = False
                direction = delta / d if d > 1e-6 else np.array([1.0, 0.0, 0.0])
                correction = (self.pair_clearance + 0.04 - d) * 0.5
                self.uav[i] = self._project_to_free(self.uav[i] - direction * correction, i, check_pairs=False)
                self.uav[j] = self._project_to_free(self.uav[j] + direction * correction, j, check_pairs=False)
            if resolved:
                break
        rospy.loginfo_throttle(2.0, "Rao state target=(%.1f,%.1f) uav1=(%.1f,%.1f) uav2=(%.1f,%.1f) uav3=(%.1f,%.1f)",
                               self.target.p[0], self.target.p[1], self.uav[0][0], self.uav[0][1],
                               self.uav[1][0], self.uav[1][1], self.uav[2][0], self.uav[2][1])

    def _project_to_free(self, point, index, check_pairs=True):
        p = np.asarray(point, dtype=float).copy()
        for _ in range(8):
            changed = False
            for o in self.obstacles:
                if o.z_min - self.vehicle_radius <= p[2] <= o.z_max + self.vehicle_radius:
                    dxy = np.array([p[0] - o.x, p[1] - o.y], dtype=float)
                    d = norm(dxy)
                    clearance = o.radius + self.vehicle_radius + 0.06
                    inner = max(o.radius - 0.10 - self.vehicle_radius, 0.0) if o.annulus else 0.0
                    if d < clearance and d > inner:
                        direction = dxy / d if d > 1e-6 else np.array([1.0, 0.0])
                        p[:2] = np.array([o.x, o.y]) + direction * clearance
                        changed = True
            if check_pairs:
                for j in range(index):
                    delta = p - self.uav[j]
                    d = norm(delta)
                    if d < self.pair_clearance:
                        direction = delta / d if d > 1e-6 else np.array([0.0, 1.0, 0.0])
                        p = self.uav[j] + direction * (self.pair_clearance + 0.04)
                        changed = True
            p[0] = clamp(p[0], -self.scene_x / 2 + self.vehicle_radius, self.scene_x / 2 - self.vehicle_radius)
            p[1] = clamp(p[1], -self.scene_y / 2 + self.vehicle_radius, self.scene_y / 2 - self.vehicle_radius)
            p[2] = clamp(p[2], self.vehicle_radius, self.scene_z - self.vehicle_radius)
            if not changed:
                break
        if self._point_collision(p):
            return self.uav[index].copy()
        return p

    def step(self, t, dt):
        target, target_v, target_yaw = self.target.step(dt)
        self.ekf.predict(dt)
        self._accept_plan(t)
        if t >= self.next_plan:
            self._start_plan(t)
            self.next_plan = t + 1.0 / max(.1, self.plan_rate)
        nodes, corridors, backend_cost = self.last_plan_stats
        self._move_uavs(t, dt)
        # Sensor visibility is evaluated at the executed state, after the
        # current control step. This avoids counting a one-frame false loss at
        # every rolling-plan handover.
        visible = [self._visible(self.uav[i], target, self.uav_yaw[i]) for i in range(3)]
        source = next((i for i, x in enumerate(visible) if x and random.random() <= self.detect_probability), -1)
        if source >= 0:
            noise = np.random.normal(0.0, math.sqrt(self.ekf.r), 2)
            self.ekf.update(target[:2] + noise)
            self.last_measurement_time = t
        self.last_visible = visible
        self.last_source = source
        obstacle_collision = 0
        for p in self.uav:
            for o in self.obstacles:
                if o.z_min <= p[2] <= o.z_max and math.hypot(p[0] - o.x, p[1] - o.y) <= o.radius + self.vehicle_radius:
                    obstacle_collision += 1
        pair_distances = [norm(self.uav[0] - self.uav[1]), norm(self.uav[0] - self.uav[2]), norm(self.uav[1] - self.uav[2])]
        pair_collision = sum(1 for d in pair_distances if d < self.pair_clearance)
        self.collision_count += int(obstacle_collision > 0)
        self.pair_collision_count += int(pair_collision > 0)
        self.min_pair_distance = min(self.min_pair_distance, min(pair_distances))
        shared, _ = self.ekf.position_velocity()
        ekf_error = norm(shared - target)
        tracking_error = float(np.mean([norm(p - target) for p in self.uav]))
        count = sum(visible)
        self.visible_counts[count] += 1
        self.samples += 1
        self.sum_ekf_error += ekf_error
        self.sum_track_error += tracking_error
        stamp = rospy.Time.now()
        self.target_pub.publish(self._pose(target, stamp, target_yaw))
        self.shared_pub.publish(self._pose(shared, stamp, math.atan2(shared[1] - target[1], shared[0] - target[0])))
        self._append(self.target_path, target, stamp)
        self._append(self.shared_path, shared, stamp)
        self.target_path_pub.publish(self.target_path)
        self.shared_path_pub.publish(self.shared_path)
        for i in range(3):
            self._append(self.uav_paths[i], self.uav[i], stamp)
            self.uav_path_pubs[i].publish(self.uav_paths[i])
            if self.plans[i] is not None:
                plan_path = Path(); plan_path.header.frame_id = self.frame_id; plan_path.header.stamp = stamp
                for k in range(18):
                    plan_path.poses.append(self._pose(self.plans[i].evaluate(self.horizon * k / 17.0)[0], stamp))
                self.plan_pubs[i].publish(plan_path)
        self._publish_markers(target, shared, visible, stamp)
        percentages = [100.0 * x / max(1, self.samples) for x in self.visible_counts]
        self.metrics_pub.publish(Float32MultiArray(data=[count, ekf_error, tracking_error, self.min_pair_distance] + list(reversed(percentages))))
        if self.csv_writer:
            shared_available = int(t - self.last_measurement_time <= max(2.0, self.horizon))
            self.csv_writer.writerow(["%.3f" % t, count] + [int(x) for x in visible] + [source, shared_available,
                                      "%.5f" % ekf_error, "%.5f" % tracking_error, "%.5f" % self.min_pair_distance, obstacle_collision,
                                      pair_collision, nodes, corridors, "%.5f" % backend_cost])
            self.csv_file.flush()

    def run(self):
        rospy.loginfo("Rao full reproduction: bicycle target, CV-EKF, kinodynamic beam search, safe corridors, L-BFGS quintic backend")
        rospy.loginfo("Scene %.1fx%.1fx%.1f m, obstacles=%d, FOV=%.1fx%.1f deg, UAV limits v=%.1f a=%.1f jerk=%.1f",
                      self.scene_x, self.scene_y, self.scene_z, len(self.obstacles), math.degrees(2 * self.hfov),
                      math.degrees(2 * self.vfov), self.uav_max_speed, self.uav_max_acc, self.uav_max_jerk)
        rate = rospy.Rate(self.rate_hz)
        start, last = rospy.Time.now(), rospy.Time.now()
        self.plan_clock = 0.0
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = clamp((now - last).to_sec(), 1.0 / self.rate_hz, 0.15)
            last = now
            t = (now - start).to_sec()
            self.step(t, dt)
            if self.duration > 0.0 and t >= self.duration:
                break
            rate.sleep()
        rospy.loginfo("Rao result: samples=%d visible[3/2/1/0]=%s, mean EKF error=%.3f m, min pair=%.3f m, collisions=%d/%d",
                      self.samples, list(reversed(self.visible_counts)), self.sum_ekf_error / max(1, self.samples),
                      self.min_pair_distance, self.collision_count, self.pair_collision_count)
        rospy.loginfo("Rao visibility rejects: range=%d fov=%d los=%d", self.visibility_failures["range"],
                      self.visibility_failures["fov"], self.visibility_failures["los"])


if __name__ == "__main__":
    rospy.init_node("rao_2025_full_reproduction")
    Rao2025Full().run()
