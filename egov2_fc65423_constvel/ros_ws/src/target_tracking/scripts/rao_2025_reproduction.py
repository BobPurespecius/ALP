#!/usr/bin/env python3
"""Standalone kinematic harness for the Rao et al. 2025 FOV experiment.

It deliberately has no dependency on PX4, Gazebo, EGO-Planner, or the existing
door/stair launch files.  The node produces ROS topics for RViz and a CSV with
visibility, tracking, and separation metrics.
"""

import csv
import math
import os
import random
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float
    z_min: float
    z_max: float


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def move_towards(current, goal, max_step):
    delta = [goal[i] - current[i] for i in range(3)]
    norm = math.sqrt(sum(v * v for v in delta))
    if norm <= max_step or norm < 1.0e-9:
        return list(goal)
    scale = max_step / norm
    return [current[i] + scale * delta[i] for i in range(3)]


class Rao2025Reproduction:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.seed = int(rospy.get_param("~random_seed", 2025))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.csv_path = rospy.get_param("~csv_path", "/tmp/rao_2025_reproduction.csv")
        self.ns = rospy.get_param("~topics/namespace", "/rao_2025").rstrip("/")
        self.scene_x = float(rospy.get_param("~scene/size_x", 65.0))
        self.scene_y = float(rospy.get_param("~scene/size_y", 30.0))
        self.scene_z = float(rospy.get_param("~scene/size_z", 4.0))
        self.obstacle_count = int(rospy.get_param("~scene/obstacle_count", 140))
        self.r_min = float(rospy.get_param("~scene/obstacle_radius_min", 0.25))
        self.r_max = float(rospy.get_param("~scene/obstacle_radius_max", 0.75))
        self.h_min = float(rospy.get_param("~scene/obstacle_height_min", 1.0))
        self.h_max = float(rospy.get_param("~scene/obstacle_height_max", 3.5))
        self.obstacle_clearance = float(rospy.get_param("~scene/obstacle_clearance", 1.0))
        self.uav_initial = rospy.get_param(
            "~uav/initial_positions",
            [[0.75, 1.4, 1.0], [-1.5, 0.0, 1.0], [0.75, -1.4, 1.0]],
        )
        self.uav_max_speed = float(rospy.get_param("~uav/max_speed", 3.0))
        self.response_time = max(0.1, float(rospy.get_param("~uav/response_time", 1.2)))
        self.uav_z = float(rospy.get_param("~uav/z", 1.5))
        self.target_z = float(rospy.get_param("~target/z", 1.5))
        self.target_speed = float(rospy.get_param("~target/max_speed", 2.2))
        self.path_half_x = float(rospy.get_param("~target/path_half_x", 22.0))
        self.path_half_y = float(rospy.get_param("~target/path_half_y", 8.0))
        self.angular_rate = float(rospy.get_param("~target/angular_rate", 0.0809))
        self.half_hfov = math.radians(float(rospy.get_param("~camera/horizontal_fov_deg", 85.0)) / 2.0)
        self.half_vfov = math.radians(float(rospy.get_param("~camera/vertical_fov_deg", 72.0)) / 2.0)
        self.min_distance = float(rospy.get_param("~camera/min_distance", 0.5))
        self.max_distance = float(rospy.get_param("~camera/max_distance", 25.0))
        self.los_step = max(0.02, float(rospy.get_param("~camera/los_sample_step", 0.20)))

        self.obstacles = self._make_obstacles()
        self.uav = [list(p) for p in self.uav_initial[:3]]
        while len(self.uav) < 3:
            self.uav.append([0.0, 0.0, self.uav_z])
        self.uav_paths = [Path() for _ in range(3)]
        self.target_path = Path()
        self.shared_path = Path()
        self.last_estimate = [0.0, 0.0, self.target_z]
        self.last_velocity = [0.0, 0.0, 0.0]
        self.last_stamp = rospy.Time.now()
        self.visible_counts = [0, 0, 0, 0]
        self.total_samples = 0
        self.sum_estimation_error = 0.0
        self.sum_tracking_error = 0.0
        self.min_pair_distance = float("inf")
        self.csv_file = None
        self.csv_writer = None
        self._open_csv()

        self.target_pub = rospy.Publisher(self.ns + "/target_true", PoseStamped, queue_size=2)
        self.shared_pub = rospy.Publisher(self.ns + "/target_shared", PoseStamped, queue_size=2)
        self.target_path_pub = rospy.Publisher(self.ns + "/target_path", Path, queue_size=1)
        self.shared_path_pub = rospy.Publisher(self.ns + "/target_shared_path", Path, queue_size=1)
        self.uav_path_pubs = [rospy.Publisher(self.ns + "/uav%d/path" % (i + 1), Path, queue_size=1) for i in range(3)]
        self.marker_pub = rospy.Publisher(self.ns + "/markers", MarkerArray, queue_size=1)
        self.metrics_pub = rospy.Publisher(self.ns + "/metrics", Float32MultiArray, queue_size=2)
        self._publish_static_markers()
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
                                      "target_estimation_error_m", "mean_tracking_error_m", "min_pair_distance_m"])
        except IOError as exc:
            rospy.logwarn("Cannot write CSV %s: %s", self.csv_path, exc)

    def _close_csv(self):
        if self.csv_file:
            self.csv_file.close()

    def _make_obstacles(self):
        rng = random.Random(self.seed)
        obstacles = []
        margin_x = self.scene_x / 2.0 - 1.0
        margin_y = self.scene_y / 2.0 - 1.0
        attempts = 0
        while len(obstacles) < self.obstacle_count and attempts < self.obstacle_count * 30:
            attempts += 1
            x = rng.uniform(-margin_x, margin_x)
            y = rng.uniform(-margin_y, margin_y)
            radius = rng.uniform(self.r_min, self.r_max)
            if any(math.hypot(x - o.x, y - o.y) < radius + o.radius + self.obstacle_clearance for o in obstacles):
                continue
            obstacles.append(Obstacle(x, y, radius, 0.0, rng.uniform(self.h_min, self.h_max)))
        return obstacles

    def _target_state(self, t):
        phase = self.angular_rate * t
        x = self.path_half_x * math.sin(phase)
        y = self.path_half_y * math.sin(2.0 * phase)
        vx = self.path_half_x * self.angular_rate * math.cos(phase)
        vy = 2.0 * self.path_half_y * self.angular_rate * math.cos(2.0 * phase)
        scale = min(1.0, self.target_speed / max(1.0e-6, math.hypot(vx, vy)))
        return [x, y, self.target_z], [vx * scale, vy * scale, 0.0]

    def _line_of_sight(self, a, b):
        length = distance(a, b)
        samples = max(2, int(math.ceil(length / self.los_step)))
        for i in range(1, samples):
            ratio = float(i) / samples
            x = a[0] + ratio * (b[0] - a[0])
            y = a[1] + ratio * (b[1] - a[1])
            z = a[2] + ratio * (b[2] - a[2])
            for obstacle in self.obstacles:
                if obstacle.z_min <= z <= obstacle.z_max and math.hypot(x - obstacle.x, y - obstacle.y) <= obstacle.radius:
                    return False
        return True

    def _visible(self, uav, target, yaw):
        rel = [target[i] - uav[i] for i in range(3)]
        dist = math.sqrt(sum(v * v for v in rel))
        if dist < self.min_distance or dist > self.max_distance:
            return False
        horizontal = abs(math.atan2(math.sin(math.atan2(rel[1], rel[0]) - yaw),
                                    math.cos(math.atan2(rel[1], rel[0]) - yaw)))
        vertical = abs(math.atan2(rel[2], math.hypot(rel[0], rel[1])))
        return horizontal <= self.half_hfov and vertical <= self.half_vfov and self._line_of_sight(uav, target)

    def _pose(self, point, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = point
        msg.pose.orientation.w = 1.0
        return msg

    def _append_path(self, path, point, stamp, limit=800):
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        path.poses.append(self._pose(point, stamp))
        if len(path.poses) > limit:
            path.poses = path.poses[-limit:]

    def _marker(self, marker_id, marker_type, scale, color, point=None):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "rao_2025"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color = ColorRGBA(*color)
        if point:
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point
        return marker

    def _publish_static_markers(self):
        array = MarkerArray()
        for i, obstacle in enumerate(self.obstacles):
            marker = self._marker(i, Marker.CYLINDER, (2.0 * obstacle.radius, 2.0 * obstacle.radius, obstacle.z_max),
                                  (0.38, 0.42, 0.48, 0.85), (obstacle.x, obstacle.y, obstacle.z_max / 2.0))
            array.markers.append(marker)
        self.marker_pub.publish(array)

    def _publish_dynamic_markers(self, target, shared, visible, yaws, stamp):
        array = MarkerArray()
        for i, obstacle in enumerate(self.obstacles):
            marker = self._marker(i, Marker.CYLINDER, (2.0 * obstacle.radius, 2.0 * obstacle.radius, obstacle.z_max),
                                  (0.38, 0.42, 0.48, 0.85), (obstacle.x, obstacle.y, obstacle.z_max / 2.0))
            marker.header.stamp = stamp
            array.markers.append(marker)
        for i, point in enumerate(self.uav):
            color = (0.18, 0.78, 0.35, 1.0) if visible[i] else (0.95, 0.55, 0.12, 1.0)
            marker = self._marker(1000 + i, Marker.SPHERE, (0.55, 0.55, 0.55), color, point)
            marker.header.stamp = stamp
            array.markers.append(marker)
            line = self._marker(1100 + i, Marker.LINE_STRIP, (0.06, 0.06, 0.06), color)
            line.header.stamp = stamp
            line.points = [Point(*point), Point(*target)]
            array.markers.append(line)
        true_marker = self._marker(1200, Marker.SPHERE, (0.7, 0.7, 0.7), (0.95, 0.15, 0.15, 1.0), target)
        true_marker.header.stamp = stamp
        array.markers.append(true_marker)
        shared_marker = self._marker(1201, Marker.SPHERE, (0.5, 0.5, 0.5), (0.15, 0.65, 0.95, 1.0), shared)
        shared_marker.header.stamp = stamp
        array.markers.append(shared_marker)
        self.marker_pub.publish(array)

    def step(self, t, dt):
        target, velocity = self._target_state(t)
        desired_offsets = [(2.5, 1.8), (-2.5, 0.0), (2.5, -1.8)]
        yaws = []
        for i in range(3):
            # Formation slots follow the target, with a small forward bias.
            goal = [target[0] + desired_offsets[i][0], target[1] + desired_offsets[i][1], self.uav_z]
            max_step = self.uav_max_speed * dt / self.response_time
            previous = list(self.uav[i])
            self.uav[i] = move_towards(self.uav[i], goal, max_step)
            yaws.append(math.atan2(self.uav[i][1] - previous[1], self.uav[i][0] - previous[0]) if distance(previous, self.uav[i]) > 1.0e-5 else 0.0)

        visible = [self._visible(self.uav[i], target, yaws[i]) for i in range(3)]
        if any(visible):
            self.last_estimate = list(target)
            self.last_velocity = list(velocity)
        else:
            self.last_estimate = [self.last_estimate[i] + self.last_velocity[i] * dt for i in range(3)]
        shared = list(self.last_estimate)
        visible_count = sum(visible)
        self.visible_counts[visible_count] += 1
        self.total_samples += 1
        estimation_error = distance(shared, target)
        tracking_error = sum(distance(self.uav[i], target) for i in range(3)) / 3.0
        pair_distances = [distance(self.uav[0], self.uav[1]), distance(self.uav[0], self.uav[2]), distance(self.uav[1], self.uav[2])]
        self.min_pair_distance = min(self.min_pair_distance, min(pair_distances))
        self.sum_estimation_error += estimation_error
        self.sum_tracking_error += tracking_error
        stamp = rospy.Time.now()
        self.target_pub.publish(self._pose(target, stamp))
        self.shared_pub.publish(self._pose(shared, stamp))
        self._append_path(self.target_path, target, stamp)
        self._append_path(self.shared_path, shared, stamp)
        self.target_path_pub.publish(self.target_path)
        self.shared_path_pub.publish(self.shared_path)
        for i in range(3):
            self._append_path(self.uav_paths[i], self.uav[i], stamp)
            self.uav_path_pubs[i].publish(self.uav_paths[i])
        self._publish_dynamic_markers(target, shared, visible, yaws, stamp)
        means = [100.0 * self.visible_counts[i] / max(1, self.total_samples) for i in range(4)]
        self.metrics_pub.publish(Float32MultiArray(data=[float(visible_count), estimation_error, tracking_error,
                                                         self.min_pair_distance, means[3], means[2], means[1], means[0]]))
        if self.csv_writer:
            self.csv_writer.writerow(["%.3f" % t, visible_count, int(visible[0]), int(visible[1]), int(visible[2]),
                                      "%.5f" % estimation_error, "%.5f" % tracking_error, "%.5f" % self.min_pair_distance])
            self.csv_file.flush()

    def run(self):
        rospy.loginfo("Rao 2025 reproduction harness: %dx%d m, %d obstacles, FOV %.1fx%.1f deg",
                      self.scene_x, self.scene_y, len(self.obstacles), math.degrees(2 * self.half_hfov),
                      math.degrees(2 * self.half_vfov))
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        last = start
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1.0 / self.rate_hz, min(0.2, (now - last).to_sec()))
            last = now
            t = (now - start).to_sec()
            self.step(t, dt)
            if self.duration > 0.0 and t >= self.duration:
                break
            rate.sleep()
        rospy.loginfo("Rao harness finished: samples=%d visible[3/2/1/0]=%s mean_error=%.3f m min_pair=%.3f m",
                      self.total_samples, list(reversed(self.visible_counts)),
                      self.sum_estimation_error / max(1, self.total_samples), self.min_pair_distance)


if __name__ == "__main__":
    rospy.init_node("rao_2025_reproduction")
    Rao2025Reproduction().run()
