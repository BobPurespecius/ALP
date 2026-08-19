#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, Header
from visualization_msgs.msg import Marker, MarkerArray


UAV_COMPONENTS = (
    ('drone250_lidar_up.STL', (0.0, 0.0, 0.0), (0.25, 0.25, 0.28)),
    ('Gemfan Hurricane 51466 CCW.STL', (0.0884, -0.0884, -0.024), (0.05, 0.25, 0.85)),
    ('Gemfan Hurricane 51466 CCW.STL', (-0.0884, 0.0884, -0.024), (0.18, 0.18, 0.20)),
    ('Gemfan Hurricane 51466 CW.STL', (0.0884, 0.0884, -0.024), (0.05, 0.25, 0.85)),
    ('Gemfan Hurricane 51466 CW.STL', (-0.0884, -0.0884, -0.024), (0.18, 0.18, 0.20)),
    ('mid-360-fov-asm.STL', (0.02, 0.0, 0.057), (0.12, 0.12, 0.14)),
)


def rotate_offset(offset, orientation):
    x, y, z = offset
    qx, qy, qz, qw = (
        orientation.x, orientation.y, orientation.z, orientation.w)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def quaternion_to_yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def segment_cylinder_intersects(start, end, cylinder):
    cx, cy, radius, z_min, z_max = cylinder
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    sx = start[0] - cx
    sy = start[1] - cy
    radial_a = dx * dx + dy * dy
    radial_b = 2.0 * (sx * dx + sy * dy)
    radial_c = sx * sx + sy * sy - radius * radius

    if radial_a < 1.0e-9:
        if radial_c > 0.0:
            return False
        radial_t0, radial_t1 = 0.0, 1.0
    else:
        discriminant = radial_b * radial_b - 4.0 * radial_a * radial_c
        if discriminant < 0.0:
            return False
        root = math.sqrt(max(discriminant, 0.0))
        radial_t0 = max(0.0, (-radial_b - root) / (2.0 * radial_a))
        radial_t1 = min(1.0, (-radial_b + root) / (2.0 * radial_a))
        if radial_t0 > radial_t1:
            return False

    if abs(dz) < 1.0e-9:
        if start[2] < z_min or start[2] > z_max:
            return False
        z_t0, z_t1 = 0.0, 1.0
    else:
        z_t0 = max(0.0, min((z_min - start[2]) / dz, (z_max - start[2]) / dz))
        z_t1 = min(1.0, max((z_min - start[2]) / dz, (z_max - start[2]) / dz))
        if z_t0 > z_t1:
            return False

    return max(radial_t0, z_t0) <= min(radial_t1, z_t1)


def cylinder_points(obstacle, resolution, center=None, radius_extra=0.0):
    points = []
    center = center or obstacle.get('centerENU', [0.0, 0.0])
    cx, cy = float(center[0]), float(center[1])
    radius = float(obstacle.get('radius', 0.5)) + max(0.0, radius_extra)
    z_min = float(obstacle.get('zMin', 0.0))
    height = float(obstacle.get('height', 3.0))
    angle_count = max(12, int(math.ceil(2.0 * math.pi * radius / resolution)))
    z_count = max(1, int(math.ceil(height / resolution)))
    for angle_index in range(angle_count):
        angle = 2.0 * math.pi * angle_index / angle_count
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        for z_index in range(z_count + 1):
            points.append((x, y, z_min + min(z_index * resolution, height)))
    return points


def moving_obstacle_state(obstacle, elapsed):
    center = obstacle.get('centerENU', [0.0, 0.0])
    axis = obstacle.get('axisENU', [1.0, 0.0])
    axis_norm = math.hypot(float(axis[0]), float(axis[1]))
    if axis_norm < 1.0e-6:
        axis_x, axis_y = 1.0, 0.0
    else:
        axis_x = float(axis[0]) / axis_norm
        axis_y = float(axis[1]) / axis_norm
    amplitude = float(obstacle.get('amplitude', 0.0))
    omega = 2.0 * math.pi / max(float(obstacle.get('period', 1.0)), 1.0e-6)
    angle = omega * elapsed + float(obstacle.get('phase', 0.0))
    offset = amplitude * math.sin(angle)
    speed = amplitude * omega * math.cos(angle)
    return (
        (float(center[0]) + axis_x * offset,
         float(center[1]) + axis_y * offset),
        (axis_x * speed, axis_y * speed),
    )


def parse_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


class NativeEgoV2RvizScene:
    def __init__(self, scene_file, count, resolution, visibility_csv,
                 trajectory_csv, early_avoidance_enabled,
                 moving_obstacle_time_aware_cost_enabled):
        with open(scene_file, 'r') as stream:
            self.scene = json.load(stream)
        self.resolution = max(resolution, 0.05)
        self.static_points = []
        for obstacle in (self.scene.get('obstacleData') or {}).values():
            self.static_points.extend(cylinder_points(obstacle, self.resolution))
        self.moving_obstacles = list(
            (self.scene.get('movingObstacleData') or {}).values())
        self.time_aware_cost_enabled = moving_obstacle_time_aware_cost_enabled
        self.early_avoidance_enabled = (
            early_avoidance_enabled and not self.time_aware_cost_enabled)
        ego_config = self.scene.get('egoPlanner') or {}
        self.prediction_horizon = float(
            ego_config.get('movingObstaclePredictionHorizon', 3.0))
        self.prediction_dt = max(float(
            ego_config.get('movingObstaclePredictionDt', 0.5)), 0.05)
        self.prediction_radius_growth = float(
            ego_config.get('movingObstaclePredictionRadiusGrowth', 0.04))
        self.prediction_max_extra_radius = float(
            ego_config.get('movingObstaclePredictionMaxExtraRadius', 0.18))
        self.motion_start = rospy.Time.now().to_sec()
        self.motion_start_pub = rospy.Publisher(
            '/dynamic/motion_start_time', Float64, queue_size=1, latch=True)
        self.motion_start_pub.publish(Float64(data=self.motion_start))
        self.dynamic_cylinders = []
        self.fov_half_angle_deg = 42.5
        self.fov_half_angle = math.radians(self.fov_half_angle_deg)
        self.fov_max_distance = 8.0
        self.occlusion_margin = 0.08
        default_height = float((self.scene.get('egoPlanner') or {}).get('obstacleHeight', 3.0))
        self.cylinders = []
        for obstacle in (self.scene.get('obstacleData') or {}).values():
            center = obstacle.get('centerENU', [0.0, 0.0])
            z_min = float(obstacle.get('zMin', 0.0))
            self.cylinders.append((
                float(center[0]),
                float(center[1]),
                float(obstacle.get('radius', 0.5)) + self.occlusion_margin,
                z_min,
                z_min + float(obstacle.get('height', default_height)),
            ))
        self.raw_cylinders = []
        for obstacle in (self.scene.get('obstacleData') or {}).values():
            center = obstacle.get('centerENU', [0.0, 0.0])
            z_min = float(obstacle.get('zMin', 0.0))
            self.raw_cylinders.append((
                float(center[0]), float(center[1]),
                float(obstacle.get('radius', 0.5)), z_min,
                z_min + float(obstacle.get('height', default_height)),
            ))
        self.ready = set()
        self.odometry = [None for _ in range(count)]
        self.target_odom = None
        self.tracking_start = None
        self.target_has_moved = False
        self.visibility_finished = False
        self.visibility_elapsed = 0.0
        self.visibility_total = [0 for _ in range(count)]
        self.visibility_visible = [0 for _ in range(count)]
        self.visibility_buckets = [0 for _ in range(count + 1)]
        self.visibility_csv_path = os.path.abspath(visibility_csv)
        stem, extension = os.path.splitext(self.visibility_csv_path)
        self.visibility_summary_path = stem + '_summary' + (extension or '.csv')
        self.visibility_file = open(self.visibility_csv_path, 'w', newline='')
        self.visibility_writer = csv.writer(self.visibility_file)
        self.visibility_writer.writerow(
            ['time_s', 'visible_count'] +
            ['visible_uav{}'.format(index + 1) for index in range(count)])
        self.visibility_closed = False
        self.trajectory_csv_path = os.path.abspath(trajectory_csv)
        trajectory_dir = os.path.dirname(self.trajectory_csv_path)
        if trajectory_dir:
            os.makedirs(trajectory_dir, exist_ok=True)
        self.trajectory_file = open(self.trajectory_csv_path, 'w', newline='')
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.trajectory_writer.writerow(
            ['time_s', 'uav_id', 'x', 'y', 'z', 'vx', 'vy', 'vz',
             'target_x', 'target_y', 'target_z', 'target_vx', 'target_vy',
             'target_vz', 'target_distance_m', 'centroid_target_distance_m',
             'static_clearance_m', 'moving_clearance_m'])
        self.trajectory_closed = False
        self.map_pub = rospy.Publisher(
            '/map_generator/global_cloud', PointCloud2, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher(
            '/target_tracking/target_marker', Marker, queue_size=1)
        self.obstacle_pub = rospy.Publisher(
            '/native_egov2/obstacles', MarkerArray, queue_size=1, latch=True)
        self.dynamic_pose_pubs = [
            rospy.Publisher(
                '/dynamic/pose_{}'.format(index), PoseStamped, queue_size=10)
            for index in range(len(self.moving_obstacles))
        ]
        self.dynamic_prediction_pubs = [
            rospy.Publisher(
                '/dynamic/prediction_{}'.format(index),
                Path,
                queue_size=1,
                latch=True)
            for index in range(len(self.moving_obstacles))
        ]
        self.dynamic_marker_pub = rospy.Publisher(
            '/dynamic/obj', Marker, queue_size=10)
        self.uav_pubs = [
            rospy.Publisher(
                '/native_egov2/uav{}_marker'.format(index + 1),
                MarkerArray,
                queue_size=1)
            for index in range(count)
        ]
        self.ready_pubs = [
            rospy.Publisher(
                '/target_tracking/ready/uav{}'.format(index + 1),
                Bool,
                queue_size=1,
                latch=True)
            for index in range(count)
        ]
        self.visibility_pubs = [
            rospy.Publisher(
                '/native_egov2/visibility/uav{}'.format(index + 1),
                Bool,
                queue_size=1)
            for index in range(count)
        ]
        self.odom_subs = [
            rospy.Subscriber(
                '/drone_{}_visual_slam/odom'.format(index),
                Odometry,
                self.odom_callback,
                callback_args=index,
                queue_size=1)
            for index in range(count)
        ]
        self.target_sub = rospy.Subscriber(
            '/object_odom', Odometry, self.target_callback, queue_size=1)
        self.start_sub = rospy.Subscriber(
            '/target_tracking/target_start_time',
            Float64,
            self.start_time_callback,
            queue_size=1)
        self.map_timer = rospy.Timer(rospy.Duration(0.2), self.publish_map)
        self.uav_timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_uavs)
        rospy.on_shutdown(self.close_visibility)
        if self.time_aware_cost_enabled:
            avoidance_mode = 'time_aware_cost_gradient'
        elif self.early_avoidance_enabled:
            avoidance_mode = 'point_cloud_augmentation'
        else:
            avoidance_mode = 'no_early_avoidance'
        rospy.loginfo(
            'Native EGOv2 avoidance_mode=%s moving_obstacles=%d',
            avoidance_mode, len(self.moving_obstacles))
        self.publish_map(None)

    def publish_map(self, _event):
        stamp = rospy.Time.now()
        elapsed = stamp.to_sec() - self.motion_start
        states = [
            moving_obstacle_state(obstacle, elapsed)
            for obstacle in self.moving_obstacles
        ]
        points = list(self.static_points)
        for obstacle, (center, _velocity) in zip(self.moving_obstacles, states):
            points.extend(cylinder_points(obstacle, self.resolution, center))
            if self.early_avoidance_enabled:
                tau = self.prediction_dt
                while tau <= self.prediction_horizon + 1.0e-6:
                    predicted_center, _ = moving_obstacle_state(
                        obstacle, elapsed + tau)
                    radius_extra = min(
                        self.prediction_radius_growth * tau,
                        self.prediction_max_extra_radius)
                    points.extend(cylinder_points(
                        obstacle, self.resolution, predicted_center, radius_extra))
                    tau += self.prediction_dt

        header = Header(stamp=stamp, frame_id='world')
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.publish_obstacles(stamp, states)

    def publish_obstacles(self, stamp, states):
        markers = MarkerArray()
        elapsed = stamp.to_sec() - self.motion_start
        for index, obstacle in enumerate((self.scene.get('obstacleData') or {}).values()):
            center = obstacle.get('centerENU', [0.0, 0.0])
            radius = float(obstacle.get('radius', 0.5))
            z_min = float(obstacle.get('zMin', 0.0))
            height = float(obstacle.get('height', 3.0))
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'world'
            marker.ns = 'cylinder_forest'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = z_min + 0.5 * height
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = height
            marker.color.r = 0.12
            marker.color.g = 0.58
            marker.color.b = 0.24
            marker.color.a = 0.88
            markers.markers.append(marker)

        self.dynamic_cylinders = []
        for index, (obstacle, (center, velocity)) in enumerate(
                zip(self.moving_obstacles, states)):
            radius = float(obstacle.get('radius', 0.5))
            z_min = float(obstacle.get('zMin', 0.0))
            height = float(obstacle.get('height', 3.0))
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'world'
            marker.ns = 'dynamic_obstacles'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = z_min + 0.5 * height
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = height
            marker.color.r = 0.95
            marker.color.g = 0.65
            marker.color.b = 0.05
            marker.color.a = 0.9
            markers.markers.append(marker)
            self.dynamic_marker_pub.publish(marker)

            pose = PoseStamped()
            pose.header = marker.header
            pose.pose = marker.pose
            self.dynamic_pose_pubs[index].publish(pose)
            if self.time_aware_cost_enabled:
                prediction = Path()
                prediction.header.stamp = stamp
                prediction.header.frame_id = 'world'
                tau = 0.0
                while tau <= self.prediction_horizon + 1.0e-6:
                    predicted_center, _ = moving_obstacle_state(
                        obstacle, elapsed + tau)
                    predicted_pose = PoseStamped()
                    predicted_pose.header.stamp = stamp + rospy.Duration(tau)
                    predicted_pose.header.frame_id = 'world'
                    predicted_pose.pose.position.x = predicted_center[0]
                    predicted_pose.pose.position.y = predicted_center[1]
                    predicted_pose.pose.position.z = z_min + 0.5 * height
                    predicted_pose.pose.orientation.w = 1.0
                    prediction.poses.append(predicted_pose)
                    tau += self.prediction_dt
                self.dynamic_prediction_pubs[index].publish(prediction)
            self.dynamic_cylinders.append((
                center[0], center[1], radius + self.occlusion_margin,
                z_min, z_min + height))
        self.obstacle_pub.publish(markers)

    def odom_callback(self, msg, index):
        self.odometry[index] = msg
        if index not in self.ready:
            self.ready.add(index)
            self.ready_pubs[index].publish(Bool(data=True))

    def start_time_callback(self, msg):
        if self.tracking_start is None:
            self.tracking_start = float(msg.data)

    def target_visible(self, odometry, dynamic_cylinders=None):
        observer = odometry.pose.pose.position
        target = self.target_odom.pose.pose.position
        dx = target.x - observer.x
        dy = target.y - observer.y
        distance = math.hypot(dx, dy)
        if distance < 0.2 or distance > self.fov_max_distance:
            return False

        yaw = quaternion_to_yaw(odometry.pose.pose.orientation)
        bearing = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        if abs(angle_error) > self.fov_half_angle:
            return False

        start = (observer.x, observer.y, observer.z)
        end = (target.x, target.y, target.z)
        if dynamic_cylinders is None:
            dynamic_cylinders = self.dynamic_cylinders
        return not any(
            segment_cylinder_intersects(start, end, cylinder)
            for cylinder in self.cylinders + dynamic_cylinders)

    def current_dynamic_cylinders(self, stamp):
        elapsed = stamp - self.motion_start
        cylinders = []
        for obstacle in self.moving_obstacles:
            center, _ = moving_obstacle_state(obstacle, elapsed)
            z_min = float(obstacle.get('zMin', 0.0))
            height = float(obstacle.get('height', 3.0))
            cylinders.append((
                center[0], center[1],
                float(obstacle.get('radius', 0.5)) + self.occlusion_margin,
                z_min, z_min + height))
        return cylinders

    def sample_visibility(self):
        if (self.visibility_finished or self.tracking_start is None or
                self.target_odom is None or
                any(odometry is None for odometry in self.odometry)):
            return
        now = rospy.Time.now().to_sec()
        if now < self.tracking_start:
            return

        velocity = self.target_odom.twist.twist.linear
        target_speed = math.sqrt(
            velocity.x * velocity.x +
            velocity.y * velocity.y +
            velocity.z * velocity.z)
        if not self.target_has_moved:
            if target_speed <= 0.05:
                return
            self.target_has_moved = True
        elif target_speed <= 0.05:
            self.visibility_finished = True
            self.visibility_file.flush()
            self.write_visibility_summary(self.visibility_elapsed)
            rospy.loginfo(
                'Visibility sampling finished with target motion: samples=%d',
                sum(self.visibility_buckets))
            return

        # The map publisher is intentionally low-rate. Visibility uses the
        # obstacle state at this sample time so stale map markers cannot create
        # false reacquisition samples.
        dynamic_cylinders = self.current_dynamic_cylinders(now)
        visible = [
            self.target_visible(odometry, dynamic_cylinders)
            for odometry in self.odometry
        ]
        visible_count = sum(visible)
        self.visibility_buckets[visible_count] += 1
        for index, value in enumerate(visible):
            self.visibility_total[index] += 1
            self.visibility_visible[index] += int(value)
            self.visibility_pubs[index].publish(Bool(data=value))
        self.visibility_elapsed = now - self.tracking_start
        self.visibility_writer.writerow(
            ['{:.6f}'.format(self.visibility_elapsed), visible_count] +
            [int(value) for value in visible])

        samples = sum(self.visibility_buckets)
        if samples % 30 == 0:
            self.visibility_file.flush()
            self.write_visibility_summary(self.visibility_elapsed)
            rates = [
                100.0 * self.visibility_visible[index] / self.visibility_total[index]
                for index in range(len(self.odometry))
            ]
            rospy.loginfo_throttle(
                10.0,
                'Visibility samples=%d %s',
                samples,
                ' '.join('uav{}={:.1f}%'.format(index + 1, rate)
                         for index, rate in enumerate(rates)))

    def write_visibility_summary(self, elapsed):
        samples = sum(self.visibility_buckets)
        header = [
            'elapsed_s',
            'samples',
            'fov_half_angle_deg',
            'fov_max_distance_m',
            'occlusion_margin_m',
        ]
        values = [
            '{:.6f}'.format(max(0.0, elapsed)),
            samples,
            '{:.3f}'.format(self.fov_half_angle_deg),
            '{:.3f}'.format(self.fov_max_distance),
            '{:.3f}'.format(self.occlusion_margin),
        ]
        for index in range(len(self.odometry)):
            visible = self.visibility_visible[index]
            total = self.visibility_total[index]
            header.extend([
                'uav{}_visible_samples'.format(index + 1),
                'uav{}_occluded_samples'.format(index + 1),
                'uav{}_visibility_ratio'.format(index + 1),
            ])
            values.extend([
                visible,
                total - visible,
                '{:.6f}'.format(float(visible) / total if total else 0.0),
            ])
        for visible_count, bucket_samples in enumerate(self.visibility_buckets):
            header.extend([
                'visible_{}_samples'.format(visible_count),
                'visible_{}_ratio'.format(visible_count),
            ])
            values.extend([
                bucket_samples,
                '{:.6f}'.format(float(bucket_samples) / samples if samples else 0.0),
            ])
        with open(self.visibility_summary_path, 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerow(values)

    def close_visibility(self):
        if self.visibility_closed:
            return
        self.visibility_closed = True
        elapsed = self.visibility_elapsed
        if not self.visibility_finished and self.tracking_start is not None:
            elapsed = rospy.Time.now().to_sec() - self.tracking_start
        self.write_visibility_summary(elapsed)
        self.visibility_file.flush()
        self.visibility_file.close()
        self.close_trajectory()
        rates = [
            100.0 * self.visibility_visible[index] / self.visibility_total[index]
            if self.visibility_total[index] else 0.0
            for index in range(len(self.odometry))
        ]
        rospy.loginfo(
            'Final visibility: samples=%d %s',
            sum(self.visibility_buckets),
            ' '.join('uav{}={:.2f}%'.format(index + 1, rate)
                     for index, rate in enumerate(rates)))

    @staticmethod
    def point_to_cylinder_clearance(point, cylinder):
        x, y, z = point
        cx, cy, radius, z_min, z_max = cylinder
        radial_clearance = math.hypot(x - cx, y - cy) - radius
        if z < z_min:
            vertical_clearance = z_min - z
        elif z > z_max:
            vertical_clearance = z - z_max
        else:
            vertical_clearance = 0.0
        if radial_clearance <= 0.0 and vertical_clearance > 0.0:
            return vertical_clearance
        if vertical_clearance <= 0.0 and radial_clearance > 0.0:
            return radial_clearance
        return math.hypot(max(0.0, radial_clearance), vertical_clearance)

    def current_clearances(self, point, elapsed):
        static_clearance = min(
            (self.point_to_cylinder_clearance(point, cylinder)
             for cylinder in self.raw_cylinders), default=float('inf'))
        moving_cylinders = []
        for obstacle in self.moving_obstacles:
            center, _ = moving_obstacle_state(obstacle, elapsed)
            z_min = float(obstacle.get('zMin', 0.0))
            moving_cylinders.append((
                center[0], center[1], float(obstacle.get('radius', 0.5)),
                z_min, z_min + float(obstacle.get('height', 3.0))))
        moving_clearance = min(
            (self.point_to_cylinder_clearance(point, cylinder)
             for cylinder in moving_cylinders), default=float('inf'))
        return static_clearance, moving_clearance

    def record_trajectory(self, now):
        if self.tracking_start is None or self.target_odom is None:
            return
        elapsed = now - self.tracking_start
        target_pose = self.target_odom.pose.pose.position
        target_velocity = self.target_odom.twist.twist.linear
        positions = [
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z)
            for msg in self.odometry if msg is not None]
        centroid = None
        if positions:
            centroid = tuple(sum(values) / len(values) for values in zip(*positions))
        for index, msg in enumerate(self.odometry):
            if msg is None:
                continue
            pose = msg.pose.pose.position
            velocity = msg.twist.twist.linear
            point = (pose.x, pose.y, pose.z)
            static_clearance, moving_clearance = self.current_clearances(
                point, now - self.motion_start)
            target_distance = math.sqrt(
                (pose.x - target_pose.x) ** 2 +
                (pose.y - target_pose.y) ** 2 +
                (pose.z - target_pose.z) ** 2)
            centroid_distance = math.sqrt(
                (centroid[0] - target_pose.x) ** 2 +
                (centroid[1] - target_pose.y) ** 2 +
                (centroid[2] - target_pose.z) ** 2) if centroid else float('nan')
            self.trajectory_writer.writerow([
                '{:.6f}'.format(elapsed), index + 1,
                '{:.6f}'.format(pose.x), '{:.6f}'.format(pose.y),
                '{:.6f}'.format(pose.z), '{:.6f}'.format(velocity.x),
                '{:.6f}'.format(velocity.y), '{:.6f}'.format(velocity.z),
                '{:.6f}'.format(target_pose.x), '{:.6f}'.format(target_pose.y),
                '{:.6f}'.format(target_pose.z), '{:.6f}'.format(target_velocity.x),
                '{:.6f}'.format(target_velocity.y), '{:.6f}'.format(target_velocity.z),
                '{:.6f}'.format(target_distance), '{:.6f}'.format(centroid_distance),
                '{:.6f}'.format(static_clearance), '{:.6f}'.format(moving_clearance),
            ])
        if int(now * 30.0) % 30 == 0:
            self.trajectory_file.flush()

    def close_trajectory(self):
        if self.trajectory_closed:
            return
        self.trajectory_closed = True
        self.trajectory_file.flush()
        self.trajectory_file.close()

    def publish_uavs(self, _event):
        for index, msg in enumerate(self.odometry):
            if msg is None:
                continue

            markers = MarkerArray()
            pose = msg.pose.pose
            for component_id, (mesh, offset, color) in enumerate(UAV_COMPONENTS):
                marker = Marker()
                marker.header = msg.header
                marker.header.frame_id = 'world'
                marker.ns = 'neverlost_uav_{}'.format(index + 1)
                marker.id = component_id
                marker.type = Marker.MESH_RESOURCE
                marker.action = Marker.ADD
                dx, dy, dz = rotate_offset(offset, pose.orientation)
                marker.pose.position.x = pose.position.x + dx
                marker.pose.position.y = pose.position.y + dy
                marker.pose.position.z = pose.position.z + dz
                marker.pose.orientation = pose.orientation
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                marker.mesh_resource = (
                    'package://multi_uav_formation/px4_models/neverlost/meshes/' + mesh)
                markers.markers.append(marker)
            self.uav_pubs[index].publish(markers)
        self.sample_visibility()
        self.record_trajectory(rospy.Time.now().to_sec())

    def target_callback(self, msg):
        self.target_odom = msg
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = 'world'
        marker.ns = 'tracking_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.02
        marker.color.a = 1.0
        self.target_pub.publish(marker)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-file', required=True)
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--resolution', type=float, default=0.18)
    parser.add_argument('--visibility-csv', required=True)
    parser.add_argument('--trajectory-csv', required=True)
    parser.add_argument('--early-avoidance-enabled', default='false')
    parser.add_argument(
        '--moving-obstacle-time-aware-cost-enabled', default='false')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('native_egov2_rviz_scene')
    NativeEgoV2RvizScene(
        args.scene_file, args.count, args.resolution, args.visibility_csv,
        args.trajectory_csv,
        parse_bool(args.early_avoidance_enabled),
        parse_bool(args.moving_obstacle_time_aware_cost_enabled))
    rospy.spin()


if __name__ == '__main__':
    main()
