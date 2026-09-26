#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool, Float64, Header, UInt8
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


def quaternion_multiply(first, second):
    """Hamilton product for (x, y, z, w) tuples."""
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate_by_quaternion(vector, quaternion):
    class Orientation:
        pass
    orientation = Orientation()
    orientation.x, orientation.y, orientation.z, orientation.w = quaternion
    return rotate_offset(vector, orientation)


def orientation_from_yaw(yaw):
    """Create a level body orientation for a planner-commanded yaw."""
    class Orientation:
        pass
    orientation = Orientation()
    orientation.x, orientation.y, orientation.z, orientation.w = \
        quaternion_from_rpy(0.0, 0.0, float(yaw))
    return orientation


def tracking_camera_measurement(observer_position, body_orientation,
                                target_position, camera_contract):
    """Camera-frame FOV result shared by execution visibility telemetry.

    Camera axes are +X forward, +Y left, +Z up.  The fixed extrinsic is the
    camera pose in body coordinates (body_from_camera).
    """
    body_quaternion = (
        float(body_orientation.x), float(body_orientation.y),
        float(body_orientation.z), float(body_orientation.w))
    body_norm = math.sqrt(sum(value * value for value in body_quaternion))
    if body_norm <= 1.0e-12:
        return {
            'valid': False, 'in_front': False, 'range_valid': False,
            'distance': float('nan'), 'horizontal_angle': float('nan'),
            'vertical_angle': float('nan')}
    body_quaternion = tuple(value / body_norm for value in body_quaternion)
    extrinsic_quaternion = quaternion_from_rpy(
        camera_contract['extrinsic_roll'],
        camera_contract['extrinsic_pitch'],
        camera_contract['extrinsic_yaw'])
    world_from_camera = quaternion_multiply(
        body_quaternion, extrinsic_quaternion)
    camera_offset_world = rotate_by_quaternion(
        camera_contract['translation_body'], body_quaternion)
    camera_position = tuple(
        float(observer_position[index]) + camera_offset_world[index]
        for index in range(3))
    relative_world = tuple(
        float(target_position[index]) - camera_position[index]
        for index in range(3))
    inverse_camera = (-world_from_camera[0], -world_from_camera[1],
                      -world_from_camera[2], world_from_camera[3])
    relative_camera = rotate_by_quaternion(relative_world, inverse_camera)
    distance = math.sqrt(sum(value * value for value in relative_camera))
    horizontal = math.atan2(relative_camera[1], relative_camera[0])
    vertical = math.atan2(
        relative_camera[2], math.hypot(relative_camera[0],
                                       relative_camera[1]))
    in_front = relative_camera[0] > 0.0
    range_valid = (camera_contract['min_range'] <= distance <=
                   camera_contract['max_range'])
    fov_valid = (in_front and range_valid and
                 abs(horizontal) <= 0.5 * camera_contract['hfov'] + 1.0e-12 and
                 abs(vertical) <= 0.5 * camera_contract['vfov'] + 1.0e-12)
    return {
        'valid': fov_valid,
        'in_front': in_front,
        'range_valid': range_valid,
        'distance': distance,
        'horizontal_angle': horizontal,
        'vertical_angle': vertical,
    }


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


def wall_geometry(obstacle, default_height=3.0, margin=0.0):
    center = obstacle.get('centerENU', [0.0, 0.0])
    size = obstacle.get('sizeENU', [1.0, 0.2, default_height])
    if len(size) < 2:
        raise ValueError('wall sizeENU needs at least length and thickness')
    height = float(size[2] if len(size) >= 3 else
                   obstacle.get('height', default_height))
    yaw = float(obstacle.get('yawRad', 0.0))
    return (
        float(center[0]), float(center[1]),
        0.5 * float(size[0]) + max(0.0, margin),
        0.5 * float(size[1]) + max(0.0, margin),
        yaw,
        float(obstacle.get('zMin', 0.0)),
        float(obstacle.get('zMin', 0.0)) + height,
    )


def segment_wall_intersects(start, end, wall):
    """Exact segment-vs-yaw-oriented rectangular-prism intersection."""
    cx, cy, half_x, half_y, yaw, z_min, z_max = wall
    cosine, sine = math.cos(yaw), math.sin(yaw)

    def local(point):
        dx, dy = point[0] - cx, point[1] - cy
        return (cosine * dx + sine * dy,
                -sine * dx + cosine * dy,
                point[2])

    first, second = local(start), local(end)
    t_min, t_max = 0.0, 1.0
    for origin, delta, lower, upper in (
            (first[0], second[0] - first[0], -half_x, half_x),
            (first[1], second[1] - first[1], -half_y, half_y),
            (first[2], second[2] - first[2], z_min, z_max)):
        if abs(delta) < 1.0e-12:
            if origin < lower or origin > upper:
                return False
            continue
        enter = (lower - origin) / delta
        leave = (upper - origin) / delta
        if enter > leave:
            enter, leave = leave, enter
        t_min = max(t_min, enter)
        t_max = min(t_max, leave)
        if t_min > t_max:
            return False
    return True


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


def wall_points(obstacle, resolution, default_height=3.0):
    """Fill a wall prism on the same cloud lattice used by the planner map."""
    cx, cy, half_x, half_y, yaw, z_min, z_max = wall_geometry(
        obstacle, default_height)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x_count = max(1, int(math.ceil(2.0 * half_x / resolution)))
    y_count = max(1, int(math.ceil(2.0 * half_y / resolution)))
    z_count = max(1, int(math.ceil((z_max - z_min) / resolution)))
    points = []
    for x_index in range(x_count + 1):
        local_x = -half_x + 2.0 * half_x * x_index / float(x_count)
        for y_index in range(y_count + 1):
            local_y = -half_y + 2.0 * half_y * y_index / float(y_count)
            world_x = cx + cosine * local_x - sine * local_y
            world_y = cy + sine * local_x + cosine * local_y
            for z_index in range(z_count + 1):
                z = z_min + (z_max - z_min) * z_index / float(z_count)
                points.append((world_x, world_y, z))
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
                 moving_obstacle_time_aware_cost_enabled,
                 visibility_occlusion_margin,
                 sync_motion_to_target_start, camera_hfov_deg,
                 camera_image_width, camera_image_height,
                 camera_extrinsic_x, camera_extrinsic_y,
                 camera_extrinsic_z, camera_extrinsic_roll,
                 camera_extrinsic_pitch, camera_extrinsic_yaw):
        with open(scene_file, 'r') as stream:
            self.scene = json.load(stream)
        self.resolution = max(resolution, 0.05)
        self.static_points = []
        for obstacle in (self.scene.get('obstacleData') or {}).values():
            self.static_points.extend(cylinder_points(obstacle, self.resolution))
        default_height = float(
            (self.scene.get('egoPlanner') or {}).get('obstacleHeight', 3.0))
        for wall in (self.scene.get('wallData') or {}).values():
            self.static_points.extend(
                wall_points(wall, self.resolution, default_height))
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
        self.sync_motion_to_target_start = sync_motion_to_target_start
        self.motion_start = (None if self.sync_motion_to_target_start
                             else rospy.Time.now().to_sec())
        self.motion_start_pub = rospy.Publisher(
            '/dynamic/motion_start_time', Float64, queue_size=1, latch=True)
        if self.motion_start is not None:
            self.motion_start_pub.publish(Float64(data=self.motion_start))
        self.dynamic_cylinders = []
        self.camera_hfov_deg = float(camera_hfov_deg)
        self.camera_image_width = float(camera_image_width)
        self.camera_image_height = float(camera_image_height)
        self.camera_vfov = 2.0 * math.atan(
            math.tan(math.radians(self.camera_hfov_deg) / 2.0) *
            self.camera_image_height / self.camera_image_width)
        self.camera_vfov_deg = math.degrees(self.camera_vfov)
        self.fov_half_angle_deg = 0.5 * self.camera_hfov_deg
        self.fov_half_angle = math.radians(self.fov_half_angle_deg)
        self.fov_max_distance = 8.0
        self.camera_contract = {
            'hfov': math.radians(self.camera_hfov_deg),
            'vfov': self.camera_vfov,
            'min_range': 0.2,
            'max_range': self.fov_max_distance,
            'translation_body': (
                float(camera_extrinsic_x), float(camera_extrinsic_y),
                float(camera_extrinsic_z)),
            'extrinsic_roll': float(camera_extrinsic_roll),
            'extrinsic_pitch': float(camera_extrinsic_pitch),
            'extrinsic_yaw': float(camera_extrinsic_yaw),
        }
        self.occlusion_margin = max(float(visibility_occlusion_margin), 0.0)
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
        self.walls = [
            wall_geometry(wall, default_height, self.occlusion_margin)
            for wall in (self.scene.get('wallData') or {}).values()
        ]
        self.raw_walls = [
            wall_geometry(wall, default_height)
            for wall in (self.scene.get('wallData') or {}).values()
        ]
        self.ready = set()
        self.odometry = [None for _ in range(count)]
        self.commands = [None for _ in range(count)]
        self.target_odom = None
        self.tracking_start = None
        self.target_has_moved = False
        self.visibility_finished = False
        self.visibility_elapsed = 0.0
        self.visibility_total = [0 for _ in range(count)]
        self.visibility_visible = [0 for _ in range(count)]
        self.fov_mismatch_total = [0 for _ in range(count)]
        self.fov_mismatch_count = [0 for _ in range(count)]
        self.fov_mismatch_team_total = 0
        self.fov_mismatch_team_count = 0
        self.previous_visibility_time = None
        self.previous_visible_count = None
        self.current_k2_loss_duration = 0.0
        self.current_blackout_duration = 0.0
        self.longest_k2_loss = 0.0
        self.longest_blackout = 0.0
        self.command_times = [None for _ in range(count)]
        self.command_yaw_rates = [None for _ in range(count)]
        self.max_command_yaw_rate = [0.0 for _ in range(count)]
        self.max_command_yaw_acceleration = [0.0 for _ in range(count)]
        self.visibility_buckets = [0 for _ in range(count + 1)]
        self.visibility_csv_path = os.path.abspath(visibility_csv)
        self.visibility_io_lock = threading.Lock()
        stem, extension = os.path.splitext(self.visibility_csv_path)
        self.visibility_summary_path = stem + '_summary' + (extension or '.csv')
        self.visibility_file = open(self.visibility_csv_path, 'w', newline='')
        self.visibility_writer = csv.writer(self.visibility_file)
        visibility_header = ['time_s', 'visible_count'] + [
            'visible_uav{}'.format(index + 1) for index in range(count)]
        for index in range(count):
            uav = index + 1
            visibility_header.extend([
                'uav{}_actual_yaw_rad'.format(uav),
                'uav{}_target_bearing_rad'.format(uav),
                'uav{}_relative_bearing_rad'.format(uav),
                'uav{}_fov_margin_deg'.format(uav),
                'uav{}_commanded_yaw_rad'.format(uav),
                'uav{}_commanded_yaw_dot_rad_s'.format(uav),
                'uav{}_vertical_angle_rad'.format(uav),
                'uav{}_vertical_fov_margin_deg'.format(uav),
                'uav{}_range_valid'.format(uav),
                'uav{}_static_los_clear'.format(uav),
                'uav{}_dynamic_los_clear'.format(uav),
                'uav{}_camera_fov_valid'.format(uav),
                'uav{}_planned_visible'.format(uav),
                'uav{}_planned_yaw_rad'.format(uav),
                'uav{}_planned_camera_fov_valid'.format(uav),
                'uav{}_fov_mismatch'.format(uav),
            ])
        self.visibility_writer.writerow(visibility_header)
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
             'static_clearance_m', 'moving_clearance_m', 'actual_yaw_rad',
             'timestamp', 'trajectory_source', 'trajectory_id', 'generation',
             'team_solution_id', 'encirclement_generation', 'hypothesis_id',
             'source_timestamp', 'safety_validated'])
        self.trajectory_closed = False
        self.execution_sources = {}
        self.execution_source_sub = rospy.Subscriber('/trajectory_execution/source', String,
            self.execution_source_callback, queue_size=100)
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
                '/target_tracking/sim_ready/uav{}'.format(index + 1),
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
        self.visible_count_pub = rospy.Publisher(
            '/native_egov2/visibility/visible_count', UInt8, queue_size=10)
        self.odom_subs = [
            rospy.Subscriber(
                '/drone_{}_visual_slam/odom'.format(index),
                Odometry,
                self.odom_callback,
                callback_args=index,
                queue_size=1)
            for index in range(count)
        ]
        self.command_subs = [
            rospy.Subscriber(
                '/drone_{}_planning/pos_cmd'.format(index),
                PositionCommand,
                self.command_callback,
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
        self.camera_fov_pub = rospy.Publisher(
            '/native_egov2/camera_fov', MarkerArray, queue_size=1)
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
        rospy.loginfo(
            '[alp-camera-yaw] camera_yaw_source=actual_odometry '
            'camera_forward_axis=+X hfov_deg=%.6f vfov_deg=%.6f '
            'image=%.0fx%.0f max_range_m=8.0 min_range_m=0.2 '
            'extrinsic_xyz=(%.6f,%.6f,%.6f) extrinsic_rpy=(%.6f,%.6f,%.6f)',
            self.camera_hfov_deg, self.camera_vfov_deg,
            self.camera_image_width, self.camera_image_height,
            *self.camera_contract['translation_body'],
            self.camera_contract['extrinsic_roll'],
            self.camera_contract['extrinsic_pitch'],
            self.camera_contract['extrinsic_yaw'])
        self.publish_map(None)

    def publish_map(self, _event):
        if rospy.is_shutdown():
            return
        stamp = rospy.Time.now()
        elapsed = self.dynamic_elapsed(stamp.to_sec())
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
        if rospy.is_shutdown():
            return
        markers = MarkerArray()
        elapsed = self.dynamic_elapsed(stamp.to_sec())
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

        for index, obstacle in enumerate((self.scene.get('wallData') or {}).values()):
            center = obstacle.get('centerENU', [0.0, 0.0])
            size = obstacle.get('sizeENU', [1.0, 0.2, 3.0])
            height = float(size[2] if len(size) >= 3 else
                           obstacle.get('height', 3.0))
            z_min = float(obstacle.get('zMin', 0.0))
            yaw = float(obstacle.get('yawRad', 0.0))
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'world'
            marker.ns = 'visibility_walls'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = z_min + 0.5 * height
            marker.pose.orientation.z = math.sin(0.5 * yaw)
            marker.pose.orientation.w = math.cos(0.5 * yaw)
            marker.scale.x = float(size[0])
            marker.scale.y = float(size[1])
            marker.scale.z = height
            marker.color.r = 0.22
            marker.color.g = 0.38
            marker.color.b = 0.78
            marker.color.a = 0.92
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
            rospy.loginfo(
                '[startup-sim-ready] uav=uav%d ready=1 source=simulator_odometry',
                index + 1)

    def command_callback(self, msg, index):
        if math.isfinite(msg.yaw) and math.isfinite(msg.yaw_dot):
            now = rospy.Time.now().to_sec()
            previous_time = self.command_times[index]
            previous_rate = self.command_yaw_rates[index]
            self.max_command_yaw_rate[index] = max(
                self.max_command_yaw_rate[index], abs(float(msg.yaw_dot)))
            if (previous_time is not None and previous_rate is not None and
                    now > previous_time + 1.0e-6):
                acceleration = abs(
                    (float(msg.yaw_dot) - previous_rate) /
                    (now - previous_time))
                if math.isfinite(acceleration):
                    self.max_command_yaw_acceleration[index] = max(
                        self.max_command_yaw_acceleration[index], acceleration)
            self.command_times[index] = now
            self.command_yaw_rates[index] = float(msg.yaw_dot)
            self.commands[index] = (float(msg.yaw), float(msg.yaw_dot))

    def start_time_callback(self, msg):
        if self.tracking_start is None:
            self.tracking_start = float(msg.data)
        if self.sync_motion_to_target_start and self.motion_start is None:
            self.motion_start = float(msg.data)
            self.motion_start_pub.publish(Float64(data=self.motion_start))
            rospy.loginfo(
                '[frozen-runtime-clock] dynamic motion synchronized to target start %.6f',
                self.motion_start)

    def dynamic_elapsed(self, stamp):
        if self.motion_start is None:
            return 0.0
        return max(0.0, float(stamp) - self.motion_start)

    def visibility_measurement(self, odometry, dynamic_cylinders=None,
                               yaw_override=None):
        observer = odometry.pose.pose.position
        target = self.target_odom.pose.pose.position
        dx = target.x - observer.x
        dy = target.y - observer.y
        dz = target.z - observer.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        actual_yaw = quaternion_to_yaw(odometry.pose.pose.orientation)
        yaw = actual_yaw if yaw_override is None else float(yaw_override)
        bearing = math.atan2(dy, dx)
        body_orientation = (odometry.pose.pose.orientation if
                            yaw_override is None else
                            orientation_from_yaw(yaw))
        camera = tracking_camera_measurement(
            (observer.x, observer.y, observer.z),
            body_orientation,
            (target.x, target.y, target.z), self.camera_contract)
        angle_error = camera['horizontal_angle']
        fov_margin_deg = self.fov_half_angle_deg - math.degrees(abs(angle_error))
        vertical_margin_deg = (
            0.5 * self.camera_vfov_deg -
            math.degrees(abs(camera['vertical_angle'])))
        start = (observer.x, observer.y, observer.z)
        end = (target.x, target.y, target.z)
        if dynamic_cylinders is None:
            dynamic_cylinders = self.dynamic_cylinders
        static_los_clear = not any(
            segment_cylinder_intersects(start, end, cylinder)
            for cylinder in self.cylinders)
        if static_los_clear:
            static_los_clear = not any(
                segment_wall_intersects(start, end, wall)
                for wall in self.walls)
        dynamic_los_clear = not any(
            segment_cylinder_intersects(start, end, cylinder)
            for cylinder in dynamic_cylinders)
        visible = (camera['range_valid'] and static_los_clear and
                   dynamic_los_clear and camera['valid'])
        return (visible, actual_yaw, bearing, angle_error, fov_margin_deg,
                camera['vertical_angle'], vertical_margin_deg,
                camera['range_valid'], static_los_clear,
                dynamic_los_clear, camera['valid'])

    def target_visible(self, odometry, dynamic_cylinders=None):
        return self.visibility_measurement(odometry, dynamic_cylinders)[0]

    def current_dynamic_cylinders(self, stamp):
        elapsed = self.dynamic_elapsed(stamp)
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
            with self.visibility_io_lock:
                if not self.visibility_closed:
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
        measurements = [
            self.visibility_measurement(odometry, dynamic_cylinders)
            for odometry in self.odometry
        ]
        planned_measurements = [
            self.visibility_measurement(
                odometry, dynamic_cylinders,
                self.commands[index][0] if self.commands[index] else None)
            for index, odometry in enumerate(self.odometry)
        ]
        visible = [measurement[0] for measurement in measurements]
        planned_visible = [measurement[0]
                           for measurement in planned_measurements]
        visible_count = sum(visible)
        planned_visible_count = sum(planned_visible)
        if (self.previous_visibility_time is not None and
                self.previous_visible_count is not None):
            sample_dt = max(0.0, now - self.previous_visibility_time)
            if self.previous_visible_count < 2:
                self.current_k2_loss_duration += sample_dt
            else:
                self.current_k2_loss_duration = 0.0
            if self.previous_visible_count == 0:
                self.current_blackout_duration += sample_dt
            else:
                self.current_blackout_duration = 0.0
            self.longest_k2_loss = max(
                self.longest_k2_loss, self.current_k2_loss_duration)
            self.longest_blackout = max(
                self.longest_blackout, self.current_blackout_duration)
        self.previous_visibility_time = now
        self.previous_visible_count = visible_count
        self.visible_count_pub.publish(UInt8(data=visible_count))
        self.visibility_buckets[visible_count] += 1
        for index, value in enumerate(visible):
            self.visibility_total[index] += 1
            self.visibility_visible[index] += int(value)
            self.fov_mismatch_total[index] += 1
            actual = measurements[index]
            planned = planned_measurements[index]
            fov_only_mismatch = (
                actual[7] == planned[7] and actual[8] == planned[8] and
                actual[9] == planned[9] and actual[10] != planned[10])
            self.fov_mismatch_count[index] += int(fov_only_mismatch)
            self.visibility_pubs[index].publish(Bool(data=value))
        team_fov_mismatch = (visible_count != planned_visible_count and
                             all(measurements[index][7] ==
                                 planned_measurements[index][7] and
                                 measurements[index][8] ==
                                 planned_measurements[index][8] and
                                 measurements[index][9] ==
                                 planned_measurements[index][9]
                                 for index in range(len(self.odometry))))
        self.fov_mismatch_team_total += 1
        self.fov_mismatch_team_count += int(team_fov_mismatch)
        self.visibility_elapsed = now - self.tracking_start
        with self.visibility_io_lock:
            if self.visibility_closed:
                return
            self.visibility_writer.writerow(
                ['{:.6f}'.format(self.visibility_elapsed), visible_count] +
                [int(value) for value in visible] +
                [formatted for index, measurement in enumerate(measurements)
                 for formatted in (
                     '{:.9f}'.format(measurement[1]),
                     '{:.9f}'.format(measurement[2]),
                     '{:.9f}'.format(measurement[3]),
                     '{:.6f}'.format(measurement[4]),
                     '{:.9f}'.format(self.commands[index][0] if self.commands[index] else 0.0),
                     '{:.9f}'.format(self.commands[index][1] if self.commands[index] else 0.0),
                     '{:.9f}'.format(measurement[5]),
                     '{:.6f}'.format(measurement[6]),
                     int(measurement[7]),
                     int(measurement[8]),
                     int(measurement[9]),
                     int(measurement[10]),
                     int(planned_visible[index]),
                     '{:.9f}'.format(self.commands[index][0]
                                     if self.commands[index] else
                                     measurement[1]),
                     int(planned_measurements[index][10]),
                     int(measurement[10] != planned_measurements[index][10] and
                         measurement[7] == planned_measurements[index][7] and
                         measurement[8] == planned_measurements[index][8] and
                         measurement[9] == planned_measurements[index][9]),
                 )])

        samples = sum(self.visibility_buckets)
        if samples % 30 == 0:
            with self.visibility_io_lock:
                if not self.visibility_closed:
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
            'horizontal_fov_deg',
            'vertical_fov_deg',
            'camera_image_width',
            'camera_image_height',
            'fov_max_distance_m',
            'occlusion_margin_m',
        ]
        values = [
            '{:.6f}'.format(max(0.0, elapsed)),
            samples,
            '{:.3f}'.format(self.fov_half_angle_deg),
            '{:.6f}'.format(self.camera_hfov_deg),
            '{:.6f}'.format(self.camera_vfov_deg),
            '{:.0f}'.format(self.camera_image_width),
            '{:.0f}'.format(self.camera_image_height),
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
                'uav{}_fov_mismatch_rate'.format(index + 1),
            ])
            values.extend([
                visible,
                total - visible,
                '{:.6f}'.format(float(visible) / total if total else 0.0),
                '{:.6f}'.format(
                    float(self.fov_mismatch_count[index]) /
                    self.fov_mismatch_total[index]
                    if self.fov_mismatch_total[index] else 0.0),
            ])
        header.extend(['fov_mismatch_rate_team'])
        values.extend(['{:.6f}'.format(
            float(self.fov_mismatch_team_count) /
            self.fov_mismatch_team_total
            if self.fov_mismatch_team_total else 0.0)])
        header.extend([
            'executed_longest_k2_loss_s',
            'executed_longest_blackout_s',
        ])
        values.extend([
            '{:.6f}'.format(self.longest_k2_loss),
            '{:.6f}'.format(self.longest_blackout),
        ])
        for index in range(len(self.odometry)):
            header.extend([
                'uav{}_max_command_yaw_rate_rad_s'.format(index + 1),
                'uav{}_max_command_yaw_acceleration_rad_s2'.format(index + 1),
            ])
            values.extend([
                '{:.6f}'.format(self.max_command_yaw_rate[index]),
                '{:.6f}'.format(self.max_command_yaw_acceleration[index]),
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
        with self.visibility_io_lock:
            if self.visibility_closed:
                return
            self.visibility_closed = True
            self.visibility_file.flush()
            self.visibility_file.close()
        elapsed = self.visibility_elapsed
        if not self.visibility_finished and self.tracking_start is not None:
            elapsed = rospy.Time.now().to_sec() - self.tracking_start
        self.write_visibility_summary(elapsed)
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
        mismatch_rates = [
            100.0 * self.fov_mismatch_count[index] /
            self.fov_mismatch_total[index]
            if self.fov_mismatch_total[index] else 0.0
            for index in range(len(self.odometry))]
        rospy.loginfo(
            '[alp-fov-mismatch] %s team=%.2f%% planned_vs_executed_fov_only=1',
            ' '.join('uav{}={:.2f}%'.format(index + 1, rate)
                     for index, rate in enumerate(mismatch_rates)),
            100.0 * self.fov_mismatch_team_count /
            self.fov_mismatch_team_total
            if self.fov_mismatch_team_total else 0.0)
        rospy.loginfo(
            '[alp-executed-continuity] EXECUTED_LONGEST_K2_LOSS=%.6f '
            'EXECUTED_LONGEST_BLACKOUT=%.6f',
            self.longest_k2_loss, self.longest_blackout)
        rospy.loginfo(
            '[alp-executed-yaw-dynamics] %s',
            ' '.join(
                'uav{}_max_rate={:.6f},max_acc={:.6f}'.format(
                    index + 1, self.max_command_yaw_rate[index],
                    self.max_command_yaw_acceleration[index])
                for index in range(len(self.odometry))))

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

    @staticmethod
    def point_to_wall_clearance(point, wall):
        x, y, z = point
        cx, cy, half_x, half_y, yaw, z_min, z_max = wall
        cosine, sine = math.cos(yaw), math.sin(yaw)
        dx, dy = x - cx, y - cy
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        horizontal_x = max(abs(local_x) - half_x, 0.0)
        horizontal_y = max(abs(local_y) - half_y, 0.0)
        if z < z_min:
            vertical = z_min - z
        elif z > z_max:
            vertical = z - z_max
        else:
            vertical = 0.0
        return math.sqrt(horizontal_x * horizontal_x +
                         horizontal_y * horizontal_y + vertical * vertical)

    def current_clearances(self, point, elapsed):
        static_clearance = min(
            list(self.point_to_cylinder_clearance(point, cylinder)
                 for cylinder in self.raw_cylinders) +
            list(self.point_to_wall_clearance(point, wall)
                 for wall in self.raw_walls), default=float('inf'))
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

    def execution_source_callback(self, message):
        try:
            source = json.loads(message.data)
            self.execution_sources[int(source['drone_id'])] = source
        except (ValueError, KeyError, TypeError):
            rospy.logwarn_throttle(1.0, 'Invalid executed trajectory source telemetry')

    def record_trajectory(self, now):
        if self.trajectory_closed or rospy.is_shutdown():
            return
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
                point, self.dynamic_elapsed(now))
            target_distance = math.sqrt(
                (pose.x - target_pose.x) ** 2 +
                (pose.y - target_pose.y) ** 2 +
                (pose.z - target_pose.z) ** 2)
            centroid_distance = math.sqrt(
                (centroid[0] - target_pose.x) ** 2 +
                (centroid[1] - target_pose.y) ** 2 +
                (centroid[2] - target_pose.z) ** 2) if centroid else float('nan')
            source = self.execution_sources.get(index, {})
            with self.visibility_io_lock:
                if self.trajectory_closed:
                    return
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
                    '{:.9f}'.format(quaternion_to_yaw(msg.pose.pose.orientation)),
                    now, source.get('trajectory_source', 'OTHER'), source.get('trajectory_id', -1),
                    source.get('generation', 0), source.get('team_solution_id', 0),
                    source.get('encirclement_generation', 0), source.get('hypothesis_id', 0),
                    source.get('timestamp', ''), source.get('safety_validated', False),
                ])
        if int(now * 30.0) % 30 == 0:
            with self.visibility_io_lock:
                if not self.trajectory_closed:
                    self.trajectory_file.flush()

    def close_trajectory(self):
        with self.visibility_io_lock:
            if self.trajectory_closed:
                return
            self.trajectory_closed = True
            self.trajectory_file.flush()
            self.trajectory_file.close()

    def publish_uavs(self, _event):
        if rospy.is_shutdown():
            return
        fov_markers = MarkerArray()
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
            pose = msg.pose.pose
            yaw = (quaternion_to_yaw(pose.orientation) +
                   self.camera_contract['extrinsic_yaw'])
            rospy.loginfo_throttle(
                5.0,
                '[alp-camera-yaw] uav%d actual_yaw_rad=%.4f',
                index + 1, yaw)
            camera_offset = rotate_offset(
                self.camera_contract['translation_body'], pose.orientation)
            origin = (pose.position.x + camera_offset[0],
                      pose.position.y + camera_offset[1],
                      pose.position.z + camera_offset[2])
            color = ((0.15, 0.75, 1.0), (1.0, 0.65, 0.1), (0.75, 0.35, 1.0))[index % 3]
            marker = Marker()
            marker.header = msg.header
            marker.header.frame_id = 'world'
            marker.ns = 'alp_camera_fov/uav{}'.format(index + 1)
            marker.id = 0
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.9
            def add_segment(a, b):
                marker.points.append(Point(x=a[0], y=a[1], z=a[2]))
                marker.points.append(Point(x=b[0], y=b[1], z=b[2]))
            left_yaw = yaw + self.fov_half_angle
            right_yaw = yaw - self.fov_half_angle
            forward = (origin[0] + self.fov_max_distance * math.cos(yaw),
                       origin[1] + self.fov_max_distance * math.sin(yaw), origin[2])
            left = (origin[0] + self.fov_max_distance * math.cos(left_yaw),
                    origin[1] + self.fov_max_distance * math.sin(left_yaw), origin[2])
            right = (origin[0] + self.fov_max_distance * math.cos(right_yaw),
                     origin[1] + self.fov_max_distance * math.sin(right_yaw), origin[2])
            add_segment(origin, forward)
            add_segment(origin, left)
            add_segment(origin, right)
            arc_steps = 16
            previous = left
            for step in range(1, arc_steps + 1):
                angle = left_yaw + (right_yaw - left_yaw) * step / float(arc_steps)
                current = (origin[0] + self.fov_max_distance * math.cos(angle),
                           origin[1] + self.fov_max_distance * math.sin(angle), origin[2])
                add_segment(previous, current)
                previous = current
            fov_markers.markers.append(marker)
        self.camera_fov_pub.publish(fov_markers)
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
    parser.add_argument('--visibility-occlusion-margin', type=float, default=0.08)
    parser.add_argument('--sync-motion-to-target-start', default='false')
    parser.add_argument('--camera-hfov-deg', type=float, default=85.0)
    parser.add_argument('--camera-image-width', type=float, default=1280.0)
    parser.add_argument('--camera-image-height', type=float, default=720.0)
    parser.add_argument('--camera-extrinsic-x', type=float, default=0.0)
    parser.add_argument('--camera-extrinsic-y', type=float, default=0.0)
    parser.add_argument('--camera-extrinsic-z', type=float, default=0.0)
    parser.add_argument('--camera-extrinsic-roll', type=float, default=0.0)
    parser.add_argument('--camera-extrinsic-pitch', type=float, default=0.0)
    parser.add_argument('--camera-extrinsic-yaw', type=float, default=0.0)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('native_egov2_rviz_scene')
    NativeEgoV2RvizScene(
        args.scene_file, args.count, args.resolution, args.visibility_csv,
        args.trajectory_csv,
        parse_bool(args.early_avoidance_enabled),
        parse_bool(args.moving_obstacle_time_aware_cost_enabled),
        args.visibility_occlusion_margin,
        parse_bool(args.sync_motion_to_target_start),
        args.camera_hfov_deg, args.camera_image_width,
        args.camera_image_height, args.camera_extrinsic_x,
        args.camera_extrinsic_y, args.camera_extrinsic_z,
        args.camera_extrinsic_roll, args.camera_extrinsic_pitch,
        args.camera_extrinsic_yaw)
    rospy.spin()


if __name__ == '__main__':
    main()
