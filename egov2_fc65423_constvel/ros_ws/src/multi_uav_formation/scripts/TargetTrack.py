#!/usr/bin/env python3

import math
import os
import sys
import threading
import time
import csv
import json

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Float64, Header
from visualization_msgs.msg import Marker, MarkerArray

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    PositionCommand = None

pkg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
module_path = os.path.join(pkg_path, 'multi_uav_formation')
sys.path.append(module_path)

from models.P230.P230 import P230


class TargetTrack:
    def __init__(self):
        self.uav_name = rospy.get_param('~uav_name', 'uav1')
        self.rate_hz = float(rospy.get_param('~rate', 30.0))
        self.takeoff_height = float(rospy.get_param('~takeoff_height', 1.2))
        self.target_height = float(rospy.get_param('~target_height', 1.2))
        self.follow_distance = float(rospy.get_param('~follow_distance', 1.2))
        self.goal_tolerance = float(rospy.get_param('~goal_tolerance', 0.35))
        self.max_vel_xy = float(rospy.get_param('~max_vel_xy', 0.9))
        self.max_vel_z = float(rospy.get_param('~max_vel_z', 0.45))
        self.position_gain = float(rospy.get_param('~position_gain', 0.65))
        self.track_position_gain_xy = float(rospy.get_param('~track_position_gain_xy', self.position_gain))
        self.track_position_gain_z = float(rospy.get_param('~track_position_gain_z', 0.45))
        self.track_vertical_damping = float(rospy.get_param('~track_vertical_damping', 0.45))
        self.track_max_vel_z = float(rospy.get_param('~track_max_vel_z', 0.25))
        self.use_track_pose_setpoint = bool(rospy.get_param('~use_track_pose_setpoint', True))
        self.track_altitude_compensation = bool(rospy.get_param('~track_altitude_compensation', True))
        self.track_altitude_comp_gain = float(rospy.get_param('~track_altitude_comp_gain', 1.8))
        self.track_altitude_comp_damping = float(rospy.get_param('~track_altitude_comp_damping', 0.8))
        self.track_min_control_z = float(rospy.get_param('~track_min_control_z', 0.25))
        self.track_max_control_z = float(rospy.get_param('~track_max_control_z', self.target_height))
        self.takeoff_z_tolerance = float(rospy.get_param('~takeoff_z_tolerance', 0.08))
        self.takeoff_speed_tolerance = float(rospy.get_param('~takeoff_speed_tolerance', 0.08))
        self.takeoff_settle_time = float(rospy.get_param('~takeoff_settle_time', 0.8))
        self.offboard_retry_interval = float(rospy.get_param('~offboard_retry_interval', 2.0))
        self.arm_retry_interval = float(rospy.get_param('~arm_retry_interval', 2.5))
        self.yaw_gain = float(rospy.get_param('~yaw_gain', 1.0))
        self.start_x = rospy.get_param('~start_x', None)
        self.start_y = rospy.get_param('~start_y', None)
        self.target_model = rospy.get_param('~target_model', '')
        self.use_gazebo_target = bool(rospy.get_param('~use_gazebo_target', False))
        self.target_radius_x = float(rospy.get_param('~target_radius_x', 1.8))
        self.target_radius_y = float(rospy.get_param('~target_radius_y', 1.0))
        self.target_center_x = float(rospy.get_param('~target_center_x', 2.5))
        self.target_center_y = float(rospy.get_param('~target_center_y', 0.0))
        self.target_omega = float(rospy.get_param('~target_omega', 0.18))
        self.target_mode = rospy.get_param('~target_mode', 'waypoints')
        self.target_speed = float(rospy.get_param('~target_speed', 0.35))
        self.target_waypoints_use_z = bool(rospy.get_param('~target_waypoints_use_z', False))
        self.target_segment_speeds = self.parse_segment_speeds(
            rospy.get_param('~target_segment_speeds', '')
        )
        self.target_waypoints = self.parse_waypoints(
            rospy.get_param(
                '~target_waypoints',
                '0.4,-0.8;1.4,-1.5;3.0,-1.5;4.2,-0.4;5.5,0.6;6.7,0.8;8.6,0.6;10.0,0.2',
            )
        )
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.show_gazebo_target = bool(rospy.get_param('~show_gazebo_target', True))
        self.gazebo_target_model = rospy.get_param('~gazebo_target_model', 'virtual_tracking_target')
        self.target_visual_radius = float(rospy.get_param('~target_visual_radius', 0.25))
        self.track_log_file = rospy.get_param('~track_log_file', '')
        self.track_log_suffix = rospy.get_param('~track_log_suffix', '')
        self.use_ego_planner = bool(rospy.get_param('~use_ego_planner', True))
        self.use_gazebo_truth_for_feedback = bool(rospy.get_param('~use_gazebo_truth_for_feedback', True))
        self.use_gazebo_truth_velocity = bool(rospy.get_param(
            '~use_gazebo_truth_velocity',
            self.use_gazebo_truth_for_feedback,
        ))
        self.gazebo_velocity_alpha = float(rospy.get_param('~gazebo_velocity_alpha', 0.35))
        self.gazebo_velocity_alpha = float(np.clip(self.gazebo_velocity_alpha, 0.0, 1.0))
        self.ego_drone_id = int(rospy.get_param('~ego_drone_id', 0))
        self.ego_odom_topic = rospy.get_param('~ego_odom_topic', '/drone_{}_visual_slam/odom'.format(self.ego_drone_id))
        self.ego_object_odom_topic = rospy.get_param('~ego_object_odom_topic', '/object_odom')
        self.shared_target_timeout = max(float(rospy.get_param('~shared_target_timeout', 1.0)), 0.05)
        self.ego_position_cmd_topic = rospy.get_param(
            '~ego_position_cmd_topic',
            '/drone_{}_planning/pos_cmd'.format(self.ego_drone_id),
        )
        self.gazebo_pose_topic = rospy.get_param(
            '~gazebo_pose_topic',
            '/uav_{}/gazebo_pose'.format(self.ego_drone_id + 1),
        )
        self.ego_command_timeout = float(rospy.get_param('~ego_command_timeout', 1.0))
        self.ego_position_gain = float(rospy.get_param('~ego_position_gain', 0.8))
        self.ego_velocity_gain = float(rospy.get_param('~ego_velocity_gain', 0.1))
        self.ego_max_vel_xy = float(rospy.get_param('~ego_max_vel_xy', self.max_vel_xy))
        self.ego_max_vel_z = float(rospy.get_param('~ego_max_vel_z', self.max_vel_z))
        self.lock_ego_tracking_height = bool(rospy.get_param('~lock_ego_tracking_height', True))
        self.ego_tracking_height = float(rospy.get_param('~ego_tracking_height', self.target_height))
        self.relative_tracking_p = np.array([
            float(rospy.get_param('~relative_tracking_x', -self.follow_distance)),
            float(rospy.get_param('~relative_tracking_y', 0.0)),
            float(rospy.get_param('~relative_tracking_z', 0.0)),
        ], dtype=float)
        self.publish_object_odom = bool(rospy.get_param('~publish_object_odom', True))
        self.fov_sharing_enabled = bool(rospy.get_param('~fov_sharing_enabled', False))
        self.use_external_target_state = bool(rospy.get_param('~use_external_target_state', False))
        self.target_ground_truth_topic = rospy.get_param(
            '~target_ground_truth_topic',
            '/target_tracking/ground_truth',
        )
        self.target_observation_topic = rospy.get_param(
            '~target_observation_topic',
            '/target_tracking/observations/{}'.format(self.uav_name),
        )
        self.target_ready_topic = rospy.get_param(
            '~target_ready_topic',
            '/target_tracking/ready/{}'.format(self.uav_name),
        )
        self.target_start_time_topic = rospy.get_param(
            '~target_start_time_topic',
            '/target_tracking/target_start_time',
        )
        self.external_target_timeout = max(float(rospy.get_param('~external_target_timeout', 1.0)), 0.05)
        self.fov_half_angle = math.radians(float(rospy.get_param('~fov_half_angle_deg', 60.0)))
        self.fov_max_distance = float(rospy.get_param('~fov_max_distance', 8.0))
        self.fov_min_distance = float(rospy.get_param('~fov_min_distance', 0.2))
        self.fov_occlusion_margin = float(rospy.get_param('~fov_occlusion_margin', 0.08))
        self.fov_use_velocity_yaw = bool(rospy.get_param('~fov_use_velocity_yaw', False))
        self.fov_velocity_yaw_weight = float(rospy.get_param('~fov_velocity_yaw_weight', 1.0))
        self.fov_visibility_warmup = float(rospy.get_param('~fov_visibility_warmup', 1.2))
        self.scene_file = rospy.get_param('~scene_file', '')
        self.scene_config = self.load_scene_config(self.scene_file)
        self.obstacle_data = self.scene_config.get('obstacleData') or {}
        self.box_obstacle_data = self.scene_config.get('boxObstacleData') or {}
        self.moving_obstacle_data = self.scene_config.get('movingObstacleData') or {}
        scene_ego = self.scene_config.get('egoPlanner') or {}
        self.publish_ego_obstacle_cloud = bool(rospy.get_param('~publish_ego_obstacle_cloud', True))
        self.ego_cloud_topic = rospy.get_param(
            '~ego_cloud_topic',
            '/drone_{}_pcl_render_node/cloud'.format(self.ego_drone_id),
        )
        self.ego_cloud_resolution = float(rospy.get_param(
            '~ego_cloud_resolution',
            scene_ego.get('cloudResolution', 0.2),
        ))
        self.ego_obstacle_height = float(rospy.get_param(
            '~ego_obstacle_height',
            scene_ego.get('obstacleHeight', 3.0),
        ))
        self.ego_cloud_publish_period = float(rospy.get_param(
            '~ego_cloud_publish_period',
            scene_ego.get('cloudPublishPeriod', 0.5),
        ))
        self.ego_last_cloud_publish_time = 0.0
        if self.use_ego_planner and PositionCommand is None:
            raise ImportError('quadrotor_msgs/PositionCommand is required for EGO-Planner-v2 tracking')

        self.model_states = None
        self.gazebo_pose = None
        self.gazebo_velocity = np.zeros(3, dtype=float)
        self.last_gazebo_position = None
        self.last_gazebo_time = None
        self.ego_last_cmd = None
        self.ego_last_cmd_time = None
        self.last_control_velocity = np.zeros(3, dtype=float)
        self.last_control_velocity_valid = False
        self.last_follow_point = None
        self.last_target_visible = False
        self.last_object_odom_published = False
        self.last_target_yaw = 0.0
        self.external_target_odom = None
        self.external_target_received_at = None
        self.shared_target_odom = None
        self.shared_target_received_at = None
        self.last_shared_target = None
        self.last_shared_target_yaw = 0.0
        self.shared_track_start_ros = None
        self.model_lock = threading.Lock()
        self.start_time = rospy.Time.now()
        self.stage = 'INIT'
        self.stage_start = time.time()
        self.track_start_time = None
        self.takeoff_settle_start = None
        self.takeoff_point = None
        self.offboard_requested = False
        self.armed_requested = False
        self.last_offboard_request_time = None
        self.last_arm_request_time = None
        self.drone = None

        self.target_pub = rospy.Publisher('~target_pose', PoseStamped, queue_size=10)
        self.true_target_pub = rospy.Publisher('~true_target_pose', PoseStamped, queue_size=10)
        self.error_pub = rospy.Publisher('~tracking_error', Float32, queue_size=10)
        self.track_pose_pub = rospy.Publisher(f'/{self.uav_name}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/target_tracking/target_marker', Marker, queue_size=10)
        self.marker_array_pub = rospy.Publisher('/target_tracking/markers', MarkerArray, queue_size=10)
        self.gazebo_target_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.target_observation_pub = rospy.Publisher(self.target_observation_topic, Odometry, queue_size=10)
        self.target_ready_pub = rospy.Publisher(self.target_ready_topic, Bool, queue_size=1, latch=True)
        if self.use_ego_planner:
            self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=20)
            self.ego_object_odom_pub = (
                rospy.Publisher(self.ego_object_odom_topic, Odometry, queue_size=20)
                if self.publish_object_odom else None
            )
            self.ego_cloud_pub = (
                rospy.Publisher(self.ego_cloud_topic, PointCloud2, queue_size=1, latch=True)
                if self.publish_ego_obstacle_cloud else None
            )
            rospy.Subscriber(self.ego_position_cmd_topic, PositionCommand, self.ego_position_cmd_cb, queue_size=10)
        else:
            self.ego_odom_pub = None
            self.ego_object_odom_pub = None
            self.ego_cloud_pub = None
        if self.use_gazebo_target:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb, queue_size=1)
        if self.use_external_target_state:
            rospy.Subscriber(
                self.target_ground_truth_topic,
                Odometry,
                self.external_target_cb,
                queue_size=10,
            )
            rospy.Subscriber(
                self.target_start_time_topic,
                Float64,
                self.target_start_time_cb,
                queue_size=1,
            )
            rospy.Subscriber(
                self.ego_object_odom_topic,
                Odometry,
                self.shared_target_cb,
                queue_size=10,
            )
        rospy.Subscriber(self.gazebo_pose_topic, PoseStamped, self.gazebo_pose_cb, queue_size=1)

        self.drone = P230(name=self.uav_name)
        self.drone.velMax = np.array([self.max_vel_xy, self.max_vel_xy, self.max_vel_z], dtype=float)
        self.drone.kp = self.position_gain
        self.gazebo_target_spawned = False
        self.track_log_handle = None
        self.track_log_writer = None
        self.open_track_log()
        self.target_ready_pub.publish(Bool(data=False))

    def load_scene_config(self, scene_file):
        if not scene_file or scene_file == '__auto__':
            return {}
        if not os.path.exists(scene_file):
            rospy.logwarn('TargetTrack scene file not found: %s', scene_file)
            return {}
        try:
            with open(scene_file, 'r') as f:
                return json.load(f)
        except (OSError, ValueError) as exc:
            rospy.logwarn('TargetTrack could not load scene file %s: %s', scene_file, exc)
            return {}

    def sorted_obstacle_items(self, obstacle_data):
        def key(item):
            name, _obstacle = item
            try:
                return int(name)
            except ValueError:
                return name

        return sorted((obstacle_data or {}).items(), key=key)

    def moving_obstacle_center(self, obstacle, stamp=None):
        if stamp is None:
            stamp = rospy.Time.now().to_sec()
        center = obstacle.get('centerENU', [0.0, 0.0])
        axis = obstacle.get('axisENU', [1.0, 0.0])
        amplitude = float(obstacle.get('amplitude', 0.0))
        period = max(float(obstacle.get('period', 1.0)), 1e-6)
        phase = float(obstacle.get('phase', 0.0))

        if len(axis) < 2:
            unit_axis = np.array([1.0, 0.0], dtype=float)
        else:
            unit_axis = np.array([float(axis[0]), float(axis[1])], dtype=float)
            norm = np.linalg.norm(unit_axis)
            unit_axis = np.array([1.0, 0.0], dtype=float) if norm < 1e-6 else unit_axis / norm

        offset = amplitude * math.sin(2.0 * math.pi * stamp / period + phase)
        return [float(center[0]) + unit_axis[0] * offset, float(center[1]) + unit_axis[1] * offset]

    def current_obstacle_states(self, stamp=None):
        obstacles = []
        for _name, obstacle in self.sorted_obstacle_items(self.obstacle_data):
            obstacles.append(obstacle)
        for _name, obstacle in self.sorted_obstacle_items(self.box_obstacle_data):
            current = dict(obstacle)
            current['shape'] = 'box'
            obstacles.append(current)

        for _name, obstacle in self.sorted_obstacle_items(self.moving_obstacle_data):
            current = dict(obstacle)
            current['centerENU'] = self.moving_obstacle_center(obstacle, stamp=stamp)
            obstacles.append(current)

        return obstacles

    def segment_point_distance_xy(self, start, end, point):
        vx = end[0] - start[0]
        vy = end[1] - start[1]
        wx = point[0] - start[0]
        wy = point[1] - start[1]
        denom = vx * vx + vy * vy
        if denom < 1e-9:
            return float(np.linalg.norm(np.array(point[:2]) - np.array(start[:2])))
        ratio = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
        closest = np.array([start[0] + ratio * vx, start[1] + ratio * vy], dtype=float)
        return float(np.linalg.norm(closest - np.array(point[:2], dtype=float)))

    def segment_cylinder_intersects(self, start, end, center, radius, z_min, z_max):
        start = np.array(start, dtype=float)
        end = np.array(end, dtype=float)
        direction = end - start
        sx = start[0] - float(center[0])
        sy = start[1] - float(center[1])
        dx = direction[0]
        dy = direction[1]
        radial_a = dx * dx + dy * dy
        radial_b = 2.0 * (sx * dx + sy * dy)
        radial_c = sx * sx + sy * sy - radius * radius

        if radial_a < 1e-9:
            if radial_c > 0.0:
                return False
            radial_t0, radial_t1 = 0.0, 1.0
        else:
            disc = radial_b * radial_b - 4.0 * radial_a * radial_c
            if disc < 0.0:
                return False
            root = math.sqrt(max(disc, 0.0))
            radial_t0 = (-radial_b - root) / (2.0 * radial_a)
            radial_t1 = (-radial_b + root) / (2.0 * radial_a)
            radial_t0, radial_t1 = max(0.0, min(radial_t0, radial_t1)), min(1.0, max(radial_t0, radial_t1))
            if radial_t0 > radial_t1:
                return False

        dz = direction[2]
        if abs(dz) < 1e-9:
            if start[2] < z_min or start[2] > z_max:
                return False
            z_t0, z_t1 = 0.0, 1.0
        else:
            tz0 = (z_min - start[2]) / dz
            tz1 = (z_max - start[2]) / dz
            z_t0 = max(0.0, min(tz0, tz1))
            z_t1 = min(1.0, max(tz0, tz1))
            if z_t0 > z_t1:
                return False

        return max(radial_t0, z_t0) <= min(radial_t1, z_t1)

    def segment_box_intersects(self, start, end, center, size, z_min, yaw, margin):
        start = np.array(start, dtype=float)
        end = np.array(end, dtype=float)
        sx = float(size[0]) if len(size) >= 1 else 1.0
        sy = float(size[1]) if len(size) >= 2 else 1.0
        sz = float(size[2]) if len(size) >= 3 else self.ego_obstacle_height
        box_center = np.array([float(center[0]), float(center[1]), float(z_min) + sz * 0.5], dtype=float)
        c = math.cos(-float(yaw))
        s = math.sin(-float(yaw))

        def to_local(point):
            shifted = np.array(point, dtype=float) - box_center
            return np.array([
                c * shifted[0] - s * shifted[1],
                s * shifted[0] + c * shifted[1],
                shifted[2],
            ], dtype=float)

        p0 = to_local(start)
        p1 = to_local(end)
        direction = p1 - p0
        half = np.array([sx * 0.5 + margin, sy * 0.5 + margin, sz * 0.5 + margin], dtype=float)
        t_min, t_max = 0.0, 1.0
        for axis in range(3):
            if abs(direction[axis]) < 1e-9:
                if p0[axis] < -half[axis] or p0[axis] > half[axis]:
                    return False
                continue
            t0 = (-half[axis] - p0[axis]) / direction[axis]
            t1 = (half[axis] - p0[axis]) / direction[axis]
            t_min = max(t_min, min(t0, t1))
            t_max = min(t_max, max(t0, t1))
            if t_min > t_max:
                return False
        return True

    def target_visible_from_self(self, target, yaw, stamp=None):
        observer = self.feedback_position()
        delta = np.array(target[:2], dtype=float) - observer[:2]
        distance = float(np.linalg.norm(delta))
        if distance < self.fov_min_distance or distance > self.fov_max_distance:
            return False

        bearing = math.atan2(delta[1], delta[0])
        angle_error = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        if abs(angle_error) > self.fov_half_angle:
            return False

        for obstacle in self.current_obstacle_states(stamp=stamp):
            center = obstacle.get('centerENU', [0.0, 0.0])
            if obstacle.get('shape') == 'box':
                size = obstacle.get('sizeENU', [1.0, 1.0, 1.0])
                if self.segment_box_intersects(
                    observer,
                    target,
                    center,
                    size,
                    float(obstacle.get('zMin', 0.0)),
                    float(obstacle.get('yaw', 0.0)),
                    self.fov_occlusion_margin,
                ):
                    return False
            else:
                radius = float(obstacle.get('radius', 0.5)) + self.fov_occlusion_margin
                height = float(obstacle.get('height', self.ego_obstacle_height))
                z_min = float(obstacle.get('zMin', 0.0))
                if self.segment_cylinder_intersects(observer, target, center, radius, z_min, z_min + height):
                    return False
        return True

    def build_ego_obstacle_cloud(self, stamp=None):
        if not (self.obstacle_data or self.box_obstacle_data or self.moving_obstacle_data):
            return []

        resolution = max(float(self.ego_cloud_resolution), 0.05)
        points = []
        for obstacle in self.current_obstacle_states(stamp=stamp):
            center = obstacle.get('centerENU', [0.0, 0.0])
            height = float(obstacle.get('height', self.ego_obstacle_height))
            z_min = float(obstacle.get('zMin', 0.0))
            cx, cy = float(center[0]), float(center[1])

            if obstacle.get('shape') == 'box':
                size = obstacle.get('sizeENU', [1.0, 1.0, height])
                sx = float(size[0])
                sy = float(size[1])
                sz = float(size[2]) if len(size) >= 3 else height
                xs = np.arange(cx - sx * 0.5, cx + sx * 0.5 + resolution * 0.5, resolution)
                ys = np.arange(cy - sy * 0.5, cy + sy * 0.5 + resolution * 0.5, resolution)
                zs = np.arange(z_min, z_min + sz + resolution * 0.5, resolution)
                for x in xs:
                    for y in ys:
                        for z in zs:
                            points.append([float(x), float(y), float(z)])
                continue

            radius = float(obstacle.get('radius', 0.5))

            xs = np.arange(cx - radius, cx + radius + resolution * 0.5, resolution)
            ys = np.arange(cy - radius, cy + radius + resolution * 0.5, resolution)
            zs = np.arange(z_min, z_min + height + resolution * 0.5, resolution)

            for x in xs:
                for y in ys:
                    if (x - cx) ** 2 + (y - cy) ** 2 > radius ** 2:
                        continue
                    for z in zs:
                        points.append([float(x), float(y), float(z)])

        return points

    def publish_ego_obstacle_cloud_msg(self, stamp):
        if self.ego_cloud_pub is None:
            return
        now_sec = stamp.to_sec()
        if now_sec - self.ego_last_cloud_publish_time < self.ego_cloud_publish_period:
            return

        points = self.build_ego_obstacle_cloud(stamp=now_sec)
        if not points:
            self.ego_last_cloud_publish_time = now_sec
            return

        header = Header()
        header.stamp = stamp
        header.frame_id = 'world'
        self.ego_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.ego_last_cloud_publish_time = now_sec

    def parse_waypoints(self, text):
        waypoints = []
        for item in str(text).split(';'):
            item = item.strip()
            if not item:
                continue
            parts = [float(v.strip()) for v in item.split(',')]
            if len(parts) == 2:
                parts.append(self.target_height)
            if len(parts) != 3:
                raise ValueError('target_waypoints item must be x,y or x,y,z: {}'.format(item))
            if not self.target_waypoints_use_z:
                parts[2] = self.target_height
            waypoints.append(np.array(parts, dtype=float))
        if not waypoints:
            waypoints.append(np.array([0.8, 0.0, self.target_height], dtype=float))
        return waypoints

    def parse_segment_speeds(self, text):
        speeds = []
        for item in str(text).split(';'):
            item = item.strip()
            if not item:
                continue
            speeds.append(max(float(item), 1e-3))
        return speeds

    def waypoint_segment_speed(self, segment_index):
        if 0 <= segment_index < len(self.target_segment_speeds):
            return self.target_segment_speeds[segment_index]
        return max(self.target_speed, 1e-3)

    def yaw_to_quaternion(self, yaw):
        return (
            math.cos(yaw * 0.5),
            0.0,
            0.0,
            math.sin(yaw * 0.5),
        )

    def quaternion_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def feedback_yaw(self):
        with self.model_lock:
            gazebo_pose = self.gazebo_pose
        if self.use_gazebo_truth_for_feedback and gazebo_pose is not None:
            return self.quaternion_to_yaw(gazebo_pose.orientation)
        return math.atan2(
            2.0 * (self.drone.meQuaternionENU[0] * self.drone.meQuaternionENU[3] +
                   self.drone.meQuaternionENU[1] * self.drone.meQuaternionENU[2]),
            1.0 - 2.0 * (self.drone.meQuaternionENU[2] ** 2 + self.drone.meQuaternionENU[3] ** 2),
        )

    def yaw_from_velocity(self, velocity, fallback=None):
        speed_xy = float(np.linalg.norm(velocity[:2]))
        if speed_xy > 1e-4:
            yaw = math.atan2(velocity[1], velocity[0])
            self.last_target_yaw = yaw
            return yaw
        if fallback is not None:
            self.last_target_yaw = fallback
            return fallback
        return self.last_target_yaw

    def yaw_from_motion(self, fallback):
        feedback_velocity = self.feedback_velocity()
        if np.linalg.norm(feedback_velocity[:2]) > 0.08:
            return math.atan2(feedback_velocity[1], feedback_velocity[0])
        with self.model_lock:
            cmd = self.ego_last_cmd
        if cmd is not None:
            cmd_velocity = np.array([cmd.velocity.x, cmd.velocity.y, cmd.velocity.z], dtype=float)
            if np.linalg.norm(cmd_velocity[:2]) > 0.08:
                return math.atan2(cmd_velocity[1], cmd_velocity[0])
        return fallback

    def blend_yaw(self, source_yaw, target_yaw, target_weight):
        weight = float(np.clip(target_weight, 0.0, 1.0))
        delta = math.atan2(math.sin(target_yaw - source_yaw), math.cos(target_yaw - source_yaw))
        return source_yaw + weight * delta

    def open_track_log(self):
        if not self.track_log_file:
            return
        if self.track_log_suffix:
            root, ext = os.path.splitext(self.track_log_file)
            self.track_log_file = '{}_{}{}'.format(root, self.track_log_suffix, ext or '.csv')
        directory = os.path.dirname(os.path.abspath(self.track_log_file))
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.track_log_handle = open(self.track_log_file, 'w', newline='')
        self.track_log_writer = csv.DictWriter(
            self.track_log_handle,
            fieldnames=[
                'wall_time',
                'ros_time',
                'track_time',
                'stage',
                'uav_x',
                'uav_y',
                'uav_z',
                'feedback_vx',
                'feedback_vy',
                'feedback_vz',
                'gazebo_x',
                'gazebo_y',
                'gazebo_z',
                'ego_cmd_x',
                'ego_cmd_y',
                'ego_cmd_z',
                'ego_cmd_vx',
                'ego_cmd_vy',
                'ego_cmd_vz',
                'sent_vx',
                'sent_vy',
                'sent_vz',
                'true_target_x',
                'true_target_y',
                'true_target_z',
                'target_x',
                'target_y',
                'target_z',
                'target_source',
                'follow_x',
                'follow_y',
                'follow_z',
                'control_x',
                'control_y',
                'control_z',
                'yaw_rad',
                'feedback_yaw_rad',
                'target_visible',
                'shared_object_odom_published',
                'tracking_error_m',
            ],
        )
        self.track_log_writer.writeheader()
        self.track_log_handle.flush()

    def write_track_log(self, stamp, true_target, target, target_source, follow_point, control_point, yaw, error):
        if self.track_log_writer is None:
            return
        track_time = ''
        if self.use_external_target_state and self.shared_track_start_ros is not None:
            track_time = max(0.0, stamp.to_sec() - self.shared_track_start_ros)
        elif self.track_start_time is not None:
            track_time = max(0.0, time.time() - self.track_start_time)
        with self.model_lock:
            gazebo_pose = self.gazebo_pose
        if gazebo_pose is None:
            gazebo_x = gazebo_y = gazebo_z = ''
        else:
            gazebo_x = gazebo_pose.position.x
            gazebo_y = gazebo_pose.position.y
            gazebo_z = gazebo_pose.position.z
        feedback_position = self.feedback_position()
        feedback_velocity = self.feedback_velocity()
        with self.model_lock:
            ego_cmd = self.ego_last_cmd
        if ego_cmd is None:
            ego_cmd_pos = ego_cmd_vel = ['', '', '']
        else:
            ego_cmd_pos = [ego_cmd.position.x, ego_cmd.position.y, ego_cmd.position.z]
            ego_cmd_vel = [ego_cmd.velocity.x, ego_cmd.velocity.y, ego_cmd.velocity.z]
        if self.last_control_velocity_valid:
            sent_velocity = self.last_control_velocity
        else:
            sent_velocity = ['', '', '']
        self.track_log_writer.writerow({
            'wall_time': time.time(),
            'ros_time': stamp.to_sec(),
            'track_time': track_time,
            'stage': self.stage,
            'uav_x': feedback_position[0],
            'uav_y': feedback_position[1],
            'uav_z': feedback_position[2],
            'feedback_vx': feedback_velocity[0],
            'feedback_vy': feedback_velocity[1],
            'feedback_vz': feedback_velocity[2],
            'gazebo_x': gazebo_x,
            'gazebo_y': gazebo_y,
            'gazebo_z': gazebo_z,
            'ego_cmd_x': ego_cmd_pos[0],
            'ego_cmd_y': ego_cmd_pos[1],
            'ego_cmd_z': ego_cmd_pos[2],
            'ego_cmd_vx': ego_cmd_vel[0],
            'ego_cmd_vy': ego_cmd_vel[1],
            'ego_cmd_vz': ego_cmd_vel[2],
            'sent_vx': sent_velocity[0],
            'sent_vy': sent_velocity[1],
            'sent_vz': sent_velocity[2],
            'true_target_x': true_target[0],
            'true_target_y': true_target[1],
            'true_target_z': true_target[2],
            'target_x': target[0],
            'target_y': target[1],
            'target_z': target[2],
            'target_source': target_source,
            'follow_x': follow_point[0],
            'follow_y': follow_point[1],
            'follow_z': follow_point[2],
            'control_x': control_point[0],
            'control_y': control_point[1],
            'control_z': control_point[2],
            'yaw_rad': yaw,
            'feedback_yaw_rad': self.feedback_yaw(),
            'target_visible': int(bool(self.last_target_visible)),
            'shared_object_odom_published': int(bool(self.last_object_odom_published)),
            'tracking_error_m': error,
        })
        self.track_log_handle.flush()

    def target_sdf(self):
        radius = self.target_visual_radius
        return """
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <visual name="target_visual">
        <geometry>
          <sphere><radius>{radius:.3f}</radius></sphere>
        </geometry>
        <material>
          <ambient>1.0 0.05 0.02 1.0</ambient>
          <diffuse>1.0 0.05 0.02 1.0</diffuse>
          <emissive>0.35 0.02 0.01 1.0</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(name=self.gazebo_target_model, radius=radius)

    def spawn_gazebo_target(self, target):
        if self.gazebo_target_spawned or not self.show_gazebo_target:
            return
        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
            spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            pose = Pose()
            pose.position.x = target[0]
            pose.position.y = target[1]
            pose.position.z = target[2]
            pose.orientation.w = 1.0
            result = spawn(self.gazebo_target_model, self.target_sdf(), '', pose, 'world')
            if result.success:
                rospy.loginfo('Spawned Gazebo target visual model: %s', self.gazebo_target_model)
            else:
                rospy.logwarn('Gazebo target visual spawn returned false: %s', result.status_message)
            self.gazebo_target_spawned = True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn_throttle(5.0, 'Waiting to spawn Gazebo target visual: %s', exc)

    def publish_gazebo_target(self, target):
        if not self.show_gazebo_target:
            return
        self.spawn_gazebo_target(target)
        state = ModelState()
        state.model_name = self.gazebo_target_model
        state.reference_frame = 'world'
        state.pose.position.x = target[0]
        state.pose.position.y = target[1]
        state.pose.position.z = target[2]
        state.pose.orientation.w = 1.0
        self.gazebo_target_pub.publish(state)

    def model_states_cb(self, msg):
        with self.model_lock:
            self.model_states = msg

    def gazebo_pose_cb(self, msg):
        world_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        stamp = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()

        with self.model_lock:
            self.gazebo_pose = msg.pose
            if self.last_gazebo_position is not None and self.last_gazebo_time is not None:
                dt = stamp - self.last_gazebo_time
                if 1e-3 <= dt <= 0.5:
                    raw_velocity = (world_position - self.last_gazebo_position) / dt
                    if np.all(np.isfinite(raw_velocity)):
                        alpha = self.gazebo_velocity_alpha
                        self.gazebo_velocity = alpha * raw_velocity + (1.0 - alpha) * self.gazebo_velocity
                elif dt > 0.5:
                    self.gazebo_velocity = np.zeros(3, dtype=float)
            self.last_gazebo_position = world_position
            self.last_gazebo_time = stamp
        if self.use_gazebo_truth_for_feedback and self.drone is not None:
            self.drone.applyWorldPositionCorrectionENU(world_position)

    def ego_position_cmd_cb(self, msg):
        with self.model_lock:
            self.ego_last_cmd = msg
            self.ego_last_cmd_time = time.time()

    def external_target_cb(self, msg):
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, 'Ignoring non-finite external target state')
            return
        with self.model_lock:
            self.external_target_odom = msg
            self.external_target_received_at = rospy.Time.now()

    def target_start_time_cb(self, msg):
        if math.isfinite(msg.data) and msg.data > 0.0:
            self.shared_track_start_ros = float(msg.data)

    def shared_target_cb(self, msg):
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, 'Ignoring non-finite shared target state')
            return
        with self.model_lock:
            self.shared_target_odom = msg
            self.shared_target_received_at = rospy.Time.now()

    def external_target_state(self, stamp):
        with self.model_lock:
            msg = self.external_target_odom
            received_at = self.external_target_received_at
        if msg is None or received_at is None:
            return None
        if (stamp - received_at).to_sec() > self.external_target_timeout:
            rospy.logwarn_throttle(1.0, 'External target state on %s is stale', self.target_ground_truth_topic)
            return None
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=float)
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ], dtype=float)
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_target_yaw = yaw
        return position, velocity, yaw

    def external_target_is_fresh(self, stamp):
        with self.model_lock:
            received_at = self.external_target_received_at
        return received_at is not None and (stamp - received_at).to_sec() <= self.external_target_timeout

    def shared_target_state(self, stamp):
        with self.model_lock:
            msg = self.shared_target_odom
            received_at = self.shared_target_received_at
        if msg is None or received_at is None:
            return None
        if (stamp - received_at).to_sec() > self.shared_target_timeout:
            return None
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=float)
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ], dtype=float)
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        return position, velocity, yaw, msg.child_frame_id or 'shared_target'

    def feedback_position(self):
        with self.model_lock:
            gazebo_pose = self.gazebo_pose
        if self.use_gazebo_truth_for_feedback and gazebo_pose is not None:
            return np.array([
                gazebo_pose.position.x,
                gazebo_pose.position.y,
                gazebo_pose.position.z,
            ], dtype=float)
        return self.drone.mePositionENU.copy()

    def feedback_velocity(self):
        if self.use_gazebo_truth_for_feedback and self.use_gazebo_truth_velocity:
            with self.model_lock:
                return self.gazebo_velocity.copy()
        return self.drone.meVelocityENU.copy()

    def get_gazebo_target(self):
        if not self.target_model:
            return None
        with self.model_lock:
            msg = self.model_states
        if msg is None or self.target_model not in msg.name:
            return None
        pose = msg.pose[msg.name.index(self.target_model)]
        return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

    def simulated_target_state(self, stamp):
        if self.stage != 'TRACK' or self.track_start_time is None:
            t = 0.0
        else:
            t = max(0.0, time.time() - self.track_start_time)
        position = np.array([
            self.target_center_x + self.target_radius_x * math.sin(self.target_omega * t),
            self.target_center_y + self.target_radius_y * math.sin(0.7 * self.target_omega * t + 0.8),
            self.target_height,
        ], dtype=float)
        velocity = np.array([
            self.target_radius_x * self.target_omega * math.cos(self.target_omega * t),
            self.target_radius_y * 0.7 * self.target_omega * math.cos(0.7 * self.target_omega * t + 0.8),
            0.0,
        ], dtype=float)
        yaw = self.yaw_from_velocity(velocity)
        return position, velocity, yaw

    def simulated_target(self, stamp):
        return self.simulated_target_state(stamp)[0]

    def waypoint_target_state(self):
        fallback_yaw = 0.0
        if len(self.target_waypoints) >= 2:
            first_segment = self.target_waypoints[1] - self.target_waypoints[0]
            if np.linalg.norm(first_segment[:2]) > 1e-6:
                fallback_yaw = math.atan2(first_segment[1], first_segment[0])
        if self.track_start_time is None:
            position = self.target_waypoints[0].copy()
            velocity = np.zeros(3, dtype=float)
            yaw = self.yaw_from_velocity(velocity, fallback=fallback_yaw)
            return position, velocity, yaw

        elapsed = max(0.0, time.time() - self.track_start_time)
        for segment_index, (start, end) in enumerate(zip(self.target_waypoints[:-1], self.target_waypoints[1:])):
            segment = end - start
            length = float(np.linalg.norm(segment))
            if length < 1e-6:
                continue
            speed = self.waypoint_segment_speed(segment_index)
            duration = length / speed
            if elapsed <= duration:
                ratio = elapsed / duration if duration > 1e-6 else 1.0
                position = start + segment * ratio
                direction = segment / length
                velocity = np.array([
                    direction[0] * speed,
                    direction[1] * speed,
                    direction[2] * speed,
                ], dtype=float)
                yaw = self.yaw_from_velocity(velocity)
                return position, velocity, yaw
            elapsed -= duration
        position = self.target_waypoints[-1].copy()
        velocity = np.zeros(3, dtype=float)
        if len(self.target_waypoints) >= 2:
            last_segment = self.target_waypoints[-1] - self.target_waypoints[-2]
            if np.linalg.norm(last_segment[:2]) > 1e-6:
                fallback_yaw = math.atan2(last_segment[1], last_segment[0])
        yaw = self.yaw_from_velocity(velocity, fallback=fallback_yaw)
        return position, velocity, yaw

    def waypoint_target(self):
        return self.waypoint_target_state()[0]

    def target_state(self, stamp):
        if self.use_external_target_state:
            state = self.external_target_state(stamp)
            if state is not None:
                return state
            fallback = self.target_waypoints[0].copy()
            return fallback, np.zeros(3, dtype=float), self.last_target_yaw
        if self.target_mode == 'ellipse':
            return self.simulated_target_state(stamp)
        return self.waypoint_target_state()

    def current_target(self, stamp):
        if self.use_external_target_state:
            state = self.external_target_state(stamp)
            if state is not None:
                return state[0], 'shared_ground_truth'
            return self.target_waypoints[0].copy(), 'target_waiting'
        target = self.get_gazebo_target() if self.use_gazebo_target else None
        if target is None:
            if self.target_mode == 'ellipse':
                target = self.simulated_target(stamp)
            else:
                target = self.waypoint_target()
        if self.target_mode == 'ellipse' or not self.target_waypoints_use_z:
            target[2] = self.target_height
        return target, 'truth_fallback'

    def desired_follow_point(self, target):
        ego_xy = self.feedback_position()[:2]
        target_xy = target[:2]
        delta = ego_xy - target_xy
        norm = np.linalg.norm(delta)
        if norm < 1e-3:
            delta = np.array([-1.0, 0.0])
            norm = 1.0
        follow_xy = target_xy + self.follow_distance * delta / norm
        return np.array([follow_xy[0], follow_xy[1], target[2]], dtype=float)

    def ego_tracking_point(self, target, target_yaw):
        cos_yaw = math.cos(target_yaw)
        sin_yaw = math.sin(target_yaw)
        rot = np.array([
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        return np.array(target, dtype=float) + rot.dot(self.relative_tracking_p)

    def yaw_to_target(self, target):
        delta = target[:2] - self.feedback_position()[:2]
        if np.linalg.norm(delta) < 1e-3:
            return 0.0
        return math.atan2(delta[1], delta[0])

    def ego_command_is_fresh(self):
        with self.model_lock:
            command_time = self.ego_last_cmd_time
        return (
            command_time is not None and
            time.time() - command_time <= self.ego_command_timeout
        )

    def ego_command_point(self, fallback_point=None):
        with self.model_lock:
            cmd = self.ego_last_cmd
        if cmd is None:
            return None
        _raw_pos, _raw_vel, cmd_pos, cmd_vel = self.ego_command_vectors(cmd, fallback_point=fallback_point)
        if not (np.all(np.isfinite(cmd_pos)) and np.all(np.isfinite(cmd_vel))):
            return None
        return cmd_pos

    def ego_command_vectors(self, cmd, fallback_point=None):
        raw_pos = np.array([cmd.position.x, cmd.position.y, cmd.position.z], dtype=float)
        raw_vel = np.array([cmd.velocity.x, cmd.velocity.y, cmd.velocity.z], dtype=float)
        cmd_pos = raw_pos.copy()
        cmd_vel = raw_vel.copy()
        if self.lock_ego_tracking_height:
            if fallback_point is not None and np.all(np.isfinite(fallback_point)):
                locked_z = float(fallback_point[2])
            else:
                locked_z = self.ego_tracking_height
            cmd_pos[2] = locked_z
            cmd_vel[2] = 0.0
        else:
            clamped_z = float(np.clip(cmd_pos[2], self.track_min_control_z, self.track_max_control_z))
            if abs(clamped_z - cmd_pos[2]) > 1e-6:
                cmd_pos[2] = clamped_z
                if (cmd_pos[2] <= self.track_min_control_z and cmd_vel[2] < 0.0) or (
                    cmd_pos[2] >= self.track_max_control_z and cmd_vel[2] > 0.0
                ):
                    cmd_vel[2] = 0.0
        return raw_pos, raw_vel, cmd_pos, cmd_vel

    def publish_ego_inputs(self, stamp, true_target, target_velocity, target_yaw, target_visible=True):
        if not self.use_ego_planner:
            return

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id = '{}/base_link'.format(self.uav_name)
        feedback = self.feedback_position()
        odom.pose.pose.position.x = feedback[0]
        odom.pose.pose.position.y = feedback[1]
        odom.pose.pose.position.z = feedback[2]
        with self.model_lock:
            gazebo_pose = self.gazebo_pose
        if gazebo_pose is not None:
            odom.pose.pose.orientation = gazebo_pose.orientation
        else:
            odom.pose.pose.orientation.w = self.drone.meQuaternionENU[0]
            odom.pose.pose.orientation.x = self.drone.meQuaternionENU[1]
            odom.pose.pose.orientation.y = self.drone.meQuaternionENU[2]
            odom.pose.pose.orientation.z = self.drone.meQuaternionENU[3]
        feedback_velocity = self.feedback_velocity()
        odom.twist.twist.linear.x = feedback_velocity[0]
        odom.twist.twist.linear.y = feedback_velocity[1]
        odom.twist.twist.linear.z = feedback_velocity[2]
        self.ego_odom_pub.publish(odom)
        self.publish_ego_obstacle_cloud_msg(stamp)

        if self.ego_object_odom_pub is None:
            return

        # Legacy compatibility only. The target-tracking launch disables this
        # path and obtains /object_odom from target_state_coordinator.py.
        if self.fov_sharing_enabled and not target_visible:
            return
        obj = Odometry()
        obj.header.stamp = stamp
        obj.header.frame_id = 'world'
        obj.child_frame_id = 'tracking_target'
        obj.pose.pose.position.x = true_target[0]
        obj.pose.pose.position.y = true_target[1]
        obj.pose.pose.position.z = true_target[2]
        qw, qx, qy, qz = self.yaw_to_quaternion(target_yaw)
        obj.pose.pose.orientation.w = qw
        obj.pose.pose.orientation.x = qx
        obj.pose.pose.orientation.y = qy
        obj.pose.pose.orientation.z = qz
        obj.twist.twist.linear.x = target_velocity[0]
        obj.twist.twist.linear.y = target_velocity[1]
        obj.twist.twist.linear.z = target_velocity[2]
        self.ego_object_odom_pub.publish(obj)

    def visibility_warmup_complete(self, stamp):
        if self.shared_track_start_ros is None:
            return False
        return stamp.to_sec() - self.shared_track_start_ros >= self.fov_visibility_warmup

    def publish_target_observation(self, stamp, true_target, target_velocity, target_yaw, visible):
        self.last_object_odom_published = False
        if not self.fov_sharing_enabled or self.stage != 'TRACK':
            return
        if self.use_external_target_state and not self.external_target_is_fresh(stamp):
            return
        if not self.visibility_warmup_complete(stamp) or not visible:
            return

        observation = Odometry()
        observation.header.stamp = stamp
        observation.header.frame_id = 'world'
        observation.child_frame_id = 'target_observed_by_{}'.format(self.uav_name)
        observation.pose.pose.position.x = true_target[0]
        observation.pose.pose.position.y = true_target[1]
        observation.pose.pose.position.z = true_target[2]
        qw, qx, qy, qz = self.yaw_to_quaternion(target_yaw)
        observation.pose.pose.orientation.w = qw
        observation.pose.pose.orientation.x = qx
        observation.pose.pose.orientation.y = qy
        observation.pose.pose.orientation.z = qz
        observation.twist.twist.linear.x = target_velocity[0]
        observation.twist.twist.linear.y = target_velocity[1]
        observation.twist.twist.linear.z = target_velocity[2]
        self.target_observation_pub.publish(observation)
        self.last_object_odom_published = True

    def hold_follow_point(self, follow_point, yaw):
        if follow_point is None:
            feedback = self.feedback_position()
            feedback_velocity = self.feedback_velocity()
            if self.lock_ego_tracking_height:
                target_z = self.ego_tracking_height
            elif self.last_follow_point is not None and np.all(np.isfinite(self.last_follow_point)):
                target_z = float(self.last_follow_point[2])
            else:
                target_z = self.target_height
            target_z = float(np.clip(target_z, self.track_min_control_z, self.track_max_control_z))
            vel_z = (
                self.track_position_gain_z * (target_z - feedback[2]) -
                self.track_vertical_damping * feedback_velocity[2]
            )
            vel_z = float(np.clip(vel_z, -self.ego_max_vel_z, self.ego_max_vel_z))
            self.last_control_velocity = np.array([0.0, 0.0, vel_z], dtype=float)
            self.last_control_velocity_valid = True
            self.drone.velocityENUControl(self.last_control_velocity, yaw)
            return
        self.track_velocity_control(follow_point, yaw)

    def apply_ego_position_command(self, yaw, fallback_point=None):
        if not self.ego_command_is_fresh():
            rospy.logwarn_throttle(1.0, 'Waiting for fresh EGO-Planner-v2 PositionCommand on %s', self.ego_position_cmd_topic)
            self.hold_follow_point(None, yaw)
            return False

        with self.model_lock:
            cmd = self.ego_last_cmd
        if cmd is None:
            self.hold_follow_point(None, yaw)
            return False

        raw_pos, raw_vel, cmd_pos, cmd_vel = self.ego_command_vectors(cmd, fallback_point=fallback_point)
        if not (np.all(np.isfinite(raw_pos)) and np.all(np.isfinite(raw_vel))):
            rospy.logwarn_throttle(1.0, 'Ignoring non-finite EGO-Planner-v2 PositionCommand')
            self.hold_follow_point(None, yaw)
            return False
        if abs(float(raw_pos[2] - cmd_pos[2])) > 0.05:
            rospy.logwarn_throttle(
                1.0,
                'Clamping EGO-Planner-v2 z command %.2f to %.2f in Gazebo world/ENU',
                raw_pos[2],
                cmd_pos[2],
            )

        feedback = self.feedback_position()
        feedback_velocity = self.feedback_velocity()
        vel = (
            cmd_vel +
            self.ego_position_gain * (cmd_pos - feedback) +
            self.ego_velocity_gain * (cmd_vel - feedback_velocity)
        )
        vel[0] = np.clip(vel[0], -self.ego_max_vel_xy, self.ego_max_vel_xy)
        vel[1] = np.clip(vel[1], -self.ego_max_vel_xy, self.ego_max_vel_xy)
        vel[2] = np.clip(vel[2], -self.ego_max_vel_z, self.ego_max_vel_z)
        self.last_control_velocity = vel.copy()
        self.last_control_velocity_valid = True
        self.drone.velocityENUControl(vel, yaw)
        return True

    def takeoff_settled(self):
        if self.takeoff_point is None:
            return False
        z_error = abs(float(self.takeoff_point[2] - self.feedback_position()[2]))
        vz = abs(float(self.feedback_velocity()[2]))
        settled_now = z_error <= self.takeoff_z_tolerance and vz <= self.takeoff_speed_tolerance
        if not settled_now:
            self.takeoff_settle_start = None
            return False
        if self.takeoff_settle_start is None:
            self.takeoff_settle_start = time.time()
        return time.time() - self.takeoff_settle_start >= self.takeoff_settle_time

    def track_velocity_control(self, follow_point, yaw):
        error = follow_point - self.feedback_position()
        vel = np.array([
            self.track_position_gain_xy * error[0],
            self.track_position_gain_xy * error[1],
            self.track_position_gain_z * error[2] - self.track_vertical_damping * self.feedback_velocity()[2],
        ], dtype=float)
        vel[0] = np.clip(vel[0], -self.max_vel_xy, self.max_vel_xy)
        vel[1] = np.clip(vel[1], -self.max_vel_xy, self.max_vel_xy)
        vel[2] = np.clip(vel[2], -self.track_max_vel_z, self.track_max_vel_z)
        self.last_control_velocity = vel.copy()
        self.last_control_velocity_valid = True
        self.drone.velocityENUControl(vel, yaw)

    def track_xy_velocity_z_position_control(self, follow_point, yaw):
        error = follow_point - self.feedback_position()
        vel_xy = np.array([
            self.track_position_gain_xy * error[0],
            self.track_position_gain_xy * error[1],
        ], dtype=float)
        vel_xy = np.clip(vel_xy, -self.max_vel_xy, self.max_vel_xy)

        self.drone.setVelocityControlMode()
        self.drone.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.drone.setpoint.type_mask = (
            PositionTarget.IGNORE_PX +
            PositionTarget.IGNORE_PY +
            PositionTarget.IGNORE_VZ +
            PositionTarget.IGNORE_AFX +
            PositionTarget.IGNORE_AFY +
            PositionTarget.IGNORE_AFZ +
            PositionTarget.IGNORE_YAW_RATE
        )
        self.drone.setpoint.position.z = follow_point[2]
        self.drone.setpoint.velocity.x = vel_xy[0]
        self.drone.setpoint.velocity.y = vel_xy[1]
        self.drone.setpoint.velocity.z = 0.0
        self.drone.setpoint.yaw = yaw
        self.last_control_velocity = np.array([vel_xy[0], vel_xy[1], 0.0], dtype=float)
        self.last_control_velocity_valid = True

    def track_pose_setpoint_control(self, stamp, follow_point, yaw):
        local_point = np.array(follow_point, dtype=float) - self.drone.localOffsetENU
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = local_point[0]
        pose.pose.position.y = local_point[1]
        pose.pose.position.z = local_point[2]
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        self.track_pose_pub.publish(pose)

    def compensated_track_point(self, follow_point):
        control_point = follow_point.copy()
        if not self.track_altitude_compensation:
            return control_point
        z_error = self.feedback_position()[2] - self.target_height
        z_correction = (
            self.track_altitude_comp_gain * z_error +
            self.track_altitude_comp_damping * self.feedback_velocity()[2]
        )
        control_point[2] = np.clip(
            self.target_height - z_correction,
            self.track_min_control_z,
            self.track_max_control_z,
        )
        return control_point

    def marker_common(self, stamp, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'target_tracking'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.4)
        return marker

    def publish_pose(self, publisher, stamp, target):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        pose.pose.position.z = target[2]
        pose.pose.orientation.w = 1.0
        publisher.publish(pose)
        return pose

    def publish_target(self, stamp, true_target, target, target_source, follow_point, control_point, yaw, error):
        true_pose = self.publish_pose(self.true_target_pub, stamp, true_target)
        pose = self.publish_pose(self.target_pub, stamp, target)
        self.error_pub.publish(Float32(data=float(error)))
        self.write_track_log(stamp, true_target, target, target_source, follow_point, control_point, yaw, error)
        self.publish_gazebo_target(true_target)

        marker = self.marker_common(stamp, 0, Marker.SPHERE)
        marker.pose = true_pose.pose
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.marker_pub.publish(marker)

        follow_marker = self.marker_common(stamp, 1, Marker.SPHERE)
        follow_marker.pose.position.x = follow_point[0]
        follow_marker.pose.position.y = follow_point[1]
        follow_marker.pose.position.z = follow_point[2]
        follow_marker.pose.orientation.w = 1.0
        follow_marker.scale.x = 0.22
        follow_marker.scale.y = 0.22
        follow_marker.scale.z = 0.22
        follow_marker.color.r = 0.05
        follow_marker.color.g = 0.45
        follow_marker.color.b = 1.0
        follow_marker.color.a = 0.9

        drone_to_target = self.marker_common(stamp, 2, Marker.LINE_STRIP)
        drone_to_target.scale.x = 0.035
        drone_to_target.color.r = 1.0
        drone_to_target.color.g = 0.95
        drone_to_target.color.b = 0.1
        drone_to_target.color.a = 0.8
        drone_to_target.points = [
            Point(self.feedback_position()[0], self.feedback_position()[1], self.feedback_position()[2]),
            Point(target[0], target[1], target[2]),
        ]

        target_to_follow = self.marker_common(stamp, 3, Marker.LINE_STRIP)
        target_to_follow.scale.x = 0.035
        target_to_follow.color.r = 0.1
        target_to_follow.color.g = 0.55
        target_to_follow.color.b = 1.0
        target_to_follow.color.a = 0.8
        target_to_follow.points = [
            Point(target[0], target[1], target[2]),
            Point(follow_point[0], follow_point[1], follow_point[2]),
        ]

        text_marker = self.marker_common(stamp, 4, Marker.TEXT_VIEW_FACING)
        text_marker.pose.position.x = true_target[0]
        text_marker.pose.position.y = true_target[1]
        text_marker.pose.position.z = true_target[2] + 0.45
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.28
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 0.95
        text_marker.text = 'target'

        marker_array = MarkerArray()
        marker_array.markers = [marker, follow_marker, drone_to_target, target_to_follow, text_marker]
        self.marker_array_pub.publish(marker_array)

    def set_stage(self, stage):
        if self.stage != stage:
            rospy.loginfo('TargetTrack stage: %s -> %s', self.stage, stage)
            self.stage = stage
            self.stage_start = time.time()
            if stage == 'OFFBOARD':
                self.offboard_requested = False
                self.last_offboard_request_time = None
            elif stage == 'ARM':
                self.armed_requested = False
                self.last_arm_request_time = None
            if stage == 'TRACK':
                if not self.use_external_target_state:
                    self.track_start_time = time.time()
                self.target_ready_pub.publish(Bool(data=True))

    def retry_due(self, last_request_time, interval):
        return last_request_time is None or time.time() - last_request_time >= interval

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            true_target, target_velocity, target_yaw = self.target_state(stamp)
            shared_available = True
            if self.use_ego_planner:
                shared_state = self.shared_target_state(stamp) if self.use_external_target_state else None
                shared_available = not self.use_external_target_state or shared_state is not None
                if shared_state is not None:
                    target, _shared_velocity, shared_target_yaw, target_source = shared_state
                    self.last_shared_target = target.copy()
                    self.last_shared_target_yaw = shared_target_yaw
                    follow_point = self.ego_tracking_point(target, shared_target_yaw)
                elif self.use_external_target_state:
                    target_source = 'shared_target_waiting'
                    if self.last_shared_target is not None:
                        target = self.last_shared_target.copy()
                        follow_point = self.ego_tracking_point(target, self.last_shared_target_yaw)
                    else:
                        target = self.feedback_position()
                        follow_point = target.copy()
                else:
                    target = true_target.copy()
                    target_source = 'local_truth'
                    follow_point = self.ego_tracking_point(target, target_yaw)
            else:
                target, target_source = self.current_target(stamp)
                follow_point = self.desired_follow_point(target)
            self.last_follow_point = follow_point.copy()
            if self.stage == 'TRACK' and not self.use_ego_planner:
                control_point = self.compensated_track_point(follow_point)
            else:
                control_point = follow_point.copy()
            if self.use_ego_planner and self.stage == 'TRACK':
                ego_cmd_point = self.ego_command_point(fallback_point=follow_point)
                if ego_cmd_point is not None:
                    control_point = ego_cmd_point
            shared_yaw = self.yaw_to_target(target) if target_source != 'shared_target_waiting' else self.feedback_yaw()
            yaw = shared_yaw if self.use_ego_planner else self.yaw_to_target(target)
            if self.fov_sharing_enabled and self.fov_use_velocity_yaw:
                motion_yaw = self.yaw_from_motion(shared_yaw)
                yaw = self.blend_yaw(shared_yaw, motion_yaw, self.fov_velocity_yaw_weight)
            visibility_yaw = self.feedback_yaw()
            self.last_target_visible = self.target_visible_from_self(
                true_target,
                visibility_yaw,
                stamp=stamp.to_sec(),
            )
            self.publish_target_observation(
                stamp,
                true_target,
                target_velocity,
                target_yaw,
                self.last_target_visible,
            )
            self.publish_ego_inputs(
                stamp,
                true_target,
                target_velocity,
                target_yaw,
                target_visible=self.last_target_visible,
            )
            feedback_position = self.feedback_position()
            error = np.linalg.norm(feedback_position - follow_point)
            self.publish_target(stamp, true_target, target, target_source, follow_point, control_point, yaw, error)

            if self.takeoff_point is None:
                x = float(self.start_x) if self.start_x is not None else float(feedback_position[0])
                y = float(self.start_y) if self.start_y is not None else float(feedback_position[1])
                self.takeoff_point = np.array([x, y, self.takeoff_height], dtype=float)

            if self.stage == 'INIT':
                self.drone.positionENUControl(self.takeoff_point, yaw)
                if self.drone.meState.connected and time.time() - self.stage_start > 2.0:
                    self.set_stage('OFFBOARD')

            elif self.stage == 'OFFBOARD':
                self.drone.positionENUControl(self.takeoff_point, yaw)
                if self.drone.meState.mode != 'OFFBOARD' and self.retry_due(
                    self.last_offboard_request_time,
                    self.offboard_retry_interval,
                ):
                    self.last_offboard_request_time = time.time()
                    self.offboard_requested = bool(self.drone.intoOffboardMode())
                if self.drone.meState.mode == 'OFFBOARD':
                    self.set_stage('ARM')

            elif self.stage == 'ARM':
                self.drone.positionENUControl(self.takeoff_point, yaw)
                if not self.drone.isArmed() and self.retry_due(
                    self.last_arm_request_time,
                    self.arm_retry_interval,
                ):
                    self.last_arm_request_time = time.time()
                    self.armed_requested = bool(self.drone.arm())
                if self.drone.isArmed():
                    self.set_stage('TAKEOFF')

            elif self.stage == 'TAKEOFF':
                self.drone.velocityToPointENUControl(self.takeoff_point, yaw)
                if self.takeoff_settled():
                    self.set_stage('TRACK')

            elif self.stage == 'TRACK':
                if self.use_ego_planner:
                    if shared_available:
                        self.apply_ego_position_command(yaw, fallback_point=follow_point)
                    else:
                        rospy.logwarn_throttle(1.0, 'Shared target unavailable; holding instead of executing stale planner commands')
                        self.hold_follow_point(None, yaw)
                elif self.use_track_pose_setpoint:
                    self.track_pose_setpoint_control(stamp, control_point, yaw)
                else:
                    self.drone.positionENUControl(control_point, yaw)

            else:
                self.drone.hoverWithYaw(yaw)

            self.drone.sendHeartbeat()
            rospy.loginfo_throttle(
                1.0,
                'stage=%s pos=(%.2f %.2f %.2f) target=(%.2f %.2f %.2f source=%s) follow=(%.2f %.2f %.2f) err=%.2f',
                self.stage,
                feedback_position[0], feedback_position[1], feedback_position[2],
                target[0], target[1], target[2], target_source,
                follow_point[0], follow_point[1], follow_point[2],
                error,
            )
            rate.sleep()


def main():
    rospy.init_node('target_track')
    try:
        TargetTrack().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
