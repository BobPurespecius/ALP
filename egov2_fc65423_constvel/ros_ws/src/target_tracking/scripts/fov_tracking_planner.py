#!/usr/bin/env python3

import math
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError as exc:
    PositionCommand = None
    _POSITION_COMMAND_IMPORT_ERROR = exc
else:
    _POSITION_COMMAND_IMPORT_ERROR = None


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quaternion(yaw):
    return math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)


def rotation_z(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def point_to_array(point):
    return np.array([point.x, point.y, point.z], dtype=float)


def vector_to_array(vector):
    return np.array([vector.x, vector.y, vector.z], dtype=float)


def parse_drone_ids(value):
    if isinstance(value, (list, tuple)):
        return [int(v) for v in value]
    return [int(v.strip()) for v in str(value).split(',') if v.strip()]


def parse_relative_positions(value, drone_count, default_distance):
    rels = []
    if isinstance(value, str):
        for item in value.split(';'):
            item = item.strip()
            if not item:
                continue
            parts = [float(v.strip()) for v in item.split(',')]
            if len(parts) == 2:
                parts.append(0.0)
            rels.append(np.array(parts[:3], dtype=float))
    elif isinstance(value, (list, tuple)):
        for item in value:
            parts = list(item)
            if len(parts) == 2:
                parts.append(0.0)
            rels.append(np.array(parts[:3], dtype=float))

    if not rels:
        if drone_count == 1:
            rels.append(np.array([-default_distance, 0.0, 0.0], dtype=float))
        else:
            for idx in range(drone_count):
                angle = math.pi + 2.0 * math.pi * idx / drone_count
                rels.append(np.array([
                    default_distance * math.cos(angle),
                    default_distance * math.sin(angle),
                    0.0,
                ], dtype=float))

    while len(rels) < drone_count:
        rels.append(rels[-1].copy())
    return rels[:drone_count]


def parse_obstacles(value):
    obstacles = []
    if isinstance(value, str):
        for item in value.split(';'):
            item = item.strip()
            if not item:
                continue
            parts = [float(v.strip()) for v in item.split(',')]
            if len(parts) >= 3:
                z_min = parts[3] if len(parts) > 3 else -math.inf
                z_max = parts[4] if len(parts) > 4 else math.inf
                obstacles.append({
                    'x': parts[0],
                    'y': parts[1],
                    'radius': parts[2],
                    'z_min': z_min,
                    'z_max': z_max,
                })
    elif isinstance(value, (list, tuple)):
        for item in value:
            if isinstance(item, dict):
                obstacles.append({
                    'x': float(item.get('x', 0.0)),
                    'y': float(item.get('y', 0.0)),
                    'radius': float(item.get('radius', 0.0)),
                    'z_min': float(item.get('z_min', -math.inf)),
                    'z_max': float(item.get('z_max', math.inf)),
                })
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                obstacles.append({
                    'x': float(item[0]),
                    'y': float(item[1]),
                    'radius': float(item[2]),
                    'z_min': float(item[3]) if len(item) > 3 else -math.inf,
                    'z_max': float(item[4]) if len(item) > 4 else math.inf,
                })
    return obstacles


def segment_circle_distance(a_xy, b_xy, center_xy):
    ab = b_xy - a_xy
    denom = float(np.dot(ab, ab))
    if denom <= 1e-9:
        return float(np.linalg.norm(a_xy - center_xy))
    t = float(np.clip(np.dot(center_xy - a_xy, ab) / denom, 0.0, 1.0))
    closest = a_xy + t * ab
    return float(np.linalg.norm(closest - center_xy))


def min_jerk_state(p0, v0, a0, p1, v1, a1, duration, sample_t):
    duration = max(float(duration), 1e-3)
    t = float(np.clip(sample_t, 0.0, duration))
    pos = np.zeros(3, dtype=float)
    vel = np.zeros(3, dtype=float)
    acc = np.zeros(3, dtype=float)
    jerk = np.zeros(3, dtype=float)

    T = duration
    mat = np.array([
        [T ** 3, T ** 4, T ** 5],
        [3.0 * T ** 2, 4.0 * T ** 3, 5.0 * T ** 4],
        [6.0 * T, 12.0 * T ** 2, 20.0 * T ** 3],
    ], dtype=float)

    for dim in range(3):
        c0 = p0[dim]
        c1 = v0[dim]
        c2 = 0.5 * a0[dim]
        rhs = np.array([
            p1[dim] - (c0 + c1 * T + c2 * T ** 2),
            v1[dim] - (c1 + 2.0 * c2 * T),
            a1[dim] - (2.0 * c2),
        ], dtype=float)
        c3, c4, c5 = np.linalg.solve(mat, rhs)

        pos[dim] = c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5
        vel[dim] = c1 + 2.0 * c2 * t + 3.0 * c3 * t ** 2 + 4.0 * c4 * t ** 3 + 5.0 * c5 * t ** 4
        acc[dim] = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t ** 2 + 20.0 * c5 * t ** 3
        jerk[dim] = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t ** 2

    return pos, vel, acc, jerk


class FovTrackingPlanner:
    def __init__(self):
        if PositionCommand is None:
            raise ImportError('quadrotor_msgs/PositionCommand is required') from _POSITION_COMMAND_IMPORT_ERROR

        self.lock = threading.Lock()
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.drone_ids = parse_drone_ids(rospy.get_param('~drone_ids', [0]))
        self.object_odom_topic = rospy.get_param('~object_odom_topic', '/object_odom')
        self.odom_topic_template = rospy.get_param('~odom_topic_template', '/drone_{id}_visual_slam/odom')
        self.command_topic_template = rospy.get_param('~command_topic_template', '/drone_{id}_planning/pos_cmd')
        self.marker_topic = rospy.get_param('~marker_topic', '/fov_tracking/markers')

        self.rate_hz = float(rospy.get_param('~rate', 30.0))
        self.prediction_dt = float(rospy.get_param('~prediction_dt', 0.25))
        self.prediction_steps = int(rospy.get_param('~prediction_steps', 8))
        self.lookahead_time = float(rospy.get_param('~lookahead_time', 1.0))
        self.plan_duration = float(rospy.get_param('~plan_duration', 1.6))
        self.command_time_forward = float(rospy.get_param('~command_time_forward', 0.25))

        self.tracking_distance = float(rospy.get_param('~tracking_distance', 1.2))
        self.relative_positions = parse_relative_positions(
            rospy.get_param('~relative_positions', []),
            len(self.drone_ids),
            self.tracking_distance,
        )
        self.min_target_distance = float(rospy.get_param('~min_target_distance', 0.7))
        self.max_target_distance = float(rospy.get_param('~max_target_distance', 3.0))
        self.min_uav_distance = float(rospy.get_param('~min_uav_distance', 0.8))
        self.fov_half_angle = math.radians(float(rospy.get_param('~fov_half_angle_deg', 45.0)))
        self.enable_candidate_goal = bool(rospy.get_param('~enable_candidate_goal', False))
        self.candidate_count = int(rospy.get_param('~candidate_count', 33))
        self.candidate_angle_span = math.radians(float(rospy.get_param('~candidate_angle_span_deg', 220.0)))
        self.candidate_radius_samples = [
            float(v) for v in rospy.get_param('~candidate_radius_samples', [0.85, 1.0, 1.2])
        ]

        self.max_vel_xy = float(rospy.get_param('~max_vel_xy', 1.0))
        self.max_vel_z = float(rospy.get_param('~max_vel_z', 0.45))
        self.max_acc = float(rospy.get_param('~max_acc', 1.8))
        self.target_height = float(rospy.get_param('~target_height', 1.2))
        self.lock_tracking_height = bool(rospy.get_param('~lock_tracking_height', True))

        self.desired_weight = float(rospy.get_param('~desired_weight', 1.0))
        self.smooth_weight = float(rospy.get_param('~smooth_weight', 0.2))
        self.distance_weight = float(rospy.get_param('~distance_weight', 0.6))
        self.occlusion_weight = float(rospy.get_param('~occlusion_weight', 3.0))
        self.shared_visibility_weight = float(rospy.get_param('~shared_visibility_weight', 0.8))
        self.lost_visibility_weight = float(rospy.get_param('~lost_visibility_weight', 6.0))
        self.inter_uav_weight = float(rospy.get_param('~inter_uav_weight', 4.0))
        self.obstacles = parse_obstacles(rospy.get_param('~obstacles', []))

        self.target_odom = None
        self.drone_odom = {}
        self.last_goals = {}
        self.last_visibility = {}

        self.command_pubs = {}
        for drone_id in self.drone_ids:
            odom_topic = self.odom_topic_template.format(id=drone_id, index=drone_id)
            command_topic = self.command_topic_template.format(id=drone_id, index=drone_id)
            rospy.Subscriber(odom_topic, Odometry, self.odom_cb, callback_args=drone_id, queue_size=1)
            self.command_pubs[drone_id] = rospy.Publisher(command_topic, PositionCommand, queue_size=10)

        rospy.Subscriber(self.object_odom_topic, Odometry, self.target_cb, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=10)

    def target_cb(self, msg):
        with self.lock:
            self.target_odom = msg

    def odom_cb(self, msg, drone_id):
        with self.lock:
            self.drone_odom[drone_id] = msg

    def target_state(self):
        with self.lock:
            msg = self.target_odom
        if msg is None:
            return None
        pos = point_to_array(msg.pose.pose.position)
        vel = vector_to_array(msg.twist.twist.linear)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if np.linalg.norm(vel[:2]) > 1e-3:
            yaw = math.atan2(vel[1], vel[0])
        if self.lock_tracking_height:
            pos[2] = self.target_height
            vel[2] = 0.0
        return pos, vel, yaw

    def drone_state(self, drone_id):
        with self.lock:
            msg = self.drone_odom.get(drone_id)
        if msg is None:
            return None
        return (
            point_to_array(msg.pose.pose.position),
            vector_to_array(msg.twist.twist.linear),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def predicted_target(self, target_pos, target_vel, target_yaw, t):
        pos = target_pos + target_vel * max(0.0, t)
        yaw = target_yaw
        if np.linalg.norm(target_vel[:2]) > 1e-3:
            yaw = math.atan2(target_vel[1], target_vel[0])
        if self.lock_tracking_height:
            pos[2] = self.target_height
        return pos, target_vel.copy(), yaw

    def desired_tracking_point(self, drone_index, target_pos, target_yaw):
        rel = self.relative_positions[drone_index]
        desired = target_pos + rotation_z(target_yaw).dot(rel)
        if self.lock_tracking_height:
            desired[2] = self.target_height + rel[2]
        return desired

    def line_of_sight_clear(self, candidate, target):
        a_xy = candidate[:2]
        b_xy = target[:2]
        z_min = min(candidate[2], target[2])
        z_max = max(candidate[2], target[2])
        for obs in self.obstacles:
            if obs['z_max'] < z_min or obs['z_min'] > z_max:
                continue
            center = np.array([obs['x'], obs['y']], dtype=float)
            if segment_circle_distance(a_xy, b_xy, center) <= obs['radius']:
                return False
        return True

    def visible_from(self, observer_pos, target_pos, observer_yaw=None):
        distance = float(np.linalg.norm(observer_pos - target_pos))
        if distance < self.min_target_distance or distance > self.max_target_distance:
            return False
        if not self.line_of_sight_clear(observer_pos, target_pos):
            return False
        if observer_yaw is not None:
            bearing = math.atan2(target_pos[1] - observer_pos[1], target_pos[0] - observer_pos[0])
            if abs(wrap_pi(bearing - observer_yaw)) > self.fov_half_angle:
                return False
        return True

    def team_has_visibility(self, target_pos):
        for drone_id in self.drone_ids:
            state = self.drone_state(drone_id)
            if state is None:
                continue
            pos, _vel, yaw = state
            if self.visible_from(pos, target_pos, observer_yaw=yaw):
                return True
        return False

    def inter_uav_cost(self, candidate, drone_id):
        cost = 0.0
        for other_id, other_goal in self.last_goals.items():
            if other_id == drone_id:
                continue
            distance = float(np.linalg.norm(candidate - other_goal))
            if distance < self.min_uav_distance:
                cost += self.inter_uav_weight * (self.min_uav_distance - distance) ** 2
        return cost

    def candidate_points(self, desired, target_pos, target_yaw, rel):
        if not self.enable_candidate_goal:
            return [desired.copy()]

        rel_xy = desired[:2] - target_pos[:2]
        base_angle = math.atan2(rel_xy[1], rel_xy[0]) if np.linalg.norm(rel_xy) > 1e-3 else target_yaw + math.pi
        base_radius = max(float(np.linalg.norm(rel[:2])), self.tracking_distance)
        count = max(1, self.candidate_count)
        offsets = np.linspace(-0.5 * self.candidate_angle_span, 0.5 * self.candidate_angle_span, count)
        points = [desired.copy()]
        for scale in self.candidate_radius_samples:
            radius = base_radius * scale
            for offset in offsets:
                angle = base_angle + offset
                points.append(np.array([
                    target_pos[0] + radius * math.cos(angle),
                    target_pos[1] + radius * math.sin(angle),
                    desired[2],
                ], dtype=float))
        return points

    def select_goal(self, drone_id, drone_index, drone_pos, drone_vel, target_pos, target_vel, target_yaw):
        pred_pos, pred_vel, pred_yaw = self.predicted_target(
            target_pos,
            target_vel,
            target_yaw,
            self.lookahead_time,
        )
        desired = self.desired_tracking_point(drone_index, pred_pos, pred_yaw)
        rel = self.relative_positions[drone_index]
        team_visible = self.team_has_visibility(target_pos)

        best = None
        for candidate in self.candidate_points(desired, pred_pos, pred_yaw, rel):
            distance = float(np.linalg.norm(candidate - pred_pos))
            if distance < self.min_target_distance or distance > self.max_target_distance:
                continue

            yaw = math.atan2(pred_pos[1] - candidate[1], pred_pos[0] - candidate[0])
            visible = self.visible_from(candidate, pred_pos, observer_yaw=yaw)
            desired_cost = self.desired_weight * float(np.linalg.norm(candidate - desired) ** 2)
            smooth_cost = self.smooth_weight * float(np.linalg.norm(candidate - drone_pos - drone_vel * self.plan_duration) ** 2)
            distance_cost = self.distance_weight * (distance - self.tracking_distance) ** 2
            obstacle_cost = 0.0 if self.line_of_sight_clear(candidate, pred_pos) else self.occlusion_weight
            if visible:
                visibility_cost = 0.0
            elif team_visible:
                visibility_cost = self.shared_visibility_weight
            else:
                visibility_cost = self.lost_visibility_weight

            score = (
                desired_cost +
                smooth_cost +
                distance_cost +
                obstacle_cost +
                visibility_cost +
                self.inter_uav_cost(candidate, drone_id)
            )
            if best is None or score < best['score']:
                best = {
                    'point': candidate,
                    'yaw': yaw,
                    'visible': visible,
                    'team_visible': team_visible,
                    'score': score,
                    'target_vel': pred_vel,
                }

        if best is None:
            yaw = math.atan2(pred_pos[1] - desired[1], pred_pos[0] - desired[0])
            best = {
                'point': desired,
                'yaw': yaw,
                'visible': False,
                'team_visible': team_visible,
                'score': math.inf,
                'target_vel': pred_vel,
            }
        return best

    def bounded_velocity(self, vel):
        vel = np.asarray(vel, dtype=float).copy()
        xy_norm = float(np.linalg.norm(vel[:2]))
        if xy_norm > self.max_vel_xy > 1e-6:
            vel[:2] *= self.max_vel_xy / xy_norm
        vel[2] = float(np.clip(vel[2], -self.max_vel_z, self.max_vel_z))
        return vel

    def publish_command(self, drone_id, drone_pos, drone_vel, goal):
        goal_pos = goal['point']
        goal_vel = goal['target_vel'].copy()
        if np.linalg.norm(goal_vel[:2]) > self.max_vel_xy:
            goal_vel[:2] *= self.max_vel_xy / np.linalg.norm(goal_vel[:2])
        if self.lock_tracking_height:
            goal_vel[2] = 0.0

        pos, vel, acc, _jerk = min_jerk_state(
            drone_pos,
            drone_vel,
            np.zeros(3, dtype=float),
            goal_pos,
            goal_vel,
            np.zeros(3, dtype=float),
            self.plan_duration,
            self.command_time_forward,
        )
        vel = self.bounded_velocity(vel)
        acc = np.clip(acc, -self.max_acc, self.max_acc)

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.frame_id
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.position.x = pos[0]
        cmd.position.y = pos[1]
        cmd.position.z = pos[2]
        cmd.velocity.x = vel[0]
        cmd.velocity.y = vel[1]
        cmd.velocity.z = vel[2]
        cmd.acceleration.x = acc[0]
        cmd.acceleration.y = acc[1]
        cmd.acceleration.z = acc[2]
        cmd.yaw = goal['yaw']
        cmd.yaw_dot = 0.0
        self.command_pubs[drone_id].publish(cmd)

    def make_marker(self, marker_id, marker_type, ns, color, scale):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.4)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        return marker

    def publish_markers(self, target_pos):
        array = MarkerArray()
        target = self.make_marker(0, Marker.SPHERE, 'target', (1.0, 0.15, 0.05, 0.9), (0.25, 0.25, 0.25))
        target.pose.position.x = target_pos[0]
        target.pose.position.y = target_pos[1]
        target.pose.position.z = target_pos[2]
        target.pose.orientation.w = 1.0
        array.markers.append(target)

        marker_id = 1
        for drone_id, goal in self.last_goals.items():
            visible = self.last_visibility.get(drone_id, False)
            color = (0.0, 0.85, 0.25, 0.9) if visible else (1.0, 0.65, 0.0, 0.9)
            point = self.make_marker(marker_id, Marker.SPHERE, 'fov_goal', color, (0.18, 0.18, 0.18))
            point.pose.position.x = goal[0]
            point.pose.position.y = goal[1]
            point.pose.position.z = goal[2]
            point.pose.orientation.w = 1.0
            array.markers.append(point)
            marker_id += 1

            line = self.make_marker(marker_id, Marker.LINE_STRIP, 'fov_los', color, (0.025, 0.0, 0.0))
            line.points = [
                Point(goal[0], goal[1], goal[2]),
                Point(target_pos[0], target_pos[1], target_pos[2]),
            ]
            array.markers.append(line)
            marker_id += 1

        self.marker_pub.publish(array)

    def spin_once(self):
        target = self.target_state()
        if target is None:
            rospy.logwarn_throttle(2.0, 'FOV planner waiting for target odom on %s', self.object_odom_topic)
            return

        target_pos, target_vel, target_yaw = target
        for idx, drone_id in enumerate(self.drone_ids):
            state = self.drone_state(drone_id)
            if state is None:
                rospy.logwarn_throttle(2.0, 'FOV planner waiting for drone %s odom', drone_id)
                continue
            drone_pos, drone_vel, _drone_yaw = state
            goal = self.select_goal(drone_id, idx, drone_pos, drone_vel, target_pos, target_vel, target_yaw)
            self.last_goals[drone_id] = goal['point'].copy()
            self.last_visibility[drone_id] = bool(goal['visible'])
            self.publish_command(drone_id, drone_pos, drone_vel, goal)

        self.publish_markers(target_pos)
        visible_count = sum(1 for value in self.last_visibility.values() if value)
        rospy.loginfo_throttle(
            1.0,
            'FOV planner target=(%.2f %.2f %.2f) visible=%d/%d',
            target_pos[0],
            target_pos[1],
            target_pos[2],
            visible_count,
            len(self.drone_ids),
        )

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.spin_once()
            rate.sleep()


def main():
    rospy.init_node('fov_tracking_planner')
    FovTrackingPlanner().run()


if __name__ == '__main__':
    main()
