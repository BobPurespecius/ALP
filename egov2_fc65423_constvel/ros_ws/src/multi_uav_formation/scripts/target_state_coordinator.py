#!/usr/bin/env python3

import math
import json
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def yaw_to_quaternion(yaw):
    return math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)


class TargetStateCoordinator:
    """Owns the target clock and turns visible UAV observations into a shared estimate."""

    def __init__(self):
        get = rospy.get_param
        self.rate_hz = float(get('~rate', 30.0))
        self.frame_id = get('~frame_id', 'world')
        self.target_mode = get('~target_mode', 'waypoints')
        self.target_height = float(get('~target_height', 1.4))
        self.target_speed = float(get('~target_speed', 0.45))
        self.align_yaw_to_velocity = bool(get('~align_yaw_to_velocity', True))
        self.target_waypoints_use_z = bool(get('~target_waypoints_use_z', False))
        self.target_waypoints = self.parse_waypoints(get('~target_waypoints', '0.0,0.0'))
        self.target_segment_speeds = self.parse_segment_speeds(get('~target_segment_speeds', ''))
        self.target_center_x = float(get('~target_center_x', 4.0))
        self.target_center_y = float(get('~target_center_y', 0.0))
        self.target_radius_x = float(get('~target_radius_x', 1.8))
        self.target_radius_y = float(get('~target_radius_y', 1.0))
        self.target_omega = float(get('~target_omega', 0.18))
        self.sharing_enabled = bool(get('~sharing_enabled', False))
        self.max_prediction_age = max(float(get('~max_prediction_age', 3.0)), 0.0)
        self.required_uavs = list(get('~required_uavs', ['uav1', 'uav2', 'uav3']))
        self.ground_truth_topic = get('~ground_truth_topic', '/target_tracking/ground_truth')
        self.shared_topic = get('~shared_topic', '/object_odom')
        self.start_time_topic = get('~start_time_topic', '/target_tracking/target_start_time')
        self.dynamic_obstacle_scene_file = get('~dynamic_obstacle_scene_file', '')
        self.dynamic_obstacle_avoidance_enabled = bool(
            get('~dynamic_obstacle_avoidance_enabled', False))
        self.target_radius = max(float(get('~target_radius', 0.25)), 0.0)
        self.target_obstacle_clearance = max(
            float(get('~target_obstacle_clearance', 0.05)), 0.0)
        self.speed_scale_min = max(float(get('~target_speed_scale_min', 0.35)), 0.1)
        self.speed_scale_max = max(float(get('~target_speed_scale_max', 1.5)),
                                   self.speed_scale_min)
        self.motion_start_time = None
        self.adjusted_speeds_ready = False
        self.startup_settle_time = max(float(get('~startup_settle_time', 2.0)), 0.0)
        self.ready_since = None

        self.lock = threading.Lock()
        self.ready = set()
        self.start_time = None
        self.observations = {}
        self.last_target_yaw = 0.0

        self.ground_truth_pub = rospy.Publisher(self.ground_truth_topic, Odometry, queue_size=10)
        self.shared_pub = rospy.Publisher(self.shared_topic, Odometry, queue_size=10)
        self.start_time_pub = rospy.Publisher(self.start_time_topic, Float64, queue_size=1, latch=True)
        self.motion_start_sub = rospy.Subscriber(
            '/dynamic/motion_start_time', Float64,
            self.motion_start_cb, queue_size=1)

        for uav_name in self.required_uavs:
            rospy.Subscriber(
                '/target_tracking/ready/{}'.format(uav_name),
                Bool,
                self.ready_cb,
                callback_args=uav_name,
                queue_size=1,
            )
            rospy.Subscriber(
                '/target_tracking/observations/{}'.format(uav_name),
                Odometry,
                self.observation_cb,
                callback_args=uav_name,
                queue_size=10,
            )

    def parse_waypoints(self, text):
        result = []
        for item in str(text).split(';'):
            item = item.strip()
            if not item:
                continue
            values = [float(value.strip()) for value in item.split(',')]
            if len(values) == 2:
                values.append(self.target_height)
            if len(values) != 3:
                raise ValueError('target waypoint must be x,y or x,y,z: {}'.format(item))
            if not self.target_waypoints_use_z:
                values[2] = self.target_height
            result.append(np.asarray(values, dtype=float))
        if not result:
            result.append(np.array([0.0, 0.0, self.target_height], dtype=float))
        return result

    @staticmethod
    def parse_segment_speeds(text):
        return [max(float(value.strip()), 1.0e-3) for value in str(text).split(';') if value.strip()]

    def ready_cb(self, msg, uav_name):
        with self.lock:
            if msg.data:
                self.ready.add(uav_name)
            else:
                self.ready.discard(uav_name)

    def observation_cb(self, msg, uav_name):
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, 'Ignoring non-finite target observation from %s', uav_name)
            return
        with self.lock:
            self.observations[uav_name] = (msg, rospy.Time.now())

    def motion_start_cb(self, msg):
        if math.isfinite(msg.data):
            self.motion_start_time = float(msg.data)

    def maybe_start(self, stamp):
        with self.lock:
            if self.start_time is not None or not set(self.required_uavs).issubset(self.ready):
                return
            if self.ready_since is None:
                self.ready_since = stamp
                return
            if (stamp - self.ready_since).to_sec() < self.startup_settle_time:
                return
        # Build the target speed schedule before opening the target clock.
        # Otherwise a slow calculation advances elapsed time and makes the
        # first published target pose jump ahead of the initial waypoint.
        if self.dynamic_obstacle_avoidance_enabled:
            self.prepare_dynamic_safe_speeds(stamp)
        start_stamp = rospy.Time.now()
        with self.lock:
            if self.start_time is not None:
                return
            self.start_time = start_stamp
            start_sec = start_stamp.to_sec()
        self.start_time_pub.publish(Float64(data=start_sec))
        rospy.loginfo('Target clock started at %.6f after all UAVs entered TRACK', start_sec)

    def load_dynamic_obstacles(self):
        if not self.dynamic_obstacle_scene_file:
            return []
        try:
            with open(self.dynamic_obstacle_scene_file, 'r') as stream:
                scene = json.load(stream)
            return list((scene.get('movingObstacleData') or {}).values())
        except (OSError, ValueError) as exc:
            rospy.logwarn('Cannot load dynamic obstacle scene for target timing: %s', exc)
            return []

    @staticmethod
    def moving_obstacle_center(obstacle, elapsed):
        axis = obstacle.get('axisENU', [1.0, 0.0])
        norm = math.hypot(float(axis[0]), float(axis[1]))
        axis_x, axis_y = (1.0, 0.0) if norm < 1.0e-6 else (
            float(axis[0]) / norm, float(axis[1]) / norm)
        omega = 2.0 * math.pi / max(float(obstacle.get('period', 1.0)), 1.0e-6)
        offset = float(obstacle.get('amplitude', 0.0)) * math.sin(
            omega * elapsed + float(obstacle.get('phase', 0.0)))
        center = obstacle.get('centerENU', [0.0, 0.0])
        return (float(center[0]) + axis_x * offset,
                float(center[1]) + axis_y * offset)

    def dynamic_clearance(self, position, motion_elapsed, obstacles):
        result = float('inf')
        for obstacle in obstacles:
            center = self.moving_obstacle_center(obstacle, motion_elapsed)
            result = min(result, math.hypot(
                position[0] - center[0], position[1] - center[1]) -
                float(obstacle.get('radius', 0.5)))
        return result

    def prepare_dynamic_safe_speeds(self, stamp):
        obstacles = self.load_dynamic_obstacles()
        if not obstacles or len(self.target_waypoints) < 2:
            self.adjusted_speeds_ready = True
            return
        if self.motion_start_time is None:
            # The scene publisher is latched; this fallback only handles a
            # manually started coordinator without that scene node.
            self.motion_start_time = stamp.to_sec()
        motion_offset = self.motion_start_time
        elapsed_target = 0.0
        adjusted = []
        required_clearance = self.target_radius + self.target_obstacle_clearance
        scales = [
            self.speed_scale_min + 0.02 * index
            for index in range(int(
                math.floor((self.speed_scale_max - self.speed_scale_min) / 0.02)) + 1)
        ]
        if scales[-1] < self.speed_scale_max - 1.0e-6:
            scales.append(self.speed_scale_max)
        for index, (start, end) in enumerate(zip(
                self.target_waypoints[:-1], self.target_waypoints[1:])):
            segment = end - start
            length = float(np.linalg.norm(segment))
            if length < 1.0e-6:
                adjusted.append(self.target_speed)
                continue
            nominal = (self.target_segment_speeds[index]
                       if index < len(self.target_segment_speeds)
                       else max(self.target_speed, 1.0e-3))
            choices = []
            for scale in scales:
                speed = nominal * scale
                duration = length / speed
                minimum = float('inf')
                for sample in range(41):
                    fraction = sample / 40.0
                    position = start + fraction * segment
                    minimum = min(minimum, self.dynamic_clearance(
                        position,
                        stamp.to_sec() - motion_offset + elapsed_target + fraction * duration,
                        obstacles))
                choices.append((minimum, scale, speed, duration))
            safe = [choice for choice in choices if choice[0] >= required_clearance]
            choice = min(safe, key=lambda item: abs(item[1] - 1.0)) if safe else max(
                choices, key=lambda item: item[0])
            adjusted.append(choice[2])
            if abs(choice[1] - 1.0) > 0.03:
                rospy.loginfo(
                    'Target segment %d speed %.3f m/s (scale %.2f, clearance %.3f m)',
                    index, choice[2], choice[1], choice[0])
            elapsed_target += choice[3]
        self.target_segment_speeds = adjusted
        self.adjusted_speeds_ready = True
        rospy.loginfo('Target dynamic-obstacle timing prepared for %d segments', len(adjusted))

    def target_state(self, stamp):
        with self.lock:
            start_time = self.start_time
        if start_time is None:
            initial_yaw = self.last_target_yaw
            if len(self.target_waypoints) >= 2:
                initial_yaw = self.velocity_yaw(self.target_waypoints[1] - self.target_waypoints[0], initial_yaw)
            if self.target_mode == 'ellipse':
                initial_position = np.array([
                    self.target_center_x,
                    self.target_center_y + self.target_radius_y * math.sin(0.8),
                    self.target_height,
                ], dtype=float)
            else:
                initial_position = self.target_waypoints[0].copy()
            return initial_position, np.zeros(3, dtype=float), initial_yaw

        elapsed = max(0.0, (stamp - start_time).to_sec())

        if self.target_mode == 'ellipse':
            position = np.array([
                self.target_center_x + self.target_radius_x * math.sin(self.target_omega * elapsed),
                self.target_center_y + self.target_radius_y * math.sin(0.7 * self.target_omega * elapsed + 0.8),
                self.target_height,
            ], dtype=float)
            velocity = np.array([
                self.target_radius_x * self.target_omega * math.cos(self.target_omega * elapsed),
                self.target_radius_y * 0.7 * self.target_omega * math.cos(0.7 * self.target_omega * elapsed + 0.8),
                0.0,
            ], dtype=float)
            yaw = self.velocity_yaw(velocity, self.last_target_yaw)
            return position, velocity, yaw

        fallback_yaw = self.last_target_yaw
        for index, (start, end) in enumerate(zip(self.target_waypoints[:-1], self.target_waypoints[1:])):
            segment = end - start
            length = float(np.linalg.norm(segment))
            if length < 1.0e-6:
                continue
            speed = self.target_segment_speeds[index] if index < len(self.target_segment_speeds) else max(self.target_speed, 1.0e-3)
            duration = length / speed
            direction = segment / length
            fallback_yaw = self.velocity_yaw(direction, fallback_yaw)
            if elapsed <= duration:
                velocity = direction * speed
                return start + segment * (elapsed / duration), velocity, self.velocity_yaw(velocity, fallback_yaw)
            elapsed -= duration
        return self.target_waypoints[-1].copy(), np.zeros(3), fallback_yaw

    def velocity_yaw(self, velocity, fallback):
        if not self.align_yaw_to_velocity:
            self.last_target_yaw = 0.0
            return 0.0
        if float(np.linalg.norm(velocity[:2])) > 1.0e-5:
            self.last_target_yaw = math.atan2(velocity[1], velocity[0])
        else:
            self.last_target_yaw = fallback
        return self.last_target_yaw

    def make_odom(self, stamp, position, velocity, yaw, child_frame):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = child_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        qw, qx, qy, qz = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.w = qw
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        return msg

    def shared_from_observation(self, stamp):
        with self.lock:
            if not self.observations:
                return None
            source, (observation, received_at) = max(
                self.observations.items(),
                key=lambda item: item[1][1].to_sec(),
            )
        observation_stamp = observation.header.stamp if observation.header.stamp != rospy.Time() else received_at
        age = max(0.0, (stamp - observation_stamp).to_sec())
        if age > self.max_prediction_age:
            rospy.logwarn_throttle(1.0, 'Shared target observation is stale (age %.2f s)', age)
            return None
        position = np.array([
            observation.pose.pose.position.x,
            observation.pose.pose.position.y,
            observation.pose.pose.position.z,
        ], dtype=float)
        velocity = np.array([
            observation.twist.twist.linear.x,
            observation.twist.twist.linear.y,
            observation.twist.twist.linear.z,
        ], dtype=float)
        position += velocity * age
        yaw = quaternion_to_yaw(observation.pose.pose.orientation)
        return self.make_odom(stamp, position, velocity, yaw, 'shared_target_from_{}'.format(source))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.maybe_start(stamp)
            position, velocity, yaw = self.target_state(stamp)
            truth = self.make_odom(stamp, position, velocity, yaw, 'ground_truth_target')
            self.ground_truth_pub.publish(truth)
            if self.sharing_enabled:
                shared = self.shared_from_observation(stamp)
                if shared is not None:
                    self.shared_pub.publish(shared)
            else:
                truth.child_frame_id = 'baseline_truth_target'
                self.shared_pub.publish(truth)
            rate.sleep()


def main():
    rospy.init_node('target_state_coordinator')
    try:
        TargetStateCoordinator().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
