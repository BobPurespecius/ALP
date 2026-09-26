#!/usr/bin/env python3

import argparse
import copy
import json
import math
import os

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def model_candidates(uav_index, vehicle):
    model_id = uav_index - 1
    candidates = [
        f'{vehicle}_{model_id}',
        f'{vehicle}{model_id}',
        f'uav{uav_index}',
        f'uav_{uav_index}',
        f'iris_{model_id}',
        f'iris{model_id}',
    ]
    if uav_index == 1:
        candidates.insert(0, f'{vehicle}')
    return candidates


def load_expected_positions(scene_file, count):
    if not scene_file or scene_file == '__auto__' or not os.path.exists(scene_file):
        return {}

    try:
        with open(scene_file, 'r') as f:
            config = json.load(f)
    except (OSError, ValueError) as exc:
        rospy.logwarn('Could not load Gazebo pose bridge scene file %s: %s', scene_file, exc)
        return {}

    points = config.get('takeoffPointENU') or config.get('preparePointENU') or []
    expected = {}
    for idx, point in enumerate(points[:count], start=1):
        if len(point) >= 2:
            expected[idx] = [float(point[0]), float(point[1])]
    return expected


class GazeboPoseBridge:
    def __init__(self, count, vehicle, frame_id, scene_file='', marker_topic='/gazebo_visualization_marker_array',
                 publish_legacy_pose=False, publish_mavros_vision=False, use_truth_odom=False):
        self.count = count
        self.vehicle = vehicle
        self.frame_id = frame_id
        self.publish_legacy_pose = publish_legacy_pose
        self.publish_mavros_vision = publish_mavros_vision
        self.use_truth_odom = use_truth_odom
        self.truth_odom_timeout = rospy.Duration(0.5)
        self.last_truth_odom_time = {}
        self.expected_positions = load_expected_positions(scene_file, count)
        self.scene_models = self.load_scene_models(scene_file)
        self.model_names = {}
        self.gazebo_pose_publishers = {
            i: rospy.Publisher(f'/uav_{i}/gazebo_pose', PoseStamped, queue_size=10)
            for i in range(1, count + 1)
        }
        self.gazebo_odom_publishers = {
            i: rospy.Publisher(f'/uav_{i}/gazebo_odom', Odometry, queue_size=10)
            for i in range(1, count + 1)
        }
        if publish_mavros_vision:
            self.vision_pose_publishers = {
                i: rospy.Publisher(f'/uav{i}/mavros/vision_pose/pose', PoseStamped, queue_size=10)
                for i in range(1, count + 1)
            }
        else:
            self.vision_pose_publishers = {}
        if publish_legacy_pose:
            self.pose_publishers = {
                i: rospy.Publisher(f'/uav_{i}/pose', PoseStamped, queue_size=10)
                for i in range(1, count + 1)
            }
        else:
            self.pose_publishers = {}
        self.marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=10)
        self.subscriber = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.model_states_cb,
            queue_size=1
        )
        if use_truth_odom:
            self.truth_odom_subscribers = [
                rospy.Subscriber(
                    f'/uav{i}/gazebo/truth_odom',
                    Odometry,
                    self.truth_odom_cb,
                    callback_args=i,
                    queue_size=1
                )
                for i in range(1, count + 1)
            ]
        else:
            self.truth_odom_subscribers = []
        self.colors = [
            (1.0, 0.1, 0.0, 1.0),
            (1.0, 0.65, 0.0, 1.0),
            (0.2, 0.9, 1.0, 1.0),
            (0.1, 0.55, 1.0, 1.0),
            (0.1, 1.0, 0.45, 1.0),
        ]

    def load_scene_models(self, scene_file):
        if not scene_file or scene_file == '__auto__' or not os.path.exists(scene_file):
            return {}

        try:
            with open(scene_file, 'r') as f:
                config = json.load(f)
        except (OSError, ValueError) as exc:
            rospy.logwarn('Could not load Gazebo scene marker models from %s: %s', scene_file, exc)
            return {}

        models = {}
        world_name = os.path.splitext(os.path.basename(scene_file))[0]
        ego = config.get('egoPlanner') or {}
        default_height = float(ego.get('obstacleHeight', 3.0))

        def sorted_items(data):
            def key(item):
                name, _value = item
                try:
                    return int(name)
                except ValueError:
                    return name
            return sorted((data or {}).items(), key=key)

        marker_id = 1000
        for idx, (_key, obstacle) in enumerate(sorted_items(config.get('obstacleData'))):
            name = obstacle.get('modelName', f'{world_name}_obstacle_{idx}')
            models[name] = {
                'id': marker_id,
                'type': 'cylinder',
                'radius': float(obstacle.get('radius', 0.5)),
                'height': float(obstacle.get('height', default_height)),
                'color': (0.0, 0.8, 0.0, 0.85),
                'ns': 'gazebo_static_obstacles',
            }
            marker_id += 1

        for idx, (_key, obstacle) in enumerate(sorted_items(config.get('boxObstacleData'))):
            name = obstacle.get('modelName', f'{world_name}_box_obstacle_{idx}')
            size = obstacle.get('sizeENU', [1.0, 1.0, default_height])
            models[name] = {
                'id': marker_id,
                'type': 'box',
                'size': [float(size[0]), float(size[1]), float(size[2]) if len(size) >= 3 else default_height],
                'color': (0.0, 0.8, 0.0, 0.85),
                'ns': 'gazebo_box_obstacles',
            }
            marker_id += 1

        for idx, (_key, obstacle) in enumerate(sorted_items(config.get('movingObstacleData'))):
            name = obstacle.get('modelName', f'{world_name}_moving_obstacle_{idx}')
            models[name] = {
                'id': marker_id,
                'type': 'cylinder',
                'radius': float(obstacle.get('radius', 0.5)),
                'height': float(obstacle.get('height', default_height)),
                'color': (0.0, 0.8, 0.0, 0.85),
                'ns': 'gazebo_moving_obstacles',
            }
            marker_id += 1

        platform = config.get('platformData') or {}
        size = platform.get('sizeENU')
        if size is not None and len(size) >= 3:
            name = platform.get('modelName', 'goal_platform')
            models[name] = {
                'id': marker_id,
                'type': 'box',
                'size': [float(size[0]), float(size[1]), float(size[2])],
                'color': (0.48, 0.48, 0.48, 1.0),
                'ns': 'gazebo_platforms',
            }

        return models

    def ignored_model_name(self, name):
        lower = name.lower()
        ignored_words = ('ground', 'sun', 'obstacle', 'platform')
        return any(word in lower for word in ignored_words)

    def resolve_model_name(self, names, uav_index, poses=None, used_names=None):
        if used_names is None:
            used_names = set()

        exact_name = self.resolve_exact_model_name(names, uav_index, used_names=used_names)
        if exact_name is not None:
            return exact_name

        model_id = str(uav_index - 1)
        for name in names:
            if name in used_names:
                continue
            if name.endswith(f'_{model_id}') and self.vehicle in name:
                return name
            if name.endswith(f'/{self.vehicle}_{model_id}') or name.endswith(f'::{self.vehicle}_{model_id}'):
                return name

        expected = self.expected_positions.get(uav_index)
        if poses is None or expected is None:
            return None

        position_model_name = self.resolve_model_name_by_position(
            names,
            poses,
            expected,
            used_names=used_names,
        )
        if position_model_name is not None:
            return position_model_name
        return None

    def resolve_model_name_by_position(self, names, poses, expected, used_names=None):
        if used_names is None:
            used_names = set()

        best_name = None
        best_distance = math.inf
        for idx, name in enumerate(names):
            if name in used_names or self.ignored_model_name(name):
                continue

            lower = name.lower()
            if self.vehicle not in lower and 'uav' not in lower and 'iris' not in lower:
                continue

            pose = poses[idx]
            distance = math.hypot(
                pose.position.x - expected[0],
                pose.position.y - expected[1]
            )
            if distance < best_distance:
                best_name = name
                best_distance = distance

        if best_name is not None and best_distance <= 0.8:
            return best_name
        return None

    def resolve_exact_model_name(self, names, uav_index, used_names=None):
        if used_names is None:
            used_names = set()

        for candidate in model_candidates(uav_index, self.vehicle):
            if candidate in names and candidate not in used_names:
                return candidate
        return None

    def has_recent_truth_odom(self, uav_index):
        stamp = self.last_truth_odom_time.get(uav_index)
        if stamp is None:
            return False
        return rospy.Time.now() - stamp <= self.truth_odom_timeout

    def publish_pose(self, uav_index, pose_msg):
        self.gazebo_pose_publishers[uav_index].publish(pose_msg)
        if self.publish_legacy_pose:
            self.pose_publishers[uav_index].publish(pose_msg)
        vision_pub = self.vision_pose_publishers.get(uav_index)
        if vision_pub is not None:
            vision_pub.publish(pose_msg)

    def publish_odom(self, uav_index, pose, twist, stamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = f'uav{uav_index}/base_link'
        odom_msg.pose.pose = copy.deepcopy(pose)
        odom_msg.twist.twist = copy.deepcopy(twist)
        self.gazebo_odom_publishers[uav_index].publish(odom_msg)

    def truth_odom_cb(self, msg, uav_index):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = msg.pose.pose
        self.last_truth_odom_time[uav_index] = rospy.Time.now()
        self.publish_pose(uav_index, pose_msg)
        self.publish_odom(uav_index, msg.pose.pose, msg.twist.twist, pose_msg.header.stamp)

    def model_states_cb(self, msg):
        names = msg.name
        marker_array = MarkerArray()
        used_names = set()
        for uav_index, publisher in self.gazebo_pose_publishers.items():
            if self.use_truth_odom and self.has_recent_truth_odom(uav_index):
                continue
            old_model_name = self.model_names.get(uav_index)
            model_name = old_model_name if old_model_name in names and old_model_name not in used_names else None

            expected = self.expected_positions.get(uav_index)
            if model_name is None and expected is not None:
                model_name = self.resolve_model_name_by_position(
                    names,
                    msg.pose,
                    expected,
                    used_names=used_names,
                )
            if model_name is None:
                exact_model_name = self.resolve_exact_model_name(
                    names,
                    uav_index,
                    used_names=used_names
                )
                model_name = exact_model_name or self.model_names.get(uav_index)
            if model_name not in names or model_name in used_names:
                model_name = self.resolve_model_name(
                    names,
                    uav_index,
                    poses=msg.pose,
                    used_names=used_names
                )
            if model_name is None:
                rospy.logwarn_throttle(
                    5.0,
                    'Could not resolve Gazebo model for uav%d. Available: %s',
                    uav_index,
                    ', '.join(names)
                )
                continue

            if old_model_name != model_name:
                self.model_names[uav_index] = model_name
                rospy.loginfo('Mapped /uav_%d/gazebo_pose to Gazebo model %s', uav_index, model_name)
            used_names.add(model_name)

            idx = names.index(model_name)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose = msg.pose[idx]
            self.publish_pose(uav_index, pose_msg)
            self.publish_odom(uav_index, msg.pose[idx], msg.twist[idx], pose_msg.header.stamp)
            marker_array.markers.extend(self.uav_markers(uav_index, pose_msg))

        marker_array.markers.extend(self.scene_markers(msg))
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def scene_markers(self, msg):
        markers = []
        for name, model in self.scene_models.items():
            if name not in msg.name:
                continue

            idx = msg.name.index(name)
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.frame_id
            marker.ns = model['ns']
            marker.id = model['id']
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(msg.pose[idx])
            marker.lifetime = rospy.Duration(0.5)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = model['color']

            if model['type'] == 'cylinder':
                marker.type = Marker.CYLINDER
                marker.scale.x = 2.0 * model['radius']
                marker.scale.y = 2.0 * model['radius']
                marker.scale.z = model['height']
            elif model['type'] == 'box':
                marker.type = Marker.CUBE
                marker.scale.x, marker.scale.y, marker.scale.z = model['size']
            else:
                continue

            markers.append(marker)
        return markers

    def uav_markers(self, uav_index, pose_msg):
        color = self.colors[(uav_index - 1) % len(self.colors)]
        markers = []

        body = Marker()
        body.header = pose_msg.header
        body.ns = 'uavs'
        body.id = uav_index
        body.type = Marker.SPHERE
        body.action = Marker.ADD
        body.pose = copy.deepcopy(pose_msg.pose)
        body.scale.x = 0.45
        body.scale.y = 0.45
        body.scale.z = 0.18
        body.color.r, body.color.g, body.color.b, body.color.a = color
        body.lifetime = rospy.Duration(0.5)
        markers.append(body)

        label = Marker()
        label.header = pose_msg.header
        label.ns = 'uav_labels'
        label.id = uav_index
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = copy.deepcopy(pose_msg.pose)
        label.pose.position.z += 0.45
        label.scale.z = 0.35
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = f'uav{uav_index}'
        label.lifetime = rospy.Duration(0.5)
        markers.append(label)

        return markers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', type=int, default=5)
    parser.add_argument('--vehicle', default='iris')
    parser.add_argument('--frame-id', default='map')
    parser.add_argument('--scene-file', default='')
    parser.add_argument('--marker-topic', default='/gazebo_visualization_marker_array')
    parser.add_argument(
        '--publish-legacy-pose',
        action='store_true',
        help='Also publish Gazebo truth to /uav_i/pose. Disabled by default to avoid mixing display truth with control/communication pose topics.'
    )
    parser.add_argument(
        '--publish-mavros-vision',
        action='store_true',
        help='Also publish Gazebo truth pose to /uavN/mavros/vision_pose/pose for PX4 external-vision EKF.'
    )
    parser.add_argument(
        '--use-truth-odom',
        action='store_true',
        help='Prefer each model p3d odometry topic /uavN/gazebo/truth_odom when it is available.'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('gazebo_pose_bridge', anonymous=False)
    GazeboPoseBridge(
        args.count,
        args.vehicle,
        args.frame_id,
        scene_file=args.scene_file,
        marker_topic=args.marker_topic,
        publish_legacy_pose=args.publish_legacy_pose,
        publish_mavros_vision=args.publish_mavros_vision,
        use_truth_odom=args.use_truth_odom
    )
    rospy.spin()


if __name__ == '__main__':
    main()
