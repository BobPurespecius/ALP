#!/usr/bin/env python3

import argparse
import json
import math
import os
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties, SetModelState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


def sorted_items(data):
    def key(item):
        name, _value = item
        try:
            return int(name)
        except ValueError:
            return name

    return sorted(data.items(), key=key)


def unit_axis(axis):
    if len(axis) < 2:
        return [1.0, 0.0]
    norm = math.hypot(float(axis[0]), float(axis[1]))
    if norm < 1e-6:
        return [1.0, 0.0]
    return [float(axis[0]) / norm, float(axis[1]) / norm]


def obstacle_pose_and_velocity(obstacle, t):
    center = obstacle.get('centerENU', [0.0, 0.0])
    axis = unit_axis(obstacle.get('axisENU', [1.0, 0.0]))
    amplitude = float(obstacle.get('amplitude', 0.0))
    period = max(float(obstacle.get('period', 1.0)), 1e-6)
    phase = float(obstacle.get('phase', 0.0))
    omega = 2.0 * math.pi / period
    angle = omega * t + phase
    offset = amplitude * math.sin(angle)
    speed = amplitude * omega * math.cos(angle)
    return (
        [float(center[0]) + axis[0] * offset, float(center[1]) + axis[1] * offset],
        [axis[0] * speed, axis[1] * speed],
    )


def load_moving_obstacles(scene_file):
    if not scene_file or not os.path.exists(scene_file):
        rospy.logwarn('Moving obstacle scene file not found: %s', scene_file)
        return []
    with open(scene_file, 'r') as f:
        config = json.load(f)
    return [
        (name, obstacle)
        for name, obstacle in sorted_items(config.get('movingObstacleData', {}))
    ]


def resolve_scene_file(scene_file, world_file):
    if scene_file and scene_file != '__auto__':
        if os.path.exists(scene_file):
            return scene_file
        package_scene = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'scenes',
            os.path.basename(scene_file)
        )
        if os.path.exists(package_scene):
            return package_scene
        return scene_file

    if not world_file:
        return ''

    scene_name = os.path.splitext(os.path.basename(world_file))[0]
    candidates = []
    world_dir = os.path.dirname(os.path.realpath(world_file)) if os.path.exists(world_file) else ''
    if os.path.basename(world_dir) == 'worlds':
        candidates.append(os.path.join(os.path.dirname(world_dir), 'scenes', f'{scene_name}.json'))
    candidates.append(os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'scenes',
        f'{scene_name}.json'
    ))

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return ''


def wait_for_models(get_world_properties, obstacles, timeout):
    expected = [
        obstacle.get('modelName', f'complex_moving_obstacle_{idx}')
        for idx, (_name, obstacle) in enumerate(obstacles)
    ]
    deadline = time.time() + timeout
    missing = expected

    while not rospy.is_shutdown():
        try:
            response = get_world_properties()
            model_names = set(response.model_names)
            missing = [name for name in expected if name not in model_names]
            if not missing:
                return True
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, 'Could not query Gazebo model list: %s', exc)

        if time.time() >= deadline:
            rospy.logwarn('Moving obstacle models not found in Gazebo: %s', ', '.join(missing))
            return False
        rospy.sleep(0.2)

    return False


def build_model_state(idx, obstacle, t, default_height):
    position, velocity = obstacle_pose_and_velocity(obstacle, t)
    height = float(obstacle.get('height', default_height))
    z_min = float(obstacle.get('zMin', 0.0))

    state = ModelState()
    state.model_name = obstacle.get('modelName', f'complex_moving_obstacle_{idx}')
    state.reference_frame = 'world'
    state.pose.position.x = position[0]
    state.pose.position.y = position[1]
    state.pose.position.z = z_min + height / 2.0
    state.pose.orientation.w = 1.0
    state.twist.linear.x = velocity[0]
    state.twist.linear.y = velocity[1]
    return state


def simulation_time(start_wall_time):
    try:
        now = rospy.Time.now().to_sec()
        if now > 0.0:
            return now
    except Exception:
        pass
    return time.time() - start_wall_time


def publish_dynamic_prediction_inputs(pose_pubs, marker_pub, idx, state, obstacle):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'world'
    pose.pose = state.pose
    pose_pubs[idx].publish(pose)

    marker = Marker()
    marker.header = pose.header
    marker.ns = 'dynamic_obstacles'
    marker.id = idx
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    radius = float(obstacle.get('radius', 0.5))
    height = float(obstacle.get('height', 3.0))
    marker.scale.x = 2.0 * radius
    marker.scale.y = 2.0 * radius
    marker.scale.z = height
    marker.pose = state.pose
    marker.color.r = 0.0
    marker.color.g = 0.8
    marker.color.b = 0.0
    marker.color.a = 0.8
    marker_pub.publish(marker)


def publish_obstacles(set_model_state, model_state_pub, pose_pubs, marker_pub,
                      obstacles, default_height,
                      start_wall_time, transport, debug=False):
    t = simulation_time(start_wall_time)
    for idx, (_name, obstacle) in enumerate(obstacles):
        state = build_model_state(idx, obstacle, t, default_height)

        if transport in ('topic', 'both'):
            model_state_pub.publish(state)

        if transport in ('service', 'both'):
            response = set_model_state(state)
            if not response.success:
                rospy.logwarn_throttle(
                    2.0,
                    'Gazebo rejected moving obstacle update for %s: %s',
                    state.model_name,
                    response.status_message
                )

        publish_dynamic_prediction_inputs(pose_pubs, marker_pub, idx, state, obstacle)

        if debug:
            rospy.loginfo_throttle(
                1.0,
                'Moving %s to x=%.2f y=%.2f using %s',
                state.model_name,
                state.pose.position.x,
                state.pose.position.y,
                transport
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-file', default='__auto__')
    parser.add_argument('--world-file', default='')
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--default-height', type=float, default=3.0)
    parser.add_argument('--model-wait-timeout', type=float, default=20.0)
    parser.add_argument('--transport', choices=['service', 'topic', 'both'], default='both')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    rospy.init_node('move_obstacles', anonymous=False)
    scene_file = resolve_scene_file(args.scene_file, args.world_file)
    rospy.loginfo('Moving obstacle scene file resolved to: %s', scene_file or '<none>')
    obstacles = load_moving_obstacles(scene_file)
    if not obstacles:
        rospy.loginfo('No moving obstacles configured; move_obstacles exits.')
        return

    rospy.loginfo('Moving obstacle controller loaded %d obstacles.', len(obstacles))
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/get_world_properties')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    pose_pubs = [
        rospy.Publisher(f'/dynamic/pose_{idx}', PoseStamped, queue_size=10)
        for idx in range(len(obstacles))
    ]
    marker_pub = rospy.Publisher('/dynamic/obj', Marker, queue_size=10)
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    wait_for_models(get_world_properties, obstacles, args.model_wait_timeout)
    rospy.sleep(0.5)

    start_wall_time = time.time()
    period = 1.0 / max(args.rate, 1e-6)
    while not rospy.is_shutdown():
        try:
            publish_obstacles(
                set_model_state,
                model_state_pub,
                pose_pubs,
                marker_pub,
                obstacles,
                args.default_height,
                start_wall_time,
                args.transport,
                debug=args.debug
            )
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, 'Failed to update moving obstacles: %s', exc)
        time.sleep(period)


if __name__ == '__main__':
    main()
