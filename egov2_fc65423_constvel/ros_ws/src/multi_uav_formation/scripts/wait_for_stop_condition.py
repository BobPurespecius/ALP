#!/usr/bin/env python3

import argparse
import json
import math
import os
import re
import sys
import time

import rospy
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty, Trigger


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def status_json(response_message):
    marker = ' json='
    if marker not in response_message:
        return {}
    try:
        return json.loads(response_message.split(marker, 1)[1])
    except json.JSONDecodeError:
        return {}


def status_value(response_message, key):
    data = status_json(response_message)
    if key in data:
        return data[key]
    match = re.search(rf'{re.escape(key)}=([^ ]+)', response_message)
    return match.group(1) if match else None


def goal_reached(response_message):
    return (
        'egoGoalReached=1' in response_message or
        re.search(r'egoGoalReached[^A-Za-z0-9_]+true', response_message) is not None
    )


def actual_goal_reached(response_message, xy_tolerance, z_tolerance):
    xy_error = finite_float(status_value(response_message, 'egoGoalXYError'))
    z_error = finite_float(status_value(response_message, 'egoGoalZError'))
    if xy_error is None or z_error is None:
        return False
    return xy_error <= xy_tolerance and z_error <= z_tolerance


def scene_goal_tolerances(scene_file):
    fallback_xy = 0.45
    fallback_z = 0.12
    if not scene_file or not os.path.exists(scene_file):
        return fallback_xy, fallback_z

    try:
        with open(scene_file, 'r') as f:
            config = json.load(f)
    except (OSError, json.JSONDecodeError):
        return fallback_xy, fallback_z

    ego = config.get('egoPlanner', {})
    platform = config.get('platformData', {})

    xy_tolerance = ego.get('goalXYTolerance', ego.get('goalTolerance', fallback_xy))
    z_tolerance = ego.get('goalZTolerance', fallback_z)
    if platform.get('landOnTop', False):
        xy_tolerance = platform.get('landingXYTolerance', xy_tolerance)
        z_tolerance = platform.get('landingHeightTolerance', z_tolerance)

    return float(xy_tolerance), float(z_tolerance)


def pause_gazebo():
    try:
        rospy.wait_for_service('/gazebo/pause_physics', timeout=1.0)
        rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--expected-uavs', type=int, required=True)
    parser.add_argument('--scene-file', default='')
    parser.add_argument('--goal-check-period', type=float, default=0.5)
    parser.add_argument('--timeout-scale', type=float, default=20.0)
    args = parser.parse_args()
    xy_tolerance, z_tolerance = scene_goal_tolerances(args.scene_file)

    rospy.init_node('multi_uav_batch_stop_waiter', anonymous=True)

    state = {
        'start_sim_time': None,
        'current_sim_time': None,
        'stop_reason': None,
        'last_goal_check_wall': 0.0,
    }

    def clock_callback(msg):
        sim_time = msg.clock.to_sec()
        if state['start_sim_time'] is None:
            state['start_sim_time'] = sim_time
        state['current_sim_time'] = sim_time
        if sim_time - state['start_sim_time'] >= args.duration:
            elapsed = sim_time - state['start_sim_time']
            state['stop_reason'] = f'sim_time_duration:{elapsed:.3f}'
            pause_gazebo()

    rospy.Subscriber('/clock', Clock, clock_callback, queue_size=10)

    wall_deadline = time.time() + max(args.duration * args.timeout_scale, 30.0)
    status_services = [
        rospy.ServiceProxy(f'/single_run_{uav}/status', Trigger)
        for uav in range(1, args.expected_uavs + 1)
    ]

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if state['stop_reason']:
            print(state['stop_reason'])
            return 0

        now = time.time()
        if now >= wall_deadline:
            print('wall_timeout')
            pause_gazebo()
            return 0

        if now - state['last_goal_check_wall'] >= args.goal_check_period:
            state['last_goal_check_wall'] = now
            reached = 0
            for service in status_services:
                try:
                    message = service().message
                    if (
                        goal_reached(message) or
                        actual_goal_reached(message, xy_tolerance, z_tolerance)
                    ):
                        reached += 1
                except Exception:
                    break
            if reached == args.expected_uavs:
                state['stop_reason'] = 'all_goals_reached'
                pause_gazebo()
                continue

        rate.sleep()

    print('ros_shutdown')
    return 0


if __name__ == '__main__':
    sys.exit(main())
