#!/usr/bin/env python3

import argparse
import csv
import json
import os
import threading
import time
from collections import defaultdict

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    PositionCommand = None


FIELDS = [
    'wall_time',
    'ros_time',
    'uav',
    'mavros_pos_x',
    'mavros_pos_y',
    'mavros_pos_z',
    'musk_pos_x',
    'musk_pos_y',
    'musk_pos_z',
    'mavros_vel_x',
    'mavros_vel_y',
    'mavros_vel_z',
    'musk_vel_x',
    'musk_vel_y',
    'musk_vel_z',
    'setpoint_pos_x',
    'setpoint_pos_y',
    'setpoint_pos_z',
    'setpoint_vel_x',
    'setpoint_vel_y',
    'setpoint_vel_z',
    'ego_pos_x',
    'ego_pos_y',
    'ego_pos_z',
    'ego_vel_x',
    'ego_vel_y',
    'ego_vel_z',
]


def empty_sample():
    return {field: '' for field in FIELDS}


class DebugRecorder:
    def __init__(self, count, out_dir, rate_hz, scene_file=None):
        self.count = count
        self.rate_hz = rate_hz
        self.lock = threading.Lock()
        self.samples = defaultdict(empty_sample)
        self.offsets = self.load_offsets(scene_file)

        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.path = os.path.join(out_dir, f'uav_debug_{stamp}.csv')
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()

        self.subscribers = []
        for i in range(1, count + 1):
            drone = i - 1
            self.subscribers.append(rospy.Subscriber(
                f'/uav{i}/mavros/local_position/pose',
                PoseStamped,
                self.pose_cb,
                callback_args=i,
                queue_size=10
            ))
            self.subscribers.append(rospy.Subscriber(
                f'/uav{i}/mavros/local_position/velocity_local',
                TwistStamped,
                self.velocity_cb,
                callback_args=i,
                queue_size=10
            ))
            self.subscribers.append(rospy.Subscriber(
                f'/uav{i}/mavros/setpoint_raw/local',
                PositionTarget,
                self.setpoint_cb,
                callback_args=i,
                queue_size=10
            ))
            if PositionCommand is not None:
                self.subscribers.append(rospy.Subscriber(
                    f'/drone_{drone}_planning/pos_cmd',
                    PositionCommand,
                    self.ego_cb,
                    callback_args=i,
                    queue_size=10
                ))

        rospy.loginfo(f'Debug recorder writing {self.path}')

    def pose_cb(self, msg, uav):
        with self.lock:
            offset = self.offsets[uav - 1] if len(self.offsets) >= uav else [0.0, 0.0, 0.0]
            s = self.samples[uav]
            s['mavros_pos_x'] = msg.pose.position.x
            s['mavros_pos_y'] = msg.pose.position.y
            s['mavros_pos_z'] = msg.pose.position.z
            s['musk_pos_x'] = msg.pose.position.x + offset[0]
            s['musk_pos_y'] = msg.pose.position.y + offset[1]
            s['musk_pos_z'] = msg.pose.position.z + offset[2]

    def load_offsets(self, scene_file):
        offsets = [[0.0, 0.0, 0.0] for _ in range(self.count)]
        if not scene_file:
            return offsets
        try:
            with open(scene_file, 'r') as f:
                config = json.load(f)
            configured = config.get('mavrosLocalOffsetENU', [])
            for idx, offset in enumerate(configured[:self.count]):
                offsets[idx] = [float(offset[0]), float(offset[1]), float(offset[2])]
        except Exception as exc:
            rospy.logwarn(f'Could not load debug offsets from {scene_file}: {exc}')
        return offsets

    def velocity_cb(self, msg, uav):
        with self.lock:
            s = self.samples[uav]
            s['mavros_vel_x'] = msg.twist.linear.x
            s['mavros_vel_y'] = msg.twist.linear.y
            s['mavros_vel_z'] = msg.twist.linear.z
            s['musk_vel_x'] = msg.twist.linear.x
            s['musk_vel_y'] = msg.twist.linear.y
            s['musk_vel_z'] = msg.twist.linear.z

    def setpoint_cb(self, msg, uav):
        with self.lock:
            s = self.samples[uav]
            s['setpoint_pos_x'] = msg.position.x
            s['setpoint_pos_y'] = msg.position.y
            s['setpoint_pos_z'] = msg.position.z
            s['setpoint_vel_x'] = msg.velocity.x
            s['setpoint_vel_y'] = msg.velocity.y
            s['setpoint_vel_z'] = msg.velocity.z

    def ego_cb(self, msg, uav):
        with self.lock:
            s = self.samples[uav]
            s['ego_pos_x'] = msg.position.x
            s['ego_pos_y'] = msg.position.y
            s['ego_pos_z'] = msg.position.z
            s['ego_vel_x'] = msg.velocity.x
            s['ego_vel_y'] = msg.velocity.y
            s['ego_vel_z'] = msg.velocity.z

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            wall = rospy.get_time()
            with self.lock:
                for i in range(1, self.count + 1):
                    row = dict(self.samples[i])
                    row['wall_time'] = wall
                    row['ros_time'] = now
                    row['uav'] = i
                    self.writer.writerow(row)
                self.file.flush()
            rate.sleep()

    def close(self):
        self.file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', type=int, default=5)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--out-dir', default='/app/guidance/ros_ws/debug_logs')
    parser.add_argument('--scene-file', default='/app/guidance/ros_ws/src/multi_uav_formation/scenes/more_obstacles.json')
    args = parser.parse_args()

    rospy.init_node('multi_uav_debug_recorder', anonymous=True)
    recorder = DebugRecorder(args.count, args.out_dir, args.rate, args.scene_file)
    try:
        recorder.spin()
    finally:
        recorder.close()


if __name__ == '__main__':
    main()
