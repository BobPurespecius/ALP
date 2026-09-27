#!/usr/bin/env python3
"""Publish the stage-2 bicycle target and Hybrid A* execution state."""

import argparse
import json
import math
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from rao_2025_protocol import validate_manifest
from rao_scene_model import scene_from_dict
from rao_target_model import HybridAStar, TargetState, integrate_primitive, propagate


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TargetNode:
    def __init__(self, args):
        manifest_msg = rospy.wait_for_message(args.manifest_topic, String, timeout=20.0)
        self.manifest = json.loads(manifest_msg.data)
        validate_manifest(self.manifest)
        scene_msg = rospy.wait_for_message(args.scene_topic, String, timeout=20.0)
        self.scene = scene_from_dict(json.loads(scene_msg.data))
        self.profile = self.manifest["profile"]
        if self.scene.profile != self.profile or self.scene.protocol_sha256 != self.manifest["config_sha256"]:
            raise RuntimeError("scene geometry does not match the selected protocol manifest")
        target_config = dict(self.manifest["target_generation"])
        self.target_config = target_config
        self.altitude = float(target_config["altitude_m"])
        self.control_rate_hz = float(self.manifest["reproduction_timing"]["control_rate_hz"])
        self.control_dt = 1.0 / self.control_rate_hz
        self.max_speed = float(self.manifest["paper_common_parameters"]["target_max_velocity_mps"])
        self.planner = HybridAStar(self.scene, target_config, self.max_speed, self.control_rate_hz)
        initial = target_config["initial_state"][self.profile]
        self.state = TargetState.from_vector(initial)
        self.goals = [tuple(point) for point in target_config["waypoints"][self.profile]]
        self.goal_index = 0
        self.actions = []
        self.action_index = 0
        self.action_elapsed = 0.0
        self.pending_lock = threading.Lock()
        self.pending_goal_index = None
        self.pending_actions = None
        self.pending_error = None
        self.preplan_thread = None
        self.path = Path()
        self.path.header.frame_id = args.frame_id
        self.frame_id = args.frame_id
        self.ground_truth_pub = rospy.Publisher("~ground_truth", Odometry, queue_size=10)
        self.path_pub = rospy.Publisher("~path", Path, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("~state", String, queue_size=10)
        self.marker_pub = rospy.Publisher("~marker", Marker, queue_size=10)
        self.ready_pub = rospy.Publisher("~ready", Bool, queue_size=1, latch=True)
        self._plan_next_goal()
        self._start_preplan()
        self.ready_pub.publish(Bool(data=True))
        rospy.loginfo("Rao target ready: profile=%s initial=(%.2f, %.2f) goals=%d actions=%d",
                      self.profile, self.state.x, self.state.y, len(self.goals), len(self.actions))

    def _plan_next_goal(self):
        if not self.goals:
            self.actions = []
            return
        goal = self.goals[self.goal_index % len(self.goals)]
        self.actions = self.planner.plan(self.state, goal)
        self.action_index = 0
        self.action_elapsed = 0.0
        if not self.actions:
            raise RuntimeError("Hybrid A* did not find a route to goal {} at ({:.2f}, {:.2f})".format(
                self.goal_index, goal[0], goal[1]))

    def _predicted_endpoint(self):
        predicted = TargetState.from_vector(self.state.as_vector())
        for steering, acceleration in self.actions[self.action_index:]:
            predicted = integrate_primitive(
                predicted, steering, acceleration, self.planner.dt, self.control_dt,
                self.planner.wheelbase, self.max_speed)
        return predicted

    def _start_preplan(self):
        if not self.actions or self.preplan_thread is not None and self.preplan_thread.is_alive():
            return
        next_index = (self.goal_index + 1) % len(self.goals)
        start_state = self._predicted_endpoint()
        goal = self.goals[next_index]
        with self.pending_lock:
            self.pending_goal_index = next_index
            self.pending_actions = None
            self.pending_error = None

        def worker():
            try:
                result = self.planner.plan(start_state, goal)
                if not result:
                    raise RuntimeError("Hybrid A* found no route to goal {}".format(next_index))
                with self.pending_lock:
                    self.pending_actions = result
            except Exception as exc:
                with self.pending_lock:
                    self.pending_error = str(exc)

        self.preplan_thread = threading.Thread(target=worker, name="rao_target_preplan", daemon=True)
        self.preplan_thread.start()

    def _activate_preplan(self):
        with self.pending_lock:
            error = self.pending_error
            actions = self.pending_actions
            goal_index = self.pending_goal_index
        if error:
            raise RuntimeError(error)
        if actions is None:
            raise RuntimeError("next target segment was not planned before the current segment ended")
        current_goal = self.goals[self.goal_index]
        if math.hypot(self.state.x - current_goal[0], self.state.y - current_goal[1]) > self.planner.goal_tolerance:
            raise RuntimeError("target segment ended outside the execution tolerance")
        self.goal_index = goal_index
        self.actions = actions
        self.action_index = 0
        self.action_elapsed = 0.0
        self.preplan_thread = None
        self._start_preplan()
        return True

    def step(self):
        if not self.actions:
            self._activate_preplan()
        steering, acceleration = self.actions[self.action_index]
        self.state = propagate(self.state, steering, acceleration, self.control_dt,
                               self.planner.wheelbase, self.max_speed)
        self.action_elapsed += self.control_dt
        if self.action_elapsed >= self.planner.dt - 1e-9:
            self.action_elapsed = 0.0
            self.action_index += 1
            if self.action_index >= len(self.actions):
                self.actions = []
                self._activate_preplan()

    def publish(self):
        stamp = rospy.Time.now()
        quaternion = yaw_quaternion(self.state.yaw)
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.frame_id
        odometry.child_frame_id = "rao_target/base_link"
        odometry.pose.pose.position.x = self.state.x
        odometry.pose.pose.position.y = self.state.y
        odometry.pose.pose.position.z = self.altitude
        odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, \
            odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w = quaternion
        odometry.twist.twist.linear.x = self.state.speed * math.cos(self.state.yaw)
        odometry.twist.twist.linear.y = self.state.speed * math.sin(self.state.yaw)
        odometry.twist.twist.angular.z = self.state.speed * self.state.curvature
        self.ground_truth_pub.publish(odometry)

        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > 4000:
            self.path.poses = self.path.poses[-4000:]
        self.path_pub.publish(self.path)

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "rao_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = odometry.pose.pose
        marker.scale.x, marker.scale.y, marker.scale.z = 1.2, 0.18, 0.18
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.95, 1.0
        self.marker_pub.publish(marker)
        self.state_pub.publish(String(data=json.dumps({
            "state": self.state.as_vector(),
            "goal_index": self.goal_index,
            "goal": self.goals[self.goal_index % len(self.goals)],
            "remaining_actions": len(self.actions) - self.action_index,
        }, separators=(",", ":"))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest-topic", default="/rao_2025/protocol/manifest")
    parser.add_argument("--scene-topic", default="/rao_2025/scene/geometry")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args([item for item in sys.argv[1:] if ":=" not in item])
    rospy.init_node("rao_2025_target")
    target = TargetNode(args)
    rate = rospy.Rate(target.control_rate_hz)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        target.step()
        target.publish()
        if args.duration > 0.0 and (rospy.Time.now() - start_time).to_sec() >= args.duration:
            break
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == "__main__":
    main()
