#!/usr/bin/env python3
"""Publish deterministic stage-2 scene geometry and RViz markers."""

import argparse
import json
import math
import sys

import rospy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from rao_2025_protocol import validate_manifest
from rao_scene_model import Annulus, Cube, make_scene_from_manifest


def marker_for_cube(index, obstacle, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "rao_2025_scene"
    marker.id = index
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = obstacle.center_x
    marker.pose.position.y = obstacle.center_y
    marker.pose.position.z = obstacle.height / 2.0
    marker.scale.x = marker.scale.y = obstacle.width
    marker.scale.z = obstacle.height
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.28, 0.34, 0.42, 0.8
    return marker


def marker_for_annulus(index, obstacle, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "rao_2025_annulus"
    marker.id = index
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = obstacle.wall_thickness
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.95, 0.55, 0.12, 0.95
    axis_x, axis_y = math.cos(obstacle.tilt_rad), math.sin(obstacle.tilt_rad)
    lateral_x, lateral_y = -axis_y, axis_x
    for step in range(65):
        angle = 2.0 * math.pi * step / 64.0
        marker.points.append(Point(
            x=obstacle.center_x + lateral_x * obstacle.outer_radius * math.cos(angle),
            y=obstacle.center_y + lateral_y * obstacle.outer_radius * math.cos(angle),
            z=obstacle.center_z + obstacle.outer_radius * math.sin(angle)))
    return marker


def build_markers(scene, frame_id):
    markers = MarkerArray()
    for index, obstacle in enumerate(scene.obstacles):
        if isinstance(obstacle, Cube):
            markers.markers.append(marker_for_cube(index, obstacle, frame_id))
        elif isinstance(obstacle, Annulus):
            markers.markers.append(marker_for_annulus(index, obstacle, frame_id))
    return markers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest-topic", default="/rao_2025/protocol/manifest")
    parser.add_argument("--frame-id", default="world")
    args = parser.parse_args([item for item in sys.argv[1:] if ":=" not in item])
    rospy.init_node("rao_2025_scene")
    manifest = json.loads(rospy.wait_for_message(args.manifest_topic, String, timeout=20.0).data)
    validate_manifest(manifest)
    scene = make_scene_from_manifest(manifest)
    geometry_pub = rospy.Publisher("~geometry", String, queue_size=1, latch=True)
    markers_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)
    ready_pub = rospy.Publisher("~ready", Bool, queue_size=1, latch=True)
    geometry_pub.publish(String(data=scene.as_json()))
    markers_pub.publish(build_markers(scene, args.frame_id))
    ready_pub.publish(Bool(data=True))
    rospy.loginfo("Rao scene ready: profile=%s obstacles=%d size=%s", manifest["profile"],
                  len(scene.obstacles), [scene.size_x, scene.size_y, scene.size_z])
    rospy.spin()


if __name__ == "__main__":
    main()
