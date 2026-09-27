#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry
from object_detection_msgs.msg import BoundingBox, BoundingBoxes


def make_odom(stamp):
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = "world"
    msg.child_frame_id = "drone_0"
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 1.5
    msg.pose.pose.orientation.w = 1.0
    return msg


def make_bbox(stamp, target_x, target_y, target_z):
    fx = rospy.get_param("~cam_fx", 846.044120)
    fy = rospy.get_param("~cam_fy", 846.044120)
    cx = rospy.get_param("~cam_cx", 639.5)
    cy = rospy.get_param("~cam_cy", 359.5)
    object_height = rospy.get_param("~object_height", 0.7)

    # Invert detection_filter's default R_cam:
    # world-relative point [x, y, z] = [cam_x, cam_z, -cam_y].
    cam_x = target_x
    cam_y = 1.5 - target_z
    cam_z = target_y

    if cam_z <= 0.2:
        return None

    center_u = fx * cam_x / cam_z + cx
    center_v = fy * cam_y / cam_z + cy
    height_px = fy * object_height / cam_z
    width_px = height_px * 0.7

    bbox = BoundingBox()
    bbox.id = 0
    bbox.Class = "sim_target"
    bbox.probability = 0.95
    bbox.xmin = max(0, int(center_u - width_px / 2.0))
    bbox.xmax = min(1279, int(center_u + width_px / 2.0))
    bbox.ymin = max(6, int(center_v - height_px / 2.0))
    bbox.ymax = min(714, int(center_v + height_px / 2.0))

    if bbox.xmax <= bbox.xmin or bbox.ymax <= bbox.ymin:
        return None

    msg = BoundingBoxes()
    msg.header.stamp = stamp
    msg.header.frame_id = "detection"
    msg.bounding_boxes.append(bbox)
    return msg


def main():
    rospy.init_node("sim_tracking_inputs")
    odom_pub = rospy.Publisher("drone_odom", Odometry, queue_size=10)
    bbox_pub = rospy.Publisher("bboxes", BoundingBoxes, queue_size=10)

    rate_hz = rospy.get_param("~rate", 20.0)
    rate = rospy.Rate(rate_hz)
    start = rospy.Time.now()

    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        t = (stamp - start).to_sec()

        target_x = 0.8 * math.sin(0.35 * t)
        target_y = 5.0 + 0.8 * math.cos(0.25 * t)
        target_z = 0.7

        odom_pub.publish(make_odom(stamp))
        bbox_msg = make_bbox(stamp, target_x, target_y, target_z)
        if bbox_msg is not None:
            bbox_pub.publish(bbox_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
