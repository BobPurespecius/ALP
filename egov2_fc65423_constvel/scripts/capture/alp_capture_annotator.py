#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ALP 抓帧工作流 S2a:capture 侧标注节点(纯可视化 + 时钟采样)。

职责(全部只读于运行、只写 capture 目录、不向规划链路发布任何东西):
  1. 订阅三机 odom 与目标真值,在 /alp_capture/vehicle_overlays 上发布三个
     实心覆盖球(每机一色)。生产里三架 UAV 用同一套同色 mesh,俯视下无法
     用像素区分;覆盖球放在机体位置上,颜色即身份。
  2. 广播 TF: world -> alp_capture/target,供派生 RViz 配置的 Orbit 相机
     追踪目标红球。帧名带 alp_capture/ 前缀,规划栈没有任何消费者。
  3. 以 10 Hz 把 (wall_ns, ros_ns) 采样写入 viz_clock.jsonl —— 抓帧脚本
     用它把"第几帧"插值到 ros/日志时间轴。本仿真日志时间戳是 wall epoch,
     wall_ns 一列即可直接与 roslaunch_stdout.log 对齐;ros_ns 备用。

红线:不改场景、不改路线、不改阈值、不喂 planner;发布只出现在
/alp_capture/* 与 alp_capture/* 帧,生产节点没有订阅者。
"""
import argparse
import json
import time

import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

# 与 analyze_alp_pixels.py 的 HSV 区间一一对应(整数值 0-255)。
# 选取原则:避开场景已有色相(森林绿 ~135、动态障碍琥珀 ~40、
# 可见性墙蓝 ~223、红球 ~2、FOV 线青/橙/紫),且三机之间互相远离。
UAV_COLORS = {
    0: (1.0, 0.0, 0.65),    # magenta -> hue ~321
    1: (0.60, 1.0, 0.0),    # lime    -> hue ~84
    2: (1.0, 0.15, 0.35),   # rose    -> hue ~346
}
OVERLAY_RADIUS = 0.45
OVERLAY_ALPHA = 0.92
FRESH_S = 1.5

DRONE_ODOM_TOPICS = ['/drone_{}_visual_slam/odom'.format(i) for i in range(3)]
TARGET_ODOM_TOPIC = '/target_tracking/ground_truth'


class Annotator(object):
    def __init__(self, viz_clock_path, publish_overlays=True):
        # publish_overlays=False 时不发覆盖球，保留生产自带的正常无人机模型
        # （三机是同一套同色 mesh，像素法分不开；需要按机辨识时才开覆盖球）。
        self.publish_overlays_enabled = bool(publish_overlays)
        self.markers_pub = rospy.Publisher(
            '/alp_capture/vehicle_overlays', MarkerArray, queue_size=1)
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=1)
        self.latest_odom = {i: None for i in range(3)}
        self.target_pose = None
        self.clock_file = open(viz_clock_path, 'a', buffering=1)
        for i, topic in enumerate(DRONE_ODOM_TOPICS):
            rospy.Subscriber(topic, Odometry, self.make_odom_cb(i),
                             queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(TARGET_ODOM_TOPIC, Odometry, self.target_cb,
                         queue_size=1, tcp_nodelay=True)

    def make_odom_cb(self, index):
        def cb(msg):
            p = msg.pose.pose.position
            self.latest_odom[index] = (p.x, p.y, p.z, rospy.Time.now().to_sec())
        return cb

    def target_cb(self, msg):
        p = msg.pose.pose.position
        self.target_pose = (p.x, p.y, p.z, rospy.Time.now().to_sec())

    def sample_clock(self):
        now_wall_ns = time.time_ns()
        now_ros_ns = rospy.rostime.get_rostime().to_nsec()
        self.clock_file.write(json.dumps(
            {'wall_ns': now_wall_ns, 'ros_ns': now_ros_ns}) + '\n')

    def publish_overlays(self):
        if not self.publish_overlays_enabled:
            return
        now = rospy.Time.now().to_sec()
        arr = MarkerArray()
        for index, pose in self.latest_odom.items():
            if pose is None or now - pose[3] > FRESH_S:
                continue
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = 'world'
            m.ns = 'alp_capture_uav_{}'.format(index)
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pose[0]
            m.pose.position.y = pose[1]
            m.pose.position.z = pose[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 2.0 * OVERLAY_RADIUS
            r, g, b = UAV_COLORS[index]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, OVERLAY_ALPHA
            m.lifetime = rospy.Duration(0.6)
            arr.markers.append(m)
        if arr.markers:
            self.markers_pub.publish(arr)

    def broadcast_target_tf(self):
        if self.target_pose is None:
            return
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = 'alp_capture/target'
        t.transform.translation.x = self.target_pose[0]
        t.transform.translation.y = self.target_pose[1]
        t.transform.translation.z = self.target_pose[2]
        t.transform.rotation.w = 1.0
        self.tf_pub.publish(TFMessage(transforms=[t]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--viz-clock', required=True)
    ap.add_argument('--overlays', choices=('spheres', 'none'), default='spheres',
                    help='spheres=发布三机覆盖球(像素法辨识用); '
                         'none=不发覆盖球,保留生产正常模型')
    # rospy 消费它自己的 argv;把未知参数隔离掉
    args, _ = ap.parse_known_args(
        [a for a in __import__('sys').argv[1:]
         if not a.startswith('__')])
    rospy.init_node('alp_capture_annotator', anonymous=False)
    ann = Annotator(args.viz_clock,
                    publish_overlays=(args.overlays == 'spheres'))
    rate = rospy.Rate(20.0)
    clock_count = 0
    while not rospy.is_shutdown():
        ann.publish_overlays()
        ann.broadcast_target_tf()
        clock_count += 1
        if clock_count % 2 == 0:  # 20 Hz 主循环 -> 10 Hz 时钟采样
            ann.sample_clock()
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    ann.clock_file.flush()


if __name__ == '__main__':
    main()
