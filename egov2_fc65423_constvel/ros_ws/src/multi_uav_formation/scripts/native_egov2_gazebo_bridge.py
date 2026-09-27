#!/usr/bin/env python3

import argparse
import json
import math
import threading

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header

from quadrotor_msgs.msg import PositionCommand


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def load_scene(scene_file):
    with open(scene_file, 'r') as stream:
        return json.load(stream)


def build_cylinder_cloud(scene, resolution):
    ego = scene.get('egoPlanner') or {}
    resolution = max(float(resolution or ego.get('cloudResolution', 0.22)), 0.05)
    points = []
    obstacles = scene.get('obstacleData') or {}
    for obstacle in obstacles.values():
        center = obstacle.get('centerENU', [0.0, 0.0])
        cx, cy = float(center[0]), float(center[1])
        radius = float(obstacle.get('radius', 0.5))
        z_min = float(obstacle.get('zMin', 0.0))
        height = float(obstacle.get('height', ego.get('obstacleHeight', 3.0)))
        x = cx - radius
        while x <= cx + radius + resolution * 0.5:
            y = cy - radius
            while y <= cy + radius + resolution * 0.5:
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2:
                    z = z_min
                    while z <= z_min + height + resolution * 0.5:
                        points.append((float(x), float(y), float(z)))
                        z += resolution
                y += resolution
            x += resolution
    for wall in (scene.get('wallData') or {}).values():
        center = wall.get('centerENU', [0.0, 0.0])
        size = wall.get('sizeENU', [1.0, 0.2, ego.get('obstacleHeight', 3.0)])
        if len(size) < 2:
            raise ValueError('wall sizeENU needs at least length and thickness')
        half_x = 0.5 * float(size[0])
        half_y = 0.5 * float(size[1])
        height = float(size[2] if len(size) >= 3 else
                       wall.get('height', ego.get('obstacleHeight', 3.0)))
        z_min = float(wall.get('zMin', 0.0))
        yaw = float(wall.get('yawRad', 0.0))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        x_count = max(1, int(math.ceil(2.0 * half_x / resolution)))
        y_count = max(1, int(math.ceil(2.0 * half_y / resolution)))
        z_count = max(1, int(math.ceil(height / resolution)))
        for x_index in range(x_count + 1):
            local_x = -half_x + 2.0 * half_x * x_index / float(x_count)
            for y_index in range(y_count + 1):
                local_y = -half_y + 2.0 * half_y * y_index / float(y_count)
                world_x = float(center[0]) + cosine * local_x - sine * local_y
                world_y = float(center[1]) + sine * local_x + cosine * local_y
                for z_index in range(z_count + 1):
                    points.append((world_x, world_y,
                                   z_min + height * z_index / float(z_count)))
    return points


def target_sdf(model_name='virtual_tracking_target', radius=0.25):
    return """
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <visual name="target_visual">
        <geometry><sphere><radius>{radius:.3f}</radius></sphere></geometry>
        <material>
          <ambient>1.0 0.05 0.02 1.0</ambient>
          <diffuse>1.0 0.05 0.02 1.0</diffuse>
          <emissive>0.35 0.02 0.01 1.0</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(name=model_name, radius=radius)


class NativeEgoV2GazeboBridge:
    def __init__(self, count, scene_file, cloud_period, cloud_resolution):
        self.count = count
        self.lock = threading.Lock()
        self.states = [State() for _ in range(count)]
        self.poses = [None for _ in range(count)]
        self.commands = [None for _ in range(count)]
        self.command_times = [rospy.Time(0) for _ in range(count)]
        self.last_mode_request = [rospy.Time(0) for _ in range(count)]
        self.last_arm_request = [rospy.Time(0) for _ in range(count)]
        self.started_at = rospy.Time.now()
        self.target_model = 'virtual_tracking_target'
        self.target_spawned = False
        self.last_target_spawn_attempt = rospy.Time(0)

        scene = load_scene(scene_file)
        ego = scene.get('egoPlanner') or {}
        takeoff_points = scene.get('takeoffPointENU') or []
        self.hold_points = []
        for index in range(count):
            point = takeoff_points[index] if index < len(takeoff_points) else [0.0, float(index), 1.5]
            self.hold_points.append((float(point[0]), float(point[1]), float(point[2])))

        self.cloud_points = build_cylinder_cloud(
            scene,
            cloud_resolution if cloud_resolution > 0.0 else ego.get('cloudResolution', 0.22),
        )
        self.cloud_period = max(float(cloud_period), 0.1)

        self.odom_pubs = []
        self.cloud_pubs = []
        self.setpoint_pubs = []
        self.ready_pubs = []
        self.state_subs = []
        self.odom_subs = []
        self.command_subs = []
        self.arm_services = []
        self.mode_services = []
        self.target_state_pub = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=1)
        self.target_sub = rospy.Subscriber(
            '/target_tracking/ground_truth', Odometry,
            self.target_callback, queue_size=10)

        for index in range(count):
            uav = index + 1
            self.odom_pubs.append(rospy.Publisher(
                '/drone_{}_visual_slam/odom'.format(index), Odometry, queue_size=10))
            self.cloud_pubs.append(rospy.Publisher(
                '/drone_{}_pcl_render_node/cloud'.format(index),
                PointCloud2,
                queue_size=1,
                latch=True))
            self.setpoint_pubs.append(rospy.Publisher(
                '/uav{}/mavros/setpoint_position/local'.format(uav),
                PoseStamped,
                queue_size=20))
            self.ready_pubs.append(rospy.Publisher(
                '/target_tracking/ready/uav{}'.format(uav),
                Bool,
                queue_size=1,
                latch=True))
            self.state_subs.append(rospy.Subscriber(
                '/uav{}/mavros/state'.format(uav), State,
                self.state_callback, index, queue_size=10))
            self.odom_subs.append(rospy.Subscriber(
                '/uav_{}/gazebo_odom'.format(uav), Odometry,
                self.odom_callback, index, queue_size=10))
            self.command_subs.append(rospy.Subscriber(
                '/drone_{}_planning/pos_cmd'.format(index), PositionCommand,
                self.command_callback, index, queue_size=10))
            self.arm_services.append(rospy.ServiceProxy(
                '/uav{}/mavros/cmd/arming'.format(uav), CommandBool))
            self.mode_services.append(rospy.ServiceProxy(
                '/uav{}/mavros/set_mode'.format(uav), SetMode))

        self.cloud_timer = rospy.Timer(rospy.Duration(self.cloud_period), self.publish_clouds)
        self.setpoint_timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_setpoints)

    def state_callback(self, msg, index):
        with self.lock:
            self.states[index] = msg

    def odom_callback(self, msg, index):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose = msg.pose
        odom.twist = msg.twist
        with self.lock:
            self.poses[index] = odom
        self.odom_pubs[index].publish(odom)

    def command_callback(self, msg, index):
        values = (msg.position.x, msg.position.y, msg.position.z, msg.yaw)
        if not all(math.isfinite(value) for value in values):
            return
        with self.lock:
            self.commands[index] = msg
            self.command_times[index] = rospy.Time.now()

    def target_callback(self, msg):
        now = rospy.Time.now()
        if not self.target_spawned and (now - self.last_target_spawn_attempt).to_sec() >= 2.0:
            self.last_target_spawn_attempt = now
            try:
                rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=0.5)
                spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                pose = Pose()
                pose.position.x = msg.pose.pose.position.x
                pose.position.y = msg.pose.pose.position.y
                pose.position.z = msg.pose.pose.position.z
                pose.orientation.w = 1.0
                result = spawn(self.target_model, target_sdf(), '', pose, 'world')
                self.target_spawned = bool(result.success) or 'already exists' in result.status_message.lower()
                if self.target_spawned:
                    rospy.loginfo('Spawned Gazebo target model: %s', self.target_model)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn_throttle(5.0, 'Waiting for Gazebo target model: %s', exc)

        if not self.target_spawned:
            return

        state = ModelState()
        state.model_name = self.target_model
        state.reference_frame = 'world'
        state.pose = msg.pose.pose
        self.target_state_pub.publish(state)

    def publish_clouds(self, _event):
        if not self.cloud_points:
            return
        header = Header(stamp=rospy.Time.now(), frame_id='world')
        message = point_cloud2.create_cloud_xyz32(header, self.cloud_points)
        for publisher in self.cloud_pubs:
            publisher.publish(message)

    def current_setpoint(self, index):
        with self.lock:
            command = self.commands[index]
            odom = self.poses[index]
        if (command is not None and
                command.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_READY):
            return command.position.x, command.position.y, command.position.z, command.yaw
        x, y, z = self.hold_points[index]
        return x, y, z, 0.0

    def request_px4_mode(self, index):
        now = rospy.Time.now()
        state = self.states[index]
        if not state.connected or (now - self.started_at).to_sec() < 2.0:
            return
        if state.mode != 'OFFBOARD' and (now - self.last_mode_request[index]).to_sec() > 2.0:
            try:
                self.mode_services[index](base_mode=0, custom_mode='OFFBOARD')
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, 'UAV%d OFFBOARD request failed: %s', index + 1, exc)
            self.last_mode_request[index] = now
        if state.mode != 'OFFBOARD':
            return
        if not state.armed and (now - self.last_arm_request[index]).to_sec() > 2.0:
            try:
                self.arm_services[index](value=True)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, 'UAV%d arm request failed: %s', index + 1, exc)
            self.last_arm_request[index] = now

    def publish_ready(self, index):
        with self.lock:
            state = self.states[index]
            odom = self.poses[index]
        takeoff_z = self.hold_points[index][2]
        ready = (
            state.connected and state.armed and state.mode == 'OFFBOARD' and
            odom is not None and odom.pose.pose.position.z >= takeoff_z - 0.2
        )
        self.ready_pubs[index].publish(Bool(data=ready))

    def publish_setpoints(self, event):
        stamp = event.current_real
        for index, publisher in enumerate(self.setpoint_pubs):
            x, y, z, yaw = self.current_setpoint(index)
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = 'world'
            message.pose.position.x = x
            message.pose.position.y = y
            message.pose.position.z = z
            qx, qy, qz, qw = yaw_quaternion(yaw)
            message.pose.orientation.x = qx
            message.pose.orientation.y = qy
            message.pose.orientation.z = qz
            message.pose.orientation.w = qw
            publisher.publish(message)
            self.request_px4_mode(index)
            self.publish_ready(index)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--scene-file', required=True)
    parser.add_argument('--cloud-period', type=float, default=0.5)
    parser.add_argument('--cloud-resolution', type=float, default=0.22)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('native_egov2_gazebo_bridge', anonymous=False)
    NativeEgoV2GazeboBridge(
        args.count,
        args.scene_file,
        args.cloud_period,
        args.cloud_resolution,
    )
    rospy.spin()


if __name__ == '__main__':
    main()
