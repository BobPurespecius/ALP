# import rospy
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# import math
# import time

# class Visualizer:
#     def __init__(self):
#         self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
#         self.markers = []
#         self.moving = {}            # id -> (vx, vy)
#         self.last_update = rospy.Time.now()

#     def add_cylinder(self, position_xy, radius):
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.ns = "cylinders"
#         marker.id = len(self.markers)
#         marker.type = Marker.CYLINDER
#         marker.action = Marker.ADD

#         marker.pose.position.x = position_xy[0]
#         marker.pose.position.y = position_xy[1]
#         marker.pose.position.z = 2.5
#         marker.pose.orientation.w = 1.0

#         marker.scale.x = 2 * radius
#         marker.scale.y = 2 * radius
#         marker.scale.z = 5.0

#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0

#         marker.lifetime = rospy.Duration(0)

#         self.markers.append(marker)

    # def add_box(self, position_xy, size_xyz, yaw=0.0):
    #     marker = Marker()
    #     marker.header.frame_id = "map"
    #     marker.ns = "boxes"
    #     marker.id = len(self.markers)
    #     marker.type = Marker.CUBE
    #     marker.action = Marker.ADD

    #     marker.pose.position.x = position_xy[0]
    #     marker.pose.position.y = position_xy[1]
    #     marker.pose.position.z = size_xyz[2] / 2.0
    #     # simple yaw -> quaternion (z only)
    #     marker.pose.orientation.z = math.sin(yaw/2.0)
    #     marker.pose.orientation.w = math.cos(yaw/2.0)

    #     marker.scale.x = size_xyz[0]
    #     marker.scale.y = size_xyz[1]
    #     marker.scale.z = size_xyz[2]

    #     marker.color.r = 1.0
    #     marker.color.g = 0.5
    #     marker.color.b = 0.0
    #     marker.color.a = 0.9

    #     marker.lifetime = rospy.Duration(0)

    #     self.markers.append(marker)

    # def add_polygon(self, vertices_xy, height=2.0):
    #     # visualize polygon as LINE_STRIP + filled semi-transparent boxes at centroid
    #     marker = Marker()
    #     marker.header.frame_id = "map"
    #     marker.ns = "polygons"
    #     marker.id = len(self.markers)
    #     marker.type = Marker.LINE_STRIP
    #     marker.action = Marker.ADD
    #     marker.scale.x = 0.05
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 1.0
    #     marker.color.a = 1.0

    #     marker.pose.orientation.w = 1.0
    #     for (x, y) in vertices_xy:
    #         p = rospy.msg.Point()
    #         p.x = x
    #         p.y = y
    #         p.z = height
    #         marker.points.append(p)
    #     # close loop
    #     if vertices_xy:
    #         p0 = rospy.msg.Point()
    #         p0.x, p0.y, p0.z = vertices_xy[0][0], vertices_xy[0][1], height
    #         marker.points.append(p0)

    #     marker.lifetime = rospy.Duration(0)
    #     self.markers.append(marker)

    # def add_moving(self, position_xy, radius, velocity_xy):
    #     marker = Marker()
    #     marker.header.frame_id = "map"
    #     marker.ns = "moving"
    #     marker.id = len(self.markers)
    #     marker.type = Marker.SPHERE
    #     marker.action = Marker.ADD

    #     marker.pose.position.x = position_xy[0]
    #     marker.pose.position.y = position_xy[1]
    #     marker.pose.position.z = 1.0
    #     marker.pose.orientation.w = 1.0

    #     marker.scale.x = 2 * radius
    #     marker.scale.y = 2 * radius
    #     marker.scale.z = 2 * radius

    #     marker.color.r = 0.0
    #     marker.color.g = 0.0
    #     marker.color.b = 1.0
    #     marker.color.a = 1.0

    #     marker.lifetime = rospy.Duration(0)

    #     self.markers.append(marker)
    #     self.moving[marker.id] = (velocity_xy[0], velocity_xy[1])

    # def publish_once(self):
    #     now = rospy.Time.now()
    #     dt = (now - self.last_update).to_sec()
    #     if dt > 0.0:
    #         # update moving markers
    #         for mid, vel in self.moving.items():
    #             if 0 <= mid < len(self.markers):
    #                 self.markers[mid].pose.position.x += vel[0] * dt
    #                 self.markers[mid].pose.position.y += vel[1] * dt
    #     self.last_update = now

    #     marker_array = MarkerArray()
    #     if self.markers:
    #         for marker in self.markers:
    #             marker.header.stamp = rospy.Time.now()
    #             marker_array.markers.append(marker)
    #     self.marker_pub.publish(marker_array)
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Visualizer:
    def __init__(self):
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.markers = []

    def add_cylinder(self, position_xy, radius, height=5.0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "cylinders"
        marker.id = len(self.markers)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = position_xy[0]
        marker.pose.position.y = position_xy[1]
        marker.pose.position.z = height / 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2 * radius
        marker.scale.y = 2 * radius
        marker.scale.z = height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)

        self.markers.append(marker)

    def add_box(self, center_xy, size_enu, top_z=None):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "platforms"
        marker.id = len(self.markers)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        height = float(size_enu[2])
        marker.pose.position.x = center_xy[0]
        marker.pose.position.y = center_xy[1]
        marker.pose.position.z = float(top_z) - height / 2.0 if top_z is not None else height / 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = float(size_enu[0])
        marker.scale.y = float(size_enu[1])
        marker.scale.z = height

        marker.color.r = 0.45
        marker.color.g = 0.45
        marker.color.b = 0.45
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)

        self.markers.append(marker)

    def set_cylinders(self, obstacles, default_height=5.0, platform=None):
        self.markers = []
        for obstacle in obstacles:
            height = float(obstacle.get('height', default_height))
            self.add_cylinder(obstacle['centerENU'], obstacle['radius'], height=height)
        if platform:
            size = platform.get('sizeENU')
            center = platform.get('centerENU')
            if size and center and len(size) >= 3 and len(center) >= 2:
                self.add_box(center, size, top_z=platform.get('topZ'))

    def publish_once(self):
        marker_array = MarkerArray()

        if self.markers:
            for marker in self.markers:
                marker.header.stamp = rospy.Time.now()
                marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
