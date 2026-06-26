# ...existing code...
#!/usr/bin/env python3
import rospy
import os
import json
import sys
import signal
from Visualizer import Visualizer

class SingleRun:
    """
    负责加载 scene JSON，初始化可视化，并以固定频率发布障碍物 Marker。
    这个文件保持轻量，不直接控制无人机；planner/控制器按需读取同一 scene 文件。
    """
    def __init__(self, scene_file=None, publish_rate=5.0):
        self.node_name = rospy.get_name() or 'single_run'
        # scene_file 优先级：传参 -> 私有参数 ~scene_file -> 默认 two_obstacles.json
        if scene_file is None:
            try:
                scene_file = rospy.get_param('~scene_file', 'two_obstacles.json')
            except Exception:
                scene_file = 'two_obstacles.json'
        self.scene_file = self._find_scene_file(scene_file)
        rospy.loginfo("[%s] Using scene file: %s", self.node_name, self.scene_file)

        self.visualizer = Visualizer()
        self.number = 0
        self.leaders = 0
        self.head = 0
        self.velocityVectorENU = [0.0, 0.0, 0.0]
        self.formationTime = 0.0
        self.takeoffPointENU = []
        self.preparePointENU = []
        # load and publish once
        self._load_scene(self.scene_file)
        self.rate = rospy.Rate(publish_rate)
        self._running = True

    def _find_scene_file(self, scene):
        # 支持绝对路径或相对 scenes/ 目录（包内）
        if os.path.isabs(scene) and os.path.isfile(scene):
            return scene
        # 若为文件名或相对名，尝试包内 scenes 目录
        base = os.path.dirname(__file__)
        scenes_dir = os.path.normpath(os.path.join(base, '..', 'scenes'))
        candidate = scene
        if not candidate.endswith('.json'):
            candidate = candidate + '.json'
        candidate_path = os.path.normpath(os.path.join(scenes_dir, candidate))
        if os.path.isfile(candidate_path):
            return candidate_path
        # 退回：如果传入的是相对路径相对于 cwd
        cwd_path = os.path.normpath(os.path.join(os.getcwd(), scene))
        if os.path.isfile(cwd_path):
            return cwd_path
        raise FileNotFoundError("Scene file not found: %s (checked %s and %s)" %
                                (scene, candidate_path, cwd_path))

    def _load_scene(self, scene_file):
        with open(scene_file, 'r') as f:
            js = json.load(f)
        # 基本字段
        self.number = int(js.get('number', 0))
        self.leaders = int(js.get('leaders', 0))
        self.head = int(js.get('head', 0))
        self.velocityVectorENU = js.get('velocityVectorENU', [0.0, 0.0, 0.0])
        self.formationTime = float(js.get('formationTime', 0.0))
        self.takeoffPointENU = js.get('takeoffPointENU', [])
        self.preparePointENU = js.get('preparePointENU', [])

        # 兼容旧格式 obstacleData 和新格式 obstacles
        if 'obstacles' in js:
            for o in js.get('obstacles', []):
                self._add_obstacle_from_obj(o)
        else:
            # 旧格式：obstacleData 是 dict
            for k, v in js.get('obstacleData', {}).items():
                c = v.get('centerENU', [0.0, 0.0])
                r = v.get('radius', 0.5)
                try:
                    self.visualizer.add_cylinder([c[0], c[1]], r)
                except Exception as e:
                    rospy.logwarn("Visualizer add_cylinder failed: %s", e)

        # publish initial markers
        try:
            self.visualizer.publish_once()
            rospy.loginfo("[%s] Scene loaded and visualized", self.node_name)
        except Exception as e:
            rospy.logwarn("[%s] Failed to publish initial visualization: %s", self.node_name, e)

    def _add_obstacle_from_obj(self, o):
        t = o.get('type', 'cylinder')
        if t == 'cylinder':
            c = o.get('centerENU', [0.0, 0.0, 0.0])
            r = o.get('radius', 0.5)
            self.visualizer.add_cylinder([c[0], c[1]], r)
        elif t == 'box':
            c = o.get('centerENU', [0.0, 0.0, 0.0])
            size = o.get('size', [1.0, 1.0, 1.0])
            yaw = o.get('yaw', 0.0)
            self.visualizer.add_box([c[0], c[1]], size, yaw)
        elif t == 'polygon':
            verts = o.get('vertices', [])
            height = o.get('height', 2.0)
            self.visualizer.add_polygon(verts, height=height)
        elif t == 'moving':
            c = o.get('centerENU', [0.0, 0.0, 0.0])
            r = o.get('radius', 0.5)
            vel = o.get('velocity', [0.0, 0.0, 0.0])
            self.visualizer.add_moving([c[0], c[1]], r, [vel[0], vel[1]])
        else:
            rospy.logwarn("Unknown obstacle type '%s' in scene, skipping.", t)

    def stop(self):
        self._running = False

    def run(self):
        rospy.loginfo("[%s] Running main loop (publishing markers at %.1f Hz)", self.node_name, self.rate.sleep_dur.to_sec() if hasattr(self.rate, 'sleep_dur') else  self.rate.sleep)
        while not rospy.is_shutdown() and self._running:
            try:
                self.visualizer.publish_once()
            except Exception as e:
                rospy.logwarn("[%s] Exception during publish: %s", self.node_name, e)
            self.rate.sleep()

def _signal_handler(sig, frame):
    rospy.loginfo("SingleRun received signal %s", sig)
    rospy.signal_shutdown("signal %s" % sig)

if __name__ == '__main__':
    # initialize node
    rospy.init_node('single_run', anonymous=False)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # read optional private param ~scene_file
    try:
        scene_param = rospy.get_param('~scene_file', None)
    except Exception:
        scene_param = None

    try:
        runner = SingleRun(scene_file=scene_param, publish_rate=float(rospy.get_param('~publish_rate', 5.0)))
    except Exception as e:
        rospy.logerr("Failed to initialize SingleRun: %s", e)
        sys.exit(1)

    try:
        runner.run()
    except rospy.ROSInterruptException:
        pass
# ...existing code...