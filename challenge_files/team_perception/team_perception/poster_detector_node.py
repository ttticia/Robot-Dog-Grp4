"""
Poster detector node: reports pictures of people, ignores real people (robot build).

Takes person boxes from human_detector and decides, for each one, whether it is a poster
using two tests from poster_utils:
  1. edge-ring test: on a poster the surface continues just outside the box; behind a
     real person the depth jumps backwards.
  2. static-map test: a poster hangs on a wall/board that is in the static map; a real
     person stands in free space.
A poster is reported once it wins `vote_need` of the last `vote_window` frames.

Topics (all names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : /humans/detections   std_msgs/String  JSON from human_detector (boxes + stamp)
  in : depth_topic          sensor_msgs/Image  depth ALIGNED to the colour image
  in : camera_info_topic    sensor_msgs/CameraInfo  (intrinsics of that depth image)
  in : map_topic            nav_msgs/OccupancyGrid  static map (transient local)
  out: /posters/detections        std_msgs/String  JSON, one per newly confirmed poster
  out: /found                     std_msgs/String  "Human found", once per new poster position
"""
import bisect
import csv
from collections import deque
import json
import os
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)

from team_perception.poster_utils import (MAP_MODES, MapParams, PosterTracker, RingParams,
                                          StaticMap, combine, edge_ring_test, scale_box,
                                          static_map_test, to_metres, torso_pixel_and_depth)


def _stamp(msg_header):
    return Time.from_msg(msg_header.stamp).nanoseconds * 1e-9


class StampBuffer:
    """Keep the last few seconds of messages and find the one closest to a stamp."""

    def __init__(self, keep_s=2.0):
        self.keep = keep_s
        self.stamps, self.msgs = deque(), deque()

    def add(self, stamp, msg):
        self.stamps.append(stamp)
        self.msgs.append(msg)
        while self.stamps and stamp - self.stamps[0] > self.keep:
            self.stamps.popleft()
            self.msgs.popleft()

    def closest(self, stamp, max_dt):
        if not self.stamps:
            return None, None
        s = list(self.stamps)
        i = bisect.bisect_left(s, stamp)
        best = min((j for j in (i - 1, i) if 0 <= j < len(s)), key=lambda j: abs(s[j] - stamp))
        dt = abs(s[best] - stamp)
        return (self.msgs[best], dt) if dt <= max_dt else (None, dt)


class PosterDetector(Node):
    def __init__(self):
        super().__init__('poster_detector')
        p = self.declare_parameter
        p('detections_topic', '/humans/detections')
        p('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        p('camera_info_topic', '/camera/color/camera_info')
        p('map_topic', '/map')
        p('map_frame', 'map')
        p('use_map', True)
        p('map_mode', 'veto')          # 'require' | 'veto' | 'off' (see poster_utils.combine)
        p('max_dt_s', 0.1)             # max stamp difference between detection and depth
        # edge-ring test
        p('ring_tol_m', 0.05)
        p('ring_tol_quad', 0.01)
        p('ring_step_m', 0.25)
        p('ring_frac', 0.10)
        p('max_range_m', 4.0)
        p('min_sides', 2)
        # static-map test
        p('map_on_radius_m', 0.30)
        p('map_free_radius_m', 0.60)
        p('map_occupied', 65)
        # voting / reporting
        p('merge_radius_m', 0.5)
        p('vote_window', 8)
        p('vote_need', 5)
        p('repeat_after_s', 60.0)
        p('log_dir', os.path.expanduser('~/poster_logs'))

        def g(name):
            return self.get_parameter(name).value
        self.g = g

        self.map_frame = g('map_frame')
        self.use_map = g('use_map')
        self.map_mode = g('map_mode') if self.use_map else 'off'
        if self.map_mode not in MAP_MODES:
            raise ValueError(f'map_mode must be one of {MAP_MODES}')
        self.max_dt = g('max_dt_s')
        self.ring_p = RingParams(tol_m=g('ring_tol_m'), tol_quad=g('ring_tol_quad'),
                                 step_m=g('ring_step_m'), ring_frac=g('ring_frac'),
                                 max_range_m=g('max_range_m'), min_sides=g('min_sides'))
        self.map_p = MapParams(on_radius_m=g('map_on_radius_m'),
                               free_radius_m=g('map_free_radius_m'),
                               occupied=g('map_occupied'))
        self.tracker = PosterTracker(g('merge_radius_m'), g('vote_window'), g('vote_need'),
                                     repeat_after_s=g('repeat_after_s'))

        self.bridge = CvBridge()
        self.depth_buf = StampBuffer()
        self.K = None
        self.static_map = None

        self.create_subscription(String, g('detections_topic'), self.on_detections, 10)
        self.create_subscription(Image, g('depth_topic'),
                                 lambda m: self.depth_buf.add(_stamp(m.header), m),
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        if self.use_map:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, latched)

        self.pub = self.create_publisher(String, '/posters/detections', 10)
        self.found_pub = self.create_publisher(String, '/found', 10)   # "Human found", once each
        self.announced = []            # positions of posters already announced

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.log_dir = g('log_dir')
        # record of confirmed posters; created on the first one, not at startup
        self.csv_path = os.path.join(self.log_dir, time.strftime('posters_%Y%m%d_%H%M%S.csv'))
        self.get_logger().info(
            f'Poster detector: {g("detections_topic")} + {g("depth_topic")}, map_mode='
            f'{self.map_mode}{" (" + g("map_topic") + ")" if self.use_map else ""}')

    # --- inputs ------------------------------------------------------------------
    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)

    def on_map(self, msg):
        q = msg.info.origin.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.static_map = StaticMap(msg.data, msg.info.width, msg.info.height,
                                    msg.info.resolution,
                                    (msg.info.origin.position.x, msg.info.origin.position.y),
                                    float(yaw))
        self.get_logger().info(f'Static map received: {msg.info.width}x{msg.info.height} '
                               f'@ {msg.info.resolution:.3f} m')

    # --- main step ---------------------------------------------------------------
    def on_detections(self, msg):
        try:
            det = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn('bad JSON on detections topic', throttle_duration_sec=5.0)
            return
        stamp = det.get('stamp', 0.0)
        depth_msg, dt = self.depth_buf.closest(stamp, self.max_dt)
        if depth_msg is None:
            self.get_logger().warn(
                'no depth image within max_dt_s of the detection'
                + (f' (closest {dt * 1000:.0f} ms)' if dt is not None else ' (no depth yet)'),
                throttle_duration_sec=5.0)
            return
        depth = to_metres(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'))

        for person in det.get('people', []):
            r = self.classify(person, det, depth, depth_msg.header)
            if r['report']:
                self.report(r, stamp)

    def classify(self, person, det, depth, depth_header):
        box = scale_box(person['box'], det.get('image_size'), depth.shape)
        ring, ring_m = edge_ring_test(depth, box, self.ring_p)
        out = {'box': person['box'], 'conf': person.get('conf'), 'ring': ring,
               'ring_metrics': ring_m, 'map': 'unknown', 'map_metrics': {},
               'position': None, 'frame': None, 'report': False}

        point = self.locate(depth, box, depth_header)
        if point is not None:
            out['frame'] = point.header.frame_id
            out['position'] = [round(point.point.x, 3), round(point.point.y, 3),
                               round(point.point.z, 3)]
            if self.map_mode != 'off' and point.header.frame_id == self.map_frame:
                out['map'], out['map_metrics'] = static_map_test(
                    self.static_map, point.point.x, point.point.y, self.map_p)
            elif self.map_mode != 'off':
                out['map_metrics'] = {'reason': f'no tf to {self.map_frame}'}

        out['label'] = combine(ring, out['map'], self.map_mode)
        if out['position'] is not None:
            track, out['report'] = self.tracker.update(out['position'], out['label'],
                                                       time.time())
            out['track'] = track.id
            out['votes'] = self.tracker.votes(track)
            out['confirmed_poster'] = self.tracker.is_poster(track)
        return out

    def locate(self, depth, box, header):
        """Torso point as PointStamped in the map frame (camera frame if no tf), or None."""
        if self.K is None:
            self.get_logger().warn('no camera_info yet', throttle_duration_sec=5.0)
            return None
        tp = torso_pixel_and_depth(depth, box, self.ring_p)
        if tp is None:
            return None
        u, v, z = tp
        fx, fy, cx, cy, w, h = self.K
        if w and h and (w != depth.shape[1] or h != depth.shape[0]):   # intrinsics for
            sx, sy = depth.shape[1] / w, depth.shape[0] / h            # another resolution
            fx, cx, fy, cy = fx * sx, cx * sx, fy * sy, cy * sy
        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y, pt.point.z = (u - cx) * z / fx, (v - cy) * z / fy, z
        try:
            return self.tf_buffer.transform(pt, self.map_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'No tf to {self.map_frame} ({e}); tracking in camera '
                                   'frame, map test skipped', throttle_duration_sec=5.0)
            return pt

    # --- outputs -----------------------------------------------------------------
    def report(self, r, stamp):
        # Announce each poster once. Checked by position, not track id, because a track
        # that is out of view for forget_s is dropped and comes back with a new id.
        merge = self.tracker.merge
        if not any(np.linalg.norm(np.subtract(r['position'], p)) <= merge
                   for p in self.announced):
            self.announced.append(r['position'])
            self.get_logger().info('Human found')
            self.found_pub.publish(String(data='Human found'))
        os.makedirs(self.log_dir, exist_ok=True)
        msg = {'stamp': stamp, 'id': r['track'], 'frame': r['frame'], 'position': r['position'],
               'votes': r['votes'], 'conf': r['conf'], 'box': r['box'],
               'ring': r['ring_metrics'].get('reason'), 'map': r['map_metrics'].get('reason')}
        self.pub.publish(String(data=json.dumps(msg)))
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            if new_file:
                csv.writer(f).writerow(['wall_time', 'poster_id', 'frame', 'x', 'y', 'z',
                                        'votes', 'conf', 'ring', 'map'])
            csv.writer(f).writerow([time.time(), r['track'], r['frame'], *r['position'],
                                    r['votes'], r['conf'], msg['ring'], msg['map']])
        self.get_logger().info(f'POSTER #{r["track"]} at {r["frame"]} {r["position"]} '
                               f'(votes {r["votes"]})')


def main():
    rclpy.init()
    node = PosterDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
