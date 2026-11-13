"""
Human detector node (Ultralytics YOLO, 'person' class).

Runs YOLO on a background thread on the newest camera frame, so the debug video is
republished on EVERY camera frame with the latest boxes, however slow inference is.
Optionally uses an aligned depth image to place each person in the map frame.

Topics (all names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : image_topic        sensor_msgs/Image, or <image_topic>/compressed
                          sensor_msgs/CompressedImage if image_transport:=compressed
  in : depth_topic        sensor_msgs/Image   (aligned to image_topic; optional)
  in : camera_info_topic  sensor_msgs/CameraInfo
  out: /humans/detections           std_msgs/String  JSON per inference with >=1 person
  out: /humans/debug_image          sensor_msgs/Image (boxes; only if publish_debug)
  out: /humans/debug_image/compressed  sensor_msgs/CompressedImage (same, as JPEG)
"""
import csv
import json
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)

from team_perception.human_utils import HumanDetector, box_depth_pixel, draw_humans
from team_perception.qr_utils import DepthAlignState, deproject


class HumanDetectorNode(Node):
    def __init__(self):
        super().__init__('human_detector')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('image_transport', 'raw')    # 'raw', or 'compressed' = <image_topic>/compressed
        p('depth_topic', '/camera/depth/image_raw')
        p('depth_camera_info_topic', '/camera/depth/camera_info')  # '' = depth is
        #                                already aligned to colour
        p('camera_info_topic', '/camera/color/camera_info')
        p('use_depth', True)
        p('map_frame', 'map')
        p('model', 'yolo11n.pt')       # bare name -> ~/.cache/team_perception/, or a path
        p('conf', 0.4)                 # minimum confidence
        p('imgsz', 640)                # YOLO input size (smaller = faster, misses far people)
        p('device', 'cpu')             # 'cpu', or '0' for the first CUDA GPU
        p('rate_hz', 10.0)             # max inferences per second (0 = as fast as possible)
        p('hold_s', 0.5)               # keep drawing boxes this long after last inference
        p('publish_debug', True)
        p('log_dir', os.path.expanduser('~/human_logs'))
        p('log_every_s', 0.0)          # debugging: save an annotated snapshot + CSV row at
        #                                most this often while people are seen (0 = off)

        def g(name):
            return self.get_parameter(name).value

        self.use_depth = g('use_depth')
        self.map_frame = g('map_frame')
        self.publish_debug = g('publish_debug')
        self.hold_s = g('hold_s')
        self.log_every = g('log_every_s')
        rate = g('rate_hz')
        self.min_period = 1.0 / rate if rate > 0 else 0.0

        self.get_logger().info(f'Loading YOLO model {g("model")} on {g("device")} ...')
        self.detector = HumanDetector(g('model'), g('conf'), g('imgsz'), g('device'))
        self.detector.detect(np.zeros((g('imgsz'), g('imgsz'), 3), np.uint8))   # warm-up

        self.bridge = CvBridge()
        self.latest_img = None
        self.latest_depth = None
        self.K = None                  # (fx, fy, cx, cy)
        self.align_state = DepthAlignState(bool(g('depth_camera_info_topic')))
        self.new_frame = threading.Event()
        self.detections = []           # [(box, conf)] from the latest inference
        self.labels = []
        self.det_time = 0.0
        self.infer_ms = 0.0
        self.infer_hz = 0.0
        self.video_hz = 0.0
        self._last_video = None
        self._last_log = 0.0

        if g('image_transport') == 'compressed':
            image_topic = g('image_topic').rstrip('/') + '/compressed'
            self.create_subscription(CompressedImage, image_topic, self.on_image,
                                     qos_profile_sensor_data)
        else:
            image_topic = g('image_topic')
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(Image, g('depth_topic'), self.on_depth,
                                     qos_profile_sensor_data)
            if g('depth_camera_info_topic'):
                self.create_subscription(CameraInfo, g('depth_camera_info_topic'),
                                         self.on_depth_info, qos_profile_sensor_data)

        self.pub = self.create_publisher(String, '/humans/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/humans/debug_image', 1)
        self.debug_jpeg_pub = self.create_publisher(
            CompressedImage, '/humans/debug_image/compressed', 1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.log_dir = g('log_dir')   # used only when log_every_s > 0
        self.csv_path = os.path.join(self.log_dir, time.strftime('humans_%Y%m%d_%H%M%S.csv'))

        self.running = True
        self.worker = threading.Thread(target=self.infer_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f'Human detector on {image_topic}'
                               + (f', snapshots every {self.log_every:g}s to {self.log_dir}'
                                  if self.log_every > 0 else ''))

    # --- callbacks ---------------------------------------------------------------
    def on_image(self, msg):
        try:
            img = self.to_bgr(msg)
        except Exception as e:  # e.g. unexpected encoding or corrupt JPEG
            self.get_logger().warn(f'image conversion failed: {e}', throttle_duration_sec=5.0)
            return
        self.latest_img = (img, msg.header)
        self.new_frame.set()
        now = time.time()
        if self._last_video is not None:
            self.video_hz = 0.9 * self.video_hz + 0.1 / max(now - self._last_video, 1e-6)
        self._last_video = now
        if self.publish_debug and (self.debug_pub.get_subscription_count() > 0 or
                                   self.debug_jpeg_pub.get_subscription_count() > 0):
            self.publish_debug_image(img, msg.header)

    def on_depth(self, msg):
        self.latest_depth = msg

    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.align_state.set_colour(self.K, (msg.width, msg.height), msg.header.frame_id)

    def on_depth_info(self, msg):
        self.align_state.set_depth((msg.k[0], msg.k[4], msg.k[2], msg.k[5]),
                                   msg.header.frame_id)

    def to_bgr(self, msg):
        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError(f'could not decode {msg.format!r} image')
            return img
        if msg.encoding == 'bgr8' and msg.step == msg.width * 3:
            return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def publish_debug_image(self, img, header):
        img = img.copy()
        recent = time.time() - self.det_time < self.hold_s
        dets = self.detections if recent else []
        status = [f'video {self.video_hz:4.1f} fps | yolo {self.infer_hz:4.1f}/s '
                  f'{self.infer_ms:3.0f} ms | people: {len(dets)}']
        draw_humans(img, dets, self.labels if recent else None, status)
        if self.debug_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out.header = header
            self.debug_pub.publish(out)
        if self.debug_jpeg_pub.get_subscription_count() > 0:
            jpg = CompressedImage(header=header, format='jpeg')
            jpg.data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
            self.debug_jpeg_pub.publish(jpg)

    # --- inference (background thread) -------------------------------------------
    def infer_loop(self):
        last = time.time()
        while self.running and rclpy.ok():
            if not self.new_frame.wait(timeout=0.5):
                continue
            self.new_frame.clear()
            frame = self.latest_img
            if frame is None:
                continue
            img, header = frame
            start = time.time()
            try:
                dets = self.detector.detect(img)
            except Exception as e:
                self.get_logger().error(f'inference failed: {e}', throttle_duration_sec=5.0)
                continue
            end = time.time()
            self.infer_ms = 0.8 * self.infer_ms + 0.2 * (end - start) * 1000
            self.infer_hz = 0.8 * self.infer_hz + 0.2 / max(end - last, 1e-6)
            last = end

            people = [self.describe(box, conf, header, img.shape) for box, conf in dets]
            self.labels = [[p['where']] for p in people]
            self.detections, self.det_time = dets, end
            if people:
                self.report(people, dets, img, header, end)

            if self.min_period:
                time.sleep(max(0.0, self.min_period - (time.time() - start)))

    def describe(self, box, conf, header, shape):
        u, v = box_depth_pixel(box)
        point = self.locate(u, v, header, shape)
        frame, xyz = 'none', None
        if point is not None:
            frame, xyz = point.header.frame_id, (point.point.x, point.point.y, point.point.z)
        where = f'{frame} ({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})' if xyz else 'no position'
        return {'box': [float(b) for b in box], 'conf': round(conf, 3),
                'pixel': [float(u), float(v)], 'frame': frame,
                'position': list(xyz) if xyz else None, 'where': where}

    def report(self, people, dets, img, header, now):
        snap = ''
        if self.log_every > 0 and now - self._last_log >= self.log_every:
            self._last_log = now
            os.makedirs(self.log_dir, exist_ok=True)
            snap = os.path.join(self.log_dir, f'humans_{int(now * 1000)}.jpg')
            cv2.imwrite(snap, draw_humans(img.copy(), dets, [[p['where']] for p in people]))
            new_file = not os.path.exists(self.csv_path)
            with open(self.csv_path, 'a', newline='') as f:
                if new_file:
                    csv.writer(f).writerow(['wall_time', 'count', 'confidences', 'positions',
                                            'snapshot'])
                csv.writer(f).writerow([now, len(people),
                                        ' '.join(f'{p["conf"]:.2f}' for p in people),
                                        ' | '.join(p['where'] for p in people), snap])
            self.get_logger().info(f'{len(people)} person(s): '
                                   + '; '.join(f'{p["conf"]:.2f} at {p["where"]}'
                                               for p in people))
        msg = {'stamp': Time.from_msg(header.stamp).nanoseconds * 1e-9,
               'frame_id': header.frame_id,
               'image_size': [int(img.shape[1]), int(img.shape[0])],   # boxes are in these px
               'count': len(people), 'snapshot': snap or None,
               'people': [{k: v for k, v in p.items() if k != 'where'} for p in people]}
        self.pub.publish(String(data=json.dumps(msg)))

    def locate(self, u, v, header, shape):
        """Return a PointStamped in map frame (or camera frame if tf fails), else None."""
        if not self.use_depth or self.latest_depth is None or self.K is None:
            return None
        depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
        align = self.depth_align(depth.shape)     # colour pixel -> (unaligned) depth pixel
        if align is None:
            return None
        found = align.depth_at(depth, u, v, image_size=(shape[1], shape[0]))
        if found is None:
            return None
        z = found[0]
        x, y, z = deproject(u, v, z, *self.K)
        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        try:
            return self.tf_buffer.transform(pt, self.map_frame, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'No tf to {self.map_frame} ({e}); reporting camera frame',
                                   throttle_duration_sec=5.0)
            return pt

    def depth_align(self, depth_shape):
        """Return the DepthAlign for this depth image, or None (looks up the sensor TF once)."""
        need = self.align_state.needs_tf()
        if need is not None:
            try:
                tr = self.tf_buffer.lookup_transform(need[0], need[1], Time()).transform
                self.align_state.t = (tr.translation.x, tr.translation.y, tr.translation.z)
            except Exception as e:
                self.get_logger().warn(f'No TF {need[1]} -> {need[0]} ({e}); mapping colour '
                                       'to depth without the sensor offset',
                                       throttle_duration_sec=10.0)
        align, why = self.align_state.get(depth_shape)
        if align is None:
            self.get_logger().warn(f'depth not used: {why}', throttle_duration_sec=5.0)
        return align

    def destroy_node(self):
        self.running = False
        self.new_frame.set()
        self.worker.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = HumanDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
