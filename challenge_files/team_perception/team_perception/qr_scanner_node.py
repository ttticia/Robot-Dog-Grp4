"""
QR scanner node (robot build: no debug video).

Subscribes to a colour (or left rectified) image, decodes QR codes in a background
thread, optionally uses an aligned depth image to place each code in the map frame,
and publishes/logs every NEW code once (with a saved snapshot as evidence).

Topics (all names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : image_topic        sensor_msgs/Image, or <image_topic>/compressed
                          sensor_msgs/CompressedImage if image_transport:=compressed
  in : depth_topic        sensor_msgs/Image   (aligned to image_topic; optional)
  in : camera_info_topic  sensor_msgs/CameraInfo
  out: /qr/detections     std_msgs/String     (JSON, one message per new code)
  out: /found             std_msgs/String     "QR found", once per new code text
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

from team_perception import qr_utils
from team_perception.qr_utils import decode_qr, depth_at, deproject, draw_detections


class QrScanner(Node):
    def __init__(self):
        super().__init__('qr_scanner')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('image_transport', 'raw')    # 'raw', or 'compressed' = <image_topic>/compressed
        p('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        p('camera_info_topic', '/camera/color/camera_info')
        p('use_depth', True)
        p('map_frame', 'map')
        p('rate_hz', 30.0)             # max decodes per second (0 = as fast as possible)
        p('full_scan_every', 5)        # every Nth decode also tries contrast boost + 2x
        #                                upscale (finds small/far codes, ~5x slower); 0 = never
        p('repeat_after_s', 30.0)      # re-report the same code after this long
        p('log_dir', os.path.expanduser('~/qr_logs'))

        def g(name):
            return self.get_parameter(name).value

        self.use_depth = g('use_depth')
        self.map_frame = g('map_frame')
        self.repeat_after = g('repeat_after_s')
        self.full_every = int(g('full_scan_every'))
        rate = g('rate_hz')
        self.min_period = 1.0 / rate if rate > 0 else 0.0

        self.bridge = CvBridge()
        self.latest_img = None
        self.latest_depth = None
        self.K = None                  # (fx, fy, cx, cy)
        self.seen = {}                 # text -> last report time (s)
        self.new_frame = threading.Event()

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

        self.pub = self.create_publisher(String, '/qr/detections', 10)
        self.found_pub = self.create_publisher(String, '/found', 10)   # "QR found", once per code

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.log_dir = g('log_dir')
        # Record of found codes (snapshot + CSV row per new code); the folder and file are
        # only created when the first code is found, so startup touches no files.
        self.csv_path = os.path.join(self.log_dir, time.strftime('qr_%Y%m%d_%H%M%S.csv'))

        self.running = True
        self.worker = threading.Thread(target=self.decode_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f'QR scanner on {image_topic} using {qr_utils.BACKENDS}, '
                               f'logging to {self.csv_path}')

    # --- callbacks ---------------------------------------------------------------
    def on_image(self, msg):
        try:
            img = self.to_bgr(msg)     # converted once here, shared with the decode thread
        except Exception as e:  # e.g. unexpected encoding or corrupt JPEG
            self.get_logger().warn(f'image conversion failed: {e}', throttle_duration_sec=5.0)
            return
        self.latest_img = (img, msg.header)
        self.new_frame.set()           # wake the decode thread

    def on_depth(self, msg):
        self.latest_depth = msg

    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def to_bgr(self, msg):
        """Image/CompressedImage -> BGR array (bgr8 Image: zero-copy view, no cv_bridge copy)."""
        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError(f'could not decode {msg.format!r} image')
            return img
        if msg.encoding == 'bgr8' and msg.step == msg.width * 3:
            return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # --- decoding (background thread) ---------------------------------------------
    def decode_loop(self):
        n = 0
        while self.running and rclpy.ok():
            if not self.new_frame.wait(timeout=0.5):
                continue
            self.new_frame.clear()
            frame = self.latest_img
            if frame is None:
                continue
            img, header = frame
            start = time.time()

            full = self.full_every > 0 and n % self.full_every == 0
            n += 1
            try:
                codes = decode_qr(img, try_upscale=full, try_clahe=full)
            except Exception as e:
                self.get_logger().error(f'decode failed: {e}', throttle_duration_sec=5.0)
                continue

            now = time.time()
            for text, corners in codes:
                self.on_code(text, corners, img, header, now)

            if self.min_period:
                time.sleep(max(0.0, self.min_period - (time.time() - start)))

    def on_code(self, text, corners, img, header, now):
        if now - self.seen.get(text, -1e9) < self.repeat_after:
            return
        if text not in self.seen:      # first time this code is seen: announce it once
            self.get_logger().info('QR found')
            self.found_pub.publish(String(data='QR found'))
        self.seen[text] = now
        try:
            self.report(text, corners, img, header)
        except Exception as e:
            self.get_logger().error(f'report failed for "{text}": {e}')

    def report(self, text, corners, img, header):
        u, v = corners.mean(axis=0)
        frame, xyz = 'none', None

        point = self.locate(u, v, header, img.shape)
        if point is not None:
            frame, xyz = point.header.frame_id, (point.point.x, point.point.y, point.point.z)
        where = f'{frame} ({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})' if xyz else 'no position'

        os.makedirs(self.log_dir, exist_ok=True)
        snap = os.path.join(self.log_dir, f'qr_{int(time.time() * 1000)}.jpg')
        cv2.imwrite(snap, draw_detections(img.copy(), [(text, corners)], {text: [where]}))

        result = {'text': text, 'pixel': [float(u), float(v)], 'frame': frame,
                  'position': list(xyz) if xyz else None, 'snapshot': snap,
                  'stamp': Time.from_msg(header.stamp).nanoseconds * 1e-9}
        self.pub.publish(String(data=json.dumps(result)))
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            if new_file:
                csv.writer(f).writerow(['wall_time', 'text', 'frame', 'x', 'y', 'z', 'snapshot'])
            csv.writer(f).writerow([time.time(), text, frame, *(xyz or ('', '', '')), snap])
        self.get_logger().info(f'QR "{text}" at {where}')

    def locate(self, u, v, header, shape):
        """Return a PointStamped in map frame (or camera frame if tf fails), else None."""
        if not self.use_depth or self.latest_depth is None or self.K is None:
            return None
        depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
        # If depth and colour resolutions differ, scale the pixel (assumes aligned FOV).
        sx = depth.shape[1] / shape[1]
        sy = depth.shape[0] / shape[0]
        z = depth_at(depth, u * sx, v * sy)
        if z is None:
            return None
        x, y, z = deproject(u, v, z, *self.K)

        pt = PointStamped()
        pt.header = header                   # camera optical frame + image time
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        try:
            return self.tf_buffer.transform(pt, self.map_frame, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'No tf to {self.map_frame} ({e}); reporting camera frame',
                                   throttle_duration_sec=5.0)
            return pt

    def destroy_node(self):
        self.running = False
        self.new_frame.set()
        self.worker.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = QrScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
