"""
QR scanner node.

Subscribes to a colour (or left rectified) image, decodes QR codes in a background
thread, optionally uses an aligned depth image to place each code in the map frame,
and publishes/logs every NEW code once (with a saved snapshot as evidence).

Decoding runs on its own thread so the debug video is republished on EVERY camera
frame (with the most recent boxes and labels drawn), however slow decoding is.

Topics (all names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : image_topic        sensor_msgs/Image, or <image_topic>/compressed
                          sensor_msgs/CompressedImage if image_transport:=compressed
  in : depth_topic        sensor_msgs/Image   (aligned to image_topic; optional)
  in : camera_info_topic  sensor_msgs/CameraInfo
  out: /qr/detections     std_msgs/String     (JSON, one message per new code)
  out: /qr/debug_image    sensor_msgs/Image   (boxes + text; only if publish_debug)
  out: /qr/debug_image/compressed  sensor_msgs/CompressedImage (same, as JPEG)

Use image_transport:=compressed whenever you can: raw 720p frames are 2.7 MB and DDS
drops many of them (and they cannot cross Wi-Fi at 30 fps); JPEG is ~30x smaller.
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
        p('hold_s', 0.5)               # keep drawing a box this long after last decode
        p('repeat_after_s', 30.0)      # re-report the same code after this long
        p('publish_debug', True)
        p('log_dir', os.path.expanduser('~/qr_logs'))

        def g(name):
            return self.get_parameter(name).value

        self.use_depth = g('use_depth')
        self.map_frame = g('map_frame')
        self.repeat_after = g('repeat_after_s')
        self.publish_debug = g('publish_debug')
        self.full_every = int(g('full_scan_every'))
        self.hold_s = g('hold_s')
        rate = g('rate_hz')
        self.min_period = 1.0 / rate if rate > 0 else 0.0

        self.bridge = CvBridge()
        self.latest_img = None
        self.latest_depth = None
        self.K = None                  # (fx, fy, cx, cy)
        self.seen = {}                 # text -> last report time (s)
        self.info = {}                 # text -> dict(first, last, count, where) for labels

        # Shared between the decode thread and the image callback.
        self.new_frame = threading.Event()
        self.detections = []           # [(text, corners)] from the latest decoded frame
        self.det_time = 0.0
        self.decode_ms = 0.0
        self.decode_hz = 0.0
        self.video_hz = 0.0
        self._last_video = None

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
        self.debug_pub = self.create_publisher(Image, '/qr/debug_image', 1)
        # JPEG version: ~30x smaller, so much less lag in rqt. rqt shows it as transport
        # "compressed" once ros-humble-image-transport-plugins is installed.
        self.debug_jpeg_pub = self.create_publisher(
            CompressedImage, '/qr/debug_image/compressed', 1)

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

    # --- debug video: every frame, with the latest detections overlaid -------------
    def publish_debug_image(self, img, header):
        img = img.copy()  # never draw on the frame the decode thread is reading
        dets = self.detections if time.time() - self.det_time < self.hold_s else []
        status = [f'video {self.video_hz:4.1f} fps | decode {self.decode_hz:4.1f}/s '
                  f'{self.decode_ms:3.0f} ms | codes found: {len(self.info)}']
        # list() snapshots the dict in one step; the decode thread may add codes meanwhile.
        recent = sorted(list(self.info.items()), key=lambda kv: -kv[1]['last'])[:5]
        status += [f'  {t[:40]}  ({time.time() - i["last"]:.0f}s ago)' for t, i in recent]
        draw_detections(img, dets, self.labels(dets), status)
        if self.debug_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out.header = header
            self.debug_pub.publish(out)
        if self.debug_jpeg_pub.get_subscription_count() > 0:
            jpg = CompressedImage(header=header, format='jpeg')
            jpg.data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
            self.debug_jpeg_pub.publish(jpg)

    def labels(self, dets):
        out = {}
        now = time.time()
        for text, _ in dets:
            i = self.info.get(text)
            if i:
                out[text] = [i['where'], f'seen {i["count"]}x, first {now - i["first"]:.0f}s ago']
        return out

    # --- decoding (background thread) ---------------------------------------------
    def decode_loop(self):
        n = 0
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

            full = self.full_every > 0 and n % self.full_every == 0
            n += 1
            try:
                codes = decode_qr(img, try_upscale=full, try_clahe=full)
            except Exception as e:
                self.get_logger().error(f'decode failed: {e}', throttle_duration_sec=5.0)
                continue

            end = time.time()
            self.decode_ms = 0.8 * self.decode_ms + 0.2 * (end - start) * 1000
            self.decode_hz = 0.8 * self.decode_hz + 0.2 / max(end - last, 1e-6)
            last = end
            if codes:
                self.detections, self.det_time = codes, end

            for text, corners in codes:
                self.on_code(text, corners, img, header, end)

            if self.min_period:
                time.sleep(max(0.0, self.min_period - (time.time() - start)))

    def on_code(self, text, corners, img, header, now):
        i = self.info.setdefault(text, {'first': now, 'last': now, 'count': 0,
                                        'where': 'position: unknown'})
        i['last'] = now
        if now - self.seen.get(text, -1e9) < self.repeat_after:
            return
        if text not in self.seen:      # first time this code is seen: announce it once
            self.get_logger().info('QR found')
            self.found_pub.publish(String(data='QR found'))
        self.seen[text] = now
        i['count'] += 1
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
        self.info[text]['where'] = where

        os.makedirs(self.log_dir, exist_ok=True)
        snap = os.path.join(self.log_dir, f'qr_{int(time.time() * 1000)}.jpg')
        dbg = draw_detections(img.copy(), [(text, corners)], {text: [where]})
        cv2.imwrite(snap, dbg)

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
