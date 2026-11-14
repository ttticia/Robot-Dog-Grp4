"""
Human detector node (Ultralytics YOLO, 'person' class; external-computer build, no debug video).

Runs YOLO on a background thread on the newest camera frame. Optionally uses an aligned
depth image to place each person in the map frame. poster_detector consumes its output.
On the external computer it reads the robot's throttled JPEG stream and runs without depth
(external.launch.py); the stamp in each message lets poster_detector, on the robot, pair the
boxes with its local depth image.

Topics (all names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : image_topic        sensor_msgs/Image, or <image_topic>/compressed
                          sensor_msgs/CompressedImage if image_transport:=compressed
  in : depth_topic        sensor_msgs/Image   (aligned to image_topic; optional)
  in : camera_info_topic  sensor_msgs/CameraInfo
  out: /humans/detections  std_msgs/String  JSON per inference with >=1 person
"""
import json
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

from team_perception_external.human_utils import HumanDetector, box_depth_pixel
from team_perception_external.qr_utils import DepthAlignState, deproject


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

        def g(name):
            return self.get_parameter(name).value

        self.use_depth = g('use_depth')
        self.map_frame = g('map_frame')
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.running = True
        self.worker = threading.Thread(target=self.infer_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f'Human detector on {image_topic}')

    # --- callbacks ---------------------------------------------------------------
    def on_image(self, msg):
        try:
            img = self.to_bgr(msg)
        except Exception as e:  # e.g. unexpected encoding or corrupt JPEG
            self.get_logger().warn(f'image conversion failed: {e}', throttle_duration_sec=5.0)
            return
        self.latest_img = (img, msg.header)
        self.new_frame.set()

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

    # --- inference (background thread) -------------------------------------------
    def infer_loop(self):
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

            people = [self.describe(box, conf, header, img.shape) for box, conf in dets]
            if people:
                self.report(people, img, header)

            if self.min_period:
                time.sleep(max(0.0, self.min_period - (time.time() - start)))

    def describe(self, box, conf, header, shape):
        u, v = box_depth_pixel(box)
        point = self.locate(u, v, header, shape)
        frame, xyz = 'none', None
        if point is not None:
            frame, xyz = point.header.frame_id, (point.point.x, point.point.y, point.point.z)
        return {'box': [float(b) for b in box], 'conf': round(conf, 3),
                'pixel': [float(u), float(v)], 'frame': frame,
                'position': list(xyz) if xyz else None}

    def report(self, people, img, header):
        msg = {'stamp': Time.from_msg(header.stamp).nanoseconds * 1e-9,
               'frame_id': header.frame_id,
               'image_size': [int(img.shape[1]), int(img.shape[0])],   # boxes are in these px
               'count': len(people), 'people': people}
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
