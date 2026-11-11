"""
Webcam bridge node (laptop testing only).

Receives JPEG frames from scripts/webcam_sender_windows.py over TCP and publishes
them, so qr_scanner can be tested with a laptop webcam that WSL2 cannot access:
  <topic>/compressed  sensor_msgs/CompressedImage  the JPEG as received (no re-encoding)
  <topic>             sensor_msgs/Image (bgr8)     only decoded if someone subscribes

  ros2 run team_perception webcam_bridge
  ros2 run team_perception qr_scanner --ros-args -p use_depth:=false \
      -p image_transport:=compressed

ROS parameters
  port      (int, 5005)                      TCP port to listen on
  topic     (str, /camera/color/image_raw)  output image topic
  frame_id  (str, camera_color_optical_frame)
"""
import socket
import struct
import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

# First bytes webcam_sender_windows.py sends on each connection. Anything else
# (VS Code port-forward probes, browsers sending "GET /") is dropped quietly.
MAGIC = b'QRCAM1\n'


def _recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


class WebcamBridge(Node):
    def __init__(self):
        super().__init__('webcam_bridge')
        self.declare_parameter('port', 5005)
        self.declare_parameter('topic', '/camera/color/image_raw')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 1)
        self.jpeg_pub = self.create_publisher(CompressedImage, topic.rstrip('/') + '/compressed',
                                              1)
        self.count = 0
        self.create_timer(5.0, self.report_rate)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 0.0.0.0 so Windows' localhost forwarding (and mirrored networking) both reach us.
        self.server.bind(('0.0.0.0', self.port))
        self.server.listen(1)
        threading.Thread(target=self.serve, daemon=True).start()
        self.get_logger().info(f'Waiting for webcam_sender_windows.py on port {self.port}, '
                               f'publishing to {topic} and {topic}/compressed')

    def serve(self):
        while rclpy.ok():
            try:
                conn, addr = self.server.accept()
            except OSError:
                return  # socket closed on shutdown
            with conn:
                conn.settimeout(5.0)
                try:
                    hello = _recv_exact(conn, len(MAGIC))
                except OSError:
                    hello = None
                if hello != MAGIC:
                    self.get_logger().debug(f'Ignored non-sender connection: {hello!r}')
                    continue
                conn.settimeout(None)
                self.get_logger().info(f'Sender connected from {addr[0]}')
                try:
                    self.receive(conn)
                except OSError as e:
                    self.get_logger().warn(f'Connection error: {e}')
            self.get_logger().warn('Sender disconnected; waiting for it to reconnect')

    def receive(self, conn):
        while rclpy.ok():
            header = _recv_exact(conn, 4)
            if header is None:
                return
            (size,) = struct.unpack('>I', header)
            if size == 0 or size > 20_000_000:
                self.get_logger().error(f'Bad frame size {size}; dropping connection')
                return
            data = _recv_exact(conn, size)
            if data is None:
                return
            stamp = self.get_clock().now().to_msg()
            if self.jpeg_pub.get_subscription_count() > 0:
                jpg = CompressedImage(format='jpeg')
                jpg.header.stamp, jpg.header.frame_id = stamp, self.frame_id
                jpg.data = data
                self.jpeg_pub.publish(jpg)
            if self.pub.get_subscription_count() > 0:
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
                self.pub.publish(msg)
            self.count += 1

    def report_rate(self):
        if self.count:
            self.get_logger().info(f'publishing {self.count / 5.0:.1f} fps')
        self.count = 0

    def destroy_node(self):
        self.server.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
