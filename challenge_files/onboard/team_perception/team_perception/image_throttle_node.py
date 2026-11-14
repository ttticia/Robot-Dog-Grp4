"""
Image throttle node: forwards the camera's JPEG stream to the external computer at a lower rate.

The external computer runs YOLO (human_detector) at about 10 frames a second, so sending all
30 camera frames over Wi-Fi wastes two thirds of the bandwidth (about 18 Mbit/s instead of 6).
Messages are forwarded unchanged, header stamp included, so poster_detector can still pair the
returned person boxes with the depth image of the same moment.

Topics (names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : in_topic   sensor_msgs/CompressedImage  (default /camera/color/image_raw/compressed)
  out: out_topic  sensor_msgs/CompressedImage  (default /camera/color/image_offboard/compressed)
The output is BEST_EFFORT with depth 1: over a slow link a late frame is dropped instead of
queued, so the external computer always gets the newest frame.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class ImageThrottle(Node):
    def __init__(self):
        super().__init__('image_throttle')
        p = self.declare_parameter
        p('in_topic', '/camera/color/image_raw/compressed')
        p('out_topic', '/camera/color/image_offboard/compressed')
        p('rate_hz', 10.0)             # max frames per second sent out (0 = no limit)

        def g(name):
            return self.get_parameter(name).value

        rate = g('rate_hz')
        self.min_period = 1.0 / rate if rate > 0 else 0.0
        self.last_sent = 0.0
        out_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(CompressedImage, g('out_topic'), out_qos)
        self.create_subscription(CompressedImage, g('in_topic'), self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'Forwarding {g("in_topic")} -> {g("out_topic")} '
                               f'at <= {rate:g} Hz')

    def on_image(self, msg):
        now = time.monotonic()
        # Small tolerance so a 30 Hz camera throttled to 10 Hz sends every 3rd frame, not 4th.
        if self.min_period and now - self.last_sent < 0.9 * self.min_period:
            return
        self.last_sent = now
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImageThrottle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
