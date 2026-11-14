"""
Node ``cmd_vel_relay``: Nav2's velocity output -> the robot's ``/cmd_vel``, safely.

Nav2 publishes ``/nav_cmd_vel`` with default (VOLATILE) QoS, which doesn't connect to the
Lite3 bridge's ``/cmd_vel`` subscription. This node republishes it the way ``robot_info.md``
§6.2 asks:

- RELIABLE + TRANSIENT_LOCAL, depth 1, short lifespan (so a stale latched command isn't
  replayed to a late subscriber);
- speeds clamped to the team limits;
- republished at ``rate_hz`` while input is fresh; when the input goes quiet for
  ``stale_timeout`` s, one zero Twist is sent and then nothing, so teleop can take over;
- a zero Twist on shutdown.

Don't run it while the lab's ``twist_mux`` also publishes ``/cmd_vel`` (one publisher only).
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def clamp(v, limit):
    """Clamp ``v`` to [-limit, limit]."""
    return max(-limit, min(limit, v))


class CmdVelRelay(Node):
    """Relay with clamping, a staleness stop and the bridge's QoS."""

    def __init__(self):
        """Declare parameters and connect the two topics."""
        super().__init__('cmd_vel_relay')
        p = self.declare_parameter
        in_topic = p('input_topic', '/nav_cmd_vel').value
        out_topic = p('output_topic', '/cmd_vel').value
        self.max_vx = p('max_linear_x', 0.25).value
        self.max_vy = p('max_linear_y', 0.1).value
        self.max_wz = p('max_angular_z', 0.5).value
        self.stale = Duration(seconds=p('stale_timeout', 0.5).value)
        lifespan = p('lifespan', 0.3).value
        rate = p('rate_hz', 20.0).value
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         lifespan=Duration(seconds=lifespan))
        self.pub = self.create_publisher(Twist, out_topic, qos)
        self.create_subscription(Twist, in_topic, self.on_cmd, 10)
        self.last = Twist()
        self.last_time = None
        self.active = False
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'{in_topic} -> {out_topic} (|vx| <= {self.max_vx}, '
                               f'|vy| <= {self.max_vy}, |wz| <= {self.max_wz})')

    def on_cmd(self, msg):
        """Store the latest command, clamped."""
        t = Twist()
        t.linear.x = clamp(msg.linear.x, self.max_vx)
        t.linear.y = clamp(msg.linear.y, self.max_vy)
        t.angular.z = clamp(msg.angular.z, self.max_wz)
        self.last = t
        self.last_time = self.get_clock().now()

    def tick(self):
        """Republish while fresh; one zero Twist when the input goes quiet."""
        fresh = (self.last_time is not None
                 and self.get_clock().now() - self.last_time <= self.stale)
        if fresh:
            self.pub.publish(self.last)
            self.active = True
        elif self.active:
            self.pub.publish(Twist())
            self.active = False

    def stop(self):
        """Send a zero Twist (on shutdown)."""
        self.pub.publish(Twist())


def main(args=None):
    """Run the relay until Ctrl+C, then send a zero Twist."""
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
