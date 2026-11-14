"""
Node ``step_height``: print the live foot lift from ``/joint_states``, to calibrate the gait check.

Read-only (it publishes nothing). Typical use, with the operator driving the robot:

    ros2 run maze_explorer step_height

1. Normal gait: walk slowly or turn in place for ~10 s and note the lift.
2. Operator switches to H-Step: do the same and note the lift.
3. Set ``gait_min_lift_m`` in ``config/supervisor.yaml`` between the two values.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from maze_explorer.gait_utils import StepHeightMeter


class StepHeight(Node):
    """Print the median and per-leg foot lift every ``period_s`` seconds."""

    def __init__(self):
        """Subscribe to the joint states and start the print timer."""
        super().__init__('step_height')
        window = self.declare_parameter('window_s', 4.0).value
        period = self.declare_parameter('period_s', 1.0).value
        topic = self.declare_parameter('joint_states_topic', '/joint_states').value
        self.meter = StepHeightMeter(window)
        self.count = 0
        self.create_subscription(JointState, topic, self.on_joints, 50)
        self.create_timer(period, self.report)
        self.get_logger().info(f'foot lift over the last {window:.0f} s from {topic} '
                               f'(standing still reads ~0)')

    def on_joints(self, msg):
        """Add one joint-state sample (time of receipt)."""
        self.count += 1
        self.meter.add(self.get_clock().now().nanoseconds * 1e-9, msg.name, msg.position)

    def report(self):
        """Print the current lift."""
        lift = self.meter.lift()
        if lift is None:
            self.get_logger().info(f'no joint states yet ({self.count} messages)')
            return
        legs = '  '.join(f'{k} {v * 100:4.1f}' for k, v in sorted(self.meter.lifts().items()))
        stand = self.meter.standing_height()
        self.get_logger().info(f'lift {lift * 100:4.1f} cm   [{legs}]   feet '
                               f'{-stand * 100:.0f} cm below hips')


def main(args=None):
    """Run until Ctrl+C."""
    rclpy.init(args=args)
    node = StepHeight()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
