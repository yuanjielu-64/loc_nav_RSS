"""Relay the Go2's native /utlidar/robot_odom to /odometry/filtered.

The Go2 main board publishes a fully formed nav_msgs/Odometry on
/utlidar/robot_odom (~150 Hz, frame_id=odom, child_frame_id=base_link) with
BEST_EFFORT QoS. Downstream consumers (e.g. dynamics_planner_nav) were written
for a Jackal and subscribe to /odometry/filtered with default (RELIABLE) QoS.

This node subscribes BEST_EFFORT to the robot odom and republishes the message
unchanged on /odometry/filtered with RELIABLE QoS so those consumers connect
without a QoS mismatch. No filtering/transform is applied: the Go2 odom is
already fused, so it stands in directly for robot_localization's output.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__('nav_loc_odom_relay')

        self.declare_parameter('in_topic', '/utlidar/robot_odom')
        self.declare_parameter('out_topic', '/odometry/filtered')

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(Odometry, out_topic, pub_qos)
        self.sub = self.create_subscription(
            Odometry, in_topic, self._on_odom, sub_qos)

        self._count = 0
        self._last_report = time.monotonic()
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"odom_relay up: {in_topic} (BEST_EFFORT) -> "
            f"{out_topic} (RELIABLE)")

    def _on_odom(self, msg: Odometry):
        self.pub.publish(msg)
        self._count += 1

    def _report(self):
        now = time.monotonic()
        dt = now - self._last_report
        rate = self._count / dt if dt > 0 else 0.0
        self.get_logger().info(f"relayed {self._count} odom msgs ({rate:.1f} Hz)")
        self._count = 0
        self._last_report = now


def main():
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
