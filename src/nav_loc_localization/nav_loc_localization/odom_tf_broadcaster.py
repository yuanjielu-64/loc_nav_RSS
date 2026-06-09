"""Broadcast odom -> base_link TF from the Go2's native /utlidar/robot_odom.

In cyclonedds mode go2_robot_sdk does NOT publish the odom->base_link transform
(its _on_cyclonedds_pose callback is a `pass`), so rviz reports
"Frame [odom] does not exist". The Go2 main board already publishes a fully
formed nav_msgs/Odometry on /utlidar/robot_odom (~150 Hz, header.frame_id=odom,
child_frame_id=base_link). This node simply re-broadcasts that pose as a TF so
the whole TF tree (odom -> base_link -> URDF links) is connected.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('nav_loc_odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        odom_topic = self.get_parameter('odom_topic').value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos)

        self._count = 0
        self._last_report = time.monotonic()
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"odom_tf_broadcaster up: {odom_topic} -> TF "
            f"{self.odom_frame} -> {self.base_frame}")

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        # Stamp with the PC clock, NOT msg.header.stamp: the Go2 main board's
        # clock is not synced to the PC (can be off by months), and
        # robot_state_publisher stamps the URDF link TFs with the PC clock from
        # /joint_states. Using the robot stamp here would split the TF tree in
        # time and make odom->base_link->URDF lookups fail in rviz.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)
        self._count += 1

    def _report(self):
        now = time.monotonic()
        dt = now - self._last_report
        rate = self._count / dt if dt > 0 else 0.0
        self.get_logger().info(
            f"broadcast {self._count} odom TFs ({rate:.1f} Hz)")
        self._count = 0
        self._last_report = now


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
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
