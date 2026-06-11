"""Re-stamp the Go2's native deskewed LiDAR cloud with the PC clock.

The Go2 main board publishes /utlidar/cloud_deskewed (~15 Hz, frame_id=odom)
stamped with the ROBOT clock, which is not synced to the PC (can be off by
months). Our odom->base_link TF (odom_tf_broadcaster) is stamped with the PC
clock, as are the URDF link TFs from robot_state_publisher. Feeding the
robot-stamped cloud straight into pointcloud_to_laserscan makes its tf2 message
filter drop every frame ("timestamp ... earlier than all the data in the
transform cache"), so /front/scan stays empty.

This node simply republishes the cloud with header.stamp = PC now(), so the
downstream TF lookup (base_link <- odom at PC-now) succeeds. The robot streams
in real time with ~ms latency, so using now() is the same approximation we
already make for the odom TF itself.
"""

import time

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudRestamp(Node):
    def __init__(self):
        super().__init__('nav_loc_cloud_restamp')

        self.declare_parameter('in_topic', '/utlidar/cloud_deskewed')
        self.declare_parameter('out_topic', '/utlidar/cloud_deskewed_pcstamp')
        # Subscription reliability: 'best_effort' (Go2 native cloud) or
        # 'reliable' (HESAI hesai_ros_driver_node publishes RELIABLE).
        self.declare_parameter('reliability', 'best_effort')
        # Drop returns within this horizontal radius (m) of the LiDAR origin
        # (the robot's own mount/structure). 0.0 disables the filter.
        self.declare_parameter('self_filter_radius', 0.0)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        reliability = str(self.get_parameter('reliability').value).lower()
        self._self_r = float(self.get_parameter('self_filter_radius').value)

        sub_rel = (ReliabilityPolicy.RELIABLE if reliability == 'reliable'
                   else ReliabilityPolicy.BEST_EFFORT)

        sub_qos = QoSProfile(reliability=sub_rel,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self._on_cloud, sub_qos)

        self._count = 0
        self._last_report = time.monotonic()
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"cloud_restamp up: {in_topic} ({reliability}) -> {out_topic} "
            f"(PC clock)")

    def _on_cloud(self, msg: PointCloud2):
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._self_r > 0.0:
            msg = self._drop_self(msg)
        self.pub.publish(msg)
        self._count += 1

    def _drop_self(self, msg: PointCloud2) -> PointCloud2:
        """Remove points within self_filter_radius (horizontal) of the LiDAR
        origin. x,y are float32 at offsets 0,4."""
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, step)
        xy = raw[:, 0:8].copy().view(np.float32).reshape(-1, 2)
        d2 = xy[:, 0] * xy[:, 0] + xy[:, 1] * xy[:, 1]
        keep = d2 >= (self._self_r * self._self_r)  # NaN -> False -> dropped
        kept = raw[keep]
        msg.data = kept.tobytes()
        n = int(kept.shape[0])
        msg.height = 1
        msg.width = n
        msg.row_step = n * step
        msg.is_dense = True
        return msg

    def _report(self):
        now = time.monotonic()
        dt = now - self._last_report
        if dt > 0:
            rate = self._count / dt
            self.get_logger().info(
                f"restamped {self._count} clouds ({rate:.1f} Hz)")
        self._count = 0
        self._last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = CloudRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
