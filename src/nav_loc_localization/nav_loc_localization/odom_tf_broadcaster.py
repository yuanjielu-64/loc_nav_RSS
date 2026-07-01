"""Broadcast odom -> base_link TF from the Go2's native /utlidar/robot_odom.

In cyclonedds mode go2_robot_sdk does NOT publish the odom->base_link transform
(its _on_cyclonedds_pose callback is a `pass`), so rviz reports
"Frame [odom] does not exist". The Go2 main board already publishes a fully
formed nav_msgs/Odometry on /utlidar/robot_odom (~150 Hz, header.frame_id=odom,
child_frame_id=base_link). This node simply re-broadcasts that pose as a TF so
the whole TF tree (odom -> base_link -> URDF links) is connected.

ZUPT (zero-velocity update) — added 2026-06-17:
  Go2 main board's "odom" is purely IMU/leg integration (no magnetometer, no
  EKF correction), so even when the dog stands perfectly still, yaw drifts
  monotonically (measured: +0.66°/s ≈ +40°/min). On RViz this looks like the
  dog spinning, and worse: laser-built obstacles (in odom frame) appear to
  swirl around the robot, so the costmap fills with ghost obstacles.
  Empirically the lidar-IMU drifts the OPPOSITE direction by a smaller amount,
  and the lowstate IMU shares the same gyro as robot_odom (no help via EKF).
  Therefore: when the body is supposed to be stationary, snap the broadcast
  pose to the last "moving" pose (freeze yaw AND xy). The instant any motion
  is detected we release the freeze and pass through. Detection uses the
  Odometry msg's own twist (no extra topics), with hysteresis on |v|+|ω| over
  a short window so we don't latch on a single noisy frame.
"""

import math
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
        # ZUPT thresholds. Defaults tuned for Go2 standing in place: balance
        # corrections never exceed ~0.05 m/s linear and ~0.05 rad/s yaw rate.
        # Real walking always exceeds these by 5–10×, so freeze releases
        # within one frame of any commanded motion.
        self.declare_parameter('zupt_enabled', True)
        self.declare_parameter('zupt_lin_thresh', 0.05)   # m/s
        self.declare_parameter('zupt_ang_thresh', 0.05)   # rad/s
        # Frames of consecutive "below threshold" needed to enter freeze (~0.1s
        # at 150 Hz). Single noisy frame won't latch. Releasing is INSTANT
        # (one above-threshold frame breaks the freeze).
        self.declare_parameter('zupt_enter_frames', 15)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        self.zupt_enabled = bool(self.get_parameter('zupt_enabled').value)
        self.zupt_lin_thresh = float(self.get_parameter('zupt_lin_thresh').value)
        self.zupt_ang_thresh = float(self.get_parameter('zupt_ang_thresh').value)
        self.zupt_enter_frames = int(self.get_parameter('zupt_enter_frames').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos)

        # ZUPT state
        self._still_count = 0          # consecutive below-threshold frames
        self._frozen = False           # currently latched?
        self._frozen_pose = None       # (px, py, pz, qx, qy, qz, qw)
        self._zupt_freeze_n = 0        # how many TFs published while frozen
        self._zupt_release_n = 0       # how many releases this report period

        self._count = 0
        self._last_report = time.monotonic()
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"odom_tf_broadcaster up: {odom_topic} -> TF "
            f"{self.odom_frame} -> {self.base_frame} | "
            f"ZUPT={'on' if self.zupt_enabled else 'off'} "
            f"(lin<{self.zupt_lin_thresh} ang<{self.zupt_ang_thresh}, "
            f"enter after {self.zupt_enter_frames} frames)")

    def _on_odom(self, msg: Odometry):
        # ---- ZUPT decision: still or moving? ----
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        lin_speed = math.sqrt(v.x * v.x + v.y * v.y)
        ang_speed = abs(w.z)
        is_still = (lin_speed < self.zupt_lin_thresh and
                    ang_speed < self.zupt_ang_thresh)

        if self.zupt_enabled and is_still:
            self._still_count += 1
            if self._still_count >= self.zupt_enter_frames:
                if not self._frozen:
                    # Latch the pose at THIS instant — last "good" reading.
                    self._frozen = True
                    self._frozen_pose = (
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w,
                    )
        else:
            # Above threshold (or ZUPT disabled) — release freeze immediately.
            if self._frozen:
                self._zupt_release_n += 1
            self._still_count = 0
            self._frozen = False
            self._frozen_pose = None

        # ---- Build TF ----
        t = TransformStamped()
        # Stamp with the PC clock, NOT msg.header.stamp: the Go2 main board's
        # clock is not synced to the PC (can be off by months), and
        # robot_state_publisher stamps the URDF link TFs with the PC clock from
        # /joint_states. Using the robot stamp here would split the TF tree in
        # time and make odom->base_link->URDF lookups fail in rviz.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        if self._frozen and self._frozen_pose is not None:
            px, py, pz, qx, qy, qz, qw = self._frozen_pose
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = pz
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._zupt_freeze_n += 1
        else:
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
        froz_pct = (100.0 * self._zupt_freeze_n / self._count) if self._count else 0.0
        self.get_logger().info(
            f"broadcast {self._count} odom TFs ({rate:.1f} Hz) | "
            f"ZUPT frozen {self._zupt_freeze_n} ({froz_pct:.0f}%) "
            f"releases={self._zupt_release_n}")
        self._count = 0
        self._zupt_freeze_n = 0
        self._zupt_release_n = 0
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
