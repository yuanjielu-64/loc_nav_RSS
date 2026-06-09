"""Bridge: Unitree Go2 /lowstate -> sensor_msgs/JointState on /joint_states.

go2_robot_sdk's cyclonedds-mode driver registers a JointState publisher but its
_on_cyclonedds_low_state callback is a `pass`, so /joint_states never gets data
and rviz renders the URDF in its default zero pose. This node subscribes to
/lowstate (published natively by the Go2 main board over the wired CycloneDDS
link) and republishes the 12 leg joint angles in the order the URDF /
robot_state_publisher expects.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


# Order must match the URDF joints expected by robot_state_publisher.
# go2_robot_sdk's go2.urdf uses these exact joint names (single-robot mode).
JOINT_NAMES = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]

# Map JOINT_NAMES position -> motor_state index. Same convention used inside
# go2_robot_sdk/infrastructure/ros2/ros2_publisher.py::publish_joint_state.
MOTOR_INDEX = [
    3, 4, 5,    # FL
    0, 1, 2,    # FR
    9, 10, 11,  # RL
    6, 7, 8,    # RR
]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('nav_loc_joint_state_bridge')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # /lowstate arrives at ~500 Hz from the Go2 main board. Republishing
        # 1:1 floods robot_state_publisher/TF and pins rviz at >200% CPU. Cap
        # the /joint_states output rate (50 Hz is plenty for viz + nav).
        self.declare_parameter('publish_rate_hz', 50.0)
        rate_hz = self.get_parameter('publish_rate_hz').value
        self._min_period = (1.0 / rate_hz) if rate_hz > 0 else 0.0
        self._last_pub = 0.0

        self.pub = self.create_publisher(JointState, '/joint_states', qos)
        self.sub = self.create_subscription(
            LowState, '/lowstate', self._on_lowstate, qos)

        self._count = 0
        self._last_report = time.monotonic()
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            "joint_state_bridge up: /lowstate -> /joint_states "
            f"({len(JOINT_NAMES)} joints, throttled to {rate_hz:.0f} Hz)")

    def _on_lowstate(self, msg: LowState):
        # Throttle: drop samples that arrive faster than publish_rate_hz.
        now = time.monotonic()
        if self._min_period and (now - self._last_pub) < self._min_period:
            return
        self._last_pub = now
        try:
            ms = msg.motor_state
            positions = [float(ms[i].q) for i in MOTOR_INDEX]
            velocities = [float(ms[i].dq) for i in MOTOR_INDEX]
            efforts = [float(ms[i].tau_est) for i in MOTOR_INDEX]
        except (IndexError, AttributeError) as e:
            self.get_logger().warn(f"unexpected LowState shape: {e}")
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = JOINT_NAMES
        out.position = positions
        out.velocity = velocities
        out.effort = efforts
        self.pub.publish(out)
        self._count += 1

    def _report(self):
        now = time.monotonic()
        dt = now - self._last_report
        rate = self._count / dt if dt > 0 else 0.0
        self.get_logger().info(
            f"published {self._count} joint_states ({rate:.1f} Hz)")
        self._count = 0
        self._last_report = now


def main():
    rclpy.init()
    node = JointStateBridge()
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
