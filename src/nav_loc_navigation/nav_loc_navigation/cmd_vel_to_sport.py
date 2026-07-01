# Copyright (c) 2026
# SPDX-License-Identifier: BSD-2-Clause
"""
cmd_vel_to_sport.py — bridge Nav2's /cmd_vel to the Go2's native sport API.

The Go2 main board natively subscribes to /api/sport/request
(unitree_api/msg/Request) over wired cyclonedds. We don't need WebRTC at all:
this node converts each geometry_msgs/Twist on /cmd_vel into a sport-mode
"Move" request (api_id 1008, parameter {"x","y","z"}) and publishes it there.

Safety:
  * Velocities are clamped to configurable limits.
  * A watchdog sends a single zero-velocity stop if no /cmd_vel arrives for
    `cmd_timeout` seconds (Nav2 also sends 0 on goal completion).

Run standalone:
    ros2 run nav_loc_navigation cmd_vel_to_sport
"""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request

SPORT_API_ID_MOVE = 1008


class CmdVelToSport(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_sport')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sport_topic', '/api/sport/request')
        self.declare_parameter('max_x', 1.0)        # m/s forward/back
        self.declare_parameter('max_y', 1.0)        # m/s strafe(同 max_x：planner 已限准，桥不再当瓶颈)
        self.declare_parameter('max_yaw', 3.0)      # rad/s turn
        self.declare_parameter('cmd_timeout', 0.5)  # s; stop if no cmd

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        sport_topic = self.get_parameter('sport_topic').value
        self.max_x = float(self.get_parameter('max_x').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.max_yaw = float(self.get_parameter('max_yaw').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.pub = self.create_publisher(Request, sport_topic, 10)
        self.sub = self.create_subscription(
            Twist, cmd_vel_topic, self._on_cmd_vel, 10)

        self._last_cmd_time = self.get_clock().now()
        self._was_moving = False
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'cmd_vel_to_sport: {cmd_vel_topic} -> {sport_topic} '
            f'(Move api_id={SPORT_API_ID_MOVE}); '
            f'limits x={self.max_x} y={self.max_y} yaw={self.max_yaw}')

    @staticmethod
    def _clamp(v: float, lim: float) -> float:
        return max(-lim, min(lim, v))

    def _send_move(self, x: float, y: float, z: float) -> None:
        req = Request()
        req.header.identity.api_id = SPORT_API_ID_MOVE
        req.parameter = json.dumps({'x': round(x, 3),
                                    'y': round(y, 3),
                                    'z': round(z, 3)})
        self.pub.publish(req)

    def _on_cmd_vel(self, msg: Twist) -> None:
        x = self._clamp(msg.linear.x, self.max_x)
        y = self._clamp(msg.linear.y, self.max_y)
        z = self._clamp(msg.angular.z, self.max_yaw)
        self._send_move(x, y, z)
        self._last_cmd_time = self.get_clock().now()
        self._was_moving = (x != 0.0 or y != 0.0 or z != 0.0)

    def _watchdog(self) -> None:
        if not self._was_moving:
            return
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age >= self.cmd_timeout:
            self._send_move(0.0, 0.0, 0.0)
            self._was_moving = False
            self.get_logger().warn(
                f'No /cmd_vel for {age:.2f}s — sent stop.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToSport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
