#!/usr/bin/env python3
"""Keyboard emergency-stop for the Go2 dog.

Run this in its OWN dedicated terminal during any navigation / movement test.
Keep your hand on the keyboard. The moment anything looks wrong:

    *** PRESS  SPACE  (or any key) ***  -> EMERGENCY STOP

On trigger it does two things, in this order, so the dog cannot keep moving:
  1. Kills the navigation stack + cmd_vel->sport bridge so NO more Move
     commands can be produced.
  2. Floods StopMove (sport api_id 1003) directly to /api/sport/request for a
     few seconds. StopMove halts motion but KEEPS the dog balanced/standing
     (unlike Damp, which makes it go limp and can fall over).

Keys:
  SPACE / any key  -> emergency stop
  q                -> quit this tool (does NOT stop the dog)

This talks straight to the robot board over cyclonedds, independent of Nav2,
the SDK, or the bridge -- so it still works even if those have hung.
"""

import json
import os
import select
import signal
import subprocess
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from unitree_api.msg import Request

SPORT_TOPIC = "/api/sport/request"
SPORT_API_ID_STOPMOVE = 1003   # StopMove: stop motion, keep standing
SPORT_API_ID_MOVE = 1008       # Move: used to send an explicit zero velocity

# Process name patterns to kill so no further Move commands are generated.
# NOTE: keep these specific. Do NOT use a broad pattern like "nav_loc_navigation"
# because this e-stop tool itself runs as `ros2 run nav_loc_navigation estop`
# and would kill itself before it finishes flooding StopMove.
KILL_PATTERNS = [
    "cmd_vel_to_sport",
    "navigation.launch.py",
    "controller_server",
    "bt_navigator",
    "planner_server",
    "behavior_server",
    "waypoint_follower",
]

FLOOD_HZ = 20.0
FLOOD_SECONDS = 3.0


class EStop(Node):
    def __init__(self):
        super().__init__("estop")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._pub = self.create_publisher(Request, SPORT_TOPIC, qos)
        self._triggered = False
        self._flood_ticks_left = 0
        # High-rate timer; only does work once triggered.
        self.create_timer(1.0 / FLOOD_HZ, self._tick)
        self._prompt()

    # ---- console UI helpers (printed straight to the terminal) ----
    @staticmethod
    def _say(msg):
        print(msg, flush=True)

    def _prompt(self):
        self._say("")
        self._say("  \033[32m\u25cf 待命中 (ARMED)\033[0m  \u2014  "
                  "按 [\033[1m空格/任意键\033[0m]=急停   [\033[1mq\033[0m]=退出")

    def _kill_movers(self):
        for pat in KILL_PATTERNS:
            try:
                subprocess.run(["pkill", "-f", pat],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL,
                               check=False)
            except Exception:
                pass

    def _send_stopmove(self):
        req = Request()
        req.header.identity.api_id = SPORT_API_ID_STOPMOVE
        req.parameter = ""
        self._pub.publish(req)

    def _send_zero_move(self):
        req = Request()
        req.header.identity.api_id = SPORT_API_ID_MOVE
        req.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.0})
        self._pub.publish(req)

    def trigger(self):
        if self._triggered:
            # Re-triggering just refreshes the flood window.
            self._flood_ticks_left = int(FLOOD_HZ * FLOOD_SECONDS)
            return
        self._triggered = True
        self._say("")
        self._say("  \033[41;97;1m  >>> 急停 EMERGENCY STOP <<<  \033[0m")
        self._say("  正在切断导航 + 连发 StopMove 给狗 "
                  f"({FLOOD_SECONDS:.0f} 秒)...")
        # 1) Cut off the source of Move commands.
        self._kill_movers()
        # 2) Start flooding StopMove.
        self._flood_ticks_left = int(FLOOD_HZ * FLOOD_SECONDS)

    def _tick(self):
        if self._flood_ticks_left > 0:
            self._send_stopmove()
            self._send_zero_move()
            self._flood_ticks_left -= 1
            if self._flood_ticks_left == 0:
                # Flood finished; let the user know we are idle but still armed.
                self._say("  \033[32m\u2713 已发送停止指令\033[0m "
                          "\u2014 狗应已停住并保持站立。")
                self._triggered = False
                self._prompt()


def _keyboard_loop(node: EStop):
    """Read single keypresses in raw mode and trigger e-stop."""
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        node.get_logger().warn(
            "stdin is not a TTY -> keyboard e-stop disabled. "
            "Run this directly in an interactive terminal. "
            "(Ctrl-C still triggers an emergency stop.)")
        return
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == "q":
                node._say("  \033[33m已退出急停工具(未对狗发停止指令)。\033[0m")
                rclpy.shutdown()
                return
            # Any other key = emergency stop.
            node.trigger()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = EStop()

    import threading
    t = threading.Thread(target=_keyboard_loop, args=(node,), daemon=True)
    t.start()

    # Also treat Ctrl-C as an emergency stop, not just a quit.
    def _on_sigint(signum, frame):
        node.trigger()
        # Give the flood a moment, then exit.
        node._say("  (Ctrl+C) 急停后退出...")
        end = node.get_clock().now().nanoseconds + int(FLOOD_SECONDS * 1e9)
        while node.get_clock().now().nanoseconds < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
