#!/usr/bin/env python3
"""
global_path_provider — 只产出一条全局路径，别的什么都不做。

设计意图：
  自定义局部规划器(dynamics_planner_nav)只需要 Nav2 持续给一条 /plan，
  不需要 Nav2 去“执行导航”(FollowPath / 进度检查 / abort / 恢复)。
  bt_navigator 的 NavigateToPose 会在机器人不动时判失败并掐掉 /plan，
  这里用一个独立节点周期性调用 planner_server 的 compute_path_to_pose
  action，planner_server 每次计算都会把路径发布到 /plan —— 于是 /plan
  持续输出，与机器人是否移动完全无关。

用法：
  - 发一个目标到 /goal_pose（RViz 的 “2D Goal Pose” 工具即可）。
  - 本节点会以固定频率不断重算路径 -> planner_server 持续发 /plan。
  - 再发一个新 /goal_pose 即可切换目标；不发就一直对着上一个目标算。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


class GlobalPathProvider(Node):
    def __init__(self):
        super().__init__('global_path_provider')

        self.declare_parameter('rate', 2.0)            # 重算频率 (Hz)
        self.declare_parameter('planner_id', 'GridBased')
        rate = float(self.get_parameter('rate').value)
        self.planner_id = self.get_parameter('planner_id').value

        self._goal = None       # 最新目标 (PoseStamped)
        self._busy = False      # 是否有一次 compute 正在进行

        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, 10)
        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._on_timer)

        self.get_logger().info(
            'global_path_provider 就绪：向 /goal_pose 发一个目标即可持续产出 /plan')

    def _on_goal(self, msg: PoseStamped):
        self._goal = msg
        self.get_logger().info(
            f'收到新目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) '
            f'frame={msg.header.frame_id}')

    def _on_timer(self):
        if self._goal is None or self._busy:
            return
        if not self._client.server_is_ready():
            self.get_logger().warn('compute_path_to_pose action server 还没准备好...',
                                   throttle_duration_sec=5.0)
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._goal
        goal_msg.use_start = False          # 用机器人当前位姿作为起点(经 TF)
        goal_msg.planner_id = self.planner_id

        self._busy = True
        self._client.send_goal_async(goal_msg).add_done_callback(self._on_sent)

    def _on_sent(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'发送 compute 目标失败: {exc}')
            self._busy = False
            return
        if not handle.accepted:
            self._busy = False
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _future):
        # planner_server 在计算时已把路径发布到 /plan，这里无需再处理。
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathProvider()
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
