#!/usr/bin/env bash
# stop_planner.sh — 关闭 dynamics_planner_nav_node(DDP/DWA/MPPI 本地规划器节点)
#
# 用途：调试时快速停掉规划器。先发 SIGINT 让节点走正常 rclcpp 关闭流程
#       (析构、停止再发 /cmd_vel)，给桥的看门狗 0.5s 后停狗；若 2s 内没退就
#       再 SIGKILL 兜底，避免残留进程占着 /cmd_vel。
#
# 注意：本脚本只关【规划器节点】，不动 nav2、桥(cmd_vel_to_sport)、定位栈、
#       edge_model_publisher。要停别的请用各自的脚本/命令。
#
# 用法：
#   ./deploy/stop_planner.sh
#   bash <pkg>/deploy/stop_planner.sh
set -uo pipefail

# 节点二进制名(ros2 run 起来的可执行)与 ROS 节点名,两者都匹配以防漏网。
PROC_PAT="dynamics_planner_nav_node"
NODE_PAT="__node:=dynamics_planner_nav"

# 收集匹配进程(排除 grep 自己和本脚本)。
mapfile -t PIDS < <(pgrep -f "${PROC_PAT}|${NODE_PAT}" 2>/dev/null | grep -v "$$" || true)

if [[ ${#PIDS[@]} -eq 0 ]]; then
    echo "[stop_planner] 没有正在运行的 dynamics_planner_nav_node。"
    exit 0
fi

echo "[stop_planner] 发现进程: ${PIDS[*]}"
echo "[stop_planner] 发送 SIGINT(优雅关闭)..."
kill -INT "${PIDS[@]}" 2>/dev/null || true

# 等最多 2s 让它自行退出。
for _ in {1..20}; do
    sleep 0.1
    mapfile -t LEFT < <(pgrep -f "${PROC_PAT}|${NODE_PAT}" 2>/dev/null | grep -v "$$" || true)
    [[ ${#LEFT[@]} -eq 0 ]] && { echo "[stop_planner] 已正常退出。"; exit 0; }
done

echo "[stop_planner] 仍未退出,SIGKILL 兜底: ${LEFT[*]}"
kill -9 "${LEFT[@]}" 2>/dev/null || true
sleep 0.3

mapfile -t FINAL < <(pgrep -f "${PROC_PAT}|${NODE_PAT}" 2>/dev/null | grep -v "$$" || true)
if [[ ${#FINAL[@]} -eq 0 ]]; then
    echo "[stop_planner] 已强制关闭。"
else
    echo "[stop_planner] 警告:仍残留 ${FINAL[*]},请手动检查。" >&2
    exit 1
fi

