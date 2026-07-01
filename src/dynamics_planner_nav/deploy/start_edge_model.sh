#!/usr/bin/env bash
# start_edge_model.sh — 启动 edge_model_publisher 节点
#   订阅 /robot_edge_profile，发布 /robot_collision_models(+_viz)。
#
# 源码位于 py_scripts/edge_model_publisher.py（与 scripts/ 里的杂项区分开）。
#
# 用法：
#   ./deploy/start_edge_model.sh            # 前台运行
#   nohup ./deploy/start_edge_model.sh &    # 后台运行
set -euo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_DIR="/home/v-yuanjielu/Desktop/navigation/nav_loc_ws"
PY="${WS_DIR}/src/dynamics_planner_nav/py_scripts/edge_model_publisher.py"

# 安全膨胀 margin（米）：模型每条边向外扩这么多。默认源码是 0.05，这里设 0.065(=5cm+1.5cm)。
MARGIN="${EDGE_MARGIN:-0.065}"
ROS_ARGS=(--ros-args -p "margin:=${MARGIN}")

# ROS/ament setup 脚本会引用未定义变量(如 AMENT_TRACE_SETUP_FILES)，在 set -u
# 下会直接报错退出，故 source 期间临时关闭 -u，结束后恢复。
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"
set -u

if [[ -s "${PY}" ]]; then
    echo "[start_edge_model] running: ${PY} (margin=${MARGIN})"
    exec python3 "${PY}" "${ROS_ARGS[@]}"
else
    echo "[start_edge_model] ERROR: 找不到 ${PY}" >&2
    exit 1
fi

