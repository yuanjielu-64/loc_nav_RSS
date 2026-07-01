#!/usr/bin/env bash
# clear_costmaps.sh — 手动清空 Nav2 的 global + local costmap
#
# 用途：DDP/dynamics_planner_nav 在调试时常需把上一次扫描堆出的"幽灵障碍/旧
#       inflation"清掉，回到从下一帧扫描重新积累的状态。
#
# 为何不直接用 /clear_costmaps：那个统一服务会等所有 lifecycle 节点 active；
#       如果 controller_server 还卡在等 TF(odom)，就会一直 hang。本脚本分两个
#       服务调用并各自带短超时，互不阻塞，定位栈没起来时也能照样清 global。
#
# 用法：
#   ./deploy/clear_costmaps.sh           # 在 nav_loc_ws 下
#   bash <pkg>/deploy/clear_costmaps.sh  # 任何位置
set -uo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

clear_one() {
    local svc="$1"
    local label="$2"
    # 5s 足够本地 service round-trip；超时即认为该 costmap 不在线/卡住，跳过。
    if timeout 5 ros2 service call "${svc}" \
            nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1; then
        echo "[clear-cm] ${label}  ✓"
    else
        echo "[clear-cm] ${label}  ✗ (服务无响应/未启动)"
    fi
}

clear_one /global_costmap/clear_entirely_global_costmap "global"
clear_one /local_costmap/clear_entirely_local_costmap   "local "

