#!/usr/bin/env bash
# force_state.sh — DDP 状态调试器
# 交互式地把 dynamics_planner_nav(Go2)强制切到指定状态分支，方便单独 debug。
# 通过向 /force_state(std_msgs/String) 发布实现：NORMAL / LOW_SPEED / RECOVERY / AUTO。
#   - NORMAL / LOW_SPEED / RECOVERY：每帧锁定到该状态(覆盖自动状态机)
#   - AUTO：解除锁定，恢复正常状态机
# 用法：./force_state.sh        (进入交互菜单，可反复切换)
#       ./force_state.sh low    (一次性切到 LOW_SPEED 后退出)

# ---- source ROS2 环境(ROS 的 setup.bash 会引用未定义变量，需临时关闭 set -u)----
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
# 尝试 source 工作区(脚本位于 <ws>/src/dynamics_planner_nav/deploy/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$SCRIPT_DIR/../../../install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    # shellcheck disable=SC1091
    source "$WS_SETUP"
fi
set -u

TOPIC="/force_state"

publish_state() {
    local state="$1"
    echo ">> 切换到: $state"
    ros2 topic pub --once "$TOPIC" std_msgs/msg/String "{data: '$state'}" >/dev/null 2>&1
    echo "   已发布到 $TOPIC"
}

# 读取并打印当前真实状态(订阅 /robot_mode，节点持续发布)
show_state() {
    local cur
    cur="$(ros2 topic echo --once /robot_mode std_msgs/msg/String 2>/dev/null | sed -n "s/^data: //p" | tr -d "'\"")"
    if [ -n "$cur" ]; then
        echo "   当前状态(/robot_mode): $cur"
    else
        echo "   读不到 /robot_mode(节点没在跑？)"
    fi
}

# 把用户输入归一化成合法状态名
normalize() {
    local in
    in="$(echo "$1" | tr '[:lower:]' '[:upper:]' | tr -d '[:space:]')"
    case "$in" in
        N|NORMAL)               echo "NORMAL" ;;
        L|LOW|LOW_SPEED|LOWSPEED) echo "LOW_SPEED" ;;
        R|RECOVERY)             echo "RECOVERY" ;;
        A|AUTO)                 echo "AUTO" ;;
        S|STATUS|SHOW)          echo "STATUS" ;;
        Q|QUIT|EXIT)            echo "QUIT" ;;
        *)                      echo "" ;;
    esac
}

# ---- 一次性模式：./force_state.sh <state> ----
if [ "$#" -ge 1 ]; then
    s="$(normalize "$1")"
    if [ -z "$s" ] || [ "$s" = "QUIT" ]; then
        echo "未知状态: $1 (可用: normal/low/recovery/auto/status)"; exit 1
    fi
    if [ "$s" = "STATUS" ]; then
        show_state
        exit 0
    fi
    publish_state "$s"
    exit 0
fi

# ---- 交互模式 ----
echo "==========================================="
echo "  DDP 状态调试器  ( -> $TOPIC )"
echo "==========================================="
echo "  输入对应字母/单词切换状态，可反复操作："
echo "    n / normal     -> 锁定 NORMAL_PLANNING"
echo "    l / low        -> 锁定 LOW_SPEED_PLANNING"
echo "    r / recovery   -> 锁定 RECOVERY"
echo "    a / auto       -> 恢复自动状态机"
echo "    s / status     -> 显示当前真实状态(/robot_mode)"
echo "    q / quit       -> 退出脚本(不改变当前状态)"
echo "-------------------------------------------"

while true; do
    printf "state> "
    read -r ans || break
    s="$(normalize "$ans")"
    if [ -z "$s" ]; then
        echo "   ?? 无法识别 '$ans'，请输入 n/l/r/a/s/q"
        continue
    fi
    if [ "$s" = "QUIT" ]; then
        echo "退出(当前强制状态保持不变；如需恢复自动请先输入 a)。"
        break
    fi
    if [ "$s" = "STATUS" ]; then
        show_state
        continue
    fi
    publish_state "$s"
    sleep 0.3
    show_state
done

