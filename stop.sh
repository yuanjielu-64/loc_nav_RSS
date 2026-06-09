#!/usr/bin/env bash
# ============================================================================
# stop.sh —— Go2 机器狗「键盘急停」一键启动器
# ----------------------------------------------------------------------------
# 用途：测试导航/让狗走路时，专门开一个终端跑这个脚本，手放在键盘上。
#       一旦狗不对劲：
#           按 空格 / 任意键  -> 立即急停
#       它会做两件事：
#         1) 杀掉导航 + cmd_vel->sport 桥接，断掉一切移动指令来源
#         2) 直接向狗连发 StopMove(1003)：停止移动但保持站立（不会瘫倒）
#
# 按键说明（在脚本启动后的终端里）：
#       空格 / 任意键  -> 急停
#       Ctrl + C       -> 也会急停，然后退出
#       q              -> 只退出本工具（不会停狗）
#
# 注意：必须在「真实终端」里直接运行（不要用管道或重定向输入），
#       否则键盘急停会被禁用。
# ============================================================================
set -eo pipefail

# --- 1. 加载 ROS2 环境 ---
source /opt/ros/humble/setup.bash
source /home/v-yuanjielu/Desktop/navigation/nav_loc_ws/install/setup.bash

# --- 2. 设置 cyclonedds（和狗通信必须一致）---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/v-yuanjielu/cyclonedds_pc.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# --- 3. 启动键盘急停节点 ---
echo "============================================================"
echo "  Go2 键盘急停已就绪"
echo "    空格 / 任意键 = 急停（停住并保持站立）"
echo "    Ctrl+C        = 急停后退出"
echo "    q             = 仅退出（不停狗）"
echo "============================================================"
exec ros2 run nav_loc_navigation estop
