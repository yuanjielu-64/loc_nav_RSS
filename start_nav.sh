#!/usr/bin/env bash
# Helper: start the self-control Nav2 stack + cmd_vel_to_sport bridge.
set -eo pipefail

NAV_WS="${NAV_WS:-$HOME/Desktop/navigation/nav_loc_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$NAV_WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${HOME}/cyclonedds_pc.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0

exec ros2 launch nav_loc_navigation navigation_self_ctrl.launch.py use_cmd_bridge:=true

