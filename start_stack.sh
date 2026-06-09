#!/usr/bin/env bash
# start_stack.sh — bring up the PC-side Go2 helpers.
#
#   1. go2_front_camera        (front fisheye videohub api_id=1001 ->
#                                sensor_msgs/Image)
#
# MOVED TO THE DOG (2026-06-09): the whole localization stack
# (robot_state_publisher, joint_state_bridge, odom_tf_broadcaster, odom_relay,
# cloud_restamp x2, pointcloud_to_laserscan) now runs ON THE JETSON via the
# lu_localization systemd service (~/lu_ws on 192.168.123.18). The dog publishes
# /robot_description, /tf(_static), /joint_states, /odometry/filtered and
# /front/scan directly, so the PC no longer pulls the raw high-rate /lowstate or
# the dense /lidar_points cloud and no longer needs cloud_restamp (single Jetson
# clock). Running them here too would create a SECOND publisher of each topic
# fighting the dog's — so this script only starts the front fisheye now.
#
# To temporarily run the old PC-side localization (e.g. dog service down), use:
#     ros2 launch nav_loc_localization localization.launch.py
#
# This is NOT a systemd service (WSL has no systemd PID 1). Run it in a
# terminal; Ctrl+C stops the whole stack.
set -eo pipefail

NAV_WS="${NAV_WS:-$HOME/Desktop/navigation/nav_loc_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "$NAV_WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${HOME}/cyclonedds_pc.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0

LOG_DIR="${LOG_DIR:-/tmp}"

cleanup() {
    echo
    echo "Stopping stack..."
    [[ -n "${LOC_PID:-}"    ]] && kill "$LOC_PID"    2>/dev/null || true
    [[ -n "${CAM_PID:-}"    ]] && kill "$CAM_PID"    2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup INT TERM

# NOTE: localization now runs on the dog (lu_localization.service). Only the
# front fisheye is started here.
echo "[1/1] starting go2_front_camera (front fisheye videohub api_id=1001)..."
ros2 launch go2_front_camera front_camera.launch.py poll_rate_hz:=10.0 \
    > "$LOG_DIR/front_camera.log" 2>&1 &
CAM_PID=$!

echo
echo "Stack up:"
echo "  camera       pid $CAM_PID  -> $LOG_DIR/front_camera.log"
echo "  (localization runs on the dog: ssh go2-jetson systemctl status lu_localization)"
echo "Ctrl+C to stop everything."

# Return as soon as ANY child exits (not only when ALL of them have).
# A bare `wait` blocks until every background job is gone, so if just one node
# dies (common after a Go2 power-cycle: the front camera or a localization
# bridge drops) the stack would limp along forever with no fisheye or no odom
# TF and the supervisor would never notice. With `wait -n` we tear the whole
# stack down on the first death so the autostart supervisor relaunches it clean.
wait -n || true
echo "A stack component exited — tearing the stack down for a clean restart."
cleanup
exit 1
