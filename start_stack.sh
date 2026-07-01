#!/usr/bin/env bash
# start_stack.sh — bring up the PC-side Go2 helpers.
#
#   1. odom_tf_broadcaster    (/odometry/filtered -> TF odom->base_link, stamped
#                              with the PC clock, generated LOCALLY)
#   2. go2_front_camera        (front fisheye videohub api_id=1001 ->
#                                sensor_msgs/Image)
#
# WHY odom_tf_broadcaster runs HERE (not on the dog) (2026-06-10): RViz/nav2 all
# run on the PC and look up odom->base_link every frame. When the dog broadcast
# that TF at 150 Hz over the network, RViz (software-rendered, CPU-bound) could
# not drain the stream fast enough; its TF buffer stayed shallow and scan/Axes
# lookups at slightly-past stamps missed ~60% of the time -> visible flutter and
# "extrapolation" spam. Generating the TF locally from the small RELIABLE
# /odometry/filtered relay (stamped with PC now() on receipt) keeps the buffer
# tip always at ~now with zero network jitter, so lookups hit. The dog's launch
# no longer broadcasts odom->base_link (only relays /odometry/filtered) to avoid
# two publishers of the same transform.
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
    # CRITICAL: $LOC_PID/$CAM_PID are only the `ros2 run`/`ros2 launch` WRAPPERS.
    # ros2 launch frequently does NOT forward SIGTERM to the actual node, so the
    # node binary is orphaned and keeps running at full CPU. Each supervise
    # restart then leaks one orphan -> over a long session they pile up
    # (observed: 6x front_camera_node ~300% CPU). Reap the real node binaries by
    # path. Safe here: autostart holds a single-instance flock, so only ONE
    # stack ever exists; we are not killing a peer instance.
    pkill -f "go2_front_camera/lib/go2_front_camera/front_camera_node"       2>/dev/null || true
    pkill -f "nav_loc_localization/lib/nav_loc_localization/odom_tf_broadcaster" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup INT TERM

# NOTE: localization now runs on the dog (lu_localization.service). The PC only
# generates the odom->base_link TF locally (see header) and starts the fisheye.
echo "[1/2] starting local odom_tf_broadcaster (/odometry/filtered -> odom->base_link)..."
ros2 run nav_loc_localization odom_tf_broadcaster --ros-args \
    -p odom_topic:=/odometry/filtered \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    > "$LOG_DIR/odom_tf.log" 2>&1 &
LOC_PID=$!

echo "[2/2] starting go2_front_camera (front fisheye videohub api_id=1001)..."
ros2 launch go2_front_camera front_camera.launch.py poll_rate_hz:=10.0 \
    > "$LOG_DIR/front_camera.log" 2>&1 &
CAM_PID=$!

echo
echo "Stack up:"
echo "  odom_tf      pid $LOC_PID  -> $LOG_DIR/odom_tf.log"
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
