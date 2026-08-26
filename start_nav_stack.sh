#!/bin/bash
# Starts the full WSL-side ROS2/Nav2 stack in one terminal:
# static_transform_publisher -> Nav2 bringup -> velocity_smoother relay,
# then runs dispatch_bridge.py in the foreground. Ctrl+C here kills
# everything cleanly (trap below).
#
# Run from WSL:
#   bash /mnt/c/isaacsim/projects/surveillance-proj/start_nav_stack.sh
#
# NOTE: start this AFTER unified_tracking.py is running in Isaac Sim
# (Nav2 needs the sim's ROS2 bridge already publishing).

set -e

PROJECT_DIR="/mnt/c/isaacsim/projects/surveillance-proj"
LOG_DIR="$PROJECT_DIR/nav_stack_logs"
mkdir -p "$LOG_DIR"

PIDS=()
cleanup() {
    echo ""
    echo "=== Shutting down nav stack ==="
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "=== Done. ==="
}
trap cleanup EXIT INT TERM

echo "[1/4] Starting static_transform_publisher (map -> odom)..."
ros2 run tf2_ros static_transform_publisher --x -13.84 --y 5.06 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id odom --ros-args -p use_sim_time:=true \
    > "$LOG_DIR/tf.log" 2>&1 &
PIDS+=($!)
sleep 2

echo "  Verifying transform is publishing..."
if timeout 5 ros2 run tf2_ros tf2_echo map odom > "$LOG_DIR/tf_check.log" 2>&1; then
    echo "  OK - map->odom transform confirmed."
else
    echo "  WARNING: could not confirm map->odom transform - check $LOG_DIR/tf.log"
fi

echo "[2/4] Launching Nav2 bringup (this takes a bit)..."
ros2 launch nav2_bringup bringup_launch.py \
    map:=/home/popli/warehouse_map.yaml \
    params_file:="$PROJECT_DIR/nav2_params.yaml" \
    use_sim_time:=true \
    > "$LOG_DIR/nav2.log" 2>&1 &
PIDS+=($!)

echo "  Waiting for Nav2 to finish activating..."
for i in $(seq 1 30); do
    if grep -q "Timed out waiting for transform" "$LOG_DIR/nav2.log" 2>/dev/null; then
        : # still settling, keep waiting
    fi
    if grep -qi "lifecycle manager successfully" "$LOG_DIR/nav2.log" 2>/dev/null; then
        echo "  Nav2 activated."
        break
    fi
    sleep 1
done
echo "  (if it's not fully up yet, give it a few more seconds - check $LOG_DIR/nav2.log)"

echo "[3/4] Starting velocity_smoother relay bypass..."
ros2 run topic_tools relay /cmd_vel_nav /cmd_vel \
    > "$LOG_DIR/relay.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[4/4] Nav stack up. Logs in $LOG_DIR/. Starting dispatch_bridge.py..."
echo "=================================================================="
python3 "$PROJECT_DIR/dispatch_bridge.py"
