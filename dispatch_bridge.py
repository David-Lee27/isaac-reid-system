#!/usr/bin/env python3
"""
Nav2 dispatch bridge - runs in WSL, alongside the static_transform_publisher,
Nav2 bringup, and the velocity_smoother relay (see NOTES.md for full
per-session checklist).

Responsibilities:
  - Default behavior: cycle the robot through PATROL_WAYPOINTS_FILE
    (written by unified_tracking.py at startup) forever.
  - Watches EVENT_LOG_FILE for new "dispatch_alert" entries. When one shows
    up, it interrupts patrol, sends a NavigateToPose goal to the alert's
    coordinates, waits for Nav2 to finish (or time out), then resumes
    patrol from wherever it ends up.

Unified_tracking.py (running on the Windows side inside Isaac Sim) never
commands movement itself anymore - it only watches the robot's REAL
position to detect arrival and trigger the on-arrival scan. This script is
the only thing actually driving navigation.

Both files are read directly off the Windows filesystem via the /mnt/c
mount, so no network/IPC setup is needed beyond the existing ROS2 bridge.

Run (after static_transform_publisher, Nav2 bringup, and the relay are all
already up and confirmed working):
    python3 dispatch_bridge.py

Press Ctrl+C to stop.
"""

import json
import os
import subprocess
import time

EVENT_LOG_FILE = "/mnt/c/isaacsim/projects/surveillance-proj/event_log.json"
PATROL_WAYPOINTS_FILE = "/mnt/c/isaacsim/projects/surveillance-proj/robot_patrol_waypoints.json"

POLL_INTERVAL = 2.0        # seconds between checking for new dispatch alerts
GOAL_TIMEOUT = 60          # seconds to wait for a nav goal before giving up

GOAL_TEMPLATE = (
    "{{pose: {{header: {{frame_id: 'map'}}, "
    "pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
    "orientation: {{w: 1.0}}}}}}}}"
)


def load_json(path):
    if not os.path.exists(path):
        return None
    with open(path, "r") as f:
        try:
            return json.load(f)
        except json.JSONDecodeError:
            return None


def load_patrol_waypoints():
    data = load_json(PATROL_WAYPOINTS_FILE)
    if not data:
        return []
    return data.get("waypoints", [])


def find_next_alert(handled_timestamps):
    log = load_json(EVENT_LOG_FILE)
    if not log:
        return None
    for e in log:
        if (e.get("event_type") == "dispatch_alert"
                and e.get("coords")
                and e.get("timestamp") not in handled_timestamps):
            return e
    return None


def send_nav_goal(x, y, label):
    """Blocks until Nav2 reports success/failure or GOAL_TIMEOUT is hit.
    Uses the plain `ros2 action send_goal` CLI (synchronous, blocks until
    the action completes) rather than rclpy, since it's already confirmed
    working end-to-end and keeps this script dependency-free."""
    goal_str = GOAL_TEMPLATE.format(x=x, y=y)
    cmd = (
        f"ros2 action send_goal /navigate_to_pose "
        f"nav2_msgs/action/NavigateToPose \"{goal_str}\""
    )
    print(f"[dispatch_bridge] sending goal ({label}): x={x}, y={y}")
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=GOAL_TIMEOUT
        )
    except subprocess.TimeoutExpired:
        print(f"[dispatch_bridge] goal timed out after {GOAL_TIMEOUT}s: {label}")
        return False

    if "Goal was rejected" in result.stdout:
        print(f"[dispatch_bridge] goal REJECTED: {label}")
        print("  (check: is static_transform_publisher still running? "
              "run `ros2 run tf2_ros tf2_echo map odom` to verify)")
        return False
    if result.returncode != 0:
        print(f"[dispatch_bridge] goal command failed: {result.stderr.strip()[:300]}")
        return False

    print(f"[dispatch_bridge] goal finished: {label}")
    return True


def main():
    handled_timestamps = set()
    patrol_index = 0

    print("=== dispatch_bridge.py running. Watching for dispatch alerts. ===")
    print("Make sure static_transform_publisher, Nav2 bringup, and the "
          "velocity_smoother relay are already running before this does anything useful.")

    try:
        while True:
            alert = find_next_alert(handled_timestamps)

            if alert:
                x, y = alert["coords"]
                label = f"ALERT: {alert.get('reason')} in {alert.get('zone')}"
                send_nav_goal(x, y, label)
                # Mark handled regardless of success/failure/timeout - we
                # don't want to get stuck retrying an unreachable point
                # forever. unified_tracking.py separately confirms arrival
                # by watching real position before running the scan.
                handled_timestamps.add(alert["timestamp"])
                continue

            waypoints = load_patrol_waypoints()
            if waypoints:
                x, y = waypoints[patrol_index % len(waypoints)]
                send_nav_goal(x, y, f"patrol waypoint {patrol_index % len(waypoints)}")
                patrol_index += 1
            else:
                print("[dispatch_bridge] no patrol waypoints file found yet, waiting...")
                time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\n=== dispatch_bridge.py stopped by user. ===")


if __name__ == "__main__":
    main()
