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
import math
import os
import subprocess
import time

EVENT_LOG_FILE = "/mnt/c/isaacsim/projects/surveillance-proj/event_log.json"
PATROL_WAYPOINTS_FILE = "/mnt/c/isaacsim/projects/surveillance-proj/robot_patrol_waypoints.json"
FACE_SEARCH_GOAL_FILE = "/mnt/c/isaacsim/projects/surveillance-proj/robot_face_search_goal.json"

POLL_INTERVAL = 2.0        # seconds between checking for new dispatch alerts
GOAL_TIMEOUT = 240        # seconds to wait for a nav goal before giving up - generous
                          # because unified_tracking.py's camera sweeps still block the
                          # sim's main loop for several seconds each (~every 8s), so real
                          # navigation progress is slower than wall-clock time suggests

# Zone-camera coordinates (ZONE_CENTERS in unified_tracking.py) are chosen
# for camera FRAMING - they sit right up against the walls/corners the
# cameras are mounted near. Sending the robot literally TO that exact point
# means its destination is inside or hard against a wall, which is why it
# kept driving into corners/pillars: not a costmap/obstacle-avoidance bug,
# the goal itself was physically unreachable. Pull every dispatch goal
# inward toward the map's rough center by this many meters before sending
# it, so the robot parks near the alert zone instead of on top of the wall.
GOAL_STANDOFF_METERS = 3.0

GOAL_TEMPLATE = (
    "{{pose: {{header: {{frame_id: 'map'}}, "
    "pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
    "orientation: {{z: {qz}, w: {qw}}}}}}}}}"
)


def yaw_deg_to_quat(yaw_deg):
    """Pure-yaw (rotation about Z only) quaternion, since the robot never
    needs to pitch/roll for navigation goals."""
    half = math.radians(yaw_deg) / 2.0
    return math.sin(half), math.cos(half)  # (qz, qw)


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


def apply_goal_standoff(x, y):
    """Pulls a raw zone-center coordinate inward toward the map's rough
    center (0,0) by GOAL_STANDOFF_METERS, so the robot's actual nav goal is
    a reachable point near the alert zone instead of the exact wall-mounted
    camera position."""
    dist = math.hypot(x, y)
    if dist <= GOAL_STANDOFF_METERS:
        return x, y
    scale = (dist - GOAL_STANDOFF_METERS) / dist
    return x * scale, y * scale


def find_face_search_goal(handled_ids):
    """Checks for a pending face-search sub-goal (a same-location, new-
    heading 'turn to look this way' request written by unified_tracking.py
    while the robot is actively hunting for a usable face at a dispatch
    location). Checked with the HIGHEST priority in the main loop - while a
    search is in progress we want the robot committed to it, not wandering
    back to patrol or accepting a newer, unrelated dispatch alert mid-turn.
    """
    data = load_json(FACE_SEARCH_GOAL_FILE)
    if not data:
        return None
    if data.get("request_id") in handled_ids:
        return None
    return data


def send_nav_goal(x, y, label, yaw_deg=0.0):
    """Blocks until Nav2 reports success/failure or GOAL_TIMEOUT is hit.
    Uses the plain `ros2 action send_goal` CLI (synchronous, blocks until
    the action completes) rather than rclpy, since it's already confirmed
    working end-to-end and keeps this script dependency-free.

    NOTE: originally only checked for the literal string "Goal was rejected"
    as a failure - meaning goals that were ACCEPTED but later ABORTED (which
    turned out to be the actual common case - see NOTES.md) were silently
    reported as successes. Now explicitly checks the real result status.
    """
    qz, qw = yaw_deg_to_quat(yaw_deg)
    goal_str = GOAL_TEMPLATE.format(x=x, y=y, qz=qz, qw=qw)
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

    if "status: SUCCEEDED" in result.stdout:
        print(f"[dispatch_bridge] goal SUCCEEDED: {label}")
        return True
    elif "status: ABORTED" in result.stdout:
        print(f"[dispatch_bridge] goal ABORTED (accepted but failed mid-navigation): {label}")
        return False
    elif "status: CANCELED" in result.stdout:
        print(f"[dispatch_bridge] goal CANCELED: {label}")
        return False
    else:
        print(f"[dispatch_bridge] goal ended with unrecognized status, treating as failure: {label}")
        print(f"  raw output tail: {result.stdout.strip()[-300:]}")
        return False


def main():
    handled_timestamps = set()
    handled_search_ids = set()
    patrol_index = 0

    print("=== dispatch_bridge.py running. Watching for dispatch alerts. ===")
    print("Make sure static_transform_publisher, Nav2 bringup, and the "
          "velocity_smoother relay are already running before this does anything useful.")

    try:
        while True:
            # Highest priority: an in-progress face search. Same X/Y as
            # wherever the robot already is, just a new heading to try.
            search_goal = find_face_search_goal(handled_search_ids)
            if search_goal:
                x, y = search_goal["x"], search_goal["y"]
                yaw = search_goal.get("yaw_deg", 0.0)
                label = f"FACE SEARCH: heading {yaw:.0f} deg"
                send_nav_goal(x, y, label, yaw_deg=yaw)
                handled_search_ids.add(search_goal["request_id"])
                continue

            alert = find_next_alert(handled_timestamps)

            if alert:
                raw_x, raw_y = alert["coords"]
                x, y = apply_goal_standoff(raw_x, raw_y)
                label = f"ALERT: {alert.get('reason')} in {alert.get('zone')}"
                if (x, y) != (raw_x, raw_y):
                    print(f"[dispatch_bridge] pulling goal in from wall: "
                          f"({raw_x}, {raw_y}) -> ({x:.2f}, {y:.2f})")
                send_nav_goal(x, y, label)
                # Mark handled regardless of success/failure/timeout - we
                # don't want to get stuck retrying an unreachable point
                # forever. unified_tracking.py separately confirms arrival
                # by watching real position before running the scan.
                handled_timestamps.add(alert["timestamp"])
                continue

            waypoints = load_patrol_waypoints()
            if waypoints:
                raw_x, raw_y = waypoints[patrol_index % len(waypoints)]
                # Same standoff treatment as dispatch alerts above -
                # ROBOT_WAYPOINTS in unified_tracking.py were never run
                # through this, and at least one of them ((-20, 0)) turned
                # out to sit too close to a pillar, causing the robot to
                # wedge against it and jitter in place on its very first
                # patrol leg instead of actually navigating.
                x, y = apply_goal_standoff(raw_x, raw_y)
                if (x, y) != (raw_x, raw_y):
                    print(f"[dispatch_bridge] pulling patrol waypoint in from wall: "
                          f"({raw_x}, {raw_y}) -> ({x:.2f}, {y:.2f})")
                send_nav_goal(x, y, f"patrol waypoint {patrol_index % len(waypoints)}")
                patrol_index += 1
            else:
                print("[dispatch_bridge] no patrol waypoints file found yet, waiting...")
                time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\n=== dispatch_bridge.py stopped by user. ===")


if __name__ == "__main__":
    main()
