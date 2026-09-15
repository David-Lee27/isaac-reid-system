r"""
Unified tracking + movement loop.
People continuously patrol their waypoint loop WHILE the system sweeps all
4 zone cameras + checkpoint camera for detection/re-ID/authorization/fall
detection/loitering, all in one running Isaac Sim instance.

Press Ctrl+C to stop.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\unified_tracking.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\optimized room.usd"  # real filename has a SPACE, not an underscore - confirmed from the Isaac Sim title bar
HEADLESS = False  # you asked to watch it - render/physics decoupling and the muted spam filter both still apply, this just also opens the actual viewport window instead of running invisibly.
# "RTX - Minimal" (visible in the viewport's render-mode dropdown) is the
# actual lightweight RTX preset for this Isaac Sim build - NOT a separate
# "Storm" renderer (that's not even an option in this build's menu; an
# earlier attempt to pass "renderer": "Storm" here was wrong and got
# silently ignored). This just makes explicit, at every launch, the same
# preset your screenshot showed already selected by default - real-time
# RTX with reflections/AO/GI/path-tracing off (matches the settings
# already disabled below), instead of relying on incidental default
# behavior that could change between Isaac Sim versions/machines.
USE_MINIMAL_RENDERER = True
PROJECT_DIR = r"C:\isaacsim\projects\surveillance-proj"
KNOWN_FACES_DIR = PROJECT_DIR + r"\known_faces"
ID_COUNTER_FILE = PROJECT_DIR + r"\next_id.txt"
CAMERA_ZONES_FILE = PROJECT_DIR + r"\camera_zones.json"
AUTHORIZED_IDS_FILE = PROJECT_DIR + r"\authorized_ids.json"
EVENT_LOG_FILE = PROJECT_DIR + r"\event_log.json"
DEBUG_DIR = PROJECT_DIR + r"\debug_captures"
APPEARANCE_PROFILES_FILE = PROJECT_DIR + r"\appearance_profiles.json"
APPEARANCE_MATCH_THRESHOLD = 0.7   # HSV histogram correlation, 0-1, higher = stricter

CHECKPOINT_NAME = "EntryCamera"
CHECKPOINT_PATH = "/World/EntryCamera"
# The new optimized room.usd scene has NO checkpoint/entry camera at all
# (confirmed via diagnose_new_scene_cameras.py - only Camera1-4 exist).
# Calling capture_frame() on a camera prim that doesn't exist would throw
# and crash the whole run the first time a sweep tried to scan it, not
# fail gracefully. CHECKPOINT_AVAILABLE is set for real in main() once the
# stage is loaded (by actually checking whether the prim exists), and
# run_sweep skips the checkpoint scan entirely when it's False. Identity
# checking now happens only via the robot's own orbit search on arrival -
# consistent with the current direction (zone cameras are fall/loitering
# detection only, the robot does the actual person-level investigation).
CHECKPOINT_AVAILABLE = False

SCAN_INTERVAL_SECONDS = 8   # how often to run a full camera sweep
LOITER_THRESHOLD = 3        # consecutive same-zone sightings to flag loitering

# --- People + patrol path ---
PEOPLE = {
    "Person1": {"prim_path": "/World/xbot_person_1", "start_delay": 0},
    "Person2": {"prim_path": "/World/xbot_person_2", "start_delay": 5},
    "Person3": {"prim_path": "/World/xbot_person_3", "start_delay": 10},
}

# Everyone moves the same way now: idle until their assigned scripted
# fall/loiter event (see update_person_position) - the old ACTIVE_MOVER_COUNT
# concept (some people patrol, others frozen at spawn) doesn't apply anymore
# since NOBODY continuously patrols.

WAYPOINTS = [
    (27.46, 16.33, 0),
    (29.06, 27.76, 0),
    (-25.60, 28.53, 0),
    (-28.41, -7.00, 0),
    (25.09, -8.53, 0),
    (26.03, 12.98, 0),
    (29.9, 13.4, 0),
]
SECONDS_PER_LEG = 8  # halved patrol speed (was 4) per request
LOOP_PATROL = True

# When a fall is detected, the nearest patrol character to that zone gets
# frozen in place for this long - used both by the camera-detection-side
# freeze (pause_nearest_mover, a heuristic proxy) and as the lie-down
# duration for the new SCRIPTED fall event below.
FALL_PAUSE_DURATION = 45  # seconds

# --- Randomized scripted fall event ---
# One random person actually, physically collapses at a random time during
# the run (rotated to lie on the ground, then frozen) - a REAL event for the
# camera's aspect-ratio fall heuristic to detect, instead of that heuristic
# only ever firing on incidental noise from a person who's still standing.
FALL_ENABLED = True
FALL_START_RANGE = (8, 20)       # shortened from (15,55) so something visibly happens within the first ~20s of a demo instead of possibly waiting a minute
FALL_ROTATION_OPTIONS = [(90, 0, 0), (-90, 0, 0), (0, 90, 0), (0, -90, 0)]  # random fall direction, degrees

# --- Randomized loitering test ---
# One random person gets assigned a random window during the run where,
# instead of patrolling, they wander in small circles near a random zone -
# a realistic test of the loitering detector instead of manually posing someone.
LOITER_ENABLED = True
LOITER_START_RANGE = (8, 20)       # shortened from (20,50), same reason as FALL_START_RANGE
LOITER_DURATION_RANGE = (25, 40)   # how long they wander before resuming patrol
LOITER_RADIUS = 2.0                # kept for backward compat / reference, no longer drives the wander shape - see the room-box random-waypoint wander below

# Zone camera positions used DIRECTLY as fall/loiter event locations - a
# security camera's actual field of view is centered near where it's
# mounted, so nudging the event location away from that (an earlier
# attempt, done purely to keep the robot's orbit search off the walls)
# was almost certainly why detections stopped happening entirely - the
# events were probably landing outside what the camera could actually see.
# Robot orbit safety near walls/corners is now handled separately by
# clamping orbit standpoints to FLOOR_BOUNDS (see search_for_face) instead
# of moving the event itself.
ZONE_CENTERS = {
    "RoomA_Cam1": (22.60, 27.36),
    "RoomA_Cam2": (39.21, 9.00),
    "RoomB_Cam3": (21.64, 8.91),
    "RoomB_Cam4": (5.83, 27.40),
}
DOORWAY_CENTER = (22, 17.5)  # your directly-observed value (close to the earlier computed 22.1,18.12 - using yours since you watched it directly)

# Room bounding boxes, from corners you gave directly by watching the scene
# (more trustworthy than anything computed from camera position alone) -
# used below for genuine randomized wandering within each room during a
# loitering event, instead of tracing a small fixed circle (which is what
# looked like "just walking the perimeter").
ROOM_BOUNDS = {
    "west": (5.0, 21.5, 9.0, 27.0),   # (xmin, xmax, ymin, ymax) - corners (5,9),(6,27),(21.5,27),(21,9)
    "east": (22.0, 39.0, 9.0, 26.0),  # corners (22,9),(39,9),(39,26), doorway (22,17.5)
}

# Real overall floor bounds (from diagnose_room_bounds.py: x:[4.93,39.93],
# y:[8.29,28.29]), with a small safety margin. Used as a HARD CLAMP - no
# matter what upstream logic (fall_center, loiter wander, robot chase
# tracking) computed a coordinate, nothing can ever actually place a
# person or the robot outside the real building. This exists because a
# fallen character was observed clearly outside the walls with no code
# path in this file that should currently be able to produce that -
# rather than keep guessing at which stale/cached code path did it, this
# makes it structurally impossible regardless of cause.
FLOOR_BOUNDS = (5.5, 39.4, 8.8, 27.8)  # (xmin, xmax, ymin, ymax)


def clamp_to_floor(x, y):
    xmin, xmax, ymin, ymax = FLOOR_BOUNDS
    return (max(xmin, min(xmax, x)), max(ymin, min(ymax, y)))


# The dividing wall between the two rooms is two solid segments (measured
# via diagnose_room_bounds.py: Cube at x:[22.00,22.20] y:[8.12,16.12], and
# Cube_01 at x:[22.00,22.20] y:[20.12,28.12]) with a walkable doorway gap
# between y=16.12 and y=20.12. Since movement here is purely kinematic (we
# teleport prims directly - there's no PhysX simulation driving anything,
# so a collider alone would do nothing; PhysX only pushes back DYNAMIC
# bodies on contact, not something being repositioned by hand every
# frame), this is a real software wall-check using the actual measured
# wall geometry, not a decorative collider.
DIVIDER_X_RANGE = (21.7, 22.4)     # small margin either side of the real 22.00-22.20 wall
DOORWAY_Y_RANGE = (15.8, 20.4)     # small margin either side of the real 16.12-20.12 gap


def blocked_by_divider(x, y):
    """True if (x, y) is inside the solid part of the dividing wall (i.e.
    NOT in the doorway gap)."""
    xmin, xmax = DIVIDER_X_RANGE
    if not (xmin <= x <= xmax):
        return False
    ymin, ymax = DOORWAY_Y_RANGE
    return not (ymin <= y <= ymax)


def move_with_wall_check(cur_x, cur_y, new_x, new_y):
    """Real wall-collision enforcement for kinematic movement: if the
    straight-line step from (cur_x, cur_y) to (new_x, new_y) would end up
    inside the solid dividing wall, the crossing axis (x) gets clamped back
    to whichever side of the wall the mover was already on, instead of
    just letting them phase straight through it - which is exactly what
    was happening when a chase target on the other side of the building
    pulled a straight kinematic line right through the middle wall.
    """
    if not blocked_by_divider(new_x, new_y):
        return new_x, new_y
    xmin, xmax = DIVIDER_X_RANGE
    if cur_x <= xmin:
        return min(new_x, xmin), new_y
    else:
        return max(new_x, xmax), new_y


DOORWAY_WAYPOINT = (22.1, sum(DOORWAY_Y_RANGE) / 2.0)  # center of the actual gap


def get_next_hop(cur_x, cur_y, target_x, target_y):
    """REAL fix for the robot getting stuck pressed against the dividing
    wall: move_with_wall_check correctly BLOCKS illegal crossings, but on
    its own that just leaves the mover stuck at the wall forever if the
    final target is on the other side and the current y isn't already
    inside the doorway band - confirmed exactly in a log where the robot
    sat frozen at x=21.70 for 15+ seconds trying to reach x=39.

    This is real obstacle-avoidance routing, not just collision blocking:
    if the current position and the target are on opposite sides of the
    dividing wall AND the current y isn't already within the doorway gap,
    the immediate movement target becomes the doorway waypoint first -
    once through it, the direct line to the real target no longer needs to
    cross the wall at all.
    """
    dxmin, dxmax = DIVIDER_X_RANGE
    cur_side = "west" if cur_x <= dxmin else ("east" if cur_x >= dxmax else "mid")
    target_side = "west" if target_x < dxmin else ("east" if target_x > dxmax else "mid")
    if cur_side != "mid" and target_side != "mid" and cur_side != target_side:
        ymin, ymax = DOORWAY_Y_RANGE
        if not (ymin <= cur_y <= ymax):
            return DOORWAY_WAYPOINT
    return (target_x, target_y)

# --- Robot dispatch ---
# The robot patrols on its own loop by default. Periodically it checks the
# event log for unhandled dispatch_alert entries (from fall/loitering/
# unauthorized-presence detections). If one exists, it drives to that
# alert's coordinates, runs a checkpoint-style face-ID scan on arrival to
# double-check the person's identity/authorization, then resumes patrol.
ROBOT_PRIM_PATH = "/World/mobile_manipulator_ros"
ROBOT_BASE_PATH = "/World/mobile_manipulator_ros/base_footprint"  # actual physics-driven prim - moving the outer group prim does nothing, PhysX drives this child directly
ROBOT_CAMERA_PATH = "/World/mobile_manipulator_ros/base_footprint/sensors/camera_front/Gemini335L/Gemini335L/camera_rgb/Camera_rgb"
ROBOT_CHECK_INTERVAL = 3.0    # seconds between checking for new dispatch alerts / arrival

# --- Real Nav2 navigation (replaces the old kinematic-teleport robot movement) ---
# The robot now moves via genuine PhysX physics, driven externally by Nav2's
# controller through the confirmed-working velocity_smoother relay bypass
# (see NOTES.md). This script no longer commands robot movement directly -
# a separate WSL-side script (dispatch_bridge.py) watches EVENT_LOG_FILE for
# dispatch_alert entries and PATROL_WAYPOINTS_FILE for the patrol loop, and
# sends real NavigateToPose goals to Nav2. This script's only job re: the
# robot is to (a) write the patrol waypoints out for the bridge to read, and
# (b) watch the robot's REAL position (driven by physics, not by us) to
# detect arrival at a dispatch location and trigger the on-arrival scan.
PATROL_WAYPOINTS_FILE = PROJECT_DIR + r"\robot_patrol_waypoints.json"
ARRIVAL_RADIUS = 3.0          # meters - bumped from 2.0: with continuous live tracking (see update_robot) this is now a real "stop a comfortable distance in front of them" radius, not just a tolerance for a static point

# --- Active face search on arrival (replaces the old single-static-
# snapshot scan) ---
# On arrival, the robot no longer just takes one picture from whatever
# direction it happened to stop facing. It sweeps through a series of
# headings AT THE SAME LOCATION (via small yaw-only Nav2 sub-goals - see
# dispatch_bridge.py's FACE_SEARCH_GOAL_FILE handling), running a full
# YOLO -> crop -> DeepFace attempt at each heading, and stops as soon as
# one attempt gets a usable face. This is what makes the robot actually
# "look for" the person instead of hoping they're in frame by luck.
FACE_SEARCH_GOAL_FILE = PROJECT_DIR + r"\robot_face_search_goal.json"
FACE_SEARCH_YAW_STEPS = 8              # full circle in 45-degree increments
FACE_SEARCH_YAW_TOLERANCE_DEG = 8.0    # how close to the target heading counts as "turned"
FACE_SEARCH_YAW_WAIT_TIMEOUT = 20.0    # seconds to wait for a single turn before giving up on it

# Robot patrols the actual room corners/doorway you gave directly, looping
# west room -> doorway -> east room -> doorway.
ROBOT_WAYPOINTS = [
    (5, 9, 0),
    (6, 27, 0),
    (21.5, 27, 0),
    (22, 17.5, 0),   # doorway
    (21, 9, 0),
    (22, 9, 0),
    (39, 9, 0),
    (39, 26, 0),
    (22, 17.5, 0),   # doorway, loop back
]

# --- SIMPLIFIED KINEMATIC ROBOT MOVEMENT (replaces real Nav2/PhysX driving) ---
# Real wheel physics through Nav2 turned out not to be worth the cost for
# this project's actual goal: the robot reliably reaching alert locations
# so its camera can run a face search, not accurate differential-drive
# kinematics. Diagnosed (not guessed) that the wheel geometry itself was
# fine (measured wheelDistance matched the authored 0.34m exactly) - the
# real problem was the whole Nav2/PhysX/ROS2 stack being fragile, slow to
# debug, and sensitive to render-rate/render-event assumptions that kept
# eating entire sessions. The robot now moves exactly like the patrol
# people already do (see update_person_position) - direct kinematic
# translation toward a target each tick, no physics, no wheels, no Nav2,
# no WSL dependency for movement at all. Nav2/dispatch_bridge.py can still
# run harmlessly alongside this (it's just not doing anything useful for
# the robot anymore) or you can stop bothering with start_nav_stack.sh
# entirely - movement no longer needs it.
ROBOT_MOVE_SPEED = 3.0  # halved (was 6.0) per request

# NOTE (known limitation, documented rather than solved - see NOTES.md):
# wheel_left/wheel_right/camera_hand_link are SIBLINGS of base_footprint
# (not children), not connected to it by a physics joint. In the old
# kinematic-teleport design we moved them manually every frame to fake
# attachment. In real-physics Nav2 mode, base_footprint now moves under
# genuine PhysX + the robot's own ROS2 diff-drive plugin (confirmed working
# in the Nav2 relay test), but these sibling parts have no joint tying them
# to it, so they may visually lag/stay behind during navigation. Cosmetic
# only - it doesn't affect navigation, detection, or the checkpoint-style
# scan, since ROBOT_CAMERA_PATH is the properly-nested camera under
# base_footprint/sensors/ and moves correctly with it automatically.

# Isaac Sim's ROS2 bridge extension used to be needed here for the old
# Nav2/PhysX-driven robot. That path was fully abandoned in favor of pure
# kinematic movement (see ROBOT_MOVE_SPEED's comment history) and the heavy
# mobile_manipulator_ros asset itself was replaced with a lightweight
# custom robot (see create_lightweight_robot) - ROS2 isn't driving
# anything anymore, so enabling that whole extension was pure unnecessary
# startup/runtime cost. Removed rather than left harmlessly enabled.
import os

from isaacsim import SimulationApp

# All of the settings below are passed directly into SimulationApp's launch
# config instead of being set afterward via carb.settings.set(). This is a
# deliberate change: post-hoc carb.settings calls (even moved as early as
# possible in the script, even before enabling the ROS2 bridge extension)
# were confirmed NOT to suppress the PoseTree log spam or fix the
# "sequence size exceeds remaining buffer" issue across two separate
# attempts. Extension logging channels and some renderer internals are
# locked in at Kit's actual process startup - passing these as launch
# config is the earliest possible point, before any extension registers,
# and is far more likely to actually take effect.
simulation_app = SimulationApp({
    "headless": HEADLESS,
    # Dropped from 640x480 - this is the internal offscreen render surface
    # size, paid on every rendered tick regardless of headless mode. You
    # never see this buffer directly in headless mode, so there's no
    # visual-quality reason to keep it large - only the actual camera
    # capture resolutions (set separately per-camera in capture_frame()
    # calls) matter for what YOLO/DeepFace actually see.
    "width": 854,
    "height": 480,
    "/log/level": "Error",
    "/log/fileLogLevel": "Error",
    "/log/outputStreamLevel": "Error",
    "/rtx/scenedb/maxHistoryTransformCount": 2048,  # bumped from 512 - that value stopped the spam in an earlier session but it's back in force (see run this session, heavy "sequence size exceeds remaining buffer" volume even before any camera/sweep activity), so whatever's generating transform-history entries now needs more headroom than 512 gave it
    "/rtx/pathtracing/enabled": False,
    "/rtx/reflections/enabled": False,
    "/rtx/ambientOcclusion/enabled": False,
    "/rtx/indirectDiffuseGI/enabled": False,
    "/rtx/directLighting/sampledLighting/enabled": False,
    "/rtx/post/dlss/execMode": 0,
    # Real, concrete render-cost cuts on top of the above - shadows and
    # antialiasing are both real per-frame GPU cost with zero benefit in
    # headless mode where nothing is being visually watched, and neither
    # affects what a captured camera frame looks like to YOLO/DeepFace
    # (object detection/face-matching don't care about shadow softness or
    # jagged edges).
    "/rtx/shadows/enabled": False,
    "/rtx/post/aa/op": 0,
    "/app/runLoops/main/rateLimitEnabled": False,
})

import carb
# Per-channel log suppression via flat "/log/channels/<name>" = "<level>"
# string entries - this IS Kit's real schema (confirmed by the previous
# attempt's error: setting "/level"/"/enabled" sub-keys under each channel
# turned that entry into a nested dict, and Kit's own startup code, which
# expects to read each channel entry as a plain string, errored on every
# single one - "getStringRawInternal: item <channel> is not a string" -
# which broke far more than it fixed). No sub-keys this time.
for _channel in ("isaacsim.ros2.nodes", "isaacsim.ros2.bridge", "isaacsim.ros2.core",
                  "omni.hydra", "rtx.hydra", "omni.syntheticdata.plugin",
                  "isaacsim.sensors.camera.camera", "rtx.scenedb.plugin",
                  "isaacsim.ros2.core.impl.camera_info_utils", "omni.timeline.plugin",
                  "carb"):
    try:
        # Reverted back to "Error" (from an attempted "Fatal" bump) - that
        # bump did NOT fix the "sequence size exceeds remaining buffer"
        # spam (confirming it isn't going through carb's channel logging at
        # all - likely a raw fprintf from a native plugin, which no log-
        # level setting can touch), while ALSO suppressing other genuinely
        # useful console output. The actual fix for that specific message
        # is run_sim_filtered.bat, which filters the real text stream
        # instead of fighting the logging system further.
        carb.settings.get_settings().set(f"/log/channels/{_channel}", "Error")
    except Exception:
        pass

# ROS2 bridge extension enabling removed - no longer needed, see the
# comment above `import os` for why.

# EXPERIMENTAL: attempt to force the viewport's "RTX - Minimal" render mode
# (confirmed present/selectable in the viewport dropdown, not guaranteed to
# be settable via this exact key). Post-hoc carb.settings calls were
# already confirmed NOT to reliably affect renderer internals earlier in
# this project (see the log-channel-suppression comment above), so this is
# a low-risk try: if the key/value is wrong it should just be silently
# ignored (same as the earlier "Storm" attempt was), not break startup.
# If the dropdown doesn't show Minimal selected after launch, this didn't
# work - just click it manually in the dropdown, which is the confirmed-
# working zero-risk fallback.
if USE_MINIMAL_RENDERER:
    try:
        carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLighting")
        carb.settings.get_settings().set("/rtx-transient/dlssg/enabled", False)
        print("Attempted to force RTX-Minimal render mode (experimental - verify in the viewport dropdown).")
    except Exception as e:
        print(f"RTX-Minimal render mode attempt failed harmlessly: {e}")

import omni.usd
import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdSkel
import cv2
import numpy as np
import subprocess
import json
import shutil
import time
import random
import math
from datetime import datetime, timezone
from isaacsim.sensors.camera import Camera
from isaacsim.core.prims import RigidPrim

# THE actual max-speed lever: SimulationContext.step(render=...) lets us
# step physics/OmniGraph (ROS2 publishers/subscribers, TF, odom, /cmd_vel
# consumption - all of it) every tick while only paying for a full Hydra
# render on a fraction of those ticks. Rendering, not physics, is what was
# capping real-time factor at ~12 updates/sec even with ZERO cameras
# active (confirmed via the [PERF] log - that ceiling existed before the
# first camera was ever created) - simulation_app.update() couples a full
# render to every single tick with no way to separate them. This is the
# standard, documented Isaac Sim pattern for headless/bulk-physics speed;
# not a guess, but also not yet verified on THIS build/robot asset, so it's
# wrapped in the same fail-safe try/except pattern as every other
# uncertain API call in this file - if SimulationContext or .step(render=)
# doesn't behave as expected here, this falls back to the old always-render
# simulation_app.update() behavior rather than silently breaking physics,
# ROS2, or movement.
try:
    from isaacsim.core.api import SimulationContext
except ImportError:
    from isaacsim.core.simulation_context import SimulationContext

# Render only 1 in this many ticks when nothing needs a guaranteed real
# frame this tick (see _tick()'s force_render param, used by capture_frame
# to always get a real frame when actually capturing). Physics/OmniGraph/
# ROS2 still step on EVERY tick regardless - only the expensive Hydra
# render pass gets skipped on the other (RENDER_EVERY_N_TICKS - 1) ticks.
#
# Tried 20: steps/sec jumped to 170-290/sec, but the robot never moved at
# all that run - turned out the WSL nav stack (start_nav_stack.sh) wasn't
# even running that time, so there were no Nav2 goals being sent at ANY
# render setting - that test didn't actually tell us anything about this
# number, my earlier comment claiming it as "confirmed" broken was wrong.
# Back at 6 (previously confirmed moving the robot end to end WITH the nav
# stack running) as the safe known-good value. Real test of going higher
# than 6 has to be run with start_nav_stack.sh actually up in WSL first -
# check that before touching this number again.
RENDER_EVERY_N_TICKS = 6
_sim_context = None
_render_decouple_supported = True


# ---------- Movement helpers ----------

def repair_broken_tf_publisher_targets(stage):
    """The robot asset has long-known broken TF frames (base_link,
    lidar_frame, etc. reporting '[PoseTree] getObjectType eInvalid' every
    tick - see NOTES.md). This was previously treated as harmless log noise,
    but it likely also means Nav2's obstacle-avoidance costmap can't
    correctly place lidar points (it needs a working transform from each
    scan's frame to the costmap's frame) - consistent with the robot
    driving straight into a pillar despite genuine navigation working.

    Rather than hand-guessing which exact prim paths are broken and what
    they should be instead (risky and asset-specific), this walks every
    USD relationship stage-wide looking for 'target'/'parent'-type
    relationships (how OmniGraph nodes like the ROS2 TF-tree publisher
    store their prim references under the hood) whose targets don't
    resolve to a valid prim, and tries to repair each one by finding a
    valid prim elsewhere in the robot's hierarchy with the exact same leaf
    name. Self-healing where the match is unambiguous; loudly prints
    exactly what it could and couldn't fix otherwise, rather than silently
    guessing.
    """
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        print("  [tf_repair] robot prim not found, skipping.")
        return

    name_to_paths = {}
    for prim in Usd.PrimRange(robot_prim):
        name_to_paths.setdefault(prim.GetName(), []).append(prim.GetPath())

    checked = 0
    repaired = 0
    unresolved = []

    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        for rel in prim.GetRelationships():
            rel_name = rel.GetName()
            if "target" not in rel_name.lower() and "parent" not in rel_name.lower():
                continue
            targets = rel.GetTargets()
            if not targets:
                continue
            new_targets = []
            changed = False
            for t in targets:
                checked += 1
                t_prim = stage.GetPrimAtPath(t)
                if t_prim.IsValid():
                    new_targets.append(t)
                    continue
                leaf = t.name
                candidates = name_to_paths.get(leaf, [])
                if len(candidates) == 1:
                    fixed = candidates[0]
                    print(f"  [tf_repair] {prim.GetPath()}.{rel_name}: {t} -> {fixed}")
                    new_targets.append(fixed)
                    changed = True
                    repaired += 1
                else:
                    unresolved.append((str(prim.GetPath()), rel_name, str(t), len(candidates)))
                    new_targets.append(t)
            if changed:
                try:
                    rel.SetTargets(new_targets)
                except Exception as e:
                    print(f"  [tf_repair] failed to apply fix on {prim.GetPath()}.{rel_name}: {e}")

    print(f"  [tf_repair] checked {checked} relationship target(s) stage-wide, repaired {repaired}.")
    if unresolved:
        print(f"  [tf_repair] {len(unresolved)} target(s) NOT auto-repaired (ambiguous or no same-name match):")
        for node_path, rel_name, t_str, n in unresolved[:15]:
            print(f"    {node_path}.{rel_name}: {t_str} (found {n} same-name candidates)")


def get_animation_loop_seconds(stage, default=1.0):
    """Scans all UsdSkel.Animation prims stage-wide for the furthest real
    time sample (rotation/translation) and converts it from timeCodes to
    seconds using the stage's own timeCodesPerSecond. Used to tell the
    timeline exactly where the baked Mixamo walk clip ends, so looping
    wraps back to frame 0 right as the clip finishes instead of guessing
    an fps/duration and holding on the last frame or looping mid-stride.
    """
    from pxr import UsdSkel as _UsdSkel
    max_time_code = 0.0
    found_any = False
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(_UsdSkel.Animation):
            continue
        anim = _UsdSkel.Animation(prim)
        for attr in (anim.GetRotationsAttr(), anim.GetTranslationsAttr()):
            if not attr:
                continue
            samples = attr.GetTimeSamples()
            if samples:
                found_any = True
                max_time_code = max(max_time_code, samples[-1])
    if not found_any or max_time_code <= 0:
        return default
    tcps = stage.GetTimeCodesPerSecond() or 24.0
    return max_time_code / tcps


def get_translate(prim):
    # NOTE: this used to just look for a literal `translate` xformOp and
    # return (0,0,0) if none was found - fine while the robot was
    # kinematic-teleported (we created that op ourselves), but PhysX writes
    # physics-driven position updates through a different representation
    # (a combined transform matrix), so that approach silently always
    # returned (0,0,0) for the real-physics robot and update_robot() never
    # detected arrival no matter how close it actually got. Computing the
    # full local-to-world transform works regardless of how the underlying
    # ops are structured.
    xform = UsdGeom.Xformable(prim)
    matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return matrix.ExtractTranslation()


def set_translate(prim, pos):
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
            return
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))


def set_world_translate(prim, world_pos):
    """Sets a prim's position given a WORLD-space target, correctly
    accounting for its parent's transform - unlike set_translate() (which
    writes the prim's LOCAL translate op directly), this actually converts
    world_pos into the right local value first via the parent's inverse
    world transform.

    Needed specifically for the robot: get_translate() reads WORLD
    position (via ComputeLocalToWorldTransform), but the patrol people's
    set_translate() calls write LOCAL translate directly - harmless for
    them because their prims sit right under /World with an identity
    parent transform, so local == world. The robot's parent prim
    (/World/mobile_manipulator_ros) does NOT have an identity transform
    (almost certainly a scale factor from the URDF/USD import) - writing a
    world-sized delta straight into local space under a scaled parent
    caused the position to blow up exponentially every tick (confirmed:
    the robot flew off to tens of thousands of meters away, accelerating).
    This is the real fix - always use this for the robot, never raw
    set_translate().
    """
    parent = prim.GetParent()
    parent_xform = UsdGeom.Xformable(parent)
    parent_matrix = parent_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    local_pos = parent_matrix.GetInverse().Transform(Gf.Vec3d(*world_pos))
    set_translate(prim, local_pos)


def set_rotation(prim, degrees_xyz):
    """Sets (or creates) a rotateXYZ xformOp on this prim, in degrees.
    Used to physically collapse a character for the scripted fall event -
    rotating the whole standing character 90 degrees about a horizontal
    axis makes their bounding box genuinely go wide/short (matching what
    check_for_fall()'s aspect-ratio heuristic is actually looking for),
    instead of that heuristic only ever firing on incidental noise."""
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            op.Set(Gf.Vec3f(*degrees_xyz))
            return
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*degrees_xyz))


def get_skeleton_and_clips(model_prim):
    """Finds the UsdSkel.Skeleton under this character's model prim, plus
    the sibling walk/fall/held-fall SkelAnimation clip paths next to it, so
    the fall event can retarget animationSource between them at runtime
    instead of faking a collapse via crude whole-body rotation (see the old
    set_rotation-based fall, replaced below - that just rigidly rotated a
    mid-stride walk pose, which looked exactly as wrong as it sounds).

    Also computes the fall clip's own real duration in seconds (from its
    last authored time sample / this stage's timeCodesPerSecond), so
    update_person_position can switch to the static held-pose clip right
    when the fall animation finishes playing - without this, the whole
    scene's shared global timeline loop (much shorter than the ~45s
    lie-down pause) would keep wrapping back to frame 0 and replaying the
    collapse from scratch every few seconds during the pause.
    """
    if not model_prim.IsValid():
        return None, None, None, None, 2.0
    stage = model_prim.GetStage()
    tcps = stage.GetTimeCodesPerSecond() or 24.0
    for prim in Usd.PrimRange(model_prim):
        if prim.IsA(UsdSkel.Skeleton):
            skel_root = prim.GetParent()
            walk_path = skel_root.GetPath().AppendChild("mixamo_com")
            fall_path = skel_root.GetPath().AppendChild("mixamo_fall")
            held_path = skel_root.GetPath().AppendChild("mixamo_fall_held")
            fall_duration = 2.0
            fall_prim = stage.GetPrimAtPath(fall_path)
            if fall_prim.IsValid():
                fall_anim = UsdSkel.Animation(fall_prim)
                rot_attr = fall_anim.GetRotationsAttr()
                samples = rot_attr.GetTimeSamples() if rot_attr else []
                if samples:
                    fall_duration = samples[-1] / tcps
            return prim, walk_path, fall_path, held_path, fall_duration
    return None, None, None, None, 2.0


def set_animation_clip(skel_prim, clip_path):
    """Retargets a Skeleton's animationSource relationship to a different
    SkelAnimation clip prim (walk <-> fall). Both clips already live side
    by side under the same SkelRoot (see bind_fall_animation.py) - this
    just repoints which one is currently bound/playing."""
    if skel_prim is None or not skel_prim.IsValid():
        return
    binding = UsdSkel.BindingAPI(skel_prim)
    rel = binding.GetAnimationSourceRel()
    if not rel:
        rel = binding.CreateAnimationSourceRel()
    rel.SetTargets([clip_path])


def disable_physics_recursive(prim):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(prim)
        attr = rigid_api.GetRigidBodyEnabledAttr()
        if attr:
            attr.Set(False)
        else:
            rigid_api.CreateRigidBodyEnabledAttr(False)
    for child in prim.GetChildren():
        disable_physics_recursive(child)


# Module-level state for _tick(), the single shared per-frame update function.
# Populated once in main() right before the main loop starts. Needed because
# _tick() must be callable from deep inside blocking helpers (run_subprocess,
# capture_frame) that don't otherwise have access to movers/robot_state/etc,
# without threading those as parameters through every call site.
_tick_state = {
    "sim_start": None,
    "movers": [],
    "robot_state": None,
    "authorized_ids_cache": [],
    # FPS/real-time-factor instrumentation - see _tick()'s comment below.
    "tick_count": 0,
    "tick_window_start": None,
}


def _tick(force_render=False):
    """The ONE place per-frame work happens: advances mover positions,
    checks robot arrival, and steps the sim. Every single simulation step
    anywhere in this script - whether in the main loop or buried inside a
    blocking wait in run_subprocess()/capture_frame() - goes through this
    function instead, so movement never silently freezes during the
    many-second blocking calls a sweep makes (YOLO/DeepFace subprocesses,
    camera render waits). See run_subprocess() for the full story on why
    this was necessary (the "teleporting" bug).

    force_render=True guarantees a real Hydra render this tick (used by
    capture_frame(), which actually needs a frame back) - every other
    caller lets the render/no-render decision fall through to the
    RENDER_EVERY_N_TICKS throttle below, since physics/robot-arrival-
    checking/ROS2 don't need a picture, just a physics step.
    """
    global _render_decouple_supported
    if _tick_state["sim_start"] is None:
        simulation_app.update()
        return
    elapsed = time.time() - _tick_state["sim_start"]
    for m in _tick_state["movers"]:
        update_person_position(m, elapsed)
    if _tick_state["robot_state"] is not None:
        update_robot(_tick_state["robot_state"], elapsed, _tick_state["authorized_ids_cache"])

    do_render = force_render or (_tick_state["tick_count"] % RENDER_EVERY_N_TICKS == 0)
    if _render_decouple_supported and _sim_context is not None:
        try:
            _sim_context.step(render=do_render)
        except Exception as e:
            _render_decouple_supported = False
            print(f"  [WARN] SimulationContext.step(render=) failed ({e}) - "
                  f"falling back to always-render simulation_app.update() for the rest of this run.")
            simulation_app.update()
    else:
        simulation_app.update()

    # FPS/real-time-factor instrumentation - diagnosing whether robot
    # "slowness" is actually a rendering-bottleneck problem (few real
    # sim steps per real second, meaning little simulated time elapses per
    # real second regardless of how fast Nav2/the differential controller
    # are CONFIGURED to move things) rather than a navigation config
    # problem - every navigation speed knob tuned so far had zero
    # measurable effect on real-world robot speed, which is what this is
    # checking for. Now also reports how many of those ticks actually
    # rendered, so it's obvious whether the render/physics decoupling
    # above is doing anything.
    _tick_state["tick_count"] += 1
    if do_render:
        _tick_state["render_count"] = _tick_state.get("render_count", 0) + 1
    if _tick_state["tick_window_start"] is None:
        _tick_state["tick_window_start"] = time.time()
    window_elapsed = time.time() - _tick_state["tick_window_start"]
    if window_elapsed >= 5.0:
        fps = _tick_state["tick_count"] / window_elapsed
        render_fps = _tick_state.get("render_count", 0) / window_elapsed
        print(f"  [PERF] {_tick_state['tick_count']} sim steps ({_tick_state.get('render_count', 0)} rendered) "
              f"in {window_elapsed:.1f}s real time = {fps:.1f} steps/sec, {render_fps:.1f} rendered/sec")
        _tick_state["tick_count"] = 0
        _tick_state["render_count"] = 0
        _tick_state["tick_window_start"] = time.time()


def update_person_position(mover, elapsed):
    """SIMPLIFIED per explicit direction: people no longer continuously
    patrol a waypoint loop - that was real, unnecessary movement/animation
    cost for characters that mostly just need to stand there until a
    scripted event happens to them. Default behavior now is simply IDLE
    (do nothing, stay exactly where they are) unless they're the one
    person currently fallen or loitering.

    Also: invisible by default, only made visible for the duration of
    their own fall/loiter event (see MakeVisible/MakeInvisible calls
    below) - there's no reason for an idle bystander nobody's testing
    right now to be rendered at all, and it makes clear exactly who/when
    a camera is supposed to be able to see anything. Bound to the HELD
    (static) pose by default rather than the walk clip too - previously
    they'd stand there endlessly playing a walk-cycle animation in place
    since nothing ever explicitly stopped it while idle.
    """
    # Stand back up once a scripted fall's lie-down duration has elapsed -
    # switch the skeleton back to the walk clip.
    if mover.get("fallen") and elapsed >= mover.get("paused_until", 0):
        set_animation_clip(mover["skel_prim"], mover["held_clip_path"])
        mover["fallen"] = False
        mover["fall_held_triggered"] = False
        # Restore standing height (see fall-trigger below for why it was
        # dropped).
        pos = get_translate(mover["prim"])
        set_translate(mover["prim"], (pos[0], pos[1], mover.get("base_height", pos[2])))
        UsdGeom.Imageable(mover["prim"]).MakeInvisible()

    # Once the fall ANIMATION itself has finished playing (its own short
    # duration, typically a couple seconds - much shorter than the whole
    # lie-down pause), switch to the static held-pose clip. Without this,
    # the scene's one shared global timeline loop (a few seconds long, set
    # by whichever clip is longest) keeps wrapping back to frame 0 and
    # replaying the fall from standing pose over and over for the entire
    # ~45s pause - this holds the final collapsed pose steady instead.
    if (mover.get("fallen") and not mover.get("fall_held_triggered")
            and mover.get("fall_start") is not None
            and elapsed >= mover["fall_start"] + mover.get("fall_clip_duration", 2.0)):
        set_animation_clip(mover["skel_prim"], mover["held_clip_path"])
        mover["fall_held_triggered"] = True

    # Trigger this mover's scripted fall once, at their assigned time - a
    # REAL physical collapse at wherever they currently are (patrol or
    # loiter), not a heuristic guess. Fires once via fall_triggered.
    #
    # Real fall ANIMATION now (see bind_fall_animation.py) instead of
    # rigidly rotating the whole model 90 degrees around a mid-stride walk
    # pose (which looked exactly as broken as it sounds - a contorted,
    # clearly-not-actually-fallen mess). Movement stays frozen for the
    # whole FALL_PAUSE_DURATION exactly as before (see the paused_until
    # check below). Root translation on the fall clip itself is zeroed
    # (see fix_fall_translation_and_hold.py - the fall clip's raw
    # translation used a totally different unit/axis convention than the
    # walk clip and caused floating), so only the fall's ROTATIONS drive
    # the pose; the character's position stays exactly where they were
    # standing when they collapsed.
    fall_start = mover.get("fall_start")
    if (fall_start is not None and not mover.get("fall_triggered")
            and elapsed >= fall_start):
        mover["fall_triggered"] = True
        mover["fallen"] = True
        mover["fall_held_triggered"] = False
        mover["paused_until"] = elapsed + FALL_PAUSE_DURATION
        set_animation_clip(mover["skel_prim"], mover["fall_clip_path"])
        UsdGeom.Imageable(mover["prim"]).MakeVisible()
        # Teleport to the assigned fall_center FIRST (see main()'s setup) -
        # with no more patrol walking, the person's "current position"
        # would otherwise just be their unmoving spawn point, generally not
        # inside any camera's view. This guarantees the fall actually
        # happens somewhere detectable, exactly like loiter_center already
        # does for the loitering test below.
        fx, fy = mover.get("fall_center", (None, None))
        if fx is not None:
            fx, fy = clamp_to_floor(fx, fy)
        # The fall clip's own root translation is intentionally zeroed (see
        # fix_fall_translation_and_hold.py - it used a totally different
        # unit/axis convention than the walk clip and caused floating/wrong
        # scale), so the character's root stays at STANDING hip height even
        # though the pose itself rotates to lie flat - that's what was
        # causing the "floating" look. Drop the outer position down to
        # compensate, roughly to ground level, restored on stand-up above.
        FALL_HEIGHT_DROP = 0.75
        base_z = mover.get("base_height", 0.0)
        if fx is not None:
            set_translate(mover["prim"], (fx, fy, base_z - FALL_HEIGHT_DROP))
        else:
            pos = get_translate(mover["prim"])
            set_translate(mover["prim"], (pos[0], pos[1], base_z - FALL_HEIGHT_DROP))
        print(f"*** {mover['name']} has fallen (scripted event) - lying down for {FALL_PAUSE_DURATION}s ***")

    # Frozen in place (scripted fall above, or a camera-detection-triggered
    # pause via pause_nearest_mover) - skip all movement, patrol and loiter
    # alike, until the pause expires.
    if elapsed < mover.get("paused_until", 0):
        return

    # If this mover is in their assigned loiter window, wander RANDOMLY
    # within the actual room box they're in (see ROOM_BOUNDS) - picking a
    # new random point inside the room and walking to it, repeatedly - not
    # tracing a small fixed circle around one spot, which is what looked
    # like "just walking the perimeter."
    loiter_start = mover.get("loiter_start")
    if loiter_start is not None:
        loiter_end = loiter_start + mover["loiter_duration"]
        if loiter_start <= elapsed <= loiter_end:
            if not mover.get("loiter_was_active"):
                mover["loiter_was_active"] = True
                UsdGeom.Imageable(mover["prim"]).MakeVisible()
                set_animation_clip(mover["skel_prim"], mover["walk_clip_path"])
                cx, cy = mover["loiter_center"]
                mover["loiter_bounds"] = (ROOM_BOUNDS["west"] if cx < 21.75 else ROOM_BOUNDS["east"])
                mover["loiter_waypoint"] = None
                mover["last_loiter_move_time"] = elapsed

            xmin, xmax, ymin, ymax = mover["loiter_bounds"]
            if mover.get("loiter_waypoint") is None:
                # 1m margin so wander targets don't sit literally against a wall
                mover["loiter_waypoint"] = (random.uniform(xmin + 1, xmax - 1), random.uniform(ymin + 1, ymax - 1))

            pos = get_translate(mover["prim"])
            tx, ty = mover["loiter_waypoint"]
            dx, dy = tx - pos[0], ty - pos[1]
            dist = math.hypot(dx, dy)
            dt = elapsed - mover.get("last_loiter_move_time", elapsed)
            mover["last_loiter_move_time"] = elapsed
            LOITER_WALK_SPEED = 1.0  # m/s - an unhurried wandering pace, not a purposeful walk

            if dist < 0.3:
                mover["loiter_waypoint"] = None  # arrived - a new random point gets picked next tick
            else:
                step = min(LOITER_WALK_SPEED * dt, dist)
                raw_x = pos[0] + dx / dist * step
                raw_y = pos[1] + dy / dist * step
                nx, ny = move_with_wall_check(pos[0], pos[1], raw_x, raw_y)
                yaw_deg = math.degrees(math.atan2(dx, -dy))
                set_rotation(mover["prim"], (0.0, 0.0, yaw_deg))
                set_translate(mover["prim"], (nx, ny, mover.get("base_height", 0.0)))
            return
        elif mover.get("loiter_was_active"):
            # Loiter window just ended - go back to idle/invisible, same
            # as after a fall recovers.
            mover["loiter_was_active"] = False
            mover["loiter_waypoint"] = None
            set_animation_clip(mover["skel_prim"], mover["held_clip_path"])
            UsdGeom.Imageable(mover["prim"]).MakeInvisible()

    # Otherwise: idle. No more continuous patrol walking - see this
    # function's docstring for why. They simply stay exactly where they
    # are (wherever their last event, or spawn, left them) until the next
    # scripted fall/loiter event moves them.


def get_nearest_mover_coords(movers, zone_name):
    """Returns the actual live (x, y) of whichever mover is currently
    closest to the given zone - used so the robot is dispatched to where
    someone actually is right now, not a fixed zone-camera coordinate.
    Simplified: used to restrict this to "active" (patrolling) movers only,
    back when some people patrolled and others sat frozen at spawn - now
    that NOBODY continuously patrols (everyone's idle until their own
    scripted fall/loiter event, which explicitly teleports them into a
    zone - see fall_center/loiter_center), every mover is equally valid to
    consider.
    """
    zone_pos = ZONE_CENTERS.get(zone_name)
    if zone_pos is None or not movers:
        return zone_pos
    nearest = min(movers, key=lambda m: (
        (get_translate(m["prim"])[0] - zone_pos[0]) ** 2 +
        (get_translate(m["prim"])[1] - zone_pos[1]) ** 2
    ))
    npos = get_translate(nearest["prim"])
    return clamp_to_floor(npos[0], npos[1])


def pause_nearest_mover(movers, zone_name, elapsed):
    """Freezes whichever patrol character is currently closest to the
    given zone's coordinates, for FALL_PAUSE_DURATION seconds - simple
    stand-in for 'the person who fell stays down' until real fall
    animation exists."""
    zone_pos = ZONE_CENTERS.get(zone_name)
    if zone_pos is None or not movers:
        return

    nearest = min(movers, key=lambda m: (
        (get_translate(m["prim"])[0] - zone_pos[0]) ** 2 +
        (get_translate(m["prim"])[1] - zone_pos[1]) ** 2
    ))
    nearest["paused_until"] = elapsed + FALL_PAUSE_DURATION
    print(f"  [{zone_name}] freezing {nearest['name']} in place for {FALL_PAUSE_DURATION}s (simulating staying down)")


def get_robot_yaw_degrees(rigid_prim):
    """Current robot heading, extracted from its live physics orientation.

    NOTE (one real unverified assumption): assumes Isaac's
    RigidPrim.get_world_poses() returns orientation quaternions in
    (w, x, y, z) order - the standard Isaac Sim/pxr convention, but not
    independently confirmed on THIS robot asset here. If the search loop
    below seems to wait forever / never detects the robot as having turned
    to the requested heading, this ordering is the first thing to check
    (try swapping to (x, y, z, w) and see if it starts working).
    """
    _, orientations = rigid_prim.get_world_poses()
    w, x, y, z = orientations[0]
    yaw_rad = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(yaw_rad)


def request_robot_search_heading(x, y, yaw_deg, request_id):
    """UNUSED now that the robot moves kinematically (see search_for_face
    below, which just sets rotation directly) - kept only because
    dispatch_bridge.py on the WSL side still references this file if
    anyone's still running the old nav stack alongside this. Safe to
    ignore/delete once you've confirmed you don't need that path anymore.
    """
    with open(FACE_SEARCH_GOAL_FILE, "w") as f:
        json.dump({"x": x, "y": y, "yaw_deg": yaw_deg, "request_id": request_id}, f)


def wait_for_robot_yaw(rigid_prim, target_yaw_deg, timeout):
    """Polls the robot's real heading (via _tick(), so physics/rendering
    keep stepping the whole time - same pattern as run_subprocess's wait)
    until it's within FACE_SEARCH_YAW_TOLERANCE_DEG of the target, or the
    timeout is hit. Returns whether it actually got there - if not, the
    caller scans anyway rather than getting stuck forever on one heading
    Nav2 can't quite reach (e.g. an obstacle preventing the exact turn).
    """
    start = time.time()
    while time.time() - start < timeout:
        _tick()
        time.sleep(0.05)
        current_yaw = get_robot_yaw_degrees(rigid_prim)
        diff = abs(((current_yaw - target_yaw_deg + 180) % 360) - 180)
        if diff <= FACE_SEARCH_YAW_TOLERANCE_DEG:
            return True
    return False


def find_next_unhandled_alert(handled_timestamps):
    """Reads the event log for the oldest dispatch_alert that has real
    coordinates and hasn't already been handled by the robot this run."""
    if not os.path.exists(EVENT_LOG_FILE):
        return None
    with open(EVENT_LOG_FILE, "r") as f:
        try:
            log = json.load(f)
        except json.JSONDecodeError:
            return None
    for e in log:
        if (e.get("event_type") == "dispatch_alert"
                and e.get("coords")
                and e.get("timestamp") not in handled_timestamps):
            return e
    return None


def attempt_robot_scan(camera_path, authorized_ids, alert, heading_label=""):
    """One scan attempt from wherever the robot is currently facing -
    YOLO -> crop -> DeepFace -> authorization, same pipeline as the fixed
    checkpoint camera. Returns True only on a CONFIRMED identification
    (face matched an existing person, or a new person was registered) -
    that's the real success signal the search loop below uses to know
    when to stop turning and try more headings. "Person visible but no
    clear face" and "nobody in view" both return False so the search
    keeps going.
    """
    label = f" ({heading_label})" if heading_label else ""
    print(f"  [ROBOT] scanning{label}...")
    bgr = capture_frame(camera_path, resolution=(480, 360))
    if bgr is None:
        print("  [ROBOT] no frame captured")
        return False

    # Save the FULL raw frame (not just the eventual person crop below) for
    # every single scan attempt, labeled by standpoint - this is the actual
    # "let me see what the robot sees" fix. Whatever's going wrong with the
    # orbit not landing a face (person too small/far, wrong angle, face
    # occluded, standpoint math off) should be directly visible by paging
    # through these in order after a run, in
    # C:\isaacsim\projects\surveillance-proj\debug_captures\.
    safe_label = (heading_label or "noheading").replace(" ", "_").replace("/", "-")
    raw_debug_name = f"{datetime.now().strftime('%H%M%S')}_robot_RAWFRAME_{safe_label}.jpg"
    cv2.imwrite(os.path.join(DEBUG_DIR, raw_debug_name), bgr)

    frame_path = os.path.join(os.environ["TEMP"], "robot_scan.jpg")
    cv2.imwrite(frame_path, bgr)
    yolo_result = _workers["yolo"].call(frame_path)
    detections = yolo_result.get("detections", []) if not yolo_result.get("error") else []
    if not detections:
        print(f"  [ROBOT] nobody in view{label}")
        return False

    best_detection = max(detections, key=lambda d: d["confidence"])
    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return False

    crop_path = os.path.join(os.environ["TEMP"], "robot_crop.jpg")
    cv2.imwrite(crop_path, crop)
    debug_name = f"{datetime.now().strftime('%H%M%S')}_robot_crop.jpg"
    cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    reid_result = _workers["deepface"].call(f"{crop_path}\t{KNOWN_FACES_DIR}")
    status = reid_result.get("status")
    person_id = None

    if status == "match":
        matched_filename = os.path.basename(reid_result["matched_path"])
        person_id = os.path.splitext(matched_filename)[0]
        print(f"  [ROBOT] confirmed identity: {person_id}")
    elif status == "new":
        person_id = f"ID_{get_next_id():04d}"
        cv2.imwrite(os.path.join(KNOWN_FACES_DIR, f"{person_id}.jpg"), crop)
        print(f"  [ROBOT] new person found on-site: {person_id}")
    else:
        print(f"  [ROBOT] person visible but no clear face{label}: {reid_result.get('reason')}")
        return False

    save_appearance_profile(person_id, crop)
    is_authorized = person_id in authorized_ids
    print(f"  [ROBOT] double-check result: {person_id} -> {'AUTHORIZED' if is_authorized else 'NOT AUTHORIZED'}")
    append_log_entry({
        "event_type": "robot_response", "found": True,
        "person_id": person_id, "authorized": is_authorized,
        "alert_reason": alert.get("reason"), "zone": alert.get("zone"),
        "timestamp": datetime.now(timezone.utc).isoformat()
    })
    return True


def search_for_face(robot_state, alert, authorized_ids):
    """REAL fix for "the robot just spins in place near the body and finds
    nothing": rotating the camera at ONE fixed standing spot can never see
    a face that happens to be oriented away from that spot - no amount of
    turning changes WHERE the robot is standing. This now actually orbits
    the target: it physically moves to several standpoints around them
    (a circle of ORBIT_RADIUS meters, ORBIT_STANDPOINTS positions around
    it), faces the center from each one, and runs a real vision-based scan
    (YOLO -> crop -> DeepFace, via attempt_robot_scan) at each standpoint -
    stopping as soon as one actually succeeds. No ground-truth aiming at a
    known mover position anymore either - every standpoint is judged
    purely by what the robot's own camera + YOLO/DeepFace actually see,
    same as a real system would have to.

    Purely kinematic - moving between standpoints and facing the center is
    just set_world_translate()/set_rotation() (same as the rest of this
    file's movement), no physics, no Nav2, instant and reliable by
    construction.
    """
    ORBIT_RADIUS = 2.5       # meters - shrunk from 4.0: several zone centers are close enough to walls/corners that a 4m orbit sent standpoints straight outside the building (confirmed via diagnose_room_bounds.py - real floor is only ~35x20m and zones sit near corners)
    ORBIT_STANDPOINTS = 6    # trimmed from 8 - fewer forced-render sequences per search now that the target is actually frozen and correctly aimed at

    center = alert["coords"]
    print(f"  [ROBOT] arrived near {alert['zone']} ({alert['reason']}) - "
          f"orbiting the target across {ORBIT_STANDPOINTS} standpoints to find a clear view of their face...")

    for step in range(ORBIT_STANDPOINTS):
        angle = (2 * math.pi / ORBIT_STANDPOINTS) * step
        raw_x = center[0] + ORBIT_RADIUS * math.sin(angle)
        raw_y = center[1] + ORBIT_RADIUS * math.cos(angle)
        # Clamp each standpoint to the real floor bounds - zone centers are
        # back to being the actual camera positions (near walls/corners),
        # so some orbit standpoints would otherwise land outside the
        # building again. This keeps the robot physically inside no matter
        # what, even if it means a slightly squashed orbit near a corner.
        sx, sy = clamp_to_floor(raw_x, raw_y)
        standpoint = (sx, sy, robot_state["base_height"])
        # Face the CENTER (the person). Now that the camera is our own
        # SimpleRobot asset (see create_lightweight_robot), it was mounted
        # specifically to match the SAME body-forward convention used
        # everywhere else in this file - plain atan2(dx, -dy), no more
        # special-cased offset math for a mystery mount rotation.
        dx = center[0] - standpoint[0]
        dy = center[1] - standpoint[1]
        face_yaw = math.degrees(math.atan2(dx, -dy))
        set_world_translate(robot_state["prim"], standpoint)
        set_rotation(robot_state["prim"], (0.0, 0.0, face_yaw))
        for _ in range(3):  # let the move/rotation land and a real frame catch up before scanning
            _tick()
        if attempt_robot_scan(robot_state["camera_path"], authorized_ids, alert,
                               heading_label=f"standpoint {step + 1}/{ORBIT_STANDPOINTS}"):
            print(f"  [ROBOT] face search succeeded from standpoint {step + 1}/{ORBIT_STANDPOINTS}")
            return

    print(f"  [ROBOT] face search exhausted all {ORBIT_STANDPOINTS} orbit standpoints without a confirmed identification")
    append_log_entry({
        "event_type": "robot_response", "found": False,
        "alert_reason": alert.get("reason"), "zone": alert.get("zone"),
        "note": "face search exhausted all orbit standpoints",
        "timestamp": datetime.now(timezone.utc).isoformat()
    })


def simplify_robot_visuals(stage, robot_prim_path, keep_path_substrings):
    """Hides every Mesh prim under the robot EXCEPT ones whose path
    contains one of keep_path_substrings (the camera mount chain, so you
    can still see it turn to look at people) - pure render-cost cut, zero
    functional impact, since none of this script's movement/rotation logic
    touches mesh geometry at all, only the base_footprint prim's transform
    ops. This is the concrete "make it a skeleton except the part that
    needs to rotate" request - a full mobile-manipulator mesh (wheels,
    chassis, sensor housings, etc.) is a lot of triangles to keep rendering
    every frame for a demo that doesn't need to look detailed.

    Hiding via UsdGeom.Imageable.MakeInvisible() rather than deleting/
    unloading - completely reversible (just call MakeVisible() later) and
    doesn't disturb the prim hierarchy or any transform ops.
    """
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"  [simplify_robot] robot prim not found at {robot_prim_path}, skipping.")
        return
    hidden = 0
    kept = 0
    for prim in Usd.PrimRange(robot_prim):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        path_str = str(prim.GetPath())
        if any(sub in path_str for sub in keep_path_substrings):
            kept += 1
            continue
        UsdGeom.Imageable(prim).MakeInvisible()
        hidden += 1
    print(f"  [simplify_robot] hid {hidden} mesh(es), kept {kept} mesh(es) matching {keep_path_substrings}.")


def create_lightweight_robot(stage):
    """Replaces the heavy mobile_manipulator_ros asset entirely. That asset
    dragged in a full ROS2/Nav2/PhysX-oriented rig (wheel joints, a
    differential controller, lidar sensors, a manipulator arm, broken TF
    frames) for a project that, by this point, uses none of that -
    movement has been fully kinematic for a while, and ROS2 was never
    actually driving anything by the end. Keeping that whole asset around
    just for its camera mount was real, unnecessary prim-count/render cost,
    on top of being the actual source of the 120-degree camera-mount-offset
    problem that took a whole session to reverse-engineer.

    This creates a tiny robot from scratch instead: a slim, person-height
    body (a cylinder, not a squat cube - you specifically asked for
    "personable height" so the camera sits at realistic face-scanning
    height instead of somewhere down near people's knees) plus a camera
    mounted near the top, at a rotation WE choose and control -
    specifically chosen so the camera's forward axis matches the EXACT
    SAME body-forward convention already used everywhere else in this file
    (atan2(dx, -dy), same as update_person_position/update_robot's
    movement-facing math). So "aim the camera at something" is now just
    "use the same yaw formula you'd use to face that direction" - no more
    special-cased offset math, anywhere, ever again for this asset.
    """
    BODY_HEIGHT = 1.6   # meters - person-scale, not a squat box
    BODY_RADIUS = 0.18
    CAMERA_HEIGHT = 1.55  # near the top of the body - roughly adult face height

    root_path = "/World/SimpleRobot"
    root_xform = UsdGeom.Xform.Define(stage, root_path)
    root_prim = root_xform.GetPrim()
    root_xform.AddTranslateOp()
    root_xform.AddRotateXYZOp()

    body_path = root_path + "/body"
    body = UsdGeom.Cylinder.Define(stage, body_path)
    body.CreateHeightAttr(BODY_HEIGHT)
    body.CreateRadiusAttr(BODY_RADIUS)
    body.CreateAxisAttr("Z")
    UsdGeom.Xformable(body.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, BODY_HEIGHT / 2))

    camera_path = root_path + "/camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    cam_xform = UsdGeom.Xformable(camera.GetPrim())
    cam_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, CAMERA_HEIGHT))
    # A USD camera's default local look direction is -Z. Rotating -90 deg
    # about local X maps local -Z onto local -Y - i.e. at yaw=0 (root
    # unrotated), this camera looks toward world -Y, exactly matching
    # update_person_position's f0=(0,-1,0) body-forward convention. Verified
    # by hand (Rodrigues rotation of (0,0,-1) by -90 deg about X = (0,-1,0)),
    # not left to guesswork this time.
    cam_xform.AddRotateXYZOp().Set(Gf.Vec3f(-90, 0, 0))

    print(f"Created lightweight replacement robot at {root_path} (camera at {camera_path}).")
    return root_prim, camera_path


def update_robot(robot_state, elapsed, authorized_ids):
    """Fully kinematic robot movement - moves and rotates the robot prim
    directly, exactly like update_person_position() does for the patrol
    people, instead of watching real PhysX/Nav2-driven motion. No physics,
    no wheels, no ROS2/WSL dependency for movement at all.

    Behavior: patrols ROBOT_WAYPOINTS in a loop by default. Every
    ROBOT_CHECK_INTERVAL seconds, checks the event log for a new unhandled
    dispatch alert - if one exists and isn't already the current target,
    it INTERRUPTS the patrol and heads there instead (higher priority than
    finishing the current patrol leg). On arrival at an alert's coords,
    runs the same active face search as before, then resumes patrol.
    """
    # search_for_face() calls _tick() internally (to keep rendering/physics
    # moving during the pause-then-scan sequence), which calls straight
    # back into this function. Without this guard, every one of those
    # inner calls saw no current_target (cleared right before entering the
    # search, to stop a different recursion bug) and picked a fresh patrol
    # waypoint - so the robot visibly walked away mid-search instead of
    # standing still to scan, and the alert-check below kept firing
    # "new dispatch alert" repeatedly during the same search. This makes
    # every nested call a complete no-op until the search actually finishes.
    if robot_state.get("in_search"):
        return

    dt = elapsed - robot_state.get("last_move_time", elapsed)
    robot_state["last_move_time"] = elapsed

    # Periodically check for a new higher-priority alert to interrupt
    # patrol with - gated by ROBOT_CHECK_INTERVAL since this does file I/O
    # (find_next_unhandled_alert reads EVENT_LOG_FILE), unlike the actual
    # movement below which needs to run every single tick to be smooth.
    if elapsed - robot_state["last_check"] >= ROBOT_CHECK_INTERVAL:
        robot_state["last_check"] = elapsed
        alert = find_next_unhandled_alert(robot_state["handled_timestamps"])
        if alert is not None and alert is not robot_state.get("current_alert"):
            robot_state["current_alert"] = alert
            robot_state["current_target"] = tuple(alert["coords"])
            print(f"  [ROBOT] new dispatch alert - interrupting patrol, heading to {alert['coords']} ({alert['reason']})")

    # No target yet (first tick, or just finished handling something) -
    # pick the next patrol waypoint.
    if robot_state.get("current_target") is None:
        wp = ROBOT_WAYPOINTS[robot_state["waypoint_index"]]
        robot_state["current_target"] = (wp[0], wp[1])

    # THE REAL FIX for "the robot drives right past them and finds nothing":
    # current_target for a person-triggered alert used to be a ONE-TIME
    # coordinate snapshot taken the instant the alert fired. The person
    # keeps walking after that. By the time the robot crosses the room to
    # that now-stale point, they're gone - the robot arrives to an empty
    # spot every time, exactly what was happening. Fix: while chasing an
    # alert (current_alert is set), re-fetch the actual person's LIVE
    # position and overwrite current_target with it, every single tick -
    # continuous tracking, not a snapshot. Only correct in general when
    # there's exactly one mover total - with more than one there's no way
    # to know which one the alert was actually about from here, so this
    # deliberately falls back to the static snapshot coords in that case
    # rather than guessing wrong.
    if robot_state.get("current_alert") is not None:
        movers = _tick_state.get("movers", [])
        if len(movers) == 1:
            live_pos = get_translate(movers[0]["prim"])
            robot_state["current_target"] = clamp_to_floor(live_pos[0], live_pos[1])

    pos = get_translate(robot_state["prim"])
    tx, ty = robot_state["current_target"]
    dist = math.hypot(tx - pos[0], ty - pos[1])  # arrival check is always against the REAL final target, never the doorway hop below

    if dist <= ARRIVAL_RADIUS:
        alert = robot_state.get("current_alert")
        # Use the LIVE target position at the moment of arrival as the
        # orbit center, not the alert's original stale coordinate snapshot
        # - for a loitering/unauthorized alert the person may have kept
        # moving during the chase (current_target was continuously updated
        # above to track them), so by arrival time current_target IS their
        # real current position and alert["coords"] is outdated. For a
        # fall alert they're frozen so both are the same anyway - this is
        # strictly more correct in both cases, never worse.
        if alert is not None:
            alert = dict(alert)
            alert["coords"] = robot_state["current_target"]
        # Clear state BEFORE calling search_for_face, not after - real bug
        # found here: search_for_face() calls _tick() internally (to keep
        # rendering/physics moving during the pause-then-scan sequence),
        # and _tick() calls straight back into update_robot(). With the
        # old order (clear state after search_for_face returns), every one
        # of those inner _tick() calls saw the robot still "arrived" with
        # current_alert still set, and called search_for_face() AGAIN from
        # inside itself - infinite recursion until Python's stack blew up.
        # Clearing current_alert/current_target first means those inner
        # _tick() calls see no active alert and just fall through to
        # normal movement/patrol logic instead of re-triggering the search.
        robot_state["current_alert"] = None
        robot_state["current_target"] = None
        if alert is not None:
            # REAL FIX for consistent 8/8 orbit failures: nothing was
            # stopping the person from continuing to walk their patrol
            # route WHILE the robot spent real seconds moving between all
            # 8 orbit standpoints. The orbit center was fixed at arrival,
            # but by standpoint 3-4 the person had already walked away
            # from it - every single search was chasing a target that kept
            # moving out from under it. Freeze every mover for the
            # duration of the search, then restore their exact original
            # paused_until afterward (so this can't accidentally shorten a
            # legitimate longer freeze already in progress, e.g. mid
            # scripted fall).
            movers = _tick_state.get("movers", [])
            saved_pause = {m["name"]: m.get("paused_until", 0) for m in movers}
            for m in movers:
                m["paused_until"] = elapsed + 300  # comfortably longer than any search will take
            robot_state["in_search"] = True
            try:
                search_for_face(robot_state, alert, authorized_ids)
            finally:
                robot_state["in_search"] = False
                for m in movers:
                    m["paused_until"] = saved_pause[m["name"]]
            robot_state["handled_timestamps"].add(alert["timestamp"])
        else:
            robot_state["waypoint_index"] = (robot_state["waypoint_index"] + 1) % len(ROBOT_WAYPOINTS)
        return

    # Move toward the NEXT HOP (doorway detour if crossing rooms, else the
    # real target directly - see get_next_hop) at ROBOT_MOVE_SPEED, capped
    # so a big dt (e.g. right after a long blocking sweep call) can't
    # overshoot past it and start oscillating.
    hop_x, hop_y = get_next_hop(pos[0], pos[1], tx, ty)
    dx = hop_x - pos[0]
    dy = hop_y - pos[1]
    hop_dist = math.hypot(dx, dy)
    step_dist = min(ROBOT_MOVE_SPEED * dt, hop_dist)
    if hop_dist > 0:
        raw_x = pos[0] + dx / hop_dist * step_dist
        raw_y = pos[1] + dy / hop_dist * step_dist
        new_x, new_y = move_with_wall_check(pos[0], pos[1], raw_x, raw_y)  # real wall-collision enforcement, not decorative - see move_with_wall_check's comment
        new_world_pos = (new_x, new_y, robot_state["base_height"])
        yaw_deg = math.degrees(math.atan2(dx, -dy))  # same forward-axis convention as update_person_position
        set_rotation(robot_state["prim"], (0.0, 0.0, yaw_deg))
        set_world_translate(robot_state["prim"], new_world_pos)  # NOT set_translate() - see set_world_translate()'s comment for why

    if elapsed - robot_state.get("last_move_print", -999) >= 3.0 and dt > 0:
        robot_state["last_move_print"] = elapsed
        print(f"  [ROBOT] moving: at ({pos[0]:.2f}, {pos[1]:.2f}), "
              f"target ({tx:.2f}, {ty:.2f}), dist={dist:.2f}m")


# ---------- Tracking helpers ----------

def get_next_id():
    if not os.path.exists(ID_COUNTER_FILE):
        next_id = 1
    else:
        with open(ID_COUNTER_FILE, "r") as f:
            next_id = int(f.read().strip())
    with open(ID_COUNTER_FILE, "w") as f:
        f.write(str(next_id + 1))
    return next_id


def load_authorized_ids():
    if not os.path.exists(AUTHORIZED_IDS_FILE):
        return []
    with open(AUTHORIZED_IDS_FILE, "r") as f:
        return json.load(f).get("authorized", [])


def append_log_entry(entry):
    log = []
    if os.path.exists(EVENT_LOG_FILE):
        with open(EVENT_LOG_FILE, "r") as f:
            try:
                log = json.load(f)
            except json.JSONDecodeError:
                log = []
    log.append(entry)
    with open(EVENT_LOG_FILE, "w") as f:
        json.dump(log, f, indent=2)


_camera_cache = {}


_camera_pause_supported = True  # flips to False if .pause()/.resume() ever throw, so we stop trying and stop pretending it's helping


def capture_frame(camera_path, resolution=(320, 240)):
    # Reuse one Camera object per path instead of creating a new one (and a
    # new render product/annotator) on every single scan call - doing that
    # every 8s for 5 cameras leaked render products, overran internal render
    # buffers ("sequence size exceeds remaining buffer" spam), and stalled
    # the physics step rate badly enough to starve the ROS2 odom publisher.
    global _camera_pause_supported
    camera = _camera_cache.get(camera_path)
    just_created = camera is None
    if camera is None:
        camera = Camera(prim_path=camera_path, resolution=resolution)
        camera.initialize()
        # Set aperture to match this resolution's aspect ratio up front -
        # otherwise Isaac Sim silently auto-corrects it (and logs a warning)
        # every time. Real fix instead of just suppressing the warning.
        aspect = resolution[0] / resolution[1]
        camera.set_horizontal_aperture(2.0955)
        camera.set_vertical_aperture(2.0955 / aspect)
        _camera_cache[camera_path] = camera
        for _ in range(10):
            simulation_app.update()

    # REAL FIX for the "robot crawls at 0.1 m/s" ceiling: once a camera got
    # cached above, it was left permanently active for the rest of the run -
    # every one of the 7 cameras in this scene (5 zones + checkpoint + robot)
    # was rendering its own full render product on EVERY simulation_app.update()
    # call, including the 90%+ of wall-clock time between sweeps when the
    # robot/people should be moving at full speed and nothing is even being
    # captured. That's the actual "GPU render cost / 6+ simultaneous render
    # products" ceiling from NOTES.md - it isn't a fixed hardware wall, it's
    # 6 idle cameras rendering for no reason. Nav2's vx_max and the
    # differential_controller node are both already maxed (10.0 m/s) -
    # raising either further does nothing, because the bottleneck is real
    # SIMULATED seconds per REAL second (see the [PERF] updates/sec log
    # line), not the commanded velocity.
    #
    # Fix: only the camera actively being captured stays "resumed" (its
    # render product ticking); every cached camera goes back to .pause()
    # immediately after its frame is grabbed, so idle cameras cost ~nothing
    # per tick instead of a full render each. Camera.pause()/.resume() are
    # real public isaacsim.sensors.camera.Camera methods, but wrapped
    # defensively anyway per this project's own convention - if this Isaac
    # Sim build's version behaves differently, it fails safe (frames keep
    # working exactly as before, just without the speed gain) instead of
    # breaking capture. Watch the [PERF] updates/sec line after this change -
    # that's the real test, not just "did the robot look faster."
    if _camera_pause_supported and not just_created:
        try:
            camera.resume()
        except Exception as e:
            _camera_pause_supported = False
            print(f"  [WARN] Camera.resume() not supported on this build ({e}) - "
                  f"disabling the pause/resume optimization, falling back to always-on cameras.")

    rgba = None
    for i in range(20):  # cut from 60 - each iteration in this loop is a full forced real render at full window resolution; most successful captures land in the first few iterations anyway, this just caps the worst-case lag spike from a slow/stuck capture
        _tick(force_render=True)  # this loop actually needs a real frame back every time, unlike every other _tick() caller in this file
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            break

    if _camera_pause_supported:
        try:
            camera.pause()
        except Exception as e:
            _camera_pause_supported = False
            print(f"  [WARN] Camera.pause() not supported on this build ({e}) - "
                  f"disabling the pause/resume optimization, falling back to always-on cameras.")

    if rgba is None or rgba.size == 0:
        return None

    rgb = rgba[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


class PersistentWorker:
    """Keeps a single long-running python.bat subprocess alive for the
    whole session, communicating via stdin/stdout - one request per line
    in, one JSON response per line out. Used for yolo_worker.py and
    deepface_worker.py, which load their (expensive) models ONCE at
    startup instead of on every single call, unlike run_subprocess()'s
    spawn-fresh-every-time pattern (still used elsewhere for cheap one-off
    scripts). This was the dominant real cost behind the whole "everything
    is so slow" investigation - not render resolution, not GPU rendering
    contention (both real, but secondary) - full model reloads happening
    5-6 times per single 8-second sweep cycle, every sweep, for the entire
    session.

    Reads happen on a background thread into a queue rather than a
    blocking readline() in the main thread, so the caller can keep calling
    _tick() (physics/movement/rendering) while waiting for a response,
    same pattern as run_subprocess()'s own polling loop.
    """

    def __init__(self, script_name):
        import threading
        import queue as _queue
        script_path = os.path.join(PROJECT_DIR, script_name)
        self.proc = subprocess.Popen(
            ['C:\\isaacsim\\python.bat', script_path],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", errors="replace", bufsize=1
        )
        self._queue_mod = _queue
        self.out_queue = _queue.Queue()
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()
        self.ready = False

    def _read_loop(self):
        for line in self.proc.stdout:
            stripped = line.strip()
            if stripped:
                self.out_queue.put(stripped)

    def wait_ready(self, timeout=120):
        start = time.time()
        while time.time() - start < timeout:
            _tick()
            try:
                line = self.out_queue.get(timeout=0.05)
                if line == "READY":
                    self.ready = True
                    return True
            except self._queue_mod.Empty:
                continue
        return False

    def call(self, request_line, timeout=30):
        if self.proc.poll() is not None:
            return {"error": "worker process has exited"}
        try:
            self.proc.stdin.write(request_line + "\n")
            self.proc.stdin.flush()
        except Exception as e:
            return {"error": f"failed to write to worker: {e}"}
        start = time.time()
        while time.time() - start < timeout:
            _tick()
            try:
                line = self.out_queue.get(timeout=0.05)
            except self._queue_mod.Empty:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
        return {"error": "worker call timed out"}


# Module-level worker handles, populated once in main() before the sweep
# loop starts. Needed at module level (not local to main()) since
# scan_zone_camera/scan_checkpoint/attempt_robot_scan are all separate
# top-level functions that need to reach them.
_workers = {"yolo": None, "deepface": None}


def run_subprocess(script_name, args):
    script_path = os.path.join(PROJECT_DIR, script_name)
    arg_str = " ".join(f'"{a}"' for a in args)
    # Was subprocess.run() (fully blocking) - during that block,
    # simulation_app.update() never ran, so PhysX never stepped and Nav2's
    # /cmd_vel commands never actually got applied to the robot, even though
    # they kept arriving fine on the ROS2 wire (which is why `ros2 topic hz`
    # looked healthy while the robot barely moved). Using Popen + polling
    # keeps physics/rendering stepping the whole time this runs.
    #
    # IMPORTANT: this polling loop calls _tick() (movement + robot update +
    # simulation_app.update()), NOT just simulation_app.update() alone. A
    # single sweep runs 5 cameras x 2 subprocess calls (YOLO + DeepFace)
    # each taking multiple seconds, so this loop is where the vast majority
    # of each sweep's ~25-30s wall-clock time is actually spent. Previously
    # movement was computed only in main()'s outer loop, which is blocked
    # here for that entire duration - elapsed (wall-clock time.time()) kept
    # advancing the whole time regardless, so the very next movement update
    # after a sweep finished would jump straight to wherever ~25-30s of
    # progress along the patrol path landed, looking like a teleport. Now
    # movement is updated on every single simulation_app.update() call,
    # including all the ones spent waiting here, so it stays smooth
    # throughout a sweep instead of freezing then jumping.
    proc = subprocess.Popen(
        f'"C:\\isaacsim\\python.bat" "{script_path}" {arg_str}',
        shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding="utf-8", errors="replace"
    )
    while proc.poll() is None:
        _tick()
        time.sleep(0.01)
    stdout, stderr = proc.communicate()
    if proc.returncode != 0:
        return {"error": stderr}
    try:
        return json.loads(stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        return {"error": f"failed to parse output: {stdout}"}


def crop_person(bgr_image, detection, padding=25):
    h, w = bgr_image.shape[:2]
    x1 = max(0, int(detection["x1"]) - padding)
    y1 = max(0, int(detection["y1"]) - padding)
    x2 = min(w, int(detection["x2"]) + padding)
    y2 = min(h, int(detection["y2"]) + padding)
    return bgr_image[y1:y2, x1:x2]


def extract_appearance_signature(crop):
    """Computes a normalized HSV color histogram of the crop, used as a
    lightweight 'what are they wearing' fingerprint - a fallback identity
    signal for when face detection fails (common on wide zone cameras)."""
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], None, [8, 8, 8], [0, 180, 0, 256, 0, 256])
    hist = cv2.normalize(hist, hist).flatten()
    return hist.tolist()


def compare_appearance(sig1, sig2):
    h1 = np.array(sig1, dtype=np.float32)
    h2 = np.array(sig2, dtype=np.float32)
    return cv2.compareHist(h1, h2, cv2.HISTCMP_CORREL)


def load_appearance_profiles():
    if not os.path.exists(APPEARANCE_PROFILES_FILE):
        return {}
    with open(APPEARANCE_PROFILES_FILE, "r") as f:
        try:
            return json.load(f)
        except json.JSONDecodeError:
            return {}


def save_appearance_profile(person_id, crop):
    """Called whenever we have a CONFIRMED identity (from a face match/new
    at the checkpoint) - records what they're wearing so zone cameras can
    recognize them later even without a clean face shot."""
    profiles = load_appearance_profiles()
    profiles[person_id] = extract_appearance_signature(crop)
    with open(APPEARANCE_PROFILES_FILE, "w") as f:
        json.dump(profiles, f)


def find_appearance_match(crop, threshold=APPEARANCE_MATCH_THRESHOLD):
    """Compares this crop's color signature against all known appearance
    profiles. Returns (person_id, score) for the best match above threshold,
    or (None, None) if nothing matches closely enough."""
    profiles = load_appearance_profiles()
    if not profiles:
        return None, None

    sig = extract_appearance_signature(crop)
    best_id, best_score = None, threshold
    for pid, known_sig in profiles.items():
        score = compare_appearance(sig, known_sig)
        if score > best_score:
            best_id, best_score = pid, score
    return (best_id, best_score) if best_id else (None, None)


def check_for_fall(detection):
    """A standing person's bounding box is taller than wide. A fallen
    person's box becomes wider than tall. Reuses the YOLO detection we
    already computed - no extra model needed."""
    width = detection["x2"] - detection["x1"]
    height = detection["y2"] - detection["y1"]
    if height == 0:
        return False
    aspect_ratio = width / height
    print(f"    (aspect ratio: {aspect_ratio:.2f}, fall threshold is 1.3)")
    return aspect_ratio > 1.3


def check_loitering(zone_name, streak_state, threshold=LOITER_THRESHOLD):
    """Tracks a live consecutive-presence streak per zone, in memory, across
    sweeps. A sweep that finds someone in the zone increments the streak;
    a sweep that finds nobody resets it to zero. This correctly captures
    'someone has been continuously here' rather than 'someone has been
    spotted here a few times' (which the old log-based version conflated,
    since empty sweeps were never logged and gaps between different people
    passing through looked identical to one person staying put).

    Also only fires the alert once per streak (not every sweep past the
    threshold) via an 'alerted' flag that resets alongside the streak.
    """
    if streak_state[zone_name]["streak"] >= threshold and not streak_state[zone_name]["alerted"]:
        streak_state[zone_name]["alerted"] = True
        return True
    return False


def scan_zone_camera(camera_path, zone_name, movers):
    """Zone cameras ONLY detect + check fall/loitering now - DeepFace
    identification was removed entirely from this path per explicit scope
    cut: fall and loitering detection only need YOLO's bounding box shape
    and a presence streak, never WHO the person is. Running full face
    re-identification on every one of 4 zone cameras every single 8-second
    sweep was a large, unnecessary, recurring cost that had nothing to do
    with this project's actual stated goal. Identification still happens
    at the fixed checkpoint camera and via the robot's own orbit search -
    neither of those changed.
    """
    print(f"  scanning {zone_name}...")
    bgr = capture_frame(camera_path, resolution=(640, 480))  # bumped from 160x120 - confirmed via saved debug frames that a person across a ~35m room was only a few blurry pixels tall at that resolution, undetectable by YOLO regardless of whether they were in frame. DeepFace (the actually expensive part) is no longer run on zone cameras, so a much higher YOLO-only resolution is cheap by comparison.
    if bgr is None:
        return None

    tag = zone_name.replace(" ", "_")

    # ALWAYS save the raw frame, regardless of whether anyone gets detected
    # - this is the actual visibility gap that made "why isn't the camera
    # finding anything" impossible to diagnose: the old code only ever
    # saved a debug image AFTER a detection already succeeded, so a camera
    # that detects NOTHING left zero visual evidence behind to check
    # against. Now every single scan leaves a real frame in debug_captures
    # to look at, success or failure.
    raw_debug_name = f"{datetime.now().strftime('%H%M%S')}_{tag}_RAWFRAME.jpg"
    cv2.imwrite(os.path.join(DEBUG_DIR, raw_debug_name), bgr)

    frame_path = os.path.join(os.environ["TEMP"], f"scan_{tag}.jpg")
    cv2.imwrite(frame_path, bgr)

    yolo_result = _workers["yolo"].call(frame_path)
    if yolo_result.get("error"):
        return {"error": f"YOLO: {yolo_result['error']}"}

    detections = yolo_result.get("detections", [])
    if not detections:
        return None

    best_detection = max(detections, key=lambda d: d["confidence"])
    fell = check_for_fall(best_detection)

    if fell:
        print(f"  [{zone_name}] *** FALL DETECTED ***")
        append_log_entry({
            "event_type": "fall_alert",
            "zone": zone_name,
            "camera": camera_path,
            "timestamp": datetime.now(timezone.utc).isoformat()
        })
        append_log_entry({
            "event_type": "dispatch_alert",
            "reason": "fall",
            "person_id": None,
            "zone": zone_name,
            "coords": get_nearest_mover_coords(movers, zone_name),
            "timestamp": datetime.now(timezone.utc).isoformat()
        })

    # Still save a debug crop image for visual sanity-checking - this is
    # just a file write, not a DeepFace inference call, effectively free.
    crop = crop_person(bgr, best_detection)
    if crop.size > 0:
        debug_name = f"{datetime.now().strftime('%H%M%S')}_{tag}_crop.jpg"
        cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    return {"detected": True, "fell": fell}


def scan_checkpoint(authorized_ids):
    print(f"  scanning checkpoint...")
    bgr = capture_frame(CHECKPOINT_PATH, resolution=(480, 360))
    if bgr is None:
        return

    frame_path = os.path.join(os.environ["TEMP"], "checkpoint_scan.jpg")
    cv2.imwrite(frame_path, bgr)
    yolo_result = _workers["yolo"].call(frame_path)
    detections = yolo_result.get("detections", []) if not yolo_result.get("error") else []
    if not detections:
        return

    best_detection = max(detections, key=lambda d: d["confidence"])

    if check_for_fall(best_detection):
        print(f"  [CHECKPOINT] *** FALL DETECTED ***")
        append_log_entry({
            "event_type": "fall_alert",
            "zone": "CHECKPOINT",
            "camera": CHECKPOINT_PATH,
            "timestamp": datetime.now(timezone.utc).isoformat()
        })

    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return

    crop_path = os.path.join(os.environ["TEMP"], "checkpoint_crop.jpg")
    cv2.imwrite(crop_path, crop)

    debug_name = f"{datetime.now().strftime('%H%M%S')}_checkpoint_crop.jpg"
    cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    reid_result = _workers["deepface"].call(f"{crop_path}\t{KNOWN_FACES_DIR}")

    person_id = None
    status = reid_result.get("status")

    if status == "match":
        matched_filename = os.path.basename(reid_result["matched_path"])
        person_id = os.path.splitext(matched_filename)[0]
        print(f"  [CHECKPOINT] IDENTIFIED: {person_id}")
    elif status == "new":
        person_id = f"ID_{get_next_id():04d}"
        cv2.imwrite(os.path.join(KNOWN_FACES_DIR, f"{person_id}.jpg"), crop)
        print(f"  [CHECKPOINT] NEW PERSON: {person_id}")
    else:
        print(f"  [CHECKPOINT] face detection failed, skipping: {reid_result.get('reason')}")

    if person_id:
        # Face confirmed their identity here, so this is a trustworthy moment
        # to record/update their clothing signature for zone cameras to use.
        save_appearance_profile(person_id, crop)

        is_authorized = person_id in authorized_ids
        print(f"  [CHECKPOINT] {person_id} -> {'AUTHORIZED' if is_authorized else 'NOT AUTHORIZED'}")
        append_log_entry({
            "event_type": "checkpoint",
            "person_id": person_id,
            "checkpoint": CHECKPOINT_NAME,
            "authorized": is_authorized,
            "timestamp": datetime.now(timezone.utc).isoformat()
        })


def run_sweep(camera_zones, authorized_ids, streak_state, movers, elapsed):
    timestamp = datetime.now(timezone.utc).isoformat()
    print(f"\n--- Sweep at {timestamp} ---")

    for camera_path, zone_name in camera_zones.items():
        result = scan_zone_camera(camera_path, zone_name, movers)

        if result is None or result.get("error"):
            # Nobody detected (or a scan error) - the presence streak breaks here.
            streak_state[zone_name]["streak"] = 0
            streak_state[zone_name]["alerted"] = False
            if result and result.get("error"):
                print(f"  [{zone_name}] error: {result['error']}")
            continue

        if result.get("fell"):
            pause_nearest_mover(movers, zone_name, elapsed)

        streak_state[zone_name]["streak"] += 1
        print(f"  [{zone_name}] person detected (streak={streak_state[zone_name]['streak']})")

        # unauthorized_zone_presence removed along with zone-camera face/
        # clothing identification - that alert type depended entirely on
        # knowing WHO was detected, which zone cameras no longer check
        # (out of scope per explicit direction: zone cameras are fall/
        # loitering detection only). Identity/authorization checks still
        # happen at the checkpoint camera and via the robot's own search.

        if check_loitering(zone_name, streak_state):
            coords = get_nearest_mover_coords(movers, zone_name)
            print(f"  [{zone_name}] *** LOITERING (someone present {LOITER_THRESHOLD}+ sweeps in a row) ***")
            pause_nearest_mover(movers, zone_name, elapsed)
            append_log_entry({
                "event_type": "loitering_alert",
                "zone": zone_name,
                "coords": coords,
                "timestamp": timestamp
            })
            append_log_entry({
                "event_type": "dispatch_alert",
                "reason": "loitering",
                "person_id": None,
                "zone": zone_name,
                "coords": coords,
                "timestamp": timestamp
            })

        append_log_entry({
            "event_type": "zone",
            "zone": zone_name,
            "camera": camera_path,
            "timestamp": timestamp
        })

    if CHECKPOINT_AVAILABLE:
        scan_checkpoint(authorized_ids)


# ---------- Main ----------

def main():
    os.makedirs(KNOWN_FACES_DIR, exist_ok=True)

    # Clear out the debug folder from the previous run so it doesn't pile up,
    # then recreate it fresh for this run's captures.
    if os.path.exists(DEBUG_DIR):
        shutil.rmtree(DEBUG_DIR)
    os.makedirs(DEBUG_DIR, exist_ok=True)

    # REAL FIX for "why is it chasing (28.67, 28.73) / Zone_A_NorthEast /
    # unauthorized_zone_presence" - none of that exists anywhere in this
    # file anymore. event_log.json was never cleared between runs - it just
    # accumulates forever across every session, including ones from way
    # before this room/scene even existed. Every fresh run resets
    # handled_timestamps to empty, so find_next_unhandled_alert() happily
    # re-served every ancient unhandled dispatch_alert ever written to that
    # file, from any past session, as if it were brand new. Wiping it at
    # the start of every run is the actual fix - old alerts from a
    # previous run have no business being acted on in a new one anyway.
    with open(EVENT_LOG_FILE, "w") as f:
        json.dump([], f)

    with open(CAMERA_ZONES_FILE, "r") as f:
        camera_zones = json.load(f)

    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    # Check for real (not guessed) whether a checkpoint camera actually
    # exists in this scene - see CHECKPOINT_AVAILABLE's comment above.
    global CHECKPOINT_AVAILABLE
    CHECKPOINT_AVAILABLE = stage.GetPrimAtPath(CHECKPOINT_PATH).IsValid()
    if not CHECKPOINT_AVAILABLE:
        print(f"No checkpoint camera found at {CHECKPOINT_PATH} in this scene - "
              f"checkpoint scanning disabled for this run (zone cameras + robot orbit search still work normally).")

    # Set up the physics/render decoupling (see the RENDER_EVERY_N_TICKS
    # comment near the top of this file) now that a stage is actually
    # loaded - SimulationContext needs a stage to attach to. Wrapped
    # defensively: if this build's SimulationContext doesn't accept these
    # exact kwargs or .step() doesn't support render=, _tick() already
    # falls back to plain simulation_app.update() (checked via
    # _sim_context is None / _render_decouple_supported), so a failure
    # here just means no speed gain, not a broken run.
    global _sim_context, _render_decouple_supported
    try:
        _sim_context = SimulationContext.instance() or SimulationContext()
        print("SimulationContext acquired - physics/render decoupling active "
              f"(rendering 1 in every {RENDER_EVERY_N_TICKS} ticks outside of active captures).")
    except Exception as e:
        _render_decouple_supported = False
        _sim_context = None
        print(f"  [WARN] Could not acquire SimulationContext ({e}) - "
              f"physics/render decoupling disabled, falling back to always-render ticks.")

    # The piper arm / TF-repair logic below was specific to the old heavy
    # mobile_manipulator_ros asset, which has been fully replaced by
    # create_lightweight_robot() - removed rather than left as dead code
    # that silently does nothing every run.

    # Bump ambient/dome light intensity so the scene is flat-lit and bright
    # enough for detection without needing expensive indirect
    # lighting/GI bounces to look right - requested speed lever, applied
    # directly to whatever light prims actually exist in the stage rather
    # than guessing a carb setting name for "ambient light".
    from pxr import UsdLux
    lights_adjusted = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.DistantLight) or prim.IsA(UsdLux.SphereLight):
            light = UsdLux.LightAPI(prim)
            intensity_attr = light.GetIntensityAttr()
            if intensity_attr:
                intensity_attr.Set(2.0)
            else:
                light.CreateIntensityAttr(2.0)
            lights_adjusted += 1
    print(f"  Adjusted {lights_adjusted} light prim(s) to intensity 2.0 for flatter/cheaper lighting.")

    movers = []
    for i, (name, info) in enumerate(PEOPLE.items()):
        prim = stage.GetPrimAtPath(info["prim_path"])
        if not prim.IsValid():
            print(f"WARNING: {name} prim not found at {info['prim_path']} - skipping")
            continue
        disable_physics_recursive(prim)
        start = get_translate(prim)
        base_height = start[2]
        # Force spawn position into a REAL room box, alternating west/east,
        # instead of trusting whatever position happens to be authored in
        # the USD file - per explicit direction, people should only ever
        # occupy the actual room coordinates, never wherever they happened
        # to be placed in the scene file (they're invisible at this point
        # anyway, but this guarantees it's never wrong if anything ever
        # makes them visible unexpectedly).
        room = ROOM_BOUNDS["west"] if i % 2 == 0 else ROOM_BOUNDS["east"]
        spawn_x, spawn_y = random.uniform(room[0] + 1, room[1] - 1), random.uniform(room[2] + 1, room[3] - 1)
        set_translate(prim, (spawn_x, spawn_y, base_height))
        model_prim = stage.GetPrimAtPath(info["prim_path"] + "/model")
        if not model_prim.IsValid():
            print(f"WARNING: {name} has no /model child prim - fall rotation will be a no-op for them.")
        skel_prim, walk_clip_path, fall_clip_path, held_clip_path, fall_clip_duration = get_skeleton_and_clips(model_prim)
        if skel_prim is None:
            print(f"WARNING: {name} has no Skeleton under /model - fall animation will be a no-op for them.")
        movers.append({
            "name": name, "prim": prim, "model_prim": model_prim,
            "skel_prim": skel_prim, "walk_clip_path": walk_clip_path, "fall_clip_path": fall_clip_path,
            "held_clip_path": held_clip_path, "fall_clip_duration": fall_clip_duration,
            "base_height": base_height, "start_delay": info["start_delay"],
            "loiter_start": None, "loiter_duration": None, "loiter_center": None,
            "loiter_was_active": False, "fall_center": None,
            "paused_until": 0, "fall_start": None, "fall_triggered": False, "fallen": False,
            "fall_held_triggered": False,
        })
        # Start invisible and in the static idle pose - see
        # update_person_position's docstring. They become visible/animated
        # only for the duration of their own assigned fall/loiter event.
        UsdGeom.Imageable(prim).MakeInvisible()
        if skel_prim is not None:
            set_animation_clip(skel_prim, held_clip_path)

    # --- Robot setup ---
    # Lightweight custom robot (see create_lightweight_robot) instead of the
    # old heavy mobile_manipulator_ros asset - no physics to disable, no
    # visuals to strip down, it was built minimal from the start.
    robot_base_prim, robot_camera_path = create_lightweight_robot(stage)
    base_pos = get_translate(robot_base_prim)

    robot_state = {
        "prim": robot_base_prim,
        "camera_path": robot_camera_path,
        "base_height": base_pos[2],
        "last_check": 0,
        "last_move_time": 0,
        "handled_timestamps": set(),
        "current_target": None,
        "current_alert": None,
        "waypoint_index": 0,
    }
    print(f"Lightweight robot ready, moving kinematically at {ROBOT_MOVE_SPEED} m/s, "
          f"patrolling {len(ROBOT_WAYPOINTS)} waypoints.")

    # Switch the viewport from "Stage Lights" to "Camera Light" - confirmed
    # real API (not guessed) from an NVIDIA forum answer: the viewport menu
    # bar's lighting dropdown is driven by omni.kit.actions.core actions,
    # not a plain carb setting. "Camera Light" keeps whatever you're looking
    # through lit from the camera's own position, which matters here since
    # this scene's only real light is one dome light - stage-lit corners far
    # from it can otherwise render too dark to see clearly.
    try:
        import omni.kit.actions.core as _actions_core  # aliased on purpose - a bare "import omni.kit.actions.core" here rebinds the name "omni" as LOCAL to this whole function (Python function-scoping quirk), which broke the earlier omni.usd.get_context() call above with an UnboundLocalError
        action_registry = _actions_core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        action.execute()
        print("Viewport lighting mode set to Camera Light.")
    except Exception as e:
        print(f"Could not set viewport lighting mode to Camera Light ({e}) - "
              f"you can switch it manually via the lamp icon in the viewport menu bar.")

    # NOW play the timeline, after all kinematic flags are set - order matters here.
    timeline = omni.timeline.get_timeline_interface()

    # Enable looping for the Mixamo walk-cycle SkelAnimations. Without this
    # the clip plays through once and holds its final frame (characters
    # appear to stop walking mid-patrol even though their waypoint
    # translation - handled separately in update_person_position - keeps
    # moving them). Loop end is computed from the actual baked animation's
    # last real time sample rather than assumed, so it wraps exactly on
    # the clip boundary instead of stuttering mid-stride.
    anim_loop_seconds = get_animation_loop_seconds(stage, default=1.0)
    timeline.set_start_time(0.0)
    timeline.set_end_time(anim_loop_seconds)
    timeline.set_looping(True)
    print(f"Animation loop set: 0 -> {anim_loop_seconds:.3f}s, looping enabled.")

    timeline.play()
    for _ in range(20):
        simulation_app.update()

    # Live per-zone loitering streak tracker, persists across sweeps for the
    # whole run - see check_loitering() for why this replaced a log-based check.
    streak_state = {zone_name: {"streak": 0, "alerted": False} for zone_name in camera_zones.values()}

    sim_start = time.time()
    last_sweep_time = 0
    authorized_ids_cache = load_authorized_ids()

    # Populate shared _tick_state so _tick() (called both here and from
    # inside run_subprocess/capture_frame's blocking waits) has everything
    # it needs to keep movement/robot updates running continuously, even
    # during multi-second subprocess calls mid-sweep.
    _tick_state["sim_start"] = sim_start
    _tick_state["movers"] = movers
    _tick_state["robot_state"] = robot_state
    _tick_state["authorized_ids_cache"] = authorized_ids_cache

    # Start the persistent YOLO/DeepFace workers ONCE here, before the main
    # loop. Both scripts' model loading happens exactly once now, not on
    # every single scan. wait_ready() blocks (while still calling _tick() so
    # physics/movement keep going) until each worker prints "READY".
    print("Starting persistent YOLO worker (loading model once)...")
    _workers["yolo"] = PersistentWorker("yolo_worker.py")
    if not _workers["yolo"].wait_ready():
        print("WARNING: YOLO worker did not report ready in time - scans may fail or hang.")
    else:
        print("YOLO worker ready.")

    print("Starting persistent DeepFace worker (loading models once)...")
    _workers["deepface"] = PersistentWorker("deepface_worker.py")
    if not _workers["deepface"].wait_ready():
        print("WARNING: DeepFace worker did not report ready in time - scans may fail or hang.")
    else:
        print("DeepFace worker ready.")

    # REAL FIX for "I never saw anyone become visible": FALL_START_RANGE/
    # LOITER_START_RANGE used to count from raw sim start, but the YOLO/
    # DeepFace workers above can easily take 15-40+ real seconds to finish
    # loading models - the entire fall-and-recover or loiter-and-end cycle
    # could happen, invisibly, before you ever get a chance to actually
    # look at a running window. Rebasing both to count from THIS point
    # (workers confirmed ready) instead, so the timer only starts once
    # there's actually something worth watching on screen.
    ready_elapsed = time.time() - sim_start

    if LOITER_ENABLED and movers:
        loiterer = random.choice(movers)
        loiterer["loiter_start"] = ready_elapsed + random.uniform(*LOITER_START_RANGE)
        loiterer["loiter_duration"] = random.uniform(*LOITER_DURATION_RANGE)
        loiterer["loiter_center"] = random.choice(list(ZONE_CENTERS.values()))
        print(f"*** {loiterer['name']} will loiter near {loiterer['loiter_center']} "
              f"starting at t={loiterer['loiter_start']:.1f}s for {loiterer['loiter_duration']:.1f}s ***")

    if FALL_ENABLED and movers:
        faller = random.choice(movers)
        faller["fall_start"] = ready_elapsed + random.uniform(*FALL_START_RANGE)
        faller["fall_center"] = random.choice(list(ZONE_CENTERS.values()))
        print(f"*** {faller['name']} will fall (scripted) near {faller['fall_center']} at t={faller['fall_start']:.1f}s ***")

    print("=== Unified tracking + movement loop started. Press Ctrl+C to stop. ===")

    try:
        while True:
            elapsed = time.time() - sim_start

            _tick()

            if elapsed - last_sweep_time >= SCAN_INTERVAL_SECONDS:
                authorized_ids_cache = load_authorized_ids()
                _tick_state["authorized_ids_cache"] = authorized_ids_cache
                run_sweep(camera_zones, authorized_ids_cache, streak_state, movers, elapsed)
                last_sweep_time = elapsed

    except KeyboardInterrupt:
        print("\n=== Stopped by user. ===")
    finally:
        timeline.stop()
        for worker in _workers.values():
            if worker is not None and worker.proc.poll() is None:
                try:
                    worker.proc.stdin.write("__EXIT__\n")
                    worker.proc.stdin.flush()
                except Exception:
                    pass
                worker.proc.terminate()


if __name__ == "__main__":
    main()
    simulation_app.close()