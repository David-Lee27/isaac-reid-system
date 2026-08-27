r"""
Unified tracking + movement loop.
People continuously patrol their waypoint loop WHILE the system sweeps all
4 zone cameras + checkpoint camera for detection/re-ID/authorization/fall
detection/loitering, all in one running Isaac Sim instance.

Press Ctrl+C to stop.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\unified_tracking.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False
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

SCAN_INTERVAL_SECONDS = 8   # how often to run a full camera sweep
LOITER_THRESHOLD = 3        # consecutive same-zone sightings to flag loitering

# --- People + patrol path ---
PEOPLE = {
    "Person1": {"prim_path": "/World/male_adult_police_04",          "start_delay": 0},
    "Person2": {"prim_path": "/World/male_adult_construction_03",     "start_delay": 5},
    "Person3": {"prim_path": "/World/male_adult_construction_01_new", "start_delay": 10},
}

WAYPOINTS = [
    (27.46, 16.33, 0),
    (29.06, 27.76, 0),
    (-25.60, 28.53, 0),
    (-28.41, -7.00, 0),
    (25.09, -8.53, 0),
    (26.03, 12.98, 0),
    (29.9, 13.4, 0),
]
SECONDS_PER_LEG = 4
LOOP_PATROL = True

# When a fall is detected, the nearest patrol character to that zone gets
# frozen in place for this long - simple placeholder so the robot has
# someone to actually find on arrival, rather than the "fallen" person
# just continuing to walk their patrol loop. Real fallen-pose animation is
# a separate later task.
FALL_PAUSE_DURATION = 45  # seconds

# --- Randomized loitering test ---
# One random person gets assigned a random window during the run where,
# instead of patrolling, they wander in small circles near a random zone -
# a realistic test of the loitering detector instead of manually posing someone.
LOITER_ENABLED = True
LOITER_START_RANGE = (20, 50)      # seconds into the run it can begin
LOITER_DURATION_RANGE = (25, 40)   # how long they wander before resuming patrol
LOITER_RADIUS = 2.0                # meters, how far they wander from the center point

# Zone camera positions, reused as loiter centers (from earlier camera placement)
ZONE_CENTERS = {
    "Zone_A_NorthEast": (28.67, 28.73),
    "Zone_B_SouthEast": (28.97, -7.97),
    "Zone_C_SouthWest": (-28.71, -9.28),
    "Zone_D_NorthWest": (-28.7, 28.73),
}

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
ARRIVAL_RADIUS = 2.0          # meters - how close counts as "arrived" for triggering a scan

# Robot gets its own patrol loop, pulled in from the walls compared to the
# people's path - it's a bigger vehicle and (being kinematic) doesn't get
# physically stopped by collisions, it'll just visually clip through walls
# if driven too close to them.
ROBOT_WAYPOINTS = [
    (20, 20, 0),
    (20, 0, 0),
    (-20, 0, 0),
    (-20, 20, 0),
]

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

# Isaac Sim's ROS2 bridge extension needs its OWN internal environment
# variables set (pointing at its bundled ROS2 libraries) before it starts -
# unrelated to the WSL/Jazzy discovery env vars set by set_ros_env.ps1. Must
# be set BEFORE `from isaacsim import SimulationApp` - confirmed in
# headless_slam_session.py.
import os
os.environ["ROS_DISTRO"] = "humble"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["PATH"] = os.environ["PATH"] + ";c:/isaacsim/exts/isaacsim.ros2.core/humble/lib"
# Silence the ROS2 bridge's own logger (separate from the carb log-level
# settings below - the "[PoseTree] ... getObjectType eInvalid" spam comes
# from here, not carb, so it wasn't being caught by /log/level=Error).
# These are non-fatal warnings from the known broken TF frames on this
# robot asset (see NOTES.md) - harmless, just very noisy.
os.environ["RCUTILS_LOGGING_SEVERITY_THRESHOLD"] = "ERROR"

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
    "width": 640,
    "height": 480,
    "/log/level": "Error",
    "/log/fileLogLevel": "Error",
    "/log/outputStreamLevel": "Error",
    "/rtx/scenedb/maxHistoryTransformCount": 512,
    "/rtx/pathtracing/enabled": False,
    "/rtx/reflections/enabled": False,
    "/rtx/ambientOcclusion/enabled": False,
    "/rtx/indirectDiffuseGI/enabled": False,
    "/rtx/directLighting/sampledLighting/enabled": False,
    "/rtx/post/dlss/execMode": 0,
})

import carb
# Per-channel log suppression still needs to happen via Python calls (no
# launch-config equivalent for arbitrary channel names), but now runs
# immediately after SimulationApp() instead of after enabling the ROS2
# bridge extension, so it's in place before that extension's nodes start
# ticking and logging.
for _channel in ("isaacsim.ros2.nodes", "isaacsim.ros2.bridge", "isaacsim.ros2.core",
                  "omni.hydra", "rtx.hydra", "omni.syntheticdata.plugin",
                  "isaacsim.sensors.camera.camera", "rtx.scenedb.plugin",
                  "isaacsim.ros2.core.impl.camera_info_utils", "omni.timeline.plugin",
                  "carb"):
    try:
        carb.settings.get_settings().set(f"/log/channels/{_channel}/level", "Error")
        carb.settings.get_settings().set(f"/log/channels/{_channel}/enabled", False)
    except Exception:
        pass
try:
    import omni.log
    for _channel in ("isaacsim.ros2.nodes", "omni.hydra", "rtx.hydra"):
        omni.log.set_channel_enabled(_channel, False, omni.log.SettingBehavior.OVERRIDE)
except Exception:
    pass

# Explicitly enable the ROS2 bridge extension - toggling it on manually in
# the GUI only applies to that running session and does NOT carry over to a
# fresh script launch. Without this, /odom, /cmd_vel, /tf etc. are absent
# and Nav2 has nothing to navigate with.
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
print("ROS2 bridge extension explicitly enabled.")
for _ in range(20):
    simulation_app.update()

import omni.usd
import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics
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


def update_person_position(mover, elapsed):
    # Frozen in place from a recent fall detection - skip all movement,
    # patrol and loiter alike, until the pause expires.
    if elapsed < mover.get("paused_until", 0):
        return

    # If this mover is in their assigned loiter window, wander in small
    # circles near the loiter center instead of following the patrol path.
    loiter_start = mover.get("loiter_start")
    if loiter_start is not None:
        loiter_end = loiter_start + mover["loiter_duration"]
        if loiter_start <= elapsed <= loiter_end:
            cx, cy = mover["loiter_center"]
            x = cx + LOITER_RADIUS * math.sin(elapsed * 0.7)
            y = cy + LOITER_RADIUS * math.cos(elapsed * 0.5)
            set_translate(mover["prim"], (x, y, 0))
            return

    personal_elapsed = elapsed - mover["start_delay"]
    if personal_elapsed < 0:
        return

    total_leg_time = SECONDS_PER_LEG
    path = mover["path"]
    full_loop_time = total_leg_time * (len(path) - 1)

    if LOOP_PATROL:
        personal_elapsed = personal_elapsed % full_loop_time

    leg_index = int(personal_elapsed // total_leg_time)
    leg_t = (personal_elapsed % total_leg_time) / total_leg_time

    if leg_index >= len(path) - 1:
        set_translate(mover["prim"], path[-1])
        return

    p1 = path[leg_index]
    p2 = path[leg_index + 1]
    new_pos = (
        p1[0] + (p2[0] - p1[0]) * leg_t,
        p1[1] + (p2[1] - p1[1]) * leg_t,
        p1[2] + (p2[2] - p1[2]) * leg_t,
    )
    set_translate(mover["prim"], new_pos)


def pause_nearest_mover(movers, zone_name, elapsed):
    """Freezes whichever patrol character is currently closest to the given
    zone's coordinates, for FALL_PAUSE_DURATION seconds - simple stand-in
    for 'the person who fell stays down' until real fall animation exists."""
    zone_pos = ZONE_CENTERS.get(zone_name)
    if zone_pos is None or not movers:
        return

    nearest = min(movers, key=lambda m: (
        (get_translate(m["prim"])[0] - zone_pos[0]) ** 2 +
        (get_translate(m["prim"])[1] - zone_pos[1]) ** 2
    ))
    nearest["paused_until"] = elapsed + FALL_PAUSE_DURATION
    print(f"  [{zone_name}] freezing {nearest['name']} in place for {FALL_PAUSE_DURATION}s (simulating staying down)")


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


def scan_robot(camera_path, authorized_ids, alert):
    """Robot's on-arrival scan: same YOLO -> crop -> DeepFace -> authorization
    pipeline as the fixed checkpoint camera, but mobile - this is the actual
    'double-check' behavior: a zone camera flagged something, the robot goes
    there and confirms identity/authorization up close."""
    print(f"  [ROBOT] arrived at {alert['zone']} ({alert['reason']}) - scanning...")
    bgr = capture_frame(camera_path, resolution=(1280, 720))
    if bgr is None:
        print("  [ROBOT] no frame captured")
        return

    frame_path = os.path.join(os.environ["TEMP"], "robot_scan.jpg")
    cv2.imwrite(frame_path, bgr)
    yolo_result = run_subprocess("yolo_detect.py", [frame_path])
    detections = yolo_result.get("detections", []) if not yolo_result.get("error") else []
    if not detections:
        print("  [ROBOT] nobody found at the alert location")
        append_log_entry({
            "event_type": "robot_response", "found": False,
            "alert_reason": alert.get("reason"), "zone": alert.get("zone"),
            "timestamp": datetime.now(timezone.utc).isoformat()
        })
        return

    best_detection = max(detections, key=lambda d: d["confidence"])
    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return

    crop_path = os.path.join(os.environ["TEMP"], "robot_crop.jpg")
    cv2.imwrite(crop_path, crop)
    debug_name = f"{datetime.now().strftime('%H%M%S')}_robot_crop.jpg"
    cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    reid_result = run_subprocess("deepface_reid.py", [crop_path, KNOWN_FACES_DIR])
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
        print(f"  [ROBOT] could not get a clear face: {reid_result.get('reason')}")

    if person_id:
        save_appearance_profile(person_id, crop)
        is_authorized = person_id in authorized_ids
        print(f"  [ROBOT] double-check result: {person_id} -> {'AUTHORIZED' if is_authorized else 'NOT AUTHORIZED'}")
        append_log_entry({
            "event_type": "robot_response", "found": True,
            "person_id": person_id, "authorized": is_authorized,
            "alert_reason": alert.get("reason"), "zone": alert.get("zone"),
            "timestamp": datetime.now(timezone.utc).isoformat()
        })


def update_robot(robot_state, elapsed, authorized_ids):
    """The robot is no longer moved by this script at all - it's driven by
    real PhysX physics under Nav2's control, via the WSL-side
    dispatch_bridge.py sending real NavigateToPose goals (patrol waypoints
    when idle, dispatch_alert coords when one exists - see PATROL_WAYPOINTS_FILE
    and EVENT_LOG_FILE). This function's only job is to watch the robot's
    REAL position and, once it's physically close enough to an unhandled
    dispatch alert's coordinates, trigger the on-arrival checkpoint-style scan.
    """
    if elapsed - robot_state["last_check"] < ROBOT_CHECK_INTERVAL:
        return
    robot_state["last_check"] = elapsed

    alert = find_next_unhandled_alert(robot_state["handled_timestamps"])
    if alert is None:
        return

    # Query the LIVE physics position directly via RigidPrim, not the USD
    # stage's authored xform attributes. PhysX runs on a fast internal data
    # layer ("Fabric") for performance and does not reliably write results
    # back into the USD stage attributes that UsdGeom.Xformable reads from -
    # the robot visually moves fine (rendering reads from Fabric directly),
    # but get_translate() was reading stale/default data, so arrival never
    # triggered no matter how close the robot actually got. get_world_poses()
    # queries the physics simulation state directly and is reliable.
    positions, _ = robot_state["rigid_prim"].get_world_poses()
    current = positions[0]
    target = alert["coords"]
    dist = math.hypot(current[0] - target[0], current[1] - target[1])
    print(f"  [ROBOT] checking arrival: at ({current[0]:.2f}, {current[1]:.2f}), "
          f"target ({target[0]:.2f}, {target[1]:.2f}), dist={dist:.2f}m")
    if dist <= ARRIVAL_RADIUS:
        scan_robot(ROBOT_CAMERA_PATH, authorized_ids, alert)
        robot_state["handled_timestamps"].add(alert["timestamp"])


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


def capture_frame(camera_path, resolution=(640, 480)):
    # Reuse one Camera object per path instead of creating a new one (and a
    # new render product/annotator) on every single scan call - doing that
    # every 8s for 5 cameras leaked render products, overran internal render
    # buffers ("sequence size exceeds remaining buffer" spam), and stalled
    # the physics step rate badly enough to starve the ROS2 odom publisher.
    camera = _camera_cache.get(camera_path)
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

    rgba = None
    for i in range(60):
        simulation_app.update()
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            break

    if rgba is None or rgba.size == 0:
        return None

    rgb = rgba[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def run_subprocess(script_name, args):
    script_path = os.path.join(PROJECT_DIR, script_name)
    arg_str = " ".join(f'"{a}"' for a in args)
    # Was subprocess.run() (fully blocking) - during that block,
    # simulation_app.update() never ran, so PhysX never stepped and Nav2's
    # /cmd_vel commands never actually got applied to the robot, even though
    # they kept arriving fine on the ROS2 wire (which is why `ros2 topic hz`
    # looked healthy while the robot barely moved). Using Popen + polling
    # keeps physics/rendering stepping the whole time this runs.
    proc = subprocess.Popen(
        f'"C:\\isaacsim\\python.bat" "{script_path}" {arg_str}',
        shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding="utf-8", errors="replace"
    )
    while proc.poll() is None:
        simulation_app.update()
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


def scan_zone_camera(camera_path, zone_name):
    print(f"  scanning {zone_name}...")
    bgr = capture_frame(camera_path)
    if bgr is None:
        return None

    tag = zone_name.replace(" ", "_")
    frame_path = os.path.join(os.environ["TEMP"], f"scan_{tag}.jpg")
    cv2.imwrite(frame_path, bgr)

    yolo_result = run_subprocess("yolo_detect.py", [frame_path])
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
            "coords": ZONE_CENTERS.get(zone_name),
            "timestamp": datetime.now(timezone.utc).isoformat()
        })

    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return {"error": "empty crop", "fell": fell}

    crop_path = os.path.join(os.environ["TEMP"], f"crop_{tag}.jpg")
    cv2.imwrite(crop_path, crop)

    debug_name = f"{datetime.now().strftime('%H%M%S')}_{tag}_crop.jpg"
    cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    reid_result = run_subprocess("deepface_reid.py", [crop_path, KNOWN_FACES_DIR])
    if reid_result.get("status") == "match":
        matched_filename = os.path.basename(reid_result["matched_path"])
        person_id = os.path.splitext(matched_filename)[0]
        return {"person_id": person_id, "distance": reid_result["distance"], "match_type": "face", "fell": fell}
    else:
        # Face detection failed or didn't match - fall back to clothing/color
        # signature, which is far more reliable from wide zone camera angles.
        appearance_id, appearance_score = find_appearance_match(crop)
        if appearance_id:
            return {"person_id": appearance_id, "score": appearance_score, "match_type": "clothing", "fell": fell}
        return {"person_id": "unidentified", "reason": reid_result.get("reason", "no match"), "fell": fell}


def scan_checkpoint(authorized_ids):
    print(f"  scanning checkpoint...")
    bgr = capture_frame(CHECKPOINT_PATH, resolution=(1280, 720))
    if bgr is None:
        return

    frame_path = os.path.join(os.environ["TEMP"], "checkpoint_scan.jpg")
    cv2.imwrite(frame_path, bgr)
    yolo_result = run_subprocess("yolo_detect.py", [frame_path])
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

    reid_result = run_subprocess("deepface_reid.py", [crop_path, KNOWN_FACES_DIR])

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
        result = scan_zone_camera(camera_path, zone_name)

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

        person_id = result["person_id"]
        match_type = result.get("match_type", "none")
        tag = f" (via {match_type})" if match_type != "none" else ""
        print(f"  [{zone_name}] {person_id}{tag}")

        # Authorization check happens here too, not just at the checkpoint -
        # if we know who someone is (via face OR clothing) and they're not
        # on the authorized list, that's a dispatchable alert with a real
        # location attached for the robot to go investigate/double-check.
        if person_id not in (None, "unidentified") and person_id not in authorized_ids:
            coords = ZONE_CENTERS.get(zone_name)
            print(f"  [{zone_name}] *** UNAUTHORIZED PRESENCE: {person_id} - dispatch coords {coords} ***")
            append_log_entry({
                "event_type": "dispatch_alert",
                "reason": "unauthorized_zone_presence",
                "person_id": person_id,
                "zone": zone_name,
                "coords": coords,
                "match_type": match_type,
                "timestamp": timestamp
            })

        if check_loitering(zone_name, streak_state):
            coords = ZONE_CENTERS.get(zone_name)
            print(f"  [{zone_name}] *** LOITERING (someone present {LOITER_THRESHOLD}+ sweeps in a row) ***")
            if person_id not in (None, "unidentified"):
                pause_nearest_mover(movers, zone_name, elapsed)
            append_log_entry({
                "event_type": "loitering_alert",
                "person_id": person_id,
                "zone": zone_name,
                "coords": coords,
                "timestamp": timestamp
            })
            append_log_entry({
                "event_type": "dispatch_alert",
                "reason": "loitering",
                "person_id": person_id,
                "zone": zone_name,
                "coords": coords,
                "timestamp": timestamp
            })

        append_log_entry({
            "event_type": "zone",
            "person_id": person_id,
            "zone": zone_name,
            "camera": camera_path,
            "timestamp": timestamp
        })

    scan_checkpoint(authorized_ids)


# ---------- Main ----------

def main():
    os.makedirs(KNOWN_FACES_DIR, exist_ok=True)

    # Clear out the debug folder from the previous run so it doesn't pile up,
    # then recreate it fresh for this run's captures.
    if os.path.exists(DEBUG_DIR):
        shutil.rmtree(DEBUG_DIR)
    os.makedirs(DEBUG_DIR, exist_ok=True)

    with open(CAMERA_ZONES_FILE, "r") as f:
        camera_zones = json.load(f)

    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    # The piper arm is set up as a USD payload - unloading it excludes it
    # from the scene entirely (non-destructive, can be re-enabled later via
    # .Load()) which stops the ungoverned arm-flailing we saw, since there's
    # simply no arm geometry/physics present to flail.
    piper_arm_prim = stage.GetPrimAtPath("/World/mobile_manipulator_ros/piper_arm")
    if piper_arm_prim.IsValid():
        piper_arm_prim.Unload()
        print("Piper arm payload unloaded (disabled).")
    else:
        print("WARNING: piper_arm prim not found - nothing to unload.")

    print("Attempting to repair broken TF publisher prim references...")
    repair_broken_tf_publisher_targets(stage)

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
    for name, info in PEOPLE.items():
        prim = stage.GetPrimAtPath(info["prim_path"])
        if not prim.IsValid():
            print(f"WARNING: {name} prim not found at {info['prim_path']} - skipping")
            continue
        disable_physics_recursive(prim)
        start = get_translate(prim)
        full_path = [(start[0], start[1], start[2])] + WAYPOINTS
        movers.append({
            "name": name, "prim": prim, "path": full_path, "start_delay": info["start_delay"],
            "loiter_start": None, "loiter_duration": None, "loiter_center": None,
            "paused_until": 0
        })

    # --- Robot setup ---
    # The robot runs in REAL PHYSICS mode for the whole session (kinematic
    # OFF), driven by Nav2 through the confirmed-working velocity_smoother
    # relay bypass - see NOTES.md. Per prior debugging, the kinematic flag
    # must be set BEFORE timeline.play() to reliably take effect, so this
    # happens once here and is never toggled at runtime.
    robot_state = None
    robot_base_prim = stage.GetPrimAtPath(ROBOT_BASE_PATH)
    if robot_base_prim.IsValid() and robot_base_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(robot_base_prim)
        attr = rigid_api.GetKinematicEnabledAttr()
        if attr:
            attr.Set(False)
        else:
            rigid_api.CreateKinematicEnabledAttr(False)

        # Write the patrol waypoints out for the WSL-side dispatch_bridge.py
        # to read, so there's a single source of truth for the patrol route
        # instead of hardcoding it twice.
        with open(PATROL_WAYPOINTS_FILE, "w") as f:
            json.dump({"waypoints": [[x, y] for x, y, _ in ROBOT_WAYPOINTS]}, f, indent=2)

        robot_rigid_prim = RigidPrim(prim_paths_expr=ROBOT_BASE_PATH)

        robot_state = {
            "prim": robot_base_prim,
            "rigid_prim": robot_rigid_prim,
            "last_check": 0,
            "handled_timestamps": set(),
        }
        print(f"Robot found at {ROBOT_BASE_PATH}, set to real physics mode (kinematic off). "
              f"Patrol waypoints written to {PATROL_WAYPOINTS_FILE} for dispatch_bridge.py.")
    else:
        print(f"WARNING: robot base prim not found/valid at {ROBOT_BASE_PATH} - robot dispatch disabled this run.")

    # NOW play the timeline, after all kinematic flags are set - order matters here.
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    # Live per-zone loitering streak tracker, persists across sweeps for the
    # whole run - see check_loitering() for why this replaced a log-based check.
    streak_state = {zone_name: {"streak": 0, "alerted": False} for zone_name in camera_zones.values()}

    if LOITER_ENABLED and movers:
        loiterer = random.choice(movers)
        loiterer["loiter_start"] = random.uniform(*LOITER_START_RANGE)
        loiterer["loiter_duration"] = random.uniform(*LOITER_DURATION_RANGE)
        loiterer["loiter_center"] = random.choice(list(ZONE_CENTERS.values()))
        print(f"*** {loiterer['name']} will loiter near {loiterer['loiter_center']} "
              f"starting at t={loiterer['loiter_start']:.1f}s for {loiterer['loiter_duration']:.1f}s ***")

    print("=== Unified tracking + movement loop started. Press Ctrl+C to stop. ===")

    sim_start = time.time()
    last_sweep_time = 0
    last_tick_elapsed = 0.0
    authorized_ids_cache = load_authorized_ids()

    try:
        while True:
            elapsed = time.time() - sim_start
            dt = elapsed - last_tick_elapsed
            last_tick_elapsed = elapsed

            for m in movers:
                update_person_position(m, elapsed)

            if robot_state is not None:
                update_robot(robot_state, elapsed, authorized_ids_cache)

            simulation_app.update()

            if elapsed - last_sweep_time >= SCAN_INTERVAL_SECONDS:
                authorized_ids_cache = load_authorized_ids()
                run_sweep(camera_zones, authorized_ids_cache, streak_state, movers, elapsed)
                last_sweep_time = elapsed

    except KeyboardInterrupt:
        print("\n=== Stopped by user. ===")
    finally:
        timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()