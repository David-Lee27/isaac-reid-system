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

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom, UsdPhysics
import cv2
import numpy as np
import subprocess
import json
import os
import shutil
import time
import random
import math
from datetime import datetime, timezone
from isaacsim.sensors.camera import Camera
import carb


# ---------- Movement helpers ----------

def get_translate(prim):
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op.Get()
    return Gf.Vec3d(0, 0, 0)


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


def capture_frame(camera_path, resolution=(640, 480)):
    camera = Camera(prim_path=camera_path, resolution=resolution)
    camera.initialize()

    rgba = None
    for i in range(150):
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
    result = subprocess.run(
        f'"C:\\isaacsim\\python.bat" "{script_path}" {arg_str}',
        shell=True, capture_output=True, text=True,
        encoding="utf-8", errors="replace"
    )
    if result.returncode != 0:
        return {"error": result.stderr}
    try:
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        return {"error": f"failed to parse output: {result.stdout}"}


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

    if check_for_fall(best_detection):
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
        return {"error": "empty crop"}

    crop_path = os.path.join(os.environ["TEMP"], f"crop_{tag}.jpg")
    cv2.imwrite(crop_path, crop)

    debug_name = f"{datetime.now().strftime('%H%M%S')}_{tag}_crop.jpg"
    cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    reid_result = run_subprocess("deepface_reid.py", [crop_path, KNOWN_FACES_DIR])
    if reid_result.get("status") == "match":
        matched_filename = os.path.basename(reid_result["matched_path"])
        person_id = os.path.splitext(matched_filename)[0]
        return {"person_id": person_id, "distance": reid_result["distance"], "match_type": "face"}
    else:
        # Face detection failed or didn't match - fall back to clothing/color
        # signature, which is far more reliable from wide zone camera angles.
        appearance_id, appearance_score = find_appearance_match(crop)
        if appearance_id:
            return {"person_id": appearance_id, "score": appearance_score, "match_type": "clothing"}
        return {"person_id": "unidentified", "reason": reid_result.get("reason", "no match")}


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


def run_sweep(camera_zones, authorized_ids, streak_state):
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
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

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
            "loiter_start": None, "loiter_duration": None, "loiter_center": None
        })

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

    try:
        while True:
            elapsed = time.time() - sim_start

            for m in movers:
                update_person_position(m, elapsed)
            simulation_app.update()

            if elapsed - last_sweep_time >= SCAN_INTERVAL_SECONDS:
                authorized_ids = load_authorized_ids()
                run_sweep(camera_zones, authorized_ids, streak_state)
                last_sweep_time = elapsed

    except KeyboardInterrupt:
        print("\n=== Stopped by user. ===")
    finally:
        timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()