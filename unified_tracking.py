r"""
Unified tracking + movement loop.
People continuously patrol their waypoint loop WHILE the system sweeps all
4 zone cameras + checkpoint camera for detection/re-ID/authorization, all
in one running Isaac Sim instance. Press Ctrl+C to stop.

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

CHECKPOINT_NAME = "EntryCamera"
CHECKPOINT_PATH = "/World/EntryCamera"

SCAN_INTERVAL_SECONDS = 8  # how often to run a full camera sweep

# --- People + patrol path (from move_people.py) ---
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
LOOP_PATROL = True  # once they finish the path, start over from the beginning

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom, UsdPhysics
import cv2
import subprocess
import json
import os
import time
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
    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return {"error": "empty crop"}

    crop_path = os.path.join(os.environ["TEMP"], f"crop_{tag}.jpg")
    cv2.imwrite(crop_path, crop)

    reid_result = run_subprocess("deepface_reid.py", [crop_path, KNOWN_FACES_DIR])
    if reid_result.get("status") == "match":
        matched_filename = os.path.basename(reid_result["matched_path"])
        return {"person_id": os.path.splitext(matched_filename)[0], "distance": reid_result["distance"]}
    else:
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
    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return

    crop_path = os.path.join(os.environ["TEMP"], "checkpoint_crop.jpg")
    cv2.imwrite(crop_path, crop)
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
        is_authorized = person_id in authorized_ids
        print(f"  [CHECKPOINT] {person_id} -> {'AUTHORIZED' if is_authorized else 'NOT AUTHORIZED'}")
        append_log_entry({
            "event_type": "checkpoint",
            "person_id": person_id,
            "checkpoint": CHECKPOINT_NAME,
            "authorized": is_authorized,
            "timestamp": datetime.now(timezone.utc).isoformat()
        })


def run_sweep(camera_zones, authorized_ids):
    timestamp = datetime.now(timezone.utc).isoformat()
    print(f"\n--- Sweep at {timestamp} ---")

    for camera_path, zone_name in camera_zones.items():
        result = scan_zone_camera(camera_path, zone_name)
        if result is None:
            continue
        if result.get("error"):
            print(f"  [{zone_name}] error: {result['error']}")
            continue
        print(f"  [{zone_name}] {result['person_id']}")
        append_log_entry({
            "event_type": "zone",
            "person_id": result["person_id"],
            "zone": zone_name,
            "camera": camera_path,
            "timestamp": timestamp
        })

    scan_checkpoint(authorized_ids)


# ---------- Main ----------

def main():
    os.makedirs(KNOWN_FACES_DIR, exist_ok=True)

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

    # Set up movers
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
            "name": name, "prim": prim, "path": full_path, "start_delay": info["start_delay"]
        })

    print("=== Unified tracking + movement loop started. Press Ctrl+C to stop. ===")

    sim_start = time.time()
    last_sweep_time = 0

    try:
        while True:
            elapsed = time.time() - sim_start

            # Continuously advance people along their patrol path
            for m in movers:
                update_person_position(m, elapsed)
            simulation_app.update()

            # Periodically run a full camera sweep
            if elapsed - last_sweep_time >= SCAN_INTERVAL_SECONDS:
                authorized_ids = load_authorized_ids()
                run_sweep(camera_zones, authorized_ids)
                last_sweep_time = elapsed

    except KeyboardInterrupt:
        print("\n=== Stopped by user. ===")
    finally:
        timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()