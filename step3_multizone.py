"""
Step 3 - Multi-zone tracking (YOLO -> crop -> DeepFace re-ID, properly chained).

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\step3_multizone.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False
PROJECT_DIR = r"C:\isaacsim\projects\surveillance-proj"
KNOWN_FACES_DIR = PROJECT_DIR + r"\known_faces"
ID_COUNTER_FILE = PROJECT_DIR + r"\next_id.txt"
CAMERA_ZONES_FILE = PROJECT_DIR + r"\camera_zones.json"
ZONE_LOG_FILE = PROJECT_DIR + r"\zone_log.json"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
import cv2
import subprocess
import json
import os
from datetime import datetime, timezone
from isaacsim.sensors.camera import Camera
import carb


def get_next_id():
    if not os.path.exists(ID_COUNTER_FILE):
        next_id = 1
    else:
        with open(ID_COUNTER_FILE, "r") as f:
            next_id = int(f.read().strip())
    with open(ID_COUNTER_FILE, "w") as f:
        f.write(str(next_id + 1))
    return next_id


def append_log_entry(entry):
    log = []
    if os.path.exists(ZONE_LOG_FILE):
        with open(ZONE_LOG_FILE, "r") as f:
            try:
                log = json.load(f)
            except json.JSONDecodeError:
                log = []
    log.append(entry)
    with open(ZONE_LOG_FILE, "w") as f:
        json.dump(log, f, indent=2)


def capture_frame(camera_path):
    camera = Camera(prim_path=camera_path, resolution=(640, 480))
    camera.initialize()

    rgba = None
    for i in range(300):
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


def crop_person(bgr_image, detection, padding=20):
    h, w = bgr_image.shape[:2]
    x1 = max(0, int(detection["x1"]) - padding)
    y1 = max(0, int(detection["y1"]) - padding)
    x2 = min(w, int(detection["x2"]) + padding)
    y2 = min(h, int(detection["y2"]) + padding)
    return bgr_image[y1:y2, x1:x2]


def process_zone(camera_path, zone_name):
    print(f"--- {zone_name} ({camera_path}) ---")

    bgr = capture_frame(camera_path)
    if bgr is None:
        print(f"  No frame captured — check camera path/position.\n")
        return

    safe_zone = zone_name.replace(" ", "_")
    frame_path = os.path.join(os.environ["TEMP"], f"frame_{safe_zone}.jpg")
    cv2.imwrite(frame_path, bgr)

    # --- Step 1: YOLO person detection ---
    yolo_result = run_subprocess("yolo_detect.py", [frame_path])
    if yolo_result.get("error"):
        print(f"  YOLO ERROR: {yolo_result['error']}\n")
        return

    detections = yolo_result.get("detections", [])
    if not detections:
        print(f"  No person detected in this zone.\n")
        return

    # Take the highest-confidence detection
    best_detection = max(detections, key=lambda d: d["confidence"])
    crop = crop_person(bgr, best_detection)

    if crop.size == 0:
        print(f"  Crop failed (empty region).\n")
        return

    crop_path = os.path.join(os.environ["TEMP"], f"crop_{safe_zone}.jpg")
    cv2.imwrite(crop_path, crop)

    # --- Step 2: DeepFace re-ID on the CROPPED person, not the full frame ---
    reid_result = run_subprocess("deepface_reid.py", [crop_path, KNOWN_FACES_DIR])
    timestamp = datetime.now(timezone.utc).isoformat()

    if reid_result.get("error"):
        print(f"  RE-ID ERROR: {reid_result['error']}\n")
        return

    if reid_result.get("match_found"):
        matched_filename = os.path.basename(reid_result["matched_path"])
        person_id = os.path.splitext(matched_filename)[0]
        distance = reid_result["distance"]
        print(f"  MATCH: {person_id} (distance={distance:.4f})\n")
    else:
        person_id = f"ID_{get_next_id():04d}"
        new_face_path = os.path.join(KNOWN_FACES_DIR, f"{person_id}.jpg")
        cv2.imwrite(new_face_path, crop)
        print(f"  NEW PERSON: assigned {person_id} ({reid_result.get('reason')})\n")

    append_log_entry({
        "person_id": person_id,
        "zone": zone_name,
        "camera": camera_path,
        "timestamp": timestamp
    })


def main():
    os.makedirs(KNOWN_FACES_DIR, exist_ok=True)

    with open(CAMERA_ZONES_FILE, "r") as f:
        camera_zones = json.load(f)

    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    print(f"\n=== Scanning {len(camera_zones)} zones ===\n")

    for camera_path, zone_name in camera_zones.items():
        process_zone(camera_path, zone_name)

    timeline.stop()
    print(f"=== Done. Log saved to {ZONE_LOG_FILE} ===")


if __name__ == "__main__":
    main()
    simulation_app.close()