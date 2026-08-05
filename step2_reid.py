"""
Step 2 - Face-based re-identification with persistent IDs.
Captures a frame, checks it against known faces, assigns a persistent
ID (reusing an existing one on match, creating a new one otherwise).

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\step2_reid.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\doctor.usd"
HEADLESS = False
PROJECT_DIR = r"C:\isaacsim\projects\surveillance-proj"
KNOWN_FACES_DIR = PROJECT_DIR + r"\known_faces"
ID_COUNTER_FILE = PROJECT_DIR + r"\next_id.txt"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
import cv2
import subprocess
import json
import os
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


def main():
    os.makedirs(KNOWN_FACES_DIR, exist_ok=True)

    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    camera = Camera(prim_path="/World/Camera", resolution=(640, 480))
    camera.initialize()

    rgba = None
    for i in range(300):
        simulation_app.update()
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            print(f"Got a valid frame after {i + 1} ticks")
            break

    if rgba is None or rgba.size == 0:
        carb.log_error("No frame captured.")
        timeline.stop()
        return

    rgb = rgba[:, :, :3]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    capture_path = os.path.join(os.environ["TEMP"], "reid_capture.jpg")
    cv2.imwrite(capture_path, bgr)
    print(f"Captured frame saved to: {capture_path}")

    # --- Run re-ID check via subprocess ---
    reid_script = os.path.join(PROJECT_DIR, "deepface_reid.py")
    result = subprocess.run(
        f'"C:\\isaacsim\\python.bat" "{reid_script}" "{capture_path}" "{KNOWN_FACES_DIR}"',
        shell=True, capture_output=True, text=True,
        encoding="utf-8", errors="replace"
    )

    if result.returncode != 0:
        carb.log_error(f"Re-ID subprocess failed: {result.stderr}")
        timeline.stop()
        return

    try:
        output = json.loads(result.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        carb.log_error(f"Failed to parse re-ID output. Raw stdout: {result.stdout}")
        timeline.stop()
        return

    if output.get("match_found"):
        matched_path = output["matched_path"]
        matched_filename = os.path.basename(matched_path)
        person_id = os.path.splitext(matched_filename)[0]
        distance = output["distance"]
        print(f"MATCH: existing person {person_id} (distance={distance:.4f})")
    else:
        new_id = f"ID_{get_next_id():04d}"
        new_face_path = os.path.join(KNOWN_FACES_DIR, f"{new_id}.jpg")
        cv2.imwrite(new_face_path, bgr)
        print(f"NEW PERSON: assigned {new_id}, reason: {output.get('reason')}")
        print(f"Saved reference image to: {new_face_path}")

    timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()