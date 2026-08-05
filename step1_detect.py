"""
Step 1 - standalone terminal version.
Boots Isaac Sim, opens your stage, captures a camera frame, runs YOLO
person detection via subprocess, and saves an annotated image.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\step1_detect.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\doctor.usd"
HEADLESS = False

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
import subprocess
import json
import os
from PIL import Image, ImageDraw
from isaacsim.sensors.camera import Camera
import carb


def main():
    # --- Open your saved stage ---
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)

    for _ in range(60):
        simulation_app.update()

    # --- Start the timeline — camera rendering needs this playing, not just app ticks ---
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    for _ in range(20):
        simulation_app.update()

    # --- Setup camera ---
    camera = Camera(prim_path="/World/Camera", resolution=(640, 480))
    camera.initialize()

    rgba = None
    max_attempts = 300
    for i in range(max_attempts):
        simulation_app.update()
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            print(f"Got a valid frame after {i + 1} ticks")
            break

    if rgba is None or rgba.size == 0:
        carb.log_error(f"No frame captured after {max_attempts} ticks.")
        timeline.stop()
        return

    temp_dir = os.environ["TEMP"]
    frame_path = os.path.join(temp_dir, "isaac_frame.png")

    img = Image.fromarray(rgba[:, :, :3])
    img.save(frame_path)
    print(f"Frame captured and saved to: {frame_path}")

    # --- Run YOLO detection via subprocess ---
    detect_script = r"C:\isaacsim\projects\surveillance-proj\yolo_detect.py"
    result = subprocess.run(
        f'"C:\\isaacsim\\python.bat" "{detect_script}" "{frame_path}"',
        shell=True, capture_output=True, text=True,
        encoding="utf-8", errors="replace"
    )

    if result.returncode != 0:
        carb.log_error(f"Detection subprocess failed: {result.stderr}")
        timeline.stop()
        return

    try:
        output = json.loads(result.stdout.strip().splitlines()[-1])
        detections = output.get("detections", [])
        print(f"Found {len(detections)} person(s)")

        draw = ImageDraw.Draw(img)
        for det in detections:
            draw.rectangle(
                [det["x1"], det["y1"], det["x2"], det["y2"]],
                outline="red", width=3
            )
            draw.text((det["x1"], det["y1"] - 15), f'{det["confidence"]:.2f}', fill="red")

        annotated_path = os.path.join(temp_dir, "isaac_frame_annotated.png")
        img.save(annotated_path)
        print(f"Annotated image saved to: {annotated_path}")

    except (json.JSONDecodeError, IndexError) as e:
        carb.log_error(f"Failed to parse detection output: {e}\nRaw stdout: {result.stdout}")

    timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()