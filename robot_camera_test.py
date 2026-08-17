r"""
Quick isolated test: does the robot's camera capture a frame WITHOUT
timeline.play() running? This determines whether skipping physics (which
fixed the robot falling through the floor) breaks camera rendering.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\robot_camera_test.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False
ROBOT_CAMERA_PATH = "/World/mobile_manipulator_ros/camera_front/Gemini335L/Gemini335L/camera_rgb/Camera_rgb"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import cv2
import os
from isaacsim.sensors.camera import Camera
import carb


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    # Deliberately no timeline.play() - testing whether this camera works
    # without it, same as the robot movement test.

    camera = Camera(prim_path=ROBOT_CAMERA_PATH, resolution=(640, 480))
    camera.initialize()

    rgba = None
    for i in range(300):
        simulation_app.update()
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            print(f"Got a valid frame after {i + 1} ticks WITHOUT timeline.play()")
            break

    if rgba is None or rgba.size == 0:
        print("FAILED: no frame captured after 300 ticks without timeline.play().")
        print("This means the robot's camera DOES need the timeline running.")
        return

    rgb = rgba[:, :, :3]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    out_path = os.path.join(os.environ["TEMP"], "robot_camera_test.jpg")
    cv2.imwrite(out_path, bgr)
    print(f"SUCCESS: frame captured and saved to {out_path}")


if __name__ == "__main__":
    main()
    simulation_app.close()
