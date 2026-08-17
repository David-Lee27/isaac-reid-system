r"""
Test: deactivate the PhysicsScene prim entirely (so nothing simulates,
nothing falls) while still calling timeline.play() (which the camera
pipeline needs). Tests both robot position stability AND camera capture
together, to see if this resolves the conflict between the two.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\robot_combined_test.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False
ROBOT_PRIM_PATH = "/World/mobile_manipulator_ros"
ROBOT_CAMERA_PATH = "/World/mobile_manipulator_ros/camera_front/Gemini335L/Gemini335L/camera_rgb/Camera_rgb"
PHYSICS_SCENE_PATH = "/PhysicsScene"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom
import cv2
import os
import time
from isaacsim.sensors.camera import Camera
import carb


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


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    physics_scene = stage.GetPrimAtPath(PHYSICS_SCENE_PATH)
    if physics_scene.IsValid():
        physics_scene.SetActive(False)
        print(f"Deactivated PhysicsScene at {PHYSICS_SCENE_PATH}")
    else:
        print(f"WARNING: no PhysicsScene found at {PHYSICS_SCENE_PATH}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        print(f"ERROR: robot prim not found at {ROBOT_PRIM_PATH}")
        return

    start_pos = get_translate(robot_prim)
    print(f"Robot starting position: {start_pos}")

    # Actually DRIVE it through a couple waypoints (not just sit idle) -
    # this replicates the real scenario that caused falling before, since
    # active transform overrides while physics steps is what seemed to
    # destabilize it, not just passive gravity on an idle body.
    test_waypoints = [(27.46, 16.33, 0), (29.06, 27.76, 0)]
    print(f"Driving through {len(test_waypoints)} test waypoints...")
    position = (start_pos[0], start_pos[1], 0)
    for wp in test_waypoints:
        drive_start = time.time()
        while time.time() - drive_start < 4:
            t = (time.time() - drive_start) / 4
            x = position[0] + (wp[0] - position[0]) * min(t, 1.0)
            y = position[1] + (wp[1] - position[1]) * min(t, 1.0)
            set_translate(robot_prim, (x, y, 0))
            simulation_app.update()
        position = wp
        actual_pos = get_translate(robot_prim)
        print(f"  Target {wp} -> actual position now: {actual_pos}")
        if abs(actual_pos[2]) > 0.5:
            print("  *** Z HEIGHT DRIFTED - it's falling/sinking while being driven ***")

    # Now test the camera
    camera = Camera(prim_path=ROBOT_CAMERA_PATH, resolution=(640, 480))
    camera.initialize()

    rgba = None
    for i in range(300):
        simulation_app.update()
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            print(f"Got a valid camera frame after {i + 1} ticks")
            break

    if rgba is None or rgba.size == 0:
        print("FAILED: camera still could not capture a frame.")
    else:
        rgb = rgba[:, :, :3]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        out_path = os.path.join(os.environ["TEMP"], "robot_combined_test.jpg")
        cv2.imwrite(out_path, bgr)
        print(f"SUCCESS: camera frame saved to {out_path}")

    timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()
