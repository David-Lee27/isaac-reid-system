r"""
Real fix attempt: target base_footprint directly (the actual rigid body
the robot's articulation is rooted at - NOT the outer /World/mobile_manipulator_ros
group prim, which we were incorrectly moving before), and set it kinematic
instead of trying to disable rigid body physics entirely (which PhysX
refused for articulation members).

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\robot_kinematic_test.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False
ROBOT_BASE_PATH = "/World/mobile_manipulator_ros/base_footprint"
ROBOT_CAMERA_PATH = "/World/mobile_manipulator_ros/camera_front/Gemini335L/Gemini335L/camera_rgb/Camera_rgb"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom, UsdPhysics
import cv2
import os
import time
from isaacsim.sensors.camera import Camera


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

    base_prim = stage.GetPrimAtPath(ROBOT_BASE_PATH)
    if not base_prim.IsValid():
        print(f"ERROR: base_footprint not found at {ROBOT_BASE_PATH}")
        return

    if base_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(base_prim)
        attr = rigid_api.GetKinematicEnabledAttr()
        if attr:
            attr.Set(True)
        else:
            rigid_api.CreateKinematicEnabledAttr(True)
        print(f"Set kinematicEnabled=True on {ROBOT_BASE_PATH}")
    else:
        print(f"WARNING: {ROBOT_BASE_PATH} has no RigidBodyAPI - unexpected")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    start_pos = get_translate(base_prim)
    print(f"base_footprint starting position: {start_pos}")

    test_waypoints = [(27.46, 16.33, 0), (29.06, 27.76, 0)]
    print(f"Driving THIS prim through {len(test_waypoints)} test waypoints...")
    position = (start_pos[0], start_pos[1], 0)
    for wp in test_waypoints:
        drive_start = time.time()
        while time.time() - drive_start < 4:
            t = (time.time() - drive_start) / 4
            x = position[0] + (wp[0] - position[0]) * min(t, 1.0)
            y = position[1] + (wp[1] - position[1]) * min(t, 1.0)
            set_translate(base_prim, (x, y, 0))
            simulation_app.update()
        position = wp
        actual_pos = get_translate(base_prim)
        print(f"  Target {wp} -> actual position now: {actual_pos}")
        if abs(actual_pos[2]) > 0.5:
            print("  *** Z HEIGHT DRIFTED - still falling/sinking ***")
        else:
            print("  Z height held steady - good.")

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
        print("FAILED: camera could not capture a frame.")
    else:
        rgb = rgba[:, :, :3]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        out_path = os.path.join(os.environ["TEMP"], "robot_kinematic_test.jpg")
        cv2.imwrite(out_path, bgr)
        print(f"SUCCESS: camera frame saved to {out_path}")

    timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()
