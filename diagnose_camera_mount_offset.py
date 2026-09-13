r"""
diagnose_camera_mount_offset.py

Orbit standpoints are producing views that are either point-blank on the
person (radius too small) or pointed at a wall/empty floor entirely (wrong
angle) - the second one means setting base_footprint's yaw to "face the
target" is NOT actually aiming the camera at the target. That only makes
sense if the camera is mounted at some rotational offset relative to
base_footprint that hasn't been accounted for.

This prints the camera's LOCAL transform (translation + rotation) relative
to base_footprint, so the actual offset can be read directly instead of
guessed at again.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_camera_mount_offset.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1_animated.usd"
BASE_PATH = "/World/mobile_manipulator_ros/base_footprint"
CAMERA_PATH = "/World/mobile_manipulator_ros/base_footprint/sensors/camera_front/Gemini335L/Gemini335L/camera_rgb/Camera_rgb"


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    base_prim = stage.GetPrimAtPath(BASE_PATH)
    cam_prim = stage.GetPrimAtPath(CAMERA_PATH)

    if not base_prim.IsValid():
        print(f"[ERROR] base prim not found at {BASE_PATH}")
        return
    if not cam_prim.IsValid():
        print(f"[ERROR] camera prim not found at {CAMERA_PATH}")
        return

    base_xform = UsdGeom.Xformable(base_prim)
    cam_xform = UsdGeom.Xformable(cam_prim)

    base_world = base_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    cam_world = cam_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # Camera's transform relative to base = base_world^-1 * cam_world
    cam_relative_to_base = cam_world * base_world.GetInverse()

    print("=" * 70)
    print("Camera transform relative to base_footprint:")
    print("=" * 70)
    translation = cam_relative_to_base.ExtractTranslation()
    print(f"  Relative translation: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")

    rotation = cam_relative_to_base.ExtractRotation()
    # Print as euler angles in a couple common orders for interpretability
    print(f"  Rotation matrix decomposition (Gf.Rotation quaternion axis/angle):")
    print(f"    axis: {rotation.axis}, angle: {rotation.angle:.2f} deg")

    # Also directly print each individual authored xformOp on the camera and
    # every intermediate prim in the chain from base_footprint down to the
    # camera - one of THESE is very likely where a fixed rotation offset is
    # actually authored (a mount bracket prim with its own rotateXYZ, etc.)
    print()
    print("=" * 70)
    print("Every prim in the chain from base_footprint to the camera, with its OWN authored ops:")
    print("=" * 70)
    path_parts = CAMERA_PATH.replace(BASE_PATH + "/", "").split("/")
    current_path = BASE_PATH
    for part in path_parts:
        current_path = current_path + "/" + part
        prim = stage.GetPrimAtPath(current_path)
        if not prim.IsValid():
            print(f"  {current_path}: [INVALID PRIM]")
            continue
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        print(f"  {current_path}:")
        if not ops:
            print(f"    (no authored xformOps)")
        for op in ops:
            print(f"    {op.GetOpType()}: {op.Get()}")

    print()
    print("=" * 70)
    print("For reference, base_footprint's own world position/rotation right now:")
    print("=" * 70)
    base_pos = base_world.ExtractTranslation()
    base_rot = base_world.ExtractRotation()
    print(f"  world position: ({base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f})")
    print(f"  world rotation axis/angle: {base_rot.axis}, {base_rot.angle:.2f} deg")


if __name__ == "__main__":
    main()
    simulation_app.close()
