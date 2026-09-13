r"""
diagnose_wheel_kinematics.py

/cmd_vel is asking for mostly-forward motion (linear.x ramping up,
angular.z heading toward 0) but /odom shows almost no forward progress and
a constant nonzero angular.z - the robot is effectively spinning in place
regardless of what Nav2 commands. That pattern is the classic symptom of a
wrong wheelRadius or wheelDistance (track width) on the differential drive
conversion: those two numbers are what turn a (linear, angular) velocity
command into (left_wheel_speed, right_wheel_speed), and if either is off
from the robot's REAL physical geometry, "drive straight" gets converted
into asymmetric wheel speeds that produce net rotation instead.

This script does NOT change anything. It just prints, side by side:
  1. Whatever wheelRadius/wheelDistance are currently AUTHORED on the
     differential_controller OmniGraph node.
  2. The robot's ACTUAL measured wheel radius (from each wheel mesh's
     bounding box) and ACTUAL measured track width (straight-line distance
     between the two wheel prims' world positions).

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_wheel_kinematics.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1_animated.usd"
NODE_PATH = "/World/mobile_manipulator_ros/Graph/differential_controller/ActionGraph/differential_controller"
ROBOT_PRIM_PATH = "/World/mobile_manipulator_ros"


def find_wheel_prims(stage, robot_prim_path):
    """Walks the robot's hierarchy looking for prims with 'wheel' in the
    name (case-insensitive) - doesn't assume exact names since those
    weren't confirmed anywhere in this project's notes."""
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    wheels = []
    if not robot_prim.IsValid():
        print(f"[ERROR] robot prim not found at {robot_prim_path}")
        return wheels
    for prim in Usd.PrimRange(robot_prim):
        if "wheel" in prim.GetName().lower():
            wheels.append(prim)
    return wheels


def get_world_translate(prim):
    xform = UsdGeom.Xformable(prim)
    matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return matrix.ExtractTranslation()


def get_prim_radius_estimate(prim):
    """Estimates a wheel's radius from its world-space bounding box - takes
    half of whichever of the box's 3 dimensions is smallest, on the
    (usually correct for a wheel-shaped mesh) assumption that a wheel's
    thickness (axle direction) is its smallest dimension, so the other two
    (both close to the wheel's diameter) are what we want, and radius is
    half of that."""
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bbox = bbox_cache.ComputeWorldBound(prim)
    box_range = bbox.ComputeAlignedRange()
    size = box_range.GetSize()
    dims = sorted([size[0], size[1], size[2]])
    diameter_estimate = (dims[1] + dims[2]) / 2.0
    return diameter_estimate / 2.0


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    print("=" * 70)
    print("PART 1: currently AUTHORED values on the differential_controller node")
    print("=" * 70)
    node_prim = stage.GetPrimAtPath(NODE_PATH)
    if not node_prim.IsValid():
        print(f"[ERROR] node not found at {NODE_PATH}")
    else:
        for attr in sorted(node_prim.GetAttributes(), key=lambda a: a.GetName()):
            if attr.GetName().startswith("inputs:"):
                print(f"  {attr.GetName()}: {attr.Get()}")

    print()
    print("=" * 70)
    print("PART 2: ACTUAL measured wheel geometry from the stage")
    print("=" * 70)
    wheels = find_wheel_prims(stage, ROBOT_PRIM_PATH)
    if len(wheels) < 2:
        print(f"[WARN] found only {len(wheels)} prim(s) with 'wheel' in the name - "
              f"expected at least 2 (left/right). Names found:")
        for w in wheels:
            print(f"    {w.GetPath()}")
    else:
        print(f"Found {len(wheels)} wheel-named prim(s):")
        for w in wheels:
            pos = get_world_translate(w)
            radius = get_prim_radius_estimate(w)
            print(f"  {w.GetPath()}")
            print(f"    world position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
            print(f"    estimated radius (from bounding box): {radius:.4f} m")

        left = next((w for w in wheels if "left" in w.GetName().lower()), None)
        right = next((w for w in wheels if "right" in w.GetName().lower()), None)
        if left is not None and right is not None:
            p1 = get_world_translate(left)
            p2 = get_world_translate(right)
            track_width = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2) ** 0.5
            print(f"\n  ACTUAL measured track width ({left.GetName()} <-> {right.GetName()}): {track_width:.4f} m")
        else:
            print("\n  [WARN] couldn't confidently identify a left/right pair by name - "
                  "check the full wheel list above and measure track width manually if needed.")

    print()
    print("=" * 70)
    print("Compare PART 1's inputs:wheelRadius / inputs:wheelDistance (or similarly")
    print("named attributes - check the full printout above for the real names)")
    print("against PART 2's measured values. A mismatch there - not torque, not")
    print("render settings - would explain forward commands turning into rotation.")
    print("=" * 70)


if __name__ == "__main__":
    main()
    simulation_app.close()
