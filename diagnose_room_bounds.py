r"""
diagnose_room_bounds.py

The robot walked off the actual lit floor into darkness when patrolling
toward a "zone center" that was just the raw camera prim's XY position. A
camera mounted near/on a wall watching a room is NOT necessarily positioned
over the middle of that room's floor - using its raw XY as a patrol/loiter
target can easily land outside the actual walkable area.

This prints the real world-space bounding box of the floor and every wall
prim in optimized room.usd, so real interior-safe zone center points can be
picked instead of guessed from camera position alone.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_room_bounds.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\optimized room.usd"


def print_bbox(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"  {prim_path}: [INVALID]")
        return
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bbox = bbox_cache.ComputeWorldBound(prim)
    box_range = bbox.ComputeAlignedRange()
    lo = box_range.GetMin()
    hi = box_range.GetMax()
    center = (lo + hi) / 2.0
    print(f"  {prim_path}:")
    print(f"    min: ({lo[0]:.2f}, {lo[1]:.2f}, {lo[2]:.2f})")
    print(f"    max: ({hi[0]:.2f}, {hi[1]:.2f}, {hi[2]:.2f})")
    print(f"    center: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    print("=" * 70)
    print("Floor and wall bounding boxes")
    print("=" * 70)
    for path in ["/World/GroundPlane", "/World/Wall1", "/World/wall2",
                 "/World/wall3", "/World/wall3_01", "/World/wall3_02"]:
        print_bbox(stage, path)

    print()
    print("=" * 70)
    print("Also, for reference, /World/Cube and /World/Cube_01 (unidentified extras in this scene)")
    print("=" * 70)
    for path in ["/World/Cube", "/World/Cube_01"]:
        print_bbox(stage, path)


if __name__ == "__main__":
    main()
    simulation_app.close()
