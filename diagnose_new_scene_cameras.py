r"""
diagnose_new_scene_cameras.py

Lists every Camera prim in optimized_room.usd along with its world
position, so ZONE_CENTERS / camera_zones.json / CHECKPOINT_PATH can be
wired up to REAL names and coordinates instead of guessed ones - guessing
scene contents has caused enough "why is it going to an empty spot" bugs
this session already.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_new_scene_cameras.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\optimized room.usd"  # real filename has a SPACE, not an underscore - confirmed from the Isaac Sim title bar


def main():
    ctx = omni.usd.get_context()
    result = ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    print("=" * 70)
    print(f"Stage: {STAGE_PATH}")
    print("=" * 70)

    print("\nAll Camera prims:")
    cameras = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdGeom.Camera):
            xform = UsdGeom.Xformable(prim)
            pos = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            cameras.append((str(prim.GetPath()), pos))
            print(f"  {prim.GetPath()}  ->  world position ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

    if not cameras:
        print("  [WARN] no Camera prims found at all - check STAGE_PATH is correct.")

    print("\nAll top-level prims under /World (for finding people/room/wall names):")
    world_prim = stage.GetPrimAtPath("/World")
    if world_prim.IsValid():
        for child in world_prim.GetChildren():
            print(f"  {child.GetPath()}  ({child.GetTypeName()})")
    else:
        print("  [WARN] /World not found.")


if __name__ == "__main__":
    main()
    simulation_app.close()
