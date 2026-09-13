r"""
diagnose_new_character.py

Checks a converted Mixamo character USD for: skeleton joint naming
(to see if it matches xbot_walk.usd's "mixamorig_*" convention, meaning we
could reuse that walk animation directly) and whether any real baked
animation came with this download, vs. a bare T-pose mesh.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_new_character.py
"""

USD_STAGE_PATH = r"C:\isaacsim\projects\surveillance-proj\assets\character.usd"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = usd_context.get_stage()

    print("\n" + "=" * 70)
    print(f"PRIM TREE: {USD_STAGE_PATH}")
    print("=" * 70)

    def describe(prim, depth=0):
        marker = ""
        if prim.IsA(UsdSkel.Root):
            marker = "  <-- SkelRoot"
        elif prim.IsA(UsdSkel.Skeleton):
            marker = "  <-- Skeleton"
        elif prim.IsA(UsdSkel.Animation):
            marker = "  <-- SkelAnimation"
        print("  " * depth + f"{prim.GetName()} [{prim.GetTypeName()}]{marker}")
        for child in prim.GetChildren():
            describe(child, depth + 1)

    for child in stage.GetPseudoRoot().GetChildren():
        describe(child)

    print("\n" + "=" * 70)
    print("SKEL DETAIL")
    print("=" * 70)
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdSkel.Skeleton):
            skel = UsdSkel.Skeleton(prim)
            joints = skel.GetJointsAttr().Get()
            print(f"\nSkeleton: {prim.GetPath()}  joints={len(joints) if joints else 0}")
            if joints:
                print(f"  first 5 joint names: {list(joints[:5])}")
        if prim.IsA(UsdSkel.Animation):
            anim = UsdSkel.Animation(prim)
            rot_attr = anim.GetRotationsAttr()
            n_rot = rot_attr.GetNumTimeSamples() if rot_attr else 0
            print(f"\nSkelAnimation: {prim.GetPath()}  rotation time samples: {n_rot}")

    print("\nDONE. Paste this output back.")


if __name__ == "__main__":
    main()
    simulation_app.close()
