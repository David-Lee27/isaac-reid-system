r"""
diagnose_xbot.py

Read-only check on the freshly converted Mixamo xbot_walk.usd - confirms
SkelRoot/Skeleton/SkelAnimation actually exist, joint count, and whether
translation/rotation time samples are present (i.e. real baked animation,
not just a static bind pose). Makes NO changes to any file.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_xbot.py
"""

USD_STAGE_PATH = r"C:\isaacsim\projects\surveillance-proj\assets\xbot_walk.usd"
HEADLESS = True

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
from pxr import Usd, UsdSkel


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = usd_context.get_stage()

    print("\n" + "=" * 70)
    print(f"FULL PRIM TREE: {USD_STAGE_PATH}")
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

    root = stage.GetPseudoRoot()
    for child in root.GetChildren():
        describe(child)

    print("\n" + "=" * 70)
    print("SKEL DETAIL")
    print("=" * 70)
    found = False
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdSkel.Skeleton):
            found = True
            skel = UsdSkel.Skeleton(prim)
            joints = skel.GetJointsAttr().Get()
            print(f"\nSkeleton: {prim.GetPath()}  joints={len(joints) if joints else 0}")
            if joints:
                print(f"  first 5 joint names: {list(joints[:5])}")
            binding = UsdSkel.BindingAPI(prim)
            anim_targets = binding.GetAnimationSourceRel().GetTargets()
            print(f"  animationSource targets: {anim_targets}")
        if prim.IsA(UsdSkel.Animation):
            found = True
            anim = UsdSkel.Animation(prim)
            joints = anim.GetJointsAttr().Get()
            trans_attr = anim.GetTranslationsAttr()
            rot_attr = anim.GetRotationsAttr()
            n_trans = trans_attr.GetNumTimeSamples() if trans_attr else 0
            n_rot = rot_attr.GetNumTimeSamples() if rot_attr else 0
            print(f"\nSkelAnimation: {prim.GetPath()}")
            print(f"  joints animated: {len(joints) if joints else 0}")
            print(f"  translation time samples: {n_trans}")
            print(f"  rotation time samples: {n_rot}")
            if rot_attr and n_rot:
                ts = rot_attr.GetTimeSamples()
                print(f"  time range: {ts[0]} to {ts[-1]}")

    if not found:
        print("NO Skeleton or SkelAnimation prims found - conversion did not carry animation.")

    print("\n" + "=" * 70)
    print("DONE. Paste this whole output back.")
    print("=" * 70)


if __name__ == "__main__":
    main()
    simulation_app.close()
