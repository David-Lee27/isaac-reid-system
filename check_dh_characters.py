"""
ISOLATED, read-only. Investigates whether DH_Characters (likely "Digital
Human" - a different, NVIDIA-native character family from the Reallusion-
rigged "Characters" folder we already proved incompatible) matches
MotionLibrary/HumanMotionLibrary.usd's joint naming.

Run from PowerShell:
    C:\\isaacsim\\python.bat C:\\isaacsim\\projects\\surveillance-proj\\check_dh_characters.py
"""

HEADLESS = False

DH_CHAR_FOLDER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
                   "Assets/Isaac/6.0/Isaac/People/DH_Characters/"
                   "02c80685-06e3-11ef-ae8a-f4b30194174e/")

MOTION_LIBRARY_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
                       "Assets/Isaac/6.0/Isaac/People/MotionLibrary/HumanMotionLibrary.usd")

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.client
from pxr import Usd, UsdSkel


def main():
    print("=" * 70)
    print(f"STEP 1: Listing contents of one DH_Characters folder")
    print("=" * 70)
    result, entries = omni.client.list(DH_CHAR_FOLDER)
    print(f"  result: {result}")
    dh_char_usd = None
    if entries:
        for entry in entries:
            print(f"  {entry.relative_path}")
            if entry.relative_path.endswith(".usd") and dh_char_usd is None:
                dh_char_usd = DH_CHAR_FOLDER + entry.relative_path

    if dh_char_usd is None:
        print("  No .usd file found directly in this folder - it may be nested deeper.")
        simulation_app.close()
        return
    print(f"\n  Using: {dh_char_usd}")

    print("\n" + "=" * 70)
    print("STEP 2: Loading DH character, finding its skeleton")
    print("=" * 70)
    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    for _ in range(20):
        simulation_app.update()
    stage = usd_context.get_stage()

    char_prim = stage.DefinePrim("/World/DHChar", "Xform")
    char_prim.GetReferences().AddReference(dh_char_usd)
    for _ in range(30):
        simulation_app.update()

    dh_skeleton = None
    for descendant in Usd.PrimRange(char_prim):
        if descendant.IsA(UsdSkel.Skeleton):
            dh_skeleton = descendant
            break

    if dh_skeleton is None:
        print("  No Skeleton found under this DH character.")
    else:
        skel_obj = UsdSkel.Skeleton(dh_skeleton)
        joints_attr = skel_obj.GetJointsAttr()
        joints = list(joints_attr.Get()) if joints_attr else []
        print(f"  DH character skeleton has {len(joints)} joints. First 10: {joints[:10]}")

        # Check if it already has an animationSource bound out of the box.
        binding = UsdSkel.BindingAPI(dh_skeleton)
        rel = binding.GetAnimationSourceRel()
        targets = rel.GetTargets() if rel else []
        print(f"  Existing animationSource: {targets}")

    print("\n" + "=" * 70)
    print("STEP 3: Inspecting HumanMotionLibrary.usd's SkelAnimation joint names")
    print("=" * 70)
    lib_prim = stage.DefinePrim("/World/MotionLib", "Xform")
    lib_prim.GetReferences().AddReference(MOTION_LIBRARY_URL)
    for _ in range(30):
        simulation_app.update()

    anim_prims_found = []
    for descendant in Usd.PrimRange(lib_prim):
        if descendant.IsA(UsdSkel.Animation):
            anim_prims_found.append(descendant)

    print(f"  Found {len(anim_prims_found)} SkelAnimation prim(s) in the motion library.")
    for anim_prim in anim_prims_found[:5]:
        anim_obj = UsdSkel.Animation(anim_prim)
        joints_attr = anim_obj.GetJointsAttr()
        joints = list(joints_attr.Get()) if joints_attr else []
        print(f"\n  {anim_prim.GetPath()}")
        print(f"    {len(joints)} joints. First 10: {joints[:10]}")

        if dh_skeleton is not None:
            dh_set = set(str(j) for j in list(UsdSkel.Skeleton(dh_skeleton).GetJointsAttr().Get() or []))
            anim_set = set(str(j) for j in joints)
            overlap = dh_set & anim_set
            print(f"    Overlap with DH character skeleton: {len(overlap)} / {len(anim_set)}")

    print("\nDONE. Paste this whole output back.")


if __name__ == "__main__":
    main()
    simulation_app.close()
