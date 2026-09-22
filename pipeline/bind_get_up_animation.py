r"""
bind_get_up_animation.py

Adds a THIRD named animation clip ("mixamo_get_up") onto each character's
own USD file, alongside the existing walk ("mixamo_com") and fall
("mixamo_fall") clips - same canonical-name joint remap pattern as
bind_fall_animation.py, sourced from get_up_clip_source.usd instead of
fall_clip_source.usd.

Does NOT touch each Skeleton's CURRENT animationSource binding -
unified_tracking.py retargets that relationship at runtime (fall_held ->
get_up -> walk on recovery).

Deliberately does NOT strip root motion (same reasoning as the fall clip -
unified_tracking.py freezes translation for the whole pause, so this
clip's own baked rise-from-ground motion is exactly what should visually
happen).

Edits each assets\character*.usd file IN PLACE.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\bind_get_up_animation.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import re
import omni.usd
from pxr import Usd, UsdSkel, Sdf

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
SOURCE_GET_UP_USD = ASSETS_DIR + r"\get_up_clip_source.usd"

TARGET_FILES = ["character.usd", "character_1.usd", "character_2.usd", "character_3.usd"]  # the 3 originally in the scene, plus character_3.usd - the new 4th "intruder" character

NEW_CLIP_NAME = "mixamo_get_up"


def canonicalize(joint_path):
    return re.sub(r"mixamorig\d*_", "", str(joint_path))


def get_real_clip_data(context):
    """Same approach as bind_fall_animation.py's get_real_clip_data - find
    whichever SkelAnimation prim actually has real (nonzero) rotation time
    samples, rather than assuming a fixed clip name."""
    context.open_stage(SOURCE_GET_UP_USD)
    for _ in range(30):
        simulation_app.update()
    stage = context.get_stage()

    best_prim = None
    best_count = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(UsdSkel.Animation):
            continue
        anim = UsdSkel.Animation(prim)
        rot_attr = anim.GetRotationsAttr()
        n = rot_attr.GetNumTimeSamples() if rot_attr else 0
        print(f"[FOUND] {prim.GetPath()}: {n} rotation time samples")
        if n > best_count:
            best_count = n
            best_prim = prim

    if best_prim is None:
        raise RuntimeError(f"no SkelAnimation with real data found in {SOURCE_GET_UP_USD}")

    anim = UsdSkel.Animation(best_prim)
    joints = anim.GetJointsAttr().Get()
    rot_attr = anim.GetRotationsAttr()
    trans_attr = anim.GetTranslationsAttr()
    scale_attr = anim.GetScalesAttr()

    time_samples = rot_attr.GetTimeSamples()
    data = []
    for t in time_samples:
        rot_val = rot_attr.Get(t)
        trans_val = trans_attr.Get(t) if trans_attr else None
        scale_val = scale_attr.Get(t) if scale_attr else None
        data.append((t, rot_val, trans_val, scale_val))

    print(f"[SOURCE] using {best_prim.GetPath()}: {len(joints)} joints, {len(data)} time samples")
    return joints, data


def apply_to_character(context, filename, joints, data):
    path = f"{ASSETS_DIR}\\{filename}"
    context.open_stage(path)
    for _ in range(30):
        simulation_app.update()
    stage = context.get_stage()

    skel_prim = None
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdSkel.Skeleton):
            skel_prim = prim
            break
    if skel_prim is None:
        print(f"[SKIP] {filename}: no Skeleton found")
        return False

    target_joints = UsdSkel.Skeleton(skel_prim).GetJointsAttr().Get()
    if len(target_joints) != len(joints):
        print(f"[MISMATCH] {filename}: joint count differs ({len(target_joints)} vs {len(joints)}) - skipping.")
        return False

    source_index_by_name = {canonicalize(j): i for i, j in enumerate(joints)}
    unmatched = [tj for tj in target_joints if canonicalize(tj) not in source_index_by_name]
    if unmatched:
        print(f"[MISMATCH] {filename}: {len(unmatched)} unmatched joint(s), e.g. {unmatched[0]} - skipping.")
        return False
    remap_indices = [source_index_by_name[canonicalize(tj)] for tj in target_joints]

    skel_root = skel_prim.GetParent()
    anim_path = skel_root.GetPath().AppendChild(NEW_CLIP_NAME)
    anim_prim = stage.DefinePrim(anim_path, "SkelAnimation")
    anim = UsdSkel.Animation(anim_prim)
    anim.CreateJointsAttr().Set(target_joints)

    rot_attr = anim.CreateRotationsAttr()
    trans_attr = anim.CreateTranslationsAttr()
    scale_attr = anim.CreateScalesAttr()
    for t, rot_val, trans_val, scale_val in data:
        if rot_val is not None:
            rot_attr.Set([rot_val[i] for i in remap_indices], t)
        if trans_val is not None:
            trans_attr.Set([trans_val[i] for i in remap_indices], t)
        if scale_val is not None:
            scale_attr.Set([scale_val[i] for i in remap_indices], t)

    stage.GetRootLayer().Save()
    print(f"[OK] {filename}: added '{NEW_CLIP_NAME}' clip ({len(data)} samples) at {anim_path}, "
          f"animationSource left untouched")
    return True


def main():
    context = omni.usd.get_context()
    joints, data = get_real_clip_data(context)

    results = {}
    for filename in TARGET_FILES:
        results[filename] = apply_to_character(context, filename, joints, data)

    print("\n=== SUMMARY ===")
    for filename, ok in results.items():
        print(f"{filename}: {'OK' if ok else 'FAILED/SKIPPED'}")


if __name__ == "__main__":
    main()
    simulation_app.close()
