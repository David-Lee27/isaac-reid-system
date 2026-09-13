r"""
bind_fall_animation.py

Adds a SECOND named animation clip ("mixamo_fall") onto each character's
own USD file, alongside the existing walk clip ("mixamo_com") - same
canonical-name joint remap as apply_walk_animation.py, sourced from
fall_clip_source.usd instead of xbot_walk.usd.

Does NOT touch each Skeleton's CURRENT animationSource binding (leaves it
on the walk clip) - unified_tracking.py's fall-event code retargets that
relationship at runtime (walk -> fall on collapse, fall -> walk on
recovery) instead.

Deliberately does NOT strip root motion from this clip (unlike the walk
clip) - unified_tracking.py freezes translation entirely for the whole
fall duration, so the fall clip's own baked forward-pitch/drop-to-ground
motion is exactly what should visually happen, not something to cancel out.

Edits each assets\character*.usd file IN PLACE.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\bind_fall_animation.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import re
import omni.usd
from pxr import Usd, UsdSkel, Sdf

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
SOURCE_FALL_USD = ASSETS_DIR + r"\fall_clip_source.usd"

TARGET_FILES = ["character.usd", "character_1.usd", "character_2.usd"]  # the 3 actually used in the scene

NEW_CLIP_NAME = "mixamo_fall"


def canonicalize(joint_path):
    return re.sub(r"mixamorig\d*_", "", str(joint_path))


def get_real_clip_data(context):
    """Unlike the walk clip, we don't know this download's exact clip name
    up front (source character 'Ch27' may name it differently than 'Ch21'
    did) - so find whichever SkelAnimation prim actually HAS real (nonzero)
    rotation time samples, instead of assuming a fixed name."""
    context.open_stage(SOURCE_FALL_USD)
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
        raise RuntimeError(f"no SkelAnimation with real data found in {SOURCE_FALL_USD}")

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

    # Deliberately NOT touching the Skeleton's animationSource here - it
    # stays on the walk clip by default. unified_tracking.py retargets it
    # to this new clip at runtime when a fall triggers.

    stage.GetRootLayer().Save()
    print(f"[OK] {filename}: added '{NEW_CLIP_NAME}' clip ({len(data)} samples) at {anim_path}, "
          f"animationSource left untouched (still on walk clip)")
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
