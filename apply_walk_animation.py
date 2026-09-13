r"""
apply_walk_animation.py

Copies the already-fixed walk animation (root motion already stripped -
see strip_root_motion.py) from xbot_walk.usd's "mixamo_com" SkelAnimation
directly onto each of the new textured Mixamo characters. Works because
their skeletons are confirmed identical (65 joints, same mixamorig_* names
and hierarchy - Mixamo standardizes rigging across all its characters), so
no retargeting/remapping is needed, just a direct copy of the joints/
rotations/translations time-sample data onto a new local SkelAnimation
prim under each character's own SkelRoot.

Edits each assets\character*.usd file IN PLACE (our own derived/generated
assets, not the original scene file).

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\apply_walk_animation.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import re
import omni.usd
from pxr import Usd, UsdSkel, Sdf


def canonicalize(joint_path):
    """Strips the per-export 'mixamorigN_' prefix (e.g. 'mixamorig8_',
    'mixamorig9_', or plain 'mixamorig_') from every segment of a joint
    path, so joints can be matched by their actual bone name regardless of
    which numeric suffix this particular Mixamo/FBX conversion happened to
    assign. Confirmed necessary via diagnose_all_persons.py: joint ORDER
    also isn't guaranteed identical across separate exports even when
    prefixes DO match (LeftShoulder/RightShoulder swapped at index 7
    between character.usd and xbot_walk.usd despite matching prefixes), so
    matching must be done by name, never by raw array position.
    """
    return re.sub(r"mixamorig\d*_", "", str(joint_path))

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
SOURCE_WALK_USD = ASSETS_DIR + r"\xbot_walk.usd"
SOURCE_CLIP_NAME = "mixamo_com"

TARGET_FILES = [
    "character.usd",
    "character_1.usd",
    "character_2.usd",
    "character_3.usd",
    "character_4.usd",
]


def get_real_clip_data(context):
    """Opens xbot_walk.usd and pulls out the real animation data:
    joints list + every (time, rotations, translations, scales) sample."""
    context.open_stage(SOURCE_WALK_USD)
    for _ in range(30):
        simulation_app.update()
    stage = context.get_stage()

    anim_prim = None
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdSkel.Animation) and prim.GetName() == SOURCE_CLIP_NAME:
            anim_prim = prim
            break
    if anim_prim is None:
        raise RuntimeError(f"could not find '{SOURCE_CLIP_NAME}' SkelAnimation in {SOURCE_WALK_USD}")

    anim = UsdSkel.Animation(anim_prim)
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

    print(f"[SOURCE] {len(joints)} joints, {len(data)} time samples pulled from {SOURCE_WALK_USD}")
    return joints, data


def apply_to_character(context, filename, joints, data):
    """joints/data are the SOURCE (xbot_walk.usd) animation's own joint
    list and per-time-sample values, each value array in SOURCE order.
    We build a name->index lookup into that source data, then construct
    the new animation using the TARGET skeleton's OWN joints list/order
    (authoritative for correct skinning), pulling each target joint's
    rotation/translation/scale from the source by canonical name match -
    NOT by raw array position, which was the bug that caused T-posing.
    """
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
        print(f"[MISMATCH] {filename}: {len(target_joints)} joints vs source's {len(joints)} - skipping, "
              f"needs manual remapping.")
        return False

    # name (canonicalized) -> index in the SOURCE joints list
    source_index_by_name = {canonicalize(j): i for i, j in enumerate(joints)}

    unmatched = [tj for tj in target_joints if canonicalize(tj) not in source_index_by_name]
    if unmatched:
        print(f"[MISMATCH] {filename}: {len(unmatched)} target joint(s) have no canonical match in source "
              f"(e.g. {unmatched[0]}) - skipping, needs manual remapping.")
        return False

    # For each time sample, build new rotation/translation/scale arrays in
    # TARGET joint order, pulling each value from the source array at the
    # matched source index.
    remap_indices = [source_index_by_name[canonicalize(tj)] for tj in target_joints]

    skel_root = skel_prim.GetParent()
    anim_path = skel_root.GetPath().AppendChild("mixamo_com")
    anim_prim = stage.DefinePrim(anim_path, "SkelAnimation")
    anim = UsdSkel.Animation(anim_prim)
    anim.CreateJointsAttr().Set(target_joints)  # target's own order - authoritative for skinning

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

    binding = UsdSkel.BindingAPI(skel_prim)
    rel = binding.GetAnimationSourceRel()
    if not rel:
        rel = binding.CreateAnimationSourceRel()
    rel.SetTargets([anim_path])

    stage.GetRootLayer().Save()
    print(f"[OK] {filename}: applied {len(data)}-sample walk animation (name-remapped) to {skel_prim.GetPath()}, "
          f"animationSource -> {anim_path}")
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
