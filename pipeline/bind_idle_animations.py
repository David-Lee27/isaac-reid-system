r"""
bind_idle_animations.py

Adds 4 new named idle/social animation clips onto each character's own USD
file, alongside the existing walk ("mixamo_com"), fall ("mixamo_fall") and
get-up ("mixamo_get_up") clips - same canonical-name joint remap pattern
as bind_fall_animation.py/bind_get_up_animation.py, looped over all 4 new
clips instead of one script per clip since it's the exact same binding
step each time.

Does NOT touch each Skeleton's CURRENT animationSource binding -
unified_tracking.py retargets that relationship at runtime (see the new
"idle pause" behavior in update_person_position: an ambient wanderer who
just arrived at a wander target has a chance to stop and play one of
these instead of immediately picking a new destination).

Unlike the fall/get-up clips, root motion IS stripped here (translations
zeroed at every sample, rotations kept) - these are stationary social
animations (idle stand, salute, excited, talking on phone), not a
scripted collapse/recovery with an intentional baked rise/fall. Keeping
the source FBX's own root translation would make characters visibly
slide across the floor while "idling," which defeats the point.

Edits each assets\character*.usd file IN PLACE.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\bind_idle_animations.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import re
import omni.usd
from pxr import Usd, UsdSkel, Gf

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"

# (source clip usd, new clip name on each character)
CLIPS = [
    ("idle_stand_clip_source.usd", "mixamo_idle_stand"),
    ("idle_salute_clip_source.usd", "mixamo_idle_salute"),
    ("idle_excited_clip_source.usd", "mixamo_idle_excited"),
    ("idle_phone_clip_source.usd", "mixamo_idle_phone"),
]

# All character files this project has ever spawned a mover from - Person1/
# Person2 use character.usd/character_1.usd/character_2.usd (the 3
# originally in the scene - see bind_get_up_animation.py's own comment for
# why all 3 are always processed as a group rather than trying to prove
# exactly which of the two currently-tracked people uses which one).
# character_3.usd/character_4.usd (Person4/Person5, the intruders) don't
# actually idle in practice (see main()'s FALL_ENABLED/LOITER_ENABLED
# is_intruder exclusion - they walk straight to the door and never wander
# ambiently), but binding the clips onto them too is harmless and keeps
# every character file consistent in case that ever changes.
TARGET_FILES = ["character.usd", "character_1.usd", "character_2.usd", "character_3.usd", "character_4.usd"]


def canonicalize(joint_path):
    return re.sub(r"mixamorig\d*_", "", str(joint_path))


def get_real_clip_data(context, source_usd):
    """Same approach as bind_fall_animation.py's get_real_clip_data - find
    whichever SkelAnimation prim actually has real (nonzero) rotation time
    samples, rather than assuming a fixed clip name. Strips translation
    (root motion) - see this file's own docstring for why."""
    context.open_stage(f"{ASSETS_DIR}\\{source_usd}")
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
        if n > best_count:
            best_count = n
            best_prim = prim

    if best_prim is None:
        raise RuntimeError(f"no SkelAnimation with real data found in {source_usd}")

    anim = UsdSkel.Animation(best_prim)
    joints = anim.GetJointsAttr().Get()
    rot_attr = anim.GetRotationsAttr()
    scale_attr = anim.GetScalesAttr()

    time_samples = rot_attr.GetTimeSamples()
    data = []
    for t in time_samples:
        rot_val = rot_attr.Get(t)
        scale_val = scale_attr.Get(t) if scale_attr else None
        data.append((t, rot_val, scale_val))

    print(f"[SOURCE] {source_usd}: using {best_prim.GetPath()}, {len(joints)} joints, {len(data)} time samples")
    return joints, data


def apply_to_character(context, filename, clip_name, joints, data):
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
    anim_path = skel_root.GetPath().AppendChild(clip_name)
    anim_prim = stage.DefinePrim(anim_path, "SkelAnimation")
    anim = UsdSkel.Animation(anim_prim)
    anim.CreateJointsAttr().Set(target_joints)

    rot_attr = anim.CreateRotationsAttr()
    scale_attr = anim.CreateScalesAttr()
    trans_attr = anim.CreateTranslationsAttr()
    # Root motion is stripped by holding every joint at the walk clip's first-sample
    # translations (the rig's real bone offsets, no travel). Do NOT zero them: a
    # skeleton animation's per-joint translations ARE the bone lengths, and zeroing
    # all of them collapsed every character into a lump (see fix_idle_translations.py).
    walk_prim = stage.GetPrimAtPath(skel_root.GetPath().AppendChild("mixamo_com"))
    walk_attr = UsdSkel.Animation(walk_prim).GetTranslationsAttr()
    walk_samples = walk_attr.GetTimeSamples() if walk_attr else []
    if not walk_samples:
        print(f"[SKIP] {filename}: no walk clip translations to take bone offsets from")
        return False
    rest_translations = walk_attr.Get(walk_samples[0])
    for t, rot_val, scale_val in data:
        if rot_val is not None:
            rot_attr.Set([rot_val[i] for i in remap_indices], t)
        if scale_val is not None:
            scale_attr.Set([scale_val[i] for i in remap_indices], t)
        trans_attr.Set(rest_translations, t)  # root motion stripped, bone offsets kept - see above

    stage.GetRootLayer().Save()
    print(f"[OK] {filename}: added '{clip_name}' clip ({len(data)} samples) at {anim_path}")
    return True


def main():
    context = omni.usd.get_context()

    for source_usd, clip_name in CLIPS:
        joints, data = get_real_clip_data(context, source_usd)
        print(f"\n=== Binding '{clip_name}' ===")
        for filename in TARGET_FILES:
            apply_to_character(context, filename, clip_name, joints, data)


if __name__ == "__main__":
    main()
    simulation_app.close()
