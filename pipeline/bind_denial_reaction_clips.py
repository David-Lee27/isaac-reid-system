r"""
bind_denial_reaction_clips.py

Converts and binds the "denied at the door" reaction clips (user-supplied
Mixamo FBXs, dropped into assets/): Angry (the reaction beat that plays the
instant someone is turned away, before they turn to leave - replaces Sad Idle
here per explicit direction, "use the angry one... instead of the sad one when
getting rejected because it looks better"), Sad Idle (kept bound and available,
just no longer the default reaction clip used for this), and Sad Walk (used for
the walk out itself, instead of the normal walk clip, per explicit direction -
"i want to see it... play the animation and then they walk out so people can see
that theirs something wrong with them and they leave rejected" - and, on which
clip plays which beat, "only use that one when they are getting rejected, when
walkign away use the sad one").

Angry and Sad Idle are both bound exactly like the existing idle/social clips
(see bind_idle_animations.py): every joint held at the walk clip's own
rest-pose translation (the rig's real bone offsets - only ROTATIONS animate
the gesture), because they're stationary poses.

Sad Walk is NOT bound that way, on purpose: a walk clip's per-joint
TRANSLATIONS are what actually swing the legs frame to frame, so holding them
constant (the idle-clip technique) would render as a frozen body with no visible
walking at all - the exact "collapsed"/"frozen" class of bug already hit once
this session (see fix_idle_translations.py) and worth not repeating. Instead
this keeps the Sad Walk clip's OWN real per-sample translations (remapped per
joint, same as its rotations), and strips only the ROOT joint's horizontal (X/Y)
drift across samples - same targeted technique as strip_root_motion.py used for
the main walk clip - so the character doesn't slide across the floor under our
own position-driven movement while the legs still swing naturally.

Edits each assets\character*.usd file IN PLACE, adding "mixamo_angry_reaction",
"mixamo_sad_idle" and "mixamo_sad_walk" SkelAnimation clips alongside the
existing ones. Does not touch any Skeleton's current animationSource binding.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\pipeline\bind_denial_reaction_clips.py
"""
import os
import re

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel, Gf

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
TARGET_FILES = ["character.usd", "character_1.usd", "character_2.usd", "character_3.usd", "character_4.usd"]

# (already-converted source usd, new clip name on each character, is a locomotion clip)
# Conversion (FBX->USD) is a separate script per clip (convert_angry_clip.py,
# convert_sad_walk.py) - see either one's own docstring for why mixing the
# async converter with this script's synchronous stage-opening in one process
# corrupted the converter's event loop.
CLIPS = [
    ("angry_reaction_clip_source.usd", "mixamo_angry_reaction", False),
    ("sad_idle_clip_source.usd", "mixamo_sad_idle", False),
    ("sad_walk_clip_source.usd", "mixamo_sad_walk", True),
]


def canonicalize(joint_path):
    return re.sub(r"mixamorig\d*_", "", str(joint_path))


def get_real_clip_data(context, source_usd):
    """Same "find whichever SkelAnimation actually has real rotation samples"
    approach as bind_idle_animations.py's own helper - now also keeps the
    source's own translations per sample (not discarded), since Sad Walk needs
    them."""
    context.open_stage(f"{ASSETS_DIR}\\{source_usd}")
    for _ in range(30):
        simulation_app.update()
    stage = context.get_stage()

    best_prim, best_count = None, 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(UsdSkel.Animation):
            continue
        anim = UsdSkel.Animation(prim)
        rot_attr = anim.GetRotationsAttr()
        n = rot_attr.GetNumTimeSamples() if rot_attr else 0
        if n > best_count:
            best_count, best_prim = n, prim
    if best_prim is None:
        raise RuntimeError(f"no SkelAnimation with real data found in {source_usd}")

    anim = UsdSkel.Animation(best_prim)
    joints = anim.GetJointsAttr().Get()
    rot_attr = anim.GetRotationsAttr()
    scale_attr = anim.GetScalesAttr()
    trans_attr = anim.GetTranslationsAttr()
    time_samples = rot_attr.GetTimeSamples()
    data = []
    for t in time_samples:
        data.append((
            t,
            rot_attr.Get(t),
            scale_attr.Get(t) if scale_attr else None,
            trans_attr.Get(t) if trans_attr else None,
        ))
    print(f"[SOURCE] {source_usd}: using {best_prim.GetPath()}, {len(joints)} joints, {len(data)} time samples")
    return joints, data


def apply_to_character(context, filename, clip_name, joints, data, is_locomotion):
    path = f"{ASSETS_DIR}\\{filename}"
    context.open_stage(path)
    for _ in range(30):
        simulation_app.update()
    stage = context.get_stage()

    skel_prim = next((p for p in Usd.PrimRange(stage.GetPseudoRoot()) if p.IsA(UsdSkel.Skeleton)), None)
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

    if is_locomotion:
        # Root-joint index in the TARGET's own joint order (index 0 by USD/this
        # rig's convention - same fallback strip_root_motion.py uses).
        root_idx = next((i for i, j in enumerate(target_joints) if str(j).count("/") == 0), 0)
        for t, rot_val, scale_val, trans_val in data:
            if rot_val is not None:
                rot_attr.Set([rot_val[i] for i in remap_indices], t)
            if scale_val is not None:
                scale_attr.Set([scale_val[i] for i in remap_indices], t)
            if trans_val is not None:
                remapped = [trans_val[i] for i in remap_indices]
                # Strip horizontal root drift only - keep vertical bob AND every
                # other joint's own natural walking-motion translation untouched.
                rv = remapped[root_idx]
                remapped[root_idx] = Gf.Vec3f(0.0, 0.0, rv[2])
                trans_attr.Set(remapped, t)
    else:
        walk_attr = UsdSkel.Animation(stage.GetPrimAtPath(skel_root.GetPath().AppendChild("mixamo_com"))).GetTranslationsAttr()
        walk_samples = walk_attr.GetTimeSamples() if walk_attr else []
        if not walk_samples:
            print(f"[SKIP] {filename}: no walk clip translations to take rest-pose bone offsets from")
            return False
        rest_translations = walk_attr.Get(walk_samples[0])
        for t, rot_val, scale_val, _ in data:
            if rot_val is not None:
                rot_attr.Set([rot_val[i] for i in remap_indices], t)
            if scale_val is not None:
                scale_attr.Set([scale_val[i] for i in remap_indices], t)
            trans_attr.Set(rest_translations, t)  # stationary pose - hold at the rig's real bone offsets

    stage.GetRootLayer().Save()
    print(f"[OK] {filename}: added '{clip_name}' clip ({len(data)} samples)")
    return True


def main():
    context = omni.usd.get_context()
    for usd_name, clip_name, is_locomotion in CLIPS:
        if not os.path.exists(f"{ASSETS_DIR}\\{usd_name}"):
            print(f"[SKIP] {usd_name} not converted yet - run convert_angry_clip.py / convert_sad_walk.py first.")
            continue
        joints, data = get_real_clip_data(context, usd_name)
        print(f"\n=== Binding '{clip_name}' (locomotion={is_locomotion}) ===")
        for filename in TARGET_FILES:
            apply_to_character(context, filename, clip_name, joints, data, is_locomotion)


main()
simulation_app.close()
