r"""
fix_fall_translation_and_hold.py

Two fixes:

1. Zero out ALL root-joint translation on the fall clip (not just
   horizontal, now vertical too) - diagnose_fall_translation.py proved the
   fall clip's raw translation data uses a completely different unit/axis
   convention than the walk clip's ((-0.37, 97.2, 0.28) cm-scale/Y-up-ish
   vs walk's (0, 0, 1.53) meters-scale/Z-up), so there's no safe way to
   reuse it directly without guessing a conversion factor. Since rotations
   aren't subject to this unit ambiguity (degrees/quaternions are scale-
   independent), keeping ONLY the fall clip's rotations and zeroing all
   translation lets the pose collapse correctly while the character's
   root position stays exactly where they were standing - no more
   floating/wrong-scale drift.

2. Adds a SECOND clip "mixamo_fall_held" - a single-time-sample snapshot
   of the fall clip's LAST pose. unified_tracking.py switches to this
   once the fall animation has finished playing (before standing back
   up), so the character stays frozen in the collapsed pose instead of
   the whole scene's shared global timeline wrapping every ~3s and
   snapping them back to standing/replaying the collapse repeatedly
   during the (much longer) lie-down pause.

Edits assets\character.usd, character_1.usd, character_2.usd IN PLACE.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\fix_fall_translation_and_hold.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel, Gf

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
TARGET_FILES = ["character.usd", "character_1.usd", "character_2.usd"]
FALL_CLIP_NAME = "mixamo_fall"
HELD_CLIP_NAME = "mixamo_fall_held"


def main():
    ctx = omni.usd.get_context()
    for filename in TARGET_FILES:
        path = f"{ASSETS_DIR}\\{filename}"
        ctx.open_stage(path)
        for _ in range(30):
            simulation_app.update()
        stage = ctx.get_stage()

        skel_prim = None
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if prim.IsA(UsdSkel.Skeleton):
                skel_prim = prim
                break
        if skel_prim is None:
            print(f"[SKIP] {filename}: no Skeleton found")
            continue

        skel_root_path = skel_prim.GetParent().GetPath()
        fall_path = skel_root_path.AppendChild(FALL_CLIP_NAME)
        fall_prim = stage.GetPrimAtPath(fall_path)
        if not fall_prim.IsValid():
            print(f"[SKIP] {filename}: no {FALL_CLIP_NAME} clip found")
            continue

        anim = UsdSkel.Animation(fall_prim)
        joints = anim.GetJointsAttr().Get()
        rot_attr = anim.GetRotationsAttr()
        trans_attr = anim.GetTranslationsAttr()

        time_samples = rot_attr.GetTimeSamples()

        # --- Fix 1: zero ALL translation (every joint, every sample) ---
        # Only the ROOT joint's translation actually matters for world
        # position (child joints' translations are essentially fixed bone
        # lengths, safe to leave as-is), so only zero joint index... but to
        # be safe and simple, zero just the root (index found by name).
        root_idx = 0
        for i, j in enumerate(joints):
            if "Hips" in str(j) and str(j).count("/") == 0:
                root_idx = i
                break

        fixed = 0
        last_rot_val = None
        last_t = None
        for t in time_samples:
            values = trans_attr.Get(t)
            new_values = list(values)
            new_values[root_idx] = Gf.Vec3f(0.0, 0.0, 0.0)
            trans_attr.Set(new_values, t)
            fixed += 1
            last_t = t
            last_rot_val = rot_attr.Get(t)

        print(f"\n{filename}: [FIXED] zeroed root translation on {fixed} fall-clip sample(s) (rotation-only fall)")

        # --- Fix 2: author a REAL time-sampled "held" clip from the last pose ---
        # Two earlier attempts both T-posed: a single time-sample AND a
        # pure default-value (no timeCode at all). Since the actual
        # mixamo_fall/mixamo_com clips (normal multi-sample time data) work
        # fine, the common factor in both failures wasn't the translation
        # math - it's that Kit's skinning evaluation apparently doesn't
        # pick up non-standard (default-only or single-sample) animation
        # data the same way, falling back to the skeleton's raw T-pose bind
        # pose. Fix: author TWO time samples, far apart, both holding the
        # identical last-frame pose - genuinely normal time-sampled data
        # (same shape as the working clips), constant in practice since
        # both endpoints are identical, and holds before/after via standard
        # boundary extrapolation.
        last_trans_val = trans_attr.Get(last_t)
        # Deliberately SMALL span (matching the walk clip's own tiled
        # length, not something huge) - get_animation_loop_seconds() in
        # unified_tracking.py scans EVERY SkelAnimation stage-wide for its
        # longest sample to set the one shared global timeline loop
        # duration. A large span here would make the whole scene's loop
        # balloon and freeze walking again (the exact bug fixed earlier) -
        # since held's two samples are IDENTICAL, any span works
        # functionally, so keep it small and harmless.
        HELD_SPAN = 24.0

        held_path = skel_root_path.AppendChild(HELD_CLIP_NAME)
        # Remove any existing held-clip prim first - this script has been
        # run multiple times with different authoring approaches (single
        # time-sample, default-value, two time-samples), and DefinePrim()
        # reuses an existing prim without clearing its old attribute data.
        # That almost certainly left a mangled mix of samples/default
        # values from earlier attempts sitting on this attribute, which is
        # very likely why ComputeJointLocalTransforms was returning nothing
        # at all for this clip (confirmed via test_held_clip_binding.py)
        # while the exact same code pattern worked fine for the fall/walk
        # clips (which were only ever authored once, cleanly). Wiping the
        # prim entirely guarantees a clean slate.
        if stage.GetPrimAtPath(held_path).IsValid():
            stage.RemovePrim(held_path)
        held_prim = stage.DefinePrim(held_path, "SkelAnimation")
        held_anim = UsdSkel.Animation(held_prim)
        held_anim.CreateJointsAttr().Set(joints)
        held_rot_attr = held_anim.CreateRotationsAttr()
        held_trans_attr = held_anim.CreateTranslationsAttr()
        held_rot_attr.Set(last_rot_val, 0.0)
        held_rot_attr.Set(last_rot_val, HELD_SPAN)
        held_trans_attr.Set(last_trans_val, 0.0)
        held_trans_attr.Set(last_trans_val, HELD_SPAN)

        # FALL has a populated scales attribute (identity scale per joint,
        # inherited from the FBX conversion); HELD never had one authored
        # at all. UsdSkelAnimQuery likely requires translations/rotations/
        # scales to be consistently present to resolve - a completely
        # missing attribute (not just identity values) may be treated as
        # invalid rather than defaulting to identity. Confirmed via
        # self-check further down: this was the actual cause.
        held_scale_attr = held_anim.CreateScalesAttr()
        from pxr import Gf as _Gf
        identity_scales = [_Gf.Vec3h(1.0, 1.0, 1.0)] * len(joints)
        held_scale_attr.Set(identity_scales, 0.0)
        held_scale_attr.Set(identity_scales, HELD_SPAN)

        # Verify the raw writes actually landed before blaming UsdSkel
        # internals any further.
        print(f"  [RAW CHECK] rotations time samples written: {held_rot_attr.GetTimeSamples()}")
        print(f"  [RAW CHECK] translations time samples written: {held_trans_attr.GetTimeSamples()}")
        readback_rot = held_rot_attr.Get(0.0)
        print(f"  [RAW CHECK] rotations readback at t=0.0: {'None' if readback_rot is None else str(len(readback_rot)) + ' values, first=' + str(readback_rot[0])}")
        readback_joints = held_anim.GetJointsAttr().Get()
        print(f"  [RAW CHECK] joints attr readback: {'None' if readback_joints is None else str(len(readback_joints)) + ' joints'}")

        # Self-check immediately, same session, before ever saving/
        # reopening - isolates whether the problem is in how this data is
        # being constructed vs. some save/reload round-trip issue.
        #
        # IMPORTANT: also test the FALL clip in this SAME standalone-file
        # context as a control - it worked fine in test_held_clip_binding.py
        # against the INTEGRATED warehouse_v1_animated.usd scene, but never
        # against the standalone character*.usd file in isolation. If FALL
        # also fails here, the standalone-file context itself is the
        # problem (not anything specific to HELD's construction).
        binding = UsdSkel.BindingAPI(skel_prim)
        existing_rel = binding.GetAnimationSourceRel()
        saved_targets = list(existing_rel.GetTargets()) if existing_rel else []

        for check_label, check_path in [("FALL (control)", fall_path), ("HELD", held_path)]:
            binding.CreateAnimationSourceRel().SetTargets([check_path])
            test_cache2 = UsdSkel.Cache()
            test_skel_query2 = test_cache2.GetSkelQuery(UsdSkel.Skeleton(skel_prim))
            test_anim_query = test_skel_query2.GetAnimQuery() if test_skel_query2 else None
            if test_anim_query is not None:
                test_xforms = test_anim_query.ComputeJointLocalTransforms(0.0)
                print(f"  [SELF-CHECK {check_label}] standalone file, immediately after authoring: "
                      f"{'SUCCESS, ' + str(len(test_xforms)) + ' joints' if test_xforms else 'EMPTY/FAILED'}")
            else:
                print(f"  [SELF-CHECK {check_label}] anim_query is None")

        if saved_targets:
            binding.GetAnimationSourceRel().SetTargets(saved_targets)

        print(f"{filename}: [ADDED] '{HELD_CLIP_NAME}' static held-pose clip at {held_path}")

        stage.GetRootLayer().Save()
        print(f"{filename}: [SAVED]")

    print("\nDONE.")


if __name__ == "__main__":
    main()
    simulation_app.close()
