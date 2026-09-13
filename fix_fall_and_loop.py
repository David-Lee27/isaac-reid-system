r"""
fix_fall_and_loop.py

Two fixes, both confirmed by diagnose_fall_translation.py's actual numbers
(not guessed):

1. Strip HORIZONTAL root drift from the fall clip (local X/Z, the ground
   plane) while KEEPING vertical (local Y - the actual standing-to-ground
   drop, confirmed going from ~97 to ~8.7 units). Same technique as
   strip_root_motion.py used on the walk clip. Without this, a fall could
   land the character up to ~1m away from where they actually collapsed -
   likely why the fallen person was hard to find.

2. TILE the walk clip's data (repeat its 31 samples 3x back-to-back, time-
   offset each repeat by its own 24.0-timecode span) so its own authored
   duration comfortably exceeds the fall clip's 53.6-timecode span. The
   global stage timeline can only have ONE loop boundary shared by every
   bound animation - get_animation_loop_seconds() in unified_tracking.py
   picks the LONGEST clip's end as that boundary, so with the fall clip
   present (even when not currently playing) the walk clip was holding
   frozen on its last frame for over a second every ~1s cycle, waiting for
   the much-longer fall-driven loop boundary. Tiling makes walk itself
   long enough that it never has idle time to freeze during - it just keeps
   stepping through repeated cycles for the whole loop duration. The walk
   clip already has zero horizontal drift (confirmed dx=dy=0), so tiling
   identical repeats is safe - no drift accumulation.

Edits assets\character.usd, character_1.usd, character_2.usd IN PLACE.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\fix_fall_and_loop.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel, Gf

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
TARGET_FILES = ["character.usd", "character_1.usd", "character_2.usd"]

WALK_CLIP_NAME = "mixamo_com"
FALL_CLIP_NAME = "mixamo_fall"
WALK_TILE_COUNT = 3  # 3 x 24.0 = 72.0, comfortably longer than fall's 53.6


def find_root_joint_index(anim):
    joints = anim.GetJointsAttr().Get()
    for i, j in enumerate(joints):
        if "Hips" in str(j) and str(j).count("/") == 0:
            return i
    return 0


def fix_fall_clip(stage, skel_root_path):
    anim_path = skel_root_path.AppendChild(FALL_CLIP_NAME)
    anim_prim = stage.GetPrimAtPath(anim_path)
    if not anim_prim.IsValid():
        print(f"  [SKIP fall] no {FALL_CLIP_NAME} at {anim_path}")
        return
    anim = UsdSkel.Animation(anim_prim)
    root_idx = find_root_joint_index(anim)
    trans_attr = anim.GetTranslationsAttr()
    fixed = 0
    for t in trans_attr.GetTimeSamples():
        values = trans_attr.Get(t)
        v = values[root_idx]
        new_values = list(values)
        # Keep Y (vertical drop-to-ground), zero X/Z (horizontal stumble drift).
        new_values[root_idx] = Gf.Vec3f(0.0, v[1], 0.0)
        trans_attr.Set(new_values, t)
        fixed += 1
    print(f"  [FIXED fall] stripped horizontal drift on {fixed} sample(s), kept vertical drop")


def tile_walk_clip(stage, skel_root_path):
    anim_path = skel_root_path.AppendChild(WALK_CLIP_NAME)
    anim_prim = stage.GetPrimAtPath(anim_path)
    if not anim_prim.IsValid():
        print(f"  [SKIP walk] no {WALK_CLIP_NAME} at {anim_path}")
        return
    anim = UsdSkel.Animation(anim_prim)
    rot_attr = anim.GetRotationsAttr()
    trans_attr = anim.GetTranslationsAttr()
    scale_attr = anim.GetScalesAttr()

    original_times = list(rot_attr.GetTimeSamples())
    if not original_times:
        print("  [SKIP walk] no time samples found")
        return
    span = original_times[-1] - original_times[0]

    # Capture original per-sample data before we start writing new samples.
    original_data = []
    for t in original_times:
        original_data.append((
            t,
            rot_attr.Get(t),
            trans_attr.Get(t) if trans_attr else None,
            scale_attr.Get(t) if scale_attr else None,
        ))

    total_written = 0
    for tile in range(WALK_TILE_COUNT):
        offset = tile * span
        for t, rot_val, trans_val, scale_val in original_data:
            new_t = t + offset
            if tile > 0 and t == original_times[0]:
                # Skip duplicate boundary sample (tile N's start == tile N-1's end)
                # to avoid writing two different values at the same timecode.
                continue
            if rot_val is not None:
                rot_attr.Set(rot_val, new_t)
            if trans_val is not None:
                trans_attr.Set(trans_val, new_t)
            if scale_val is not None:
                scale_attr.Set(scale_val, new_t)
            total_written += 1

    print(f"  [TILED walk] {WALK_TILE_COUNT}x repeat, span {span} -> {span * WALK_TILE_COUNT}, "
          f"{total_written} total samples written")


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
        print(f"\n{filename} ({skel_root_path}):")
        fix_fall_clip(stage, skel_root_path)
        tile_walk_clip(stage, skel_root_path)

        stage.GetRootLayer().Save()
        print(f"  [SAVED] {filename}")

    print("\nDONE.")


if __name__ == "__main__":
    main()
    simulation_app.close()
