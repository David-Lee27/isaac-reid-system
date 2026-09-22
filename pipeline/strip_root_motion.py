r"""
strip_root_motion.py

The Mixamo "Walking" clip wasn't downloaded "In Place", so the Hips joint's
baked translation actually carries the character forward within the
animation itself - stacking with unified_tracking.py's own waypoint-driven
translation. Result: the character visibly overshoots each loop, then
snaps back to the start position when the clip loops back to frame 0
("teleports backward").

Fix: zero out the HORIZONTAL (X/Y) translation on the root joint (Hips)
across every baked time sample, leaving vertical (Z) motion intact so the
natural up/down bob of a walk cycle survives. All the actual walking
motion (leg swing etc) lives in joint ROTATIONS, not the root's own
translation, so this only removes forward drift - it does not affect the
walk cycle itself.

Edits assets\xbot_walk.usd IN PLACE (this is our own generated/derived
asset, not the original scene file - warehouse_v1.usd is never touched).
Since warehouse_v1_animated.usd references this file rather than
flattening it, this fix applies automatically with no need to regenerate
the integrated scene.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\strip_root_motion.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel, Gf

XBOT_WALK_USD = r"C:\isaacsim\projects\surveillance-proj\assets\xbot_walk.usd"
REAL_CLIP_NAME = "mixamo_com"  # the clip with actual baked samples, not "Take_001"


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(XBOT_WALK_USD)
    for _ in range(30):
        simulation_app.update()
    stage = usd_context.get_stage()

    anim_prim = None
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdSkel.Animation) and prim.GetName() == REAL_CLIP_NAME:
            anim_prim = prim
            break

    if anim_prim is None:
        print(f"[ERROR] could not find SkelAnimation named '{REAL_CLIP_NAME}' in {XBOT_WALK_USD}")
        return

    anim = UsdSkel.Animation(anim_prim)
    joints = anim.GetJointsAttr().Get()
    if not joints:
        print("[ERROR] animation has no joints list")
        return

    root_joint_index = None
    for i, j in enumerate(joints):
        if "Hips" in str(j) and str(j).count("/") == 0:
            root_joint_index = i
            break
    if root_joint_index is None:
        root_joint_index = 0  # fall back to the first joint, USD convention for the root

    print(f"[INFO] root joint: {joints[root_joint_index]} (index {root_joint_index})")

    trans_attr = anim.GetTranslationsAttr()
    time_samples = trans_attr.GetTimeSamples()
    print(f"[INFO] {len(time_samples)} time samples on translations attr")

    fixed = 0
    for t in time_samples:
        values = trans_attr.Get(t)
        if values is None or root_joint_index >= len(values):
            continue
        v = values[root_joint_index]
        new_values = list(values)
        # Zero X/Y (horizontal drift), KEEP Z (vertical bob of the walk cycle).
        new_values[root_joint_index] = Gf.Vec3f(0.0, 0.0, v[2])
        trans_attr.Set(new_values, t)
        fixed += 1

    print(f"[FIXED] zeroed horizontal root translation on {fixed} time sample(s)")

    stage.GetRootLayer().Save()
    print(f"[SAVED] {XBOT_WALK_USD} (in place)")


if __name__ == "__main__":
    main()
    simulation_app.close()
