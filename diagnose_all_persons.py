r"""
diagnose_all_persons.py

Checks all 3 integrated characters in warehouse_v1_animated.usd for the
actual root cause of T-posing/floating/fall-not-working, instead of
guessing further:
  - Does each Skeleton's joints list match its bound SkelAnimation's
    joints list EXACTLY (a mismatch means USD can't resolve joint indices
    and silently renders bind pose = T-pose)?
  - What's the actual authored world-space translate on each outer prim
    (checking the hip-height/floating question)?
  - What's the model child's current fall-rotation op value (checking
    whether the fall event is actually writing anything)?

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_all_persons.py
"""

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1_animated.usd"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel, UsdGeom

PERSON_PATHS = ["/World/xbot_person_1", "/World/xbot_person_2", "/World/xbot_person_3"]


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    for person_path in PERSON_PATHS:
        print("\n" + "=" * 70)
        print(person_path)
        print("=" * 70)

        outer = stage.GetPrimAtPath(person_path)
        if not outer.IsValid():
            print("  [MISSING] outer prim not found")
            continue

        t_attr = outer.GetAttribute("xformOp:translate")
        print(f"  outer translate: {t_attr.Get() if t_attr and t_attr.IsValid() else 'MISSING'}")

        model = stage.GetPrimAtPath(person_path + "/model")
        if not model.IsValid():
            print("  [MISSING] model child not found")
            continue

        model_xf = UsdGeom.Xformable(model)
        for op in model_xf.GetOrderedXformOps():
            print(f"  model op: {op.GetOpName()} type={op.GetOpType()} value={op.Get()}")

        skel_prim = None
        for prim in Usd.PrimRange(model):
            if prim.IsA(UsdSkel.Skeleton):
                skel_prim = prim
                break
        if skel_prim is None:
            print("  [MISSING] no Skeleton under model")
            continue

        skel = UsdSkel.Skeleton(skel_prim)
        skel_joints = skel.GetJointsAttr().Get()
        print(f"  Skeleton path: {skel_prim.GetPath()}")
        print(f"  Skeleton joints: {len(skel_joints) if skel_joints else 0}")

        binding = UsdSkel.BindingAPI(skel_prim)
        targets = binding.GetAnimationSourceRel().GetTargets() if binding.GetAnimationSourceRel() else []
        print(f"  animationSource targets: {targets}")

        if targets:
            anim_prim = stage.GetPrimAtPath(targets[0])
            if anim_prim.IsValid():
                anim = UsdSkel.Animation(anim_prim)
                anim_joints = anim.GetJointsAttr().Get()
                print(f"  bound SkelAnimation joints: {len(anim_joints) if anim_joints else 0}")
                if skel_joints and anim_joints:
                    skel_set = list(skel_joints)
                    anim_set = list(anim_joints)
                    matches = skel_set == anim_set
                    print(f"  JOINTS MATCH EXACTLY: {matches}")
                    if not matches:
                        print(f"    skel[0:3]  = {skel_set[:3]}")
                        print(f"    anim[0:3]  = {anim_set[:3]}")
                        # find first mismatch index
                        for idx in range(min(len(skel_set), len(anim_set))):
                            if skel_set[idx] != anim_set[idx]:
                                print(f"    first mismatch at index {idx}: skel='{skel_set[idx]}' vs anim='{anim_set[idx]}'")
                                break
                rot_attr = anim.GetRotationsAttr()
                n_samples = rot_attr.GetNumTimeSamples() if rot_attr else 0
                print(f"  bound SkelAnimation rotation time samples: {n_samples}")
            else:
                print(f"  [BROKEN] animationSource target prim doesn't exist: {targets[0]}")

    print("\nDONE. Paste this output back.")


if __name__ == "__main__":
    main()
    simulation_app.close()
