r"""
test_held_clip_binding.py

Directly tests whether binding a Skeleton's animationSource to the
"mixamo_fall_held" clip actually resolves to a real (non-identity/non-
T-pose) computed joint transform, using UsdSkelCache/UsdSkelAnimQuery
directly - bypassing the live running sim entirely. This isolates whether
the problem is in how the held clip's data is authored, or somewhere else
entirely (e.g. the live sim's timing/switching logic, or a completely
unrelated per-character binding issue).

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\test_held_clip_binding.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel, UsdGeom

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1_animated.usd"
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

        model = stage.GetPrimAtPath(person_path + "/model")
        skel_prim = None
        for prim in Usd.PrimRange(model):
            if prim.IsA(UsdSkel.Skeleton):
                skel_prim = prim
                break
        if skel_prim is None:
            print("[ERROR] no skeleton found")
            continue

        skel = UsdSkel.Skeleton(skel_prim)
        joints = skel.GetJointsAttr().Get()
        arm_idx = None
        for i, j in enumerate(joints):
            if "LeftArm" in str(j) and "Fore" not in str(j) and "Hand" not in str(j):
                arm_idx = i
                break
        print(f"  arm joint index: {arm_idx} ({joints[arm_idx] if arm_idx is not None else 'NOT FOUND'})")

        skel_root_path = skel_prim.GetParent().GetPath()
        walk_path = skel_root_path.AppendChild("mixamo_com")
        fall_path = skel_root_path.AppendChild("mixamo_fall")
        held_path = skel_root_path.AppendChild("mixamo_fall_held")

        for label, clip_path in [("WALK", walk_path), ("FALL", fall_path), ("HELD", held_path)]:
            binding = UsdSkel.BindingAPI(skel_prim)
            rel = binding.GetAnimationSourceRel() or binding.CreateAnimationSourceRel()
            rel.SetTargets([clip_path])

            cache = UsdSkel.Cache()
            skel_query = cache.GetSkelQuery(skel)
            anim_query = skel_query.GetAnimQuery() if skel_query else None

            print(f"\n  --- {label} ({clip_path}) ---")
            if not skel_query:
                print("    [ERROR] skel_query is None (skeleton not resolvable)")
                continue
            if anim_query is None:
                print("    [ERROR] anim_query is None")
                continue

            for t in [0.0, 1.0, 10.0]:
                try:
                    xforms = anim_query.ComputeJointLocalTransforms(t)
                except Exception as e:
                    print(f"    t={t}: EXCEPTION computing transforms: {e}")
                    continue
                if xforms is None or len(xforms) == 0:
                    print(f"    t={t}: NO TRANSFORMS RETURNED")
                    continue
                if arm_idx is not None and arm_idx < len(xforms):
                    arm_rot = xforms[arm_idx].ExtractRotationQuat()
                    print(f"    t={t}: arm joint local rotation quat: {arm_rot}")
                else:
                    print(f"    t={t}: (no arm joint to check) root translation: {xforms[0].ExtractTranslation()}")

    print("\nDONE.")


if __name__ == "__main__":
    main()
    simulation_app.close()
