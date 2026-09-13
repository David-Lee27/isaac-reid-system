r"""
integrate_xbot_characters.py

Replaces the 3 placeholder Reallusion-rigged People characters in
warehouse_v1.usd with the Mixamo X Bot (mesh + skeleton + baked walk
animation, converted via convert_fbx_to_usd.py), fixing the known
animationSource binding bug from the FBX conversion (Skeleton's
animationSource pointed at the empty "Take_001" clip instead of the real
31-sample "mixamo_com" clip).

STRUCTURE (important - this is a rewrite from the first version):

    /World/xbot_person_N            <- OUTER prim. Owns ONLY:
                                        - translate (waypoint position, hip-
                                          height offset baked in)
                                        - rotateZ (facing/yaw direction)
                                        unified_tracking.py's normal
                                        get_translate/set_translate/
                                        set_rotation calls target THIS prim
                                        exactly like the old placeholder
                                        characters did. No orientation-
                                        correction math lives here.

        /World/xbot_person_N/model  <- INNER child prim. Owns:
                                        - the actual reference to
                                          xbot_walk.usd
                                        - scale (cm -> m correction)
                                        - rotateX (CONSTANT Y-up -> Z-up
                                          correction, set once here, never
                                          touched again by any runtime code)
                                        - a SEPARATE rotateXYZ op used
                                          exclusively for the scripted fall
                                          event (default identity). Because
                                          it's a distinct xformOp of a
                                          different type from the rotateX
                                          base-correction op, the fall event
                                          can freely overwrite it without
                                          ever touching the base correction.

Previously, the base correction and the fall/facing rotation were crammed
into the SAME single rotateXYZ op on ONE prim, so triggering a fall
overwrote the orientation fix entirely and animation played back in the
uncorrected axis space (this produced the contorted/inside-out fall pose).
Splitting these across two prims and two distinct op types fixes that at
the structural level instead of patching call sites.

SAFETY: never touches warehouse_v1.usd directly. Opens it, does all edits
on the in-memory stage, and does a Save-As to warehouse_v1_animated.usd.
The original file is untouched no matter what happens.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\integrate_xbot_characters.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel, UsdGeom, Sdf, Gf

SOURCE_STAGE = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
OUTPUT_STAGE = r"C:\Users\popli\isaacsim\scenes\warehouse_v1_animated.usd"

# Switched from the single faceless X Bot to 3 of the textured Mixamo
# characters (assets\character.usd, character_1.usd, character_2.usd -
# converted via convert_new_characters.py, walk animation applied via
# apply_walk_animation.py which copied the already-fixed - root-motion-
# stripped - clip directly from xbot_walk.usd, since all Mixamo characters
# share identical skeleton/joint naming). X Bot has no face at all, which
# silently broke DeepFace re-ID at the checkpoint camera (every scan was
# failing with "Face could not be detected") - these have real faces.
PERSON_SOURCE_USDS = [
    r"C:\isaacsim\projects\surveillance-proj\assets\character.usd",
    r"C:\isaacsim\projects\surveillance-proj\assets\character_1.usd",
    r"C:\isaacsim\projects\surveillance-proj\assets\character_2.usd",
]

# Confirmed correct sign from the last visual check (upside down at -90,
# correct at +90).
BASE_X_ROTATION_DEG = 90.0

# Approximates hip height for the default-scale X Bot so feet land near
# floor level given the outer prim's translate is otherwise copied from the
# old characters' foot-level pivot. Adjust here if feet float or clip
# through the floor.
HIP_HEIGHT_OFFSET = 1.0

# cm -> m correction for Mixamo FBX units.
CM_TO_M = 0.01

OLD_PEOPLE_PATHS = [
    "/World/male_adult_police_04",
    "/World/male_adult_construction_03",
    "/World/male_adult_construction_01_new",
]

NEW_PEOPLE_PATHS = [
    "/World/xbot_person_1",
    "/World/xbot_person_2",
    "/World/xbot_person_3",
]


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(SOURCE_STAGE)
    for _ in range(60):
        simulation_app.update()
    stage = usd_context.get_stage()

    # Record old character world translations so new characters spawn in
    # the same physical spots.
    old_translates = []
    for path in OLD_PEOPLE_PATHS:
        prim = stage.GetPrimAtPath(path)
        translate = None
        if prim.IsValid():
            attr = prim.GetAttribute("xformOp:translate")
            if attr and attr.IsValid() and attr.HasValue():
                translate = attr.Get()
        old_translates.append(translate)
        print(f"[INFO] {path} translate={translate}")

    for path in OLD_PEOPLE_PATHS:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(path)
            print(f"[REMOVED] {path}")

    for i, new_path in enumerate(NEW_PEOPLE_PATHS):
        source_usd = PERSON_SOURCE_USDS[i % len(PERSON_SOURCE_USDS)]
        old_t = old_translates[i] if i < len(old_translates) else None
        tx, ty, tz = (old_t[0], old_t[1], old_t[2]) if old_t is not None else (0.0, 0.0, 0.0)

        # --- OUTER prim: translate (position) + rotateZ (facing) only ---
        outer = stage.DefinePrim(new_path, "Xform")
        outer_xf = UsdGeom.Xformable(outer)
        outer_xf.AddTranslateOp().Set(Gf.Vec3f(tx, ty, tz + HIP_HEIGHT_OFFSET))
        outer_xf.AddRotateZOp().Set(0.0)  # facing yaw, updated at runtime by unified_tracking.py
        print(f"[OUTER] {new_path}  translate=({tx}, {ty}, {tz + HIP_HEIGHT_OFFSET})")

        # --- INNER child "model": reference + scale + base correction + fall op ---
        model_path = f"{new_path}/model"
        model = stage.DefinePrim(model_path, "Xform")
        model.GetReferences().AddReference(source_usd)
        model_xf = UsdGeom.Xformable(model)

        # The referenced xbot_walk.usd already carries its own translate/
        # rotateXYZ/scale ops from the FBX conversion. Reuse the existing
        # scale op if present so we don't collide with it; otherwise add one.
        existing_ops = {op.GetOpType(): op for op in model_xf.GetOrderedXformOps()}

        scale_op = existing_ops.get(UsdGeom.XformOp.TypeScale) or model_xf.AddScaleOp()
        scale_op.Set(Gf.Vec3f(CM_TO_M, CM_TO_M, CM_TO_M))

        # Base correction: its own op, distinct TYPE (RotateX) from the
        # fall op below (RotateXYZ) so they can NEVER be confused/overwritten
        # by the same set_rotation() call.
        base_rot_op = model_xf.AddRotateXOp(opSuffix="base")
        base_rot_op.Set(BASE_X_ROTATION_DEG)

        # Fall op: identity by default. unified_tracking.py's fall-event
        # code targets THIS op specifically (finds the RotateXYZ op on the
        # model child), leaving base_rot_op completely untouched no matter
        # what it sets here.
        fall_rot_op = existing_ops.get(UsdGeom.XformOp.TypeRotateXYZ) or model_xf.AddRotateXYZOp()
        fall_rot_op.Set(Gf.Vec3f(0.0, 0.0, 0.0))

        # Explicit op order: translate/rotateXYZ(fall)/scale from the asset
        # first (in whatever order they were authored), THEN our base
        # correction last, since USD's xformOpOrder lists ops from
        # innermost/first-applied to outermost/last-applied - we want the
        # base correction applied last so it establishes the final "which
        # way is up" after the asset's own local transform and any fall
        # pose are already applied underneath it in local space.
        ordered = model_xf.GetOrderedXformOps()
        ordered = [op for op in ordered if op.GetOpType() != UsdGeom.XformOp.TypeRotateX]
        ordered.append(base_rot_op)
        model_xf.SetXformOpOrder(ordered)

        print(f"[INNER] {model_path} <- {source_usd}  base_rotateX={BASE_X_ROTATION_DEG} scale={CM_TO_M}")

    for _ in range(30):
        simulation_app.update()

    # Fix the animationSource binding bug: point every Skeleton under each
    # new character at the real baked clip (mixamo_com), not the empty one
    # (Take_001) the FBX converter wired up by default.
    fixed_count = 0
    for new_path in NEW_PEOPLE_PATHS:
        root_prim = stage.GetPrimAtPath(new_path)
        if not root_prim.IsValid():
            continue
        for descendant in Usd.PrimRange(root_prim):
            if descendant.IsA(UsdSkel.Skeleton):
                skel_path = descendant.GetPath()
                skel_root_prim = descendant.GetParent()  # mixamorig_Hips SkelRoot
                real_clip_path = skel_root_prim.GetPath().AppendChild("mixamo_com")
                binding = UsdSkel.BindingAPI(descendant)
                rel = binding.GetAnimationSourceRel()
                if not rel:
                    rel = binding.CreateAnimationSourceRel()
                rel.SetTargets([real_clip_path])
                fixed_count += 1
                print(f"[FIXED] {skel_path} animationSource -> {real_clip_path}")

    print(f"\n[SUMMARY] {fixed_count} skeleton(s) repointed to the real animation clip.")

    for _ in range(30):
        simulation_app.update()

    stage.GetRootLayer().Export(OUTPUT_STAGE)
    print(f"\n[SAVED] {OUTPUT_STAGE}")
    print("[UNTOUCHED] original warehouse_v1.usd was never modified.")


if __name__ == "__main__":
    main()
    simulation_app.close()
