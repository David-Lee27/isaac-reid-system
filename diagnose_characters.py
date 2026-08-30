"""
Read-only diagnostic: inspects the People character assets to find out what
animation data (if any) actually exists on them - SkelRoot/Skeleton/
SkelAnimation prims, available clips, joint counts, etc. Also checks if the
isaacsim.people extension (built for exactly this - walking NPCs on a
navmesh) is available.

Makes NO changes to the stage. Run once, paste the full output back.

Run from PowerShell:
    C:\\isaacsim\\python.bat C:\\isaacsim\\projects\\surveillance-proj\\diagnose_characters.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False

PEOPLE_PATHS = [
    "/World/male_adult_police_04",
    "/World/male_adult_construction_03",
    "/World/male_adult_construction_01_new",
]

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.kit.app
from pxr import Usd, UsdSkel, UsdGeom


def describe_prim_tree(prim, depth=0, max_depth=6):
    indent = "  " * depth
    type_name = prim.GetTypeName()
    marker = ""
    if prim.IsA(UsdSkel.Root):
        marker = "  <-- SkelRoot"
    elif prim.IsA(UsdSkel.Skeleton):
        marker = "  <-- Skeleton"
    elif prim.IsA(UsdSkel.Animation):
        marker = "  <-- SkelAnimation"
    print(f"{indent}{prim.GetName()} [{type_name}]{marker}")
    if depth >= max_depth:
        return
    for child in prim.GetChildren():
        describe_prim_tree(child, depth + 1, max_depth)


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = usd_context.get_stage()

    print("\n" + "=" * 70)
    print("CHECKING isaacsim.people EXTENSION AVAILABILITY")
    print("=" * 70)
    try:
        manager = omni.kit.app.get_app().get_extension_manager()
        all_exts = manager.fetch_extension_summaries()
        people_exts = [e for e in all_exts if "people" in e["name"].lower()
                       or "character" in e["name"].lower() or "crowd" in e["name"].lower()]
        if people_exts:
            for e in people_exts:
                print(f"  FOUND: {e['name']} (enabled={e.get('enabled', '?')})")
        else:
            print("  No people/character/crowd extensions found in the registry.")
    except Exception as e:
        print(f"  Skipping extension check (API mismatch, not critical): {e}")

    for path in PEOPLE_PATHS:
        print("\n" + "=" * 70)
        print(f"PRIM TREE: {path}")
        print("=" * 70)
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print("  INVALID PRIM - not found at this path")
            continue
        describe_prim_tree(prim)

        print(f"\n  --- Root prim source layers (where this asset was loaded from) ---")
        for spec in prim.GetPrimStack():
            print(f"    {spec.layer.identifier}")
        for ref in prim.GetMetadata("references") or []:
            print(f"    reference: {ref}")

        # Specifically look for SkelRoot + its bound Skeleton + any
        # SkelAnimation prims, and report joint/clip info if found.
        print(f"\n  --- SkelRoot/Skeleton/Animation detail for {path} ---")
        found_any_skel = False
        for descendant in Usd.PrimRange(prim):
            if descendant.IsA(UsdSkel.Root):
                found_any_skel = True
                skel_root = UsdSkel.Root(descendant)
                print(f"  SkelRoot at: {descendant.GetPath()}")
                binding_api = UsdSkel.BindingAPI(descendant)
                skel_rel = binding_api.GetSkeletonRel()
                targets = skel_rel.GetTargets() if skel_rel else []
                print(f"    bound skeleton targets: {targets}")
            if descendant.IsA(UsdSkel.Skeleton):
                found_any_skel = True
                skel = UsdSkel.Skeleton(descendant)
                joints_attr = skel.GetJointsAttr()
                joints = joints_attr.Get() if joints_attr else None
                joint_count = len(joints) if joints else 0
                print(f"  Skeleton at: {descendant.GetPath()} ({joint_count} joints)")
                anim_binding = UsdSkel.BindingAPI(descendant)
                anim_rel = anim_binding.GetAnimationSourceRel()
                anim_targets = anim_rel.GetTargets() if anim_rel else []
                print(f"    animationSource targets: {anim_targets}")
                # Show the actual source layer(s)/file(s) this prim comes
                # from - tells us where the original asset (and likely a
                # companion walk-animation file) actually lives.
                print("    prim stack (source files, strongest first):")
                for spec in descendant.GetPrimStack():
                    layer_path = spec.layer.identifier
                    print(f"      {layer_path}")
            if descendant.IsA(UsdSkel.Animation):
                found_any_skel = True
                anim = UsdSkel.Animation(descendant)
                joints_attr = anim.GetJointsAttr()
                joints = joints_attr.Get() if joints_attr else None
                translations_attr = anim.GetTranslationsAttr()
                num_time_samples = translations_attr.GetNumTimeSamples() if translations_attr else 0
                print(f"  SkelAnimation at: {descendant.GetPath()}")
                print(f"    joints animated: {len(joints) if joints else 0}")
                print(f"    translation time samples: {num_time_samples}")
                if translations_attr:
                    time_samples = translations_attr.GetTimeSamples()
                    if time_samples:
                        print(f"    time range: {time_samples[0]} to {time_samples[-1]}")
        if not found_any_skel:
            print("  NO UsdSkel Root/Skeleton/Animation prims found anywhere in this subtree.")
            print("  This asset likely has NO baked animation data at all (static mesh only).")

    print("\n" + "=" * 70)
    print("DONE. Paste this whole output back.")
    print("=" * 70)


if __name__ == "__main__":
    main()
    simulation_app.close()
