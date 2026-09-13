"""
ISOLATED experiment - does NOT touch warehouse_v1.usd or any real project
file. Opens a brand new empty stage, references in one character, attempts
to reference/bind a real NVIDIA walk-cycle animation clip, and verifies
PROGRAMMATICALLY (comparing skeleton joint transforms at two different
timestamps) whether it's actually animating or stuck.

Tries several plausible animation clip filenames since the exact one isn't
confirmed - Isaac Sim will just fail to resolve whichever ones are wrong,
which itself tells us the real filename.

Run from PowerShell:
    C:\\isaacsim\\python.bat C:\\isaacsim\\projects\\surveillance-proj\\test_animation.py
"""

HEADLESS = False

# Toggle for testing whether the NVIDIA anim extensions help or hurt - the
# first real test run FAILED with "Buffer size mismatch: translations=81,
# rotations=81, expected jointCount=101" coming specifically from
# omni.anim.skelJoint.plugin (one of the extensions this script force-
# enables below). Core USD UsdSkel is built to handle joint-count
# mismatches gracefully via name-based mapping, unlike that NVIDIA runtime
# plugin's apparent rigid positional check - so it's worth testing whether
# skipping those extensions entirely lets plain UsdSkel resolve the
# animation correctly on its own.
ENABLE_ANIM_EXTENSIONS = False

CHARACTER_USD = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
                  "Assets/Isaac/6.0/Isaac/People/Characters/"
                  "original_male_adult_construction_01/male_adult_construction_01.usd")
# Previous test with male_adult_police_04 showed ZERO joint-name overlap
# with the animation clip - that character uses a Reallusion Character
# Creator rig ("RL_BoneRoot/Hip/Pelvis/..." naming), while the clip uses
# NVIDIA's own generic biped naming ("Root/Pelvis/R_UpLeg/..."). Testing a
# DIFFERENT character here in case it uses NVIDIA's native rig instead -
# cheaper to test than building a full manual joint retarget map.

# Another Isaac Sim user (NVIDIA GitHub discussion #464) confirmed this
# specific asset already has WORKING animation baked in out of the box -
# "I could play animations that were attached to biped_setup.usd" - unlike
# the named characters above, which have a skeleton but nothing bound.
# Trying several plausible paths/filenames for it since exact Isaac 6.0
# location isn't confirmed.
BIPED_SETUP_CANDIDATES = [
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/Biped_Setup.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/biped_demo/biped_demo_meters.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/biped_demo/biped_demo_meters.usd",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/biped_demo_meters.usd",
]

ANIMATIONS_BASE = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
                    "Assets/Isaac/6.0/Isaac/People/Animations/")

# CONFIRMED REAL via omni.client.list() (list_people_assets.py) - this is
# the actual filename, not a guess:
CANDIDATE_ANIM_FILES = [
    "stand_walk_loop.skelanim.usd",       # confirmed real, prioritized first
    "stand_walk_1.skelanim.usd",
    "stand_walk_loop_in_place.skelanim.usd",
]

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.kit.app
import omni.timeline
import omni.client
from pxr import Usd, UsdSkel, Sdf


def try_resolve(url):
    """Returns True if this asset path actually exists. NOTE: an earlier
    version of this used Sdf.Layer.FindOrOpen(), which was confirmed
    UNRELIABLE - it returned False even for paths later confirmed real via
    omni.client.list() (list_people_assets.py). omni.client.stat() is the
    proven-working check - it's what actually listed the real folder
    contents successfully."""
    try:
        result, entry = omni.client.stat(url)
        return result == omni.client.Result.OK
    except Exception:
        return False


def main():
    print("=" * 70)
    print("STEP 0: Checking if Biped_Setup (reportedly pre-animated) exists")
    print("=" * 70)
    biped_setup_url = None
    for candidate in BIPED_SETUP_CANDIDATES:
        ok = try_resolve(candidate)
        print(f"  {'FOUND' if ok else 'no  '}: {candidate}")
        if ok and biped_setup_url is None:
            biped_setup_url = candidate

    print("\n" + "=" * 70)
    print("STEP 1: Anim extensions (toggle: ENABLE_ANIM_EXTENSIONS)")
    print("=" * 70)
    if ENABLE_ANIM_EXTENSIONS:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        for ext_name in ("omni.anim.graph.core", "omni.anim.people", "omni.anim.skelJoint",
                          "omni.anim.graph.schema", "omni.anim.retarget.core"):
            try:
                ext_manager.set_extension_enabled_immediate(ext_name, True)
                print(f"  enabled: {ext_name}")
            except Exception as e:
                print(f"  could not enable {ext_name}: {e}")
        for _ in range(30):
            simulation_app.update()
    else:
        print("  SKIPPED - testing plain UsdSkel without NVIDIA anim extensions.")

    print("\n" + "=" * 70)
    print("STEP 2: Finding a walk animation clip filename (fallback path)")
    print("=" * 70)
    found_anim_url = None
    for candidate in CANDIDATE_ANIM_FILES:
        url = ANIMATIONS_BASE + candidate
        ok = try_resolve(url)
        print(f"  {'FOUND' if ok else 'no  '}: {url}")
        if ok and found_anim_url is None:
            found_anim_url = url

    if found_anim_url is None:
        print("\n  None of the guessed clip filenames resolved - fine if Biped_Setup")
        print("  already has its own animation bound (checked in step 4).")

    print("\n" + "=" * 70)
    print("STEP 3: Building isolated test stage")
    print("=" * 70)
    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    for _ in range(20):
        simulation_app.update()
    stage = usd_context.get_stage()

    used_biped_setup = biped_setup_url is not None
    char_prim = stage.DefinePrim("/World/TestCharacter", "Xform")
    char_prim.GetReferences().AddReference(biped_setup_url if used_biped_setup else CHARACTER_USD)
    for _ in range(30):
        simulation_app.update()

    # Find the actual Skeleton prim under the referenced character.
    skeleton_prim = None
    for descendant in Usd.PrimRange(char_prim):
        if descendant.IsA(UsdSkel.Skeleton):
            skeleton_prim = descendant
            break

    if skeleton_prim is None:
        print("  Could not find a Skeleton prim under the referenced character - aborting.")
        simulation_app.close()
        return
    print(f"  Skeleton found at: {skeleton_prim.GetPath()}")

    print("\n" + "=" * 70)
    print("STEP 4: Checking existing animationSource, or binding our own clip")
    print("=" * 70)
    existing_binding = UsdSkel.BindingAPI(skeleton_prim)
    existing_rel = existing_binding.GetAnimationSourceRel()
    existing_targets = existing_rel.GetTargets() if existing_rel else []
    if existing_targets:
        print(f"  Biped_Setup ALREADY has animationSource bound: {existing_targets}")
        print("  Skipping manual binding - testing this pre-existing animation as-is.")
    elif found_anim_url:
        anim_prim = stage.DefinePrim(f"{skeleton_prim.GetPath()}/TestAnim", "SkelAnimation")
        anim_prim.GetReferences().AddReference(found_anim_url)
        for _ in range(20):
            simulation_app.update()
        binding_api = UsdSkel.BindingAPI.Apply(skeleton_prim)
        binding_api.CreateAnimationSourceRel().SetTargets([anim_prim.GetPath()])
        print(f"  Bound {anim_prim.GetPath()} as animationSource on {skeleton_prim.GetPath()}")
    else:
        print("  No existing binding AND no animation clip found to manually bind.")
        print("  Proceeding anyway to see if it's still just a static T-pose.")

    print("\n" + "=" * 70)
    print("STEP 4.5: Comparing skeleton joint names vs animation clip joint names")
    print("=" * 70)
    skeleton_obj = UsdSkel.Skeleton(skeleton_prim)
    skel_joints_attr = skeleton_obj.GetJointsAttr()
    skel_joints = list(skel_joints_attr.Get()) if skel_joints_attr else []
    print(f"  Skeleton has {len(skel_joints)} joints. First 10: {skel_joints[:10]}")

    anim_joint_names = []
    if existing_targets:
        anim_target_prim = stage.GetPrimAtPath(existing_targets[0])
        anim_obj = UsdSkel.Animation(anim_target_prim)
    elif found_anim_url:
        anim_obj = UsdSkel.Animation(anim_prim)
    else:
        anim_obj = None
    if anim_obj:
        anim_joints_attr = anim_obj.GetJointsAttr()
        anim_joint_names = list(anim_joints_attr.Get()) if anim_joints_attr else []
    print(f"  Animation clip has {len(anim_joint_names)} joints. First 10: {anim_joint_names[:10]}")

    skel_set = set(str(j) for j in skel_joints)
    anim_set = set(str(j) for j in anim_joint_names)
    overlap = skel_set & anim_set
    print(f"  Joint NAME overlap between skeleton and clip: {len(overlap)} / {len(anim_set)} clip joints")
    if len(overlap) == 0:
        print("  ZERO overlap - completely different naming/hierarchy conventions.")
        print("  This is why nothing animates: UsdSkel can't map ANY joint by name.")
    elif len(overlap) < len(anim_set):
        print(f"  PARTIAL overlap - {len(anim_set) - len(overlap)} clip joints have no match.")
    else:
        print("  FULL overlap - all clip joints exist by name in the skeleton.")

    print("\n" + "=" * 70)
    print("STEP 5: Playing timeline and checking if joints ACTUALLY move")
    print("=" * 70)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    skeleton = UsdSkel.Skeleton(skeleton_prim)
    skel_cache = UsdSkel.Cache()
    skel_query = skel_cache.GetSkelQuery(skeleton)

    def get_joint_transforms(time_code):
        skel_cache.Clear()
        skel_query = skel_cache.GetSkelQuery(skeleton)
        xforms = skel_query.ComputeJointLocalTransforms(time_code)
        return xforms

    for _ in range(30):
        simulation_app.update()
    t0_xforms = get_joint_transforms(Usd.TimeCode(timeline.get_current_time() * timeline.get_time_codes_per_seconds()))

    for _ in range(120):
        simulation_app.update()
    t1_xforms = get_joint_transforms(Usd.TimeCode(timeline.get_current_time() * timeline.get_time_codes_per_seconds()))

    if t0_xforms is None or t1_xforms is None:
        print("  Could not compute joint transforms at all - binding likely failed.")
        print("  RESULT: FAILED (no joint transform data available)")
    else:
        differences = 0
        for a, b in zip(t0_xforms, t1_xforms):
            if a != b:
                differences += 1
        print(f"  Joints with different transforms between t0 and t1: {differences} / {len(t0_xforms)}")
        if differences > 5:
            print("  RESULT: SUCCESS - the skeleton is genuinely animating.")
        else:
            print("  RESULT: FAILED - joints are static (stuck pose, binding didn't take).")

    timeline.stop()
    print("\n" + "=" * 70)
    print("DONE. Paste this whole output back.")
    print("=" * 70)


if __name__ == "__main__":
    main()
    simulation_app.close()
