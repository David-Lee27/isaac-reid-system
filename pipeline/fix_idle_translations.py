r"""
fix_idle_translations.py

Repairs the idle clips written by the old bind_idle_animations.py. That script
zeroed the translation of EVERY joint at every sample ("root motion stripped"),
but a skeleton animation's per-joint translations are the bone offsets - zero
them all and every bone collapses onto its parent, so the character renders as
a lump. Only the root's motion should be removed.

For each character file this copies the walk clip's ("mixamo_com") first-sample
translations - the rig's real bone offsets, with no root travel - into every
time sample of the four idle clips. Rotations are untouched.

Run (idempotent):
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\fix_idle_translations.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdSkel

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
TARGET_FILES = ["character.usd", "character_1.usd", "character_2.usd", "character_3.usd", "character_4.usd"]
IDLE_CLIPS = ["mixamo_idle_stand", "mixamo_idle_salute", "mixamo_idle_excited", "mixamo_idle_phone"]

for filename in TARGET_FILES:
    stage = Usd.Stage.Open(f"{ASSETS_DIR}\\{filename}")
    skel_prim = next((p for p in stage.Traverse() if p.IsA(UsdSkel.Skeleton)), None)
    if skel_prim is None:
        print(f"[SKIP] {filename}: no skeleton")
        continue
    root = skel_prim.GetParent().GetPath()
    walk = UsdSkel.Animation(stage.GetPrimAtPath(root.AppendChild("mixamo_com")))
    walk_attr = walk.GetTranslationsAttr()
    walk_samples = walk_attr.GetTimeSamples()
    if not walk_samples:
        print(f"[SKIP] {filename}: walk clip has no translation samples")
        continue
    base = walk_attr.Get(walk_samples[0])
    for clip_name in IDLE_CLIPS:
        prim = stage.GetPrimAtPath(root.AppendChild(clip_name))
        if not prim.IsValid():
            continue
        attr = UsdSkel.Animation(prim).GetTranslationsAttr()
        samples = attr.GetTimeSamples()
        for t in samples:
            attr.Set(base, t)
        print(f"[OK] {filename}: {clip_name} - restored bone offsets on {len(samples)} samples")
    stage.GetRootLayer().Save()

simulation_app.close()
