r"""
convert_new_characters.py

Converts the newly downloaded Mixamo character FBX files (assets\character.fbx,
character (1).fbx, etc.) to USD, same converter as convert_fbx_to_usd.py.
Run this first, then diagnose_new_characters.py to check whether they came
with baked animation or are bare/T-pose meshes needing the walk clip
reused from xbot_walk.usd.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\convert_new_characters.py
"""

import os
import asyncio

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as converter

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"

FILES = [
    "character.fbx",
    "character (1).fbx",
    "character (2).fbx",
    "character (3).fbx",
    "character (4).fbx",
]


async def convert(in_path, out_path):
    task_manager = converter.get_instance()
    task = task_manager.create_converter_task(in_path, out_path, progress_callback=None)
    success = await task.wait_until_finished()
    if not success:
        print(f"[CONVERT FAILED] {in_path} -> {out_path}: {task.get_status()}, {task.get_detailed_error()}")
    else:
        print(f"[CONVERT OK] {in_path} -> {out_path}")
    return success


async def main():
    for fbx_name in FILES:
        in_path = os.path.join(ASSETS_DIR, fbx_name)
        if not os.path.exists(in_path):
            print(f"[SKIP] missing: {in_path}")
            continue
        safe_name = fbx_name.replace(" ", "_").replace("(", "").replace(")", "").replace(".fbx", "")
        out_path = os.path.join(ASSETS_DIR, f"{safe_name}.usd")
        await convert(in_path, out_path)


asyncio.get_event_loop().run_until_complete(main())
simulation_app.close()
