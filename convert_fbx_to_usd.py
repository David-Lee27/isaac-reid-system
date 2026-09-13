r"""
convert_fbx_to_usd.py

Converts Mixamo FBX files (skeleton + skin + baked animation) into USD,
using Isaac Sim's own omni.kit.asset_converter extension. Run this with
Isaac Sim's python.bat, same as unified_tracking.py.

Run from Windows PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\convert_fbx_to_usd.py

Output:
    assets\xbot_walk.usd
    assets\xbot_idle.usd
"""

import os
import asyncio

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as converter

PROJECT_DIR = r"C:\isaacsim\projects\surveillance-proj"
ASSETS_DIR = os.path.join(PROJECT_DIR, "assets")

FILES = [
    ("xbot_walk.fbx", "xbot_walk.usd"),
    ("xbot_idle.fbx", "xbot_idle.usd"),
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
    results = {}
    for fbx_name, usd_name in FILES:
        in_path = os.path.join(ASSETS_DIR, fbx_name)
        out_path = os.path.join(ASSETS_DIR, usd_name)
        if not os.path.exists(in_path):
            print(f"[SKIP] missing input: {in_path}")
            results[fbx_name] = False
            continue
        ok = await convert(in_path, out_path)
        results[fbx_name] = ok
    print("\n=== SUMMARY ===")
    for fbx_name, ok in results.items():
        print(f"{fbx_name}: {'OK' if ok else 'FAILED'}")


asyncio.get_event_loop().run_until_complete(main())

simulation_app.close()
