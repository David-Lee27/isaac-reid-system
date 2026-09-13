r"""
convert_fall_clip.py

Converts the fall animation FBX (from a "Ch27" Mixamo character, source of
the fall clip only - we don't use Ch27's mesh, just its animation data) to
USD, same converter as convert_fbx_to_usd.py.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\convert_fall_clip.py
"""

import asyncio
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as converter

IN_PATH = r"C:\isaacsim\projects\surveillance-proj\assets\fall_clip_source.fbx"
OUT_PATH = r"C:\isaacsim\projects\surveillance-proj\assets\fall_clip_source.usd"


async def main():
    task_manager = converter.get_instance()
    task = task_manager.create_converter_task(IN_PATH, OUT_PATH, progress_callback=None)
    success = await task.wait_until_finished()
    if not success:
        print(f"[CONVERT FAILED] {IN_PATH} -> {OUT_PATH}: {task.get_status()}, {task.get_detailed_error()}")
    else:
        print(f"[CONVERT OK] {IN_PATH} -> {OUT_PATH}")


asyncio.get_event_loop().run_until_complete(main())
simulation_app.close()
