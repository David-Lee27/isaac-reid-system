r"""
convert_get_up_clip.py

Converts the "getting up" animation FBX (user-supplied, Ch27_nonPBR@Getting
Up.fbx) to USD, same converter as convert_fall_clip.py.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\convert_get_up_clip.py
"""

import asyncio
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as converter

IN_PATH = r"C:\isaacsim\projects\surveillance-proj\assets\Ch27_nonPBR@Getting Up.fbx"
OUT_PATH = r"C:\isaacsim\projects\surveillance-proj\assets\get_up_clip_source.usd"


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
