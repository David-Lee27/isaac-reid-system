r"""
convert_sad_walk.py

Converts the Sad Walk FBX to USD only (the Sad Idle clip already converted and
bound cleanly via bind_sad_clips.py in one pass, but running the FBX->USD async
converter task and then immediately synchronously opening stages in the same
script/event-loop corrupted the converter's second call - "no running event
loop"/"Cannot enter into task" - so, same as this project's existing idle-clip
pipeline (convert_idle_clips.py is its own separate run from
bind_idle_animations.py), conversion gets its own dedicated script/process.

Run from PowerShell, THEN run bind_sad_clips.py again afterward (it skips any
clip it already successfully bound - see its CLIPS list):
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\convert_sad_walk.py
"""
import asyncio
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as converter

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
IN_PATH = f"{ASSETS_DIR}\\Ch27_nonPBR@Sad Walk.fbx"
OUT_PATH = f"{ASSETS_DIR}\\sad_walk_clip_source.usd"


async def main():
    task_manager = converter.get_instance()
    task = task_manager.create_converter_task(IN_PATH, OUT_PATH, progress_callback=None)
    success = await task.wait_until_finished()
    if not success:
        print(f"[CONVERT FAILED] {IN_PATH} -> {OUT_PATH}: {task.get_status()}")
    else:
        print(f"[CONVERT OK] {IN_PATH} -> {OUT_PATH}")


asyncio.get_event_loop().run_until_complete(main())
simulation_app.close()
