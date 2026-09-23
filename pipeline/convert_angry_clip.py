r"""
convert_angry_clip.py

Converts the Angry reaction FBX (user-supplied Mixamo clip, dropped into
assets/) to USD - same converter as convert_sad_walk.py, its own dedicated
script for the same reason: running the FBX->USD async converter task and
then immediately synchronously opening stages in the same script/event-loop
corrupts the converter's next call ("no running event loop"/"Cannot enter
into task"). See convert_sad_walk.py's own docstring for the full story.

Run from PowerShell, THEN run bind_denial_reaction_clips.py afterward (it
skips any clip it already successfully bound - see its CLIPS list):
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\pipeline\convert_angry_clip.py
"""
import asyncio
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as converter

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"
IN_PATH = f"{ASSETS_DIR}\\Ch27_nonPBR@Angry.fbx"
OUT_PATH = f"{ASSETS_DIR}\\angry_reaction_clip_source.usd"


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
