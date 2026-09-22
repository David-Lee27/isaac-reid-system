r"""
convert_idle_clips.py

Converts the 4 new idle/social animation FBXs (user-supplied Mixamo clips,
dropped into assets/) to USD - same converter as convert_fall_clip.py /
convert_get_up_clip.py, just looped over all 4 instead of one script per
clip since they're all doing the exact same conversion step.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\convert_idle_clips.py
"""

import asyncio
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as converter

ASSETS_DIR = r"C:\isaacsim\projects\surveillance-proj\assets"

# (source fbx filename, output clip source usd filename)
CLIPS = [
    ("Ch27_nonPBR@Idle.fbx", "idle_stand_clip_source.usd"),
    ("Ch27_nonPBR@Salute.fbx", "idle_salute_clip_source.usd"),
    ("Ch27_nonPBR@Excited.fbx", "idle_excited_clip_source.usd"),
    ("Ch27_nonPBR@Talking On Phone.fbx", "idle_phone_clip_source.usd"),
]


async def convert_one(in_path, out_path):
    task_manager = converter.get_instance()
    task = task_manager.create_converter_task(in_path, out_path, progress_callback=None)
    success = await task.wait_until_finished()
    if not success:
        print(f"[CONVERT FAILED] {in_path} -> {out_path}: {task.get_status()}, {task.get_detailed_error()}")
    else:
        print(f"[CONVERT OK] {in_path} -> {out_path}")


async def main():
    for fbx_name, usd_name in CLIPS:
        in_path = f"{ASSETS_DIR}\\{fbx_name}"
        out_path = f"{ASSETS_DIR}\\{usd_name}"
        await convert_one(in_path, out_path)


asyncio.get_event_loop().run_until_complete(main())
simulation_app.close()
