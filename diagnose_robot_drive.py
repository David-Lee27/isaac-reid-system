r"""
diagnose_robot_drive.py

nav2_params.yaml's velocity limits were confirmed loaded (ros2 param get
showed vx_max=10.0), but real robot speed didn't change at all. That gap
points at something DOWNSTREAM of Nav2 - the actual OmniGraph node(s) that
take /cmd_vel and turn it into wheel joint velocities - having its own,
separate max-speed clamp that Nav2's config can't touch. This inspects the
robot's ActionGraph for exactly that.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_robot_drive.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1_animated.usd"
ROBOT_PRIM_PATH = "/World/mobile_manipulator_ros"


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        print(f"[ERROR] robot prim not found at {ROBOT_PRIM_PATH}")
        return

    print("=" * 70)
    print("Searching for OmniGraph / controller-related prims under the robot...")
    print("=" * 70)

    keywords = ["controller", "differential", "drive", "wheel", "articulation", "twist", "cmd_vel"]

    for prim in Usd.PrimRange(robot_prim):
        type_name = str(prim.GetTypeName())
        name_lower = prim.GetName().lower()
        if any(kw in name_lower for kw in keywords) or "Graph" in type_name or "Node" in type_name:
            print(f"\n{prim.GetPath()}  [{type_name}]")
            for attr in prim.GetAttributes():
                attr_name = attr.GetName()
                if any(kw in attr_name.lower() for kw in
                       ["speed", "velocity", "max", "min", "limit", "gain", "scale"]):
                    try:
                        val = attr.Get()
                    except Exception:
                        val = "<error reading>"
                    print(f"    {attr_name} = {val}")

    print("\nDONE. Paste this whole output back.")


if __name__ == "__main__":
    main()
    simulation_app.close()
