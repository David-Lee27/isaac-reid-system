r"""
Diagnostic: finds every PhysicsScene prim actually present in the stage,
wherever it lives, instead of guessing a path. We'll use this to correctly
deactivate physics simulation entirely.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\find_physics_scenes.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
from pxr import UsdPhysics


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    print("\n--- Searching for PhysicsScene prims ---")
    found = []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            found.append(str(prim.GetPath()))
            print(f"  {prim.GetPath()}")

    if not found:
        print("  None found anywhere in the stage.")

    print("\n--- Checking robot's rigid bodies for their actual physics state ---")
    robot_root = stage.GetPrimAtPath("/World/mobile_manipulator_ros")
    if robot_root.IsValid():
        for prim in robot_root.GetChildren():
            print(f"  {prim.GetPath()} | type: {prim.GetTypeName()} | has RigidBodyAPI: {prim.HasAPI(UsdPhysics.RigidBodyAPI)}")


if __name__ == "__main__":
    main()
    simulation_app.close()
