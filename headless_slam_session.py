r"""
Headless SLAM-mapping session. Boots Isaac Sim with NO viewport rendering
(much faster, since rendering is almost always the real bottleneck) and
just keeps physics + the ROS2 bridge running, so slam_toolbox in WSL can
get clean, high-rate data while you drive the robot via /cmd_vel.

Run from PowerShell (remember to run set_ros_env.ps1 first if it's a new session):
    . C:\isaacsim\projects\surveillance-proj\set_ros_env.ps1
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\headless_slam_session.py

Then drive the robot and watch RViz2 from WSL, same as before - just no
Isaac Sim window to watch this time.

Press Ctrl+C in this terminal to stop.
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
ROBOT_BASE_PATH = "/World/mobile_manipulator_ros/base_footprint"

# Isaac Sim's ROS2 bridge extension needs its OWN internal environment
# variables set (pointing at its bundled ROS2 libraries) before it can
# start - this is unrelated to our WSL/Jazzy discovery setup. Launching via
# isaac-sim.bat (the GUI) sets these automatically; launching via python.bat
# directly (this script) does not, which is why the bridge failed to start
# and immediately shut back down. Must be set BEFORE importing isaacsim.
import os
os.environ["ROS_DISTRO"] = "humble"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["PATH"] = os.environ["PATH"] + ";c:/isaacsim/exts/isaacsim.ros2.core/humble/lib"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.timeline
import omni.kit.app
from pxr import UsdPhysics
import time

# Explicitly enable the ROS2 bridge extension - toggling it on manually in
# the GUI only applies to that running session, it does NOT carry over to
# a fresh headless script launch, which starts its own process with default
# extension state. Confirmed this was the real cause of /odom and /cmd_vel
# being completely absent in headless mode.
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
print("ROS2 bridge extension explicitly enabled.")
for _ in range(20):
    simulation_app.update()


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    # Make sure the robot is in genuine physics mode (NOT kinematic) so it
    # actually responds to real /cmd_vel commands and wheel physics.
    base_prim = stage.GetPrimAtPath(ROBOT_BASE_PATH)
    if base_prim.IsValid() and base_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(base_prim)
        attr = rigid_api.GetKinematicEnabledAttr()
        if attr:
            attr.Set(False)
        print(f"Confirmed {ROBOT_BASE_PATH} is in real physics mode (kinematic off)")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    print("=== Headless SLAM session running. Drive the robot via /cmd_vel from WSL now. ===")
    print("=== Press Ctrl+C here to stop. ===")

    try:
        while True:
            simulation_app.update()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()
