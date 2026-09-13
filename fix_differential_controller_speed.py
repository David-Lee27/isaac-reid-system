r"""
fix_differential_controller_speed.py

Found via diagnose_robot_drive.py: the robot's differential_controller
OmniGraph node (sits between Nav2's /cmd_vel output and the actual wheel
joints) had maxLinearSpeed/maxAngularSpeed/maxWheelSpeed/maxAcceleration/
maxAngularAcceleration/maxDeceleration all completely unauthored (None) -
meaning it was silently falling back to whatever conservative built-in
default this node type ships with, entirely independent of and bypassing
anything configured in nav2_params.yaml. This is why raising Nav2's own
velocity limits (1.2 -> 10.0 m/s) had zero effect on real robot speed -
the actual bottleneck was always here, downstream of Nav2 entirely.

Explicitly authors generous values on this node directly.

Edits warehouse_v1_animated.usd - the integrated scene, not the original
warehouse_v1.usd (which this NEVER touches) or the individual character
assets.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\fix_differential_controller_speed.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, Sdf

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1_animated.usd"
NODE_PATH = "/World/mobile_manipulator_ros/Graph/differential_controller/ActionGraph/differential_controller"

# Generous values - matches/exceeds the nav2_params.yaml limits we already
# set, so Nav2's own config is the binding constraint again, not this node.
VALUES = {
    "inputs:maxLinearSpeed": 10.0,
    "inputs:maxAngularSpeed": 6.0,
    "inputs:maxWheelSpeed": 20.0,
    "inputs:maxAcceleration": 15.0,
    "inputs:maxAngularAcceleration": 10.0,
    "inputs:maxDeceleration": 15.0,
}


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    node_prim = stage.GetPrimAtPath(NODE_PATH)
    if not node_prim.IsValid():
        print(f"[ERROR] node not found at {NODE_PATH}")
        return

    print(f"Found node: {node_prim.GetPath()}")
    for attr_name, value in VALUES.items():
        attr = node_prim.GetAttribute(attr_name)
        if not attr.IsValid():
            print(f"  [WARN] attribute {attr_name} doesn't exist on this node - skipping "
                  f"(the node schema may name it differently; check the printout above for the real name)")
            continue
        old_val = attr.Get()
        attr.Set(value)
        print(f"  [SET] {attr_name}: {old_val} -> {value}")

    stage.GetRootLayer().Save()
    print(f"\n[SAVED] {STAGE_PATH}")


if __name__ == "__main__":
    main()
    simulation_app.close()
