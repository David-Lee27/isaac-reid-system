r"""
Standalone robot movement test - NO camera scanning, NO subprocess calls,
so nothing freezes. Tests the FULL fix: kinematic set before timeline.play(),
plus the follower-parts fix (camera_front/wheels move with the base).

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\robot_test.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False

ROBOT_BASE_PATH = "/World/mobile_manipulator_ros/base_footprint"
ROBOT_FOLLOWER_PATHS = [
    "/World/mobile_manipulator_ros/camera_front",
    "/World/mobile_manipulator_ros/camera_hand_link",
    "/World/mobile_manipulator_ros/wheel_left",
    "/World/mobile_manipulator_ros/wheel_right",
]
ROBOT_SPEED = 3.0

WAYPOINTS = [
    (20, 20, 0),
    (20, 0, 0),
    (-20, 0, 0),
    (-20, 20, 0),
]

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom, UsdPhysics
import time
import math


def get_translate(prim):
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op.Get()
    return Gf.Vec3d(0, 0, 0)


def set_translate(prim, pos):
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
            return
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))


def move_toward(current, target, step):
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    dist = math.hypot(dx, dy)
    if dist <= step or dist < 0.05:
        return (target[0], target[1], 0), True
    ratio = step / dist
    return (current[0] + dx * ratio, current[1] + dy * ratio, 0), False


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    base_prim = stage.GetPrimAtPath(ROBOT_BASE_PATH)
    if not base_prim.IsValid():
        print(f"ERROR: base_footprint not found at {ROBOT_BASE_PATH}")
        return

    # Kinematic set BEFORE timeline.play() - the ordering fix
    if base_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(base_prim)
        attr = rigid_api.GetKinematicEnabledAttr()
        if attr:
            attr.Set(True)
        else:
            rigid_api.CreateKinematicEnabledAttr(True)
        print("Set kinematicEnabled=True on base_footprint (before timeline.play())")

    start = get_translate(base_prim)

    followers = []
    for path in ROBOT_FOLLOWER_PATHS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"WARNING: follower not found at {path}")
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            f_api = UsdPhysics.RigidBodyAPI(prim)
            f_attr = f_api.GetKinematicEnabledAttr()
            if f_attr:
                f_attr.Set(True)
            else:
                f_api.CreateKinematicEnabledAttr(True)
        f_start = get_translate(prim)
        offset = (f_start[0] - start[0], f_start[1] - start[1], f_start[2] - start[2])
        followers.append({"prim": prim, "offset": offset})
    print(f"{len(followers)} follower parts set up.")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    position = (start[0], start[1], 0)
    print(f"Starting position: {position}")
    print("Patrolling indefinitely. Ctrl+C to stop. Watch: no falling, camera_front stays attached.")

    patrol_index = 0
    last_tick = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_tick
            last_tick = now

            target = WAYPOINTS[patrol_index]
            step = ROBOT_SPEED * dt
            position, arrived = move_toward(position, target, step)
            set_translate(base_prim, position)

            for f in followers:
                ox, oy, oz = f["offset"]
                set_translate(f["prim"], (position[0] + ox, position[1] + oy, position[2] + oz))

            if arrived:
                patrol_index = (patrol_index + 1) % len(WAYPOINTS)
                print(f"Reached waypoint {patrol_index}, position: {position}")

            simulation_app.update()

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()
