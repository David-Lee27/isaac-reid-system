"""
Moves people along a perimeter patrol loop (position-only, no walk animation
yet), staggered so they don't all move at once and bump into each other.
Disables physics on each person first to fix the "falls through floor" issue.

Run from PowerShell (leave Isaac Sim window visible to watch it happen):
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\move_people.py
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\warehouse_v1.usd"
HEADLESS = False

# Each person's real prim path + how many seconds to wait before they start
# moving, so they don't all leave at once and collide.
PEOPLE = {
    "Person1": {"prim_path": "/World/male_adult_police_04",        "start_delay": 0},
    "Person2": {"prim_path": "/World/male_adult_construction_03",   "start_delay": 5},
    "Person3": {"prim_path": "/World/male_adult_construction_01_new", "start_delay": 10},
}

# Perimeter loop: exit through the gap near (30,20), turn right, go around
# the room's edge (X roughly 7 to 28), and back in. ADJUST THESE once you
# see it run - this is a first-draft guess based on your description.
WAYPOINTS = [
    (27.46, 16.33, 0),
    (29.06, 27.76, 0),
    (-25.60, 28.53, 0),
    (-28.41, -7.00, 0),
    (25.09, -8.53, 0),
    (26.03, 12.98, 0),
    (29.9, 13.4, 0),   # end at ExitCamera
]

SECONDS_PER_LEG = 4  # time to travel between each waypoint

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom, UsdPhysics
import time


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


def disable_physics_recursive(prim):
    """Walk this prim and all its children, disabling rigid body physics
    on anything that has it - so gravity/physics doesn't fight our manual
    position updates and cause it to fall through the floor."""
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(prim)
        attr = rigid_api.GetRigidBodyEnabledAttr()
        if attr:
            attr.Set(False)
        else:
            rigid_api.CreateRigidBodyEnabledAttr(False)
    for child in prim.GetChildren():
        disable_physics_recursive(child)


def main():
    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    movers = []
    for name, info in PEOPLE.items():
        prim = stage.GetPrimAtPath(info["prim_path"])
        if not prim.IsValid():
            print(f"WARNING: {name} prim not found at {info['prim_path']} - skipping")
            continue

        disable_physics_recursive(prim)

        start = get_translate(prim)
        full_path = [(start[0], start[1], start[2])] + WAYPOINTS
        movers.append({
            "name": name,
            "prim": prim,
            "path": full_path,
            "start_delay": info["start_delay"],
        })
        print(f"{name}: physics disabled, path set, starts moving at t={info['start_delay']}s")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    total_leg_time = SECONDS_PER_LEG
    max_delay = max(m["start_delay"] for m in movers) if movers else 0
    total_duration = max_delay + (len(WAYPOINTS) * SECONDS_PER_LEG) + 1

    start_time = time.time()
    while True:
        elapsed = time.time() - start_time

        for m in movers:
            personal_elapsed = elapsed - m["start_delay"]
            if personal_elapsed < 0:
                continue  # hasn't started yet

            leg_index = int(personal_elapsed // total_leg_time)
            leg_t = (personal_elapsed % total_leg_time) / total_leg_time

            path = m["path"]
            if leg_index >= len(path) - 1:
                set_translate(m["prim"], path[-1])
                continue

            p1 = path[leg_index]
            p2 = path[leg_index + 1]
            new_pos = (
                p1[0] + (p2[0] - p1[0]) * leg_t,
                p1[1] + (p2[1] - p1[1]) * leg_t,
                p1[2] + (p2[2] - p1[2]) * leg_t,
            )
            set_translate(m["prim"], new_pos)

        simulation_app.update()

        if elapsed >= total_duration:
            break

    print("Patrol loop complete.")
    timeline.stop()


if __name__ == "__main__":
    main()
    simulation_app.close()