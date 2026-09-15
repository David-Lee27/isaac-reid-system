r"""
diagnose_cam3_cam4_mismatch.py

Places a bright, unmistakable magenta marker at each zone's own assigned
ZONE_CENTERS coordinate (one at a time), captures a frame from ALL FOUR
real cameras (/World/Camera1-4) for each placement, and automatically
scores each frame for magenta-pixel presence - this directly answers
"which physical camera prim actually sees the physical area assigned to
which zone name" empirically, instead of guessing whether it's a
camera_zones.json swap or an orientation/FOV mismatch.

Also prints each camera's own world position + computed forward-direction
vector as a second, independent check.

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_cam3_cam4_mismatch.py

Output: a plain summary table printed to console, plus labeled frames in
C:\isaacsim\projects\surveillance-proj\debug_captures\cam_mismatch\ so you
can also eyeball them directly if you want to double check.
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, Gf
import cv2
import os

STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\optimized room.usd"  # literal SPACE, not underscore
PROJECT_DIR = r"C:\isaacsim\projects\surveillance-proj"
OUT_DIR = os.path.join(PROJECT_DIR, "debug_captures", "cam_mismatch")

CAMERAS = {
    "/World/Camera1": "RoomA_Cam1",
    "/World/Camera2": "RoomA_Cam2",
    "/World/Camera3": "RoomB_Cam3",
    "/World/Camera4": "RoomB_Cam4",
}

ZONE_CENTERS = {
    "RoomA_Cam1": (22.60, 27.36),
    "RoomA_Cam2": (39.21, 9.00),
    "RoomB_Cam3": (21.64, 8.91),
    "RoomB_Cam4": (5.83, 27.40),
}

MARKER_HEIGHT = 1.2  # roughly chest height, similar to where a person's torso would sit in frame
MARKER_PATH = "/World/DiagMarker"
MARKER_COLOR = (1.0, 0.0, 1.0)  # bright magenta - shouldn't occur naturally anywhere else in this scene


def create_marker(stage):
    cube = UsdGeom.Cube.Define(stage, MARKER_PATH)
    cube.CreateSizeAttr(0.6)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp()
    cube.CreateDisplayColorAttr([Gf.Vec3f(*MARKER_COLOR)])
    return cube.GetPrim()


def move_marker(prim, x, y, z):
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(x, y, z))
            return
    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))


def get_world_pos(prim):
    xform = UsdGeom.Xformable(prim)
    m = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return m.ExtractTranslation()


def get_forward_vector(prim):
    """Camera's local forward is -Z. Transform both the origin and a point
    at local (0,0,-1) into world space and take the difference, so this
    reflects the prim's real authored rotation regardless of how its
    xformOps are structured."""
    xform = UsdGeom.Xformable(prim)
    m = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    origin = m.Transform(Gf.Vec3d(0, 0, 0))
    tip = m.Transform(Gf.Vec3d(0, 0, -1))
    fwd = tip - origin
    fwd.Normalize()
    return fwd


def count_magenta_pixels(bgr):
    b = bgr[:, :, 0].astype(int)
    g = bgr[:, :, 1].astype(int)
    r = bgr[:, :, 2].astype(int)
    mask = (r > 150) & (b > 150) & (g < 100)
    return int(mask.sum())


def capture(camera_obj):
    rgba = None
    for _ in range(40):
        simulation_app.update()
        rgba = camera_obj.get_rgba()
        if rgba is not None and rgba.size > 0:
            break
    if rgba is None or rgba.size == 0:
        return None
    rgb = rgba[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    ctx = omni.usd.get_context()
    ctx.open_stage(STAGE_PATH)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(20):
        simulation_app.update()

    from isaacsim.sensors.camera import Camera

    print("=" * 70)
    print("STEP 1: real camera positions + computed forward vectors")
    print("=" * 70)
    cam_objs = {}
    for cam_path in CAMERAS:
        prim = stage.GetPrimAtPath(cam_path)
        if not prim.IsValid():
            print(f"  [WARN] {cam_path} not found in stage!")
            continue
        pos = get_world_pos(prim)
        fwd = get_forward_vector(prim)
        look_at = (pos[0] + fwd[0] * 5, pos[1] + fwd[1] * 5)
        print(f"  {cam_path} ({CAMERAS[cam_path]}): pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
              f"forward=({fwd[0]:.2f},{fwd[1]:.2f},{fwd[2]:.2f}) -> looking roughly toward "
              f"({look_at[0]:.2f},{look_at[1]:.2f})")
        cam = Camera(prim_path=cam_path, resolution=(640, 480))
        cam.initialize()
        cam.set_horizontal_aperture(2.0955)
        cam.set_vertical_aperture(2.0955 * 480 / 640)
        cam_objs[cam_path] = cam

    for _ in range(10):
        simulation_app.update()

    print()
    print("=" * 70)
    print("STEP 2: placing a magenta marker at each zone's own ZONE_CENTERS "
          "coordinate, capturing all 4 cameras each time")
    print("=" * 70)

    marker_prim = create_marker(stage)
    results = {}  # zone_name -> {cam_path: magenta_pixel_count}

    for zone_name, (zx, zy) in ZONE_CENTERS.items():
        move_marker(marker_prim, zx, zy, MARKER_HEIGHT)
        for _ in range(10):
            simulation_app.update()

        print(f"\n  Marker placed at {zone_name}'s coordinate ({zx}, {zy}):")
        results[zone_name] = {}
        for cam_path, cam in cam_objs.items():
            bgr = capture(cam)
            if bgr is None:
                print(f"    {cam_path}: capture FAILED")
                results[zone_name][cam_path] = -1
                continue
            magenta_count = count_magenta_pixels(bgr)
            results[zone_name][cam_path] = magenta_count
            tag = zone_name.replace(" ", "_")
            cam_tag = cam_path.replace("/World/", "")
            fname = f"marker_at_{tag}__seen_by_{cam_tag}.jpg"
            cv2.imwrite(os.path.join(OUT_DIR, fname), bgr)
            visible = "VISIBLE" if magenta_count > 50 else "not visible"
            print(f"    {cam_path}: magenta pixels={magenta_count} -> {visible}")

    print()
    print("=" * 70)
    print("SUMMARY: which real camera actually sees each zone's own marker position")
    print("=" * 70)
    for zone_name, cam_results in results.items():
        best_cam = max(cam_results, key=lambda c: cam_results[c])
        assigned_cam = [c for c, z in CAMERAS.items() if z == zone_name][0]
        match = "OK (matches camera_zones.json)" if best_cam == assigned_cam else "*** MISMATCH ***"
        print(f"  {zone_name} (assigned to {assigned_cam}) -> actually best seen by {best_cam}  [{match}]")

    print(f"\nLabeled frames saved to: {OUT_DIR}")
    print("If SUMMARY shows a mismatch, camera_zones.json's prim->zone mapping")
    print("needs correcting to match reality (swap the affected entries) - do")
    print("NOT touch ZONE_CENTERS, this only tests the camera_zones.json mapping.")


if __name__ == "__main__":
    main()
    simulation_app.close()
