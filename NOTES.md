# Development Notes / Debugging Log

Running log of what was built and, more importantly, what broke and how it got fixed.
Kept for writing the final README and project writeup later.

## Architecture

- **4 fixed zone cameras** (corners of warehouse) - occupancy/body detection only via YOLOv8.
  Best-effort face re-ID if a clean face happens to be visible, but not required.
- **1 checkpoint camera** (EntryCamera, close range, head height) - source of truth for
  identity. Runs YOLO -> crop -> DeepFace re-ID -> authorization check.
- **3 patrolling characters**, moved via direct USD transform overrides (no physics,
  no walk animation yet) along a manually-plotted waypoint loop.
- Person detection (YOLOv8) and face re-ID (DeepFace) both run in isolated subprocesses
  via `python.bat`, not in-process inside Isaac Sim - see "h5py/DLL conflict" below for why.

## Key bugs and fixes

**Isaac Sim 6.x namespace migration** - `isaacsim.*` not `omni.isaac.*`. Older
tutorials/docs reference the old namespace and will silently fail to import.

**Camera returns no frame / `get_rgba()` is None**
- Root cause 1: timeline was never started. `simulation_app.update()` alone advances
  the app loop but does NOT drive the renderer if the timeline is paused - need
  `omni.timeline.get_timeline_interface().play()` before capturing.
- Root cause 2: camera resolution defaulted to 64x64 in one case, below DLSS's minimum
  input resolution, so no frame could be produced at all. Fixed by explicitly setting
  `resolution=(640,480)` (or higher) on the Camera object.
- Fixed with a retry loop (check `get_rgba()` after every tick, up to ~300 attempts)
  rather than a fixed sleep/tick count, since different scenes need different warmup.

**NumPy 2.x / PyTorch ABI mismatch**
`pip install ultralytics` pulled a torch build compiled against NumPy 1.x, but Isaac
Sim's environment has NumPy 2.5.1. Fixed by reinstalling torch (and torchvision -
they must match versions) via the CPU wheel index.

**h5py DLL conflict (DeepFace/.h5 weights)**
Isaac Sim's embedded Python has a persistent `ImportError: DLL load failed while
importing defs` when loading .h5 weight files in-process - happens even with correct,
verified weight files, and does NOT reproduce in a plain terminal python.bat call.
This means it's an in-process native library conflict specific to Isaac Sim's runtime,
not a package/version issue. Pinning h5py/numpy versions did not fix it even after a
full restart.

Architectural fix (not a patch): capture the frame inside Isaac Sim, save to disk,
then shell out via `subprocess.run(...)` calling `python.bat script.py` as a
completely separate process. This sidesteps the conflict entirely rather than
continuing to chase the root cause - and arguably is the more correct design anyway,
since a real deployment would likely separate CV inference from the simulation
process regardless.

**Subprocess quirks on Windows**
- Inline code via `python -c "..."` gets mangled through `shell=True` - must write
  the script to an actual .py file on disk first.
- Default `text=True` subprocess output decodes with the Windows console's default
  codepage (cp1252), which crashes on any non-ASCII byte in stderr (e.g. emoji in a
  library's warning output). Fixed with `encoding="utf-8", errors="replace"`.
- Writing to `C:\` root fails with permission denied - use `os.environ["TEMP"]`.

**DeepFace `enforce_detection=False` silently returning garbage**
Using `enforce_detection=False` means DeepFace never raises an error when it can't
find a face - it just proceeds anyway, which caused spurious "matches" based on
whatever it fell back to (at one point, apparently comparing warehouse background
similarity rather than faces, since 4 different people all matched to one ID).
Fixed by using `enforce_detection=True` and explicitly catching the ValueError,
with a three-way `status` field (`match` / `new` / `detection_failed`) instead of
a boolean, so a failed detection is never conflated with "confirmed new person."

**Wide/elevated camera angles can't do reliable face-ID**
Zone cameras mounted high and covering a whole room physically can't get a clean,
close, front-on face shot - this isn't a bug, it's a real physical limitation
(this is why real security systems use dedicated checkpoint cameras at chokepoints,
not wide overview cameras, for identity confirmation). Solved by splitting roles:
zone cameras = occupancy/body detection only, one dedicated close-range checkpoint
camera = identity confirmation.

**Stale DeepFace `.find()` cache**
`DeepFace.find()` builds and reuses a pickle cache of known face embeddings in the
db folder. Since new reference images are being added to that same folder between
calls, the cache went stale and caused real matches to be silently missed (same
person re-scanned 5 times in a row, each time treated as brand new). Fixed by
deleting any `.pkl` cache files in the known_faces folder at the start of every
`deepface_reid.py` run, forcing a fresh index each time.

**OpenCV face detector struggling with sunglasses/hats**
Switched `detector_backend` from `"opencv"` (fast, weak, classic Haar-cascade) to
`"retinaface"` (modern deep-learning based) after confirming character accessories
were causing missed detections.

**`get_next_id()` off-by-one**
Original version wrote the new counter value to disk but returned the same value
that was passed in, causing duplicate IDs to be assigned. Fixed to always compute
and write `next_id + 1` while returning the pre-increment value.

**Fall detection heuristic**
No separate pose-estimation model - reused the existing YOLO bounding box.
A standing person's box is taller than wide; a fallen person's box becomes wider
than tall. Threshold: aspect ratio (width/height) > 1.3 flags a fall. Confirmed
working by manually rotating a character prim 90 degrees - aspect ratio correctly
jumped from ~0.3-1.1 (standing) to 1.77 (fallen) and triggered the alert.

**Isaac Sim UI freezes ("Not Responding") during scans**
`subprocess.run()` is a blocking call - while waiting for YOLO/DeepFace subprocesses,
the main thread can't service Windows UI messages, so the window shows "Not
Responding." This is expected, not a crash - the process is just busy. Confirmed via
Task Manager CPU/disk activity during a "frozen" period. Real cost: can't visually
inspect the viewport while a sweep is running, since it blocks the whole app,
including camera switching.

**Robot articulation physics vs. simple rigid bodies (human characters)**
Human characters are simple independent rigid bodies - `disable_physics_recursive()`
(setting `rigidBodyEnabled=False` on each) works fine and stops gravity entirely,
letting them be moved purely by direct transform overrides. The robot is a full
PhysX **articulation** (base + wheels + arm, all linked by joints) - PhysX explicitly
refuses `rigidBodyEnabled=False` on articulation members ("not supported if the rigid
body is part of an articulation"), so that same approach silently failed on every
link, gravity kept acting on it normally, and it fell through the floor while our
script's transform overrides fought a losing battle against live physics each frame.

Real fix: target the actual physics-driven prim directly - `base_footprint`, not the
outer `/World/mobile_manipulator_ros` group Xform (moving the parent group did
nothing, since PhysX drives the child's world transform independently) - and set
it **kinematic** (`kinematicEnabled=True`) rather than trying to disable it entirely.
Kinematic is the officially-supported way to let a script drive a physics body's
transform while collision still respects it. Critically, the kinematic flag must be
set **before** `timeline.play()` - setting it after physics has already started
stepping did not reliably take effect (confirmed by direct A/B test: same code,
different order, different result).

**Sibling prims that don't move with their parent**
`camera_front`, `camera_hand_link`, `wheel_left`, `wheel_right` looked like they
should be attached to `base_footprint`, but the asset actually has them as direct
**siblings** of `base_footprint` under the top-level robot group, not children of it.
Moving `base_footprint` alone left them behind. Fixed by manually moving each of
them in lockstep with the base every frame, preserving their original relative
offset - not a physics fix, just accounting for the asset's actual prim hierarchy
rather than assuming a logical grouping that wasn't really there.

**Floor and wheels had no collision at all**
Discovered only once real physics-driven control (ROS2 `/cmd_vel`, for Nav2) was
tested for the first time - the human characters' physics was always fully disabled,
so they never actually exercised real floor collision, and the robot's earlier
fall-through-floor issue was masked by the kinematic fix, which doesn't need working
collision to "work" for teleport-style movement. Running the robot with genuine
physics (no kinematic override, no scripted position) immediately free-fell through
the warehouse floor. Root cause: the warehouse floor mesh had no Collision API at
all, and the robot's wheels had `RigidBodyAPI` but no actual collision shapes either
(only `base_footprint` had real collision). Fixed by adding a Collider Preset to the
floor mesh (collision-only, not Rigid Body - floors must stay static) and a Collider
Preset to each wheel (collision-only, since they already had RigidBodyAPI - adding
another would have duplicated it).

**WSL2 <-> Windows ROS2 bridging (Isaac Sim on Windows, ROS2 in WSL2 Ubuntu)**
Isaac Sim runs natively on Windows; ROS2 (Jazzy, matching Ubuntu 24.04) runs inside
WSL2 - two separate network environments that don't share multicast-based DDS
discovery by default. Tried WSL2's "mirrored" networking mode first (`.wslconfig`
with `networkingMode=mirrored`) since it's supposed to make this seamless - instead
it broke the ROS2 daemon's ability to reach even itself over localhost (connection
timeouts on `ros2 daemon start`/`stop`). Confirmed this was the actual cause via A/B
test: reverting to default NAT networking immediately fixed the daemon.

Working fix instead: keep NAT mode, and explicitly configure DDS discovery instead of
relying on automatic multicast crossing the NAT boundary:
- On both the WSL side and the Windows side, set
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` and `ROS_STATIC_PEERS=<other side's IP>`
  (WSL's IP via `hostname -I`, Windows' IP-as-seen-from-WSL via
  `ip route show | grep default`, which is the NAT gateway address)
- Added a Windows Firewall inbound rule allowing UDP 7400-7500 (standard DDS
  discovery/traffic port range) from WSL's IP specifically
- Confirmed working end-to-end: enabled Isaac Sim's `isaacsim.ros2.bridge` extension,
  and `ros2 topic list` from WSL successfully showed live topics from Isaac Sim
  (`/odom`, `/tf`, `/cmd_vel`, camera and lidar topics), confirmed with
  `ros2 topic echo /odom --once` returning real (if physically falling, pre-collision-fix)
  data.

Note WSL's NAT IP can change across reboots - the static peer IPs may need updating
if discovery stops working after a restart.

**Duplicate camera_front prims - one real, one orphaned**
Discovered while testing real `/cmd_vel` physics-driven movement (no kinematic
override) for the first time: the camera visibly floated, disconnected from the
body. Turned out there are TWO `camera_front` sensor rigs in the asset - a properly
nested one at `base_footprint/sensors/camera_front/...` (a genuine USD child of
`base_footprint`, moves automatically, no hack needed) and a disconnected leftover
copy directly under `/World/mobile_manipulator_ros/camera_front` (a loose sibling,
which is what we'd been using and needed the manual follower-offset hack for this
whole time). Switched `ROBOT_CAMERA_PATH` to the real nested one and removed it from
the follower list - no more manual positioning needed for the camera specifically.
