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

**Headless launch has a different extension/env setup than the GUI**
Two separate bugs discovered when moving from the interactive GUI to a standalone
headless script (`headless_slam_session.py`) for better performance:
1. Manually toggling `isaacsim.ros2.bridge` on in the GUI's Extensions window only
   applies to that running session - it does NOT carry over to a fresh headless
   script launch, which boots its own process with default extension state.
   `/odom` and `/cmd_vel` were completely absent from `ros2 topic list` as a result.
   Fixed by explicitly enabling it in code before opening the stage:
   `omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)`.
2. Even after that, the bridge printed `"ROS2 Bridge startup failed"` and shut
   itself back down immediately. Cause: the bridge needs its OWN internal env vars
   set (pointing at its bundled ROS2 "humble" libraries - unrelated to/different
   from our external WSL Jazzy install) - `ROS_DISTRO`, `RMW_IMPLEMENTATION`, and a
   `PATH` addition. Launching via `isaac-sim.bat` (the GUI) sets these automatically;
   launching via `python.bat` directly does not. Fixed by setting them in Python
   BEFORE `from isaacsim import SimulationApp`:
   ```python
   os.environ["ROS_DISTRO"] = "humble"
   os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
   os.environ["PATH"] += ";c:/isaacsim/exts/isaacsim.ros2.core/humble/lib"
   ```
   Both fixes together got the full topic set (`/odom`, `/cmd_vel`, both lidars,
   cameras) publishing correctly in headless mode.

**Headless rendering vs GUI FPS**
GUI viewport rendering with 5+ cameras + 2 lidars + physics was the real FPS
bottleneck (down to 0-18 FPS even after switching to RTX-Minimal renderer mode +
lowering settings). True headless (`{"headless": True}`) skips rendering entirely -
lidar rate improved from ~5Hz to ~8.5-9Hz. Still not perfectly smooth (some jitter),
but workable. Note headless mode required the two extension/env fixes above to work
at all - without them it LOOKED like it was running (topics existed) but had zero
real data flowing.

**`slam_toolbox` default `scan_topic` mismatch**
Default config listens on `/scan`, but this robot's lidar publishes to
`/lidar_front/scan` - completely different topic names, so slam_toolbox was
silently receiving zero scan messages the entire time (confirmed via
`ros2 param get /slam_toolbox scan_topic` returning `/scan`, and `/map`/`map->odom`
tf never appearing despite real odom and lidar data both flowing). Fixed by copying
the default `mapper_params_online_async.yaml`, editing `scan_topic` to
`/lidar_front/scan`, and launching with `slam_params_file:=<edited copy>`.

**`/cmd_vel` has no command timeout - Ctrl+C does NOT stop the robot**
Assumed that killing the `ros2 topic pub` process would stop the robot (like
releasing a key). It does not - the diff-drive controller keeps executing the
LAST received Twist message indefinitely, with no automatic zero-velocity
timeout. Confirmed by checking odometry minutes after "stopping": position was
still climbing at exactly the last commanded velocity. This most likely
contributed to an earlier SLAM divergence disaster (translation of -16,927m,
physically impossible for the actual test duration) - the robot was probably
still driving, uncontrolled, well past when we thought each test had ended.
Real fix: always send an explicit zero-velocity message after driving
(`--once` flag with all-zero linear/angular values), and verify it actually
stopped by checking odometry twice a few seconds apart before trusting it's stationary.

**SLAM map/odom transform meaning - what's actually the "did it diverge" signal**
`map -> odom` tf is SLAM's own internal correction/localization offset, NOT the
robot's position - a small, stable value (e.g. -0.01, -0.1, -0.2) is the HEALTHY
result, meaning SLAM's belief is staying consistent. The robot's actual position is
`odom -> base_footprint`. Checked the wrong one early on and mistook a healthy small
correction offset for "is this still diverged," causing confusion.

**Broken TF frames prevented clean SLAM mapping (root cause never fully solved)**
Even after fixing the scan_topic mismatch and the stop-command bug, a real driving
test produced a garbage "starburst" map (scattered radiating lidar points instead of
clean walls) despite the robot's own odometry staying sane throughout. Isaac Sim's
console was simultaneously spamming `[PoseTree] parent/target getObjectType eInvalid`
for `base_link`, `lidar_merged`, and camera sensor frames - meaning the TF publisher
couldn't resolve several of the robot's own sensor frame prims, so individual lidar
scans were being placed at wrong orientations relative to the robot even though the
overall robot position was fine. Never root-caused (would require more prim-hierarchy
debugging in the robot asset, similar to the camera_front duplicate issue). Given
time constraints, worked around entirely rather than fixed - see next entry.

**Hand-built map instead of fixing SLAM's broken sensor TF**
Since Nav2 only needs a valid map.yaml/.pgm pair describing the room and doesn't
care how it was produced, wrote a small standalone Python script
(`build_map.py`, WSL, stdlib only) that directly draws a clean occupancy grid: solid
walls at the room's known real-world bounds (from earlier camera-position/waypoint
data) plus two blocked squares for the T-obstacle. Verified visually by converting
.pgm -> .png (`pnmtopng`) and opening via `explorer.exe \\wsl$\...` from Windows,
since RViz2 would not open in this environment (WSLg GUI issue, not investigated -
not needed once we stopped relying on it for verification). This fully replaced the
need to fix the SLAM/TF issue above.

**Nav2 without AMCL - static map->odom transform instead**
Since AMCL (Nav2's normal live localization) would hit the exact same broken-TF
sensor issue as SLAM, skipped it in favor of a permanently fixed transform:
`ros2 run tf2_ros static_transform_publisher <x> <y> 0 0 0 0 map odom`, using the
robot's known real starting position. This tells Nav2 exactly where odom's origin
sits within the hand-built map, with no drift-correction, which is fine since we
confirmed raw odometry itself is accurate and reliable (the earlier 33m straight-line
drive test tracked correctly).

**Nav2 launch argument syntax**
`ros2 launch nav2_bringup bringup_launch.py ... slam:=false` threw
`name 'false' is not defined` regardless of quoting attempts. Root cause never fully
isolated, but avoided entirely by checking `--show-args` first - the defaults
(`slam: False`, `use_sim_time: false`) already matched what we wanted, so the
argument didn't need to be passed at all. Just running
`ros2 launch nav2_bringup bringup_launch.py map:=<path>` launched cleanly with every
node (map_server, controller_server, planner_server, costmaps, bt_navigator,
waypoint_follower, behavior_server, collision_monitor, docking_server) configuring
and activating successfully - a genuinely clean full Nav2 bringup.

**Nav2 goals accepted and planned, but robot doesn't move - isolated to `velocity_smoother`**
Sent a real `NavigateToPose` goal via `ros2 action send_goal`; Nav2 accepted it and
reported a correct `distance_remaining`, but the robot's actual odometry position
never changed. Diagnosed layer by layer:
- `/cmd_vel_nav` (controller_server's raw output): publishing at ~20-25Hz, confirmed
  real - so planning + control computation is genuinely working.
- `/odom`: healthy 30Hz, confirmed not the blocker.
- `/cmd_vel` (post-smoother, what Isaac Sim's robot actually subscribes to):
  ZERO messages, checked for a full minute.
- Checked `velocity_smoother`'s config directly (`ros2 param get ...`) for common
  causes - `enable_stamped_cmd_vel` was False (ruled out a Twist-vs-TwistStamped
  type mismatch theory), other params looked normal. Root cause not found before
  running out of session time - suspected connection to the same DDS message-timing
  irregularities seen in Isaac Sim's "sequence size exceeds remaining buffer" console
  spam (a `/cmd_vel_nav` rate check showed an abnormal negative time delta,
  `min: -2.430s`, suggesting message reordering/corruption crossing the WSL/Windows
  bridge for this topic specifically).

Planned workaround for next session (not yet tested): bypass the smoother entirely
using `ros2 run topic_tools relay /cmd_vel_nav /cmd_vel` - forwards controller
output directly to the topic Isaac Sim listens to. This is a legitimate, common
Nav2 deployment pattern (skipping the smoother is supported), not just a hack.

## Current status / how to resume

**What's fully working:** core detection/re-ID/zones/checkpoint/authorization/fall/
loitering/appearance-fallback pipeline (`unified_tracking.py`) - proven stable across
many runs. Robot moves correctly via kinematic teleport for the existing
dispatch-alert system (fall/loitering/unauthorized-presence -> robot drives to
coords -> arrival scan). WSL2<->Windows ROS2 bridge fully working, including
headless. Nav2 fully launches and accepts/plans real navigation goals against a
hand-built map. Real `/cmd_vel_nav` velocity commands are being generated correctly
by Nav2's controller.

**What's NOT yet confirmed:** whether the robot actually drives in response to a
real Nav2 goal - blocked on `velocity_smoother` not passing `/cmd_vel_nav` through
to `/cmd_vel`. Next step is the `topic_tools relay` bypass above.

**Every-session setup checklist (order matters):**
1. Windows PowerShell: `. C:\isaacsim\projects\surveillance-proj\set_ros_env.ps1`
   (auto-detects WSL's current IP - was previously manual/hardcoded, now automated)
2. `C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\headless_slam_session.py`
   - wait for "Headless SLAM session running"
3. WSL: `ros2 run tf2_ros static_transform_publisher -13.84 5.06 0 0 0 0 map odom`
   (leave running in its own terminal)
4. WSL: `ros2 launch nav2_bringup bringup_launch.py map:=/home/popli/warehouse_map.yaml`
5. (next step, untested) WSL: `ros2 run topic_tools relay /cmd_vel_nav /cmd_vel`
6. Send a goal: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}}"`
7. Verify with `ros2 run tf2_ros tf2_echo odom base_footprint` - position should
   actually change this time if the relay fixes it.

**Key files added this phase:** `set_ros_env.ps1` (auto WSL-IP env setup),
`headless_slam_session.py` (headless Isaac Sim + ROS2 bridge), `robot_test.py` /
`robot_camera_test.py` / `robot_combined_test.py` / `robot_kinematic_test.py`
(isolated robot movement/camera debugging, non-freezing since no camera scan
subprocess calls), `find_physics_scenes.py` (stage diagnostic), `build_map.py`
(hand-built occupancy grid, WSL-side), `~/mapper_params.yaml` (WSL, slam_toolbox
config with corrected scan_topic - not in the Windows project folder).

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

## CONFIRMED WORKING: Nav2 velocity_smoother bypass

The `topic_tools relay /cmd_vel_nav /cmd_vel` workaround is confirmed fixed and
working end-to-end. Root cause of the goal rejection on the first retest attempt
was separate and simpler: the `static_transform_publisher` (map->odom) terminal
got killed (Ctrl+C) before Nav2 launched, so the `map` frame didn't exist and the
costmap/goal validation failed immediately ("Timed out waiting for transform from
base_link to map... frame does not exist", "Goal was rejected"). Re-ran with the
static transform publisher kept alive in its own terminal for the whole session,
verified with `ros2 run tf2_ros tf2_echo map odom` before launching Nav2 - after
that, the goal was accepted and `tf2_echo odom base_footprint` showed real,
steadily increasing translation (e.g. X: 1.859 -> 2.046 -> 2.234 -> 2.429 -> 2.616
across successive timestamps), confirming the robot is genuinely driving under real
Nav2 planner/controller output, not teleporting.

**Nav2 integration is now fully functional end-to-end**: goal sent -> planned ->
controller computes `/cmd_vel_nav` -> relay forwards to `/cmd_vel` -> Isaac Sim robot
physically drives toward the goal. This closes out the ROS2 Nav2 integration work.

## Current status / how to resume (UPDATED)

**What's fully working:** core detection/re-ID/zones/checkpoint/authorization/fall/
loitering/appearance-fallback pipeline (`unified_tracking.py`). Robot moves correctly
via kinematic teleport for the dispatch-alert system. WSL2<->Windows ROS2 bridge
fully working, including headless. **Nav2 navigation is now fully working**,
including real physics-driven movement via the velocity_smoother relay bypass -
verified with actual odometry translation change under a live goal.

**Next steps (not yet started):**
1. Decide whether to wire real Nav2 goal-sending into the existing dispatch-alert
   system (currently uses kinematic teleport) - replacing teleport with a real
   `NavigateToPose` action call when the robot is dispatched to fall/loitering/
   unauthorized-presence coordinates would make the robot's response fully physics-
   and-navigation-driven end-to-end, which is a stronger portfolio story than teleport.
2. Benchmark re-ID metrics (accuracy/latency) for portfolio documentation.
3. Record demo video.
4. Polish GitHub repo + README with clear documentation of engineering decisions,
   using this file as the source log.

**Every-session Nav2 setup checklist (order matters, confirmed working):**
1. Windows PowerShell: `. C:\isaacsim\projects\surveillance-proj\set_ros_env.ps1`
2. `C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\headless_slam_session.py`
   - wait for "Headless SLAM session running"
3. WSL (own terminal, KEEP RUNNING, do not Ctrl+C):
   `ros2 run tf2_ros static_transform_publisher -13.84 5.06 0 0 0 0 map odom`
4. Verify: `ros2 run tf2_ros tf2_echo map odom` shows repeated output before proceeding
5. WSL: `ros2 launch nav2_bringup bringup_launch.py map:=/home/popli/warehouse_map.yaml`
   - wait for full activation, no more "Timed out waiting for transform" spam
6. WSL: `ros2 run topic_tools relay /cmd_vel_nav /cmd_vel` (own terminal, keep running)
7. Send goal: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}}"`
8. Verify: `ros2 run tf2_ros tf2_echo odom base_footprint` - translation should
   steadily change over time.
