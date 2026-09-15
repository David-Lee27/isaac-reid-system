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

## IMPLEMENTED: Real Nav2 navigation wired into dispatch-alert system

Replaced the old kinematic-teleport robot movement in `unified_tracking.py`
with genuine physics-driven Nav2 navigation, for both patrol AND dispatch
(not just dispatch) - this was a bigger change than originally planned, but
necessary: the earlier finding that kinematic flags must be set BEFORE
`timeline.play()` to reliably take effect meant toggling kinematic on/off
at runtime per-dispatch (patrol=kinematic, dispatched=physics) was too
risky. So the robot now runs in real physics mode (kinematic OFF) for the
entire session, and ALL movement - patrol and dispatch alike - goes through
real Nav2 goals.

**New architecture:**
- `unified_tracking.py` (Windows/Isaac Sim side): sets the robot base to
  kinematic OFF once before `timeline.play()`, writes the patrol route to
  `robot_patrol_waypoints.json`, and no longer commands robot movement at
  all. `update_robot()` now just watches the robot's REAL position (driven
  by physics) and triggers the on-arrival checkpoint-style scan
  (`scan_robot()`) once it's within `ARRIVAL_RADIUS` (2.0m) of an unhandled
  dispatch_alert's coordinates.
- `dispatch_bridge.py` (NEW, WSL side): the actual navigation driver. Reads
  `robot_patrol_waypoints.json` and `event_log.json` directly off the
  Windows filesystem via `/mnt/c/...`. Default behavior cycles through
  patrol waypoints via `ros2 action send_goal` (blocking, synchronous CLI
  call - no rclpy dependency needed since the CLI approach is already
  confirmed working). Any unhandled `dispatch_alert` in the event log
  interrupts patrol and sends a goal to that alert's coordinates instead.

**Known limitation (documented, not solved):** `wheel_left`, `wheel_right`,
and `camera_hand_link` are sibling prims of `base_footprint`, not joined to
it by a physics joint. The old kinematic design faked attachment by
manually repositioning them every frame. In real-physics mode they may
visually lag behind during navigation - cosmetic only, does not affect
navigation or detection, since `ROBOT_CAMERA_PATH` is a genuine nested
child of `base_footprint` and moves correctly on its own.

**Not yet tested end-to-end** - this is freshly implemented, next step is
to actually run it (see updated checklist below) and confirm patrol +
dispatch + arrival-triggered scan all work together for real.

## Updated every-session checklist (supersedes the one above)

1. Windows PowerShell: `. C:\isaacsim\projects\surveillance-proj\set_ros_env.ps1`
2. WSL (own terminal, KEEP RUNNING, do not Ctrl+C):
   `ros2 run tf2_ros static_transform_publisher -13.84 5.06 0 0 0 0 map odom`
3. Verify: `ros2 run tf2_ros tf2_echo map odom` shows repeated output before proceeding
4. WSL: `ros2 launch nav2_bringup bringup_launch.py map:=/home/popli/warehouse_map.yaml`
   - wait for full activation, no more "Timed out waiting for transform" spam
5. WSL: `ros2 run topic_tools relay /cmd_vel_nav /cmd_vel` (own terminal, keep running)
6. Windows PowerShell: `C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\unified_tracking.py`
   - wait for "Robot found... set to real physics mode... Patrol waypoints written..."
7. WSL (own terminal, keep running): `python3 /mnt/c/isaacsim/projects/surveillance-proj/dispatch_bridge.py`
   - this is what actually drives the robot now - patrol starts automatically,
     dispatch alerts will interrupt it when fall/loitering/unauthorized-presence
     is detected
8. Watch for `[dispatch_bridge] sending goal (ALERT: ...)` in the
   `dispatch_bridge.py` terminal and the on-arrival scan output
   (`[ROBOT] arrived at...`) in the `unified_tracking.py` terminal.

## FIXED: "sequence size exceeds remaining buffer" / stalling sim / bad odom rate

First real test of the new Nav2-driven navigation surfaced a serious bug:
Nav2 goals all timed out after 60s, and `ros2 topic hz /odom` showed a
wildly inconsistent rate (avg ~0.7Hz, gaps up to 3.9s between messages) -
far too choppy for Nav2's costmap/planner to work with. Isaac Sim's console
was also spammed with "sequence size exceeds remaining buffer".

Root cause: `capture_frame()` created a brand NEW `Camera` object (and a
new underlying render product/annotator) from scratch on every single scan
call - and with 4 zone cameras + the checkpoint camera scanned every 8
seconds for the whole run, that's a fresh render product created (and
never cleaned up) roughly every 1.6s on average. Stale render
products/annotators piled up, overran an internal render buffer (hence the
error spam), and stalled the physics step rate badly enough to starve the
ROS2 bridge's odom publisher - which explains both symptoms at once.

**Fix:** cache one `Camera` object per camera path (`_camera_cache` dict in
`unified_tracking.py`) instead of recreating it every scan. Each camera is
now only ever initialized once, on its first use.

**Not yet re-tested** - next step is rerunning the full stack and
confirming (a) no more buffer-exceeded spam, (b) `/odom` publishes at a
steady rate, and (c) Nav2 goals actually complete instead of timing out.

## FIXED: robot arrival never detected (get_translate always returned 0,0,0)

After the odom-rate and goal-timeout fixes above, Nav2 goals were
confirmed completing ("goal finished" in dispatch_bridge.py), but
unified_tracking.py's console never printed `[ROBOT] arrived at...` no
matter how long a goal ran or how many alerts fired.

Root cause: `get_translate()` only ever looked for a literal `translate`
xformOp on the prim, falling back to `(0,0,0)` if none was found. That was
fine while the robot was kinematic-teleported, since our own code created
that exact op. But with kinematic OFF and PhysX driving the robot for
real, position updates get written through a different xform
representation (a combined transform matrix), not a separate translate op
- so `get_translate()` was silently returning `(0,0,0)` for the robot on
every call, meaning `update_robot()`'s distance-to-alert check was always
comparing against the origin instead of the robot's real position, and
never triggering arrival no matter how close it actually got.

**Fix:** `get_translate()` now computes the prim's full local-to-world
transform (`ComputeLocalToWorldTransform`) and extracts translation from
that, which works correctly regardless of how the underlying xformOps are
structured. Also fixes the same class of bug for people/movers, though
they were unaffected in practice since they use explicit translate ops.

**Not yet re-tested.**

## CONFIRMED WORKING: clean run, log-channel schema fix resolved everything

Re-ran after the flat-string channel fix - confirmed clean: no
`getStringRawInternal` errors, no PoseTree spam, no buffer-overrun spam.
Only minor one-time-per-camera notices remain (`camera_info_utils`
aperture-forcing messages), not loops - not worth further chasing.

**Step 1 (real Nav2-driven navigation replacing kinematic teleport) is now
considered DONE.** Full session summary of what's actually working:
- Real physics-driven robot movement (kinematic off), no more teleport
- `dispatch_bridge.py` (WSL) drives both patrol and dispatch via real
  Nav2 `NavigateToPose` goals, with goal-standoff so it never targets a
  point on top of a wall
- Odometry, costmap obstacle sources (real lidar topics, not the
  nonexistent stock `/scan`), `use_sim_time`, and physics-starvation
  (Popen-polling instead of blocking subprocess calls) are all fixed
- Log output is clean, render settings lightened, no more freezing

**Known remaining rough edges (not blocking, documented for later):**
- `[PoseTree] eInvalid` root cause never actually fixed (only suppressed at
  the log level) - `repair_broken_tf_publisher_targets()` consistently
  finds 0 repairable relationships, meaning the broken references aren't
  stored as plain USD relationships the way it checks for. Not investigated
  further since it turned out to be unrelated to both the wall-collision
  (that was goal placement) and the physics-starvation issues (that was
  blocking subprocess calls) - it may be entirely cosmetic.
- `camera_front` sibling mesh doesn't visually follow the robot (known,
  cosmetic, doesn't affect the real nested camera used for detection).

**Next steps (from the original step 1 follow-up list):**
1. Benchmark re-ID metrics (accuracy/latency) for portfolio documentation.
2. Record demo video.
3. Polish GitHub repo + README.

## Character animation research + implementation (fall + walk-facing)

User correctly called out that people never fall over and never move
realistically (sliding sideways, no animation), and that the robot's scan
doesn't reflect real scenarios. Investigated properly before writing code -
ran a read-only diagnostic (`diagnose_characters.py`) against the actual
character USD prims instead of guessing.

**Findings:** all three People characters (from NVIDIA's official Isaac Sim
6.0 People asset pack, loaded from
`.../Isaac/6.0/Isaac/People/Characters/.../*.usd`) have a real, correctly-
rigged 101-joint UsdSkel skeleton, but ZERO animation bound
(`animationSource targets: []`, no SkelAnimation prims anywhere). Not
broken - just never had a walk clip attached.

Researched whether to attach NVIDIA's real walk-cycle clips (from the
sibling `.../Isaac/People/Animations/` folder). Found multiple independent,
version-spanning (4.1 through 5.1) reports of this exact approach being
broken specifically when driven via standalone Python scripts (not the
interactive GUI): payloads resetting to origin, characters stuck in T-pose,
someone hitting our exact scenario ("has not worked... not able to get the
animations to work when manually placing the models and animations").
NVIDIA's own extension for this (`omni.anim.people`) is being deprecated in
favor of a heavier synthetic-data-generation-oriented system
(`isaacsim.replicator.agent`) that multiple users describe as "overkill"/
"confusing" for simple walking. Given this, attempting real skeletal
animation binding tonight was judged too high-risk for the time available -
documented as a known future improvement rather than attempted blind.

**Implemented instead (both real, working, verified-in-code changes):**

1. **Scripted fall event** (`FALL_ENABLED`/`FALL_START_RANGE` in
   `unified_tracking.py`): one random person, at a random time (15-55s into
   the run), physically collapses - rotated 90 degrees about a random
   horizontal axis via a new `set_rotation()` helper, frozen in place for
   `FALL_PAUSE_DURATION` (45s), then automatically stands back up (rotation
   reset to identity). This is a REAL event now, not just a heuristic that
   occasionally misfires - `check_for_fall()`'s aspect-ratio detection
   (unchanged) now has something genuine to actually detect when a camera
   happens to see the collapsed character, closing the loop the user asked
   for ("so the scan can scan real scenarios").

2. **Face-direction-of-travel + step bob** (in the patrol movement block of
   `update_person_position()`): characters now rotate to face their actual
   direction of travel each leg (NVIDIA People characters use -Y as forward
   per NVIDIA's docs, so yaw = atan2(dx, -dy)) instead of sliding sideways/
   backwards, plus a small sine-wave vertical bob synced to a walk-cadence
   frequency for a bit of visual life. NOT a real walk cycle - documented
   plainly as a stopgap, not a finished feature - but a real, guaranteed-to-
   work improvement over pure unoriented sliding.

**Not yet re-tested - next step is running a full session and confirming
(a) the fall looks right and gets detected by a real camera sweep, (b) the
facing direction is correct (NVIDIA's -Y-forward convention was applied
from their docs, not verified live - may need a 90/180-degree correction
if it looks off), and (c) nothing about the new rotate ops conflicts with
the existing translate-based movement/pause/loiter logic.**

**Still open, explicitly deferred (not silently dropped):**
- Real skeletal walk-cycle animation (documented above as high-risk/
  version-fragile even for NVIDIA's own recommended workflow)
- Robot's on-arrival scan is still a single static snapshot with no active
  face-search/aiming behavior (see earlier conversation - separate task,
  not started yet)

## BREAKTHROUGH: found the real animation filename - session ended here, resume next time

Session hit its limit right after a major find, documenting clearly so the
next session can pick up immediately with no re-investigation needed.

**Diagnostic method fix:** `Sdf.Layer.FindOrOpen()` (used in early versions
of `test_animation.py`) was CONFIRMED UNRELIABLE for checking whether a
remote HTTP asset path exists - it returned False for every single guess,
including paths later confirmed real. Replaced with `omni.client.list()`/
`omni.client.stat()` (new script: `list_people_assets.py`), which is the
real, correct API for listing/checking Nucleus/HTTP asset server contents.
Always use `omni.client`, not `Sdf.Layer.FindOrOpen()`, for this kind of
check going forward.

**The real animation file, confirmed via actual directory listing:**
```
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Animations/stand_walk_loop.skelanim.usd
```
A proper looping walk-cycle clip. Other useful ones in that same folder:
`stand_idle_loop.skelanim.usd`, `stand_walk_1.skelanim.usd` through `_7`
(non-looping variants + `_mirror` versions), `Sit.skelanim.usd`,
`stand_idle_wave_loop.skelanim.usd`.

`test_animation.py` has been updated to use this confirmed real filename
(was previously guessing wrong names and always failing at that step) and
to use the fixed `omni.client.stat()`-based resolve check. **This has NOT
been run yet with the real filename - that's the very next step.**

**Exact next steps for next session, in order:**
1. Run `test_animation.py` (already updated with the real filename) -
   fully isolated, touches nothing in the real project. Its final output
   ("Joints with different transforms between t0 and t1: N / 101") tells
   us definitively whether real skeletal animation actually plays in this
   standalone-script Isaac Sim environment (the `omni.anim.graph.core`
   extension-enable fix from the NVIDIA forums is already applied in this
   script, attempting to work around the documented standalone-app T-pose
   bug).
2. If SUCCESS (joints genuinely animate): port the same reference+bind
   approach into a one-time setup script that modifies the REAL
   `warehouse_v1.usd` (on a saved copy first, e.g.
   `warehouse_v1_animated.usd`, never overwrite the working file directly
   until fully verified) - reference `stand_walk_loop.skelanim.usd` onto
   each of the three People characters' skeletons, verify visually, then
   point `USD_STAGE_PATH` in `unified_tracking.py` at the new file.
3. If FAILED (joints still static): the `omni.anim.graph.core` fix wasn't
   enough on its own. Fall back cleanly to the current stopgap (face-
   direction-of-travel + step bob, already implemented and working) and
   consider this explicitly closed rather than reopening again - we will
   have then tried the two most promising approaches (manual bind + the
   documented extension fix) and both would have failed, meaning IRA
   (`isaacsim.replicator.agent`) or a from-scratch procedural joint-driver
   would be the remaining options, both substantial standalone efforts.

**Also confirmed the character folder listing** - if different/additional
character variety is ever wanted (user mentioned current three are just
placeholders, open to swapping), real available character folders include:
`F_Business_02`, `F_Medical_01`, `M_Medical_01`,
`female_adult_police_01_new`, `female_adult_police_02`,
`female_adult_police_03_new`, `male_adult_construction_01_new`,
`male_adult_construction_03`, `male_adult_construction_05_new`,
`male_adult_police_04`, plus several `original_*` variants of each.

## CONCLUSIVE FINDING: characters are Reallusion-rigged, incompatible with NVIDIA's own animation clips

Ran the isolated test with the real animation filename
(`stand_walk_loop.skelanim.usd`) found via `list_people_assets.py`.

**Attempt 1 (with omni.anim.graph.core etc. force-enabled):** FAILED with a
hard error - `Buffer size mismatch: translations=81, rotations=81,
expected jointCount=101` from `omni.anim.skelJoint.plugin`.

**Attempt 2 (skipped those NVIDIA extensions, plain UsdSkel only):** no
crash, but still 0/101 joints animating.

**Root cause, found via direct joint-name comparison (added to
`test_animation.py`):** ZERO name overlap between the skeleton and the
animation clip. The character skeleton uses `RL_BoneRoot/Hip/Pelvis/
L_Thigh/...` naming - the unmistakable signature of a **Reallusion
Character Creator** rig. The animation clip uses `Root/Pelvis/R_UpLeg/
R_LoLeg/...` - NVIDIA's own generic biped naming. These are two
completely incompatible skeleton conventions. Confirmed this isn't a
fluke of one character - tested a second character
(`male_adult_construction_01`) and got the identical `RL_BoneRoot` naming
and identical zero overlap. **Every character in the People/Characters
folder is Reallusion-rigged and will never directly bind to NVIDIA's own
Animations folder clips - this is a hard, structural incompatibility, not
a bug to fix.**

**Investigated DH_Characters ("Digital Human") as an alternative** -
these turned out to be NVIDIA's high-fidelity MetaHuman-quality digital
human assets. Loading even ONE of these took over 20 minutes (stuck
compiling MDL shader materials) before being killed - far too heavyweight
for a real-time multi-character patrol sim regardless of rig
compatibility. Ruled out on performance grounds alone, in addition to
rig-compatibility being unconfirmed.

**Decision for next session:** stop trying to use Isaac Sim's own bundled
animation assets - they don't match this character rig family. Instead,
since the characters are confirmed Reallusion-rigged, use an EXTERNAL
source built specifically to retarget animation onto arbitrary humanoid
rigs (including Reallusion ones) - this is a proven, well-trodden workflow
used by millions, not another blind compatibility guess:
- **Adobe Mixamo** (free, huge animation library, auto-retargets to
  arbitrary humanoid skeletons including Reallusion rigs) - most promising
  first try.
- **Reallusion's own ActorCore/AccuRIG ecosystem** (since the character IS
  a Reallusion asset, their own animation pipeline is natively compatible).

User confirmed they're fine with pulling animation from outside Isaac
Sim's own asset library and doing manual setup work if needed, as long as
it's something that will DEFINITELY work rather than another experiment.
Current stopgap (face-direction-of-travel + step bob in
`update_person_position()`) remains in place and working in the meantime -
not reverted, still the active behavior until real animation lands.

## FOUND (confirmed): robot drove into a pillar - costmap was never getting real lidar data

User reported the robot physically drove into a wall/pillar, and goals kept
timing out (240s, no ABORTED) even after the use_sim_time fix. Checked
`ros2 topic list` from earlier in the session: the actual published lidar
topics are `/lidar_front/scan` and `/lidar_rear/scan` - there is NO topic
named `/scan`. But `nav2_params.yaml`'s local_costmap and global_costmap
obstacle/voxel layers were both configured with `observation_sources: scan`
pointing at `topic: /scan` (the stock default from the upstream Nav2
template, never updated for this robot's actual topic names). This means
Nav2 has had ZERO real-time obstacle awareness this entire time - it was
navigating using only the static pre-built map, with no live lidar feeding
the costmap at all, which is exactly consistent with driving straight into
a physical obstacle that happened to sit in its path.

**Fix (confirmed root cause, high confidence):** `nav2_params.yaml`'s
local_costmap and global_costmap now each define two observation sources,
`scan_front` and `scan_rear`, pointing at the real topics `/lidar_front/scan`
and `/lidar_rear/scan` respectively. Also added matching `scan_front`/
`scan_rear` sources to `collision_monitor`'s observation_sources (was also
pointing at the nonexistent `scan` topic).

## ADDED (defensive/exploratory): self-healing TF relationship repair

Separately, the long-standing `[PoseTree] getObjectType eInvalid` spam for
base_link/lidar_frame/etc. (documented as a known issue previously) could
ALSO independently block obstacle avoidance even with the topic fix above,
since Nav2 needs a working transform from each LaserScan's frame to the
costmap's frame to place obstacle points correctly. Rather than guessing
blind at which exact prim paths are broken and what to replace them with
(no way to safely verify without live introspection), added
`repair_broken_tf_publisher_targets()` to `unified_tracking.py`, which runs
once at startup and:
1. Walks every USD relationship stage-wide looking for 'target'/'parent'
   type relationships (how OmniGraph node prim-references are actually
   stored at the USD level) whose targets don't resolve to a valid prim.
2. For each broken one, searches the robot's real prim hierarchy for a
   valid prim with the exact same leaf name and repairs the reference if
   the match is unambiguous.
3. Prints exactly what it checked/fixed/couldn't confidently fix, so the
   next run's console output tells us definitively whether this worked,
   rather than more guessing.

This uses only core, stable USD Python API calls (Usd.PrimRange,
GetRelationships, GetTargets, SetTargets) rather than the OmniGraph node
API directly, to minimize risk of the repair code itself failing due to
API version differences - if it can't find/fix anything it fails safe and
just prints so, without blocking the rest of the script.

**Not yet re-tested - both fixes need a real run to confirm.** Watch the
console for `[tf_repair]` output and check whether the robot now visibly
steers around obstacles instead of driving through them.

## FOUND (confirmed): robot driving into corner was a GOAL PLACEMENT bug, not obstacle avoidance

User sent a screenshot: robot literally parked against a wall-corner pillar,
never able to reach goals. Real cause, now confirmed: `ZONE_CENTERS`
coordinates are chosen for CAMERA FRAMING - they sit right at/against the
walls the zone cameras are mounted near. Sending the robot literally TO
that exact point means its destination is physically inside or hard
against a wall - not a costmap/lidar/obstacle-avoidance failure at all.

**Fix:** `dispatch_bridge.py` now applies `apply_goal_standoff()` before
sending any alert-based goal - pulls the raw zone coordinate inward toward
map center (0,0) by `GOAL_STANDOFF_METERS` (3.0m) so the robot's actual
Nav2 goal is a reachable point near the zone instead of on top of the wall.

## Log noise + render speed pass

Three more fixes in the same pass, all in `unified_tracking.py`'s startup
block:

1. **"sequence size exceeds remaining buffer" - real fix, not suppression.**
   This comes from rtx.scenedb's transform-history ring buffer overflowing;
   an earlier, related warning literally names the fix:
   `--/rtx/scenedb/maxHistoryTransformCount=245`. Set
   `/rtx/scenedb/maxHistoryTransformCount` to 512 at startup.
2. **`[PoseTree] eInvalid` spam** - the global `/log/level=Error` setting
   never caught this channel. Added defensive per-channel suppression via
   both `/log/channels/<channel>/level` settings-path overrides AND the
   `omni.log.set_channel_enabled()` API (whichever the installed Kit
   version actually supports), for `isaacsim.ros2.nodes` and a few other
   noisy channels. Wrapped in try/except so an API mismatch fails silently
   instead of crashing startup.
3. **Render speed** - added `/rtx/pathtracing/enabled=False` and disabled
   reflections/AO/indirect-GI/sampled-lighting via carb settings
   (best-effort, version-dependent, wrapped safely), and bumped every
   DomeLight/DistantLight/SphereLight in the stage to intensity 2.0 via
   direct USD edits (guaranteed to apply, not a setting-name guess) for
   flat, cheap-to-render lighting. Also reduced `capture_frame()`'s max
   wait-loop from 150 to 60 `simulation_app.update()` calls, since the
   camera-object caching fix means this rarely needs anywhere near that
   many tries now.

**None of these three re-tested yet.** The log-suppression and render-speed
items are best-effort/version-dependent (multiple approaches tried
defensively since the exact Kit API surface isn't verifiable without
running it) - the next run's console output (noise level + how many
"[log suppression didn't apply]"-style silent failures, if any) is the
real test. The goal-standoff fix is straightforward, high-confidence logic
and should visibly stop the wall-driving behavior.

## FOUND: post-hoc carb.settings calls don't work - moved everything into SimulationApp() launch config

Confirmed via a fresh run: the log-noise and render-speed fixes above did
NOTHING - identical PoseTree/buffer spam, no speed change. Root cause:
settings applied via `carb.settings.get_settings().set(...)` AFTER
`SimulationApp()` has already run are too late for a lot of this. Extension
logging channels get registered and some renderer internals get locked in
at Kit's actual process startup, before our Python code runs at all - this
held true even when the settings calls were moved as early as possible in
the script (immediately after SimulationApp(), before enabling the ROS2
bridge extension) in the previous attempt.

**Fix:** moved every log-level and rendering setting directly into the
`SimulationApp({...})` constructor dict itself, since that config is what
SimulationApp uses to build Kit's actual startup command line - the
earliest possible point settings can apply, before any extension
registers. Also added `"width": 640, "height": 480` to shrink the
interactive viewport (a real, direct rendering-cost reduction, separate
from the camera capture resolutions used for detection). Per-channel log
suppression (`/log/channels/...`) still has to happen via Python calls
since there's no launch-config equivalent for arbitrary channel names, but
now runs immediately after SimulationApp() instead of later.

Also flagged for the user: the single biggest available speed lever is
switching `HEADLESS = True` at the top of the file - GUI mode renders a
full interactive window every frame in addition to every camera capture,
and the user has mentioned the GUI freezes so much they don't really watch
it live anyway.

**Not yet re-tested.**

## CONFIRMED WORKING: log-noise fixes actually worked this time

Re-ran with settings moved into the SimulationApp() launch config -
confirmed: PoseTree eInvalid spam is gone (0 occurrences vs. hundreds
before), "sequence size exceeds remaining buffer" is completely gone, and
the sim isn't stalling/freezing the way it used to. The launch-config
approach was the right fix.

Remaining noise is low-volume, one-time-per-source warnings (not spam
loops): camera_info_utils aperture-mismatch warnings, omni.timeline
deprecation notice, a couple of carb performance-warning notices, and the
synthetic-data "counter-performant" rendervar copy note. Added those
channels (`isaacsim.ros2.core.impl.camera_info_utils`, `omni.timeline.plugin`,
`carb`) to the per-channel suppression list, and fixed the aperture warning
at the actual source: `capture_frame()` now explicitly sets each camera's
horizontal/vertical aperture to match its resolution's aspect ratio at
creation time, instead of letting Isaac Sim silently auto-correct (and log
about) it on every access.

## FOUND: the log-noise "fix" actually broke everything worse

The run right after the previous fix regressed hard - full PoseTree spam
AND "sequence size exceeds remaining buffer" both came back in force,
plus a new error block at startup:
`[Error] [carb.dictionary.plugin] getStringRawInternal: item isaacsim.ros2.nodes is not a string`
(repeated once per channel name). Root cause: Kit's real schema for
`/log/channels/<name>` is a PLAIN STRING (the level itself, e.g.
`"Error"`), not a nested object. Setting `/log/channels/<name>/level` and
`/log/channels/<name>/enabled` created a dict at that path instead,
corrupting the exact structure Kit's own startup code reads as a string
when it registers channels - breaking log-channel registration outright
and cascading into everything reappearing (and possibly contributing to
the renderer settings not landing correctly either, though those were
separate constructor-dict keys and less directly implicated).

**Fix:** switched to the correct flat schema -
`carb.settings.get_settings().set(f"/log/channels/{channel}", "Error")`,
one plain string per channel, no sub-keys. Also removed the
`omni.log.set_channel_enabled()` attempt entirely (unverified API guess,
not worth stacking another untested mechanism on top of a now-correct one).

**Not yet re-tested.**

## FOUND: real root cause - blocking subprocess calls starved physics stepping

After the RigidPrim fix, position tracking was confirmed accurate (debug
print showed sensible, live coordinates), but the robot still barely moved
(under 0.2m of real drift over 9 minutes) and `dispatch_bridge.py` reported
goals as "finished" when they were actually ending with status ABORTED
(confirmed via `ros2 action send_goal` run manually and reading the full
Result block - dispatch_bridge.py was only checking for the string "Goal
was rejected" as a failure condition, missing ABORTED/CANCELED entirely).

Nav2's own log showed a repeating `follow_path aborting -> spin/backup
recovery -> follow_path aborting` cycle for the full goal duration - the
classic "robot isn't actually making progress" pattern. `ros2 topic hz
/cmd_vel_nav` and `/cmd_vel` both showed healthy ~20Hz with real nonzero
velocity values reaching the robot, and the map's origin/bounds checked out
fine for the target coordinates - so the problem wasn't Nav2, the relay, or
the map. Confirmed visually: the robot was not moving on screen at all
during an active goal.

**Root cause:** `run_sweep()`'s YOLO/DeepFace calls used
`subprocess.run()` - fully blocking. `ros2 topic hz` measures messages on
the ROS2 DDS wire, which is completely independent of whether Isaac Sim is
actually stepping physics to consume them. While `run_subprocess()` blocked
(confirmed earlier at 9-15+ seconds per call), `simulation_app.update()`
never ran, so PhysX never stepped - meaning `/cmd_vel` commands kept
arriving on the wire the whole time but were never actually applied to the
robot. With sweeps every 8s and blocks often longer than that, the sim was
frozen for a large fraction of total time, so Nav2 saw a robot barely
making progress and kept aborting/recovering.

**Fix:** `run_subprocess()` now uses `subprocess.Popen()` and polls it in a
loop that keeps calling `simulation_app.update()` the whole time the
subprocess runs, instead of blocking completely. Also fixed
`dispatch_bridge.py` to check the actual result status (SUCCEEDED/
ABORTED/CANCELED) instead of just the absence of "Goal was rejected", so
it stops silently reporting failed goals as successes.

**Not yet re-tested** - this is the most likely real fix given how well it
explains every symptom observed (odom jitter fixed by the earlier camera
cache fix, but sweep-duration stalls remained and were long enough to
starve navigation specifically, even though basic odom publishing looked
fine in isolation).

## Robot speed increased + custom nav2_params.yaml added

Once navigation was actually working, the robot's real driving speed felt
too slow (default Nav2 caps: ~0.5 m/s linear, ~1.9 rad/s angular) and the
controller was hitting "Failed to make progress" aborts, partly from
residual sweep-timing stalls and partly from just being slow relative to
its patience window.

Copied `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` into the
project as `nav2_params.yaml` (so it can be tuned without touching the
system ROS install) and bumped: `FollowPath.vx_max` 0.5 -> 1.2,
`FollowPath.wz_max` 1.9 -> 2.5, and matching `velocity_smoother.max_velocity`
/ `max_accel` values (the smoother otherwise clamps speed right back down
regardless of what the controller requests). `start_nav_stack.sh` now
passes `params_file:="$PROJECT_DIR/nav2_params.yaml"` to the Nav2 launch.

**Not yet re-tested.**

## FOUND: real root cause of "Failed to make progress" - wall-clock vs sim-clock mismatch

After the speed bump, goals started timing out at 240s with no ABORTED at
all - a different failure mode. Nav2 log showed the real cause:
`Planner loop missed its desired rate of 20.0000 Hz. Current loop rate is
1.4031 Hz` and similar for the control loop. Isaac Sim's simulation is
running noticeably slower than real time (rendering + camera captures +
subprocess overhead), but Nav2's progress checker and control loop timing
use REAL wall-clock time by default (`use_sim_time` was never set, so it
defaulted to false everywhere). This meant Nav2's "robot must move 0.5m
within a 10-second window or abort" check was being judged against real
seconds, while actual simulated robot movement during that window was much
smaller than it should be because the sim itself was running slow -
a fundamental clock mismatch, not a speed or physics-starvation problem.

**Fix:** `start_nav_stack.sh` now passes `use_sim_time:=true` to the Nav2
bringup launch, and the static_transform_publisher is now launched with
`--ros-args -p use_sim_time:=true` too (switched from positional args to
`--x/--y/--frame-id` flags to support this). Isaac Sim already publishes
`/clock` via its ROS2 bridge (confirmed present in `ros2 topic list`
earlier), so no changes needed on the Isaac Sim side - Nav2's internal
timers should now scale with however fast/slow the sim is actually
running instead of racing against real time.

**Not yet re-tested.**

## ATTEMPT 2: get_translate() fix alone wasn't enough - switched to RigidPrim

After the ComputeLocalToWorldTransform fix above, arrival STILL never
triggered across multiple dispatch alerts and confirmed-completing Nav2
goals. Suspected cause: Isaac Sim's physics runs on an internal fast data
layer ("Fabric") for performance, and does not reliably write simulation
results back into the USD stage's authored xform attributes - the robot
visually moves correctly (rendering reads from Fabric directly), but
anything reading raw USD attributes (including ComputeLocalToWorldTransform)
can still see stale/default data.

**Fix:** `update_robot()` now queries the robot's position via
`isaacsim.core.prims.RigidPrim.get_world_poses()`, which reads the live
physics simulation state directly rather than USD stage attributes. Also
added a debug print every arrival check showing current position, target,
and distance, so if this still doesn't work we can see the actual numbers
instead of guessing blind again.

**Not yet re-tested.**

## FOUND (high confidence, not yet re-tested): real root cause of "robot only moves 0.1 m/s" - idle cameras never stop rendering

Confirmed both `nav2_params.yaml` (`FollowPath.vx_max: 10.0`) and the
differential_controller OmniGraph node (`fix_differential_controller_speed.py`,
all maxLinearSpeed/maxAngularSpeed/etc already at generous values) were
already maxed out from earlier sessions - raising either further would do
nothing, consistent with every previous "bump the speed config" attempt
having zero real-world effect.

The actual bottleneck, per `unified_tracking.py`'s own `[PERF]` updates/sec
instrumentation (already in place from the wall-clock-vs-sim-clock
investigation): real simulated time per real second is collapsing. Root
cause - `capture_frame()`'s `_camera_cache` keeps every `Camera` object
it ever creates alive and rendering for the rest of the run. This scene
has 7 total cameras (5 zone cams + checkpoint + robot cam), so by partway
through any run all 7 are simultaneously rendering their own full render
product on EVERY `simulation_app.update()` call - including the ~90%+ of
wall-clock time between 8-second sweeps when nothing is even being
captured and the robot/people should be moving at full commanded speed.
This matches the previously-logged "GPU render cost per camera capture
(6+ simultaneous render products)" note exactly, except it turns out not
to be a fixed hardware ceiling - it's 6 idle cameras rendering for no
reason, which is fixable in software.

**Fix:** `capture_frame()` now calls `camera.resume()` right before
waiting for a frame and `camera.pause()` right after (both real public
`isaacsim.sensors.camera.Camera` methods), so only the ONE camera
actively being captured is ever rendering at a time - every other cached
camera sits paused between uses instead of costing a full render every
tick. Wrapped in try/except (flips a module-level flag and falls back to
always-on cameras with a `[WARN]` print if `.pause()`/`.resume()` aren't
supported on this Isaac Sim build) so a version mismatch fails safe
instead of breaking capture.

**Not yet re-tested - this is the real test to watch for:** run
`unified_tracking.py` and check the `[PERF] N sim updates in 5.0s real
time = X updates/sec` lines. If this fix is right, updates/sec should
climb noticeably once the first sweep finishes and cameras start
pausing between uses (compare against a baseline run without this change
if you want a clean before/after number), and the robot's real-world
m/s should track much closer to what Nav2/the differential_controller are
actually commanding instead of sitting at ~0.1 m/s regardless of config.

## TESTED, PARTIALLY WRONG: camera pause/resume didn't move the needle - real ceiling is elsewhere

Ran with the camera pause/resume fix above. Result: `[PERF]` showed
~12 updates/sec BEFORE the first sweep even started - i.e. with
`_camera_cache` still completely empty, zero cameras created yet. That
proves the camera-pause hypothesis wrong as the explanation for the
baseline ceiling (it may still help marginally once multiple cameras are
actually active mid-sweep, left in place, but it isn't THE bottleneck).
Also, `sequence size exceeds remaining buffer` spam came back in heavy,
constant volume from the very start of the run.

**Two real findings from this run:**

1. **`run_sim_filtered.bat` was never actually being used** (user was
   invoking `python.bat` directly) - moved to
   `_unused_run_sim_filtered.bat.txt` at the user's request rather than
   left around to cause confusion again. Per the user, this script was
   dropped earlier because in practice it appeared to filter out
   everything, not just the buffer-spam lines - not re-investigated further
   since the user doesn't want to rely on output-filtering as the fix
   anyway.
2. Bumped `/rtx/scenedb/maxHistoryTransformCount` 512 -> 2048 as a
   belt-and-suspenders change, but this is a secondary cleanup, not
   expected to be the real speed fix on its own.

**Not yet re-tested with the buffer bump alone** - superseded by the
larger fix below before a dedicated test of just this change was run.

## REAL FIX (high confidence, not yet tested): physics/render decoupling via SimulationContext

The actual explanation for a flat ~12 updates/sec ceiling that exists even
with zero cameras active: `simulation_app.update()` couples a full Hydra
render to EVERY single tick, with no way to separate them. Rendering, not
physics or ROS2 message passing, is the expensive part - so every tick,
including the vast majority that are just "step physics, check robot
arrival, let ROS2/Nav2 do their thing," was paying full render cost for no
reason.

**Fix:** switched `_tick()` from raw `simulation_app.update()` to
`isaacsim.core.api.SimulationContext.step(render=...)` (the standard,
documented Isaac Sim pattern for headless/bulk-physics speed - physics and
OmniGraph, including the ROS2 bridge nodes that drive /odom, /tf, and
/cmd_vel consumption, still execute on every single tick; only the actual
Hydra render pass is now conditional). A new `RENDER_EVERY_N_TICKS = 6`
throttle means only 1 in 6 ticks renders by default, EXCEPT inside
`capture_frame()`'s frame-wait loop, which now calls `_tick(force_render=True)`
since that loop genuinely needs a real image back every time. Every other
`_tick()` call site (movement, robot arrival checks, subprocess-wait polling,
face-search yaw waits) doesn't need a picture, just a physics step, so it
falls through to the throttle.

Wrapped in the same fail-safe pattern as everything else in this file:
`SimulationContext.instance()`/`SimulationContext()` acquisition and every
`.step(render=...)` call are in try/except blocks that flip
`_render_decouple_supported = False` and fall back to plain
`simulation_app.update()` on any failure, so a version mismatch on this
Isaac Sim build degrades to the old (slow but working) behavior instead of
breaking physics/ROS2/movement.

Also upgraded the `[PERF]` line itself to report both total sim steps/sec
AND rendered frames/sec separately, so it's directly visible whether the
decoupling is doing anything (e.g. "60 sim steps (10 rendered) in 5.0s =
12.0 steps/sec, 2.0 rendered/sec" would mean physics is running 6x faster
than the old render-locked rate while only rendering the same 1-in-6
fraction).

**Not yet re-tested - this is the real test:** run `unified_tracking.py`
(directly with `python.bat`, NOT the removed filtered wrapper) and watch
for:
- A `[WARN] Could not acquire SimulationContext` or
  `[WARN] SimulationContext.step(render=) failed` line - if either
  appears, this build's API doesn't support the exact call used here and
  the whole thing silently fell back to the old behavior (no crash, no
  speed gain either).
- If neither WARN appears: the `[PERF] ... steps/sec, ... rendered/sec`
  line should show steps/sec meaningfully higher than rendered/sec (close
  to 6x apart, matching RENDER_EVERY_N_TICKS), and real-world robot m/s
  should finally track much closer to what Nav2/the differential_controller
  are actually commanding.
- Watch Nav2's own logs for anything about missed control-loop rate or TF
  lookup failures - if physics is now advancing much faster in wall-clock
  terms than before, double-check `use_sim_time` timing still lines up
  correctly (it should, since Isaac's `/clock` publish rate is tied to
  physics steps, which are now happening MORE often per real second, not
  less - but worth watching for on the first run regardless).
- If TF/ROS2 topics look sparse or Nav2 behaves worse, try lowering
  `RENDER_EVERY_N_TICKS` (e.g. to 3) rather than reverting outright - some
  ROS2 publishers in Isaac Sim's bridge may be tied to render events rather
  than pure physics steps, which would mean this needs tuning rather than
  a full revert.

## ADDED: diagnose_cam3_cam4_mismatch.py - empirical test for the Cam3/Cam4 zone mismatch

Written to answer the open Cam3/Cam4 bug definitively instead of guessing:
places a bright magenta marker cube at each zone's own ZONE_CENTERS
coordinate (one at a time), captures ALL 4 real cameras for each
placement, and scores magenta-pixel presence per frame automatically
(plus saves every labeled frame to `debug_captures/cam_mismatch/` for a
visual double-check). Prints a SUMMARY table: for each zone, which camera
actually saw its own marker vs. which camera `camera_zones.json` currently
says owns that zone. Also independently prints each camera's raw world
position + computed forward vector as a second check, in case a camera's
orientation (not the mapping) turns out to be the real issue.

**Not yet run.** Next step: run it and read the SUMMARY block - a
mismatch there is the direct confirmation needed before touching
`camera_zones.json`.

## CONFIRMED (diagnose_cam3_cam4_mismatch.py run): NOT a Cam3/Cam4-only swap - both room pairs were swapped, and Camera2 is unconfirmed/possibly broken

Actual run results (magenta-marker test, headless):
- Marker at RoomA_Cam1's coord (22.60, 27.36): seen by **nobody** - 0 magenta pixels on all 4 cameras.
- Marker at RoomA_Cam2's coord (39.21, 9.00): seen by **Camera1** (63 px), not Camera2.
- Marker at RoomB_Cam3's coord (21.64, 8.91): seen by **Camera4** (63 px), not Camera3.
- Marker at RoomB_Cam4's coord (5.83, 27.40): seen by **Camera3** (66 px), not Camera4.

Pattern: each camera actually watches the diagonally-opposite corner
within its OWN room, not its own mount point - Camera1 watches Camera2's
corner and vice versa (RoomA), Camera3 watches Camera4's corner and vice
versa (RoomB). This isn't a simple "Cam3/Cam4 swapped" bug as originally
suspected from the Person2-fall symptom - both room pairs were wrong.

**Fix applied:** `camera_zones.json` updated to:
```
{"/World/Camera1": "RoomA_Cam2", "/World/Camera2": "RoomA_Cam1",
 "/World/Camera3": "RoomB_Cam4", "/World/Camera4": "RoomB_Cam3"}
```
`ZONE_CENTERS` in `unified_tracking.py` was NOT touched (per the original
plan - those coordinates are correct, only the camera-prim-to-zone-name
mapping was wrong).

**Real open caveat, not resolved:** Camera2's pairing with RoomA_Cam1's
coordinate (22.60, 27.36) is inferred BY ELIMINATION, not empirically
confirmed - the marker at that exact spot registered 0 magenta pixels on
EVERY camera in the test, including Camera2. This could mean the
diagonal-pairing pattern holds and Camera2 just has a narrower FOV/some
obstruction at that specific point, or it could mean Camera2 is
genuinely broken/misoriented independent of the naming bug - not
distinguished yet. If a future run still shows RoomA_Cam1-zone events
never getting detected even after this mapping fix, treat Camera2 itself
as suspect next (re-run the diagnostic script with the marker placed at
a few other points inside Room A to map out what Camera2 can actually
see), not the mapping again.

**Not yet re-tested with a real fall/loiter run since this fix.**
