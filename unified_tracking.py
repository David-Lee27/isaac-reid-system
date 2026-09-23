r"""
unified_tracking.py - the whole simulation: scene setup, character movement,
zone-camera detection, and the robot's own decision-making, all in one
running Isaac Sim instance. See the repo's README.md for what the system
actually does; this docstring is just how to run it.

Four fixed zone cameras watch two rooms for falls and loitering. Simulated
people wander the building, check in with the robot at the front door, idle,
occasionally fall or overstay, and get identified by face and clothing.
Everything the robot does - checking someone in, helping a fallen person up,
chasing down a loiterer, escorting someone out - runs from this one file's
main loop.

Press Ctrl+C to stop.

Normally launched via run_simulation.bat (handles a console-buffering issue
plain python.bat doesn't - see that file's own comment), not directly:
    C:\isaacsim\projects\surveillance-proj\run_simulation.bat
"""

USD_STAGE_PATH = r"C:\Users\popli\isaacsim\scenes\optimized room.usd"  # real filename has a SPACE, not an underscore - confirmed from the Isaac Sim title bar
HEADLESS = False  # you asked to watch it - render/physics decoupling and the muted spam filter both still apply, this just also opens the actual viewport window instead of running invisibly.
# "RTX - Minimal" (visible in the viewport's render-mode dropdown) is the
# actual lightweight RTX preset for this Isaac Sim build - NOT a separate
# "Storm" renderer (that's not even an option in this build's menu; an
# earlier attempt to pass "renderer": "Storm" here was wrong and got
# silently ignored). This just makes explicit, at every launch, the same
# preset your screenshot showed already selected by default - real-time
# RTX with reflections/AO/GI/path-tracing off (matches the settings
# already disabled below), instead of relying on incidental default
# behavior that could change between Isaac Sim versions/machines.
USE_MINIMAL_RENDERER = True
PROJECT_DIR = r"C:\isaacsim\projects\surveillance-proj"
# Set SIM_RECORDING_MODE=1 before launching to skip writing the diagnostic
# debug_captures/ images (RAWFRAME/MASK/crop/REFERENCE JPEGs, several per camera
# sweep) - pure disk I/O with no effect on detection (blobs/aspect ratios are
# computed from the SAME in-memory frame either way). This is the fix for
# "recording makes it laggier": that I/O competing with a screen recorder's own
# disk writes is the actual, confirmed stutter source (the YOLO/DeepFace worker
# calls themselves already keep ticking/rendering while they wait - not the
# cause). The files still written in this mode are the functionally required
# ones: each temp frame handed to a worker, and known_faces/ face crops.
RECORDING_MODE = bool(__import__("os").environ.get("SIM_RECORDING_MODE"))  # `os` isn't imported yet this early in the file - see FALL_CHANCE's own comment for the same workaround
KNOWN_FACES_DIR = PROJECT_DIR + r"\known_faces"
ID_COUNTER_FILE = PROJECT_DIR + r"\next_id.txt"
CAMERA_ZONES_FILE = PROJECT_DIR + r"\config\camera_zones.json"
AUTHORIZED_IDS_FILE = PROJECT_DIR + r"\authorized_ids.json"
# Distinct from AUTHORIZED_IDS_FILE (a whitelist - "not on it" just means
# "unregistered visitor," not dangerous) - this is a real blacklist of IDs
# explicitly flagged as not allowed on premises. Matching someone against
# this list fires a real INTRUDER ALERT (a different, higher-severity
# event than the quiet "not authorized" logging a merely-unregistered
# person gets). See flag_id_as_banned.py for how an ID gets added to it.
BANNED_IDS_FILE = PROJECT_DIR + r"\banned_ids.json"
EVENT_LOG_FILE = PROJECT_DIR + r"\event_log.json"
DEBUG_DIR = PROJECT_DIR + r"\debug_captures"
APPEARANCE_PROFILES_FILE = PROJECT_DIR + r"\appearance_profiles.json"
APPEARANCE_MATCH_THRESHOLD = 0.7   # HSV histogram correlation, 0-1, higher = stricter

CHECKPOINT_NAME = "EntryCamera"
CHECKPOINT_PATH = "/World/EntryCamera"
# The new optimized room.usd scene has NO checkpoint/entry camera at all
# (confirmed via diagnose_new_scene_cameras.py - only Camera1-4 exist).
# Calling capture_frame() on a camera prim that doesn't exist would throw
# and crash the whole run the first time a sweep tried to scan it, not
# fail gracefully. CHECKPOINT_AVAILABLE is set for real in main() once the
# stage is loaded (by actually checking whether the prim exists), and
# run_sweep skips the checkpoint scan entirely when it's False. Identity
# checking now happens only via the robot's own orbit search on arrival -
# consistent with the current direction (zone cameras are fall/loitering
# detection only, the robot does the actual person-level investigation).
CHECKPOINT_AVAILABLE = False

SCAN_INTERVAL_SECONDS = 8   # how often to run a full camera sweep
LOITER_THRESHOLD = 3        # consecutive same-zone sightings to flag loitering
LOITER_STRIKES_BEFORE_BAN = 3  # separate loitering incidents (see the per-person dedup in run_sweep) an identified ID can rack up before getting banned and escorted out for good this run - not an instant one-strike ban, since ordinary ambient wandering trips the loitering threshold often enough that a one-strike policy emptied the building out within 2 minutes of testing it
LOITER_MIN_SECONDS_BETWEEN_STRIKES = 30.0  # the per-person dispatch dedup (LOITER_DISPATCH_COOLDOWN, in run_sweep) only stops a NEW alert from being created while one's still fresh - it doesn't stop the robot from processing a backlog of already-created, legitimately-separate alerts in a rapid real-time burst once it catches up. Without this floor, a burst like that counted several strikes in under a second and banned someone almost instantly.
ZONE_SCAN_RESOLUTION = (1280, 960)  # bumped from 640x480 - see scan_zone_camera()'s docstring; a bigger real pixel footprint makes the background-subtraction blob signal much less dominated by render/shadow noise at extreme zone-camera range

# --- People + patrol path ---
PEOPLE = {
    "Person1": {"prim_path": "/World/xbot_person_1", "start_delay": 0},
    "Person2": {"prim_path": "/World/xbot_person_2", "start_delay": 5},
}

# Everyone moves the same way now: idle until their assigned scripted
# fall/loiter event (see update_person_position) - the old ACTIVE_MOVER_COUNT
# concept (some people patrol, others frozen at spawn) doesn't apply anymore
# since NOBODY continuously patrols.

WAYPOINTS = [
    (27.46, 16.33, 0),
    (29.06, 27.76, 0),
    (-25.60, 28.53, 0),
    (-28.41, -7.00, 0),
    (25.09, -8.53, 0),
    (26.03, 12.98, 0),
    (29.9, 13.4, 0),
]
SECONDS_PER_LEG = 8  # halved patrol speed (was 4) per request
LOOP_PATROL = True

# Lie-down duration for the SCRIPTED fall event below (the camera-side
# detection no longer freezes anyone itself - see run_sweep's loitering
# check - so this is purely the scripted fall's own timing now).
# Raised 45 -> 90 per explicit direction ("the fall duration needs to be
# long enough so that the robot can help them up, this has to happen
# correctly"). This is only the MAXIMUM lie-down: the robot ending the
# fall early on arrival (assist_fallen_person sets paused_until = 0) is
# what normally gets them up, so a prompt response doesn't get any slower.
# Measured from real runs: when a fall IS detected, the camera flags it
# 3-34s in and the robot arrives 1-8s later - so 45s only ran short when
# detection was slow, and never helped at all when it never happened.
FALL_PAUSE_DURATION = 90  # seconds

# --- Randomized scripted fall event ---
# One random person actually, physically collapses at a random time during
# the run (rotated to lie on the ground, then frozen) - a REAL event for the
# camera's aspect-ratio fall heuristic to detect, instead of that heuristic
# only ever firing on incidental noise from a person who's still standing.
FALL_ENABLED = True
FALL_START_RANGE = (28, 160)     # widened from (28,40) - each mover now gets an INDEPENDENT fall roll (see main()'s scheduling), not just one shared random faller, so a wide staggered range keeps multiple falls from clustering all at once and gives a fuller demo over a longer window. Lower bound still pushed past ZONE_WARMUP_SWEEPS's ~24s reference-warmup window (see scan_zone_camera) - an earlier fall risked being scanned DURING warm-up and absorbed straight into the "empty room" reference itself, silently erasing the event instead of ever detecting it.
FALL_CHANCE = 1.0 if __import__("os").environ.get("SIM_FORCE_FALLS") else 0.7  # SIM_FORCE_FALLS=1 is a test hook so a verification run can't roll "nobody falls" (9% odds at 0.7 with 2 people). Per-mover independent probability of getting a scripted fall at all this run - not guaranteed for everyone, for variety
FALL_ROTATION_OPTIONS = [(90, 0, 0), (-90, 0, 0), (0, 90, 0), (0, -90, 0)]  # random fall direction, degrees

# --- Randomized loitering test ---
# One random person gets assigned a random window during the run where,
# instead of patrolling, they wander in small circles near a random zone -
# a realistic test of the loitering detector instead of manually posing someone.
LOITER_ENABLED = True
LOITER_START_RANGE = (28, 160)     # widened - see FALL_START_RANGE's comment, same reasoning (independent per-mover scheduling, staggered over a longer window)
LOITER_CHANCE = 0.7                # per-mover independent probability of getting a scripted loiter window at all this run
LOITER_DURATION_RANGE = (25, 40)   # how long they wander before resuming patrol
LOITER_RADIUS = 2.0                # kept for backward compat / reference, no longer drives the wander shape - see the room-box random-waypoint wander below

# --- Idle animations ---
# Per explicit direction ("i'm going to add some idle animations so the
# characters can do something rather than just walk around, like pause in
# certain areas and do a funny dance or ponder or something"). On arrival
# at an ambient wander target, a mover with idle clips bound (see
# bind_idle_animations.py/IDLE_CLIP_NAMES) has this chance to stop and
# play a random one for its real duration before picking a new
# destination, instead of always immediately moving on - see
# update_person_position's arrival branch.
IDLE_PAUSE_CHANCE = 0.4
IDLE_MAX_SECONDS = 10.0  # cap on any single idle hold - see update_person_position's arrival branch for why

# Zone camera positions used DIRECTLY as fall/loiter event locations - a
# security camera's actual field of view is centered near where it's
# mounted, so nudging the event location away from that (an earlier
# attempt, done purely to keep the robot's orbit search off the walls)
# was almost certainly why detections stopped happening entirely - the
# events were probably landing outside what the camera could actually see.
# Robot orbit safety near walls/corners is now handled separately by
# clamping orbit standpoints to FLOOR_BOUNDS (see search_for_face) instead
# of moving the event itself.
ZONE_CENTERS = {
    "RoomA_Cam1": (22.60, 27.36),
    "RoomA_Cam2": (39.21, 9.00),
    "RoomB_Cam3": (21.64, 8.91),
    "RoomB_Cam4": (5.83, 27.40),
}
DOORWAY_CENTER = (22, 17.5)  # your directly-observed value (close to the earlier computed 22.1,18.12 - using yours since you watched it directly)

# Room bounding boxes, from corners you gave directly by watching the scene
# (more trustworthy than anything computed from camera position alone) -
# used below for genuine randomized wandering within each room during a
# loitering event, instead of tracing a small fixed circle (which is what
# looked like "just walking the perimeter").
ROOM_BOUNDS = {
    "west": (5.0, 21.5, 9.0, 27.0),   # (xmin, xmax, ymin, ymax) - corners (5,9),(6,27),(21.5,27),(21,9)
    "east": (22.0, 39.0, 9.0, 26.0),  # corners (22,9),(39,9),(39,26), doorway (22,17.5)
}

# Real overall floor bounds (from diagnose_room_bounds.py: x:[4.93,39.93],
# y:[8.29,28.29]), with a small safety margin. Used as a HARD CLAMP - no
# matter what upstream logic (fall_center, loiter wander, robot chase
# tracking) computed a coordinate, nothing can ever actually place a
# person or the robot outside the real building. This exists because a
# fallen character was observed clearly outside the walls with no code
# path in this file that should currently be able to produce that -
# rather than keep guessing at which stale/cached code path did it, this
# makes it structurally impossible regardless of cause.
FLOOR_BOUNDS = (5.5, 39.4, 8.8, 27.8)  # (xmin, xmax, ymin, ymax)


def clamp_to_floor(x, y):
    xmin, xmax, ymin, ymax = FLOOR_BOUNDS
    return (max(xmin, min(xmax, x)), max(ymin, min(ymax, y)))


# --- Entry/exit door + ambient visitor population ---
# Per explicit direction ("have people come in to the rooms and people
# exit all the time like a real area"). First coordinate given (8.06,
# 1.68) turned out to have no real modeled wall opening nearby (confirmed
# via direct scene inspection) - visitors walked straight through solid
# wall to reach it. Corrected coordinates given live (39.75, 18.15) DO
# have real geometry there: two solid wall segments (wall3: y up to
# 17.39, wall3_03: y from 19.46) with a genuine walkable gap between them
# (wall3_04, the header piece above the gap, only starts at z=2.14 -
# above head height, not blocking at walking height) - a real exterior
# doorway on the east side of the building, confirmed by hand, not
# guessed a second time after the first coordinate's mismatch.
#
# DOOR_POS is OUTSIDE the building past the gap (where visitors actually
# spawn/despawn); DOOR_GAP_WAYPOINT is the real gap's own center, an
# intermediate waypoint every entry/exit route passes through first -
# same two-segment routing principle as the interior divider's own
# doorway (get_next_hop), just applied to this second, real opening.
# Also the intruder escort destination (see assist banned-detection
# flow) - the same door a visitor enters through is where the robot
# walks a banned person back out to.
DOOR_GAP_WAYPOINT = (39.97, 18.43)  # center of the real gap (x:[39.75,40.20], y:[17.4,19.46])

# People waiting to be checked in stand INSIDE the room, just past the door wall
# (whose inner face is x=39.75), not in the door frame over the void outside.
CHECKIN_WAIT_POS = (39.15, 18.43)
# The reception counter (walls wall3_05/06/07) sits between the door and the room;
# people go around it (north side) instead of through it. Corner points, in order.
COUNTER_VIA_NORTH = [(38.9, 20.6), (36.6, 20.6)]
COUNTER_VIA_SOUTH = [(38.9, 16.4), (36.6, 16.4)]
STARTUP_LOAD_SECONDS = 30.0  # scene/models settle before anything moves or gets timed
DOOR_POS = (41.5, 18.43)  # a few meters past the gap, fully outside the building
VISITOR_ASSET_PATH = r"C:\isaacsim\projects\surveillance-proj\assets\character_4.usd"
VISITOR_SPAWN_INTERVAL_RANGE = (25.0, 50.0)  # seconds between new visitor spawn attempts
MAX_CONCURRENT_VISITORS = 2
VISIT_DURATION_RANGE = (30.0, 70.0)  # seconds a visitor stays inside before heading back to the door
# Turned off per explicit direction ("keep it to 3 people max, the 2
# people and the 1 anomaly" / "the simulation is getting pretty laggy...
# people stop moving sometimes") - continuous ambient visitor churn on
# top of the 3 base characters pushed concurrent population as high as
# 6, which is what was actually driving the slowdown (confirmed live:
# PERF dropped from ~170 steps/sec to ~38 steps/sec, and fall dispatch
# alerts stopped getting handled in any reasonable time because of it).
# The door-entry/check-in MACHINERY itself (spawn_visitor,
# update_person_position's two-leg gap routing) stays intact and is now
# reused for Person4's own one-time entrance instead of being deleted -
# just nothing schedules a spawn_visitor() call on a timer anymore.
VISITOR_SYSTEM_ENABLED = False

# --- Second intruder ---
# Per explicit direction ("assign 2 ids to the door denial now so you can
# have multiple people denial") - Person5 spawns this many seconds after
# Person4 (who still spawns immediately, unchanged), via spawn_intruder()
# on the SAME delayed-timer pattern main()'s loop already used for
# ambient visitor spawning, kept independent of VISITOR_SYSTEM_ENABLED
# (that flag is about ordinary visitor churn, not this). Staggered on
# purpose rather than spawning both at once - they'd otherwise start
# stacked on the exact same DOOR_POS tile, and the demo reads better as
# two distinct denial events over time instead of a simultaneous pileup.
INTRUDER2_SPAWN_TIME = 60.0  # seconds after Person4 spawns

# --- Overstay time limit ---
# Per explicit direction ("make people leave at a reasonable time...
# the robot will keep in their memory this id has stayed in here for
# 5 minutes, like how some places have time limits... if the time limit
# is too high let's just do 1 minute for now... if they go over 1 minute
# then the robot go finds them and escorts them away"). Measured from
# the first time this run's robot ever positively identified that
# person's face (recorded by attempt_robot_scan - see VISIT_TIME_LIMIT's
# use in main()'s overstay-detection sweep and the "overstay" branch of
# update_robot), not from when they entered - this project has no
# per-mover door-entry timestamp for the two base ambient characters
# (they're already inside at t=0, never having "entered" through
# anything), so "how long have we known about them" is the only
# consistent clock that works for everyone alike, checkpoint entrants
# and ambient wanderers both.
VISIT_TIME_LIMIT = 60.0  # seconds - 1 minute, per explicit direction ("let's just do 1 minute for now")
# Nobody should be gone for good: once someone has been escorted out (overstay) or
# turned away (banned intruder) they come back through the door as a fresh visit,
# so a long run - or a demo video - never ends up as an empty building.
RESIDENT_REENTRY_DELAY = 20.0   # seconds outside before an escorted-out resident returns
INTRUDER_RETRY_DELAY = 90.0     # seconds before a turned-away intruder tries the door again (already banned, so it is recognised and denied)

# --- Robot checkpoint patrol ---
# Real redesign per explicit direction ("the robot is supposed to do
# identification for each person that enters the room... have the robot
# patrol the doorway, it stays around there and when someone goes
# through they get their id saved") - replaces the old
# cycle-through-ROBOT_WAYPOINTS default patrol entirely. The robot's
# default (no active fall/loitering alert) behavior is now: go to a
# fixed vantage point just inside the real doorway gap and stay there,
# periodically scanning/identifying whoever's nearest - not wander the
# whole building hoping to stumble across someone. Fall/loitering alerts
# still interrupt this exactly like they interrupted the old patrol (see
# the dispatch-check at the top of update_robot - unchanged).

# The user built a real L-shaped reception counter into the scene
# ("wall3_05/06/07") right next to the checkpoint spot, per direction:
# "make sure that the robot sits inside of it and waits for someone to
# check in" - instead of standing awkwardly out in the open doorway gap.
# Confirmed by direct inspection of the saved scene (world-space bounding
# boxes): wall3_05 is the solid back panel (x:[38.11,38.57], y:[17.05,
# 19.45], facing the door), wall3_06/07 are the two side arms extending
# west from it (y:[19.39,19.85] and y:[17.05,17.51] respectively, both
# spanning x down to 37.10) - together forming a "[" cavity open to the
# west, exactly like a real reception desk's attendant-side enclosure.
# CHECKPOINT_POS is now the center of that cavity (roughly x:[37.10,
# 38.11], y:[17.51,19.39]) instead of the bare doorway-adjacent point it
# used to be.
CHECKPOINT_POS = (37.6, 18.43)  # centered inside the reception counter's cavity, facing the door
CHECKPOINT_SCAN_INTERVAL = 5.0  # seconds between check-in scan attempts on whoever's currently waiting at the gate (retry throttle for a "no clear face yet" read, not a re-scan of already-checked-in people - see update_robot's checkpoint-arrival handling)
DENIAL_REACTION_SECONDS = 2.5  # how long a denied entrant plays the angry reaction before turning to leave - see the checkpoint denial branch


# The dividing wall between the two rooms is two solid segments with a
# walkable doorway gap between them. Since movement here is purely
# kinematic (we teleport prims directly - there's no PhysX simulation
# driving anything, so a collider alone would do nothing; PhysX only
# pushes back DYNAMIC bodies on contact, not something being repositioned
# by hand every frame), this is a real software wall-check using the
# actual measured wall geometry, not a decorative collider.
#
# DOORWAY_Y_RANGE corrected to the real measured gap (22, 16.3)-(22, 19.8),
# given directly from watching the live viewport - the previous
# diagnose_room_bounds.py-derived range (15.8-20.4) was padded WIDER than
# the actual opening, so the old code treated positions just outside the
# real gap (near its edges) as walkable, which is exactly why people were
# visibly walking into solid wall texture right at the doorway's edges.
DIVIDER_X_RANGE = (21.7, 22.4)     # small margin either side of the real 22.00-22.20 wall
DOORWAY_Y_RANGE = (16.3, 19.8)     # real measured gap, no outward padding


def blocked_by_divider(x, y):
    """True if (x, y) is inside the solid part of the dividing wall (i.e.
    NOT in the doorway gap)."""
    xmin, xmax = DIVIDER_X_RANGE
    if not (xmin <= x <= xmax):
        return False
    ymin, ymax = DOORWAY_Y_RANGE
    return not (ymin <= y <= ymax)


# The east wall with the door gap: solid across x in DOOR_FRAME_X except for the gap
# itself (measured y:[17.4,19.46]; inset a little so a body's width clears the frame).
DOOR_FRAME_X = (39.75, 40.20)
DOOR_FRAME_GAP_Y = (17.55, 19.31)


def move_with_wall_check(cur_x, cur_y, new_x, new_y):
    """Blocks movement through BOTH solid walls: the room divider and the door wall."""
    nx, ny = _move_with_divider_check(cur_x, cur_y, new_x, new_y)
    fxmin, fxmax = DOOR_FRAME_X
    gmin, gmax = DOOR_FRAME_GAP_Y
    if (cur_x <= fxmin and nx >= fxmax) or (cur_x >= fxmax and nx <= fxmin):
        dx = nx - cur_x
        y_a = cur_y + ((fxmin - cur_x) / dx) * (ny - cur_y)
        y_b = cur_y + ((fxmax - cur_x) / dx) * (ny - cur_y)
        if not (gmin <= y_a <= gmax and gmin <= y_b <= gmax):
            return (fxmin, ny) if cur_x <= fxmin else (fxmax, ny)
    elif fxmin < nx < fxmax and not (gmin <= ny <= gmax):
        return (fxmin if cur_x <= fxmin else fxmax), ny
    return nx, ny


def _move_with_divider_check(cur_x, cur_y, new_x, new_y):
    """Real wall-collision enforcement for kinematic movement: if the
    straight-line step from (cur_x, cur_y) to (new_x, new_y) would end up
    inside the solid dividing wall, the crossing axis (x) gets clamped back
    to whichever side of the wall the mover was already on, instead of
    just letting them phase straight through it - which is exactly what
    was happening when a chase target on the other side of the building
    pulled a straight kinematic line right through the middle wall.

    BUG (reported live: "the robot is also going through the wall near the
    middle part... they only enter the doorway through the wall, none of
    them want to go through the middle" - i.e. tunneling through the SOLID
    wall instead of routing through the actual doorway gap): the original
    version only checked whether the DESTINATION point landed inside the
    wall. The wall is only 0.7m thick (DIVIDER_X_RANGE), but a single
    movement tick can cover more than that (e.g. the robot at
    ROBOT_MOVE_SPEED=3.0 * MAX_MOVE_DT=0.5 covers up to 1.5m) - so a step
    that starts on one side and lands on the other has an endpoint outside
    the wall (looks "clear") even though the straight-line path punched
    straight through the solid part in between. Real fix: check the whole
    segment for a crossing, not just where it lands - if cur_x and new_x
    are on opposite sides of the wall's x-thickness, interpolate the y
    value at both x=xmin and x=xmax along that line and block unless BOTH
    crossing points fall inside the doorway's y-gap.
    """
    xmin, xmax = DIVIDER_X_RANGE
    ymin, ymax = DOORWAY_Y_RANGE
    tunnels_through = (cur_x <= xmin and new_x >= xmax) or (cur_x >= xmax and new_x <= xmin)
    if tunnels_through:
        dx = new_x - cur_x
        t_min = (xmin - cur_x) / dx
        t_max = (xmax - cur_x) / dx
        y_at_xmin = cur_y + t_min * (new_y - cur_y)
        y_at_xmax = cur_y + t_max * (new_y - cur_y)
        crossing_clear = (ymin <= y_at_xmin <= ymax) and (ymin <= y_at_xmax <= ymax)
        if not crossing_clear:
            return (min(new_x, xmin), new_y) if cur_x <= xmin else (max(new_x, xmax), new_y)
        return new_x, new_y

    if not blocked_by_divider(new_x, new_y):
        return new_x, new_y
    if cur_x <= xmin:
        return min(new_x, xmin), new_y
    else:
        return max(new_x, xmax), new_y


# Per explicit direction ("make sure people don't run into each other") -
# movement here is purely kinematic (direct XformOp writes, no PhysX
# collision driving any of it), and the only real collision logic that
# existed before this was move_with_wall_check's divider-wall check.
# Nothing stopped two people's independently-random wander targets from
# putting them on a crossing path that walked straight through one
# another. This is a deliberately simple radial push-out, not real
# steering/pathfinding - matches the kinematic style everything else in
# this file already uses, and is enough to visibly sidestep a crossing
# instead of overlapping.
PERSON_COLLISION_RADIUS = 0.6  # meters - roughly two people's combined half-widths


# The door zone: the gap in the east wall (only ~0.45 m wide) and the strip
# in front of it. Two people cannot share it, and the push-out below has no
# idea where walls are, so in here it would shove someone straight into the
# door frame (seen live: a returning resident "walked through the wall" while
# an intruder was being turned away in the same gap). Instead the door is
# handled by taking turns - see door_zone_busy and the yield in
# update_person_position.
DOOR_ZONE_MIN_X = 38.6
DOOR_ZONE_Y_RANGE = (15.5, 21.5)
DOOR_YIELD_RADIUS = 1.5     # an exiting person holds back this far from someone waiting at the gap
DOOR_YIELD_MAX_WAIT = 25.0  # ...for at most this long, so nobody can deadlock the doorway


def in_door_zone(x, y):
    return x >= DOOR_ZONE_MIN_X and DOOR_ZONE_Y_RANGE[0] <= y <= DOOR_ZONE_Y_RANGE[1]


def door_zone_busy(movers, exclude=None):
    """True while any active person other than `exclude` is in the door zone
    (or in the strip just inside the counter). Used to hold back a new
    arrival - returning resident, retrying intruder, second intruder - until
    the doorway is free instead of dropping them on top of someone."""
    for m in movers:
        if m is exclude or m.get("should_despawn"):
            continue
        mx, my = m.get("logical_x"), m.get("logical_y")
        if mx is not None and my is not None and (in_door_zone(mx, my) or (mx >= 36.5 and DOOR_ZONE_Y_RANGE[0] <= my <= DOOR_ZONE_Y_RANGE[1])):
            return True
    return False


def avoid_mover_collision(name, new_x, new_y):
    """Nudges (new_x, new_y) away from any other currently-tracked mover
    that would otherwise end up within PERSON_COLLISION_RADIUS. Not applied
    to the robot's own movement - it needs to approach and park close to
    people for scans/checkpoint duty, so avoidance there would work
    against its actual job. Not applied in the door zone either (see
    DOOR_ZONE_MIN_X)."""
    if in_door_zone(new_x, new_y):
        return new_x, new_y
    for other in _tick_state.get("movers", []):
        if other.get("name") == name or other.get("should_despawn"):
            continue
        ox, oy = other.get("logical_x"), other.get("logical_y")
        if ox is None or oy is None:
            continue
        dx, dy = new_x - ox, new_y - oy
        dist = math.hypot(dx, dy)
        if 0 < dist < PERSON_COLLISION_RADIUS:
            push = PERSON_COLLISION_RADIUS - dist
            new_x += (dx / dist) * push
            new_y += (dy / dist) * push
    return new_x, new_y


DOORWAY_WAYPOINT = (22.1, sum(DOORWAY_Y_RANGE) / 2.0)  # center of the actual gap


def get_next_hop(cur_x, cur_y, target_x, target_y, hysteresis_state=None):
    """REAL fix for the robot getting stuck pressed against the dividing
    wall: move_with_wall_check correctly BLOCKS illegal crossings, but on
    its own that just leaves the mover stuck at the wall forever if the
    final target is on the other side and the current y isn't already
    inside the doorway band - confirmed exactly in a log where the robot
    sat frozen at x=21.70 for 15+ seconds trying to reach x=39.

    This is real obstacle-avoidance routing, not just collision blocking:
    if the current position and the target are on opposite sides of the
    dividing wall AND the current y isn't already within the doorway gap,
    the immediate movement target becomes the doorway waypoint first -
    once through it, the direct line to the real target no longer needs to
    cross the wall at all.

    hysteresis_state: pass the caller's own persistent dict (a mover, or
    robot_state) to enable hysteresis on the doorway-band check - see BUG 4
    below for why a raw instantaneous check flickers. Optional/backward
    compatible: omitting it (or passing None) falls back to the old
    instantaneous check.
    """
    dxmin, dxmax = DIVIDER_X_RANGE
    cur_side = "west" if cur_x <= dxmin else ("east" if cur_x >= dxmax else "mid")
    target_side = "west" if target_x < dxmin else ("east" if target_x > dxmax else "mid")
    ymin, ymax = DOORWAY_Y_RANGE

    # BUG 4 (found live: "people are twitching when reaching the middle" -
    # confirmed by hand-tracing a real position log where a mover's y
    # hovered right around ymin=16.3, the doorway band's own edge, for
    # several consecutive ticks): the old check below was a single
    # INSTANTANEOUS boolean, ymin <= cur_y <= ymax, recomputed fresh from
    # the mover's own (naturally slightly noisy/drifting) y every single
    # tick. Right at that edge, tiny y movement flips this boolean back
    # and forth tick to tick - and each flip makes the crossing decision
    # below alternate between routing via DOORWAY_WAYPOINT (which pulls y
    # toward ~18) and aiming straight at the real target (which could pull
    # y toward a completely different value, e.g. 9 or 26) - two wildly
    # different directions, chosen freshly every tick with no memory of
    # which one was just being pursued. That's a real boundary-chatter
    # bug, and it explains an actual back-and-forth POSITION twitch, not
    # just a rendering artifact - classic fix is hysteresis: once you've
    # committed to being "in the band," require moving clearly OUT before
    # switching modes, and vice versa, instead of re-deciding from a
    # single instantaneous sample every tick.
    BAND_HYSTERESIS_MARGIN = 0.3
    if hysteresis_state is not None:
        was_in_band = hysteresis_state.get("_in_doorway_band", False)
        if was_in_band:
            in_doorway_band = (ymin - BAND_HYSTERESIS_MARGIN) <= cur_y <= (ymax + BAND_HYSTERESIS_MARGIN)
        else:
            in_doorway_band = (ymin + BAND_HYSTERESIS_MARGIN) <= cur_y <= (ymax - BAND_HYSTERESIS_MARGIN)
        hysteresis_state["_in_doorway_band"] = in_doorway_band
    else:
        in_doorway_band = ymin <= cur_y <= ymax

    # BUG 1 (found by hand-tracing a live stuck robot log): a mover pressed
    # right against the dividing wall has cur_x INSIDE DIVIDER_X_RANGE,
    # which classifies as cur_side == "mid" - the old check only routed via
    # the doorway when cur_side != "mid", so a mover already at the wall
    # (exactly the position that needs routing help) skipped it entirely,
    # aimed straight at the far-side target again, got blocked by
    # move_with_wall_check again, and crawled/oscillated in place forever.
    #
    # BUG 2 (found immediately after fixing bug 1, same live-log method):
    # being IN the doorway y-band is not enough to aim straight at a
    # far-side target. Example that actually happened: cur=(22.39,15.84)
    # (barely inside the band), target=(21.00,9.00) - a steep diagonal.
    # Aiming straight at it means y drops toward 9 almost immediately,
    # exiting the band while x is still inside the wall's thickness, so
    # the very next step gets blocked again - an oscillation trap at the
    # band's edge. The fix: while cur_side == "mid" (still literally inside
    # the wall's x-thickness), the only safe move is a PURE HORIZONTAL step
    # across the gap. Only once x has fully cleared the wall's thickness
    # (cur_side is a definite west/east) is it safe to aim diagonally at
    # the real target.
    #
    # Changed live per explicit direction ("stop making them go from the
    # very corners of the doorway, they keep hitting the corner and
    # tweaking out... make them go from the middle"): this used to hold y
    # at whatever value the mover HAPPENED to enter the gap with (clamped
    # to stay inside the band, but otherwise anywhere across its full
    # width) - a mover entering near either edge crossed near that edge,
    # right where the wall's real corner geometry is, which is also
    # exactly where the boundary-hysteresis edge case lived. Now every
    # crossing is pulled toward the doorway's actual CENTER y
    # (DOORWAY_WAYPOINT's own y) instead, so nobody ever hugs a corner.
    if cur_side == "mid":
        doorway_center_y = DOORWAY_WAYPOINT[1]
        safe_y = min(max(doorway_center_y, ymin + 0.1), ymax - 0.1)
        # Widened from 0.1 to 0.4 per explicit direction ("try your best
        # to eliminate the wall clipping... the robot still does it
        # sometimes"). Origin-point collision tracking (what
        # move_with_wall_check actually reasons about) was already exiting
        # the wall's numeric x-range by only 0.1m before continuing - fine
        # for a zero-width point, but Nova Carter's real body is ~0.73m
        # wide, so its rendered mesh could still visually clip the wall
        # geometry even while its tracked origin was technically clear.
        # More clearance here doesn't fully solve mesh-vs-point collision
        # (that would need real per-body collision, out of scope for this
        # kinematic setup), but reduces how often it's visible.
        CROSS_CLEARANCE = 0.4
        if target_side == "west":
            return (dxmin - CROSS_CLEARANCE, safe_y)
        elif target_side == "east":
            return (dxmax + CROSS_CLEARANCE, safe_y)
        return (target_x, safe_y)

    # BUG 3 (found live: the ROBOT itself got stuck at x=21.70 heading to
    # (22.00, 17.50) - one of ROBOT_WAYPOINTS, the doorway patrol point
    # itself). That target's x=22.00 falls INSIDE DIVIDER_X_RANGE, so it
    # classified as target_side == "mid" - the crossing check below used
    # to require target_side != "mid" to trigger doorway routing, so a
    # target that IS effectively the doorway got EXCLUDED from routing and
    # aimed at directly instead, the same unsafe diagonal this whole
    # function exists to prevent. Real rule: routing is needed whenever
    # reaching target_x at all requires passing through the wall's
    # x-thickness from a definite west/east side while y isn't already in
    # the band - true whether the target ends up "mid" (like the doorway
    # waypoint itself) or fully on the other side.
    needs_crossing = (
        (cur_side == "west" and target_x > dxmin) or
        (cur_side == "east" and target_x < dxmax)
    )
    if needs_crossing:
        # BUG (seen live, the last "doorway-corner crawl"): the hysteresis
        # above keeps a mover "in the band" up to BAND_HYSTERESIS_MARGIN
        # BEYOND the real gap, so someone walking at y=19.99 (gap ends at
        # 19.8) was told to aim straight at a target in the other room; the
        # straight line crosses the wall at y>19.8, move_with_wall_check
        # blocked it, and they crawled along the wall corner. Being "in the
        # band" is only permission to aim straight if that straight line
        # would actually pass through the gap - test that directly.
        margin = 0.1
        span = target_x - cur_x
        if span != 0:
            y_at_wall_near = cur_y + ((dxmin if cur_side == "west" else dxmax) - cur_x) / span * (target_y - cur_y)
            y_at_wall_far = cur_y + ((dxmax if cur_side == "west" else dxmin) - cur_x) / span * (target_y - cur_y)
            line_clear = all(ymin + margin <= y <= ymax - margin for y in (y_at_wall_near, y_at_wall_far))
        else:
            line_clear = True
        if not in_doorway_band or not line_clear:
            return DOORWAY_WAYPOINT
    return (target_x, target_y)

# --- Robot dispatch ---
# The robot patrols on its own loop by default. Periodically it checks the
# event log for unhandled dispatch_alert entries (from fall/loitering/
# unauthorized-presence detections). If one exists, it drives to that
# alert's coordinates, runs a checkpoint-style face-ID scan on arrival to
# double-check the person's identity/authorization, then resumes patrol.
ROBOT_PRIM_PATH = "/World/mobile_manipulator_ros"
ROBOT_BASE_PATH = "/World/mobile_manipulator_ros/base_footprint"  # actual physics-driven prim - moving the outer group prim does nothing, PhysX drives this child directly
ROBOT_CAMERA_PATH = "/World/mobile_manipulator_ros/base_footprint/sensors/camera_front/Gemini335L/Gemini335L/camera_rgb/Camera_rgb"
ROBOT_CHECK_INTERVAL = 3.0    # seconds between checking for new dispatch alerts / arrival

# --- Real Nav2 navigation (replaces the old kinematic-teleport robot movement) ---
# The robot now moves via genuine PhysX physics, driven externally by Nav2's
# controller through the confirmed-working velocity_smoother relay bypass
# (see NOTES.md). This script no longer commands robot movement directly -
# a separate WSL-side script (dispatch_bridge.py) watches EVENT_LOG_FILE for
# dispatch_alert entries and PATROL_WAYPOINTS_FILE for the patrol loop, and
# sends real NavigateToPose goals to Nav2. This script's only job re: the
# robot is to (a) write the patrol waypoints out for the bridge to read, and
# (b) watch the robot's REAL position (driven by physics, not by us) to
# detect arrival at a dispatch location and trigger the on-arrival scan.
PATROL_WAYPOINTS_FILE = PROJECT_DIR + r"\robot_patrol_waypoints.json"
ARRIVAL_RADIUS = 3.0          # meters - bumped from 2.0: with continuous live tracking (see update_robot) this is now a real "stop a comfortable distance in front of them" radius, not just a tolerance for a static point

# --- Active face search on arrival (replaces the old single-static-
# snapshot scan) ---
# On arrival, the robot no longer just takes one picture from whatever
# direction it happened to stop facing. It sweeps through a series of
# headings AT THE SAME LOCATION (via small yaw-only Nav2 sub-goals - see
# dispatch_bridge.py's FACE_SEARCH_GOAL_FILE handling), running a full
# YOLO -> crop -> DeepFace attempt at each heading, and stops as soon as
# one attempt gets a usable face. This is what makes the robot actually
# "look for" the person instead of hoping they're in frame by luck.
FACE_SEARCH_GOAL_FILE = PROJECT_DIR + r"\robot_face_search_goal.json"
FACE_SEARCH_YAW_STEPS = 8              # full circle in 45-degree increments
FACE_SEARCH_YAW_TOLERANCE_DEG = 8.0    # how close to the target heading counts as "turned"
FACE_SEARCH_YAW_WAIT_TIMEOUT = 20.0    # seconds to wait for a single turn before giving up on it

# Robot patrols the actual room corners/doorway you gave directly, looping
# west room -> doorway -> east room -> doorway.
ROBOT_WAYPOINTS = [
    (5, 9, 0),
    (6, 27, 0),
    (21.5, 27, 0),
    (22, 17.5, 0),   # doorway
    (21, 9, 0),
    # BUG (this was a genuine root cause of the robot repeatedly getting
    # "stuck near the wall" all session, not just a numerical routing
    # edge case): the old list went straight from (21,9) to (22,9) to
    # (39,9), assuming free passage along y=9 - but the dividing wall's
    # real solid segments span y:[8.12,16.12] and y:[20.12,28.12], with
    # the only gap at y:[16.12,20.12]. (22,9) sits almost exactly INSIDE
    # the solid wall itself, nowhere near the doorway - no amount of
    # routing logic can safely reach a point embedded in the wall, so the
    # robot got stuck trying every time this leg came up in patrol. Fixed
    # by re-crossing through the real doorway waypoint before heading to
    # the east side's south corner, instead of pretending y=9 has a gap.
    (22, 17.5, 0),   # doorway - the ONLY real crossing point, re-used here before heading east at the south end
    (39, 9, 0),
    (39, 26, 0),
    (22, 17.5, 0),   # doorway, loop back
]

# --- SIMPLIFIED KINEMATIC ROBOT MOVEMENT (replaces real Nav2/PhysX driving) ---
# Real wheel physics through Nav2 turned out not to be worth the cost for
# this project's actual goal: the robot reliably reaching alert locations
# so its camera can run a face search, not accurate differential-drive
# kinematics. Diagnosed (not guessed) that the wheel geometry itself was
# fine (measured wheelDistance matched the authored 0.34m exactly) - the
# real problem was the whole Nav2/PhysX/ROS2 stack being fragile, slow to
# debug, and sensitive to render-rate/render-event assumptions that kept
# eating entire sessions. The robot now moves exactly like the patrol
# people already do (see update_person_position) - direct kinematic
# translation toward a target each tick, no physics, no wheels, no Nav2,
# no WSL dependency for movement at all. Nav2/dispatch_bridge.py can still
# run harmlessly alongside this (it's just not doing anything useful for
# the robot anymore) or you can stop bothering with start_nav_stack.sh
# entirely - movement no longer needs it.
# Stuck/oscillation safety net thresholds - shared by ambient wander,
# scripted loiter, and (separately, see update_robot) the robot's own
# movement. Tightened live (reported: "the guys are still twitching when
# reaching the middle... put in some detection so you can fix it") from an
# earlier, much more lenient 4.0s/0.5m pair that wasn't reacting fast
# enough to be visually acceptable - this fires in under 2 real seconds
# of failing to make even slow-walk-speed progress, so any oscillation or
# stall gets caught and reset almost immediately regardless of the exact
# underlying mechanism, rather than needing that mechanism fully
# root-caused first.
MOVER_STUCK_CHECK_INTERVAL = 1.5
MOVER_STUCK_MIN_PROGRESS = 0.3
LOITER_STUCK_CHECK_INTERVAL = MOVER_STUCK_CHECK_INTERVAL
LOITER_STUCK_MIN_PROGRESS = MOVER_STUCK_MIN_PROGRESS

ROBOT_MOVE_SPEED = 3.0  # halved (was 6.0) per request
MAX_MOVE_DT = 0.5  # caps a single movement tick's dt (real elapsed seconds since last move) for people AND the robot - confirmed via direct observation (a real run showed "moved=24.988m" and "moved=7.564m" in a SINGLE tick) that whenever a mover is paused for a while (e.g. every OTHER mover gets frozen for up to 300s during the robot's own orbit search) and then resumes, the accumulated real-time gap turns the very next movement step into a huge instant jump covering the whole gap's distance at once - looks exactly like teleporting/gliding rather than walking. Capping dt means a long pause is simply picked back up at normal walking pace from wherever they stopped, instead of catching up all at once.
ROBOT_CAMERA_HEIGHT = 1.55  # must match create_lightweight_robot()'s CAMERA_HEIGHT - hoisted to a module constant so search_for_face() can compute a correct downward pitch for fallen (near-floor) targets without duplicating/hardcoding the value

# NOTE (known limitation, documented rather than solved - see NOTES.md):
# wheel_left/wheel_right/camera_hand_link are SIBLINGS of base_footprint
# (not children), not connected to it by a physics joint. In the old
# kinematic-teleport design we moved them manually every frame to fake
# attachment. In real-physics Nav2 mode, base_footprint now moves under
# genuine PhysX + the robot's own ROS2 diff-drive plugin (confirmed working
# in the Nav2 relay test), but these sibling parts have no joint tying them
# to it, so they may visually lag/stay behind during navigation. Cosmetic
# only - it doesn't affect navigation, detection, or the checkpoint-style
# scan, since ROBOT_CAMERA_PATH is the properly-nested camera under
# base_footprint/sensors/ and moves correctly with it automatically.

# Isaac Sim's ROS2 bridge extension used to be needed here for the old
# Nav2/PhysX-driven robot. That path was fully abandoned in favor of pure
# kinematic movement (see ROBOT_MOVE_SPEED's comment history) and the heavy
# mobile_manipulator_ros asset itself was replaced with a lightweight
# custom robot (see create_lightweight_robot) - ROS2 isn't driving
# anything anymore, so enabling that whole extension was pure unnecessary
# startup/runtime cost. Removed rather than left harmlessly enabled.
import os

from isaacsim import SimulationApp

# All of the settings below are passed directly into SimulationApp's launch
# config instead of being set afterward via carb.settings.set(). This is a
# deliberate change: post-hoc carb.settings calls (even moved as early as
# possible in the script, even before enabling the ROS2 bridge extension)
# were confirmed NOT to suppress the PoseTree log spam or fix the
# "sequence size exceeds remaining buffer" issue across two separate
# attempts. Extension logging channels and some renderer internals are
# locked in at Kit's actual process startup - passing these as launch
# config is the earliest possible point, before any extension registers,
# and is far more likely to actually take effect.
simulation_app = SimulationApp({
    "headless": HEADLESS,
    # Dropped from 640x480 - this is the internal offscreen render surface
    # size, paid on every rendered tick regardless of headless mode. You
    # never see this buffer directly in headless mode, so there's no
    # visual-quality reason to keep it large - only the actual camera
    # capture resolutions (set separately per-camera in capture_frame()
    # calls) matter for what YOLO/DeepFace actually see.
    "width": 854,
    "height": 480,
    "/log/level": "Error",
    "/log/fileLogLevel": "Error",
    "/log/outputStreamLevel": "Error",
    "/rtx/scenedb/maxHistoryTransformCount": 2048,  # bumped from 512 - that value stopped the spam in an earlier session but it's back in force (see run this session, heavy "sequence size exceeds remaining buffer" volume even before any camera/sweep activity), so whatever's generating transform-history entries now needs more headroom than 512 gave it
    "/rtx/pathtracing/enabled": False,
    "/rtx/reflections/enabled": False,
    "/rtx/ambientOcclusion/enabled": False,
    "/rtx/indirectDiffuseGI/enabled": False,
    "/rtx/directLighting/sampledLighting/enabled": False,
    "/rtx/post/dlss/execMode": 0,
    # Real, concrete render-cost cuts on top of the above - shadows and
    # antialiasing are both real per-frame GPU cost with zero benefit in
    # headless mode where nothing is being visually watched, and neither
    # affects what a captured camera frame looks like to YOLO/DeepFace
    # (object detection/face-matching don't care about shadow softness or
    # jagged edges).
    "/rtx/shadows/enabled": False,
    "/rtx/post/aa/op": 0,
    "/app/runLoops/main/rateLimitEnabled": False,
})

import carb
# Per-channel log suppression via flat "/log/channels/<name>" = "<level>"
# string entries - this IS Kit's real schema (confirmed by the previous
# attempt's error: setting "/level"/"/enabled" sub-keys under each channel
# turned that entry into a nested dict, and Kit's own startup code, which
# expects to read each channel entry as a plain string, errored on every
# single one - "getStringRawInternal: item <channel> is not a string" -
# which broke far more than it fixed). No sub-keys this time.
for _channel in ("isaacsim.ros2.nodes", "isaacsim.ros2.bridge", "isaacsim.ros2.core",
                  "omni.hydra", "rtx.hydra", "omni.syntheticdata.plugin",
                  "isaacsim.sensors.camera.camera", "rtx.scenedb.plugin",
                  "isaacsim.ros2.core.impl.camera_info_utils", "omni.timeline.plugin",
                  "carb"):
    try:
        # Reverted back to "Error" (from an attempted "Fatal" bump) - that
        # bump did NOT fix the "sequence size exceeds remaining buffer"
        # spam (confirming it isn't going through carb's channel logging at
        # all - likely a raw fprintf from a native plugin, which no log-
        # level setting can touch), while ALSO suppressing other genuinely
        # useful console output. The actual fix for that specific message
        # is run_sim_filtered.bat, which filters the real text stream
        # instead of fighting the logging system further.
        carb.settings.get_settings().set(f"/log/channels/{_channel}", "Error")
    except Exception:
        pass

# ROS2 bridge extension enabling removed - no longer needed, see the
# comment above `import os` for why.

# EXPERIMENTAL: attempt to force the viewport's "RTX - Minimal" render mode
# (confirmed present/selectable in the viewport dropdown, not guaranteed to
# be settable via this exact key). Post-hoc carb.settings calls were
# already confirmed NOT to reliably affect renderer internals earlier in
# this project (see the log-channel-suppression comment above), so this is
# a low-risk try: if the key/value is wrong it should just be silently
# ignored (same as the earlier "Storm" attempt was), not break startup.
# If the dropdown doesn't show Minimal selected after launch, this didn't
# work - just click it manually in the dropdown, which is the confirmed-
# working zero-risk fallback.
if USE_MINIMAL_RENDERER:
    try:
        carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLighting")
        carb.settings.get_settings().set("/rtx-transient/dlssg/enabled", False)
        print("Attempted to force RTX-Minimal render mode (experimental - verify in the viewport dropdown).")
    except Exception as e:
        print(f"RTX-Minimal render mode attempt failed harmlessly: {e}")

import omni.usd
import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdSkel, UsdShade, Sdf
import cv2
import numpy as np
import subprocess
import json
import shutil
import time
import random
import math
from datetime import datetime, timezone
from isaacsim.sensors.camera import Camera
from isaacsim.core.prims import RigidPrim
from isaacsim.storage.native import get_assets_root_path

# THE actual max-speed lever: SimulationContext.step(render=...) lets us
# step physics/OmniGraph (ROS2 publishers/subscribers, TF, odom, /cmd_vel
# consumption - all of it) every tick while only paying for a full Hydra
# render on a fraction of those ticks. Rendering, not physics, is what was
# capping real-time factor at ~12 updates/sec even with ZERO cameras
# active (confirmed via the [PERF] log - that ceiling existed before the
# first camera was ever created) - simulation_app.update() couples a full
# render to every single tick with no way to separate them. This is the
# standard, documented Isaac Sim pattern for headless/bulk-physics speed;
# not a guess, but also not yet verified on THIS build/robot asset, so it's
# wrapped in the same fail-safe try/except pattern as every other
# uncertain API call in this file - if SimulationContext or .step(render=)
# doesn't behave as expected here, this falls back to the old always-render
# simulation_app.update() behavior rather than silently breaking physics,
# ROS2, or movement.
try:
    from isaacsim.core.api import SimulationContext
except ImportError:
    from isaacsim.core.simulation_context import SimulationContext

# Render only 1 in this many ticks when nothing needs a guaranteed real
# frame this tick (see _tick()'s force_render param, used by capture_frame
# to always get a real frame when actually capturing). Physics/OmniGraph/
# ROS2 still step on EVERY tick regardless - only the expensive Hydra
# render pass gets skipped on the other (RENDER_EVERY_N_TICKS - 1) ticks.
#
# Tried 20: steps/sec jumped to 170-290/sec, but the robot never moved at
# all that run - turned out the WSL nav stack (start_nav_stack.sh) wasn't
# even running that time, so there were no Nav2 goals being sent at ANY
# render setting - that test didn't actually tell us anything about this
# number, my earlier comment claiming it as "confirmed" broken was wrong.
# Back at 6 (previously confirmed moving the robot end to end WITH the nav
# stack running) as the safe known-good value. Real test of going higher
# than 6 has to be run with start_nav_stack.sh actually up in WSL first -
# check that before touching this number again.
RENDER_EVERY_N_TICKS = 6
_sim_context = None
_render_decouple_supported = True


# ---------- Movement helpers ----------

def repair_broken_tf_publisher_targets(stage):
    """The robot asset has long-known broken TF frames (base_link,
    lidar_frame, etc. reporting '[PoseTree] getObjectType eInvalid' every
    tick - see NOTES.md). This was previously treated as harmless log noise,
    but it likely also means Nav2's obstacle-avoidance costmap can't
    correctly place lidar points (it needs a working transform from each
    scan's frame to the costmap's frame) - consistent with the robot
    driving straight into a pillar despite genuine navigation working.

    Rather than hand-guessing which exact prim paths are broken and what
    they should be instead (risky and asset-specific), this walks every
    USD relationship stage-wide looking for 'target'/'parent'-type
    relationships (how OmniGraph nodes like the ROS2 TF-tree publisher
    store their prim references under the hood) whose targets don't
    resolve to a valid prim, and tries to repair each one by finding a
    valid prim elsewhere in the robot's hierarchy with the exact same leaf
    name. Self-healing where the match is unambiguous; loudly prints
    exactly what it could and couldn't fix otherwise, rather than silently
    guessing.
    """
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        print("  [tf_repair] robot prim not found, skipping.")
        return

    name_to_paths = {}
    for prim in Usd.PrimRange(robot_prim):
        name_to_paths.setdefault(prim.GetName(), []).append(prim.GetPath())

    checked = 0
    repaired = 0
    unresolved = []

    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        for rel in prim.GetRelationships():
            rel_name = rel.GetName()
            if "target" not in rel_name.lower() and "parent" not in rel_name.lower():
                continue
            targets = rel.GetTargets()
            if not targets:
                continue
            new_targets = []
            changed = False
            for t in targets:
                checked += 1
                t_prim = stage.GetPrimAtPath(t)
                if t_prim.IsValid():
                    new_targets.append(t)
                    continue
                leaf = t.name
                candidates = name_to_paths.get(leaf, [])
                if len(candidates) == 1:
                    fixed = candidates[0]
                    print(f"  [tf_repair] {prim.GetPath()}.{rel_name}: {t} -> {fixed}")
                    new_targets.append(fixed)
                    changed = True
                    repaired += 1
                else:
                    unresolved.append((str(prim.GetPath()), rel_name, str(t), len(candidates)))
                    new_targets.append(t)
            if changed:
                try:
                    rel.SetTargets(new_targets)
                except Exception as e:
                    print(f"  [tf_repair] failed to apply fix on {prim.GetPath()}.{rel_name}: {e}")

    print(f"  [tf_repair] checked {checked} relationship target(s) stage-wide, repaired {repaired}.")
    if unresolved:
        print(f"  [tf_repair] {len(unresolved)} target(s) NOT auto-repaired (ambiguous or no same-name match):")
        for node_path, rel_name, t_str, n in unresolved[:15]:
            print(f"    {node_path}.{rel_name}: {t_str} (found {n} same-name candidates)")


def get_animation_loop_seconds(stage, default=1.0):
    """Scans all UsdSkel.Animation prims stage-wide for the furthest real
    time sample (rotation/translation) and converts it from timeCodes to
    seconds using the stage's own timeCodesPerSecond. Used to tell the
    timeline exactly where the baked Mixamo walk clip ends, so looping
    wraps back to frame 0 right as the clip finishes instead of guessing
    an fps/duration and holding on the last frame or looping mid-stride.
    """
    from pxr import UsdSkel as _UsdSkel
    max_time_code = 0.0
    found_any = False
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(_UsdSkel.Animation):
            continue
        # BUG (people slid along frozen mid-stride): this used to take the longest
        # clip of ANY kind. Once the idle clips existed, the 38.8 s phone-call clip
        # set the shared loop to 38.8 s, so the 3.0 s walk cycle played once and then
        # USD held its final frame for the other ~36 s of every loop. The walk cycle
        # is what plays continuously, so it alone sets the loop; the other clips
        # (falls, get-up, idles) are all played from the start and just get cut at
        # the loop point, which is fine for short poses and stances.
        if prim.GetName() != "mixamo_com":
            continue
        anim = _UsdSkel.Animation(prim)
        for attr in (anim.GetRotationsAttr(), anim.GetTranslationsAttr()):
            if not attr:
                continue
            samples = attr.GetTimeSamples()
            if samples:
                found_any = True
                max_time_code = max(max_time_code, samples[-1])
    if not found_any or max_time_code <= 0:
        return default
    tcps = stage.GetTimeCodesPerSecond() or 24.0
    return max_time_code / tcps


def get_translate(prim):
    # NOTE: this used to just look for a literal `translate` xformOp and
    # return (0,0,0) if none was found - fine while the robot was
    # kinematic-teleported (we created that op ourselves), but PhysX writes
    # physics-driven position updates through a different representation
    # (a combined transform matrix), so that approach silently always
    # returned (0,0,0) for the real-physics robot and update_robot() never
    # detected arrival no matter how close it actually got. Computing the
    # full local-to-world transform works regardless of how the underlying
    # ops are structured.
    xform = UsdGeom.Xformable(prim)
    matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return matrix.ExtractTranslation()


def set_translate(prim, pos):
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
            return
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))


def set_world_translate(prim, world_pos):
    """Sets a prim's position given a WORLD-space target, correctly
    accounting for its parent's transform - unlike set_translate() (which
    writes the prim's LOCAL translate op directly), this actually converts
    world_pos into the right local value first via the parent's inverse
    world transform.

    Needed specifically for the robot: get_translate() reads WORLD
    position (via ComputeLocalToWorldTransform), but the patrol people's
    set_translate() calls write LOCAL translate directly - harmless for
    them because their prims sit right under /World with an identity
    parent transform, so local == world. The robot's parent prim
    (/World/mobile_manipulator_ros) does NOT have an identity transform
    (almost certainly a scale factor from the URDF/USD import) - writing a
    world-sized delta straight into local space under a scaled parent
    caused the position to blow up exponentially every tick (confirmed:
    the robot flew off to tens of thousands of meters away, accelerating).
    This is the real fix - always use this for the robot, never raw
    set_translate().
    """
    parent = prim.GetParent()
    parent_xform = UsdGeom.Xformable(parent)
    parent_matrix = parent_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    local_pos = parent_matrix.GetInverse().Transform(Gf.Vec3d(*world_pos))
    set_translate(prim, local_pos)


def set_rotation(prim, degrees_xyz):
    """Sets (or creates) a rotateXYZ xformOp on this prim, in degrees.
    Used to physically collapse a character for the scripted fall event -
    rotating the whole standing character 90 degrees about a horizontal
    axis makes their bounding box genuinely go wide/short (matching what
    check_for_fall()'s aspect-ratio heuristic is actually looking for),
    instead of that heuristic only ever firing on incidental noise."""
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            op.Set(Gf.Vec3f(*degrees_xyz))
            return
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*degrees_xyz))


def get_model_offset(model_prim):
    """Reads the /model child prim's OWN local translate xformOp (x, y
    only) directly off the asset, instead of hardcoding measured numbers.
    Confirmed via diagnose_full_person_chain.py: each xbot_person_N/model
    prim carries its own static per-character translate (5.93 / 7.60 /
    8.24 on X for the three current characters) baked in at scene-
    integration time to align that asset's arbitrary internal pivot with
    its parent Xform - unified_tracking.py only ever moves the parent
    (mover["prim"]), so without compensating for this, any code that sets
    a specific target (x, y) - fall teleport, loiter waypoints, spawn -
    places the actual rendered mesh several meters away from the intended
    zone coordinate. Rotates with the parent's own heading (it's a child
    prim under mover["prim"]), so it must be re-rotated by the parent's
    current heading every time it's applied, not just subtracted flatly."""
    if model_prim is None or not model_prim.IsValid():
        return (0.0, 0.0)
    xform = UsdGeom.Xformable(model_prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            val = op.Get()
            return (val[0], val[1])
    return (0.0, 0.0)


def get_heading_deg(prim):
    """Current Z heading of a person prim, in degrees. Checks both
    xformOp:rotateXYZ (what set_rotation() authors) and xformOp:rotateZ
    (what these prims are authored with in the scene file) since
    set_rotation() only recognizes TypeRotateXYZ and will add a NEW
    rotateXYZ op alongside an existing rotateZ op rather than reusing it -
    the rotateZ op has always stayed at 0 in practice (nothing else in
    this file ever sets it), so whichever op actually carries a nonzero
    value is the real current heading."""
    xform = UsdGeom.Xformable(prim)
    rotate_z_val = 0.0
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            val = op.Get()
            return float(val[2])
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
            rotate_z_val = float(op.Get())
    return rotate_z_val


def set_person_translate(mover, x, y, z):
    """Like set_translate(mover["prim"], (x, y, z)), but first corrects
    for the mover's /model child prim's own static offset (see
    get_model_offset()) so the ACTUAL rendered mesh ends up at (x, y) -
    not mover["prim"] itself. The offset is authored in mover["prim"]'s
    own local frame, so it rotates along with mover["prim"]'s current
    heading and must be rotated back before subtracting.

    Also stores (x, y) as mover["logical_x"/"logical_y"] - the TRUE
    intended position. mover["prim"]'s own translate is no longer the
    same thing once this function is used (it's deliberately offset so
    the MESH ends up at (x, y), not the prim itself) - a real bug found
    live (see NOTES.md, "THIRD FOLLOW-UP"): get_nearest_mover_coords()
    and the loiter-waypoint distance/heading calculation were both still
    reading get_translate(mover["prim"]) directly and treating it as the
    real position, which sent the robot's whole orbit search to a point
    several meters from where the person actually was (matching the
    washed-out, nothing-in-view frames observed at every standpoint).
    Anything that needs "where is this person really" must use
    logical_x/logical_y, not get_translate(mover["prim"])."""
    mover["logical_x"], mover["logical_y"] = x, y
    ox, oy = mover.get("model_offset", (0.0, 0.0))
    if ox == 0.0 and oy == 0.0:
        set_translate(mover["prim"], (x, y, z))
        return
    theta = math.radians(get_heading_deg(mover["prim"]))
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    rot_ox = ox * cos_t - oy * sin_t
    rot_oy = ox * sin_t + oy * cos_t
    set_translate(mover["prim"], (x - rot_ox, y - rot_oy, z))


def get_skeleton_and_clips(model_prim):
    """Finds the UsdSkel.Skeleton under this character's model prim, plus
    the sibling walk/fall/held-fall/get-up SkelAnimation clip paths next to
    it, so the fall event can retarget animationSource between them at
    runtime instead of faking a collapse via crude whole-body rotation (see
    the old set_rotation-based fall, replaced below - that just rigidly
    rotated a mid-stride walk pose, which looked exactly as wrong as it
    sounds).

    Also computes the fall clip's and get-up clip's own real durations in
    seconds (from each one's last authored time sample /
    timeCodesPerSecond), so update_person_position can switch clips right
    when each animation finishes playing - without this, the whole scene's
    shared global timeline loop (much shorter than the ~45s lie-down pause)
    would keep wrapping back to frame 0 and replaying from the start over
    and over during the pause/stand-up.

    get_up_path/get_up_duration are None/0.0 if this character's USD
    doesn't have a "mixamo_get_up" clip (e.g. bind_get_up_animation.py
    hasn't been run against it, or the joint remap was skipped for a
    mismatch) - callers must treat that as "no get-up clip available" and
    fall back to the old instant walk-clip switch, not assume it's there.
    """
    if not model_prim.IsValid():
        return None, None, None, None, 2.0, None, 0.0
    stage = model_prim.GetStage()
    tcps = stage.GetTimeCodesPerSecond() or 24.0
    for prim in Usd.PrimRange(model_prim):
        if prim.IsA(UsdSkel.Skeleton):
            skel_root = prim.GetParent()
            walk_path = skel_root.GetPath().AppendChild("mixamo_com")
            fall_path = skel_root.GetPath().AppendChild("mixamo_fall")
            held_path = skel_root.GetPath().AppendChild("mixamo_fall_held")
            get_up_path = skel_root.GetPath().AppendChild("mixamo_get_up")
            fall_duration = 2.0
            fall_prim = stage.GetPrimAtPath(fall_path)
            if fall_prim.IsValid():
                fall_anim = UsdSkel.Animation(fall_prim)
                rot_attr = fall_anim.GetRotationsAttr()
                samples = rot_attr.GetTimeSamples() if rot_attr else []
                if samples:
                    fall_duration = samples[-1] / tcps
            get_up_duration = 0.0
            get_up_prim = stage.GetPrimAtPath(get_up_path)
            if get_up_prim.IsValid():
                get_up_anim = UsdSkel.Animation(get_up_prim)
                rot_attr = get_up_anim.GetRotationsAttr()
                samples = rot_attr.GetTimeSamples() if rot_attr else []
                if samples:
                    get_up_duration = samples[-1] / tcps
            else:
                get_up_path = None
            return prim, walk_path, fall_path, held_path, fall_duration, get_up_path, get_up_duration
    return None, None, None, None, 2.0, None, 0.0


IDLE_CLIP_NAMES = ["mixamo_idle_stand", "mixamo_idle_salute", "mixamo_idle_excited", "mixamo_idle_phone"]


def get_named_clip(model_prim, clip_name):
    """A single named SkelAnimation clip's path for this character, or None if
    the pipeline script that binds it (e.g. bind_denial_reaction_clips.py)
    hasn't been run against it yet - callers must treat that as "capability
    absent" and fall back gracefully, same convention as the get-up clip's
    own None handling."""
    if not model_prim.IsValid():
        return None
    stage = model_prim.GetStage()
    for prim in Usd.PrimRange(model_prim):
        if prim.IsA(UsdSkel.Skeleton):
            clip_path = prim.GetParent().GetPath().AppendChild(clip_name)
            return clip_path if stage.GetPrimAtPath(clip_path).IsValid() else None
    return None


def get_idle_clips(model_prim):
    """Companion to get_skeleton_and_clips() for the new idle/social clips
    (see bind_idle_animations.py) - kept as a separate small helper rather
    than folded into that function's already-large return tuple, since
    this is a variable-length LIST (however many of the 4 idle clips this
    character actually has bound, which may be none if
    bind_idle_animations.py hasn't been run against it yet) rather than a
    fixed set of named single clips.

    Returns a list of (clip_path, duration_seconds) - used by
    update_person_position's idle-pause behavior to pick a random one and
    know how long to hold it before resuming ambient wander."""
    if not model_prim.IsValid():
        return []
    stage = model_prim.GetStage()
    tcps = stage.GetTimeCodesPerSecond() or 24.0
    for prim in Usd.PrimRange(model_prim):
        if prim.IsA(UsdSkel.Skeleton):
            skel_root = prim.GetParent()
            clips = []
            for clip_name in IDLE_CLIP_NAMES:
                clip_path = skel_root.GetPath().AppendChild(clip_name)
                clip_prim = stage.GetPrimAtPath(clip_path)
                if not clip_prim.IsValid():
                    continue
                anim = UsdSkel.Animation(clip_prim)
                rot_attr = anim.GetRotationsAttr()
                samples = rot_attr.GetTimeSamples() if rot_attr else []
                duration = samples[-1] / tcps if samples else 2.0
                clips.append((clip_path, duration))
            return clips
    return []


def set_animation_clip(skel_prim, clip_path):
    """Retargets a Skeleton's animationSource relationship to a different
    SkelAnimation clip prim (walk <-> fall). Both clips already live side
    by side under the same SkelRoot (see bind_fall_animation.py) - this
    just repoints which one is currently bound/playing."""
    if skel_prim is None or not skel_prim.IsValid():
        return
    binding = UsdSkel.BindingAPI(skel_prim)
    rel = binding.GetAnimationSourceRel()
    if not rel:
        rel = binding.CreateAnimationSourceRel()
    rel.SetTargets([clip_path])


def disable_physics_recursive(prim):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_api = UsdPhysics.RigidBodyAPI(prim)
        attr = rigid_api.GetRigidBodyEnabledAttr()
        if attr:
            attr.Set(False)
        else:
            rigid_api.CreateRigidBodyEnabledAttr(False)
    for child in prim.GetChildren():
        disable_physics_recursive(child)


# Module-level state for _tick(), the single shared per-frame update function.
# Populated once in main() right before the main loop starts. Needed because
# _tick() must be callable from deep inside blocking helpers (run_subprocess,
# capture_frame) that don't otherwise have access to movers/robot_state/etc,
# without threading those as parameters through every call site.
_tick_state = {
    "sim_start": None,
    "movers": [],
    "robot_state": None,
    "authorized_ids_cache": [],
    "banned_ids_cache": [],
    # FPS/real-time-factor instrumentation - see _tick()'s comment below.
    "tick_count": 0,
    "tick_window_start": None,
}


def _tick(force_render=False):
    """The ONE place per-frame work happens: advances mover positions,
    checks robot arrival, and steps the sim. Every single simulation step
    anywhere in this script - whether in the main loop or buried inside a
    blocking wait in run_subprocess()/capture_frame() - goes through this
    function instead, so movement never silently freezes during the
    many-second blocking calls a sweep makes (YOLO/DeepFace subprocesses,
    camera render waits). See run_subprocess() for the full story on why
    this was necessary (the "teleporting" bug).

    force_render=True guarantees a real Hydra render this tick (used by
    capture_frame(), which actually needs a frame back) - every other
    caller lets the render/no-render decision fall through to the
    RENDER_EVERY_N_TICKS throttle below, since physics/robot-arrival-
    checking/ROS2 don't need a picture, just a physics step.
    """
    global _render_decouple_supported
    if _tick_state["sim_start"] is None:
        simulation_app.update()
        return
    elapsed = time.time() - _tick_state["sim_start"]
    for m in _tick_state["movers"]:
        update_person_position(m, elapsed)
    if _tick_state["robot_state"] is not None:
        update_robot(_tick_state["robot_state"], elapsed, _tick_state["authorized_ids_cache"], _tick_state["banned_ids_cache"])

    do_render = force_render or (_tick_state["tick_count"] % RENDER_EVERY_N_TICKS == 0)
    if _render_decouple_supported and _sim_context is not None:
        try:
            _sim_context.step(render=do_render)
        except Exception as e:
            _render_decouple_supported = False
            print(f"  [WARN] SimulationContext.step(render=) failed ({e}) - "
                  f"falling back to always-render simulation_app.update() for the rest of this run.")
            simulation_app.update()
    else:
        simulation_app.update()

    # FPS/real-time-factor instrumentation - diagnosing whether robot
    # "slowness" is actually a rendering-bottleneck problem (few real
    # sim steps per real second, meaning little simulated time elapses per
    # real second regardless of how fast Nav2/the differential controller
    # are CONFIGURED to move things) rather than a navigation config
    # problem - every navigation speed knob tuned so far had zero
    # measurable effect on real-world robot speed, which is what this is
    # checking for. Now also reports how many of those ticks actually
    # rendered, so it's obvious whether the render/physics decoupling
    # above is doing anything.
    _tick_state["tick_count"] += 1
    if do_render:
        _tick_state["render_count"] = _tick_state.get("render_count", 0) + 1
    if _tick_state["tick_window_start"] is None:
        _tick_state["tick_window_start"] = time.time()
    window_elapsed = time.time() - _tick_state["tick_window_start"]
    if window_elapsed >= 5.0:
        fps = _tick_state["tick_count"] / window_elapsed
        render_fps = _tick_state.get("render_count", 0) / window_elapsed
        print(f"  [PERF] {_tick_state['tick_count']} sim steps ({_tick_state.get('render_count', 0)} rendered) "
              f"in {window_elapsed:.1f}s real time = {fps:.1f} steps/sec, {render_fps:.1f} rendered/sec")
        _tick_state["tick_count"] = 0
        _tick_state["render_count"] = 0
        _tick_state["tick_window_start"] = time.time()


def find_stand_clip(model_prim):
    """Path of this character's standing-idle clip, or None if it has none."""
    if not model_prim.IsValid():
        return None
    stage = model_prim.GetStage()
    for prim in Usd.PrimRange(model_prim):
        if prim.IsA(UsdSkel.Skeleton):
            clip_path = prim.GetParent().GetPath().AppendChild("mixamo_idle_stand")
            return clip_path if stage.GetPrimAtPath(clip_path).IsValid() else None
    return None


def get_current_clip(skel_prim):
    rel = UsdSkel.BindingAPI(skel_prim).GetAnimationSourceRel()
    targets = rel.GetTargets() if rel else []
    return str(targets[0]) if targets else None


LOCOMOTION_SYNC_INTERVAL = 0.3   # seconds between checks
LOCOMOTION_MOVING_SPEED = 0.25   # m/s above which someone counts as walking


def sync_locomotion_clip(mover, elapsed):
    """Keeps a person's animation honest about what they are doing.

    Two ways it used to look wrong: someone held still (robot injury/search
    hold, waiting at the door for check-in, yielding in the doorway) kept
    playing the walk cycle - walking on the spot - and anyone who ended up
    moving under some other clip slid along without their legs moving. So:
    moving => walk clip; deliberately held still => standing idle clip.
    Falls, get-ups and scripted idle poses manage their own clips and are
    left alone."""
    skel, walk = mover.get("skel_prim"), mover.get("walk_clip_path")
    if skel is None or walk is None or not skel.IsValid():
        return
    x, y = mover.get("logical_x"), mover.get("logical_y")
    if x is None or y is None:
        return
    if (mover.get("fallen") or mover.get("standing_up") or mover.get("idling")
            or mover.get("pending_denied_exit") or elapsed < mover.get("denial_reaction_until", 0)):
        mover["sync_t"], mover["sync_pos"] = elapsed, (x, y)
        return
    last_t, last_pos = mover.get("sync_t"), mover.get("sync_pos")
    if last_t is None:
        mover["sync_t"], mover["sync_pos"] = elapsed, (x, y)
        return
    if elapsed - last_t < LOCOMOTION_SYNC_INTERVAL:
        return
    speed = math.hypot(x - last_pos[0], y - last_pos[1]) / max(elapsed - last_t, 1e-6)
    mover["sync_t"], mover["sync_pos"] = elapsed, (x, y)

    held = (elapsed < mover.get("paused_until", 0)
            or (mover.get("is_visitor") and mover.get("visitor_state") == "entering"
                and not mover.get("checked_in") and mover.get("wander_waypoint") is None)
            or (mover.get("door_hold_since") is not None))
    current = get_current_clip(skel)
    if speed > LOCOMOTION_MOVING_SPEED:
        if current != str(walk):
            # Coming out of a hold (standing clip -> walking) is the normal case; only
            # any OTHER clip under a moving person is a real mismatch worth reporting.
            if current != str(mover.get("stand_clip_path")) and elapsed - mover.get("last_anim_warn", -99) > 5:
                mover["last_anim_warn"] = elapsed
                print(f"  [ANIM] {mover['name']} was moving ({speed:.2f} m/s) without the walk clip "
                      f"({str(current).rsplit('/', 1)[-1]}) - restoring it")
            set_animation_clip(skel, walk)
    elif held and speed < 0.05:
        stand = mover.get("stand_clip_path")
        if stand is None:
            stand = find_stand_clip(mover["model_prim"]) or False
            mover["stand_clip_path"] = stand
        if stand and current != str(stand):
            set_animation_clip(skel, stand)


def update_person_position(mover, elapsed):
    """Per explicit direction, everyone is visible and wandering
    continuously now, not idle/invisible between scripted events - a
    demo where people just stood invisible until their one moment felt
    empty. Priority order each tick: (1) stand back up if a scripted
    fall's pause has elapsed, (2) trigger a scripted fall if this is the
    moment, (3) follow a scripted loiter window if one is active right
    now, (4) otherwise ambient-wander - a real, ongoing walk to a random
    point anywhere in the building (routed through the doorway, wall-
    collision checked), picking a fresh point on arrival, forever. Fall
    and loiter windows are independent per mover (see main()'s
    scheduling) - several people can be doing different things
    (walking, loitering, down and waiting to be helped up) at once.

    Visitors (mover["is_visitor"] True - see spawn_visitor) get a small
    state machine layered on top of the SAME ambient-wander machinery
    everyone else uses, instead of separate movement code. Each of
    "entering" and "exiting" is actually TWO legs, not one direct walk -
    real geometry at the door (two solid wall segments with a gap
    between them, same principle as the interior divider) means a
    straight line from outside to an arbitrary interior point (or back)
    can clip solid wall exactly like the original mismatched door
    coordinate did. So: "entering" first walks to DOOR_GAP_WAYPOINT (the
    real gap's center), then on arrival continues to the actual interior
    target (visitor_entry_target) before becoming "active" - normal
    wandering, exactly like anyone else, until their visit time is up.
    "exiting" mirrors this in reverse: first back to DOOR_GAP_WAYPOINT,
    then on to DOOR_POS (fully outside) before despawning.
    should_despawn gets set once they're fully outside; main()'s loop
    actually removes them - this function only flags it, since removing
    a mover mid-iteration of a list main() is looping over would be a
    real bug.
    """
    if mover.get("should_despawn"):
        return
    sync_locomotion_clip(mover, elapsed)
    if mover.get("is_visitor"):
        state = mover.get("visitor_state")
        if state == "entering" and mover.get("wander_waypoint") is None:
            # Arrived at the real door gap. Per explicit direction ("when
            # they come in through the front door I need you to make it
            # so they check in with the robot and the robot scans them
            # once to get their id") - hold here, doing nothing, until
            # update_robot's checkpoint-arrival check-in logic has scanned
            # them and set checked_in (it also decides allow vs deny -
            # a denial reverses visitor_state to "exiting" itself, so this
            # branch only needs to handle the "allowed to proceed" case).
            if not mover.get("checked_in"):
                if not mover.get("at_checkin_spot"):
                    # First arrival at the door gap: step inside, off the door
                    # frame, and wait there (see CHECKIN_WAIT_POS).
                    mover["at_checkin_spot"] = True
                    mover["wander_waypoint"] = CHECKIN_WAIT_POS
                # BUG caught before it ever shipped: leaving wander_waypoint
                # None here and just falling through was NOT "stand still
                # and wait" - the ambient-wander fallback further down in
                # this same function unconditionally hands anyone with a
                # None wander_waypoint a fresh random target anywhere in
                # the building the moment it saw one, which would have
                # immediately walked an unchecked entrant straight past
                # the checkpoint before the robot ever got a chance to
                # scan them. Returning here instead genuinely holds them
                # at the gate - nothing else in this function applies to
                # someone just standing there waiting to be checked in.
                return
            else:
                entry_target = mover.get("visitor_entry_target")
                if entry_target is not None:
                    mover["wander_waypoint"] = entry_target
                    mover["visitor_entry_target"] = None
                    mover["route_via"] = list(COUNTER_VIA_NORTH)
                else:
                    mover["visitor_state"] = "active"
                    mover["visit_end_time"] = elapsed + (1e9 if mover.get("no_voluntary_exit") else random.uniform(*VISIT_DURATION_RANGE))
                    print(f"  [VISITOR] {mover['name']} is in - will head back out around t={mover['visit_end_time']:.1f}s")
        elif state == "active" and elapsed >= mover.get("visit_end_time", float("inf")):
            mover["visitor_state"] = "exiting"
            mover["wander_waypoint"] = DOOR_GAP_WAYPOINT
            mover["visitor_exit_final"] = False
            print(f"  [VISITOR] {mover['name']} heading back to the door to leave")
        elif state == "exiting" and mover.get("wander_waypoint") is None:
            if not mover.get("visitor_exit_final"):
                mover["wander_waypoint"] = DOOR_POS
                mover["visitor_exit_final"] = True
            else:
                mover["should_despawn"] = True
                UsdGeom.Imageable(mover["prim"]).MakeInvisible()
                print(f"  [VISITOR] {mover['name']} left the premises")
                return

    # Stand back up once a scripted fall's lie-down duration has elapsed.
    # If this character has a real "get up" clip (see
    # bind_get_up_animation.py - user-supplied animation), play THAT first
    # instead of instant-cutting straight to walking, then switch to the
    # walk clip once the get-up animation itself finishes. Characters
    # without one (get_up_clip_path is None - see get_skeleton_and_clips)
    # fall back to the old instant-cut behavior so this never gets a
    # mover permanently stuck waiting on a clip that doesn't exist.
    if mover.get("fallen") and not mover.get("standing_up") and elapsed >= mover.get("paused_until", 0):
        if mover.get("get_up_clip_path") is not None:
            set_animation_clip(mover["skel_prim"], mover["get_up_clip_path"])
            mover["standing_up"] = True
            mover["stand_up_start"] = elapsed
            print(f"  [ANIM] {mover['name']} playing get-up clip ({mover.get('get_up_clip_duration', 0):.2f}s)")
        else:
            set_animation_clip(mover["skel_prim"], mover["walk_clip_path"])
            mover["fallen"] = False
            mover["fall_held_triggered"] = False
            pos = get_translate(mover["prim"])
            set_translate(mover["prim"], (pos[0], pos[1], mover.get("base_height", pos[2])))
            mover["wander_waypoint"] = None  # pick a fresh ambient-wander target once back up

    # Once the get-up animation itself has finished playing, switch to the
    # walk clip and actually clear "fallen" - this is the point recovery
    # is really complete (matches the same "clip needs to finish before
    # the state machine advances" pattern as the fall->held transition
    # right below).
    if (mover.get("standing_up") and mover.get("stand_up_start") is not None
            and elapsed >= mover["stand_up_start"] + mover.get("get_up_clip_duration", 2.0)):
        print(f"  [ANIM] {mover['name']} finished getting up, back to walking")
        set_animation_clip(mover["skel_prim"], mover["walk_clip_path"])
        mover["fallen"] = False
        mover["standing_up"] = False
        mover["fall_held_triggered"] = False
        pos = get_translate(mover["prim"])
        set_translate(mover["prim"], (pos[0], pos[1], mover.get("base_height", pos[2])))
        mover["wander_waypoint"] = None  # pick a fresh ambient-wander target once back up

    # Once the fall ANIMATION itself has finished playing (its own short
    # duration, typically a couple seconds - much shorter than the whole
    # lie-down pause), switch to the static held-pose clip. Without this,
    # the scene's one shared global timeline loop (a few seconds long, set
    # by whichever clip is longest) keeps wrapping back to frame 0 and
    # replaying the fall from standing pose over and over for the entire
    # ~45s pause - this holds the final collapsed pose steady instead.
    if (mover.get("fallen") and not mover.get("fall_held_triggered")
            and mover.get("fall_start") is not None
            and elapsed >= mover["fall_start"] + mover.get("fall_clip_duration", 2.0)):
        set_animation_clip(mover["skel_prim"], mover["held_clip_path"])
        mover["fall_held_triggered"] = True

    # Trigger this mover's scripted fall once, at their assigned time - a
    # REAL physical collapse at wherever they currently are (patrol or
    # loiter), not a heuristic guess. Fires once via fall_triggered.
    #
    # Real fall ANIMATION now (see bind_fall_animation.py) instead of
    # rigidly rotating the whole model 90 degrees around a mid-stride walk
    # pose (which looked exactly as broken as it sounds - a contorted,
    # clearly-not-actually-fallen mess). Movement stays frozen for the
    # whole FALL_PAUSE_DURATION exactly as before (see the paused_until
    # check below). Root translation on the fall clip itself is zeroed
    # (see fix_fall_translation_and_hold.py - the fall clip's raw
    # translation used a totally different unit/axis convention than the
    # walk clip and caused floating), so only the fall's ROTATIONS drive
    # the pose; the character's position stays exactly where they were
    # standing when they collapsed.
    fall_start = mover.get("fall_start")
    if (fall_start is not None and not mover.get("fall_triggered")
            and elapsed >= fall_start):
        mover["fall_triggered"] = True
        mover["fallen"] = True
        mover["fall_held_triggered"] = False
        mover["paused_until"] = elapsed + FALL_PAUSE_DURATION
        set_animation_clip(mover["skel_prim"], mover["fall_clip_path"])
        UsdGeom.Imageable(mover["prim"]).MakeVisible()
        # Teleport to the assigned fall_center FIRST (see main()'s setup) -
        # with no more patrol walking, the person's "current position"
        # would otherwise just be their unmoving spawn point, generally not
        # inside any camera's view. This guarantees the fall actually
        # happens somewhere detectable, exactly like loiter_center already
        # does for the loitering test below.
        fx, fy = mover.get("fall_center", (None, None))
        if fx is not None:
            fx, fy = clamp_to_floor(fx, fy)
        # The fall clip's own root translation is intentionally zeroed (see
        # fix_fall_translation_and_hold.py - it used a totally different
        # unit/axis convention than the walk clip and caused floating/wrong
        # scale), so the character's root stays at STANDING hip height even
        # though the pose itself rotates to lie flat - that's what was
        # causing the "floating" look. Drop the outer position down to
        # compensate, roughly to ground level, restored on stand-up above.
        FALL_HEIGHT_DROP = 0.75
        base_z = mover.get("base_height", 0.0)
        if fx is not None:
            set_person_translate(mover, fx, fy, base_z - FALL_HEIGHT_DROP)
        else:
            pos = get_translate(mover["prim"])
            set_translate(mover["prim"], (pos[0], pos[1], base_z - FALL_HEIGHT_DROP))
        print(f"*** {mover['name']} has fallen (scripted event) - lying down for {FALL_PAUSE_DURATION}s ***")

    # Frozen in place (scripted fall above, the robot helping them up, an
    # idle animation - see IDLE_PAUSE_CHANCE above, or a search that's
    # specifically targeting this mover) - skip all movement, patrol and
    # loiter alike, until the pause expires.
    if elapsed < mover.get("paused_until", 0) or elapsed < mover.get("denial_reaction_until", 0):
        # A frozen mover isn't "stuck" - keep the stuck-check baseline
        # fresh so it doesn't wake up from a hold (fall recovery, the
        # robot's post-fall identification hold, an idle animation) with
        # a stale baseline and read the hold itself as "0.001m progress in
        # 1.5s" (seen live right after each injury check).
        mover["wander_stuck_check_time"] = elapsed
        mover["wander_stuck_check_pos"] = (mover.get("logical_x", 0.0), mover.get("logical_y", 0.0))
        return
    if mover.get("idling"):
        # The idle pause just ended - restore the walk clip before
        # falling through to normal ambient-wander target-picking below
        # (wander_waypoint is already None from the arrival that
        # triggered this, so a fresh destination gets picked this same
        # tick).
        mover["idling"] = False
        if mover.get("skel_prim") is not None and mover.get("walk_clip_path") is not None:
            set_animation_clip(mover["skel_prim"], mover["walk_clip_path"])

    if mover.get("pending_denied_exit"):
        # The sad-idle reaction beat just ended (see update_robot's denial
        # branch) - NOW start the walk out, using the sad walk clip instead of
        # the normal one for this exit leg. walk_clip_path is temporarily
        # overridden (not sad_walk_clip_path checked separately) so every
        # existing walk-clip-driven mechanism - sync_locomotion_clip's
        # moving/held watchdog, the ordinary movement code below - just works
        # without needing to know about "sad" as a separate concept. Restored
        # to the real one on despawn/revive (see revive_mover).
        mover["pending_denied_exit"] = False
        if mover.get("sad_walk_clip_path") is not None:
            mover["walk_clip_path"] = mover["sad_walk_clip_path"]
            # Apply it immediately rather than leaving it for
            # sync_locomotion_clip's watchdog to catch a tick later - that
            # worked, but printed a "was moving without the walk clip"
            # warning for what is actually an expected, deliberate transition.
            if mover.get("skel_prim") is not None:
                set_animation_clip(mover["skel_prim"], mover["walk_clip_path"])
        mover["visitor_state"] = "exiting"
        mover["wander_waypoint"] = DOOR_POS
        mover["visitor_exit_final"] = True

    # If this mover is in their assigned loiter window, wander RANDOMLY
    # within the actual room box they're in (see ROOM_BOUNDS) - picking a
    # new random point inside the room and walking to it, repeatedly - not
    # tracing a small fixed circle around one spot, which is what looked
    # like "just walking the perimeter."
    loiter_start = mover.get("loiter_start")
    if loiter_start is not None:
        loiter_end = loiter_start + mover["loiter_duration"]
        if loiter_start <= elapsed <= loiter_end:
            if not mover.get("loiter_was_active"):
                mover["loiter_was_active"] = True
                set_animation_clip(mover["skel_prim"], mover["walk_clip_path"])
                cx, cy = mover["loiter_center"]
                mover["loiter_bounds"] = (ROOM_BOUNDS["west"] if cx < 21.75 else ROOM_BOUNDS["east"])
                mover["loiter_waypoint"] = None
                mover["last_loiter_move_time"] = elapsed

            xmin, xmax, ymin, ymax = mover["loiter_bounds"]
            if mover.get("loiter_waypoint") is None:
                # 1m margin so wander targets don't sit literally against a wall
                lwx = random.uniform(xmin + 1, xmax - 1)
                lwy = random.uniform(ymin + 1, ymax - 1)
                # Same guard as ambient wander (see its comment) - never a
                # final destination inside the wall's x-thickness at all,
                # not just the solid part. The "east" room's own xmin
                # (22.0) overlaps the dividing wall's real thickness (up to
                # 22.4), so this fires more often here than for wander.
                dxmin, dxmax = DIVIDER_X_RANGE
                CORNER_BUFFER = 0.6  # same widened buffer as ambient wander - see its comment
                if dxmin - CORNER_BUFFER <= lwx <= dxmax + CORNER_BUFFER:
                    lwx = dxmin - CORNER_BUFFER if lwx < (dxmin + dxmax) / 2 else dxmax + CORNER_BUFFER
                mover["loiter_waypoint"] = (lwx, lwy)
                # Same false-positive fix as ambient wander's identical
                # bug (see its comment) - reset the stuck-check baseline
                # the instant a new waypoint is picked, so the next
                # progress check doesn't straddle an arrival+retarget
                # transition and misread completely normal movement as
                # stuck.
                mover["loiter_stuck_check_pos"] = (
                    mover.get("logical_x", get_translate(mover["prim"])[0]),
                    mover.get("logical_y", get_translate(mover["prim"])[1]))
                mover["loiter_stuck_check_time"] = elapsed

            # Use the TRUE logical position (see set_person_translate's
            # docstring) - not get_translate(mover["prim"]), which is
            # deliberately offset-compensated and does not equal where
            # the person actually is.
            pos_x = mover.get("logical_x", get_translate(mover["prim"])[0])
            pos_y = mover.get("logical_y", get_translate(mover["prim"])[1])

            # Same stuck-safety net as ambient wander (see its comment) -
            # loiter waypoints stay within one room so shouldn't hit the
            # same cross-room routing edge case, but this costs nothing
            # and guarantees no permanent stall here either.
            check_pos = mover.get("loiter_stuck_check_pos")
            check_time = mover.get("loiter_stuck_check_time")
            if check_time is None or elapsed - check_time >= LOITER_STUCK_CHECK_INTERVAL:
                if check_pos is not None:
                    progressed = math.hypot(pos_x - check_pos[0], pos_y - check_pos[1])
                    if progressed < LOITER_STUCK_MIN_PROGRESS:
                        print(f"  [UNSTICK] {mover['name']} stuck/oscillating (loiter) near ({pos_x:.2f},{pos_y:.2f}) "
                              f"- only {progressed:.3f}m progress in {LOITER_STUCK_CHECK_INTERVAL}s, picking a fresh target")
                        mover["loiter_waypoint"] = None
                mover["loiter_stuck_check_pos"] = (pos_x, pos_y)
                mover["loiter_stuck_check_time"] = elapsed

            tx, ty = mover["loiter_waypoint"] if mover["loiter_waypoint"] is not None else (pos_x, pos_y)
            # BUG - THE definitive root cause of the recurring "stuck at
            # exactly x=21.70/22.40" reports (confirmed by hand-tracing:
            # those are literally move_with_wall_check's own clamp values,
            # not coincidence): loiter_bounds gets picked from
            # mover["loiter_center"], a fixed point assigned back at
            # scheduling time (main()'s startup, before the run even
            # begins) - completely independent of wherever the mover
            # actually is when their loiter window starts. A mover
            # ambient-wandering on the WEST side when a loiter window
            # scheduled for an EAST-side zone kicks in tries to walk
            # DIRECTLY toward an east-side target with a raw
            # straight-line vector - this block never called
            # get_next_hop() at all, so it had no doorway-routing
            # awareness whatsoever. Every tick's step got blocked and
            # clamped to exactly the wall boundary - a genuine permanent
            # deadlock, not a false-positive or a fuzzy timing edge case,
            # only ever escaped by the stuck-safety-net forcing a new
            # (possibly equally cross-room) target. Fixed by routing loiter
            # movement through get_next_hop(), same as ambient wander and
            # the robot already do - arrival is still judged against the
            # REAL final target, only the immediate movement step targets
            # the routed hop.
            full_dist = math.hypot(tx - pos_x, ty - pos_y)
            dt = min(elapsed - mover.get("last_loiter_move_time", elapsed), MAX_MOVE_DT)
            mover["last_loiter_move_time"] = elapsed
            LOITER_WALK_SPEED = 1.0  # m/s - an unhurried wandering pace, not a purposeful walk

            if full_dist < 0.3:
                mover["loiter_waypoint"] = None  # arrived - a new random point gets picked next tick
            else:
                hop_x, hop_y = get_next_hop(pos_x, pos_y, tx, ty, hysteresis_state=mover)
                hdx, hdy = hop_x - pos_x, hop_y - pos_y
                hop_dist = math.hypot(hdx, hdy)
                if hop_dist > 1e-6:
                    step = min(LOITER_WALK_SPEED * dt, hop_dist)
                    raw_x = pos_x + hdx / hop_dist * step
                    raw_y = pos_y + hdy / hop_dist * step
                    nx, ny = move_with_wall_check(pos_x, pos_y, raw_x, raw_y)
                    nx, ny = avoid_mover_collision(mover["name"], nx, ny)
                    MIN_YAW_UPDATE_DIST = 0.08  # same fast-spin guard as ambient wander - see its comment
                    if hop_dist > MIN_YAW_UPDATE_DIST:
                        yaw_deg = math.degrees(math.atan2(hdx, -hdy))
                        set_rotation(mover["prim"], (0.0, 0.0, yaw_deg))
                    set_person_translate(mover, nx, ny, mover.get("base_height", 0.0))
            return
        elif mover.get("loiter_was_active"):
            # Loiter window just ended - resume ambient wandering (not
            # idle/invisible - see this function's docstring).
            mover["loiter_was_active"] = False
            mover["loiter_waypoint"] = None
            mover["wander_waypoint"] = None  # pick a fresh ambient target, don't keep the loiter one

    # Ambient wandering - the default state whenever not fallen or in a
    # scripted loiter window. Picks a random point ANYWHERE in the real
    # floor (not restricted to one room), routing through the doorway via
    # get_next_hop() when the target is on the other side of the dividing
    # wall (same real obstacle-avoidance the robot's own patrol uses) -
    # "go all over the place, not straight lines or through walls" per
    # explicit direction, instead of everyone just standing invisible
    # until their one scripted moment.
    WANDER_MARGIN = 1.5  # keep wander targets off the outer walls
    xmin, xmax, ymin, ymax = FLOOR_BOUNDS

    # Stuck-safety net: if this mover hasn't made real progress toward
    # their current waypoint in the last few seconds, abandon it and pick
    # a fresh one, instead of trusting the routing math to always resolve
    # itself. Added after a real, directly-observed case (a live position
    # log showing a mover pinned at almost exactly the dividing wall's own
    # boundary x=22.40, barely below the doorway's y threshold, crawling
    # at a small fraction of the intended walk speed for 10+ real seconds)
    # that wasn't fully root-caused before this session's time ran out -
    # this guarantees no PERMANENT stall regardless of the exact
    # mechanism, which matters more than fully explaining it.
    # BUG (reported live: "why are there two red people in the 2nd room
    # and why are htey just stationary running" - both intruders got
    # yanked off their scripted door-entrance onto a random ambient
    # wander target and never reached the checkpoint) - this stuck-check
    # ALWAYS ran unconditionally for every mover, visitor or not. When it
    # fires it sets wander_waypoint = None and falls straight through,
    # in the SAME tick, to the generic "wander_waypoint is None -> pick a
    # random ambient point anywhere in the building" code a few lines
    # below - it doesn't loop back to the visitor-state-machine at the
    # top of this function, which is the only code that knows what None
    # is supposed to MEAN for a visitor (arrived at the gap, waiting to
    # check in) versus an ambient wanderer (arrived at nowhere-in-
    # particular, pick anywhere new). A false "stuck" read during, say,
    # the real tick-rate crash while the YOLO/DeepFace workers load
    # (confirmed live: ~16 steps/sec instead of the usual ~150-200,
    # meaning consecutive calls for the same mover can be spaced further
    # apart in real wall-clock time than this safety net expects) was
    # enough to derail a scripted door-entrant entirely, sending them off
    # to wander the building instead of the door. Visitors/intruders
    # don't need this generic safety net at all - their own two-leg
    # entering/exiting state machine already handles arrival correctly;
    # skip it for them entirely rather than risk overriding their real
    # destination.
    STUCK_CHECK_INTERVAL = MOVER_STUCK_CHECK_INTERVAL
    STUCK_MIN_PROGRESS = MOVER_STUCK_MIN_PROGRESS
    if mover.get("wander_waypoint") is not None and not mover.get("is_visitor"):
        cur_x = mover.get("logical_x", get_translate(mover["prim"])[0])
        cur_y = mover.get("logical_y", get_translate(mover["prim"])[1])
        check_pos = mover.get("wander_stuck_check_pos")
        check_time = mover.get("wander_stuck_check_time")
        if check_time is None or elapsed - check_time >= STUCK_CHECK_INTERVAL:
            if check_pos is not None:
                progressed = math.hypot(cur_x - check_pos[0], cur_y - check_pos[1])
                if progressed < STUCK_MIN_PROGRESS:
                    print(f"  [UNSTICK] {mover['name']} stuck/oscillating near ({cur_x:.2f},{cur_y:.2f}) "
                          f"- only {progressed:.3f}m progress in {STUCK_CHECK_INTERVAL}s, picking a fresh target")
                    mover["wander_waypoint"] = None
            mover["wander_stuck_check_pos"] = (cur_x, cur_y)
            mover["wander_stuck_check_time"] = elapsed

    if mover.get("wander_waypoint") is None:
        wx = random.uniform(xmin + WANDER_MARGIN, xmax - WANDER_MARGIN)
        wy = random.uniform(ymin + WANDER_MARGIN, ymax - WANDER_MARGIN)
        # Never pick a target INSIDE the dividing wall's own x-thickness AT
        # ALL - not just the solid part. Originally this only nudged a pick
        # that landed in the SOLID wall (y outside the doorway gap), since
        # a point in the walkable gap itself is technically reachable. But
        # confirmed live (traced a full position log) that arriving at a
        # target whose x sits inside DIVIDER_X_RANGE - even at a walkable
        # y - leaves the mover in the inherently ambiguous "mid-crossing"
        # state get_next_hop has to special-case, and picking the NEXT
        # random waypoint from there was producing genuine near-zero-
        # progress stalls (not just false-positive detector noise - a real
        # "moved=0.000m in 1.5s" case). Simplest robust fix: a final
        # wander destination should never be inside the wall's x-thickness
        # at all - the doorway only ever gets walked through in transit
        # (via get_next_hop's own routing), never treated as a place to
        # arrive and stop.
        # Widened further per explicit direction ("stop making them go from
        # the very corners of the doorway... make them go from the
        # middle"): a target picked just OUTSIDE DIVIDER_X_RANGE (e.g.
        # x=22.49, only 0.09m past the 22.4 boundary) still sits right at
        # the wall's actual corner - confirmed live that arriving at one
        # of these near-corner points, combined with a y also near the
        # doorway band's edge, produced a cluster of near-zero-progress
        # re-arrivals right at the wall even with the crossing itself now
        # centered. A real buffer margin around the whole wall structure
        # (not just its exact x-thickness) keeps every final destination
        # comfortably clear of it.
        dxmin, dxmax = DIVIDER_X_RANGE
        CORNER_BUFFER = 0.6
        if dxmin - CORNER_BUFFER <= wx <= dxmax + CORNER_BUFFER:
            wx = dxmin - CORNER_BUFFER if wx < (dxmin + dxmax) / 2 else dxmax + CORNER_BUFFER
        mover["wander_waypoint"] = (wx, wy)
        # BUG (the unstick detector above was itself the false-positive
        # source - confirmed live by tracing a full position log: a mover
        # walked in a completely smooth, continuous straight line toward
        # its target the whole time, then got flagged "stuck" the instant
        # it ARRIVED and picked a fresh waypoint in a new direction). The
        # stuck-check's baseline position/time weren't reset when a new
        # waypoint gets picked here, so its next 1.5s window straddled the
        # arrival+retarget moment - comparing a position from BEFORE
        # arrival against one from AFTER picking a brand new (differently
        # directed) target looks like near-zero net progress even though
        # nothing was ever actually wrong. That falsely discarded a fine
        # fresh waypoint and picked yet another one, over and over - actual
        # net effect was CONSTANT waypoint churn, which would look like
        # exactly the jittery/indecisive movement being reported, not the
        # genuine one-off deadlocks this net exists to catch. Fix: reset
        # the baseline right here, the instant a new waypoint is chosen,
        # so progress is only ever measured within one waypoint's own
        # pursuit, never across a retarget boundary.
        mover["wander_stuck_check_pos"] = (mover.get("logical_x", get_translate(mover["prim"])[0]),
                                            mover.get("logical_y", get_translate(mover["prim"])[1]))
        mover["wander_stuck_check_time"] = elapsed

    pos_x = mover.get("logical_x", get_translate(mover["prim"])[0])
    pos_y = mover.get("logical_y", get_translate(mover["prim"])[1])
    tx, ty = mover["wander_waypoint"]
    last_wander_time = mover.get("last_wander_move_time")
    dt = min(elapsed - (last_wander_time if last_wander_time is not None else elapsed), MAX_MOVE_DT)
    mover["last_wander_move_time"] = elapsed
    WANDER_WALK_SPEED = 1.0  # m/s - unhurried, matches the old loiter pace

    if (mover.get("is_visitor") and mover.get("visitor_state") == "exiting" and not mover.get("exit_via_set")
            and not mover.get("visitor_exit_final") and pos_x < 37.0):
        # Leaving from inside the room: go round the counter, not through it.
        mover["route_via"] = list(COUNTER_VIA_NORTH if pos_y >= 18.43 else COUNTER_VIA_SOUTH)[::-1]
        mover["exit_via_set"] = True
    via = mover.get("route_via")
    if via and math.hypot(via[0][0] - pos_x, via[0][1] - pos_y) < 0.45:
        via.pop(0)
    if via:
        hop_x, hop_y = get_next_hop(pos_x, pos_y, via[0][0], via[0][1], hysteresis_state=mover)
    else:
        hop_x, hop_y = get_next_hop(pos_x, pos_y, tx, ty, hysteresis_state=mover)
    hdx, hdy = hop_x - pos_x, hop_y - pos_y
    hop_dist = math.hypot(hdx, hdy)
    full_dx, full_dy = tx - pos_x, ty - pos_y
    full_dist = math.hypot(full_dx, full_dy)

    if full_dist < 0.3:
        # Per explicit direction ("i'm going to add some idle animations
        # so the characters can do something rather than just walk
        # around, like pause in certain areas and do a funny dance or
        # ponder or something") - on arrival, a real chance to stop and
        # play one of the bound idle/social clips (see
        # bind_idle_animations.py/IDLE_CLIP_NAMES) instead of
        # immediately picking a new destination. Visitors/intruders
        # never reach this branch at all (their movement is governed by
        # the visitor state machine above, which returns/branches before
        # falling through to ambient wander), so this only ever affects
        # Person1/Person2-style ambient wanderers, which is the intended
        # scope.
        idle_clips = mover.get("idle_clips")
        if idle_clips and not mover.get("fallen") and random.random() < IDLE_PAUSE_CHANCE:
            # Reuses the SAME paused_until freeze every other "stand still
            # for a while" case already uses (scripted falls, the robot's
            # own search holds) rather than inventing parallel state - the
            # existing `if elapsed < paused_until: return` guard a few
            # lines below already stops all movement for free. "idling"
            # just remembers that THIS particular freeze needs the walk
            # clip restored once it ends (see that guard's own comment).
            clip_path, duration = random.choice(idle_clips)
            # The whole scene shares ONE global animation timeline that
            # loops every ~3s (see "Animation loop set" at startup), so a
            # long clip (the phone call is ~39s) doesn't actually play
            # through - it replays its first 3 seconds over and over.
            # Confirmed live: a 38.8s hold looked like the earlier "guy
            # frozen in place" complaint. Cap the hold so nobody stands
            # there long enough for the loop to become obvious.
            duration = min(duration, IDLE_MAX_SECONDS)
            mover["paused_until"] = elapsed + duration
            mover["idling"] = True
            print(f"  [IDLE] {mover['name']} pausing {duration:.1f}s to play {str(clip_path).rsplit('/', 1)[-1]}")
            if mover.get("skel_prim") is not None:
                set_animation_clip(mover["skel_prim"], clip_path)
        mover["wander_waypoint"] = None  # arrived - a new random point gets picked next tick (once any idle pause above finishes)
    elif hop_dist > 1e-6:
        step = min(WANDER_WALK_SPEED * dt, hop_dist)
        raw_x = pos_x + hdx / hop_dist * step
        raw_y = pos_y + hdy / hop_dist * step
        # Taking turns at the doorway: someone leaving (escorted out, turned
        # away) must not walk into the person standing in the gap waiting to
        # be checked in - hold until they have been (bounded, never a deadlock).
        if mover.get("is_visitor") and in_door_zone(raw_x, raw_y) and not in_door_zone(pos_x, pos_y):
            # The gap is only wide enough for one person - anyone ABOUT TO STEP IN
            # yields to whoever is already inside, entering or exiting alike (seen
            # live: an exiting resident and a just-checked-in one crossing paths in
            # the gap from opposite directions, 0.12m apart). Someone already inside
            # keeps going (checked below via the pos_x guard) so nobody ever stops
            # mid-doorway - only the one about to enter it waits, bounded so two
            # people can never lock each other out.
            if door_zone_busy(_tick_state.get("movers", []), exclude=mover):
                held_since = mover.setdefault("door_hold_since", elapsed)
                if elapsed - held_since < DOOR_YIELD_MAX_WAIT:
                    return
            mover.pop("door_hold_since", None)
        nx, ny = move_with_wall_check(pos_x, pos_y, raw_x, raw_y)
        # BUG (reported live: "people start spinning like crazy, fast
        # spins, when they try and cross between rooms, robot does it
        # too"): the old guard here was hop_dist > 1e-6 - right at a
        # doorway crossing, get_next_hop's "mid" branch returns a hop
        # target that tracks the mover's OWN current y, so as x closes in
        # on the crossing point hop_dist can shrink to a few millimeters
        # while still passing that guard - at that scale, per-tick
        # floating-point step noise can flip hdx's sign frame to frame,
        # and atan2(hdx, -hdy) swings by up to 180 degrees each tick,
        # which looks exactly like frantic spinning in the viewport. Real
        # fix: only update the FACING direction when the hop vector is
        # meaningfully larger than that noise floor - still keep moving
        # (position update is fine at any nonzero distance), just don't
        # let a near-zero vector drive the rotation.
        nx, ny = avoid_mover_collision(mover["name"], nx, ny)
        MIN_YAW_UPDATE_DIST = 0.08
        if hop_dist > MIN_YAW_UPDATE_DIST:
            yaw_deg = math.degrees(math.atan2(hdx, -hdy))  # face the immediate hop, same convention as the robot's own patrol movement
            set_rotation(mover["prim"], (0.0, 0.0, yaw_deg))
        set_person_translate(mover, nx, ny, mover.get("base_height", 0.0))
        if elapsed - mover.get("last_wander_print", -999) >= 3.0:
            mover["last_wander_print"] = elapsed
            print(f"  [WANDER] {mover['name']} at ({nx:.2f},{ny:.2f}), hop=({hop_x:.2f},{hop_y:.2f}), "
                  f"target=({tx:.2f},{ty:.2f}), moved={math.hypot(nx - pos_x, ny - pos_y):.3f}m")


def get_nearest_mover(movers, zone_name):
    """Same nearest-mover-to-the-zone logic as get_nearest_mover_coords, but returns
    the mover itself (so callers can track it by NAME afterward, not just a one-time
    coordinate snapshot - see the loitering dispatch's own comment for why that
    snapshot going stale was the real cause of most missed identifications)."""
    zone_pos = ZONE_CENTERS.get(zone_name)
    if zone_pos is None or not movers:
        return None
    return min(movers, key=lambda m: (
        (m.get("logical_x", get_translate(m["prim"])[0]) - zone_pos[0]) ** 2 +
        (m.get("logical_y", get_translate(m["prim"])[1]) - zone_pos[1]) ** 2
    ))


def get_nearest_mover_coords(movers, zone_name):
    """Returns the actual live (x, y) of whichever mover is currently
    closest to the given zone - used so the robot is dispatched to where
    someone actually is right now, not a fixed zone-camera coordinate.
    Simplified: used to restrict this to "active" (patrolling) movers only,
    back when some people patrolled and others sat frozen at spawn - now
    that NOBODY continuously patrols (everyone's idle until their own
    scripted fall/loiter event, which explicitly teleports them into a
    zone - see fall_center/loiter_center), every mover is equally valid to
    consider.
    """
    zone_pos = ZONE_CENTERS.get(zone_name)
    if zone_pos is None or not movers:
        return zone_pos
    # Use each mover's TRUE logical position (see set_person_translate's
    # docstring), not get_translate(m["prim"]) - that prim's own translate
    # is deliberately offset-compensated to correct for the /model child's
    # static per-character offset, so it no longer equals where the
    # person actually is. Reading it directly here sent the robot's whole
    # orbit search to a point several meters from the real fallen person -
    # a real bug caught in a live run (see NOTES.md, "THIRD FOLLOW-UP").
    nearest = min(movers, key=lambda m: (
        (m.get("logical_x", get_translate(m["prim"])[0]) - zone_pos[0]) ** 2 +
        (m.get("logical_y", get_translate(m["prim"])[1]) - zone_pos[1]) ** 2
    ))
    npos_x = nearest.get("logical_x", get_translate(nearest["prim"])[0])
    npos_y = nearest.get("logical_y", get_translate(nearest["prim"])[1])
    return clamp_to_floor(npos_x, npos_y)


def get_robot_yaw_degrees(rigid_prim):
    """Current robot heading, extracted from its live physics orientation.

    NOTE (one real unverified assumption): assumes Isaac's
    RigidPrim.get_world_poses() returns orientation quaternions in
    (w, x, y, z) order - the standard Isaac Sim/pxr convention, but not
    independently confirmed on THIS robot asset here. If the search loop
    below seems to wait forever / never detects the robot as having turned
    to the requested heading, this ordering is the first thing to check
    (try swapping to (x, y, z, w) and see if it starts working).
    """
    _, orientations = rigid_prim.get_world_poses()
    w, x, y, z = orientations[0]
    yaw_rad = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(yaw_rad)


def request_robot_search_heading(x, y, yaw_deg, request_id):
    """UNUSED now that the robot moves kinematically (see search_for_face
    below, which just sets rotation directly) - kept only because
    dispatch_bridge.py on the WSL side still references this file if
    anyone's still running the old nav stack alongside this. Safe to
    ignore/delete once you've confirmed you don't need that path anymore.
    """
    with open(FACE_SEARCH_GOAL_FILE, "w") as f:
        json.dump({"x": x, "y": y, "yaw_deg": yaw_deg, "request_id": request_id}, f)


def wait_for_robot_yaw(rigid_prim, target_yaw_deg, timeout):
    """Polls the robot's real heading (via _tick(), so physics/rendering
    keep stepping the whole time - same pattern as run_subprocess's wait)
    until it's within FACE_SEARCH_YAW_TOLERANCE_DEG of the target, or the
    timeout is hit. Returns whether it actually got there - if not, the
    caller scans anyway rather than getting stuck forever on one heading
    Nav2 can't quite reach (e.g. an obstacle preventing the exact turn).
    """
    start = time.time()
    while time.time() - start < timeout:
        _tick()
        time.sleep(0.05)
        current_yaw = get_robot_yaw_degrees(rigid_prim)
        diff = abs(((current_yaw - target_yaw_deg + 180) % 360) - 180)
        if diff <= FACE_SEARCH_YAW_TOLERANCE_DEG:
            return True
    return False


def find_next_unhandled_alert(handled_alert_ids):
    """Reads the event log for the next dispatch_alert that has real
    coordinates and hasn't already been handled by the robot this run -
    preferring the oldest unhandled 'fall' alert over any 'loitering'
    alert, and only falling back to plain oldest-first within the same
    reason. A scripted fall only stays down for FALL_PAUSE_DURATION (45s)
    before standing back up on its own - confirmed via a real run that
    the robot working through several queued loitering alerts first let
    a fall's window expire before it ever got there, so every subsequent
    fall search legitimately found nobody (they'd already stood back up),
    not because of any remaining positioning/pitch bug. Falls are also
    the more urgent real-world case (loitering can be watched longer;
    someone down cannot).

    Filters against each alert's OWN unique alert_id (see next_alert_id),
    not its timestamp - two genuinely different alerts (different zones,
    different people) can share an identical timestamp string when they're
    both logged inside the same sweep batch (confirmed live: 3 loitering
    alerts for 3 different people, same microsecond). Filtering by
    timestamp meant marking ONE of those handled silently hid its same-
    timestamp siblings too, even though the robot never actually searched
    for them - dispatches were quietly vanishing. Falls back to timestamp
    only for the rare pre-this-fix log entry that has no alert_id."""
    if not os.path.exists(EVENT_LOG_FILE):
        return None
    with open(EVENT_LOG_FILE, "r") as f:
        try:
            log = json.load(f)
        except json.JSONDecodeError:
            return None
    candidates = [e for e in log
                  if e.get("event_type") == "dispatch_alert"
                  and e.get("coords")
                  and e.get("alert_id", e.get("timestamp")) not in handled_alert_ids]
    for e in candidates:
        if e.get("reason") == "fall":
            return e
    for e in candidates:
        return e
    return None


def attempt_robot_scan(robot_state, camera_path, authorized_ids, banned_ids, alert, heading_label="", mover=None):
    """One scan attempt from wherever the robot is currently facing -
    YOLO -> crop -> DeepFace -> authorization, same pipeline as the fixed
    checkpoint camera. Returns True only on a CONFIRMED identification
    (face matched an existing person, or a new person was registered) -
    that's the real success signal the search loop below uses to know
    when to stop turning and try more headings. "Person visible but no
    clear face" and "nobody in view" both return False so the search
    keeps going.

    banned_ids is a separate check from authorized_ids - see
    BANNED_IDS_FILE's comment for why "not authorized" (an unregistered
    visitor) and "banned" (a specifically flagged individual) need to be
    different severities, not the same binary check.
    """
    label = f" ({heading_label})" if heading_label else ""
    print(f"  [ROBOT] scanning{label}...")
    bgr = capture_frame(camera_path, resolution=(480, 360))
    if bgr is None:
        print("  [ROBOT] no frame captured")
        return False

    # Save the FULL raw frame (not just the eventual person crop below) for
    # every single scan attempt, labeled by standpoint - this is the actual
    # "let me see what the robot sees" fix. Whatever's going wrong with the
    # orbit not landing a face (person too small/far, wrong angle, face
    # occluded, standpoint math off) should be directly visible by paging
    # through these in order after a run, in
    # C:\isaacsim\projects\surveillance-proj\debug_captures\.
    safe_label = (heading_label or "noheading").replace(" ", "_").replace("/", "-")
    raw_debug_name = f"{datetime.now().strftime('%H%M%S')}_robot_RAWFRAME_{safe_label}.jpg"
    if not RECORDING_MODE:
        cv2.imwrite(os.path.join(DEBUG_DIR, raw_debug_name), bgr)

    frame_path = os.path.join(os.environ["TEMP"], "robot_scan.jpg")
    cv2.imwrite(frame_path, bgr)
    yolo_result = _workers["yolo"].call(frame_path)
    detections = yolo_result.get("detections", []) if not yolo_result.get("error") else []
    if not detections:
        print(f"  [ROBOT] nobody in view{label}")
        return False

    best_detection = max(detections, key=lambda d: d["confidence"])
    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return False

    crop_path = os.path.join(os.environ["TEMP"], "robot_crop.jpg")
    cv2.imwrite(crop_path, crop)
    debug_name = f"{datetime.now().strftime('%H%M%S')}_robot_crop.jpg"
    if not RECORDING_MODE:
        cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    # Fast path: match by clothing/appearance FIRST - per explicit
    # direction ("matching by clothes would make it a lot easier too but
    # still use facial id"). extract_appearance_signature/
    # find_appearance_match/save_appearance_profile already existed in
    # this file (an HSV color-histogram fingerprint, cheap - no
    # subprocess round-trip at all) but were never actually wired into
    # the identification pipeline until now; every scan paid a full
    # DeepFace call even for someone already positively identified
    # minutes ago. Only trusted for confirming someone ALREADY known,
    # never for a fresh registration - clothing alone isn't a strong
    # enough signal to assign a brand-new ID, only to skip re-running
    # DeepFace on someone it already has a real face match on record for.
    appearance_id, appearance_score = find_appearance_match(crop)
    # BUG (confirmed live: Person1 and Person2 both ended up tagged with
    # the SAME known_id, causing a real overstay dispatch to resolve to
    # the wrong person - Person1 got "escorted out" twice while Person2,
    # who genuinely had overstayed too, silently never got found again)
    # - clothing alone can't tell two people apart nearly as reliably as
    # a face can, and this project's characters can easily land above
    # APPEARANCE_MATCH_THRESHOLD's correlation against EACH OTHER, not
    # just against themselves (confirmed: a 0.92 score matched the wrong
    # person). An appearance match is only trustworthy when it ISN'T
    # contradicted by more certain evidence - if some OTHER mover
    # currently in the scene already holds this exact ID, two different
    # real people can't both BE that one identity at the same moment, so
    # distrust the clothing match here and fall through to a real
    # DeepFace check instead of blindly trusting a color histogram.
    # id_owner remembers which person each id belongs to for the whole run
    # (even while that person is outside), so identity conflicts are caught
    # for someone who has been escorted out and is due to return, not just
    # for people currently standing in the scene.
    id_owner = _tick_state.setdefault("id_owner", {})
    if appearance_id is not None and mover is not None:
        owner_name = id_owner.get(appearance_id)
        if owner_name is not None and owner_name != mover["name"]:
            print(f"  [ROBOT] clothing/appearance suggested {appearance_id} but {owner_name} "
                  f"already holds that id - falling back to a real face check instead of trusting it")
            appearance_id = None
    # BUG (found live: Person4 and Person5 - two different characters,
    # both tinted the same red - both came back as ID_0001, when the ask
    # was two separate denied IDs): this fast path's own comment says it's
    # only for confirming someone ALREADY known, but the code trusted a
    # clothing match for ANY mover, including one being identified for
    # the first time - so a new person wearing similar colors inherited a
    # stranger's ID without DeepFace ever looking at their face. When the
    # caller says WHICH mover this is, only trust the clothing match if
    # that mover already holds that exact ID; otherwise fall through to a
    # real face check.
    if appearance_id is not None and mover is not None and mover.get("known_id") != appearance_id:
        print(f"  [ROBOT] clothing suggested {appearance_id}, but {mover['name']} isn't confirmed as that id yet "
              f"- checking the face instead")
        appearance_id = None
    if appearance_id is not None:
        person_id = appearance_id
        print(f"  [ROBOT] matched by clothing/appearance: {person_id} (score={appearance_score:.2f})")
    else:
        reid_result = _workers["deepface"].call(f"{crop_path}\t{KNOWN_FACES_DIR}")
        status = reid_result.get("status")
        person_id = None

        if status == "match":
            matched_filename = os.path.basename(reid_result["matched_path"])
            person_id = os.path.splitext(matched_filename)[0]
            owner_name = id_owner.get(person_id)
            if mover is not None and owner_name is not None and owner_name != mover["name"]:
                # Two different people can't be the same identity. The face
                # embeddings of this project's synthetic characters (the two
                # intruders especially) get confused often enough that a
                # "match" to someone else's id is really a matching error -
                # seen live: an intruder matched a resident's id and got
                # that resident's identity auto-banned.
                print(f"  [ROBOT] face matched {person_id}, but that id belongs to {owner_name}, not "
                      f"{mover['name']} - treating it as a face-match error")
                status = "new"
                person_id = None
            else:
                print(f"  [ROBOT] confirmed identity: {person_id}")
        if status == "new" and mover is not None and mover.get("known_id"):
            # The face didn't match a stored one (different pose / distance /
            # lighting than the reference shot), but the robot is scanning the
            # very person it has already been tracking under this id - keep it
            # rather than registering the same person a second time (seen live:
            # Person2 became ID_0004 at a loitering search, then ID_0005 at the
            # injury check, resetting their visit clock and doubling the log).
            person_id = mover["known_id"]
            print(f"  [ROBOT] face didn't match a stored one, but this is the person already tracked as "
                  f"{person_id} - keeping that id instead of registering a duplicate")
        elif status == "new":
            person_id = f"ID_{get_next_id():04d}"
            cv2.imwrite(os.path.join(KNOWN_FACES_DIR, f"{person_id}.jpg"), crop)
            print(f"  [ROBOT] new person found on-site: {person_id}")
        elif status != "match":
            print(f"  [ROBOT] person visible but no clear face{label}: {reid_result.get('reason')}")
            return False

    save_appearance_profile(person_id, crop)
    # Per-ID "how long have they been here" tracking (see VISIT_TIME_LIMIT)
    # - only possible when the caller knows WHICH mover it just scanned
    # (checkpoint check-in and an active orbit search both do). Records
    # the first time this ID was ever seen this run, and tags the mover
    # itself with its now-confirmed ID so a later overstay dispatch can
    # find them again by ID instead of needing a live position handed to
    # it from wherever the original sighting happened.
    if mover is not None:
        mover["known_id"] = person_id
        id_owner.setdefault(person_id, mover["name"])
        first_seen = _tick_state.setdefault("person_first_seen", {})
        if person_id not in first_seen:
            first_seen[person_id] = time.time() - _tick_state["sim_start"]
    # Side-channel for callers that need the identified ID directly (e.g.
    # assist_fallen_person's post-recovery injury-log identification) -
    # simpler than threading a return value through search_for_face's own
    # standpoint loop, which already has a different True/False return
    # contract (search succeeded or not) that other callers rely on.
    robot_state["last_identified_id"] = person_id
    is_authorized = person_id in authorized_ids
    is_banned = person_id in banned_ids
    print(f"  [ROBOT] double-check result: {person_id} -> {'AUTHORIZED' if is_authorized else 'NOT AUTHORIZED'}")
    append_log_entry({
        "event_type": "robot_response", "found": True,
        "person_id": person_id, "authorized": is_authorized, "banned": is_banned,
        "alert_reason": alert.get("reason"), "alert_id": alert.get("alert_id"), "zone": alert.get("zone"),
        "timestamp": datetime.now(timezone.utc).isoformat()
    })
    # A banned match is a real, distinct security event - not just the
    # quiet "not authorized" logging above, which an ordinary unregistered
    # visitor also gets. Fires its own high-severity, clearly-labeled
    # alert so it's impossible to miss in the log/console.
    if is_banned:
        print(f"  [ROBOT] *** INTRUDER ALERT: {person_id} is on the banned list - "
              f"not permitted on premises! ***")
        append_log_entry({
            "event_type": "intruder_alert",
            "person_id": person_id,
            "zone": alert.get("zone"),
            "coords": alert.get("coords"),
            "timestamp": datetime.now(timezone.utc).isoformat()
        })
        # Escort them to the door and remove them from the premises - per
        # explicit direction ("robot escorts them to the exit"). We don't
        # track which MOVER a given face ID belongs to anywhere (identity
        # only lives in known_faces/ crops), so the practical stand-in is
        # "whichever mover the robot is currently closest to" - correct
        # in every real case here, since the robot is physically standing
        # right in front of whoever it just scanned. Reuses the exact
        # same wander-to-a-forced-waypoint + despawn-on-arrival machinery
        # spawn_visitor's "exiting" state already uses (see
        # update_person_position) - works identically whether this mover
        # started as a visitor or one of the original 4 characters, and
        # this removal is intentionally permanent (a banned person doesn't
        # get to come back), not a temporary pause.
        movers = _tick_state.get("movers", [])
        candidates = [m for m in movers if not m.get("should_despawn")]
        if candidates:
            robot_pos = get_translate(robot_state["prim"])
            # When the caller says which mover was scanned, escort THAT one -
            # "nearest to the robot" picked a resident who happened to be
            # standing near the door while an intruder was being checked in
            # (seen live: Person2 escorted out under Person4's banned id).
            nearest = mover if mover is not None and mover in candidates else min(candidates, key=lambda m: (
                (m.get("logical_x", get_translate(m["prim"])[0]) - robot_pos[0]) ** 2 +
                (m.get("logical_y", get_translate(m["prim"])[1]) - robot_pos[1]) ** 2
            ))
            print(f"  [ROBOT] escorting {nearest['name']} ({person_id}) to the exit...")
            nearest["paused_until"] = 0
            nearest["is_visitor"] = True
            nearest["visitor_state"] = "exiting"
            nearest["wander_waypoint"] = DOOR_GAP_WAYPOINT
            nearest["visitor_exit_final"] = False
    return True


def assist_fallen_person(robot_state, alert, movers, authorized_ids, banned_ids, elapsed):
    """Handles a 'fall' dispatch alert by helping the person up, then - new
    this session, for the injury log - identifying them once they're back
    on their feet.

    Finds whichever mover is actually still fallen nearest the alert's
    coordinates (the robot may have taken a while to arrive - if nobody
    is down there anymore, either they already stood up on their own via
    the normal FALL_PAUSE_DURATION timeout, or this was a false alarm)
    and ends their lie-down pause immediately - they stand back up on
    the very next tick via update_person_position's own existing
    recovery code path, exactly as if the timeout had elapsed normally,
    just sooner because the robot got there.

    Identification still never happens DURING the fall itself (the pose
    hides the face from every camera angle a ground robot can reach -
    confirmed via a direct test). But now that a real get-up animation
    exists, waiting for it to finish gives an ordinary standing/frontal
    pose - the exact case the orbit-search pipeline already handles well
    for loitering. So: wait for recovery, hold just this one person in
    place a little longer (not the whole building - no reason to freeze
    every other mover for one identification check), run the same
    search, and log the result as a real injury incident.
    """
    target = alert["coords"]
    fallen_movers = [m for m in movers if m.get("fallen")]
    if not fallen_movers:
        print(f"  [ROBOT] arrived at {alert['zone']} to help, but nobody is down anymore.")
        return
    nearest = min(fallen_movers, key=lambda m: (
        (m.get("logical_x", get_translate(m["prim"])[0]) - target[0]) ** 2 +
        (m.get("logical_y", get_translate(m["prim"])[1]) - target[1]) ** 2
    ))
    nearest["paused_until"] = 0
    print(f"  [ROBOT] arrived at {alert['zone']} - helping {nearest['name']} back up.")
    append_log_entry({
        "event_type": "robot_response", "found": True, "assisted": True,
        "person_id": nearest["name"], "alert_reason": alert.get("reason"), "alert_id": alert.get("alert_id"),
        "zone": alert.get("zone"), "timestamp": datetime.now(timezone.utc).isoformat()
    })

    # Wait (real time, bounded) for the get-up animation to actually
    # finish - mover["fallen"] only clears once update_person_position's
    # stand-up state machine completes, see its own comment.
    RECOVERY_TIMEOUT = 8.0
    wait_start = time.time()
    while nearest.get("fallen") and (time.time() - wait_start) < RECOVERY_TIMEOUT:
        _tick()
    if nearest.get("fallen"):
        print(f"  [ROBOT] {nearest['name']} hasn't finished getting up yet - "
              f"skipping injury-log identification this time.")
        return

    # Hold them here just long enough for a clean shot, then let them
    # resume wandering immediately once the search finishes either way.
    nearest["paused_until"] = elapsed + 20
    injury_check_alert = {
        "coords": (nearest.get("logical_x", get_translate(nearest["prim"])[0]),
                   nearest.get("logical_y", get_translate(nearest["prim"])[1])),
        "zone": alert.get("zone"),
        "reason": "injury_check",
        "timestamp": alert.get("timestamp"),
        "alert_id": alert.get("alert_id", alert.get("timestamp")),  # same episode as the fall alert - see the pending-fall check below
    }
    # in_search is already held True for this whole function by the
    # caller (update_robot) - see its own comment for why setting/
    # clearing it again in here too would open a real (if small) gap
    # where the robot could reassign itself mid-assist.
    robot_state["last_identified_id"] = None
    try:
        search_for_face(robot_state, injury_check_alert, authorized_ids, banned_ids, mover=nearest)
    finally:
        nearest["paused_until"] = 0

    identified_id = robot_state.get("last_identified_id")
    if identified_id:
        print(f"  [ROBOT] injury log: {nearest['name']} identified as {identified_id}")
    else:
        print(f"  [ROBOT] injury log: could not confirm identity of the person who fell near {alert.get('zone')}")
    append_log_entry({
        "event_type": "injury_incident",
        "person_id": identified_id,
        "mover_name": nearest["name"],
        "zone": alert.get("zone"),
        "timestamp": datetime.now(timezone.utc).isoformat(),
    })


def reset_robot_camera(robot_state):
    """Puts the robot camera back to its neutral mount (looking along the
    body's forward axis, level). search_for_face aims the camera with a
    per-standpoint look-at rotation and used to leave the LAST one in place,
    so every later scan from the desk pointed wherever the previous target
    had been (frames of the black void beyond the door, or an upside-down
    person) - the real reason check-in scans kept reporting "nobody in view"."""
    cam_prim = omni.usd.get_context().get_stage().GetPrimAtPath(robot_state["camera_path"])
    if not cam_prim.IsValid():
        return
    try:
        # Upright, level, looking along the body's forward axis: local X -> world -X,
        # local Y (image up) -> world +Z, local Z -> world +Y (so the camera looks
        # along -Y) - the same rotation as Euler (90, 0, 180) that
        # create_lightweight_robot mounts it with.
        neutral = Gf.Matrix4d(-1, 0, 0, 0,
                              0, 0, 1, 0,
                              0, 1, 0, 0,
                              0, 0, 0, 1).ExtractRotationQuat()
        for op in UsdGeom.Xformable(cam_prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                op.Set(Gf.Vec3f(90.0, 0.0, 180.0))
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                # in whichever precision the op was authored with (the Camera
                # wrapper swaps the op for a double-precision one)
                if isinstance(op.Get(), Gf.Quatf):
                    op.Set(Gf.Quatf(neutral.GetReal(), Gf.Vec3f(*neutral.GetImaginary())))
                else:
                    op.Set(neutral)
    except Exception as exc:  # never let a camera reset take the whole sim down
        print(f"  [ROBOT] WARNING: could not reset the camera orientation ({exc})")


def search_for_face(robot_state, alert, authorized_ids, banned_ids, mover=None):
    """Orbit search (see _search_for_face_impl); always leaves the camera
    back at its neutral mount afterwards."""
    try:
        return _search_for_face_impl(robot_state, alert, authorized_ids, banned_ids, mover=mover)
    finally:
        reset_robot_camera(robot_state)
        for _ in range(2):
            _tick()


def _search_for_face_impl(robot_state, alert, authorized_ids, banned_ids, mover=None):
    """REAL fix for "the robot just spins in place near the body and finds
    nothing": rotating the camera at ONE fixed standing spot can never see
    a face that happens to be oriented away from that spot - no amount of
    turning changes WHERE the robot is standing. This now actually orbits
    the target: it physically moves to several standpoints around them
    (a circle of ORBIT_RADIUS meters, ORBIT_STANDPOINTS positions around
    it), faces the center from each one, and runs a real vision-based scan
    (YOLO -> crop -> DeepFace, via attempt_robot_scan) at each standpoint -
    stopping as soon as one actually succeeds. No ground-truth aiming at a
    known mover position anymore either - every standpoint is judged
    purely by what the robot's own camera + YOLO/DeepFace actually see,
    same as a real system would have to.

    Purely kinematic - moving between standpoints and facing the center is
    just set_world_translate()/set_rotation() (same as the rest of this
    file's movement), no physics, no Nav2, instant and reliable by
    construction.
    """
    ORBIT_RADIUS = 2.5       # meters - shrunk from 4.0: several zone centers are close enough to walls/corners that a 4m orbit sent standpoints straight outside the building (confirmed via diagnose_room_bounds.py - real floor is only ~35x20m and zones sit near corners)
    ORBIT_STANDPOINTS = 6    # trimmed from 8 - fewer forced-render sequences per search now that the target is actually frozen and correctly aimed at

    center = alert["coords"]
    print(f"  [ROBOT] arrived near {alert['zone']} ({alert['reason']}) - "
          f"orbiting the target across {ORBIT_STANDPOINTS} standpoints to find a clear view of their face...")

    # The robot's camera is mounted at a FIXED pitch (perfectly horizontal,
    # ROBOT_CAMERA_HEIGHT - see create_lightweight_robot) - fine for a
    # standing person's face, but a fallen person is lying near the floor,
    # almost entirely BELOW that horizontal sightline at ORBIT_RADIUS
    # distance. Confirmed via real saved standpoint frames showing nothing
    # but wall/floor (or, after a botched pitch-only attempt, solid black -
    # see below) with no person visible at all - not "wrong angle," a
    # camera that was never really aimed at the actual (low) target height.
    #
    # Two earlier attempts at just adjusting the camera's own local pitch
    # (a plain X-axis rotateXYZ, then an orient quaternion after
    # discovering the isaacsim Camera wrapper silently swaps op types)
    # both failed in practice - composing a separately-set local pitch
    # with the robot BASE's own yaw rotation (set via set_rotation() per
    # standpoint) turned out to be too easy to get subtly wrong (confirmed
    # via a real captured frame that was solid black - the camera ended up
    # aimed into open space above the ceiling, not down at the target).
    #
    # Real fix: compute the camera's full WORLD-space look-at orientation
    # directly (same proven method as diagnose_fall_pose_closeup.py
    # earlier this session - right/up/forward basis vectors from the real
    # camera and target positions), then convert that into the camera's
    # LOCAL orientation using the robot base's ACTUAL live world transform
    # (read fresh each standpoint, not assumed) - this sidesteps yaw/pitch
    # composition entirely instead of trying to get it right by hand.
    target_z = 0.3 if alert.get("reason") == "fall" else ROBOT_CAMERA_HEIGHT
    camera_stage_prim = omni.usd.get_context().get_stage().GetPrimAtPath(robot_state["camera_path"])

    for step in range(ORBIT_STANDPOINTS):
        # Re-aim at the target's LIVE position before this standpoint, not just the
        # position it happened to be at when the orbit started. A fallen person is
        # frozen so this is a no-op for fall/injury_check searches, but a loitering
        # person keeps wandering the whole time (deliberately never frozen - see the
        # dispatch site's own comment) - each standpoint is a real several-second
        # YOLO+DeepFace round trip, so across all 6 standpoints they can easily have
        # walked several meters from where the search started. Confirmed via the
        # report's own numbers (only ~53% of loitering searches landed a face) that
        # this staleness, not a genuine detection limitation, was the dominant cause.
        if mover is not None and mover.get("logical_x") is not None and not mover.get("should_despawn"):
            center = clamp_to_floor(mover["logical_x"], mover["logical_y"])
        # REAL FIX for "the robot isn't even going to the person who's
        # fallen, why did it take so long" (reported live): update_robot()
        # can't act on ANY new alert while robot_state["in_search"] is set
        # (see its guard) - a fall alert that arrives while the robot is
        # mid-way through a lower-priority loitering search used to just
        # sit queued until the ENTIRE current search finished (up to
        # ORBIT_STANDPOINTS standpoints, each a real YOLO+DeepFace call),
        # by which point the fallen person's FALL_PAUSE_DURATION window
        # could already be running out. Check between every standpoint -
        # if a fall alert has shown up and this search isn't already for
        # a fall itself, abandon the rest of this search immediately
        # rather than finish it. update_robot() picks the fall up on its
        # very next tick once this returns.
        if alert.get("reason") != "fall":
            pending = find_next_unhandled_alert(robot_state["handled_alert_ids"])
            # BUG (found by tracing why every post-fall injury check logged
            # "could not confirm identity", 0 of 2 in the run report): the
            # injury_check search runs INSIDE assist_fallen_person, which
            # is itself handling a fall alert that only gets added to
            # handled_alert_ids AFTER it returns - so this check saw that
            # very same alert as an unhandled "new" fall and aborted the
            # search immediately, every time. injury_check_alert reuses the
            # parent fall alert's own alert_id, so compare against that.
            if (pending is not None and pending.get("reason") == "fall"
                    and pending.get("alert_id", pending.get("timestamp")) != alert.get("alert_id", alert.get("timestamp"))):
                print(f"  [ROBOT] a fall alert just came in - abandoning the "
                      f"in-progress {alert['reason']} search early to respond")
                return
        angle = (2 * math.pi / ORBIT_STANDPOINTS) * step
        raw_x = center[0] + ORBIT_RADIUS * math.sin(angle)
        raw_y = center[1] + ORBIT_RADIUS * math.cos(angle)
        # Clamp each standpoint to the real floor bounds - zone centers are
        # back to being the actual camera positions (near walls/corners),
        # so some orbit standpoints would otherwise land outside the
        # building again. This keeps the robot physically inside no matter
        # what, even if it means a slightly squashed orbit near a corner.
        sx, sy = clamp_to_floor(raw_x, raw_y)
        # Also keep the standpoint on the SAME SIDE of the dividing wall as
        # the target itself - confirmed by hand (and matching real search
        # failures for a target right next to the wall/a corner, e.g.
        # Camera1's own mount at (22.6, 27.36)) that a plain circular orbit
        # can put HALF its standpoints on the opposite side of the wall
        # from the target, with the wall directly between camera and
        # person - a guaranteed blocked/wrong-room view no camera pitch
        # fix can help with. clamp_to_floor alone only keeps standpoints
        # inside the outer building footprint, not clear of the inner
        # dividing wall.
        dxmin, dxmax = DIVIDER_X_RANGE
        target_side = "west" if center[0] < dxmin else ("east" if center[0] > dxmax else "mid")
        standpoint_side = "west" if sx < dxmin else ("east" if sx > dxmax else "mid")
        in_doorway = DOORWAY_Y_RANGE[0] <= sy <= DOORWAY_Y_RANGE[1]
        if target_side != "mid" and standpoint_side != target_side and not in_doorway:
            sx = dxmax if target_side == "east" else dxmin
        standpoint = (sx, sy, robot_state["base_height"])
        # Face the CENTER (the person). Now that the camera is our own
        # SimpleRobot asset (see create_lightweight_robot), it was mounted
        # specifically to match the SAME body-forward convention used
        # everywhere else in this file - plain atan2(dx, -dy), no more
        # special-cased offset math for a mystery mount rotation.
        dx = center[0] - standpoint[0]
        dy = center[1] - standpoint[1]
        face_yaw = math.degrees(math.atan2(dx, -dy))
        print(f"  [ROBOT] standpoint {step + 1}/{ORBIT_STANDPOINTS}: pos=({sx:.2f},{sy:.2f}) "
              f"target=({center[0]:.2f},{center[1]:.2f}) dist={math.hypot(dx, dy):.2f}m yaw={face_yaw:.1f}")
        set_world_translate(robot_state["prim"], standpoint)
        set_rotation(robot_state["prim"], (0.0, 0.0, face_yaw))
        for _ in range(3):  # let the move/rotation land and a real frame catch up before scanning
            _tick()

        # Aim the camera directly at the real target position in WORLD
        # space (see this function's docstring for why local-pitch-only
        # attempts failed) - a real look-at, not a guessed Euler pitch.
        if camera_stage_prim.IsValid():
            cam_xformable = UsdGeom.Xformable(camera_stage_prim)
            parent_prim = camera_stage_prim.GetParent()
            parent_world = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(0)
            cam_world_pos = parent_world.Transform(Gf.Vec3d(0, 0, ROBOT_CAMERA_HEIGHT))
            target_world_pos = Gf.Vec3d(center[0], center[1], target_z)
            forward = target_world_pos - cam_world_pos
            if forward.GetLength() > 1e-6:
                forward = forward / forward.GetLength()
                world_up = Gf.Vec3d(0, 0, 1)
                right = Gf.Cross(forward, world_up)
                if right.GetLength() < 1e-6:
                    right = Gf.Vec3d(1, 0, 0)
                else:
                    right = right / right.GetLength()
                up = Gf.Cross(right, forward)
                # World rotation matrix (row-vector convention, matches
                # this session's earlier proven look-at code): row i is
                # where local axis i ends up in world space. Camera looks
                # down local -Z, +X right, +Y up.
                world_rot = Gf.Matrix4d(
                    right[0], right[1], right[2], 0.0,
                    up[0], up[1], up[2], 0.0,
                    -forward[0], -forward[1], -forward[2], 0.0,
                    0.0, 0.0, 0.0, 1.0,
                )
                # Convert to the camera's LOCAL orientation by removing the
                # parent (robot base)'s own current world rotation - read
                # fresh, not assumed to be yaw-only.
                parent_rot_only = Gf.Matrix4d(parent_world.ExtractRotationMatrix(), Gf.Vec3d(0, 0, 0))
                local_rot = world_rot * parent_rot_only.GetInverse()
                local_quat = local_rot.ExtractRotationQuat()

                applied = False
                for op in cam_xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                        existing = op.Get()
                        if isinstance(existing, Gf.Quatf):
                            op.Set(Gf.Quatf(local_quat.GetReal(), Gf.Vec3f(*local_quat.GetImaginary())))
                        else:
                            op.Set(local_quat)
                        applied = True
                        break
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        euler = Gf.Rotation(local_quat).Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
                        op.Set(Gf.Vec3f(euler[0], euler[1], euler[2]))
                        applied = True
                        break
                if not applied:
                    cam_xformable.AddOrientOp().Set(local_quat)
                for _ in range(2):
                    _tick()
        if attempt_robot_scan(robot_state, robot_state["camera_path"], authorized_ids, banned_ids, alert,
                               heading_label=f"standpoint {step + 1}/{ORBIT_STANDPOINTS}", mover=mover):
            print(f"  [ROBOT] face search succeeded from standpoint {step + 1}/{ORBIT_STANDPOINTS}")
            return

    print(f"  [ROBOT] face search exhausted all {ORBIT_STANDPOINTS} orbit standpoints without a confirmed identification")
    append_log_entry({
        "event_type": "robot_response", "found": False,
        "alert_reason": alert.get("reason"), "alert_id": alert.get("alert_id"), "zone": alert.get("zone"),
        "note": "face search exhausted all orbit standpoints",
        "timestamp": datetime.now(timezone.utc).isoformat()
    })


def simplify_robot_visuals(stage, robot_prim_path, keep_path_substrings):
    """Hides every Mesh prim under the robot EXCEPT ones whose path
    contains one of keep_path_substrings (the camera mount chain, so you
    can still see it turn to look at people) - pure render-cost cut, zero
    functional impact, since none of this script's movement/rotation logic
    touches mesh geometry at all, only the base_footprint prim's transform
    ops. This is the concrete "make it a skeleton except the part that
    needs to rotate" request - a full mobile-manipulator mesh (wheels,
    chassis, sensor housings, etc.) is a lot of triangles to keep rendering
    every frame for a demo that doesn't need to look detailed.

    Hiding via UsdGeom.Imageable.MakeInvisible() rather than deleting/
    unloading - completely reversible (just call MakeVisible() later) and
    doesn't disturb the prim hierarchy or any transform ops.
    """
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"  [simplify_robot] robot prim not found at {robot_prim_path}, skipping.")
        return
    hidden = 0
    kept = 0
    for prim in Usd.PrimRange(robot_prim):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        path_str = str(prim.GetPath())
        if any(sub in path_str for sub in keep_path_substrings):
            kept += 1
            continue
        UsdGeom.Imageable(prim).MakeInvisible()
        hidden += 1
    print(f"  [simplify_robot] hid {hidden} mesh(es), kept {kept} mesh(es) matching {keep_path_substrings}.")


def create_lightweight_robot(stage):
    """Replaces the heavy mobile_manipulator_ros asset entirely. That asset
    dragged in a full ROS2/Nav2/PhysX-oriented rig (wheel joints, a
    differential controller, lidar sensors, a manipulator arm, broken TF
    frames) for a project that, by this point, uses none of that -
    movement has been fully kinematic for a while, and ROS2 was never
    actually driving anything by the end. Keeping that whole asset around
    just for its camera mount was real, unnecessary prim-count/render cost,
    on top of being the actual source of the 120-degree camera-mount-offset
    problem that took a whole session to reverse-engineer.

    Per explicit direction, the visible robot is now NVIDIA's official
    Nova Carter asset (not a hand-built placeholder primitive) - confirmed
    via a real live inspection (inspect_nova_carter.py / _physics.py) of
    its actual prim tree before touching anything here, not guessed:
      - real bounding box ~0.73m x 0.90m x 0.69m (x/y/z) - a genuinely
        small ground robot, not person-scale
      - physics: chassis_link (RigidBody+ArticulationRoot), 6 more rigid
        bodies (wheels/casters), 7 PhysicsRevoluteJoints
      - sensors: ALL living under one
        .../chassis_link/sensors subtree - 4 hawk stereo pairs (8
        cameras) + 4 owl cameras + 3 lidars + 5 IMUs
    None of the physics or sensors are used by this project (movement
    stays fully kinematic, exactly as before - see update_robot; and we
    use our OWN plain camera below for face search, not Nova Carter's) -
    both get stripped right after referencing, physics via the same
    disable_physics_recursive() this file already uses elsewhere, sensors
    via one SetActive(False) on their shared parent subtree.

    Since Nova Carter's real height (~0.69m) is far below the previous
    placeholder's person-height camera mount, a thin visual mast carries
    the camera up to the exact same CAMERA_HEIGHT as before instead of
    changing that constant - keeps every already-verified aiming
    calculation in search_for_face() untouched, and reads as a real,
    common design choice on actual patrol/security robots (a raised
    sensor mast), not an arbitrary floating camera.

    The camera itself is mounted at a rotation WE choose and control -
    specifically chosen so the camera's forward axis matches the EXACT
    SAME body-forward convention already used everywhere else in this file
    (atan2(dx, -dy), same as update_person_position/update_robot's
    movement-facing math). So "aim the camera at something" is now just
    "use the same yaw formula you'd use to face that direction" - no more
    special-cased offset math, anywhere, ever again for this asset.
    """
    CAMERA_HEIGHT = ROBOT_CAMERA_HEIGHT  # unchanged - see docstring for why the mast exists instead of moving this
    MAST_BASE_HEIGHT = 0.6  # meters - just above Nova Carter's real measured top (~0.55m)

    root_path = "/World/SimpleRobot"
    root_xform = UsdGeom.Xform.Define(stage, root_path)
    root_prim = root_xform.GetPrim()
    root_xform.AddTranslateOp()
    root_xform.AddRotateXYZOp()

    assets_root = get_assets_root_path()
    if assets_root is None:
        print("  WARNING: could not resolve Isaac Sim assets root - Nova Carter visual will be missing (no network/Nucleus access?). Robot will be an invisible root + camera + mast only.")
    else:
        nova_carter_usd = assets_root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        # BUG (crashed the whole sim on launch): adding our own centering
        # translate op directly onto the SAME prim that holds the
        # reference fails - referencing an asset onto a prim also pulls in
        # that asset's own authored xformOpOrder (translate/orient/scale),
        # so AddTranslateOp() here tried to add a second translate op and
        # USD raised "xformOp:translate already exists in xformOpOrder".
        # Fix: keep every op WE add on separate wrapping Xforms, never on
        # the same prim that holds the reference.
        #
        # BUG (reported live: "it's facing the wrong way" / "moves kinda
        # unnaturally") - confirmed via a direct sensor-position inspection
        # (inspect_nova_carter_forward.py: front_hawk/front_owl/front_
        # RPLidar all sit at positive local x, back_hawk/rear_RPLidar/the
        # caster wheel all sit at negative local x) that Nova Carter's own
        # forward direction is local +X. This project's whole body-facing
        # convention is local -Y at yaw=0 (matches the camera's default
        # look direction - see the camera setup below). Those two were
        # never reconciled, so the mesh always faced 90 degrees off from
        # its actual direction of travel - reads as "sliding" or "facing
        # the wrong way" while turning/moving, even though the underlying
        # movement math was always correct.
        #
        # Fixed with two separate single-op Xforms (nesting instead of
        # stacking ops on one prim also sidesteps needing to reason about
        # USD's op-order composition by hand a second time after already
        # getting that wrong once): an inner one rotates the mesh -90 deg
        # about Z (verified by hand: Rz(-90) maps local +X to local -Y,
        # exactly the needed correction), and an outer one re-centers the
        # ALREADY-ROTATED mesh - since rotating Nova Carter's own real
        # measured center offset (-0.22, 0, 0) by -90 deg about Z lands at
        # (0, 0.22, 0), the outer translate needed to bring that back to
        # the origin is (0, -0.22, 0), not the pre-rotation (0.22, 0, 0).
        rotate_path = root_path + "/nova_carter_rotate"
        rotate_xform = UsdGeom.Xform.Define(stage, rotate_path)
        rotate_xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, -90))

        offset_path = rotate_path + "/nova_carter_offset"
        offset_xform = UsdGeom.Xform.Define(stage, offset_path)
        offset_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.22, 0.0))

        visual_path = offset_path + "/nova_carter_visual"
        visual_prim = stage.DefinePrim(visual_path, "Xform")
        visual_prim.GetReferences().AddReference(nova_carter_usd)
        for _ in range(10):
            _tick()

        # BUG: this used to look for a CHILD named "nova_carter" under
        # visual_prim (i.e. visual_path + "/nova_carter") and always came
        # up empty. A USD reference composes the referenced layer's
        # default prim's own CHILDREN directly onto the referencing prim -
        # it does not nest them under a further child named after the
        # source's root prim. Nova Carter's default prim IS /nova_carter
        # in its own file, so referencing it here means visual_prim itself
        # now effectively IS that root (its children are chassis_link,
        # wheel_left, etc, directly) - confirmed by hand-checking how
        # AddReference composes, not guessed a second time after the first
        # guess (a same-prim double-translate-op) already caused one crash.
        nova_carter_root = visual_prim
        if nova_carter_root.IsValid():
            sensors_prim = stage.GetPrimAtPath(str(nova_carter_root.GetPath()) + "/chassis_link/sensors")
            if sensors_prim.IsValid():
                sensors_prim.SetActive(False)
            else:
                print(f"  WARNING: Nova Carter sensors subtree not found under {nova_carter_root.GetPath()} - sensor suite not stripped (unexpected asset layout change?).")
            disable_physics_recursive(nova_carter_root)
        else:
            print(f"  WARNING: Nova Carter root prim not found at {visual_path}/nova_carter - reference may have failed to resolve.")

    mast_path = root_path + "/mast"
    mast = UsdGeom.Cylinder.Define(stage, mast_path)
    mast.CreateHeightAttr(CAMERA_HEIGHT - MAST_BASE_HEIGHT)
    mast.CreateRadiusAttr(0.03)
    mast.CreateAxisAttr("Z")
    UsdGeom.Xformable(mast.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0, 0, MAST_BASE_HEIGHT + (CAMERA_HEIGHT - MAST_BASE_HEIGHT) / 2))

    camera_path = root_path + "/camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    cam_xform = UsdGeom.Xformable(camera.GetPrim())
    cam_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, CAMERA_HEIGHT))
    # A USD camera's default local look direction is -Z. Rotating -90 deg
    # about local X maps local -Z onto local -Y - i.e. at yaw=0 (root
    # unrotated), this camera looks toward world -Y, exactly matching
    # update_person_position's f0=(0,-1,0) body-forward convention. Verified
    # by hand (Rodrigues rotation of (0,0,-1) by -90 deg about X = (0,-1,0)),
    # not left to guesswork this time.
    # BUG (found by looking at real check-in frames - every desk scan was
    # an upside-down person, which YOLO can't see, hence "nobody in view"
    # until some orbit search happened to leave an upright look-at
    # orientation behind): rotating -90 deg about X does map the look
    # direction onto -Y as the comment above says, but it also sends the
    # camera's UP vector to world -Z, so the image was rendered upside
    # down. (90, 0, 180) looks along the same -Y with up = +Z.
    cam_xform.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, 180))

    print(f"Created Nova Carter-based robot at {root_path} (camera at {camera_path}).")
    return root_prim, camera_path


def update_robot(robot_state, elapsed, authorized_ids, banned_ids):
    """Fully kinematic robot movement - moves and rotates the robot prim
    directly, exactly like update_person_position() does for the patrol
    people, instead of watching real PhysX/Nav2-driven motion. No physics,
    no wheels, no ROS2/WSL dependency for movement at all.

    Behavior: stations at CHECKPOINT_POS (just inside the real doorway
    gap) by default and stays there, periodically identifying whoever's
    nearest - not cycling patrol waypoints. Every ROBOT_CHECK_INTERVAL
    seconds, checks the event log for a new unhandled dispatch alert - if
    one exists and isn't already the current target, it INTERRUPTS the
    checkpoint watch and heads there instead (higher priority). On
    arrival at an alert's coords, runs the same active face search as
    before, then returns to the checkpoint.
    """
    # search_for_face() calls _tick() internally (to keep rendering/physics
    # moving during the pause-then-scan sequence), which calls straight
    # back into this function. Without this guard, every one of those
    # inner calls saw no current_target (cleared right before entering the
    # search, to stop a different recursion bug) and picked a fresh patrol
    # waypoint - so the robot visibly walked away mid-search instead of
    # standing still to scan, and the alert-check below kept firing
    # "new dispatch alert" repeatedly during the same search. This makes
    # every nested call a complete no-op until the search actually finishes.
    if robot_state.get("in_search"):
        # Standing still on purpose (scan / orbit / assist / check-in). Keep
        # the stuck-check baseline fresh so that when the search ends, the
        # time spent stationary isn't read as "wedged against something" and
        # used to abandon the alert the robot was about to go chase (seen
        # live: "stuck near (37.31,18.79)" right after a check-in scan).
        robot_state["stuck_check_pos"] = None
        robot_state["stuck_check_time"] = elapsed
        return

    dt = min(elapsed - robot_state.get("last_move_time", elapsed), MAX_MOVE_DT)
    robot_state["last_move_time"] = elapsed

    # Periodically check for a new higher-priority alert to interrupt
    # patrol with - gated by ROBOT_CHECK_INTERVAL since this does file I/O
    # (find_next_unhandled_alert reads EVENT_LOG_FILE), unlike the actual
    # movement below which needs to run every single tick to be smooth.
    if elapsed - robot_state["last_check"] >= ROBOT_CHECK_INTERVAL:
        robot_state["last_check"] = elapsed
        alert = find_next_unhandled_alert(robot_state["handled_alert_ids"])
        # Someone is standing at the door waiting to be checked in: that is the
        # robot's actual job, so anything short of a fall waits (falls are
        # still first - find_next_unhandled_alert already prefers them). A
        # non-fall chase already under way is dropped without being marked
        # handled, so it is picked up again once the desk is clear.
        if checkin_pending(_tick_state.get("movers", [])):
            if alert is not None and alert.get("reason") != "fall":
                alert = None
            cur = robot_state.get("current_alert")
            if cur is not None and cur.get("reason") != "fall":
                robot_state["current_alert"] = None
                robot_state["current_target"] = None
        # BUG: this used to compare "alert is not robot_state.get('current_alert')"
        # - object identity, not logical identity. find_next_unhandled_alert
        # re-reads and re-parses the JSON log every call, so it returns a
        # FRESH dict instance each time even for the exact same underlying
        # alert entry - "is not" was true every single check, constantly
        # reprinting "new dispatch alert" and reassigning current_alert for
        # something already being chased. Compare by timestamp (the actual
        # unique identity of a log entry) instead.
        current = robot_state.get("current_alert")
        if alert is not None and (current is None
                                    or alert.get("alert_id", alert.get("timestamp")) != current.get("alert_id", current.get("timestamp"))):
            robot_state["current_alert"] = alert
            robot_state["current_target"] = tuple(alert["coords"])
            print(f"  [ROBOT] new dispatch alert - interrupting patrol, heading to {alert['coords']} ({alert['reason']})")

    # No target yet (first tick, or just finished handling something) -
    # head back to the checkpoint. Replaces the old ROBOT_WAYPOINTS cycle -
    # the robot's default job now is standing watch at the real doorway
    # gap and identifying whoever crosses, not wandering the whole
    # building hoping to stumble across someone (see CHECKPOINT_POS's
    # comment).
    if robot_state.get("current_target") is None:
        robot_state["current_target"] = CHECKPOINT_POS

    # THE REAL FIX for "the robot drives right past them and finds nothing":
    # current_target for a person-triggered alert used to be a ONE-TIME
    # coordinate snapshot taken the instant the alert fired. The person
    # keeps walking after that. By the time the robot crosses the room to
    # that now-stale point, they're gone - the robot arrives to an empty
    # spot every time, exactly what was happening. Fix: while chasing an
    # alert (current_alert is set), re-fetch the actual person's LIVE
    # position and overwrite current_target with it, every single tick -
    # continuous tracking, not a snapshot.
    if robot_state.get("current_alert") is not None:
        movers = _tick_state.get("movers", [])
        current_alert = robot_state["current_alert"]
        # An overstay alert always carries WHICH mover it's about
        # (person_id, tagged onto the mover by attempt_robot_scan at the
        # moment they were first identified - see VISIT_TIME_LIMIT) - a
        # real ID-based lookup, not a guess, so this tracks their live
        # position correctly regardless of how many other people are
        # also in the building. Falls back to the old "only safe with
        # exactly one mover total" shortcut for fall/loitering alerts,
        # which don't carry an identity yet (that's the whole reason a
        # search has to run in the first place).
        id_target = None
        if current_alert.get("person_id"):
            id_target = next((m for m in movers if m.get("known_id") == current_alert["person_id"]), None)
        elif current_alert.get("mover_name"):
            id_target = next((m for m in movers if m.get("name") == current_alert["mover_name"]
                               and not m.get("should_despawn")), None)
        # Use each mover's TRUE logical position, not get_translate(prim):
        # that prim's translate is deliberately offset-compensated for the
        # /model child's per-character offset (see set_person_translate),
        # so it can be several meters from where the person really is -
        # the overstay chase used to aim at that phantom point, arrive
        # "close enough" 7m from the person, or get stuck on a wall.
        def _live_xy(m):
            fallback = get_translate(m["prim"])
            return m.get("logical_x", fallback[0]), m.get("logical_y", fallback[1])
        if id_target is not None:
            live_x, live_y = _live_xy(id_target)
            robot_state["current_target"] = clamp_to_floor(live_x, live_y)
        elif len(movers) == 1:
            live_x, live_y = _live_xy(movers[0])
            robot_state["current_target"] = clamp_to_floor(live_x, live_y)

    pos = get_translate(robot_state["prim"])
    tx, ty = robot_state["current_target"]
    dist = math.hypot(tx - pos[0], ty - pos[1])  # arrival check is always against the REAL final target, never the doorway hop below

    # ARRIVAL_RADIUS (3.0m) is a deliberately generous "close enough, stop
    # a comfortable distance in front of them" tolerance for chasing a
    # PERSON - but the checkpoint target now sits inside a real physical
    # reception-counter cavity only about 1m wide (see CHECKPOINT_POS's
    # comment) - stopping within a 3m radius of its center would leave the
    # robot parked well short of ever actually reaching the counter at
    # all, defeating "make sure the robot sits inside of it." Use a much
    # tighter radius specifically when there's no active alert (i.e. the
    # current target IS the checkpoint, not a person).
    CHECKPOINT_ARRIVAL_RADIUS = 0.5
    effective_arrival_radius = ARRIVAL_RADIUS if robot_state.get("current_alert") is not None else CHECKPOINT_ARRIVAL_RADIUS

    # Same stuck-safety net as the people's own wander/loiter movement
    # (see update_person_position's comment for the real underlying bug,
    # still not fully root-caused) - confirmed live that the ROBOT can
    # hit the exact same near-wall-boundary crawl too (pinned at x=22.40
    # for many consecutive real seconds, distance-to-target never
    # shrinking), which is worse for the robot than for an ambient
    # wanderer since a permanently stuck robot can never reach ANY future
    # alert again. If stuck, abandon whatever it's currently doing -
    # advance to the next patrol waypoint, and if it was chasing a
    # dispatch alert, mark that alert handled (so it doesn't just
    # re-select the same unreachable target next check) and clear it so
    # patrol resumes.
    # BUG (found live after the checkpoint-patrol redesign: the log filled
    # with "stuck near (35.00,18.45) heading to (38.00,18.43) - abandoning"
    # repeating forever, even though checkpoint scans kept succeeding right
    # alongside it): this check used to run safely because the old
    # ROBOT_WAYPOINTS patrol never had the robot legitimately standing
    # still for more than an instant - arrival always immediately advanced
    # to the next waypoint. Now the robot's default state IS standing
    # still at the checkpoint (ARRIVAL_RADIUS=3.0 means "close enough"
    # stops it a few meters short of CHECKPOINT_POS's exact coordinate,
    # then it correctly never moves again) - real correct behavior, not a
    # failure to progress. Skip the stuck check entirely whenever already
    # within ARRIVAL_RADIUS of the target: "not moving because you've
    # arrived" and "not moving because you're wedged against something"
    # are different states, and only the second one should ever trigger
    # abandon-and-move-on.
    ROBOT_STUCK_CHECK_INTERVAL = MOVER_STUCK_CHECK_INTERVAL
    ROBOT_STUCK_MIN_PROGRESS = MOVER_STUCK_MIN_PROGRESS
    check_pos = robot_state.get("stuck_check_pos")
    check_time = robot_state.get("stuck_check_time")
    if dist <= effective_arrival_radius:
        robot_state["stuck_check_pos"] = (pos[0], pos[1])
        robot_state["stuck_check_time"] = elapsed
    elif check_time is None or elapsed - check_time >= ROBOT_STUCK_CHECK_INTERVAL:
        if check_pos is not None and math.hypot(pos[0] - check_pos[0], pos[1] - check_pos[1]) < ROBOT_STUCK_MIN_PROGRESS:
            print(f"  [ROBOT] stuck near ({pos[0]:.2f},{pos[1]:.2f}) heading to ({tx:.2f},{ty:.2f}) - abandoning and moving on")
            alert = robot_state.get("current_alert")
            if alert is not None:
                robot_state["handled_alert_ids"].add(alert.get("alert_id", alert["timestamp"]))
            robot_state["current_alert"] = None
            robot_state["current_target"] = None  # re-picked as CHECKPOINT_POS above next tick
            robot_state["stuck_check_pos"] = None
            robot_state["stuck_check_time"] = elapsed
            return
        robot_state["stuck_check_pos"] = (pos[0], pos[1])
        robot_state["stuck_check_time"] = elapsed

    if dist <= effective_arrival_radius:
        alert = robot_state.get("current_alert")
        # Use the LIVE target position at the moment of arrival as the
        # orbit center, not the alert's original stale coordinate snapshot
        # - for a loitering/unauthorized alert the person may have kept
        # moving during the chase (current_target was continuously updated
        # above to track them), so by arrival time current_target IS their
        # real current position and alert["coords"] is outdated. For a
        # fall alert they're frozen so both are the same anyway - this is
        # strictly more correct in both cases, never worse.
        if alert is not None:
            alert = dict(alert)
            alert["coords"] = robot_state["current_target"]
        # Clear state BEFORE calling search_for_face, not after - real bug
        # found here: search_for_face() calls _tick() internally (to keep
        # rendering/physics moving during the pause-then-scan sequence),
        # and _tick() calls straight back into update_robot(). With the
        # old order (clear state after search_for_face returns), every one
        # of those inner _tick() calls saw the robot still "arrived" with
        # current_alert still set, and called search_for_face() AGAIN from
        # inside itself - infinite recursion until Python's stack blew up.
        # Clearing current_alert/current_target first means those inner
        # _tick() calls see no active alert and just fall through to
        # normal movement/patrol logic instead of re-triggering the search.
        robot_state["current_alert"] = None
        robot_state["current_target"] = None
        if alert is not None and alert.get("reason") == "fall":
            # Face identification still never happens DURING the fall
            # itself - DeepFace can't get a clean face crop from this
            # project's real fall pose (the head tucks down against the
            # chest, hidden from every camera angle a ground robot can
            # reach - confirmed via a direct test) and forcing it through
            # risks misidentifying people. What matters first is getting
            # the robot there and getting the person back up. Identity IS
            # captured AFTER they're standing again, for the injury log -
            # see assist_fallen_person()'s second half.
            #
            # BUG (reported live: "the robot needs to stay there... that
            # would make more sense" - i.e. it visibly wasn't) -
            # current_target got cleared right above, but nothing
            # protected the robot from reassigning itself during
            # assist_fallen_person's OWN internal _tick() calls (the
            # real-time wait for the get-up animation to finish, before
            # search_for_face() even starts and sets its own in_search
            # guard). Every one of those inner ticks re-entered
            # update_robot(), saw current_target is None, and immediately
            # picked a fresh patrol waypoint - the robot was actually
            # walking away the whole time it looked like it was "helping."
            # Wrapping the ENTIRE call (not just search_for_face's own
            # portion inside it) in the same in_search guard the loitering
            # branch below already uses keeps the robot genuinely
            # stationary near the fallen person for the whole assist,
            # not just conceptually assigned to the task.
            robot_state["in_search"] = True
            try:
                assist_fallen_person(robot_state, alert, _tick_state.get("movers", []), authorized_ids, banned_ids, elapsed)
            finally:
                robot_state["in_search"] = False
            robot_state["handled_alert_ids"].add(alert.get("alert_id", alert["timestamp"]))
        elif alert is not None and alert.get("reason") == "overstay":
            # Real escort dispatch per explicit direction ("if the time
            # limit is too high let's just do 1 minute for now... if they
            # go over 1 minute then the robot go finds them and escorts
            # them away") - see the overstay-detection block in main()'s
            # loop for where this alert gets created. No identification
            # needed here - we already know exactly who this is
            # (alert["person_id"]), that's the whole reason the alert
            # fired in the first place. Look the mover up by the known_id
            # tag attempt_robot_scan sets on successful identification,
            # then force them into the SAME exit state machine banned-
            # person escort and ordinary visitor exit already both use.
            movers = _tick_state.get("movers", [])
            target_mover = next((m for m in movers if m.get("known_id") == alert.get("person_id")
                                  and not m.get("should_despawn")), None)
            if target_mover is not None and (target_mover.get("fallen") or target_mover.get("standing_up")):
                # Someone on the floor needs helping up, not escorting: leave them
                # (forcing them to stand would cancel a real fall unassisted) and
                # let the overstay check raise this again once they are back on their feet.
                target_mover["overstay_dispatched"] = False
                print(f"  [ROBOT] {target_mover['name']} is down - helping comes before escorting; will escort once they are up.")
            elif target_mover is not None:
                print(f"  [ROBOT] *** {target_mover['name']} ({alert.get('person_id')}) has overstayed "
                      f"the {VISIT_TIME_LIMIT:.0f}s visit limit - escorting out ***")
                append_log_entry({
                    "event_type": "overstay_escort", "person_id": alert.get("person_id"),
                    "zone": alert.get("zone"), "coords": robot_state["current_target"],
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                })
                target_mover["paused_until"] = 0
                target_mover["is_visitor"] = True
                target_mover["visitor_state"] = "exiting"
                target_mover["wander_waypoint"] = DOOR_GAP_WAYPOINT
                target_mover["visitor_exit_final"] = False
            else:
                print(f"  [ROBOT] arrived to escort out an overstayer but couldn't find them anymore.")
            robot_state["handled_alert_ids"].add(alert.get("alert_id", alert["timestamp"]))
        elif alert is not None:
            # REAL FIX for consistent 8/8 orbit failures: nothing was
            # stopping the person from continuing to walk their patrol
            # route WHILE the robot spent real seconds moving between all
            # 8 orbit standpoints. The orbit center was fixed at arrival,
            # but by standpoint 3-4 the person had already walked away
            # from it - every single search was chasing a target that kept
            # moving out from under it.
            #
            # BUG (reported live: "people who freeze like im watching this
            # guy just stand there walking and not moving an inch...
            # keeps happening") - this used to freeze EVERY mover
            # (paused_until = elapsed+300, an early-return in
            # update_person_position that skips movement entirely) for
            # the whole search, not just the one actually being searched
            # for. The walk-cycle ANIMATION isn't driven by that same
            # per-tick code at all - it loops continuously on the Kit
            # timeline regardless of whether position updates are
            # paused - so every OTHER ambient person in the building
            # visibly played their walk animation in place without
            # translating for the entire multi-standpoint search (each
            # standpoint is a real synchronous YOLO+DeepFace round-trip,
            # so this could run many real seconds), which is exactly the
            # reported symptom. Only the actual search target needs to
            # hold still - find_next_unhandled_alert/get_nearest_mover_coords
            # already identify who an alert is really about via its
            # coords, the same way the intruder-escort logic elsewhere
            # picks "nearest mover to a point" - reuse that instead of a
            # blanket freeze so everyone else keeps walking normally
            # while the robot searches.
            movers = _tick_state.get("movers", [])
            target_mover = None
            if alert.get("mover_name"):
                target_mover = next((m for m in movers if m.get("name") == alert["mover_name"]
                                      and not m.get("should_despawn")), None)
            if target_mover is None and movers:
                # No named mover (an older alert format, or they already despawned) -
                # fall back to nearest-to-the-alert's-coords, same as before.
                target_mover = min(movers, key=lambda m: (
                    (m.get("logical_x", get_translate(m["prim"])[0]) - alert["coords"][0]) ** 2 +
                    (m.get("logical_y", get_translate(m["prim"])[1]) - alert["coords"][1]) ** 2
                ))
            saved_pause = target_mover.get("paused_until", 0) if target_mover else None
            was_fallen = bool(target_mover.get("fallen")) if target_mover else False
            if target_mover is not None:
                target_mover["paused_until"] = elapsed + 300  # comfortably longer than any search will take
            robot_state["in_search"] = True
            robot_state["last_identified_id"] = None  # see the loitering-ban check below - must not see a stale ID from a DIFFERENT earlier search
            try:
                search_for_face(robot_state, alert, authorized_ids, banned_ids, mover=target_mover)
            finally:
                robot_state["in_search"] = False
                if target_mover is not None:
                    # BUG (seen live: a real scripted fall went undetected because the
                    # person stood back up ~5 s later): if their scripted fall began
                    # DURING this search, the fall had already replaced the freeze with
                    # its own lie-down pause - restoring the pre-search value here wiped
                    # it, and they got straight back up. Only restore when nothing
                    # replaced the freeze in the meantime.
                    if target_mover.get("fallen") and not was_fallen:
                        pass
                    else:
                        target_mover["paused_until"] = saved_pause
            # Per explicit direction ("add loitering tracker like if they
            # loitered once they get banned") - this branch only ever handles
            # "loitering" alerts in practice (fall and overstay are both
            # peeled off into their own branches above), but the reason is
            # checked explicitly anyway so this can't misfire if that ever
            # changes.
            #
            # BUG (caught live before this ever shipped - not a real bug
            # report, a direct observation while testing the first version of
            # this feature): an instant one-strike ban banned BOTH residents
            # within the first 95s of a run - ambient wandering trips the
            # loitering threshold often (see the "Known limitations" section),
            # so a one-strike policy meant every single check-in for the rest
            # of the run was an automatic, permanent denial - the building
            # emptied out and stayed empty. A real tracker - counting
            # loitering incidents per ID and only banning after several - is
            # both what "tracker" actually implies and what keeps the demo
            # showing a mix of normal activity and enforcement instead of one
            # then the other. Every incident still gets a real, visible
            # consequence (escorted out) even before the ban threshold - only
            # the BAN itself is deferred. Because identity/ban state resets
            # fresh at the start of every run (see main()'s startup reset), a
            # run can't permanently run out of people to a backlog of old
            # bans the way it would if that state persisted forever.
            if alert.get("reason") == "loitering":
                loiter_id = robot_state.get("last_identified_id")
                if loiter_id and target_mover is not None and not target_mover.get("should_despawn"):
                    if loiter_id in banned_ids:
                        # Already banned from an earlier incident this run - still
                        # escorted out every time, just no repeat ban/log noise.
                        print(f"  [ROBOT] {target_mover['name']} ({loiter_id}) is already banned - escorting out")
                    else:
                        # BUG (found live: a resident hit the strike threshold within
                        # UNDER A SECOND of real time despite the per-person dispatch
                        # dedup in run_sweep) - dedup only stops a SECOND alert from
                        # being CREATED for the same ongoing episode; it doesn't stop
                        # several already-created, already-legitimate alerts from a
                        # BACKLOG (the robot busy with something else for a while)
                        # from all getting worked through in a rapid real-time burst
                        # once it's free - each one genuinely was a separate creation
                        # event, but processed back-to-back they don't read as
                        # separate INCIDENTS the way a person would judge them. Real
                        # fix: require actual real-world time to have passed since
                        # the last COUNTED strike, enforced right here at the count
                        # site - immune to how bursty the robot's own queue
                        # processing happens to be.
                        strike_times = _tick_state.setdefault("loiter_strike_times", {})
                        last_strike = strike_times.get(loiter_id, -1e9)
                        if time.time() - last_strike < LOITER_MIN_SECONDS_BETWEEN_STRIKES:
                            # Same incident as the one just counted, not a new one -
                            # still escorted out below like every other case here,
                            # just doesn't advance the strike count.
                            print(f"  [ROBOT] {target_mover['name']} ({loiter_id}) caught loitering again so "
                                  f"soon after the last one - same incident, not a new strike - escorting out")
                        else:
                            strike_times[loiter_id] = time.time()
                            strikes = _tick_state.setdefault("loiter_strikes", {})
                            strikes[loiter_id] = strikes.get(loiter_id, 0) + 1
                            count = strikes[loiter_id]
                            if count >= LOITER_STRIKES_BEFORE_BAN:
                                banned_ids.append(loiter_id)
                                with open(BANNED_IDS_FILE, "w") as f:
                                    json.dump({"banned": banned_ids}, f, indent=2)
                                print(f"  [ROBOT] *** {loiter_id} flagged as banned - caught loitering {count} times ***")
                                append_log_entry({
                                    "event_type": "intruder_alert", "person_id": loiter_id,
                                    "zone": alert.get("zone"), "coords": alert.get("coords"),
                                    "timestamp": datetime.now(timezone.utc).isoformat()
                                })
                            else:
                                print(f"  [ROBOT] {target_mover['name']} ({loiter_id}) caught loitering "
                                      f"({count}/{LOITER_STRIKES_BEFORE_BAN} before a ban) - escorting out to cool off")
                    target_mover["paused_until"] = 0
                    target_mover["is_visitor"] = True
                    target_mover["visitor_state"] = "exiting"
                    target_mover["wander_waypoint"] = DOOR_GAP_WAYPOINT
                    target_mover["visitor_exit_final"] = False
            robot_state["handled_alert_ids"].add(alert.get("alert_id", alert["timestamp"]))
        else:
            # Parked at the checkpoint with nothing dispatched - this is
            # the default state. Explicitly face the door (not just
            # whatever direction it happened to arrive walking in) - per
            # explicit direction the robot now sits inside the reception
            # counter like an attendant, and an attendant faces the
            # entrance, not wherever their last few steps happened to
            # point.
            door_dx = DOOR_GAP_WAYPOINT[0] - CHECKPOINT_POS[0]
            door_dy = DOOR_GAP_WAYPOINT[1] - CHECKPOINT_POS[1]
            facing_yaw = math.degrees(math.atan2(door_dx, -door_dy))
            set_rotation(robot_state["prim"], (0.0, 0.0, facing_yaw))
            #
            # REDESIGNED per explicit direction ("when they come in
            # through the front door I need you to make it so they check
            # in with the robot and the robot scans them once to get
            # their id, i think its lagging too badly right now") - the
            # OLD version here scanned "whoever's nearest" on a fixed
            # timer, which (even after an earlier per-mover cooldown fix)
            # still scaled with however many ambient people simply
            # happened to be within CHECKPOINT_SCAN_RADIUS at any given
            # moment - with a busy population that was very often
            # "someone," so it kept firing real blocking YOLO/DeepFace
            # round-trips almost constantly, which is what was actually
            # producing the reported lag/stutter (on top of the separate
            # population-count issue - see VISITOR_SYSTEM_ENABLED and
            # PEOPLE above). This is now purely EVENT-DRIVEN and ONE-SHOT:
            # only ever scans a mover that is a visitor/entrant literally
            # standing at the door gap waiting to check in
            # (visitor_state == "entering", arrived, checked_in still
            # False) - not "anyone nearby." With no continuous visitor
            # churn running, that's genuinely nobody almost all the time,
            # not a constant stream.
            last_scan = robot_state.get("last_checkpoint_scan", -CHECKPOINT_SCAN_INTERVAL)
            if elapsed - last_scan >= CHECKPOINT_SCAN_INTERVAL:
                movers = _tick_state.get("movers", [])
                waiting = [m for m in movers
                           if m.get("is_visitor") and m.get("visitor_state") == "entering"
                           and not m.get("checked_in") and m.get("wander_waypoint") is None
                           and not m.get("should_despawn")]
                if waiting:
                    robot_state["last_checkpoint_scan"] = elapsed
                    entrant = waiting[0]
                    ex = entrant.get("logical_x", get_translate(entrant["prim"])[0])
                    ey = entrant.get("logical_y", get_translate(entrant["prim"])[1])
                    checkin_alert = {
                        "coords": (ex, ey), "zone": None, "reason": "checkin",
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    }
                    robot_state["last_identified_id"] = None
                    robot_state["in_search"] = True
                    try:
                        reset_robot_camera(robot_state)
                        scanned = attempt_robot_scan(robot_state, robot_state["camera_path"], authorized_ids, banned_ids,
                                                      checkin_alert, heading_label=f"check-in: {entrant['name']}",
                                                      mover=entrant)
                        if not scanned:
                            # The desk's view of the doorway is partly blocked by
                            # the counter's own back panel, so a person standing in
                            # the gap is often not in frame from here. Step out and
                            # get a clean look (same orbit the loitering response
                            # uses) instead of retrying the same blocked view.
                            checkin_alert["zone"] = "the door"
                            search_for_face(robot_state, checkin_alert, authorized_ids, banned_ids, mover=entrant)
                            scanned = robot_state.get("last_identified_id") is not None
                    finally:
                        robot_state["in_search"] = False
                    if scanned:
                        identified_id = robot_state.get("last_identified_id")
                        # is_intruder means this mover is KNOWN to be the
                        # bad actor by construction (see its spawn in
                        # main()) - real direction was "the red person is
                        # supposed to be turned away at the door", which
                        # means denial can't depend on a human having
                        # already flagged him from some earlier encounter
                        # (this IS his first encounter). So whatever ID
                        # the scan assigns him gets auto-flagged as banned
                        # right here, mirroring what flag_id_as_banned.py
                        # would do by hand after the fact, except
                        # immediate - the system already knows.
                        deny = entrant.get("is_intruder") or (identified_id in banned_ids)
                        if deny:
                            if entrant.get("is_intruder") and identified_id not in banned_ids:
                                banned_ids.append(identified_id)
                                with open(BANNED_IDS_FILE, "w") as f:
                                    json.dump({"banned": banned_ids}, f, indent=2)
                                print(f"  [ROBOT] *** {identified_id} auto-flagged as banned - known intruder ***")
                            print(f"  [ROBOT] *** {entrant['name']} ({identified_id}) DENIED entry - "
                                  f"turned away at the door ***")
                            append_log_entry({
                                "event_type": "intruder_alert", "person_id": identified_id,
                                "zone": None, "coords": (ex, ey),
                                "timestamp": datetime.now(timezone.utc).isoformat(),
                            })
                            entrant["checked_in"] = True
                            # Per explicit direction ("give them time to react, the
                            # robot turns them away immediately i want to see it...
                            # play the animation and then they walk out so people
                            # can see that theirs something wrong with them") - a
                            # visible reaction beat before they turn to leave, then
                            # the walk out itself uses a sad walk clip instead of
                            # the normal one. The reaction beat plays the angry
                            # clip (per explicit direction - "use the angry one...
                            # instead of the sad one when getting rejected because
                            # it looks better"; the sad-idle clip this replaced
                            # here is unused for the reaction now, but the sad WALK
                            # clip stays for the walk-out itself per direction -
                            # "when walkign away use the sad one"). Falls back to
                            # the old instant walk-out if this character has
                            # neither an angry nor a sad-idle clip bound (the
                            # pipeline scripts haven't been run against it yet).
                            reaction_clip = entrant.get("angry_clip_path") or entrant.get("sad_idle_clip_path")
                            if reaction_clip is not None and entrant.get("skel_prim") is not None:
                                set_animation_clip(entrant["skel_prim"], reaction_clip)
                                # BUG (found live: the reaction pose was overwritten and
                                # walking resumed only ~300ms later, not after the full
                                # reaction beat) - paused_until is ALSO written by the
                                # generic dispatch-alert search branch above, which saves
                                # a mover's paused_until, holds it for the duration of an
                                # unrelated search (loitering etc), then restores the
                                # SAVED value - if that mover happens to be this same
                                # entrant (nearest-to-some-other-alert's-coords, pure
                                # coincidence), its restore silently wipes out the value
                                # just set here. A dedicated field sidesteps that shared-
                                # state race entirely rather than trying to coordinate two
                                # independent writers of the same one.
                                # BUG (found live: the reaction never actually held - denial_reaction_until
                                # was already IN THE PAST the instant it was set): `elapsed` is a snapshot
                                # taken once when update_robot() was first entered for this tick, but by
                                # THIS line the check-in scan (and, when the desk's own view failed, a full
                                # multi-standpoint orbit-search fallback - each standpoint a real multi-
                                # second move+scan) can have consumed several more real seconds - `elapsed`
                                # itself never got refreshed across that, so computing the deadline from it
                                # here based the countdown on a clock reading that was already stale by the
                                # time this ran. Movers, by contrast, always read a FRESH elapsed at the top
                                # of every single _tick() call throughout that same scan, so they raced
                                # ahead of a deadline based on old time. Recomputing here from the real
                                # clock, exactly like movers do, is the actual fix.
                                fresh_elapsed = time.time() - _tick_state["sim_start"]
                                entrant["denial_reaction_until"] = fresh_elapsed + DENIAL_REACTION_SECONDS
                                entrant["pending_denied_exit"] = True
                            else:
                                entrant["visitor_state"] = "exiting"
                                entrant["wander_waypoint"] = DOOR_POS
                                entrant["visitor_exit_final"] = True  # already at the gap - skip the return-to-gap leg
                        else:
                            print(f"  [ROBOT] {entrant['name']} ({identified_id}) checked in - cleared to enter")
                            entrant["checked_in"] = True
                    # else: no clear face this attempt - stays put,
                    # checked_in still False, retried after another
                    # CHECKPOINT_SCAN_INTERVAL (throttled by last_scan
                    # above, not retried every single tick).
        return

    # Move toward the NEXT HOP (doorway detour if crossing rooms, else the
    # real target directly - see get_next_hop) at ROBOT_MOVE_SPEED, capped
    # so a big dt (e.g. right after a long blocking sweep call) can't
    # overshoot past it and start oscillating.
    hop_x, hop_y = get_next_hop(pos[0], pos[1], tx, ty, hysteresis_state=robot_state)
    dx = hop_x - pos[0]
    dy = hop_y - pos[1]
    hop_dist = math.hypot(dx, dy)
    step_dist = min(ROBOT_MOVE_SPEED * dt, hop_dist)
    if hop_dist > 0:
        raw_x = pos[0] + dx / hop_dist * step_dist
        raw_y = pos[1] + dy / hop_dist * step_dist
        new_x, new_y = move_with_wall_check(pos[0], pos[1], raw_x, raw_y)  # real wall-collision enforcement, not decorative - see move_with_wall_check's comment
        new_world_pos = (new_x, new_y, robot_state["base_height"])
        # Same fast-spin fix as update_person_position's wander step (see
        # its comment) - near a doorway crossing, get_next_hop's "mid"
        # branch can hand back a hop target only millimeters away, and at
        # that scale per-tick float noise flips dx's sign each frame,
        # making atan2(dx, -dy) swing wildly - confirmed live, the robot
        # did this too. Only re-face when the hop vector is large enough
        # to trust its direction.
        MIN_YAW_UPDATE_DIST = 0.08
        if hop_dist > MIN_YAW_UPDATE_DIST:
            yaw_deg = math.degrees(math.atan2(dx, -dy))  # same forward-axis convention as update_person_position
            set_rotation(robot_state["prim"], (0.0, 0.0, yaw_deg))
        set_world_translate(robot_state["prim"], new_world_pos)  # NOT set_translate() - see set_world_translate()'s comment for why

    if elapsed - robot_state.get("last_move_print", -999) >= 3.0 and dt > 0:
        robot_state["last_move_print"] = elapsed
        print(f"  [ROBOT] moving: at ({pos[0]:.2f}, {pos[1]:.2f}), "
              f"target ({tx:.2f}, {ty:.2f}), dist={dist:.2f}m")


# ---------- Tracking helpers ----------

def get_next_id():
    if not os.path.exists(ID_COUNTER_FILE):
        next_id = 1
    else:
        with open(ID_COUNTER_FILE, "r") as f:
            next_id = int(f.read().strip())
    with open(ID_COUNTER_FILE, "w") as f:
        f.write(str(next_id + 1))
    return next_id


def load_authorized_ids():
    if not os.path.exists(AUTHORIZED_IDS_FILE):
        return []
    with open(AUTHORIZED_IDS_FILE, "r") as f:
        data = json.load(f)
    # Tolerate a bare list (hand-edited/reset file) as well as {"authorized": [...]}.
    return data if isinstance(data, list) else data.get("authorized", [])


def load_banned_ids():
    if not os.path.exists(BANNED_IDS_FILE):
        return []
    with open(BANNED_IDS_FILE, "r") as f:
        data = json.load(f)
    # Tolerate a bare list (hand-edited/reset file) as well as {"banned": [...]}.
    return data if isinstance(data, list) else data.get("banned", [])


_next_alert_id = [0]  # list for closure-friendly mutability; see next_alert_id()


def next_alert_id():
    """A real unique identifier for a dispatch_alert - see find_next_unhandled_alert's
    own comment for why relying on the alert's own ISO timestamp string as its
    identity is not actually safe: multiple genuinely different people can cross a
    threshold (loitering, especially) in the SAME sweep batch and get logged with
    the IDENTICAL timestamp string down to the microsecond - confirmed live via a
    real run's event_log.json (three separate loitering alerts, three different
    people, three different zones, one shared timestamp). Since handled_timestamps
    only tracked timestamp strings, marking ONE of those as handled silently made
    its same-timestamp siblings look already-handled too, even though the robot
    never actually looked for them - dispatches were quietly disappearing. A plain
    incrementing counter can never collide."""
    _next_alert_id[0] += 1
    return _next_alert_id[0]


def append_log_entry(entry):
    log = []
    if os.path.exists(EVENT_LOG_FILE):
        with open(EVENT_LOG_FILE, "r") as f:
            try:
                log = json.load(f)
            except json.JSONDecodeError:
                log = []
    log.append(entry)
    with open(EVENT_LOG_FILE, "w") as f:
        json.dump(log, f, indent=2)


_camera_cache = {}


_zone_reference_frames = {}  # camera_path -> grayscale empty-room reference frame, for background-subtraction detection (see scan_zone_camera)


_camera_pause_supported = True  # flips to False if .pause()/.resume() ever throw, so we stop trying and stop pretending it's helping


def capture_frame(camera_path, resolution=(320, 240)):
    # Reuse one Camera object per path instead of creating a new one (and a
    # new render product/annotator) on every single scan call - doing that
    # every 8s for 5 cameras leaked render products, overran internal render
    # buffers ("sequence size exceeds remaining buffer" spam), and stalled
    # the physics step rate badly enough to starve the ROS2 odom publisher.
    global _camera_pause_supported
    camera = _camera_cache.get(camera_path)
    just_created = camera is None
    if camera is None:
        camera = Camera(prim_path=camera_path, resolution=resolution)
        camera.initialize()
        # Set aperture to match this resolution's aspect ratio up front -
        # otherwise Isaac Sim silently auto-corrects it (and logs a warning)
        # every time. Real fix instead of just suppressing the warning.
        aspect = resolution[0] / resolution[1]
        camera.set_horizontal_aperture(2.0955)
        camera.set_vertical_aperture(2.0955 / aspect)
        _camera_cache[camera_path] = camera
        for _ in range(10):
            simulation_app.update()

    # REAL FIX for the "robot crawls at 0.1 m/s" ceiling: once a camera got
    # cached above, it was left permanently active for the rest of the run -
    # every one of the 7 cameras in this scene (5 zones + checkpoint + robot)
    # was rendering its own full render product on EVERY simulation_app.update()
    # call, including the 90%+ of wall-clock time between sweeps when the
    # robot/people should be moving at full speed and nothing is even being
    # captured. That's the actual "GPU render cost / 6+ simultaneous render
    # products" ceiling from NOTES.md - it isn't a fixed hardware wall, it's
    # 6 idle cameras rendering for no reason. Nav2's vx_max and the
    # differential_controller node are both already maxed (10.0 m/s) -
    # raising either further does nothing, because the bottleneck is real
    # SIMULATED seconds per REAL second (see the [PERF] updates/sec log
    # line), not the commanded velocity.
    #
    # Fix: only the camera actively being captured stays "resumed" (its
    # render product ticking); every cached camera goes back to .pause()
    # immediately after its frame is grabbed, so idle cameras cost ~nothing
    # per tick instead of a full render each. Camera.pause()/.resume() are
    # real public isaacsim.sensors.camera.Camera methods, but wrapped
    # defensively anyway per this project's own convention - if this Isaac
    # Sim build's version behaves differently, it fails safe (frames keep
    # working exactly as before, just without the speed gain) instead of
    # breaking capture. Watch the [PERF] updates/sec line after this change -
    # that's the real test, not just "did the robot look faster."
    if _camera_pause_supported and not just_created:
        try:
            camera.resume()
        except Exception as e:
            _camera_pause_supported = False
            print(f"  [WARN] Camera.resume() not supported on this build ({e}) - "
                  f"disabling the pause/resume optimization, falling back to always-on cameras.")

    rgba = None
    for i in range(20):  # cut from 60 - each iteration in this loop is a full forced real render at full window resolution; most successful captures land in the first few iterations anyway, this just caps the worst-case lag spike from a slow/stuck capture
        _tick(force_render=True)  # this loop actually needs a real frame back every time, unlike every other _tick() caller in this file
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            break

    if _camera_pause_supported:
        try:
            camera.pause()
        except Exception as e:
            _camera_pause_supported = False
            print(f"  [WARN] Camera.pause() not supported on this build ({e}) - "
                  f"disabling the pause/resume optimization, falling back to always-on cameras.")

    if rgba is None or rgba.size == 0:
        return None

    rgb = rgba[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


class PersistentWorker:
    """Keeps a single long-running python.bat subprocess alive for the
    whole session, communicating via stdin/stdout - one request per line
    in, one JSON response per line out. Used for yolo_worker.py and
    deepface_worker.py, which load their (expensive) models ONCE at
    startup instead of on every single call, unlike run_subprocess()'s
    spawn-fresh-every-time pattern (still used elsewhere for cheap one-off
    scripts). This was the dominant real cost behind the whole "everything
    is so slow" investigation - not render resolution, not GPU rendering
    contention (both real, but secondary) - full model reloads happening
    5-6 times per single 8-second sweep cycle, every sweep, for the entire
    session.

    Reads happen on a background thread into a queue rather than a
    blocking readline() in the main thread, so the caller can keep calling
    _tick() (physics/movement/rendering) while waiting for a response,
    same pattern as run_subprocess()'s own polling loop.
    """

    def __init__(self, script_name):
        import threading
        import queue as _queue
        self.script_name = script_name
        self._threading = threading
        self._queue_mod = _queue
        self.ready = False
        self._spawn()

    def _spawn(self):
        script_path = os.path.join(PROJECT_DIR, self.script_name)
        self.proc = subprocess.Popen(
            ['C:\\isaacsim\\python.bat', script_path],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", errors="replace", bufsize=1
        )
        self.out_queue = self._queue_mod.Queue()
        self.reader_thread = self._threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def _read_loop(self):
        for line in self.proc.stdout:
            stripped = line.strip()
            if stripped:
                self.out_queue.put(stripped)

    def wait_ready(self, timeout=120):
        start = time.time()
        while time.time() - start < timeout:
            _tick()
            try:
                line = self.out_queue.get(timeout=0.05)
                if line == "READY":
                    self.ready = True
                    return True
            except self._queue_mod.Empty:
                continue
        return False

    def call(self, request_line, timeout=15):
        if self.proc.poll() is not None:
            print(f"  [WORKER] {self.script_name} had exited - restarting...")
            self._spawn()
            self.wait_ready()
        try:
            self.proc.stdin.write(request_line + "\n")
            self.proc.stdin.flush()
        except Exception as e:
            return {"error": f"failed to write to worker: {e}"}
        start = time.time()
        while time.time() - start < timeout:
            _tick()
            try:
                line = self.out_queue.get(timeout=0.05)
            except self._queue_mod.Empty:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
        # BUG (reported live: "my simulation keeps on freezing", confirmed
        # by directly inspecting the running process - the sim's main
        # process was burning real CPU the whole time via this method's
        # own _tick() calls in the wait loop above, which is why frames
        # kept rendering and it wasn't a TOTAL hang, but the worker
        # process itself sat at 0% CPU the entire time, genuinely wedged,
        # not crashed (still alive, just never producing a response - most
        # likely a desynced stdin/stdout protocol state, not something
        # worth fully root-causing here). The real problem was that a
        # timeout just gave up on THIS call and returned an error, but
        # left the SAME wedged process in place - every future scan
        # attempt against it paid another full timeout in a row, forever,
        # which is what actually produced a sustained, REPEATING freeze
        # instead of one bounded stall. Real fix: a timeout means this
        # worker is no longer trustworthy - kill it and start a fresh one
        # right away, so the next call gets a clean process instead of
        # joining the same jam.
        print(f"  [WORKER] {self.script_name} call timed out after {timeout}s - "
              f"killing and restarting the worker")
        try:
            self.proc.kill()
        except Exception:
            pass
        self._spawn()
        self.wait_ready()
        return {"error": "worker call timed out - worker restarted"}


# Module-level worker handles, populated once in main() before the sweep
# loop starts. Needed at module level (not local to main()) since
# scan_zone_camera/scan_checkpoint/attempt_robot_scan are all separate
# top-level functions that need to reach them.
_workers = {"yolo": None, "deepface": None}


def run_subprocess(script_name, args):
    script_path = os.path.join(PROJECT_DIR, script_name)
    arg_str = " ".join(f'"{a}"' for a in args)
    # Was subprocess.run() (fully blocking) - during that block,
    # simulation_app.update() never ran, so PhysX never stepped and Nav2's
    # /cmd_vel commands never actually got applied to the robot, even though
    # they kept arriving fine on the ROS2 wire (which is why `ros2 topic hz`
    # looked healthy while the robot barely moved). Using Popen + polling
    # keeps physics/rendering stepping the whole time this runs.
    #
    # IMPORTANT: this polling loop calls _tick() (movement + robot update +
    # simulation_app.update()), NOT just simulation_app.update() alone. A
    # single sweep runs 5 cameras x 2 subprocess calls (YOLO + DeepFace)
    # each taking multiple seconds, so this loop is where the vast majority
    # of each sweep's ~25-30s wall-clock time is actually spent. Previously
    # movement was computed only in main()'s outer loop, which is blocked
    # here for that entire duration - elapsed (wall-clock time.time()) kept
    # advancing the whole time regardless, so the very next movement update
    # after a sweep finished would jump straight to wherever ~25-30s of
    # progress along the patrol path landed, looking like a teleport. Now
    # movement is updated on every single simulation_app.update() call,
    # including all the ones spent waiting here, so it stays smooth
    # throughout a sweep instead of freezing then jumping.
    proc = subprocess.Popen(
        f'"C:\\isaacsim\\python.bat" "{script_path}" {arg_str}',
        shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding="utf-8", errors="replace"
    )
    while proc.poll() is None:
        _tick()
        time.sleep(0.01)
    stdout, stderr = proc.communicate()
    if proc.returncode != 0:
        return {"error": stderr}
    try:
        return json.loads(stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        return {"error": f"failed to parse output: {stdout}"}


def crop_person(bgr_image, detection, padding=25):
    h, w = bgr_image.shape[:2]
    x1 = max(0, int(detection["x1"]) - padding)
    y1 = max(0, int(detection["y1"]) - padding)
    x2 = min(w, int(detection["x2"]) + padding)
    y2 = min(h, int(detection["y2"]) + padding)
    return bgr_image[y1:y2, x1:x2]


def extract_appearance_signature(crop):
    """Computes a normalized HSV color histogram of the crop, used as a
    lightweight 'what are they wearing' fingerprint - a fallback identity
    signal for when face detection fails (common on wide zone cameras)."""
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], None, [8, 8, 8], [0, 180, 0, 256, 0, 256])
    hist = cv2.normalize(hist, hist).flatten()
    return hist.tolist()


def compare_appearance(sig1, sig2):
    h1 = np.array(sig1, dtype=np.float32)
    h2 = np.array(sig2, dtype=np.float32)
    return cv2.compareHist(h1, h2, cv2.HISTCMP_CORREL)


def load_appearance_profiles():
    if not os.path.exists(APPEARANCE_PROFILES_FILE):
        return {}
    with open(APPEARANCE_PROFILES_FILE, "r") as f:
        try:
            return json.load(f)
        except json.JSONDecodeError:
            return {}


def save_appearance_profile(person_id, crop):
    """Called whenever we have a CONFIRMED identity (from a face match/new
    at the checkpoint) - records what they're wearing so zone cameras can
    recognize them later even without a clean face shot."""
    profiles = load_appearance_profiles()
    profiles[person_id] = extract_appearance_signature(crop)
    with open(APPEARANCE_PROFILES_FILE, "w") as f:
        json.dump(profiles, f)


def find_appearance_match(crop, threshold=APPEARANCE_MATCH_THRESHOLD):
    """Compares this crop's color signature against all known appearance
    profiles. Returns (person_id, score) for the best match above threshold,
    or (None, None) if nothing matches closely enough."""
    profiles = load_appearance_profiles()
    if not profiles:
        return None, None

    sig = extract_appearance_signature(crop)
    best_id, best_score = None, threshold
    for pid, known_sig in profiles.items():
        score = compare_appearance(sig, known_sig)
        if score > best_score:
            best_id, best_score = pid, score
    return (best_id, best_score) if best_id else (None, None)


def check_for_fall(detection, threshold=1.3, max_ratio=None):
    """A standing person's bounding box is taller than wide. A fallen
    person's box becomes wider than tall (or, for the zone-camera
    background-subtraction path - see scan_zone_camera - closer to
    square: this project's actual mixamo_fall clip collapses into a
    curled/crumpled heap rather than a flat sprawled-out pose, measured
    at aspect ratio ~1.0-2.1, vs. ~0.35-0.40 for standing - so that call
    site passes a lower threshold than this default. The default here
    stays 1.3 for scan_checkpoint(), which still uses YOLO's own
    bounding box and hasn't been re-measured against this project's real
    fall pose).

    max_ratio (zone-camera path only) rejects anything ABOVE it as NOT a
    fall - confirmed via a real run (now that multiple people wander
    simultaneously) that two separate standing/walking people who happen
    to be near each other in frame merge into one background-subtraction
    contour whose combined bounding box is even WIDER than a real single
    fallen person's (measured 2.3-4.1 for two merged people, vs. 1.0-2.1
    for one real fallen person) - an upper bound catches this without
    needing to separate the contours."""
    width = detection["x2"] - detection["x1"]
    height = detection["y2"] - detection["y1"]
    if height == 0:
        return False
    aspect_ratio = width / height
    print(f"    (aspect ratio: {aspect_ratio:.2f}, fall threshold is {threshold}"
          f"{f', max {max_ratio}' if max_ratio else ''})")
    if max_ratio is not None and aspect_ratio > max_ratio:
        return False
    return aspect_ratio > threshold


def check_loitering(zone_name, streak_state, threshold=LOITER_THRESHOLD):
    """Tracks a live consecutive-presence streak per zone, in memory, across
    sweeps. A sweep that finds someone in the zone increments the streak;
    a sweep that finds nobody resets it to zero. This correctly captures
    'someone has been continuously here' rather than 'someone has been
    spotted here a few times' (which the old log-based version conflated,
    since empty sweeps were never logged and gaps between different people
    passing through looked identical to one person staying put).

    Also only fires the alert once per streak (not every sweep past the
    threshold) via an 'alerted' flag that resets alongside the streak.
    """
    if streak_state[zone_name]["streak"] >= threshold and not streak_state[zone_name]["alerted"]:
        streak_state[zone_name]["alerted"] = True
        return True
    return False


ZONE_BLOB_MIN_AREA_PX = 60  # minimum foreground-blob pixel area to count as "someone is here", at ZONE_SCAN_RESOLUTION - well above single-pixel antialiasing/dilation noise, well below a real person's footprint even at extreme zone-camera range (measured ~14x8px = 112px minimum at 1280x960 for a person ~20m away - see NOTES.md)
ZONE_FALL_ASPECT_THRESHOLD = 0.95  # raised from 0.75 - confirmed via a real run that continuous ambient WALKING (not just static standing) produces a genuinely wider mid-stride bounding box (measured 0.76-0.83, legs apart + arm swing) than pure standing's ~0.35-0.40, close enough to the old 0.75 threshold to false-positive every single sweep at a zone with someone just walking through. Real fall pose measures ~1.0-2.1 (curled heap) - 0.95 sits clearly between real walking's upper range and real fall's lower range
ZONE_FALL_ASPECT_MAX = 2.3  # see check_for_fall()'s max_ratio docstring - two merged nearby people measured 2.3-4.1, above the real single-fall range of ~1.0-2.1
FALL_CONFIRM_SWEEPS = 3  # bumped from 2 - confirmed live (adding a 4th character) that 2 reads ~1s apart still let some brief multi-person overlaps through; a real fall persists for many sweeps, a one-frame glitch doesn't - each extra read adds ~1s of real confirm delay (see the retry loop's own comment), still far faster than the original ~8s sweep-interval approach
ZONE_WARMUP_SWEEPS = 3  # how many of each camera's first live sweeps to spend re-baselining the reference frame (rather than detecting) - confirmed via two real failed runs that a dedicated pre-loop capture attempt (even retried up to 20x) can unreliably return None for every camera during early setup; discarding a few real sweeps' worth of results instead reuses the exact same code path that we know eventually works, and lets rendering fully settle before any detection is trusted
_zone_warmup_counts = {}  # camera_path -> how many CONFIRMED-EMPTY warm-up sweeps have been absorbed so far
_zone_warmup_attempts = {}  # camera_path -> how many warm-up sweeps have been tried in total (empty or not), caps the stall if a zone is always busy
_fall_alerted_zones = {}  # camera_path -> already fired a dispatch_alert for the CURRENT ongoing fall detection
_fall_camera_room = {}       # camera_path -> "RoomA"/"RoomB" (so a room's cameras can be de-duplicated against each other)
_last_fall_dispatch = {}     # room -> time.time() of the last fall dispatch_alert created for it
_last_fall_dispatch_coords = {}  # room -> (x, y, time.time()) of the last fall dispatch's target, for cross-room misattribution checks (see below)
_fall_dup_noted = {}          # camera_path -> a held duplicate detection was already announced this episode
_last_loiter_dispatch = {}    # mover_name -> time.time() of their last loitering dispatch (see the dedup above)
LOITER_DISPATCH_COOLDOWN = 45.0  # generous on purpose - a real loitering EPISODE naturally runs longer than a fall does, this only needs to outlast the gap between two cameras both catching the SAME episode
FALL_DISPATCH_COOLDOWN = 10.0  # short on purpose: the sibling-camera check covers real duplicates; a long cooldown let a false alarm swallow the next real fall (seen live: 24s later)
_zone_empty_streaks = {}  # camera_path -> consecutive not-detected reads, see the adaptive-reference BUG comment


# --- Zone-camera background subtraction robustness ---
# Found by pulling REFERENCE/RAW/MASK frames side by side (see the debug
# captures) while chasing why real scripted falls went undetected in most
# runs: the scene's glossy floor is lit differently than it was when the
# clean reference frame was captured (measured ~3x darker in one camera),
# so plain frame-vs-reference subtraction lit up the entire floor as one
# giant "person" blob (285k of ~1.2M pixels in RoomA_Cam1, permanently,
# every sweep). That blob won the "largest blob" contest every time, so
# the fall check only ever tested IT - never the real fallen person - and
# it also read as "someone present" forever (streaks in the hundreds).
ZONE_BLOB_MIN_FILL = 0.30      # blob area / bounding-box area; thin crescent streaks along floor-shading edges measured ~0.25-0.32, real people/heaps ~0.4-0.8
FALL_MIN_BLOB_AREA_PX = 300    # a curled fallen person is ~750px at 20m and ~330px at 30m at ZONE_SCAN_RESOLUTION - smaller compact specks aren't worth a fall test
FALL_CONFIRM_MAX_SHIFT_PX = 150  # the confirming re-read has to find the fall-shaped blob near the first one, not just any fall-shaped blob
# BUG (seen live: the robot arrived twice to "nobody down" right after real, correctly
# handled falls) - a normal WALKING gait occasionally reads as fall-shaped for one
# instant (aspect ratio briefly crosses ZONE_FALL_ASPECT_THRESHOLD mid-stride), and
# the confirm burst above was loose enough (150px shift allowed, no size check) to
# accept a walker who had simply taken another stride as "the same blob confirmed" -
# a real fallen person's blob does not move between reads a second apart, and its box
# size does not change; a walker's does both. Tight enough to reliably tell "held
# still" from "still walking, momentarily wide," generous enough to not reject a real
# fall for ordinary render/segmentation noise between two reads of a motionless body.
FALL_SHAPE_CONFIRM_MAX_SHIFT_PX = 25
FALL_SHAPE_CONFIRM_MAX_SIZE_DELTA = 0.20  # fraction of the larger dimension
ZONE_CHROMA_SAT = 60           # HSV saturation (0-255) above which a pixel counts as "coloured" - the walls/floor here are gray
ZONE_BLOB_MIN_CHROMA_PX = 20   # measured: real person blobs at range had 24-6000+ coloured px; the zero-colour floor residue had exactly 0
ZONE_BLOB_MIN_CHROMA_FRAC = 0.05  # ...and at least this fraction of the blob's area
_COMP_SCALE = 8
_COMP_SIGMA = 3.0


def _lighting_gain(cur_small, ref_small, weights):
    num = cv2.GaussianBlur(cur_small * weights, (0, 0), _COMP_SIGMA)
    den = cv2.GaussianBlur(ref_small * weights, (0, 0), _COMP_SIGMA)
    wsum = cv2.GaussianBlur(weights, (0, 0), _COMP_SIGMA)
    return (num + 4.0 * wsum) / (den + 4.0 * wsum + 1e-6), wsum


def compensate_reference(reference, gray):
    """Rescales the stored reference frame so its local brightness matches
    the CURRENT frame's, cancelling floor-lighting changes that aren't
    objects. Two passes: the first estimates the gain from everything;
    anything still different afterwards is a real object, so the second
    re-estimates the gain from background pixels ONLY - otherwise a person
    dilutes the gain around themselves and cancels part of their own diff
    (measured offline: a single pass shrank a real person's blob to half
    its height, the two-pass version left it within a few percent of the
    uncompensated size)."""
    h, w = reference.shape
    ref_small = cv2.resize(reference.astype(np.float32), (w // _COMP_SCALE, h // _COMP_SCALE), interpolation=cv2.INTER_AREA)
    cur_small = cv2.resize(gray.astype(np.float32), (w // _COMP_SCALE, h // _COMP_SCALE), interpolation=cv2.INTER_AREA)
    g1, _ = _lighting_gain(cur_small, ref_small, np.ones_like(ref_small))

    def apply(gain_small):
        gain = cv2.resize(np.clip(gain_small, 0.1, 8.0), (w, h), interpolation=cv2.INTER_LINEAR)
        return np.clip(reference.astype(np.float32) * gain, 0, 255).astype(np.uint8)

    first = apply(g1)
    fg = (cv2.absdiff(gray, first) > 25).astype(np.uint8)
    fg = cv2.dilate(fg, np.ones((25, 25), np.uint8))
    weights = np.clip(1.0 - cv2.resize(fg.astype(np.float32), (w // _COMP_SCALE, h // _COMP_SCALE),
                                       interpolation=cv2.INTER_AREA), 0.0, 1.0)
    g2, wsum = _lighting_gain(cur_small, ref_small, weights)
    return apply(np.where(wsum > 0.05, g2, g1))


def find_zone_blobs(gray, reference, bgr):
    """Foreground blobs in `gray` against the lighting-compensated
    reference. Returns (thresholded mask, candidate blobs sorted by area,
    largest first) - a candidate is big enough to count as "someone is
    here", solid enough not to be a floor-shading streak, AND contains real
    colour.

    The colour requirement is the discriminator that actually holds up:
    lighting compensation alone still let big irregular patches of glossy
    floor through (measured: blobs up to ~250,000px, one of which read as
    a "fallen person" and dispatched the robot to nobody, repeatedly). But
    this scene's walls and floor are achromatic gray while every person -
    whatever colour clothing they're wearing - has real saturated colour in
    frame. Measured over ~100 blobs: floor residue had exactly 0 coloured
    pixels, real people had a median ~46% coloured."""
    compensated = compensate_reference(reference, gray)
    diff = cv2.absdiff(gray, compensated)
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
    thresh = cv2.dilate(thresh, None, iterations=2)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    chroma = ((hsv[:, :, 1] > ZONE_CHROMA_SAT) & (hsv[:, :, 2] > 30)).astype(np.uint8)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < ZONE_BLOB_MIN_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w * h == 0 or area / float(w * h) < ZONE_BLOB_MIN_FILL:
            continue
        inside = np.zeros((h, w), np.uint8)
        cv2.drawContours(inside, [c], -1, 1, -1, offset=(-x, -y))
        chroma_px = int((chroma[y:y + h, x:x + w] & inside).sum())
        if chroma_px < ZONE_BLOB_MIN_CHROMA_PX or chroma_px < ZONE_BLOB_MIN_CHROMA_FRAC * area:
            continue
        blobs.append({"area": area, "x": x, "y": y, "w": w, "h": h, "chroma_px": chroma_px})
    blobs.sort(key=lambda b: b["area"], reverse=True)
    return thresh, blobs


def fall_shaped_blob(blobs):
    """The first candidate blob that is both big enough to be a person and
    shaped like the fall pose (see check_for_fall) - tested across ALL
    candidates, not just the largest, so one big artifact or a walker's
    blob can't mask a real fallen person in the same frame."""
    if __import__("os").environ.get("SIM_DISABLE_FALL_SHAPE"):
        return None  # test hook: force falls to be caught only by the inactivity path
    for b in blobs:
        if b["area"] < FALL_MIN_BLOB_AREA_PX:
            continue
        det = {"x1": b["x"], "y1": b["y"], "x2": b["x"] + b["w"], "y2": b["y"] + b["h"]}
        if check_for_fall(det, threshold=ZONE_FALL_ASPECT_THRESHOLD, max_ratio=ZONE_FALL_ASPECT_MAX):
            return b
    return None


# Inactivity-based fall detection. The aspect-ratio test alone is
# orientation-dependent: the fall pose is wide when seen side-on but narrow
# and tall (aspect ~0.55) when a camera looks along the body - and the two
# cameras in a room can sit on opposite ends of that same line, so BOTH can
# see it end-on and a real fall goes undetected for its whole duration (seen
# live: Person2 lay 90s in plain view of RoomB_Cam3/Cam4, never dispatched).
# Real fall-detection systems use prolonged inactivity for exactly this
# reason. Nobody in this scene legitimately stands motionless for long:
# idle animations are capped at IDLE_MAX_SECONDS and robot holds are ~15s,
# both well under STATIC_FALL_SWEEPS sweeps (~8s apart).
STATIC_FALL_SWEEPS = 4
STATIC_MATCH_PX_MIN = 8
_static_blob_tracks = {}  # camera_path -> [{"cx","cy","w","h","count"}]


CHECKIN_PRIORITY_MAX_WAIT = 45.0  # seconds; if someone still hasn't been checked in after this, stop putting them ahead of other alerts (never starve the rest)


def checkin_pending(movers):
    """True while someone who just came through the door is waiting to be
    checked in (has not been scanned yet), for at most CHECKIN_PRIORITY_MAX_WAIT
    seconds. Used to put check-in ahead of non-emergency alerts and to stop
    the door queue itself from raising alerts."""
    now = time.time() - _tick_state.get("sim_start", time.time())
    pending = False
    for m in movers:
        if (m.get("is_visitor") and m.get("visitor_state") == "entering"
                and not m.get("checked_in") and not m.get("should_despawn")):
            since = m.get("queued_since")
            if since is None:
                since = m["queued_since"] = now
            if now - since < CHECKIN_PRIORITY_MAX_WAIT:
                pending = True
    return pending


def door_queue_waiting(movers, zone_name):
    """True when the door queue is active and `zone_name` is a camera in the
    door's room (east / RoomA). A person waiting in the doorway is
    legitimately motionless, present for many sweeps and only partly visible
    through the opening (it reads as a squat ~35x35 blob, aspect ~1.0 - a
    false "fall"), so that room's fall and loitering rules ignore it while
    the queue lasts instead of pulling the robot away from the desk it is
    supposed to be checking them in at."""
    return zone_name.startswith("RoomA") and checkin_pending(movers)


def anyone_idling(movers):
    """True while any mover is holding an idle/social animation (see
    IDLE_PAUSE_CHANCE) - legitimately motionless, and two idle pauses can
    roll back-to-back with only a brief gap between them (confirmed live: a
    false fall from someone who simply idled twice in a row, ~9+ real
    seconds essentially motionless - not the full inactivity-fall window on
    its own, but enough that the motionless-blob tracker's sweep-to-sweep
    matching accumulated across both holds). Global rather than per-room -
    only ever 1-2 people idle at a time here, so the false-negative risk
    (briefly not catching a genuine fall elsewhere while someone idles) is
    negligible against the false-positive this closes."""
    return any(m.get("idling") for m in movers)


def update_static_tracks(camera_path, blobs):
    """Called once per sweep with that sweep's person-shaped blobs. Returns
    the first blob that has now sat in the same place at the same size for
    STATIC_FALL_SWEEPS consecutive sweeps, else None."""
    previous = _static_blob_tracks.get(camera_path, [])
    tracks = []
    static_blob = None
    for b in blobs:
        if b["area"] < FALL_MIN_BLOB_AREA_PX:
            continue
        cx, cy = b["x"] + b["w"] / 2.0, b["y"] + b["h"] / 2.0
        count = 1
        for t in previous:
            tol = max(STATIC_MATCH_PX_MIN, 0.15 * max(b["w"], b["h"]))
            if (math.hypot(cx - t["cx"], cy - t["cy"]) <= tol
                    and abs(b["w"] - t["w"]) <= 0.35 * max(b["w"], t["w"])
                    and abs(b["h"] - t["h"]) <= 0.25 * max(b["h"], t["h"])):
                count = t["count"] + 1
                break
        tracks.append({"cx": cx, "cy": cy, "w": b["w"], "h": b["h"], "count": count})
        if count >= STATIC_FALL_SWEEPS and static_blob is None:
            static_blob = dict(b, static_sweeps=count)
    _static_blob_tracks[camera_path] = tracks
    return static_blob


def blob_near(blobs, ref, max_shift=FALL_CONFIRM_MAX_SHIFT_PX):
    """The blob in `blobs` closest to `ref`'s center, if within max_shift px."""
    rx, ry = ref["x"] + ref["w"] / 2.0, ref["y"] + ref["h"] / 2.0
    best = None
    for b in blobs:
        if b["area"] < FALL_MIN_BLOB_AREA_PX:
            continue
        d = math.hypot(b["x"] + b["w"] / 2.0 - rx, b["y"] + b["h"] / 2.0 - ry)
        if d <= max_shift and (best is None or d < best[0]):
            best = (d, b)
    return best[1] if best else None


def scan_zone_camera(camera_path, zone_name, movers, robot_prim=None):
    """Zone cameras ONLY detect + check fall/loitering now - DeepFace
    identification was removed entirely from this path per explicit scope
    cut: fall and loitering detection only need a bounding-box shape and a
    presence streak, never WHO the person is. Identification still happens
    at the fixed checkpoint camera and via the robot's own orbit search -
    neither of those changed.

    Uses background-subtraction blob detection instead of YOLO's person
    classifier - confirmed via extensive diagnostics (see NOTES.md,
    "FOLLOW-UP: rotation math confirmed correct...") that YOLO cannot
    recognize this project's actual mixamo_fall pose (a curled/crumpled
    heap, not a flat sprawled-out silhouette) as "person" at ANY
    resolution up to 3200x2400 or confidence down to 0.01, in any of the
    4 real zone/camera pairs - a real limitation of the source animation
    clip + a YOLO person-classifier domain gap on this synthetic render,
    not a fixable transform/positioning bug. Background subtraction
    sidesteps this entirely: it doesn't need to recognize WHAT the shape
    is, only that something foreign appeared against a known-empty
    reference frame - which reliably finds this pose regardless of
    whether it looks human to a classifier.

    The robot itself is hidden for the duration of the capture (see
    robot_prim) - confirmed via a real live run that the robot's own
    continuous patrol through every zone's wide diagonal view was a
    second, distinct false-positive source, on top of everything else
    fixed this session: it triggered simultaneous "person detected" (and
    even a false "FALL DETECTED") across multiple zones at once, since
    background subtraction has no way to distinguish "the robot passed
    through" from "a person is here." The robot doesn't need to be
    visible to a zone camera for anything - it only ever identifies
    people through its OWN camera - so hiding it here has no downside.
    """
    robot_was_visible = None
    if robot_prim is not None and robot_prim.IsValid():
        robot_was_visible = UsdGeom.Imageable(robot_prim).ComputeVisibility() != UsdGeom.Tokens.invisible
        UsdGeom.Imageable(robot_prim).MakeInvisible()
        # BUG (found live from an actual saved debug frame: the robot -
        # Nova Carter, clearly visible - sitting right in a zone camera's
        # capture despite this MakeInvisible() call, right around when
        # the whole fall/loitering detection pipeline went completely
        # silent - 0 detections across 80 scans in one run). The render
        # pipeline doesn't necessarily reflect a visibility change on the
        # very next captured frame - camera.get_rgba() inside
        # capture_frame() can return an already-in-flight/buffered render
        # from BEFORE this MakeInvisible() call, especially right after a
        # paused camera resumes. A few real forced-render ticks here,
        # before capture_frame() even starts its own capture loop, flush
        # that stale state out so the frame actually captured reflects
        # the robot really being hidden.
        for _ in range(3):
            _tick(force_render=True)

    print(f"  scanning {zone_name}...")
    try:
        bgr = capture_frame(camera_path, resolution=ZONE_SCAN_RESOLUTION)
    finally:
        if robot_prim is not None and robot_prim.IsValid() and robot_was_visible:
            UsdGeom.Imageable(robot_prim).MakeVisible()
    if bgr is None:
        return None

    tag = zone_name.replace(" ", "_")

    # ALWAYS save the raw frame, regardless of whether anyone gets detected
    # - this is the actual visibility gap that made "why isn't the camera
    # finding anything" impossible to diagnose: the old code only ever
    # saved a debug image AFTER a detection already succeeded, so a camera
    # that detects NOTHING left zero visual evidence behind to check
    # against. Now every single scan leaves a real frame in debug_captures
    # to look at, success or failure.
    raw_debug_name = f"{datetime.now().strftime('%H%M%S')}_{tag}_RAWFRAME.jpg"
    if not RECORDING_MODE:
        cv2.imwrite(os.path.join(DEBUG_DIR, raw_debug_name), bgr)

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Bootstrap: the very first frame ever seen for this camera has nothing
    # to diff against yet - accept it as a tentative reference and wait for
    # the next sweep.
    if camera_path not in _zone_reference_frames:
        _zone_reference_frames[camera_path] = gray
        _zone_warmup_counts[camera_path] = 0
        _zone_warmup_attempts[camera_path] = 0
        return None

    reference = _zone_reference_frames[camera_path]
    thresh, blobs = find_zone_blobs(gray, reference, bgr)
    detected = len(blobs) > 0
    # Diagnostic (added while chasing why real scripted falls kept going
    # undetected): what the camera is ACTUALLY reacting to. The mask is the
    # exact thresholded diff the blob search runs on, and the summary shows
    # every candidate blob rather than just the one that got tested.
    if not RECORDING_MODE:
        cv2.imwrite(os.path.join(DEBUG_DIR, f"{datetime.now().strftime('%H%M%S')}_{tag}_MASK.jpg"), thresh)
    if blobs:
        print("    (blobs: " + ", ".join(f"{b['w']}x{b['h']}@({b['x']},{b['y']})" for b in blobs[:4]) + ")")

    # BUG (found by hand-tracing a live log: 4 different zone cameras all
    # "froze" 3 different, genuinely-still-standing people within moments
    # of each other, all BEFORE any of the run's actual scripted falls
    # were even due) - warmup used to blindly bake WHATEVER was in frame
    # during the first ZONE_WARMUP_SWEEPS sweeps as the reference,
    # regardless of content. Since everyone wanders continuously from
    # t=0, a walker in view during warmup got baked in as "background" -
    # every later real frame (a totally different, empty or
    # differently-posed scene) then diffed heavily against that
    # person-shaped baseline, producing large, oddly-shaped blobs that
    # tripped the fall aspect-ratio check on people who never fell. Real
    # fix: only count a sweep toward finishing warmup if it was actually
    # confirmed empty (detected == False) - a warmup sweep that finds
    # someone just re-baselines to the current frame (so a stale
    # non-empty reference can't persist) but doesn't graduate. Capped at
    # WARMUP_MAX_ATTEMPTS total tries so a genuinely always-busy zone
    # can't stall detection forever - falls back to accepting whatever's
    # there, same as the old behavior, only as a last resort.
    WARMUP_MAX_ATTEMPTS = 40
    warmup_count = _zone_warmup_counts.get(camera_path, 0)
    warmup_attempts = _zone_warmup_attempts.get(camera_path, 0)
    if warmup_count < ZONE_WARMUP_SWEEPS and warmup_attempts < WARMUP_MAX_ATTEMPTS:
        _zone_reference_frames[camera_path] = gray
        _zone_warmup_attempts[camera_path] = warmup_attempts + 1
        if not detected:
            _zone_warmup_counts[camera_path] = warmup_count + 1
        # Diagnostic visibility into a real suspected cause of a total
        # detection outage (0 detections across 80 scans in one run, with
        # up to 6 people now wandering the building instead of 3-4) -
        # this zone's warmup never finding a genuinely empty room to
        # baseline against within WARMUP_MAX_ATTEMPTS would silently fall
        # back to accepting a reference frame that already has people
        # baked into it as "background," breaking detection for the
        # entire rest of the run. Printed either way so this is visible
        # instead of a silent guess.
        if warmup_attempts + 1 >= WARMUP_MAX_ATTEMPTS and warmup_count < ZONE_WARMUP_SWEEPS:
            print(f"  [{zone_name}] WARNING: warmup never found {ZONE_WARMUP_SWEEPS} confirmed-empty "
                  f"reads within {WARMUP_MAX_ATTEMPTS} attempts (only {warmup_count}) - falling back to "
                  f"whatever's in frame right now as the reference. If that's not actually empty, "
                  f"detection will be unreliable for the rest of this run.")
        return None

    # Adaptive reference: whenever a sweep finds NOTHING, refresh this
    # zone's stored reference to the CURRENT frame instead of keeping the
    # one from run start. Confirmed via a real long-running live test that
    # this scene's lighting genuinely drifts over time (a saved raw frame
    # late in one run was dramatically darker than the same camera's frame
    # from early in the SAME run) - a single frozen reference eventually
    # diffs against nothing but that drift itself, producing large false
    # blobs (and climbing "aspect ratio" false positives) across every
    # zone simultaneously, worsening the longer the run goes. Only
    # refreshing on a confirmed-empty sweep (never while a real detection
    # streak is active) means this can't erase or interrupt an ongoing
    # real fall/loitering detection.
    #
    # Tightened to require 2 CONSECUTIVE not-detected reads, not just one -
    # found live (real root cause of a total detection outage this
    # session) that with several people now wandering almost continuously,
    # a single "looks basically unchanged from last time" read can happen
    # even with people actually standing in frame, if nothing moved much
    # in the ~8s between samples - trusting that one read alone to refresh
    # the reference baked people into it as "background" and broke
    # detection for the rest of the run. Requiring 2 in a row is a real
    # (if imperfect) safety margin against that, while still tracking
    # genuine lighting drift within a couple sweeps.
    if not detected:
        empty_streak = _zone_empty_streaks.get(camera_path, 0) + 1
        _zone_empty_streaks[camera_path] = empty_streak
        if empty_streak >= 2:
            _zone_reference_frames[camera_path] = gray
        _fall_alerted_zones[camera_path] = False
        _fall_dup_noted[camera_path] = False
        _static_blob_tracks[camera_path] = []
        return None
    _zone_empty_streaks[camera_path] = 0

    # Test EVERY candidate blob for fall shape, not just the largest - a
    # big artifact or a walker's blob used to win the "largest" contest and
    # hide a real fallen person elsewhere in the same frame. If nothing is
    # fall-shaped, fall back to the largest blob purely for the debug crop.
    # Someone waiting at the door for the robot to check them in is
    # legitimately motionless and only partly visible through the doorway
    # (seen live: an intruder in the door read as a squat ~35x35 blob - both
    # a false shape-based fall and a false inactivity fall). The door is in
    # the east room, so that room's fall rules pause while anyone is in the
    # check-in queue (see door_queue_waiting).
    door_queue_active = door_queue_waiting(movers, zone_name)
    fall_blob = None if door_queue_active else fall_shaped_blob(blobs)
    # The inactivity path alone (not the shape test - an idling pose doesn't
    # look fall-shaped) also pauses while anyone's idling - see anyone_idling's
    # own comment for the false positive this closes.
    if door_queue_active or anyone_idling(movers):
        _static_blob_tracks[camera_path] = []
        static_blob = None
    else:
        static_blob = update_static_tracks(camera_path, blobs)
    fall_from_inactivity = False
    if fall_blob is None and static_blob is not None:
        fall_blob = static_blob
        fall_from_inactivity = True
        print(f"    (motionless person-shaped blob for {static_blob['static_sweeps']} sweeps at "
              f"{static_blob['w']}x{static_blob['h']}@({static_blob['x']},{static_blob['y']}) - treating as a fall)")
    fell_this_sweep = fall_blob is not None
    shown = fall_blob or blobs[0]
    best_detection = {"x1": shown["x"], "y1": shown["y"], "x2": shown["x"] + shown["w"], "y2": shown["y"] + shown["h"]}

    # BUG (found live: 4 zone cameras all "detected a fall" and froze 3
    # different, genuinely-still-standing people within moments of each
    # other, ~5 minutes into a run, long after both of that run's real
    # scripted falls had already resolved) - a SINGLE sweep's aspect-ratio
    # reading was being trusted as gospel, so any one noisy frame
    # (overlapping wanderers, a render/lighting hiccup) triggered a full
    # FALL_PAUSE_DURATION freeze on an innocent person.
    #
    # First attempt at a fix (requiring FALL_CONFIRM_SWEEPS consecutive
    # "fell" reads spread across the normal ~8s-apart sweep cycle, same
    # hysteresis principle as loitering) was WRONG per direct live
    # feedback while watching the sim: that adds real seconds of latency
    # to every genuine fall too ("the robot isn't even going to the
    # person... the second they fall the robot should beeline to them").
    # A real fall needs to be caught almost immediately - the two goals
    # (fast on real falls, immune to one-frame glitches) aren't actually
    # in conflict if confirmation happens as an immediate BURST right now
    # instead of waiting for the next scheduled sweep: re-capture this
    # SAME camera again right away (sub-second), not on the next ~8s
    # cycle. A genuine fall reproduces on the very next frame; a one-off
    # glitch (a stray shadow, motion blur, two people's blobs merging for
    # an instant) almost never does.
    confirmed_reads = 1 if fell_this_sweep else 0
    if fell_this_sweep:
        for _ in range(FALL_CONFIRM_SWEEPS - 1):
            # BUG (found live after adding a 4th character: the robot fell
            # into an endless loop of false "fall" alerts - "nobody is
            # down anymore" every single time, for a full 30-minute run,
            # never once finding anyone actually fallen) - the retry
            # capture used to happen essentially back-to-back with the
            # first read (sub-second gap), which is great for confirming a
            # genuine fall fast, but gives two people whose blobs happen
            # to briefly overlap/merge in a camera's view (more likely now
            # that there are 4 people instead of 3, in the same rooms) no
            # real time to separate before the "confirm" read - so a
            # brief, genuine overlap got trusted just as much as an actual
            # fall. A short REAL delay here (with _tick() kept running
            # throughout so movement/robot dispatch don't freeze) still
            # confirms a real fall within about a second - far faster than
            # the old ~8s sweep-interval approach - while giving two
            # overlapping walkers real time to actually walk apart.
            wait_start = time.time()
            while time.time() - wait_start < 1.0:
                _tick()
            retry_robot_was_visible = None
            if robot_prim is not None and robot_prim.IsValid():
                retry_robot_was_visible = UsdGeom.Imageable(robot_prim).ComputeVisibility() != UsdGeom.Tokens.invisible
                UsdGeom.Imageable(robot_prim).MakeInvisible()
            try:
                retry_bgr = capture_frame(camera_path, resolution=ZONE_SCAN_RESOLUTION)
            finally:
                if robot_prim is not None and robot_prim.IsValid() and retry_robot_was_visible:
                    UsdGeom.Imageable(robot_prim).MakeVisible()
            if retry_bgr is None:
                break
            retry_gray = cv2.GaussianBlur(cv2.cvtColor(retry_bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            _, retry_blobs = find_zone_blobs(retry_gray, reference, retry_bgr)
            # An inactivity fall is confirmed by the SAME blob still being
            # there (it has no fall shape to re-test); a shape-based one by
            # a fall-shaped blob.
            retry_fall = blob_near(retry_blobs, fall_blob) if fall_from_inactivity else fall_shaped_blob(retry_blobs)
            if retry_fall is None:
                break
            # The confirming read must find the fall-shaped blob where the
            # first one was - a different fall-shaped blob elsewhere is a
            # different (probably spurious) event, not confirmation.
            shift = math.hypot((retry_fall["x"] + retry_fall["w"] / 2) - (fall_blob["x"] + fall_blob["w"] / 2),
                               (retry_fall["y"] + retry_fall["h"] / 2) - (fall_blob["y"] + fall_blob["h"] / 2))
            if shift > FALL_CONFIRM_MAX_SHIFT_PX:
                break
            if not fall_from_inactivity:
                w_delta = abs(retry_fall["w"] - fall_blob["w"]) / max(retry_fall["w"], fall_blob["w"])
                h_delta = abs(retry_fall["h"] - fall_blob["h"]) / max(retry_fall["h"], fall_blob["h"])
                if (shift > FALL_SHAPE_CONFIRM_MAX_SHIFT_PX
                        or w_delta > FALL_SHAPE_CONFIRM_MAX_SIZE_DELTA or h_delta > FALL_SHAPE_CONFIRM_MAX_SIZE_DELTA):
                    print(f"    (confirm read moved/resized too much for a motionless body - shift={shift:.0f}px, "
                          f"w {fall_blob['w']}->{retry_fall['w']}, h {fall_blob['h']}->{retry_fall['h']} - likely still walking)")
                    break
            confirmed_reads += 1
    fell = confirmed_reads >= FALL_CONFIRM_SWEEPS

    # BUG (found by hand-tracing a live log where the robot's chase target
    # kept jumping to a different coordinate every ~3s and it could never
    # close the distance): unlike loitering (which gates via an "alerted"
    # flag, see check_loitering), a fall used to fire a BRAND NEW
    # dispatch_alert on EVERY sweep the aspect-ratio check tripped - so a
    # single person lying down for many consecutive sweeps spammed dozens
    # of redundant alerts, and find_next_unhandled_alert/ROBOT_CHECK_INTERVAL
    # kept re-interrupting the robot's approach with each "new" one before
    # it ever arrived. Fixed with the same one-shot-per-episode debounce
    # loitering already uses: only fire once, then stay silent until this
    # zone reports NOT fallen (person stood up, or briefly left frame).
    newly_fallen = fell and not _fall_alerted_zones.get(camera_path)
    if newly_fallen:
        # Two cameras cover each room, so one person lying down is seen by
        # both a few seconds apart. Only the FIRST one may dispatch the
        # robot: a second dispatch for the same fall gets queued behind the
        # real assist and sends the robot back to "help" someone who has
        # already been helped ("nobody is down anymore"). Same room + either
        # a sibling camera still reporting the fall, or a dispatch within
        # FALL_DISPATCH_COOLDOWN, means it's the same episode.
        room = zone_name.split("_")[0]
        _fall_camera_room[camera_path] = room
        sibling_reports_fall = any(
            flagged and other != camera_path and _fall_camera_room.get(other) == room
            for other, flagged in _fall_alerted_zones.items())
        recently_dispatched = (time.time() - _last_fall_dispatch.get(room, -1e9)) < FALL_DISPATCH_COOLDOWN
        # BUG (seen live: robot "arrived... but nobody is down anymore" right after a
        # real successful assist, reason=fall dispatched from the OTHER room): the
        # dedup above only compares cameras within the same room-name prefix, but a
        # dispatch's actual coordinates come from get_nearest_mover_coords - whichever
        # MOVER is nearest to this camera's own mount point, not necessarily anyone
        # this camera actually saw. A camera in one room can end up dispatching to a
        # person physically in a totally different room if that person happens to be
        # the closest mover to its zone point (e.g. right after being helped up near a
        # doorway) - a genuine cross-room misattribution, not merely two cameras
        # sharing one real fall. Guard directly on the coordinates: if the candidate
        # dispatch point is close to wherever the last fall dispatch (any room) sent
        # the robot, and that was recent, it's the same person being re-reported, not
        # a second faller.
        candidate_coords = get_nearest_mover_coords(movers, zone_name)
        coords_match_recent = False
        if candidate_coords is not None:
            for other_room, (rx, ry, rt) in _last_fall_dispatch_coords.items():
                if other_room != room and (time.time() - rt) < FALL_DISPATCH_COOLDOWN:
                    if math.hypot(candidate_coords[0] - rx, candidate_coords[1] - ry) < 3.0:
                        coords_match_recent = True
                        break
        duplicate_of_same_fall = sibling_reports_fall or recently_dispatched or coords_match_recent
        if duplicate_of_same_fall:
            # BUG (a real fall lay unnoticed for its whole 90 s): a suppressed
            # duplicate used to mark THIS camera as already alerted too, so if the
            # sibling's flag was really about someone else's fall (e.g. the person
            # the robot had just helped up, still flagged for a sweep or two), this
            # camera never fired for the NEW fall. Leave this camera's flag alone so
            # the detection is re-evaluated every sweep and dispatches as soon as the
            # sibling's episode has ended; only announce it once.
            if not _fall_dup_noted.get(camera_path):
                _fall_dup_noted[camera_path] = True
                print(f"  [{zone_name}] *** FALL DETECTED *** (another camera in this room is already reporting a fall - "
                      f"holding this one until that episode ends)")
                append_log_entry({
                    "event_type": "fall_alert",
                    "zone": zone_name,
                    "camera": camera_path,
                    "timestamp": datetime.now(timezone.utc).isoformat()
                })
            newly_fallen = False
        else:
            _fall_alerted_zones[camera_path] = True
            _fall_dup_noted[camera_path] = False
            print(f"  [{zone_name}] *** FALL DETECTED ***")
            append_log_entry({
                "event_type": "fall_alert",
                "zone": zone_name,
                "camera": camera_path,
                "timestamp": datetime.now(timezone.utc).isoformat()
            })
            _last_fall_dispatch[room] = time.time()
            _last_fall_dispatch_coords[room] = (candidate_coords[0], candidate_coords[1], time.time()) if candidate_coords else _last_fall_dispatch_coords.get(room)
            append_log_entry({
                "event_type": "dispatch_alert",
                "alert_id": next_alert_id(),
                "reason": "fall",
                "person_id": None,
                "zone": zone_name,
                "coords": candidate_coords,
                "timestamp": datetime.now(timezone.utc).isoformat()
            })
    elif not fell:
        _fall_alerted_zones[camera_path] = False
        _fall_dup_noted[camera_path] = False

    # Still save a debug crop image for visual sanity-checking - this is
    # just a file write, not a DeepFace inference call, effectively free.
    crop = crop_person(bgr, best_detection)
    if crop.size > 0:
        debug_name = f"{datetime.now().strftime('%H%M%S')}_{tag}_crop.jpg"
        if not RECORDING_MODE:
            cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    # newly_fallen (not "fell") is what callers should act on for anything
    # one-shot (freezing a person in place, etc) - see run_sweep. "fell"
    # alone stays True for every sweep for the whole duration someone is
    # down, which used to cause pause_nearest_mover to be re-called every
    # single sweep (see that BUG below).
    return {"detected": True, "fell": fell, "newly_fallen": newly_fallen}


def scan_checkpoint(authorized_ids, banned_ids):
    print(f"  scanning checkpoint...")
    bgr = capture_frame(CHECKPOINT_PATH, resolution=(480, 360))
    if bgr is None:
        return

    frame_path = os.path.join(os.environ["TEMP"], "checkpoint_scan.jpg")
    cv2.imwrite(frame_path, bgr)
    yolo_result = _workers["yolo"].call(frame_path)
    detections = yolo_result.get("detections", []) if not yolo_result.get("error") else []
    if not detections:
        return

    best_detection = max(detections, key=lambda d: d["confidence"])

    if check_for_fall(best_detection):
        print(f"  [CHECKPOINT] *** FALL DETECTED ***")
        append_log_entry({
            "event_type": "fall_alert",
            "zone": "CHECKPOINT",
            "camera": CHECKPOINT_PATH,
            "timestamp": datetime.now(timezone.utc).isoformat()
        })

    crop = crop_person(bgr, best_detection)
    if crop.size == 0:
        return

    crop_path = os.path.join(os.environ["TEMP"], "checkpoint_crop.jpg")
    cv2.imwrite(crop_path, crop)

    debug_name = f"{datetime.now().strftime('%H%M%S')}_checkpoint_crop.jpg"
    if not RECORDING_MODE:
        cv2.imwrite(os.path.join(DEBUG_DIR, debug_name), crop)

    reid_result = _workers["deepface"].call(f"{crop_path}\t{KNOWN_FACES_DIR}")

    person_id = None
    status = reid_result.get("status")

    if status == "match":
        matched_filename = os.path.basename(reid_result["matched_path"])
        person_id = os.path.splitext(matched_filename)[0]
        print(f"  [CHECKPOINT] IDENTIFIED: {person_id}")
    elif status == "new":
        person_id = f"ID_{get_next_id():04d}"
        cv2.imwrite(os.path.join(KNOWN_FACES_DIR, f"{person_id}.jpg"), crop)
        print(f"  [CHECKPOINT] NEW PERSON: {person_id}")
    else:
        print(f"  [CHECKPOINT] face detection failed, skipping: {reid_result.get('reason')}")

    if person_id:
        # Face confirmed their identity here, so this is a trustworthy moment
        # to record/update their clothing signature for zone cameras to use.
        save_appearance_profile(person_id, crop)

        is_authorized = person_id in authorized_ids
        is_banned = person_id in banned_ids
        print(f"  [CHECKPOINT] {person_id} -> {'AUTHORIZED' if is_authorized else 'NOT AUTHORIZED'}")
        append_log_entry({
            "event_type": "checkpoint",
            "person_id": person_id,
            "checkpoint": CHECKPOINT_NAME,
            "authorized": is_authorized, "banned": is_banned,
            "timestamp": datetime.now(timezone.utc).isoformat()
        })
        if is_banned:
            print(f"  [CHECKPOINT] *** INTRUDER ALERT: {person_id} is on the banned list! ***")
            append_log_entry({
                "event_type": "intruder_alert",
                "person_id": person_id,
                "zone": "CHECKPOINT",
                "timestamp": datetime.now(timezone.utc).isoformat()
            })


def run_sweep(camera_zones, authorized_ids, banned_ids, streak_state, movers, elapsed, robot_prim=None):
    timestamp = datetime.now(timezone.utc).isoformat()
    print(f"\n--- Sweep at {timestamp} ---")

    for camera_path, zone_name in camera_zones.items():
        result = scan_zone_camera(camera_path, zone_name, movers, robot_prim)

        if result is None or result.get("error"):
            # Nobody detected (or a scan error) - the presence streak breaks here.
            streak_state[zone_name]["streak"] = 0
            streak_state[zone_name]["alerted"] = False
            if result and result.get("error"):
                print(f"  [{zone_name}] error: {result['error']}")
            continue

        # BUG (reported live: "the people are all frozen now") - camera-
        # triggered freezing on a fall detection was ENTIRELY redundant
        # and, worse, dangerous: a genuine scripted fall already freezes
        # itself the instant it triggers (mover["paused_until"] set
        # directly in update_person_position's fall-trigger block,
        # independent of whether/when any camera notices), and the
        # movement-skip check (`elapsed < mover["paused_until"]`) fires
        # for ANY mover with a future paused_until regardless of whether
        # they're actually "fallen" - pause_nearest_mover() never set
        # mover["fallen"], so a FALSE fall reading (confirmed to still
        # happen occasionally even with the burst-confirm fix - blob
        # detection on ambient wanderers isn't perfect) could freeze a
        # perfectly normal standing/walking person in place for a full
        # FALL_PAUSE_DURATION with zero visual indication why, and since
        # every zone camera runs this independently, multiple false
        # positives across different zones could freeze several different
        # people at once. Real fix: don't freeze anyone here at all - the
        # camera's only real job is getting a dispatch_alert out so the
        # robot responds; genuinely fallen people are already handled by
        # the scripted fall's own freeze.
        streak_state[zone_name]["streak"] += 1
        print(f"  [{zone_name}] person detected (streak={streak_state[zone_name]['streak']})")

        # unauthorized_zone_presence removed along with zone-camera face/
        # clothing identification - that alert type depended entirely on
        # knowing WHO was detected, which zone cameras no longer check
        # (out of scope per explicit direction: zone cameras are fall/
        # loitering detection only). Identity/authorization checks still
        # happen at the checkpoint camera and via the robot's own search.

        if not door_queue_waiting(movers, zone_name) and check_loitering(zone_name, streak_state):
            # BUG (found while investigating a poor loitering-identification rate,
            # ~53% of orbit searches landing a face): this used to hand the robot a
            # single (x, y) SNAPSHOT of wherever the nearest mover happened to be at
            # the moment of detection, with no way to re-find them afterward - a
            # loitering person is BY DEFINITION still walking around (loitering was
            # deliberately never frozen, see this block's own comment below), so by
            # the time the robot crossed the room and ran its multi-standpoint orbit
            # (each standpoint a real several-second YOLO+DeepFace round trip), they
            # were very often somewhere else entirely. Carrying the mover's own name
            # (not its recognized ID - that's the whole thing being searched for)
            # lets update_robot and search_for_face re-read this exact person's LIVE
            # position continuously - on approach, at arrival, and at every orbit
            # standpoint - the same live-tracking fall/overstay dispatches already
            # get via person_id, just keyed by simulation identity instead of a
            # not-yet-known recognized one.
            nearest_mover = get_nearest_mover(movers, zone_name)
            coords = clamp_to_floor(nearest_mover["logical_x"], nearest_mover["logical_y"]) if nearest_mover else get_nearest_mover_coords(movers, zone_name)
            mover_name = nearest_mover["name"] if nearest_mover else None
            # BUG (found live: a resident racked up 3 loitering "incidents" -
            # enough to trigger a ban - within 14 real seconds, nowhere near
            # enough time for 3 GENUINELY separate episodes at ~8s/sweep):
            # check_loitering's own one-shot "alerted" flag is keyed per
            # CAMERA, not per person - two cameras covering the same room can
            # both be watching the same person and independently cross their
            # own 3-sweep threshold within moments of each other, each firing
            # its own dispatch for what is really ONE ongoing episode. Same
            # class of bug already found and fixed for falls (see
            # _fall_alerted_zones's own history) - fix the same way, keyed on
            # the actual person this time (mover_name is already resolved,
            # more precise than the coordinate-proximity guard falls needed).
            recently_dispatched = mover_name is not None and (
                time.time() - _last_loiter_dispatch.get(mover_name, -1e9) < LOITER_DISPATCH_COOLDOWN)
            print(f"  [{zone_name}] *** LOITERING (someone present {LOITER_THRESHOLD}+ sweeps in a row) ***"
                  + (" (already reported for this person recently - no new dispatch)" if recently_dispatched else ""))
            # BUG (reported live: "people who freeze like... just stand
            # there walking and not moving an inch... keeps happening") -
            # this camera-side detection used to call pause_nearest_mover()
            # here, freezing whoever's nearest for a full FALL_PAUSE_DURATION
            # (45s) with no fall animation and no mover["fallen"] flag set -
            # just a normal walking person whose position updates stopped
            # dead while their walk-cycle animation kept looping (that
            # animation is timeline-driven, independent of our position
            # code - see update_robot's matching fix for the exact same
            # symptom from a different cause). Worse, this fires from
            # ordinary ambient wandering - 3 consecutive 8-second sweeps
            # (24+s) of a zone camera simply seeing someone nearby is easy
            # to trigger just by walking through/lingering near that zone's
            # view, not real loitering. A comment already documented the
            # intended fix ("don't freeze anyone here - the camera's only
            # real job is getting a dispatch_alert out so the robot
            # responds") but the actual pause_nearest_mover() call was
            # never removed to match it - restoring that now. The robot
            # still gets dispatched below exactly as before; the person
            # just isn't artificially frozen in place waiting for it.
            append_log_entry({
                "event_type": "loitering_alert",
                "zone": zone_name,
                "coords": coords,
                "timestamp": timestamp
            })
            if not recently_dispatched:
                if mover_name is not None:
                    _last_loiter_dispatch[mover_name] = time.time()
                append_log_entry({
                    "event_type": "dispatch_alert",
                    "alert_id": next_alert_id(),
                    "reason": "loitering",
                    "person_id": None,
                    "mover_name": mover_name,
                    "zone": zone_name,
                    "coords": coords,
                    "timestamp": timestamp
                })

        append_log_entry({
            "event_type": "zone",
            "zone": zone_name,
            "camera": camera_path,
            "timestamp": timestamp
        })

    if CHECKPOINT_AVAILABLE:
        scan_checkpoint(authorized_ids, banned_ids)


_visitor_spawn_count = 0  # module-level counter for unique visitor prim paths/names across the whole run


def spawn_visitor(stage, elapsed):
    """Creates one new transient 'visitor' mover at DOOR_POS, for the
    "people come in and out all the time like a real area" ambient
    population request. Same reference-a-spare-Mixamo-asset pattern as
    Person4 (see that block's comments for why the ops go where they go -
    character_4.usd needs the identical treatment, it's from the same
    export pipeline), using character_4.usd (the last remaining unused
    spare asset) instead so visitors don't look identical to Person4.

    Deliberately does NOT give visitors a fall_clip/get_up_clip binding -
    they only ever wander/loiter/get identified, never scripted-fall, so
    there was no reason to spend the extra bind_fall_animation.py/
    bind_get_up_animation.py setup work on a 5th character file for this.
    get_skeleton_and_clips() already handles a missing get_up clip
    gracefully (falls back to instant-cut, but visitors never trigger
    that path anyway since fall_start is never set for them).

    Returns a fully-formed mover dict, ready to append to the movers list -
    NOT yet appended here, since the caller also needs to register it with
    _tick_state.
    """
    global _visitor_spawn_count
    _visitor_spawn_count += 1
    idx = _visitor_spawn_count
    name = f"Visitor{idx}"
    prim_path = f"/World/xbot_visitor_{idx}"

    prim = stage.DefinePrim(prim_path, "Xform")
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))  # same z=1 convention as every other character here
    model_prim = stage.DefinePrim(prim_path + "/model", "Xform")
    ref_prim = stage.DefinePrim(prim_path + "/model/character_ref", "Xform")
    ref_prim.GetReferences().AddReference(VISITOR_ASSET_PATH)
    model_xf = UsdGeom.Xformable(model_prim)
    model_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    model_xf.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, 0))
    model_xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
    for _ in range(10):
        _tick()

    disable_physics_recursive(prim)
    base_height = 1.0
    door_x, door_y = DOOR_POS
    model_offset = get_model_offset(model_prim)
    set_translate(prim, (door_x - model_offset[0], door_y - model_offset[1], base_height))
    (skel_prim, walk_clip_path, fall_clip_path, held_clip_path, fall_clip_duration,
     get_up_clip_path, get_up_clip_duration) = get_skeleton_and_clips(model_prim)

    # Pick where inside the building this visitor is actually headed - a
    # random point in either room, same spread ambient wander already
    # uses. Their very first "wander_waypoint" is the real DOOR GAP
    # center, not the final interior target directly - walking straight
    # from outside to an arbitrary interior point would cut through solid
    # wall exactly like the original mismatched door coordinate did (see
    # DOOR_GAP_WAYPOINT's own comment). update_person_position's visitor
    # state machine sends them on to the real entry target once they
    # arrive at the gap.
    room = random.choice([ROOM_BOUNDS["west"], ROOM_BOUNDS["east"]])
    entry_x = random.uniform(room[0] + 1, room[1] - 1)
    entry_y = random.uniform(room[2] + 1, room[3] - 1)

    mover = {
        "name": name, "prim": prim, "model_prim": model_prim, "model_offset": model_offset,
        "skel_prim": skel_prim, "walk_clip_path": walk_clip_path, "fall_clip_path": fall_clip_path,
        "held_clip_path": held_clip_path, "fall_clip_duration": fall_clip_duration,
        "get_up_clip_path": get_up_clip_path, "get_up_clip_duration": get_up_clip_duration,
        "base_height": base_height, "start_delay": 0,
        "loiter_start": None, "loiter_duration": None, "loiter_center": None,
        "loiter_was_active": False, "fall_center": None,
        "paused_until": 0, "fall_start": None, "fall_triggered": False, "fallen": False,
        "fall_held_triggered": False, "standing_up": False, "stand_up_start": None,
        "logical_x": door_x, "logical_y": door_y,
        "wander_waypoint": DOOR_GAP_WAYPOINT, "last_wander_move_time": elapsed,
        # Visitor-specific state - see update_person_position's handling.
        # visitor_entry_target holds the REAL interior destination until
        # the gap-crossing leg is done; visitor_exit_final marks the
        # second leg of an exit (gap -> fully outside) vs the first
        # (wherever they are -> gap).
        "is_visitor": True, "visitor_state": "entering", "visit_end_time": None,
        "visitor_entry_target": (entry_x, entry_y), "visitor_exit_final": False,
        "should_despawn": False, "checked_in": False,
        # See the door-denial reaction sequence in update_robot's checkpoint
        # branch: the angry clip plays as a visible reaction beat (looks
        # better than the sad-idle pose it replaced there - the sad-idle
        # clip is kept for the walk-out beat below), then sad_walk replaces
        # walk_clip_path (restored on despawn/revive) for the walk out.
        "angry_clip_path": get_named_clip(model_prim, "mixamo_angry_reaction"),
        "sad_idle_clip_path": get_named_clip(model_prim, "mixamo_sad_idle"),
        "sad_walk_clip_path": get_named_clip(model_prim, "mixamo_sad_walk"),
        "normal_walk_clip_path": walk_clip_path, "pending_denied_exit": False,
    }
    UsdGeom.Imageable(prim).MakeVisible()
    if skel_prim is not None:
        set_animation_clip(skel_prim, walk_clip_path)
    print(f"  [VISITOR] {name} entering through the door at {DOOR_POS}, heading to ({entry_x:.2f},{entry_y:.2f})")
    return mover


def spawn_intruder(stage, elapsed, prim_path, asset_path, name):
    """Generalized out of what used to be Person4's own one-off inline
    block in main() - per explicit direction ("assign 2 ids to the door
    denial now so you can have multiple people denial"), the door-checkin/
    auto-ban/turn-away flow needed a SECOND independent intruder, not just
    one hardcoded character. Modeled directly on spawn_visitor() (same
    nested-child-prim reference pattern, same DOOR_POS/DOOR_GAP_WAYPOINT
    entry routing, same physics-disable/z=1/scale-correction steps) since
    that function already does, at runtime, exactly what Person4's spawn
    used to do only once at startup - the difference here is just the red-
    tint material override and is_intruder=True, so the door-checkin logic
    (see update_robot's checkpoint-arrival branch) auto-flags whatever ID
    this character gets scanned as, instead of waiting for a human
    operator to flag them after the fact.

    Called once immediately at startup for Person4 (unchanged timing/
    behavior from before this generalization) and again on a delayed
    timer for Person5, from main()'s loop - see INTRUDER2_SPAWN_TIME.
    """
    prim = stage.DefinePrim(prim_path, "Xform")
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))  # same z=1 convention as every other character here
    model_prim = stage.DefinePrim(prim_path + "/model", "Xform")
    ref_prim = stage.DefinePrim(prim_path + "/model/character_ref", "Xform")
    ref_prim.GetReferences().AddReference(asset_path)
    model_xf = UsdGeom.Xformable(model_prim)
    model_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    model_xf.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, 0))
    model_xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
    for _ in range(10):
        _tick()

    disable_physics_recursive(prim)
    base_height = 1.0
    door_x, door_y = DOOR_POS
    model_offset = get_model_offset(model_prim)
    set_translate(prim, (door_x - model_offset[0], door_y - model_offset[1], base_height))
    (skel_prim, walk_clip_path, fall_clip_path, held_clip_path, fall_clip_duration,
     get_up_clip_path, get_up_clip_duration) = get_skeleton_and_clips(model_prim)

    # Per explicit direction ("get rid of the red highlight... put a red
    # outline around them... not red people") - the character keeps its own
    # natural appearance now; a flagged intruder is marked by a bright red
    # ring on the floor at their feet instead of recoloring their body. The
    # ring is a CHILD of `prim`, so it automatically follows every future
    # translate() call on the character (walking, turning) with no extra
    # per-tick tracking code needed anywhere - built from two flat
    # cylinders, a red emissive outer disc with a black disc on top masking
    # its center, so only the rim reads as a ring.
    def _emissive_material(mat_path, color, emissive=False):
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        if emissive:
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    ring_outer = UsdGeom.Cylinder.Define(stage, prim_path + "/BanMarkerOuter")
    ring_outer.CreateRadiusAttr(0.5)
    ring_outer.CreateHeightAttr(0.02)
    ring_outer.CreateAxisAttr("Z")
    UsdGeom.Xformable(ring_outer.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, -base_height + 0.01))
    UsdShade.MaterialBindingAPI.Apply(ring_outer.GetPrim()).Bind(
        _emissive_material(prim_path + "/BanMarkerOuterMat", (0.95, 0.05, 0.05), emissive=True))

    ring_mask = UsdGeom.Cylinder.Define(stage, prim_path + "/BanMarkerMask")
    ring_mask.CreateRadiusAttr(0.36)
    ring_mask.CreateHeightAttr(0.03)
    ring_mask.CreateAxisAttr("Z")
    UsdGeom.Xformable(ring_mask.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, -base_height + 0.015))
    UsdShade.MaterialBindingAPI.Apply(ring_mask.GetPrim()).Bind(
        _emissive_material(prim_path + "/BanMarkerMaskMat", (0.03, 0.03, 0.03)))
    print(f"  [INTRUDER] {name} marked with a red floor ring for visual identification - this is the flag-as-banned candidate.")

    room = random.choice([ROOM_BOUNDS["west"], ROOM_BOUNDS["east"]])
    entry_x = random.uniform(room[0] + 1, room[1] - 1)
    entry_y = random.uniform(room[2] + 1, room[3] - 1)

    mover = {
        "name": name, "prim": prim, "model_prim": model_prim, "model_offset": model_offset,
        "skel_prim": skel_prim, "walk_clip_path": walk_clip_path, "fall_clip_path": fall_clip_path,
        "held_clip_path": held_clip_path, "fall_clip_duration": fall_clip_duration,
        "get_up_clip_path": get_up_clip_path, "get_up_clip_duration": get_up_clip_duration,
        "base_height": base_height, "start_delay": 0,
        "loiter_start": None, "loiter_duration": None, "loiter_center": None,
        "loiter_was_active": False, "fall_center": None,
        "paused_until": 0, "fall_start": None, "fall_triggered": False, "fallen": False,
        "fall_held_triggered": False, "standing_up": False, "stand_up_start": None,
        "logical_x": door_x, "logical_y": door_y,
        "wander_waypoint": DOOR_GAP_WAYPOINT, "last_wander_move_time": elapsed,
        "is_visitor": True, "visitor_state": "entering", "visit_end_time": None,
        "visitor_entry_target": (entry_x, entry_y), "visitor_exit_final": False,
        "should_despawn": False, "checked_in": False, "is_intruder": True,
        "angry_clip_path": get_named_clip(model_prim, "mixamo_angry_reaction"),
        "sad_idle_clip_path": get_named_clip(model_prim, "mixamo_sad_idle"),
        "sad_walk_clip_path": get_named_clip(model_prim, "mixamo_sad_walk"),
        "normal_walk_clip_path": walk_clip_path, "pending_denied_exit": False,
    }
    UsdGeom.Imageable(prim).MakeVisible()
    if skel_prim is not None:
        set_animation_clip(skel_prim, walk_clip_path)
    print(f"  [INTRUDER] {name} entering through the door at {DOOR_POS}, heading to ({entry_x:.2f},{entry_y:.2f})")
    return mover


def revive_mover(mover, elapsed):
    """Brings a previously despawned (escorted-out / turned-away) mover back
    to the door as a brand-new visit, reusing its existing prim and clips.
    Everything visit-specific is reset: check-in state, the per-ID visit
    clock (so the overstay timer restarts once the robot re-identifies
    them), any leftover fall/idle/loiter state, and - for returning
    residents - a fresh scripted fall so the cycle has something to show."""
    # known_id is deliberately KEPT: it is the robot's memory of who this
    # person is, so on return the clothing match can be confirmed against it
    # (a returning person is recognised, not re-guessed from the face - the
    # two red intruders' faces get confused often enough that a cold face
    # check mislabelled Person4 as Person5's ID). Only the visit clock is
    # reset, so the overstay timer restarts when they are re-scanned.
    old_id = mover.get("known_id")
    if old_id:
        _tick_state.get("person_first_seen", {}).pop(old_id, None)
    door_x, door_y = DOOR_POS
    room = random.choice([ROOM_BOUNDS["west"], ROOM_BOUNDS["east"]])
    entry = (random.uniform(room[0] + 1, room[1] - 1), random.uniform(room[2] + 1, room[3] - 1))
    mover.update({
        "checked_in": False, "should_despawn": False, "overstay_dispatched": False, "queued_since": None,
        "at_checkin_spot": False, "route_via": [], "exit_via_set": False,
        "is_visitor": True, "visitor_state": "entering", "visit_end_time": None,
        "visitor_entry_target": entry, "visitor_exit_final": False,
        "wander_waypoint": DOOR_GAP_WAYPOINT, "last_wander_move_time": elapsed,
        "paused_until": 0, "idling": False, "fallen": False, "fall_triggered": False,
        "fall_held_triggered": False, "standing_up": False, "stand_up_start": None,
        "fall_start": None, "fall_center": None,
        "loiter_start": None, "loiter_duration": None, "loiter_center": None,
        "loiter_was_active": False, "loiter_waypoint": None,
        "wander_stuck_check_time": None, "wander_stuck_check_pos": None,
        "pending_denied_exit": False, "denial_reaction_until": 0,
    })
    # A denied exit temporarily swaps walk_clip_path for the sad-walk clip (see
    # update_robot's denial branch) - put the real one back so a returning
    # person walks normally again, not permanently "sad."
    if mover.get("normal_walk_clip_path"):
        mover["walk_clip_path"] = mover["normal_walk_clip_path"]
    if not mover.get("is_intruder"):
        mover["no_voluntary_exit"] = True  # stays until the robot escorts them out again
        if FALL_ENABLED and random.random() < FALL_CHANCE:
            mover["fall_start"] = elapsed + random.uniform(35, 55)
            zone_name, zone_pos = random.choice(list(ZONE_CENTERS.items()))
            froom = ROOM_BOUNDS["west"] if zone_pos[0] < 21.75 else ROOM_BOUNDS["east"]
            mover["fall_center"] = (random.uniform(froom[0] + 3, froom[1] - 3), random.uniform(froom[2] + 3, froom[3] - 3))
    set_person_translate(mover, door_x, door_y, mover.get("base_height", 1.0))
    if mover.get("skel_prim") is not None:
        set_animation_clip(mover["skel_prim"], mover["walk_clip_path"])
    UsdGeom.Imageable(mover["prim"]).MakeVisible()
    kind = "INTRUDER" if mover.get("is_intruder") else "VISITOR"
    print(f"  [{kind}] {mover['name']} is back at the door (returning), heading to ({entry[0]:.2f},{entry[1]:.2f}) after check-in")


# ---------- Main ----------

def main():
    os.makedirs(KNOWN_FACES_DIR, exist_ok=True)

    # Clear out the debug folder from the previous run so it doesn't pile up,
    # then recreate it fresh for this run's captures.
    if os.path.exists(DEBUG_DIR):
        shutil.rmtree(DEBUG_DIR)
    os.makedirs(DEBUG_DIR, exist_ok=True)

    # REAL FIX for "why is it chasing (28.67, 28.73) / Zone_A_NorthEast /
    # unauthorized_zone_presence" - none of that exists anywhere in this
    # file anymore. event_log.json was never cleared between runs - it just
    # accumulates forever across every session, including ones from way
    # before this room/scene even existed. Every fresh run resets
    # handled_alert_ids to empty, so find_next_unhandled_alert() happily
    # re-served every ancient unhandled dispatch_alert ever written to that
    # file, from any past session, as if it were brand new. Wiping it at
    # the start of every run is the actual fix - old alerts from a
    # previous run have no business being acted on in a new one anyway.
    with open(EVENT_LOG_FILE, "w") as f:
        json.dump([], f)

    # Identity state is per-run too. IDs are issued from a counter and tied to the
    # face crops in known_faces/, so a leftover banned list or authorized list from
    # an earlier run would attach to whichever different person gets that number
    # this time (e.g. last run's banned intruder id becoming a resident's id).
    # Back the old state up first, then start clean - same reasoning as the event
    # log above.
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = os.path.join(PROJECT_DIR, "run_state_backups", stamp)
    leftovers = [f for f in os.listdir(KNOWN_FACES_DIR) if f.lower().endswith((".jpg", ".png"))]
    if leftovers or os.path.exists(BANNED_IDS_FILE) or os.path.exists(AUTHORIZED_IDS_FILE):
        os.makedirs(backup_dir, exist_ok=True)
        for path in (BANNED_IDS_FILE, AUTHORIZED_IDS_FILE, ID_COUNTER_FILE, APPEARANCE_PROFILES_FILE):
            if os.path.exists(path):
                shutil.copy2(path, backup_dir)
        for name in leftovers:
            shutil.move(os.path.join(KNOWN_FACES_DIR, name), os.path.join(backup_dir, name))
    with open(BANNED_IDS_FILE, "w") as f:
        json.dump({"banned": []}, f)
    with open(AUTHORIZED_IDS_FILE, "w") as f:
        json.dump({"authorized": []}, f)
    with open(ID_COUNTER_FILE, "w") as f:
        f.write("1")
    with open(APPEARANCE_PROFILES_FILE, "w") as f:
        json.dump({}, f)
    print("Identity state reset for this run" + (f" (previous state backed up to {backup_dir})." if os.path.isdir(backup_dir) else "."))

    with open(CAMERA_ZONES_FILE, "r") as f:
        camera_zones = json.load(f)

    usd_context = omni.usd.get_context()
    usd_context.open_stage(USD_STAGE_PATH)
    for _ in range(60):
        simulation_app.update()

    stage = usd_context.get_stage()

    # Check for real (not guessed) whether a checkpoint camera actually
    # exists in this scene - see CHECKPOINT_AVAILABLE's comment above.
    global CHECKPOINT_AVAILABLE
    CHECKPOINT_AVAILABLE = stage.GetPrimAtPath(CHECKPOINT_PATH).IsValid()
    if not CHECKPOINT_AVAILABLE:
        print(f"No checkpoint camera found at {CHECKPOINT_PATH} in this scene - "
              f"checkpoint scanning disabled for this run (zone cameras + robot orbit search still work normally).")

    # Set up the physics/render decoupling (see the RENDER_EVERY_N_TICKS
    # comment near the top of this file) now that a stage is actually
    # loaded - SimulationContext needs a stage to attach to. Wrapped
    # defensively: if this build's SimulationContext doesn't accept these
    # exact kwargs or .step() doesn't support render=, _tick() already
    # falls back to plain simulation_app.update() (checked via
    # _sim_context is None / _render_decouple_supported), so a failure
    # here just means no speed gain, not a broken run.
    global _sim_context, _render_decouple_supported
    try:
        _sim_context = SimulationContext.instance() or SimulationContext()
        print("SimulationContext acquired - physics/render decoupling active "
              f"(rendering 1 in every {RENDER_EVERY_N_TICKS} ticks outside of active captures).")
    except Exception as e:
        _render_decouple_supported = False
        _sim_context = None
        print(f"  [WARN] Could not acquire SimulationContext ({e}) - "
              f"physics/render decoupling disabled, falling back to always-render ticks.")

    # The piper arm / TF-repair logic below was specific to the old heavy
    # mobile_manipulator_ros asset, which has been fully replaced by
    # create_lightweight_robot() - removed rather than left as dead code
    # that silently does nothing every run.

    # Bump ambient/dome light intensity so the scene is flat-lit and bright
    # enough for detection without needing expensive indirect
    # lighting/GI bounces to look right - requested speed lever, applied
    # directly to whatever light prims actually exist in the stage rather
    # than guessing a carb setting name for "ambient light".
    from pxr import UsdLux
    lights_adjusted = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.DistantLight) or prim.IsA(UsdLux.SphereLight):
            light = UsdLux.LightAPI(prim)
            intensity_attr = light.GetIntensityAttr()
            if intensity_attr:
                intensity_attr.Set(2.0)
            else:
                light.CreateIntensityAttr(2.0)
            lights_adjusted += 1
    print(f"  Adjusted {lights_adjusted} light prim(s) to intensity 2.0 for flatter/cheaper lighting.")

    # BUG (confirmed live via two debug-capture screenshots 8s apart,
    # pixel-identical - reported as "there's literally a guy who is just
    # straight up walking in place and not doing anything" while the log
    # showed both tracked people actively moving the whole time): PEOPLE
    # was trimmed down to Person1/Person2 only ("keep it to 3 people max")
    # by removing Person3's entry from that dict - but the scene file's
    # own pre-authored /World/xbot_person_3 prim never got touched by
    # that change. Nothing constructs a mover for it anymore, so nothing
    # ever calls MakeVisible/MakeInvisible or assigns it a walk clip -
    # it just sits at whatever visibility/pose the on-disk scene file (or
    # an earlier run, if this session ever saved back to it) left it in,
    # forever. Since the base scene evidently authors it visible with a
    # baked mid-stride rest pose, that's exactly what showed up: a fully
    # visible, completely static "person" nothing in this script was
    # aware of. Explicitly hide any pre-authored xbot_person_N prim that
    # ISN'T in today's PEOPLE dict, so trimming the roster can't leave an
    # orphaned, un-animated ghost standing around.
    for orphan_path in ["/World/xbot_person_1", "/World/xbot_person_2", "/World/xbot_person_3"]:
        if orphan_path not in [info["prim_path"] for info in PEOPLE.values()]:
            orphan_prim = stage.GetPrimAtPath(orphan_path)
            if orphan_prim.IsValid():
                UsdGeom.Imageable(orphan_prim).MakeInvisible()
                print(f"  [SETUP] hid unused scene character at {orphan_path} (not in this run's PEOPLE roster).")

    movers = []
    for i, (name, info) in enumerate(PEOPLE.items()):
        prim = stage.GetPrimAtPath(info["prim_path"])
        if not prim.IsValid():
            print(f"WARNING: {name} prim not found at {info['prim_path']} - skipping")
            continue
        disable_physics_recursive(prim)
        start = get_translate(prim)
        base_height = start[2]
        # Force spawn position into a REAL room box, alternating west/east,
        # instead of trusting whatever position happens to be authored in
        # the USD file - per explicit direction, people should only ever
        # occupy the actual room coordinates, never wherever they happened
        # to be placed in the scene file (they're invisible at this point
        # anyway, but this guarantees it's never wrong if anything ever
        # makes them visible unexpectedly).
        room = ROOM_BOUNDS["west"] if i % 2 == 0 else ROOM_BOUNDS["east"]
        spawn_x, spawn_y = random.uniform(room[0] + 1, room[1] - 1), random.uniform(room[2] + 1, room[3] - 1)
        model_prim = stage.GetPrimAtPath(info["prim_path"] + "/model")
        if not model_prim.IsValid():
            print(f"WARNING: {name} has no /model child prim - fall rotation will be a no-op for them.")
        model_offset = get_model_offset(model_prim)
        # Spawn heading is always 0 (fresh rotateZ) at this point, so a
        # flat subtraction is correct here - set_person_translate() isn't
        # usable yet since it needs the mover dict, which isn't built
        # until below.
        set_translate(prim, (spawn_x - model_offset[0], spawn_y - model_offset[1], base_height))
        (skel_prim, walk_clip_path, fall_clip_path, held_clip_path, fall_clip_duration,
         get_up_clip_path, get_up_clip_duration) = get_skeleton_and_clips(model_prim)
        if skel_prim is None:
            print(f"WARNING: {name} has no Skeleton under /model - fall animation will be a no-op for them.")
        if get_up_clip_path is None:
            print(f"  ({name} has no 'mixamo_get_up' clip - stand-up will be an instant cut to walking, no transition animation)")
        idle_clips = get_idle_clips(model_prim)
        if not idle_clips:
            print(f"  ({name} has no idle clips bound - run bind_idle_animations.py to add them; falling back to walking continuously)")
        mover_dict = {
            "name": name, "prim": prim, "model_prim": model_prim, "model_offset": model_offset,
            "skel_prim": skel_prim, "walk_clip_path": walk_clip_path, "fall_clip_path": fall_clip_path,
            "held_clip_path": held_clip_path, "fall_clip_duration": fall_clip_duration,
            "get_up_clip_path": get_up_clip_path, "get_up_clip_duration": get_up_clip_duration,
            "base_height": base_height, "start_delay": info["start_delay"],
            "loiter_start": None, "loiter_duration": None, "loiter_center": None,
            "loiter_was_active": False, "fall_center": None,
            "paused_until": 0, "fall_start": None, "fall_triggered": False, "fallen": False,
            "fall_held_triggered": False, "standing_up": False, "stand_up_start": None,
            "logical_x": spawn_x, "logical_y": spawn_y,  # see set_person_translate()'s docstring - the TRUE position, distinct from mover["prim"]'s own offset-compensated translate
            "wander_waypoint": None, "last_wander_move_time": None,  # ambient wandering - see update_person_position
            "idle_clips": idle_clips, "idling": False,  # see IDLE_PAUSE_CHANCE
            "angry_clip_path": get_named_clip(model_prim, "mixamo_angry_reaction"),
            "sad_idle_clip_path": get_named_clip(model_prim, "mixamo_sad_idle"),
            "sad_walk_clip_path": get_named_clip(model_prim, "mixamo_sad_walk"),
            "normal_walk_clip_path": walk_clip_path, "pending_denied_exit": False,
        }
        movers.append(mover_dict)
        # Everyone is visible and walking around continuously by default
        # now (per explicit direction - a demo where people just stand
        # invisible until a scripted event felt empty) - see
        # update_person_position's ambient-wander fallback below.
        UsdGeom.Imageable(prim).MakeVisible()
        if skel_prim is not None:
            set_animation_clip(skel_prim, walk_clip_path)

    # --- Intruder scenario(s): Person4 (and now Person5) ---
    # Per explicit direction ("spawn in another one, flag their id and
    # save it as someone who isn't allowed", later "assign 2 ids to the
    # door denial now so you can have multiple people denial") -
    # referenced fresh at runtime instead of being pre-authored in the
    # saved scene file, same approach as Nova Carter's visual earlier
    # this session (avoids touching the user's actual scene file at
    # all). Uses spawn_intruder() (generalized from what used to be a
    # one-off inline block here for Person4 alone - see that function's
    # own docstring). character_3.usd/character_4.usd are the two spare
    # Mixamo assets already sitting in assets/ with walk/fall/get-up
    # clips bound (see apply_walk_animation.py / bind_fall_animation.py /
    # bind_get_up_animation.py) - character_4.usd is free to reuse here
    # since VISITOR_SYSTEM_ENABLED is off, so it's not already spoken
    # for by the ambient-visitor system.
    person4 = spawn_intruder(stage, 0, "/World/xbot_person_4",
                              r"C:\isaacsim\projects\surveillance-proj\assets\character_3.usd", "Person4")
    movers.append(person4)

    # Zone-camera background-subtraction reference frames are established
    # via scan_zone_camera()'s own warm-up counter now (see
    # ZONE_WARMUP_SWEEPS) - an explicit dedicated pre-loop capture used to
    # live here, but confirmed via TWO real failed runs (even with a
    # retry-until-success loop, up to 20 attempts per camera) that
    # capture_frame() can reliably return None for every camera during
    # this specific early-setup window, for reasons not fully understood
    # (possibly some Replicator/render-product state that only settles
    # once the main tick loop is actually running). Rather than keep
    # fighting that, this now reuses the exact same code path as regular
    # sweeps - which we know eventually works - and just discards results
    # from the first few live sweeps per camera while re-baselining the
    # reference each time, before actually starting to detect.

    # --- Robot setup ---
    # Lightweight custom robot (see create_lightweight_robot) instead of the
    # old heavy mobile_manipulator_ros asset - no physics to disable, no
    # visuals to strip down, it was built minimal from the start.
    robot_base_prim, robot_camera_path = create_lightweight_robot(stage)
    base_pos = get_translate(robot_base_prim)

    robot_state = {
        "prim": robot_base_prim,
        "camera_path": robot_camera_path,
        "base_height": base_pos[2],
        "last_check": 0,
        "last_move_time": 0,
        "handled_alert_ids": set(),  # see next_alert_id's comment for why this keys on alert_id, not timestamp
        "current_target": None,
        "current_alert": None,
        "last_checkpoint_scan": -CHECKPOINT_SCAN_INTERVAL,
    }
    print(f"Lightweight robot ready, moving kinematically at {ROBOT_MOVE_SPEED} m/s, "
          f"stationed at checkpoint {CHECKPOINT_POS} watching the doorway.")

    # Switch the viewport from "Stage Lights" to "Camera Light" - confirmed
    # real API (not guessed) from an NVIDIA forum answer: the viewport menu
    # bar's lighting dropdown is driven by omni.kit.actions.core actions,
    # not a plain carb setting. "Camera Light" keeps whatever you're looking
    # through lit from the camera's own position, which matters here since
    # this scene's only real light is one dome light - stage-lit corners far
    # from it can otherwise render too dark to see clearly.
    try:
        import omni.kit.actions.core as _actions_core  # aliased on purpose - a bare "import omni.kit.actions.core" here rebinds the name "omni" as LOCAL to this whole function (Python function-scoping quirk), which broke the earlier omni.usd.get_context() call above with an UnboundLocalError
        action_registry = _actions_core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        action.execute()
        print("Viewport lighting mode set to Camera Light.")
    except Exception as e:
        print(f"Could not set viewport lighting mode to Camera Light ({e}) - "
              f"you can switch it manually via the lamp icon in the viewport menu bar.")

    # NOW play the timeline, after all kinematic flags are set - order matters here.
    timeline = omni.timeline.get_timeline_interface()

    # Enable looping for the Mixamo walk-cycle SkelAnimations. Without this
    # the clip plays through once and holds its final frame (characters
    # appear to stop walking mid-patrol even though their waypoint
    # translation - handled separately in update_person_position - keeps
    # moving them). Loop end is computed from the actual baked animation's
    # last real time sample rather than assumed, so it wraps exactly on
    # the clip boundary instead of stuttering mid-stride.
    anim_loop_seconds = get_animation_loop_seconds(stage, default=1.0)
    timeline.set_start_time(0.0)
    timeline.set_end_time(anim_loop_seconds)
    timeline.set_looping(True)
    print(f"Animation loop set: 0 -> {anim_loop_seconds:.3f}s, looping enabled.")

    timeline.play()
    for _ in range(20):
        simulation_app.update()

    # Live per-zone loitering streak tracker, persists across sweeps for the
    # whole run - see check_loitering() for why this replaced a log-based check.
    streak_state = {zone_name: {"streak": 0, "alerted": False} for zone_name in camera_zones.values()}

    # The sim clock (and with it every mover, timer and scripted event) does NOT
    # start here: _tick() only renders while sim_start is None. It starts after
    # the STARTUP_LOAD_SECONDS hold and the model workers being ready, below.
    sim_start = None
    last_sweep_time = 0
    last_overstay_check_time = 0
    authorized_ids_cache = load_authorized_ids()
    banned_ids_cache = load_banned_ids()

    # Populate shared _tick_state so _tick() (called both here and from
    # inside run_subprocess/capture_frame's blocking waits) has everything
    # it needs to keep movement/robot updates running continuously, even
    # during multi-second subprocess calls mid-sweep.
    _tick_state["sim_start"] = sim_start
    _tick_state["movers"] = movers
    _tick_state["robot_state"] = robot_state
    _tick_state["authorized_ids_cache"] = authorized_ids_cache
    _tick_state["banned_ids_cache"] = banned_ids_cache

    # Start the persistent YOLO/DeepFace workers ONCE here, before the main
    # loop. Both scripts' model loading happens exactly once now, not on
    # every single scan. wait_ready() blocks (while still calling _tick() so
    # physics/movement keep going) until each worker prints "READY".
    # Start both workers first so their models load in parallel with the hold.
    print("Starting persistent YOLO worker (loading model once)...")
    _workers["yolo"] = PersistentWorker(r"workers\yolo_worker.py")
    print("Starting persistent DeepFace worker (loading models once)...")
    _workers["deepface"] = PersistentWorker(r"workers\deepface_worker.py")

    print(f"Letting the scene and models load for {STARTUP_LOAD_SECONDS:.0f}s before anything starts moving...")
    hold_start = time.time()
    last_countdown = None
    while time.time() - hold_start < STARTUP_LOAD_SECONDS:
        _tick()
        remaining = int(STARTUP_LOAD_SECONDS - (time.time() - hold_start))
        if remaining % 10 == 0 and remaining != last_countdown:
            last_countdown = remaining
            print(f"  ... {remaining}s of startup load left")

    if not _workers["yolo"].wait_ready():
        print("WARNING: YOLO worker did not report ready in time - scans may fail or hang.")
    else:
        print("YOLO worker ready.")
    if not _workers["deepface"].wait_ready():
        print("WARNING: DeepFace worker did not report ready in time - scans may fail or hang.")
    else:
        print("DeepFace worker ready.")

    # Everything is loaded: NOW the clock starts.
    sim_start = time.time()
    _tick_state["sim_start"] = sim_start

    # REAL FIX for "I never saw anyone become visible": FALL_START_RANGE/
    # LOITER_START_RANGE used to count from raw sim start, but the YOLO/
    # DeepFace workers above can easily take 15-40+ real seconds to finish
    # loading models - the entire fall-and-recover or loiter-and-end cycle
    # could happen, invisibly, before you ever get a chance to actually
    # look at a running window. Rebasing both to count from THIS point
    # (workers confirmed ready) instead, so the timer only starts once
    # there's actually something worth watching on screen.
    ready_elapsed = time.time() - sim_start
    next_visitor_spawn_time = ready_elapsed + random.uniform(*VISITOR_SPAWN_INTERVAL_RANGE)
    intruder2_spawn_time = ready_elapsed + INTRUDER2_SPAWN_TIME
    intruder2_spawned = False
    last_door_check_time = 0.0
    pending_reentry = []  # (respawn_time, mover) for escorted-out / turned-away people, see revive_mover

    # REAL FIX for a total fall/loitering detection outage (confirmed live:
    # 0 detections across 80+ scans in one run, with a clearly-visible real
    # fall sitting in a saved debug frame the whole time) - the old
    # "adaptive" reference frame was, in practice, just ALWAYS the most
    # recently captured frame: warmup overwrites it on every attempt
    # regardless of content, and post-warmup it refreshes on any single
    # "not detected" read. That's fine when the room is genuinely empty
    # most of the time, but with up to 6 people now wandering almost
    # continuously (per the visitor system added this session), the room
    # is rarely if ever actually empty - confirmed by inspecting an early
    # saved frame from this exact run that clearly showed 3 people yet
    # still got accepted as an empty warmup read. Diffing every new frame
    # against "whatever was there last time" instead of a real empty
    # baseline means only FRAME-TO-FRAME motion gets caught, not "is
    # anyone here" - and if nothing changes much between two 8-second-
    # apart samples (very possible with several people around, some
    # relatively still), nothing ever trips the threshold.
    #
    # Real fix: capture one GUARANTEED-clean reference per zone camera
    # right here, once, with every mover actually hidden - same technique
    # already proven for keeping the robot out of zone captures. Marks
    # warmup as already complete for each camera so the existing
    # adaptive-refresh-on-confirmed-empty logic doesn't immediately
    # overwrite this real baseline with a busy frame on the very next
    # scan.
    print("  Establishing clean (everyone-hidden) reference frames for all zone cameras...")
    mover_visibility = {}
    for m in movers:
        imageable = UsdGeom.Imageable(m["prim"])
        mover_visibility[m["name"]] = imageable.ComputeVisibility() != UsdGeom.Tokens.invisible
        imageable.MakeInvisible()
    # The robot has to be hidden here too - every later zone scan hides it
    # (see scan_zone_camera), so a reference that still has it in frame
    # leaves a permanent "hole" blob wherever it happened to be standing at
    # startup (by now, usually parked in the reception counter that the
    # east-room cameras look straight at) - a phantom that reads as
    # "someone is here" on every sweep and can out-compete a real fallen
    # person for the "largest blob" the fall check tests.
    ref_robot_prim = robot_state["prim"]
    ref_robot_was_visible = UsdGeom.Imageable(ref_robot_prim).ComputeVisibility() != UsdGeom.Tokens.invisible
    UsdGeom.Imageable(ref_robot_prim).MakeInvisible()
    for _ in range(5):
        _tick(force_render=True)
    for camera_path, zone_name in camera_zones.items():
        bgr = capture_frame(camera_path, resolution=ZONE_SCAN_RESOLUTION)
        if bgr is not None:
            if not RECORDING_MODE:
                cv2.imwrite(os.path.join(DEBUG_DIR, f"REFERENCE_{zone_name}.jpg"), bgr)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            _zone_reference_frames[camera_path] = gray
            _zone_warmup_counts[camera_path] = ZONE_WARMUP_SWEEPS
            _zone_warmup_attempts[camera_path] = ZONE_WARMUP_SWEEPS
            print(f"    {zone_name}: clean reference captured.")
        else:
            print(f"    {zone_name}: WARNING - capture failed, falling back to the old adaptive warmup for this camera.")
    for m in movers:
        if mover_visibility.get(m["name"], True):
            UsdGeom.Imageable(m["prim"]).MakeVisible()
    if ref_robot_was_visible:
        UsdGeom.Imageable(ref_robot_prim).MakeVisible()

    # Each mover gets an INDEPENDENT chance at a scripted loiter window and
    # a scripted fall - not just one shared random pick for the whole run
    # - so multiple people can be doing different things (walking,
    # loitering, down and getting helped up) at the same time, per
    # explicit direction that the demo should look "full" instead of
    # having exactly one thing ever happening at once.
    if LOITER_ENABLED:
        for m in movers:
            if m.get("is_intruder"):
                continue  # Person4 has one job - walk in, get checked in, get turned away - not a general ambient wanderer
            if random.random() < LOITER_CHANCE:
                m["loiter_start"] = ready_elapsed + random.uniform(*LOITER_START_RANGE)
                m["loiter_duration"] = random.uniform(*LOITER_DURATION_RANGE)
                m["loiter_center"] = random.choice(list(ZONE_CENTERS.values()))
                print(f"*** {m['name']} will loiter near {m['loiter_center']} "
                      f"starting at t={m['loiter_start']:.1f}s for {m['loiter_duration']:.1f}s ***")

    if FALL_ENABLED:
        for m in movers:
            if m.get("is_intruder"):
                continue  # see the LOITER_ENABLED loop's comment just above
            if random.random() < FALL_CHANCE:
                m["fall_start"] = ready_elapsed + random.uniform(*FALL_START_RANGE)
                # Pick a zone (for which camera should see it) but DON'T
                # fall exactly on its raw ZONE_CENTERS coordinate - those
                # are literally each camera's own mount position, right
                # against a wall/corner (confirmed directly by the user
                # watching a real run: a person fell "right underneath
                # the camera" and clipped into the wall). Land somewhere
                # well inside that zone's actual ROOM instead - the same
                # room-interior random pick loitering already uses
                # successfully, with a bigger margin since a fallen
                # pose's footprint is wider than a standing one.
                fall_zone_name, fall_zone_pos = random.choice(list(ZONE_CENTERS.items()))
                room = ROOM_BOUNDS["west"] if fall_zone_pos[0] < 21.75 else ROOM_BOUNDS["east"]
                fx = random.uniform(room[0] + 3, room[1] - 3)
                fy = random.uniform(room[2] + 3, room[3] - 3)
                # Test hook: SIM_FALL_AT="Person2:14.65,15.06" pins one mover's fall spot so a
                # detection failure seen at a random location can be reproduced exactly.
                pin = __import__("os").environ.get("SIM_FALL_AT", "")
                if pin.startswith(m["name"] + ":"):
                    fx, fy = (float(v) for v in pin.split(":", 1)[1].split(","))
                m["fall_center"] = (fx, fy)
                print(f"*** {m['name']} will fall (scripted) near {fall_zone_name} "
                      f"at ({fx:.2f},{fy:.2f}) at t={m['fall_start']:.1f}s ***")

    print("=== Unified tracking + movement loop started. Press Ctrl+C to stop. ===")

    try:
        while True:
            elapsed = time.time() - sim_start

            _tick()

            # Ambient visitor population - "people come in and out all the
            # time like a real area." Spawn a new one whenever we're under
            # the concurrent cap and the next scheduled spawn time has
            # arrived, and remove anyone update_person_position flagged
            # should_despawn (removed here, never mid-iteration inside
            # update_person_position itself, which would be a real bug -
            # _tick() loops over _tick_state["movers"] every call).
            if VISITOR_SYSTEM_ENABLED and elapsed >= next_visitor_spawn_time:
                next_visitor_spawn_time = elapsed + random.uniform(*VISITOR_SPAWN_INTERVAL_RANGE)
                active_visitor_count = sum(1 for m in movers if m.get("is_visitor") and not m.get("should_despawn"))
                if active_visitor_count < MAX_CONCURRENT_VISITORS:
                    new_visitor = spawn_visitor(stage, elapsed)
                    movers.append(new_visitor)
                    _tick_state["movers"] = movers

            # Second intruder ("Person5") - see INTRUDER2_SPAWN_TIME's
            # comment for why this is staggered instead of spawning
            # alongside Person4 at startup. One-shot (intruder2_spawned
            # guards against re-firing every tick once the time passes).
            if not intruder2_spawned and elapsed >= intruder2_spawn_time and not door_zone_busy(movers):
                intruder2_spawned = True
                person5 = spawn_intruder(stage, elapsed, "/World/xbot_person_5",
                                          VISITOR_ASSET_PATH, "Person5")
                movers.append(person5)
                _tick_state["movers"] = movers

            # Door diagnostics (cheap): flag two people overlapping in the door
            # zone, or anyone standing inside the door frame itself (the gap is
            # x:[39.75,40.20], y:[17.4,19.46]; outside that y-range at those x
            # is solid wall).
            if elapsed - last_door_check_time >= 0.5:
                last_door_check_time = elapsed
                active = [m for m in movers if not m.get("should_despawn") and m.get("logical_x") is not None]
                for i, a in enumerate(active):
                    ax, ay = a["logical_x"], a["logical_y"]
                    if 39.75 <= ax <= 40.20 and not (17.4 <= ay <= 19.46):
                        if elapsed - a.get("last_frame_warn", -99) > 5:
                            a["last_frame_warn"] = elapsed
                            print(f"  [DOOR] WARNING: {a['name']} is inside the door frame at ({ax:.2f},{ay:.2f}) "
                                  f"[state={a.get('visitor_state')}, waypoint={a.get('wander_waypoint')}, via={a.get('route_via')}]")
                    for b in active[i + 1:]:
                        bx, by = b["logical_x"], b["logical_y"]
                        if in_door_zone(ax, ay) and in_door_zone(bx, by) and math.hypot(ax - bx, ay - by) < 0.45:
                            key = "last_overlap_warn_" + b["name"]
                            if elapsed - a.get(key, -99) > 5:
                                a[key] = elapsed
                                print(f"  [DOOR] WARNING: {a['name']} and {b['name']} overlap in the doorway "
                                      f"({math.hypot(ax - bx, ay - by):.2f}m apart)")

            despawned = [m for m in movers if m.get("should_despawn")]
            if despawned:
                # BUG (confirmed live via two debug-capture screenshots 8s
                # apart, pixel-identical - reported as "there's literally
                # a guy who is just straight up walking in place and not
                # doing anything" even though the log clearly showed
                # "Person4 left the premises" already fired): the
                # MakeInvisible() call that happens inside
                # update_person_position's exiting-despawn branch doesn't
                # necessarily flush to the actual RENDERED frame the same
                # tick it's called - same underlying issue already fixed
                # once for hiding the ROBOT during zone captures (see
                # scan_zone_camera's own comment: confirmed via a real
                # screenshot that a MakeInvisible() call needed a few
                # forced-render ticks afterward before it actually took
                # visible effect). The difference here is much worse: once
                # a mover is stripped from the movers list right below,
                # NOTHING ever touches that prim again for the rest of the
                # run - if the invisibility hadn't actually rendered yet
                # at that exact moment, it never will, leaving a
                # permanent, fully-visible "ghost" frozen in its exact
                # last pose forever. Forcing a few real renders here,
                # before dropping them from tracking, gives the
                # visibility change an actual chance to land.
                for _ in range(3):
                    _tick(force_render=True)
                for gone in despawned:
                    if gone["name"].startswith("Visitor"):
                        continue  # ambient random visitors are not recycled
                    delay = INTRUDER_RETRY_DELAY if gone.get("is_intruder") else RESIDENT_REENTRY_DELAY
                    pending_reentry.append((elapsed + delay, gone))
                movers = [m for m in movers if not m.get("should_despawn")]
                _tick_state["movers"] = movers

            if pending_reentry:
                due = [(t, m) for t, m in pending_reentry if elapsed >= t]
                # one arrival at a time, and only when the doorway is clear
                due = due[:1] if not door_zone_busy(movers) else []
                for t, m in due:
                    pending_reentry.remove((t, m))
                    revive_mover(m, elapsed)
                    movers.append(m)
                    _tick_state["movers"] = movers

            # Overstay time-limit check - per explicit direction ("the
            # robot will keep in their memory this id has stayed in here
            # for 5 minutes... if they go over 1 minute then the robot go
            # finds them and escorts them away"). Cheap pure bookkeeping
            # (no camera capture), so this runs on its own short interval
            # rather than piggybacking on the full camera sweep below.
            # overstay_dispatched guards against re-creating a fresh
            # dispatch_alert every single check while the robot is still
            # en route to handle the one already queued.
            if elapsed - last_overstay_check_time >= 5.0:
                last_overstay_check_time = elapsed
                first_seen = _tick_state.get("person_first_seen", {})
                for m in movers:
                    pid = m.get("known_id")
                    if not pid or m.get("should_despawn") or m.get("overstay_dispatched") or m.get("fallen") or m.get("standing_up"):
                        continue
                    if m.get("is_visitor") and m.get("visitor_state") == "exiting":
                        continue  # already leaving for some other reason (e.g. banned-escort)
                    seen_at = first_seen.get(pid)
                    if seen_at is None or elapsed - seen_at <= VISIT_TIME_LIMIT:
                        continue
                    m["overstay_dispatched"] = True
                    mx = m.get("logical_x", get_translate(m["prim"])[0])
                    my = m.get("logical_y", get_translate(m["prim"])[1])
                    print(f"  [OVERSTAY] {m['name']} ({pid}) has been here {elapsed - seen_at:.0f}s "
                          f"(limit {VISIT_TIME_LIMIT:.0f}s) - dispatching the robot to escort them out")
                    append_log_entry({
                        "event_type": "dispatch_alert",
                        "alert_id": next_alert_id(),
                        "reason": "overstay",
                        "person_id": pid,
                        "zone": None,
                        "coords": (mx, my),
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    })

            if elapsed - last_sweep_time >= SCAN_INTERVAL_SECONDS:
                authorized_ids_cache = load_authorized_ids()
                banned_ids_cache = load_banned_ids()
                _tick_state["authorized_ids_cache"] = authorized_ids_cache
                _tick_state["banned_ids_cache"] = banned_ids_cache
                run_sweep(camera_zones, authorized_ids_cache, banned_ids_cache, streak_state, movers, elapsed, robot_state["prim"])
                last_sweep_time = elapsed

    except KeyboardInterrupt:
        print("\n=== Stopped by user. ===")
    finally:
        timeline.stop()
        for worker in _workers.values():
            if worker is not None and worker.proc.poll() is None:
                try:
                    worker.proc.stdin.write("__EXIT__\n")
                    worker.proc.stdin.flush()
                except Exception:
                    pass
                worker.proc.terminate()


if __name__ == "__main__":
    main()
    simulation_app.close()