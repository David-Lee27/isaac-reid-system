# Isaac Sim Multi-Camera Person Re-Identification & Surveillance System

A simulated surveillance pipeline built in NVIDIA Isaac Sim: fixed zone cameras detect falls and loitering across a two-room building, then dispatch a mobile robot that uses its own onboard camera and face re-identification to locate the person and verify whether they're authorized.

Built as a portfolio project for robotics/AI internship applications — developed iteratively, one working piece at a time, over several weeks.

## About Me

I'm David, a student building out a robotics/AI portfolio with a focus on perception and multi-camera tracking systems, aimed at internship applications and robotics lab opportunities (including NTU's Intelligent Robotics & Automation Lab). This project was my way of learning person re-identification, simulation-based robotics, and sensor pipelines from the ground up — I had no prior ML/CV experience going in.

## What it does

1. **Zone monitoring** — four fixed cameras cover two rooms. Each one runs YOLOv8 person detection locally and flags anomalies: a fall (via pose/aspect-ratio heuristics) or loitering (via presence-over-time tracking).
2. **Dispatch** — when a zone camera raises an alert, a mobile robot is dispatched toward the event location.
3. **Robot-side verification** — the robot navigates to the scene (with real wall-collision and doorway routing, not a straight-line teleport), then performs a physical orbit search around the target, capturing frames from multiple standpoints.
4. **Re-identification** — captured faces are matched against a known-persons gallery using DeepFace embeddings, to confirm identity and check authorization.

## Architecture

The system is organized into four layers:

- **Sensing** — Isaac Sim cameras (4 fixed zone cameras + 1 robot-mounted camera), `isaacsim.sensors.camera.Camera`
- **Processing** — YOLOv8 (person/fall detection), DeepFace (face embedding + re-ID), run as persistent worker subprocesses so models load once instead of per-frame
- **Decision** — rule-based anomaly logic (fall, loitering, unauthorized entry, rapid movement) plus a dispatch/orbit-search controller for the robot
- **Output** — event log (`event_log.json`), planned FastAPI + SQLite backend with a live dashboard frontend

## Tech stack

Python · NVIDIA Isaac Sim 6.0 · YOLOv8 · DeepFace · OpenCV · (planned) FastAPI + SQLite

## Repo layout

| Path | What it is |
|---|---|
| `unified_tracking.py` | Main simulation script — scene setup, cameras, robot control, detection loop |
| `run_sim_muted.bat` | Entry point for running the sim (handles console buffering issues that running `python.bat` directly does not) |
| `yolo_worker.py`, `deepface_worker.py` | Persistent subprocess workers for detection/re-ID models |
| `dispatch_bridge.py` | Robot dispatch and orbit-search logic |
| `camera_zones.json`, `authorized_ids.json`, `event_log.json` | Runtime config and state |
| `diagnose_*.py` | One-off diagnostic scripts used to debug specific integration issues (camera mounting, room bounds, character rigging, etc.) — kept as a record of the debugging process |
| `known_faces/` | Reference gallery used for re-ID matching |
| `NOTES.md` | Running engineering log kept throughout development |

Large binary scene assets (raw `.fbx` character exports, textures, debug camera captures, nav stack logs) are excluded via `.gitignore` to keep the repo focused on the actual system code.

## Current status

**Working / confirmed:**
- Scene loads with a two-room layout and functioning doorway between them
- Robot patrols a 9-waypoint route with real wall-collision and doorway-routing (not free teleport)
- Fall and loiter behaviors trigger correctly for simulated people
- Zone camera resolution bumped 160×120 → 640×480 after finding people were only a few blurry pixels at the old resolution
- Persistent YOLO/DeepFace subprocess workers, decoupled physics/render stepping — sim runs at 150–200+ steps/sec vs. ~12 originally

**Known open issue:** Camera3 and Camera4's raw frames appear mismatched against their assigned zone names — a fall assigned to Camera3's zone only ever shows up in Camera4's frames. Not yet root-caused; next step is a targeted diagnostic comparing labeled frames from all four cameras against a known test marker before touching the zone-to-camera mapping again.

## Roadmap

- [ ] Fix the Camera3/Camera4 zone mismatch
- [ ] Full clean end-to-end run validation
- [ ] FastAPI + SQLite backend with live dashboard
- [ ] Benchmark metrics (FPS, detection latency, re-ID accuracy)
- [ ] Demo video

---

*This project is under active development as part of an ongoing internship application portfolio.*
