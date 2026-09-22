r"""
deepface_worker.py

Persistent DeepFace worker - loads models ONCE (RetinaFace detector +
Facenet embedding model both get lazily loaded by DeepFace on first real
call, and stay resident for the life of this process), then reads
tab-separated "image_path\tdb_path" requests from stdin and writes JSON
results to stdout, one per line, for as long as the process lives.

Replaces the old deepface_reid.py, which was spawned fresh via subprocess
for every single scan and reloaded BOTH models from disk every time - the
single biggest cost behind the whole session's slowness.

Deliberately KEEPS the per-call ".pkl cache deletion + full known_faces/
re-embed" behavior from the original script for correctness (avoids stale-
match bugs since new reference images get added mid-run) - this is a
smaller, secondary cost now that model loading itself isn't repeated every
call, and a further optimization opportunity if still too slow, not
something to risk correctness on tonight.

Started once by unified_tracking.py at startup and kept alive for the
whole run via PersistentWorker (see unified_tracking.py).
"""
import sys
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")

import json
import os

def main():
    from deepface import DeepFace
    print("READY", flush=True)

    for line in sys.stdin:
        raw = line.strip()
        if not raw:
            continue
        if raw == "__EXIT__":
            break
        parts = raw.split("\t")
        if len(parts) < 2:
            print(json.dumps({"status": "detection_failed", "reason": "bad request"}), flush=True)
            continue
        image_path, db_path = parts[0], parts[1]

        try:
            if os.path.isdir(db_path):
                for f in os.listdir(db_path):
                    if f.endswith(".pkl"):
                        os.remove(os.path.join(db_path, f))

            has_known_faces = any(
                f.lower().endswith((".jpg", ".jpeg", ".png"))
                for f in os.listdir(db_path)
            ) if os.path.isdir(db_path) else False

            try:
                faces = DeepFace.extract_faces(
                    img_path=image_path,
                    detector_backend="retinaface",
                    enforce_detection=True,
                    align=False,
                )
            except ValueError as e:
                print(json.dumps({"status": "detection_failed", "reason": str(e)}), flush=True)
                continue

            # Frontal-ness gate. Confirmed via a direct DeepFace.verify
            # diagnostic on 4 real captured crops (3 confirmed-same-person,
            # 1 confirmed-different) that near-pure side-profile crops
            # (this scene's robot orbits at fixed angles independent of
            # which way the person is actually facing) produce embedding
            # distances that do NOT separate same-person from
            # different-person pairs, under Facenet, Facenet512, AND
            # VGG-Face alike - not a distance-threshold problem, a pose
            # problem. The two eyes' horizontal separation as a fraction of
            # face width collapses toward 0 in profile and grows toward
            # ~0.3+ facing the camera - confirmed on those same 4 crops
            # (0.098/0.227/0.314/0.114). Rejecting low-ratio captures here
            # forces every accepted registration/match to be a reasonably
            # front-on shot, so comparisons are apples-to-apples instead of
            # comparing two very differently-posed shots of the same person.
            area = faces[0]["facial_area"]
            left_eye, right_eye = area.get("left_eye"), area.get("right_eye")
            face_w = area.get("w", 0)
            frontal_ratio = (abs(right_eye[0] - left_eye[0]) / face_w) if (left_eye and right_eye and face_w) else 0.0
            FRONTAL_RATIO_MIN = 0.22
            if frontal_ratio < FRONTAL_RATIO_MIN:
                print(json.dumps({
                    "status": "detection_failed",
                    "reason": f"face too non-frontal to trust (ratio={frontal_ratio:.3f} < {FRONTAL_RATIO_MIN})"
                }), flush=True)
                continue

            if not has_known_faces:
                print(json.dumps({"status": "new", "reason": "no known faces yet"}), flush=True)
                continue

            results = DeepFace.find(
                img_path=image_path,
                db_path=db_path,
                enforce_detection=True,
                detector_backend="retinaface",
                model_name="Facenet",
                silent=True
            )

            if len(results) == 0 or results[0].empty:
                print(json.dumps({"status": "new", "reason": "no match in database"}), flush=True)
                continue

            best_match = results[0].iloc[0]
            print(json.dumps({
                "status": "match",
                "matched_path": str(best_match["identity"]),
                "distance": float(best_match["distance"])
            }), flush=True)

        except ValueError as e:
            print(json.dumps({"status": "detection_failed", "reason": str(e)}), flush=True)
        except Exception as e:
            print(json.dumps({"status": "detection_failed", "reason": str(e)}), flush=True)

if __name__ == "__main__":
    main()
