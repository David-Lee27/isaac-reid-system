r"""
Runs DeepFace re-identification: checks if the face in the given image
matches a known person in the known_faces/ database.

Returns one of three explicit statuses:
  "match"            - found a matching known face
  "new"               - face was detected fine, just doesn't match anyone known
  "detection_failed"  - could not detect/read a face in the image at all
                        (should NOT be treated as a new person)

Usage:
    python.bat deepface_reid.py <path_to_captured_image> <path_to_known_faces_folder>
"""
import sys
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")

import json
import os

def main():
    if len(sys.argv) < 3:
        print(json.dumps({"status": "detection_failed", "reason": "usage error"}))
        return

    image_path = sys.argv[1]
    db_path = sys.argv[2]

    from deepface import DeepFace

    # Delete any stale DeepFace cache files before searching - we're actively
    # adding new reference images between calls, and a stale cache can cause
    # real matches to be silently missed.
    if os.path.isdir(db_path):
        for f in os.listdir(db_path):
            if f.endswith(".pkl"):
                os.remove(os.path.join(db_path, f))

    has_known_faces = any(
        f.lower().endswith((".jpg", ".jpeg", ".png"))
        for f in os.listdir(db_path)
    ) if os.path.isdir(db_path) else False

    # First, confirm a face can even be detected in this image at all,
    # independent of whether it matches anyone known.
    try:
        DeepFace.extract_faces(
            img_path=image_path,
            detector_backend="retinaface",
            enforce_detection=True
        )
    except ValueError as e:
        print(json.dumps({"status": "detection_failed", "reason": str(e)}))
        return

    if not has_known_faces:
        print(json.dumps({"status": "new", "reason": "no known faces yet"}))
        return

    try:
        results = DeepFace.find(
            img_path=image_path,
            db_path=db_path,
            enforce_detection=True,
            detector_backend="retinaface",
            model_name="Facenet",
            silent=True
        )

        if len(results) == 0 or results[0].empty:
            print(json.dumps({"status": "new", "reason": "no match in database"}))
            return

        best_match = results[0].iloc[0]
        print(json.dumps({
            "status": "match",
            "matched_path": str(best_match["identity"]),
            "distance": float(best_match["distance"])
        }))

    except ValueError as e:
        print(json.dumps({"status": "detection_failed", "reason": str(e)}))

if __name__ == "__main__":
    main()