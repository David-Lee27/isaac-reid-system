"""
Runs DeepFace re-identification: checks if the face in the given image
matches a known person in the known_faces/ database. If yes, returns
their existing ID. If no, signals that this is a new person.

Usage:
    python.bat deepface_reid.py <path_to_captured_image> <path_to_known_faces_folder>
"""
import sys
import json
import os

sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
def main():
    if len(sys.argv) < 3:
        print(json.dumps({"error": "usage: deepface_reid.py <image_path> <known_faces_folder>"}))
        return

    image_path = sys.argv[1]
    db_path = sys.argv[2]

    from deepface import DeepFace

    # If the known_faces folder is empty, there's nothing to match against —
    # this is automatically a new person
    has_known_faces = any(
        f.lower().endswith((".jpg", ".jpeg", ".png"))
        for f in os.listdir(db_path)
    ) if os.path.isdir(db_path) else False

    if not has_known_faces:
        print(json.dumps({"match_found": False, "reason": "no known faces yet"}))
        return

    try:
        results = DeepFace.find(
            img_path=image_path,
            db_path=db_path,
            enforce_detection=True,   # changed from False - diagnostic test
            detector_backend="opencv",
            model_name="Facenet",
            silent=True
        )

        # results is a list of dataframes (one per detected face). We only
        # care about the first detected face for now (single-person frames).
        if len(results) == 0 or results[0].empty:
            print(json.dumps({"match_found": False, "reason": "no match in database"}))
            return

        best_match = results[0].iloc[0]
        matched_identity_path = str(best_match["identity"])
        distance = float(best_match["distance"])

        print(json.dumps({
            "match_found": True,
            "matched_path": matched_identity_path,
            "distance": distance
        }))

    except ValueError as e:
        # DeepFace raises ValueError when no face is detected in the image at all
        print(json.dumps({"match_found": False, "reason": f"no face detected: {str(e)}"}))

if __name__ == "__main__":
    main()