"""
yolo_worker.py

Persistent YOLO worker - loads the model ONCE, then reads image paths from
stdin (one per line) and writes JSON detection results to stdout (one per
line), for as long as the process lives.

Replaces the old yolo_detect.py, which was spawned fresh via subprocess for
every single scan (5-6 times per 8-second sweep cycle) and reloaded the
entire model from disk every time - confirmed to be the dominant real cost
behind the whole session's "everything is slow" symptoms, far more than
render resolution or GPU rendering contention.

Started once by unified_tracking.py at startup and kept alive for the
whole run via PersistentWorker (see unified_tracking.py).
"""
import sys
import json

def main():
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")
    print("READY", flush=True)

    for line in sys.stdin:
        image_path = line.strip()
        if not image_path:
            continue
        if image_path == "__EXIT__":
            break
        try:
            results = model.predict(source=image_path, classes=[0], verbose=False)
            detections = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    detections.append({
                        "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                        "confidence": conf
                    })
            print(json.dumps({"detections": detections}), flush=True)
        except Exception as e:
            print(json.dumps({"error": str(e)}), flush=True)

if __name__ == "__main__":
    main()
