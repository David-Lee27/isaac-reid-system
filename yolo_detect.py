"""
Runs YOLOv8 person detection on a single image and prints results as JSON.
Called via subprocess from inside Isaac Sim's script editor.

Usage:
    python.bat yolo_detect.py <path_to_image>
"""
import sys
import json

def main():
    if len(sys.argv) < 2:
        print(json.dumps({"error": "no image path provided"}))
        return

    image_path = sys.argv[1]

    from ultralytics import YOLO

    # yolov8n.pt = "nano" model, smallest/fastest, good enough for this project
    # first run will auto-download the weights (~6MB), needs internet once
    model = YOLO("yolov8n.pt")

    results = model.predict(source=image_path, classes=[0], verbose=False)
    # classes=[0] means "person" only (COCO class 0 = person)

    detections = []
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            detections.append({
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "confidence": conf
            })

    print(json.dumps({"detections": detections}))

if __name__ == "__main__":
    main()