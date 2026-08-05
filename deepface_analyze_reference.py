import cv2
import subprocess
import json
import os
from isaacsim.sensors.camera import Camera
import omni.kit.app

camera = Camera(
    prim_path="/World/Camera",
    resolution=(640, 480),
)
camera.initialize()

app = omni.kit.app.get_app()
for _ in range(100):
    app.update()

def capture_and_analyze():
    rgba = camera.get_rgba()
    if rgba is None or rgba.size == 0:
        print("Still no frame — camera/viewport may not be ready yet.")
        return

    rgb = rgba[:, :, :3]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    img_path = os.path.join(os.environ["TEMP"], "isaacsim_capture.jpg")
    success = cv2.imwrite(img_path, bgr)
    if not success:
        print(f"Failed to write image to {img_path}")
        return
    print(f"Saved capture to {img_path}")

    deepface_script_path = os.path.join(os.environ["TEMP"], "run_deepface.py")
    with open(deepface_script_path, "w") as f:
        f.write(f"""
from deepface import DeepFace
import json
results = DeepFace.analyze(
    img_path=r"{img_path}",
    actions=['age', 'gender', 'emotion', 'race'],
    enforce_detection=False,
    detector_backend='opencv'
)
print(json.dumps(results, default=str))
""")

    cmd = f'"C:\\isaacsim\\python.bat" "{deepface_script_path}"'
    proc = subprocess.run(
        cmd, capture_output=True, text=True, shell=True
    )

    for line in proc.stdout.strip().splitlines():
        if line.startswith("["):
            results = json.loads(line)
            for i, face in enumerate(results):
                print(f"--- Face {i+1} ---")
                print(f"Age: {face['age']}")
                print(f"Gender: {face['dominant_gender']}")
                print(f"Emotion: {face['dominant_emotion']}")
                print(f"Race: {face['dominant_race']}")
            return

    print("Could not parse DeepFace output.")
    print("Return code:", proc.returncode)
    print("STDOUT:", proc.stdout)
    print("STDERR:", proc.stderr)

capture_and_analyze()