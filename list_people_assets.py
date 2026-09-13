"""
ISOLATED, read-only - lists the real contents of NVIDIA's Isaac Sim People
asset folders using omni.client (the actual Nucleus/asset-server directory
listing API), instead of guessing filenames one at a time (which the
previous test showed doesn't work reliably via Sdf.Layer.FindOrOpen).

Run from PowerShell:
    C:\\isaacsim\\python.bat C:\\isaacsim\\projects\\surveillance-proj\\list_people_assets.py
"""

HEADLESS = False

FOLDERS_TO_LIST = [
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/DH_Characters/",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/DH_Characters_Extended/",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/MotionLibrary/",
]

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.client


def list_folder(url):
    print(f"\nListing: {url}")
    result, entries = omni.client.list(url)
    print(f"  result: {result}")
    if entries:
        for entry in entries:
            flag = "[DIR]" if entry.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN else "     "
            print(f"  {flag} {entry.relative_path}")
    else:
        print("  (no entries returned)")


def main():
    for _ in range(30):
        simulation_app.update()
    for folder in FOLDERS_TO_LIST:
        try:
            list_folder(folder)
        except Exception as e:
            print(f"  ERROR listing {folder}: {e}")
    print("\nDONE. Paste this whole output back.")


if __name__ == "__main__":
    main()
    simulation_app.close()
