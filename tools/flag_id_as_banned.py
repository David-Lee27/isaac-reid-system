r"""
flag_id_as_banned.py

Adds a person ID to the banned/watchlist (banned_ids.json), which fires a
real INTRUDER ALERT the next time the robot's face-ID recognizes that
person - a distinct, higher-severity event from the ordinary "not
authorized" logging an unregistered visitor gets.

This models a real security workflow: the robot registers anyone new it
sees during normal operation (an ID like ID_0004 shows up in
known_faces/), and a human operator later decides - after the fact - that
this specific person shouldn't be allowed back. That decision is what this
script records. It does NOT require re-running the simulation or
re-capturing a face; the face is already enrolled in known_faces/ from
whenever the robot first saw them.

Usage:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\flag_id_as_banned.py ID_0004

Run with no arguments to just list current known IDs and the current
banned list.
"""

import json
import os
import sys

# banned_ids.json and known_faces/ live at the PROJECT ROOT, not next to this
# file - this now lives in tools/, so go up one level.
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BANNED_IDS_FILE = os.path.join(PROJECT_DIR, "banned_ids.json")
KNOWN_FACES_DIR = os.path.join(PROJECT_DIR, "known_faces")


def load_banned():
    if not os.path.exists(BANNED_IDS_FILE):
        return []
    with open(BANNED_IDS_FILE, "r") as f:
        return json.load(f).get("banned", [])


def save_banned(banned_list):
    with open(BANNED_IDS_FILE, "w") as f:
        json.dump({"banned": sorted(set(banned_list))}, f, indent=2)


def known_ids():
    if not os.path.isdir(KNOWN_FACES_DIR):
        return []
    return sorted(
        os.path.splitext(f)[0]
        for f in os.listdir(KNOWN_FACES_DIR)
        if f.lower().endswith(".jpg")
    )


def main():
    banned = load_banned()

    if len(sys.argv) < 2:
        print(f"Known registered IDs: {known_ids()}")
        print(f"Currently banned: {banned}")
        print("\nUsage: flag_id_as_banned.py <ID_XXXX> [<ID_YYYY> ...]")
        return

    ids_to_add = sys.argv[1:]
    known = known_ids()
    for person_id in ids_to_add:
        if person_id not in known:
            print(f"WARNING: {person_id} not found in known_faces/ - it hasn't been "
                  f"registered by the robot yet. Adding it anyway, but the intruder "
                  f"alert can only fire once this ID has actually been seen/enrolled.")
        if person_id in banned:
            print(f"{person_id} is already banned - no change.")
        else:
            banned.append(person_id)
            print(f"Flagged {person_id} as banned.")

    save_banned(banned)
    print(f"\nUpdated banned list: {sorted(set(banned))}")


if __name__ == "__main__":
    main()
