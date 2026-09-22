r"""
analyze_run.py - summarises a finished (or running) simulation from Kit's engine log.

Prints fall detection / assist latency, false alarms, door check-in latency, which
ID each person ended up with, who got banned, overstay escorts, returns, and the
door/animation/stuck diagnostics - the things that tell you whether a run behaved.

    C:\isaacsim\python.bat analyze_run.py            # newest log
    C:\isaacsim\python.bat analyze_run.py <log path>
"""
import collections
import glob
import json
import os
import re
import sys

LOG_GLOB = r"C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.0\kit_*.log"
# banned_ids.json lives at the PROJECT ROOT, not next to this file - this now
# lives in reporting/, so go up one level.
BANNED_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "banned_ids.json")

log = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(LOG_GLOB))[-1]
rx = re.compile(r"\[(\d[\d,]*)ms\].*\[py (?:stdout|stderr)\]: ?(.*)")
ev = []
for line in open(log, encoding="utf-8", errors="replace"):
    m = rx.search(line)
    if m:
        ev.append((int(m.group(1).replace(",", "")) / 1000.0, m.group(2).rstrip()))


def find(pat):
    return [(t, s) for t, s in ev if re.search(pat, s)]


print(f"log: {os.path.basename(log)}  uptime {ev[-1][0] if ev else 0:.0f}s")

falls = find(r"has fallen \(scripted")
fdet = find(r"FALL DETECTED")
dup = [e for e in fdet if ("no second dispatch" in e[1] or "holding this one" in e[1])]
assist = find(r"helping .* back up")
false_alarm = find(r"nobody is down anymore")
print(f"scripted falls {len(falls)} | FALL DETECTED {len(fdet)} (duplicates suppressed {len(dup)}) | "
      f"assists {len(assist)} | false alarms {len(false_alarm)}")
for t, s in falls:
    who = re.search(r"\*\*\* (\w+) has fallen", s).group(1)
    d = next((tt for tt, ss in fdet if tt >= t and "no second dispatch" not in ss and "holding this one" not in ss), None)
    a = next((tt for tt, ss in assist if tt >= t and who in ss), None)
    print(f"   {who} fell {t:.0f}s -> detected +{(d - t) if d else float('nan'):.1f}s -> helped +{(a - t) if a else float('nan'):.1f}s")

arrivals = [(t, re.search(r"\] (\w+) (?:is back at the door|entering through the door)", s))
            for t, s in ev if re.search(r"is back at the door|entering through the door", s)]
lat = []
for t, m in arrivals:
    name = m.group(1)
    r = next((tt for tt, ss in ev if tt > t and re.search(rf"{name} \(ID_\d+\) (checked in|DENIED)", ss)), None)
    lat.append((name, t, (r - t) if r else None))
vals = [l for _, _, l in lat if l is not None]
print(f"check-ins {len(lat)} (completed {len(vals)}): " + (f"mean {sum(vals) / len(vals):.1f}s, max {max(vals):.0f}s" if vals else "none"))

print("   per check-in: " + ", ".join(f"{n}@{t:.0f}s:{'-' if l is None else f'{l:.0f}s'}" for n, t, l in lat))

ids = collections.defaultdict(collections.Counter)
for _, s in ev:
    m = re.search(r"(Person\d) \((ID_\d+)\)", s) or re.search(r"injury log: (Person\d) identified as (ID_\d+)", s)
    if m:
        ids[m.group(1)][m.group(2)] += 1
print("ids per person:", {k: dict(v) for k, v in sorted(ids.items())})
try:
    print("banned:", json.load(open(BANNED_FILE)).get("banned", []))
except Exception:
    pass
wrong = [(t, s) for t, s in ev if re.search(r"escorting (Person[12]) \(", s) and "overstayed" not in s]
print(f"residents escorted as banned intruders: {len(wrong)}")

for k, pat in [("overstay escorts", r"has overstayed"), ("returns", r"is back at the door"), ("DENIED", r"DENIED"),
               ("checked in", r"checked in - cleared"), ("robot stuck", r"stuck near"), ("person UNSTICK", r"UNSTICK"),
               ("door overlaps/frame", r"\[DOOR\]"), ("animation corrections", r"\[ANIM\].*(without|restoring)"),
               ("face-match errors caught", r"face-match error"), ("duplicate ids avoided", r"keeping that id"),
               ("Python errors", r"Traceback")]:
    print(f"{k:26s}{len(find(pat))}")
