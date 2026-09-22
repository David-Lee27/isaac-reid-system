"""Live dashboard for the surveillance sim - per the README's own
"planned" tech stack (FastAPI + SQLite), now actually wired up.

Run alongside (or after) unified_tracking.py:

    C:\\isaacsim\\python.bat -m uvicorn dashboard:app --reload --port 8000

then open http://127.0.0.1:8000/ . Reads the SAME event_log.json the
simulation writes to - this process never touches the sim's own state,
it only reads the log it already produces, so it's safe to run at the
same time as a live sim with zero risk of interfering with it.

SQLite's job here is real, not decorative: event_log.json is a flat
JSON list (fine for the simulation's own append-only writer, see
append_log_entry() in unified_tracking.py), but the dashboard needs
filtering/pagination/sorting - exactly what a database is for instead
of re-slicing a Python list on every request. dashboard.db is
re-synced from event_log.json on every request (DELETE + re-INSERT,
not an incremental diff) - simplest possible approach that can never
drift out of sync, and cheap enough at this project's event-log scale
(hundreds of rows, not millions) that per-request cost is negligible.
"""
import json
import sqlite3
from datetime import datetime, timezone

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse

from report_lib import EVENT_LOG_FILE, load_events, build_report

DB_PATH = EVENT_LOG_FILE.replace("event_log.json", "dashboard.db")

app = FastAPI(title="Surveillance Dashboard")


def _sync_db():
    events = load_events()
    conn = sqlite3.connect(DB_PATH)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY,
            event_type TEXT,
            timestamp TEXT,
            person_id TEXT,
            zone TEXT,
            data TEXT
        )
    """)
    conn.execute("DELETE FROM events")
    conn.executemany(
        "INSERT INTO events (event_type, timestamp, person_id, zone, data) VALUES (?, ?, ?, ?, ?)",
        [(e.get("event_type"), e.get("timestamp"), e.get("person_id"), e.get("zone"), json.dumps(e))
         for e in events],
    )
    conn.commit()
    conn.close()
    return events


@app.get("/api/stats")
def api_stats():
    events = _sync_db()
    return JSONResponse(build_report(events))


@app.get("/api/events")
def api_events(limit: int = 50, event_type: str | None = None):
    _sync_db()
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    if event_type:
        rows = conn.execute(
            "SELECT data FROM events WHERE event_type = ? ORDER BY timestamp DESC LIMIT ?",
            (event_type, limit),
        ).fetchall()
    else:
        rows = conn.execute(
            "SELECT data FROM events ORDER BY timestamp DESC LIMIT ?", (limit,)
        ).fetchall()
    conn.close()
    return JSONResponse([json.loads(r["data"]) for r in rows])


@app.get("/", response_class=HTMLResponse)
def dashboard():
    events = _sync_db()
    report = build_report(events)
    recent = sorted(events, key=lambda e: e.get("timestamp") or "", reverse=True)[:25]

    fr, lr, ov, inj, intr = (report["fall_response"], report["loitering_response"],
                              report["overstay"], report["injury_log"], report["intruder_alerts"])

    def pct(n, d):
        return f"{100 * n / d:.0f}%" if d else "n/a"

    rows_html = "".join(
        f"<tr><td>{e.get('timestamp', '')[11:19]}</td><td>{e.get('event_type', '')}</td>"
        f"<td>{e.get('zone') or ''}</td><td>{e.get('person_id') or ''}</td></tr>"
        for e in recent
    )

    html = f"""<!DOCTYPE html>
<html><head>
<title>Surveillance Dashboard</title>
<meta http-equiv="refresh" content="5">
<style>
  body {{ font-family: system-ui, sans-serif; background: #0f1115; color: #e6e6e6; margin: 2rem; }}
  h1 {{ font-size: 1.3rem; }}
  .cards {{ display: flex; gap: 1rem; flex-wrap: wrap; margin: 1rem 0 2rem; }}
  .card {{ background: #1a1d24; border-radius: 8px; padding: 1rem 1.25rem; min-width: 200px; }}
  .card .n {{ font-size: 1.8rem; font-weight: 600; }}
  .card .l {{ color: #9aa0aa; font-size: 0.85rem; }}
  table {{ border-collapse: collapse; width: 100%; }}
  th, td {{ text-align: left; padding: 0.3rem 0.6rem; border-bottom: 1px solid #262a33; font-size: 0.9rem; }}
  th {{ color: #9aa0aa; font-weight: 500; }}
  .warn {{ color: #ff6b6b; }}
</style>
</head><body>
<h1>Surveillance Dashboard <span style="color:#9aa0aa;font-weight:normal;font-size:0.8rem">(auto-refreshes every 5s)</span></h1>
<div class="cards">
  <div class="card"><div class="n">{fr['assisted']}/{fr['dispatched']}</div><div class="l">Falls assisted ({pct(fr['assisted'], fr['dispatched'])})</div></div>
  <div class="card"><div class="n">{lr['identified']}/{lr['dispatched']}</div><div class="l">Loitering identified ({pct(lr['identified'], lr['dispatched'])})</div></div>
  <div class="card"><div class="n">{fr['mean_response_seconds'] and f"{fr['mean_response_seconds']:.1f}s" or 'n/a'}</div><div class="l">Mean fall response time</div></div>
  <div class="card"><div class="n {'warn' if intr['count'] else ''}">{intr['count']}</div><div class="l">Intruder alerts ({', '.join(intr['distinct_ids']) or 'none'})</div></div>
  <div class="card"><div class="n">{ov['escorted']}</div><div class="l">Overstay escorts</div></div>
</div>
<h2>Recent events</h2>
<table>
  <tr><th>Time</th><th>Type</th><th>Zone</th><th>Person</th></tr>
  {rows_html}
</table>
</body></html>"""
    return HTMLResponse(html)
