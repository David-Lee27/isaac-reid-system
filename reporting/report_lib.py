"""Shared event-log analysis backing dashboard.py's live stats.

Reads EVENT_LOG_FILE (the same JSON log unified_tracking.py's
append_log_entry() writes to, one flat list of dict events tagged by
"event_type") and turns it into the summary numbers a reviewer actually
cares about: how fast the robot responded to a fall, how often a search
actually landed a face, how many intruders got caught, and so on -
instead of just "it ran and printed things."

Event schema (as written by unified_tracking.py - see that file's
append_log_entry() call sites for the authoritative source):
  fall_alert        {zone, camera, timestamp}
  dispatch_alert    {reason: fall|loitering|overstay, person_id, zone, coords, timestamp}
  robot_response    {found, assisted?, person_id?, authorized?, banned?, alert_reason, zone, note?, timestamp}
  intruder_alert    {person_id, zone, coords?, timestamp}
  injury_incident   {person_id, mover_name, zone, timestamp}
  overstay_escort   {person_id, zone, coords, timestamp}
  loitering_alert   {zone, coords, timestamp}
  checkpoint        {person_id, checkpoint, authorized, banned, timestamp}
  zone              {zone, camera, timestamp} - a per-sweep heartbeat, not an alert

No alert directly carries the id of the dispatch_alert it resolves -
this project's event log is a flat append-only stream, not a relational
DB with foreign keys, and the robot only ever works one dispatch at a
time. Matching a dispatch to its outcome is done chronologically: the
first robot_response with the same alert_reason that comes after a given
dispatch_alert (and before the NEXT dispatch_alert of that same reason)
is treated as its resolution. That's a reasonable, honestly-labeled
approximation given the log's actual shape, not a rigorous join - see
_match_dispatches_to_responses()'s docstring.
"""
import json
import os
from datetime import datetime, timezone

# event_log.json is written by unified_tracking.py at the PROJECT ROOT, not
# next to this file - this now lives in reporting/, so go up one level.
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EVENT_LOG_FILE = os.path.join(PROJECT_ROOT, "event_log.json")


def load_events(path=EVENT_LOG_FILE):
    if not os.path.exists(path):
        return []
    with open(path, "r") as f:
        try:
            return json.load(f)
        except json.JSONDecodeError:
            return []


def _parse_ts(e):
    ts = e.get("timestamp")
    if not ts:
        return None
    try:
        return datetime.fromisoformat(ts.replace("Z", "+00:00"))
    except ValueError:
        return None


def _match_dispatches_to_responses(events, reason):
    """Pairs each dispatch_alert(reason=reason) with whichever
    robot_response(alert_reason=reason) resolves it. Returns a list of
    (dispatch, response or None) tuples, response is None if the dispatch
    was never resolved within the log (robot never got to it, or the run
    ended first).

    Matches by each alert's own unique alert_id (see unified_tracking.py's
    next_alert_id) when both sides have one - a direct, exact join, not a
    guess. BUG (found by hand-checking against the kit log's own actual
    search outcomes - the real success rate was 83%, an earlier timestamp-
    windowed version of this function reported 56%): pairing by "the next
    response before the next same-reason dispatch" breaks once dispatches
    queue up faster than the robot resolves them (loitering keeps firing
    from OTHER zones while a multi-second orbit search for an EARLIER one
    is still running - the normal case with two people wandering, not an
    edge case) - a response finishing after several more same-reason
    dispatches had piled up fell outside every window and scored as a
    miss, even though the robot really did resolve it. Falls back to that
    same oldest-still-open heuristic only for legacy log entries that
    predate alert_id."""
    dispatches = sorted((e for e in events if e.get("event_type") == "dispatch_alert" and e.get("reason") == reason),
                        key=lambda e: _parse_ts(e) or datetime.min.replace(tzinfo=timezone.utc))
    responses = sorted((e for e in events if e.get("event_type") == "robot_response" and e.get("alert_reason") == reason),
                       key=lambda e: _parse_ts(e) or datetime.min.replace(tzinfo=timezone.utc))
    pairs = [(d, None) for d in dispatches]
    open_indices = list(range(len(dispatches)))
    unmatched_responses = []
    responses_by_id = {}
    for r in responses:
        rid = r.get("alert_id")
        if rid is not None:
            responses_by_id.setdefault(rid, []).append(r)
        else:
            unmatched_responses.append(r)
    for i, d in enumerate(dispatches):
        did = d.get("alert_id")
        if did is not None and responses_by_id.get(did):
            pairs[i] = (d, responses_by_id[did].pop(0))
            open_indices.remove(i)
    # Legacy fallback (no alert_id on one side or the other): oldest-still-open.
    for r in unmatched_responses + [r for rs in responses_by_id.values() for r in rs]:
        r_ts = _parse_ts(r)
        candidate = next((i for i in open_indices if (_parse_ts(dispatches[i]) or r_ts) <= (r_ts or datetime.max.replace(tzinfo=timezone.utc))), None)
        if candidate is None:
            continue
        open_indices.remove(candidate)
        pairs[candidate] = (dispatches[candidate], r)
    return pairs


def _response_seconds(pairs):
    deltas = []
    for d, r in pairs:
        if r is None:
            continue
        d_ts, r_ts = _parse_ts(d), _parse_ts(r)
        if d_ts and r_ts:
            deltas.append((r_ts - d_ts).total_seconds())
    return deltas


def _median(values):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2


def build_report(events=None):
    """Returns a plain-dict summary - JSON-serializable as-is, and what
    dashboard.py's own live stats cards are built from."""
    if events is None:
        events = load_events()

    timestamps = [t for t in (_parse_ts(e) for e in events) if t is not None]
    run_start = min(timestamps) if timestamps else None
    run_end = max(timestamps) if timestamps else None
    duration_s = (run_end - run_start).total_seconds() if run_start and run_end else None

    counts = {}
    for e in events:
        et = e.get("event_type", "unknown")
        counts[et] = counts.get(et, 0) + 1

    fall_pairs = _match_dispatches_to_responses(events, "fall")
    fall_assisted = sum(1 for d, r in fall_pairs if r is not None and r.get("assisted"))
    fall_missed = len(fall_pairs) - fall_assisted
    fall_response_times = _response_seconds([(d, r) for d, r in fall_pairs if r is not None and r.get("assisted")])

    loiter_pairs = _match_dispatches_to_responses(events, "loitering")
    loiter_found = sum(1 for d, r in loiter_pairs if r is not None and r.get("found"))
    loiter_missed = len(loiter_pairs) - loiter_found
    loiter_response_times = _response_seconds([(d, r) for d, r in loiter_pairs if r is not None and r.get("found")])

    overstay_pairs = _match_dispatches_to_responses(events, "overstay")
    overstay_escorted = sum(1 for e in events if e.get("event_type") == "overstay_escort")

    injury_events = [e for e in events if e.get("event_type") == "injury_incident"]
    injury_identified = sum(1 for e in injury_events if e.get("person_id"))

    intruder_events = [e for e in events if e.get("event_type") == "intruder_alert"]
    intruder_ids = sorted({e.get("person_id") for e in intruder_events if e.get("person_id")})

    return {
        "run_start": run_start.isoformat() if run_start else None,
        "run_end": run_end.isoformat() if run_end else None,
        "duration_seconds": duration_s,
        "event_counts": counts,
        "total_events": len(events),
        "fall_response": {
            "dispatched": len(fall_pairs),
            "assisted": fall_assisted,
            "missed": fall_missed,
            "mean_response_seconds": (sum(fall_response_times) / len(fall_response_times)
                                       if fall_response_times else None),
            "median_response_seconds": _median(fall_response_times),
        },
        "loitering_response": {
            "dispatched": len(loiter_pairs),
            "identified": loiter_found,
            "missed": loiter_missed,
            "mean_response_seconds": (sum(loiter_response_times) / len(loiter_response_times)
                                       if loiter_response_times else None),
            "median_response_seconds": _median(loiter_response_times),
        },
        "overstay": {
            "dispatched": len(overstay_pairs),
            "escorted": overstay_escorted,
        },
        "injury_log": {
            "incidents": len(injury_events),
            "identified": injury_identified,
        },
        "intruder_alerts": {
            "count": len(intruder_events),
            "distinct_ids": intruder_ids,
        },
    }


def recent_events(events=None, limit=50):
    if events is None:
        events = load_events()
    keyed = sorted(events, key=lambda e: e.get("timestamp") or "")
    return list(reversed(keyed[-limit:]))
