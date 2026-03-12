"""
rerun_importer.py — Parses Rerun .rrd files and populates strategy_events in SQLite.

This importer focuses on strategy-related Text logs recorded under:
  - /tree/RoleManager (Current Role / Lead / Possession)
  - /tree/Decide (Decision + ballrange/ballyaw)

It uses the Rerun dataframe API (rerun.dataframe), which is available in
older SDKs (e.g., rerun-sdk==0.19.x). If that API is not available,
the importer prints guidance and exits gracefully.
"""

from __future__ import annotations

import os
import re
from datetime import datetime
from typing import Iterable, Optional

try:
    import rerun.dataframe as rdf  # type: ignore
except Exception:  # pragma: no cover - depends on installed SDK version
    rdf = None

from db.tcm_db import (
    get_connection,
    insert_match,
    insert_strategy_events_bulk,
)


# ─── Parsing patterns ───────────────────────────────────────────────────────

ROLE_RE = re.compile(
    r"Current Role:\s*(?P<role>[^,]+),\s*Lead:\s*(?P<lead>true|false),\s*Possession:\s*(?P<pos>true|false)",
    re.IGNORECASE,
)

DECISION_RE = re.compile(
    r"(?:LeadStrikerDecision|Decision):\s*(?P<decision>[^\s]+)"
    r"(?:\s+ballrange:\s*(?P<ballrange>[-+0-9.]+))?"
    r"(?:\s+ballyaw:\s*(?P<ballyaw>[-+0-9.]+))?",
    re.IGNORECASE,
)


def _has_component(schema, entity_path: str, component: str) -> bool:
    target = f"Component({entity_path}:{component})"
    return any(str(c) == target for c in schema.component_columns())


def _load_table(rec, entity_path: str, component: str = "Text"):
    view = rec.view(index="time", contents={entity_path: [component]})
    reader = view.select()
    return reader.read_all() if hasattr(reader, "read_all") else reader


def _iter_text_rows(table, text_col: str) -> Iterable[tuple[int, str]]:
    for batch in table.to_batches():
        times = batch.column("time").cast("int64")
        texts = batch.column(text_col)
        for i in range(batch.num_rows):
            t_ns = times[i].as_py()
            if t_ns is None:
                continue
            vals = texts[i].as_py()
            if not vals:
                continue
            if isinstance(vals, str):
                vals = [vals]
            for text in vals:
                if text:
                    yield int(t_ns), str(text)


def _parse_role(text: str) -> list[tuple[str, str]]:
    m = ROLE_RE.search(text)
    if not m:
        return []
    role = m.group("role").strip()
    lead = m.group("lead").lower()
    poss = m.group("pos").lower()
    return [
        ("Role", role),
        ("Lead", lead),
        ("Possession", poss),
    ]


def _parse_decision(text: str) -> list[tuple[str, str]]:
    m = DECISION_RE.search(text)
    if not m:
        return []
    decision = m.group("decision")
    ballrange = m.group("ballrange")
    ballyaw = m.group("ballyaw")
    events = [("Decision", decision)]
    if ballrange is not None:
        events.append(("BallRange", ballrange))
    if ballyaw is not None:
        events.append(("BallYaw", ballyaw))
    return events


def _ensure_rrd_match(path: str, t0_ns: int) -> int:
    filename = os.path.basename(path)
    con = get_connection()
    try:
        row = con.execute("SELECT id FROM matches WHERE filename=?", (filename,)).fetchone()
    finally:
        con.close()
    if row:
        return row["id"]

    date = datetime.utcfromtimestamp(t0_ns / 1e9).strftime("%Y-%m-%d %H:%M:%S")
    match_id = insert_match(
        filename=filename,
        date=date,
        opponent="Unknown",
        half="Unknown",
        state="rrd",
    )
    if match_id is not None:
        return match_id

    # Fallback: look it up again
    con = get_connection()
    try:
        row = con.execute("SELECT id FROM matches WHERE filename=?", (filename,)).fetchone()
        if row:
            return row["id"]
    finally:
        con.close()

    raise RuntimeError("Failed to create or retrieve match for RRD")


def import_rrd_file(path: str, match_id: Optional[int] = None):
    """
    Load .rrd file and extract strategy events from Text logs.
    If match_id is None, a synthetic match row is created using the RRD filename.
    """
    filename = os.path.basename(path)
    print(f"[rerun_importer] Processing {filename}...")

    if rdf is None:
        print("[rerun_importer] rerun.dataframe not available.\n"
              "Install an older SDK (e.g., rerun-sdk==0.19.x) and run with that Python.")
        return

    try:
        rec = rdf.load_recording(path)
    except Exception as e:
        print(f"[rerun_importer] Error loading {filename}: {e}")
        return

    schema = rec.schema()
    tables: list[tuple[str, str, callable]] = []  # (entity_path, text_col, parser)

    if _has_component(schema, "/tree/RoleManager", "Text"):
        tables.append(("/tree/RoleManager", "/tree/RoleManager:Text", _parse_role))
    if _has_component(schema, "/tree/Decide", "Text"):
        tables.append(("/tree/Decide", "/tree/Decide:Text", _parse_decision))

    if not tables:
        print(f"[rerun_importer] No matching Text entities found in {filename}.")
        return

    rows: list[tuple[int, str, callable]] = []
    for entity_path, text_col, parser in tables:
        table = _load_table(rec, entity_path)
        for t_ns, text in _iter_text_rows(table, text_col):
            rows.append((t_ns, text, parser))

    if not rows:
        print(f"[rerun_importer] No Text rows found in {filename}.")
        return

    t0_ns = min(t for t, _, _ in rows)
    rows.sort(key=lambda r: r[0])

    events_to_insert = []
    last_value: dict[str, str] = {}

    for t_ns, text, parser in rows:
        for event_type, value in parser(text):
            if value is None:
                continue
            if last_value.get(event_type) == value:
                continue
            last_value[event_type] = value
            t_sec = (t_ns - t0_ns) / 1e9
            events_to_insert.append((t_sec, "self", event_type, str(value)))

    if not events_to_insert:
        print(f"[rerun_importer] No strategy events parsed in {filename}.")
        return

    if match_id is None:
        match_id = _ensure_rrd_match(path, t0_ns)

    insert_strategy_events_bulk(match_id, events_to_insert)
    print(f"[rerun_importer] ✓ Synchronized {len(events_to_insert)} events into match_id={match_id}.")

if __name__ == "__main__":
    import sys
    # Example usage: python rerun_importer.py path/to/log.rrd [match_id]
    if len(sys.argv) >= 2:
        match_id = int(sys.argv[2]) if len(sys.argv) > 2 else None
        import_rrd_file(sys.argv[1], match_id)
    else:
        print("Usage: python rerun_importer.py <file.rrd> [match_id]")
