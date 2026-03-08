"""
tcm_importer.py — Parses TCM binary .log files and populates the SQLite DB.

TCM log format (Java serialized object stream):
  - Embedded 32-byte RGrt packets: RoboCupGameControlReturnData
  - Header b'RGrt' at various offsets

Packet layout (32 bytes):
  [0:4]  header  = b'RGrt'
  [4]    version (uint8) — expected 2 or 4
  [5]    player_num (uint8)
  [6]    team_num   (uint8)
  [7]    fallen     (uint8)
  [8:20] x, y, theta  (3x little-endian float32)
  [20:24] ball_age    (float32)
  [24:32] ball_x, ball_y (2x float32)
"""

import os
import re
import struct
import glob
import math
from typing import Optional
from datetime import datetime

from db.tcm_db import (
    insert_match,
    insert_snapshots_bulk,
    insert_or_replace_summary,
    is_already_imported,
)

OUR_TEAM_NUM = 13
RGRT_HEADER  = b'RGrt'
PACKET_SIZE  = 32


def parse_filename(path: str) -> dict:
    """
    Extract match metadata from log filename.
    Format: teamcomm_DATE_TIME[-MS]_TeamA_TeamB_Half[_state].log
    """
    name = os.path.basename(path).replace(".log", "")
    parts = name.split("_")
    result = {
        "date": "",
        "opponent": "Unknown",
        "half": "Unknown",
        "state": "playing",
    }
    try:
        # Date + time: parts[1] = YYYY-MM-DD, parts[2] = HH-MM-SS[-ms]
        date_str = parts[1]  # '2026-03-06'
        time_str = parts[2].split("-")[:3]  # ['14','20','30']
        result["date"] = f"{date_str} {':'.join(time_str)}"

        # Find half indicator (1stHalf / 2ndHalf)
        half_idx = next(
            (i for i, p in enumerate(parts) if "Half" in p or "half" in p), None
        )
        if half_idx and half_idx > 3:
            # Teams sit between index 3 and half_idx
            # Note: team names with spaces (e.g. "INHA United") appear as
            # single underscore-split tokens since filenames use "_" as separator
            # between groups, but spaces within team names are preserved.
            team_parts = parts[3:half_idx]

            # Identify our team tokens (those containing 'INHA')
            opp_tokens = [p for p in team_parts if "INHA" not in p]

            if any("INHA" in p for p in team_parts):
                result["opponent"] = " ".join(opp_tokens).strip() or "Unknown"
            else:
                # We're not in the filename — whole list is the opponent(s)
                result["opponent"] = " ".join(team_parts).strip() or "Unknown"

            result["half"] = parts[half_idx]

        if len(parts) > (half_idx or 0) + 1:
            result["state"] = parts[-1]

    except Exception:
        pass

    return result


def _classify_formation(avg_x: float) -> str:
    # Positions are in meters. Middle field ~±4.5m.
    if avg_x > 1.5:
        return "공격 집중"
    elif avg_x < -1.5:
        return "수비 집중"
    return "균형"


def import_log_file(path: str, force: bool = False) -> Optional[str]:
    """
    Parse a single .log file and insert data into SQLite.

    Returns:
        'imported' — successfully imported
        'skipped'  — already in DB and force=False
        'empty'    — no valid RGrt packets found
        'error'    — exception during parsing
    """
    filename = os.path.basename(path)

    if not force and is_already_imported(filename):
        return "skipped"

    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        print(f"[importer] Cannot read {filename}: {e}")
        return "error"

    # Find all RGrt packet offsets
    offsets = [m.start() for m in re.finditer(re.escape(RGRT_HEADER), data)]
    if not offsets:
        return "empty"

    # Parse metadata from filename
    meta     = parse_filename(path)
    match_id = insert_match(
        filename=filename,
        date=meta["date"],
        opponent=meta["opponent"],
        half=meta["half"],
        state=meta["state"],
    )
    if match_id is None:
        print(f"[importer] Could not insert match for {filename}")
        return "error"

    # Parse RGrt packets
    snapshot_rows = []
    for idx, offset in enumerate(offsets):
        if offset + PACKET_SIZE > len(data):
            continue
        chunk = data[offset: offset + PACKET_SIZE]
        try:
            version, player_num, team_num, fallen = struct.unpack("BBBB", chunk[4:8])
            x, y, theta = struct.unpack("<fff", chunk[8:20])
            ball_age    = struct.unpack("<f",   chunk[20:24])[0]
            ball_x, ball_y = struct.unpack("<ff", chunk[24:32])

            # Skip NaN / junk packets
            if any(math.isnan(v) or math.isinf(v) for v in [x, y, theta, ball_x, ball_y]):
                continue

            snapshot_rows.append((
                team_num, player_num,
                round(x, 1), round(y, 1), round(theta, 4),
                int(fallen != 0),
                round(ball_x, 1), round(ball_y, 1), round(ball_age, 3),
                idx,
            ))
        except struct.error:
            continue

    if not snapshot_rows:
        return "empty"

    # Bulk insert snapshots
    insert_snapshots_bulk(match_id, snapshot_rows)

    # Compute per-match summary from our team packets
    our_pkts = [r for r in snapshot_rows if r[0] == OUR_TEAM_NUM]
    if our_pkts:
        xs   = [r[2] for r in our_pkts]
        ys   = [r[3] for r in our_pkts]
        bxs  = [r[6] for r in our_pkts]
        bys  = [r[7] for r in our_pkts]
        fall = [r[5] for r in our_pkts]

        our_avg_x  = sum(xs)  / len(xs)
        our_avg_y  = sum(ys)  / len(ys)
        ball_avg_x = sum(bxs) / len(bxs)
        ball_avg_y = sum(bys) / len(bys)
        fallen_pct = sum(fall) / len(fall)
        robots     = len({r[1] for r in our_pkts})  # unique player_nums

        insert_or_replace_summary(match_id, {
            "our_team_num":    OUR_TEAM_NUM,
            "our_robot_count": robots,
            "our_avg_x":       round(our_avg_x, 1),
            "our_avg_y":       round(our_avg_y, 1),
            "our_fallen_pct":  round(fallen_pct, 4),
            "ball_avg_x":      round(ball_avg_x, 1),
            "ball_avg_y":      round(ball_avg_y, 1),
            "total_packets":   len(snapshot_rows),
            "formation":       _classify_formation(our_avg_x),
        })

    print(f"[importer] ✓ {filename}  packets={len(snapshot_rows)}  our={len(our_pkts)}")
    return "imported"


def import_all(log_dir: str, force: bool = False) -> dict:
    """
    Import all .log files in log_dir.
    Returns summary dict: {imported, skipped, empty, error}
    """
    files = sorted(glob.glob(os.path.join(log_dir, "*.log")))
    counts = {"imported": 0, "skipped": 0, "empty": 0, "error": 0}

    for path in files:
        result = import_log_file(path, force=force)
        counts[result] = counts.get(result, 0) + 1

    print(f"[importer] Done — {counts}")
    return counts


# ── CLI entry ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    LOG_DIR = os.path.abspath(
        os.path.join(os.path.dirname(__file__),
                     "../../../3D modeling/bin/logs_teamcomm")
    )
    force = "--force" in sys.argv
    print(f"[importer] Importing from: {LOG_DIR}")
    result = import_all(LOG_DIR, force=force)
    print(f"[importer] Result: {result}")
