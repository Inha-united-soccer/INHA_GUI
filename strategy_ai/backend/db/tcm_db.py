"""
tcm_db.py — SQLite schema + CRUD for TCM log data.

Tables:
  matches          — 1 row per .log file session
  robot_snapshots  — each RGrt packet parsed from the log
  match_summaries  — per-match aggregates (avg positions, formation, etc.)
  strategy_events  — Rerun intent data (Role, Decision, etc.) synced via time
"""

import sqlite3
import os
import threading
from typing import Optional

DB_PATH = os.path.join(os.path.dirname(__file__), "game_db.sqlite")


def get_connection() -> sqlite3.Connection:
    """Return a thread-safe connection with row_factory."""
    con = sqlite3.connect(DB_PATH, check_same_thread=False)
    con.row_factory = sqlite3.Row
    return con


def init_db():
    """Create tables if they don't exist."""
    con = get_connection()
    with con:
        con.executescript("""
        CREATE TABLE IF NOT EXISTS matches (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            filename    TEXT    UNIQUE NOT NULL,
            date        TEXT,           -- '2026-03-06 14:20:30'
            opponent    TEXT,           -- 'Northern Bites'
            half        TEXT,           -- '1stHalf'
            state       TEXT,           -- 'initial' or 'playing'
            imported_at TEXT DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS robot_snapshots (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            match_id     INTEGER NOT NULL REFERENCES matches(id) ON DELETE CASCADE,
            team_num     INTEGER,
            player_num   INTEGER,
            x            REAL,    -- mm
            y            REAL,    -- mm
            theta        REAL,    -- radians
            fallen       INTEGER, -- 0 or 1
            ball_x       REAL,
            ball_y       REAL,
            ball_age     REAL,
            packet_index INTEGER  -- sequential index within the file
        );

        CREATE INDEX IF NOT EXISTS idx_snapshots_match ON robot_snapshots(match_id);
        CREATE INDEX IF NOT EXISTS idx_snapshots_team  ON robot_snapshots(team_num);

        CREATE TABLE IF NOT EXISTS match_summaries (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            match_id        INTEGER UNIQUE NOT NULL REFERENCES matches(id) ON DELETE CASCADE,
            our_team_num    INTEGER,
            our_robot_count INTEGER,
            our_avg_x       REAL,
            our_avg_y       REAL,
            our_fallen_pct  REAL,   -- 0.0-1.0
            ball_avg_x      REAL,
            ball_avg_y      REAL,
            total_packets   INTEGER,
            formation       TEXT    -- '공격 집중' | '수비 집중' | '균형'
        );

        CREATE TABLE IF NOT EXISTS strategy_events (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            match_id     INTEGER NOT NULL REFERENCES matches(id) ON DELETE CASCADE,
            game_time    REAL,    -- secsRemaining from GC
            robot_id     TEXT,    -- 'robot_1', etc.
            event_type   TEXT,    -- 'Role', 'Possession', 'Decision', 'BallYaw', etc.
            value        TEXT,
            timestamp    TEXT DEFAULT (datetime('now'))
        );

        CREATE INDEX IF NOT EXISTS idx_events_match ON strategy_events(match_id);
        CREATE INDEX IF NOT EXISTS idx_events_time  ON strategy_events(game_time);
        """)
    con.close()


# ─── Write helpers ──────────────────────────────────────────────────────────

def insert_match(filename: str, date: str, opponent: str,
                 half: str, state: str) -> Optional[int]:
    """Insert a match record. Returns new id, or None if already exists."""
    con = get_connection()
    try:
        with con:
            cur = con.execute(
                "INSERT OR IGNORE INTO matches(filename,date,opponent,half,state) "
                "VALUES(?,?,?,?,?)",
                (filename, date, opponent, half, state)
            )
            if cur.lastrowid == 0:
                row = con.execute(
                    "SELECT id FROM matches WHERE filename=?", (filename,)
                ).fetchone()
                return row["id"] if row else None
            return cur.lastrowid
    finally:
        con.close()


def insert_snapshots_bulk(match_id: int, rows: list):
    """
    Bulk-insert robot snapshot rows.
    Each row: (team_num, player_num, x, y, theta, fallen, ball_x, ball_y, ball_age, packet_idx)
    """
    con = get_connection()
    try:
        with con:
            con.executemany(
                "INSERT INTO robot_snapshots"
                "(match_id,team_num,player_num,x,y,theta,fallen,ball_x,ball_y,ball_age,packet_index)"
                " VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                [(match_id, *r) for r in rows]
            )
    finally:
        con.close()


def insert_or_replace_summary(match_id: int, summary: dict):
    """Upsert the match_summaries row for a match."""
    con = get_connection()
    try:
        with con:
            con.execute("""
            INSERT OR REPLACE INTO match_summaries
              (match_id, our_team_num, our_robot_count, our_avg_x, our_avg_y,
               our_fallen_pct, ball_avg_x, ball_avg_y, total_packets, formation)
            VALUES
              (:match_id,:our_team_num,:our_robot_count,:our_avg_x,:our_avg_y,
               :our_fallen_pct,:ball_avg_x,:ball_avg_y,:total_packets,:formation)
            """, {"match_id": match_id, **summary})
    finally:
        con.close()


def insert_strategy_event(match_id: int, game_time: float, robot_id: str,
                          event_type: str, value: str):
    """Insert a single strategy/intent event."""
    con = get_connection()
    try:
        with con:
            con.execute(
                "INSERT INTO strategy_events(match_id, game_time, robot_id, event_type, value) "
                "VALUES(?,?,?,?,?)",
                (match_id, game_time, robot_id, event_type, value)
            )
    finally:
        con.close()


def insert_strategy_events_bulk(match_id: int, rows: list):
    """Bulk insert strategy events. Row: (game_time, robot_id, event_type, value)"""
    con = get_connection()
    try:
        with con:
            con.executemany(
                "INSERT INTO strategy_events(match_id, game_time, robot_id, event_type, value) "
                "VALUES(?,?,?,?,?)",
                [(match_id, *r) for r in rows]
            )
    finally:
        con.close()


# ─── Read helpers ────────────────────────────────────────────────────────────

def get_db_status() -> dict:
    """High-level stats for the /api/db/status endpoint."""
    con = get_connection()
    try:
        matches   = con.execute("SELECT COUNT(*) FROM matches").fetchone()[0]
        snapshots = con.execute("SELECT COUNT(*) FROM robot_snapshots").fetchone()[0]
        summaries = con.execute("SELECT COUNT(*) FROM match_summaries").fetchone()[0]
        recent = [dict(r) for r in con.execute(
            "SELECT filename, date, opponent, half, state FROM matches "
            "ORDER BY imported_at DESC LIMIT 5"
        ).fetchall()]
        return {
            "total_matches":   matches,
            "total_snapshots": snapshots,
            "total_summaries": summaries,
            "recent_matches":  recent,
        }
    finally:
        con.close()


def is_already_imported(filename: str) -> bool:
    con = get_connection()
    try:
        row = con.execute(
            "SELECT 1 FROM matches WHERE filename=?", (filename,)
        ).fetchone()
        return row is not None
    finally:
        con.close()


def get_similar_situations(our_avg_x: float, score_diff: int = 0,
                            limit: int = 5) -> list[dict]:
    """
    Query match_summaries that resemble the current live situation.

    Parameters:
        our_avg_x  — current average X of our robots (mm). Positive = attacking half.
        score_diff — current score difference (our - opponent). Negative = losing.
        limit      — max rows to return.

    Returns list of dicts with opponent, formation, our_avg_x, ball_avg_x.
    """
    con = get_connection()
    try:
        rows = con.execute("""
            SELECT m.id, m.opponent, m.half, ms.formation,
                   ROUND(ms.our_avg_x,2)  AS our_avg_x,
                   ROUND(ms.ball_avg_x,2) AS ball_avg_x,
                   ROUND(ms.our_fallen_pct*100,1) AS fallen_pct,
                   ms.our_robot_count,
                   ms.total_packets
            FROM   match_summaries ms
            JOIN   matches m ON ms.match_id = m.id
            WHERE  ABS(ms.our_avg_x - ?) < 1.5
            ORDER  BY m.date DESC
            LIMIT  ?
        """, (our_avg_x, limit)).fetchall()
        return [dict(r) for r in rows]
    finally:
        con.close()


def get_match_events(match_id: int) -> list[dict]:
    """Fetch all strategy events for a match, ordered by game time."""
    con = get_connection()
    try:
        rows = con.execute("""
            SELECT game_time, robot_id, event_type, value
            FROM   strategy_events
            WHERE  match_id = ?
            ORDER  BY game_time DESC
        """, (match_id,)).fetchall()
        return [dict(r) for r in rows]
    finally:
        con.close()


# Initialise on import
init_db()
