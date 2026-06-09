"""
api/database.py — SQLite index for episode metadata.

SQLite chosen over Postgres/MySQL: no setup required, sufficient for
a single-node data collection system. The parquet files hold actual data;
this just holds the searchable index.
"""

import sqlite3
from pathlib import Path

DB_PATH = Path("data/episodes.db")


def get_conn() -> sqlite3.Connection:
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row  # rows accessible as dicts
    return conn


def init_db() -> None:
    with get_conn() as conn:
        conn.execute("""
            CREATE TABLE IF NOT EXISTS episodes (
                id              INTEGER PRIMARY KEY,
                start_time      REAL,
                end_time        REAL,
                duration_s      REAL,
                joint_states    INTEGER,
                frames_wrist_left   INTEGER,
                frames_wrist_right  INTEGER,
                frames_ceiling      INTEGER,
                frames_head         INTEGER,
                success         INTEGER DEFAULT 0,
                path            TEXT
            )
        """)
        conn.commit()


def insert_episode(meta: dict) -> None:
    fc = meta["frame_counts"]
    with get_conn() as conn:
        conn.execute("""
            INSERT INTO episodes
                (id, start_time, end_time, duration_s, joint_states,
                 frames_wrist_left, frames_wrist_right, frames_ceiling, frames_head,
                 success, path)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            meta["id"],
            meta["start_time"],
            meta["end_time"],
            meta["duration_s"],
            meta["joint_state_count"],
            fc.get("wrist_left", 0),
            fc.get("wrist_right", 0),
            fc.get("ceiling", 0),
            fc.get("head", 0),
            0,
            meta["path"],
        ))
        conn.commit()


def get_all_episodes() -> list[dict]:
    with get_conn() as conn:
        rows = conn.execute("SELECT * FROM episodes ORDER BY id").fetchall()
        return [dict(r) for r in rows]


def get_episode(episode_id: int) -> dict | None:
    with get_conn() as conn:
        row = conn.execute(
            "SELECT * FROM episodes WHERE id = ?", (episode_id,)
        ).fetchone()
        return dict(row) if row else None
