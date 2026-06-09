"""
api/app.py — REST API for the OpenArm data collection pipeline.

Endpoints:
    GET  /status                  → recording status
    POST /episodes/start          → start recording
    POST /episodes/stop           → stop recording + write to disk
    GET  /episodes                → list all episodes
    GET  /episodes/{id}           → single episode metadata
    GET  /episodes/{id}/download  → streaming .tar.gz download
"""

import io
import tarfile
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse

from api.database import get_all_episodes, get_episode, init_db, insert_episode
from api.models import EpisodeSummary, FrameCounts, RecordingStatus

app = FastAPI(title="OpenArm Data Collection API")

# Module-level recorder instance — set by main.py before uvicorn starts.
# Using a mutable container so the reference survives uvicorn's module import.
_state = {"recorder": None}


def set_recorder(recorder) -> None:
    """Called by main.py to inject the recorder before the server starts."""
    _state["recorder"] = recorder


def _get_recorder():
    r = _state["recorder"]
    if r is None:
        raise HTTPException(503, "Recorder not initialised")
    return r


@app.on_event("startup")
def startup():
    init_db()


# ── Status ────────────────────────────────────────────────────────────────────

@app.get("/status", response_model=RecordingStatus)
def get_status():
    r = _state["recorder"]
    return RecordingStatus(
        recording=r.is_recording if r else False,
        episode_id=r.current_episode_id if r else None,
    )


# ── Recording controls ────────────────────────────────────────────────────────

@app.post("/episodes/start", response_model=RecordingStatus)
def start_recording():
    r = _get_recorder()
    try:
        episode_id = r.start_recording()
        return RecordingStatus(recording=True, episode_id=episode_id)
    except RuntimeError as e:
        raise HTTPException(409, str(e))


@app.post("/episodes/stop", response_model=EpisodeSummary)
def stop_recording():
    r = _get_recorder()
    try:
        meta = r.stop_recording()
    except RuntimeError as e:
        raise HTTPException(409, str(e))

    insert_episode(meta)
    fc = meta["frame_counts"]
    return EpisodeSummary(
        id=meta["id"],
        start_time=meta["start_time"],
        end_time=meta["end_time"],
        duration_s=meta["duration_s"],
        joint_states=meta["joint_state_count"],
        frame_counts=FrameCounts(
            wrist_left=fc["wrist_left"],
            wrist_right=fc["wrist_right"],
            ceiling=fc["ceiling"],
            head=fc["head"],
        ),
        success=False,
    )


# ── Episode listing ───────────────────────────────────────────────────────────

@app.get("/episodes", response_model=list[EpisodeSummary])
def list_episodes():
    return [_row_to_summary(r) for r in get_all_episodes()]


@app.get("/episodes/{episode_id}", response_model=EpisodeSummary)
def get_episode_metadata(episode_id: int):
    row = get_episode(episode_id)
    if row is None:
        raise HTTPException(404, f"Episode {episode_id} not found")
    return _row_to_summary(row)


# ── Download ──────────────────────────────────────────────────────────────────

@app.get("/episodes/{episode_id}/download")
def download_episode(episode_id: int):
    """Stream episode as .tar.gz — avoids loading large episodes into memory."""
    row = get_episode(episode_id)
    if row is None:
        raise HTTPException(404, f"Episode {episode_id} not found")

    episode_path = Path(row["path"])
    if not episode_path.exists():
        raise HTTPException(404, f"Episode {episode_id} files not found on disk")

    def tar_generator():
        buf = io.BytesIO()
        with tarfile.open(fileobj=buf, mode="w:gz") as tar:
            tar.add(episode_path, arcname=f"episode_{episode_id}")
        buf.seek(0)
        while chunk := buf.read(65536):
            yield chunk

    return StreamingResponse(
        tar_generator(),
        media_type="application/gzip",
        headers={"Content-Disposition": f"attachment; filename=episode_{episode_id}.tar.gz"},
    )


# ── Helper ────────────────────────────────────────────────────────────────────

def _row_to_summary(row: dict) -> EpisodeSummary:
    return EpisodeSummary(
        id=row["id"],
        start_time=row["start_time"],
        end_time=row["end_time"],
        duration_s=row["duration_s"],
        joint_states=row["joint_states"],
        frame_counts=FrameCounts(
            wrist_left=row["frames_wrist_left"],
            wrist_right=row["frames_wrist_right"],
            ceiling=row["frames_ceiling"],
            head=row["frames_head"],
        ),
        success=bool(row["success"]),
    )