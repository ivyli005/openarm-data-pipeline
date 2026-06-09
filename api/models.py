"""
api/models.py — Pydantic response models for the REST API.
"""

from pydantic import BaseModel


class FrameCounts(BaseModel):
    wrist_left: int
    wrist_right: int
    ceiling: int
    head: int


class EpisodeSummary(BaseModel):
    id: int
    start_time: float
    end_time: float
    duration_s: float
    joint_states: int
    frame_counts: FrameCounts
    success: bool


class RecordingStatus(BaseModel):
    recording: bool
    episode_id: int | None
