"""
camera_sync/frame.py — CameraFrame and SyncedFrameSet dataclasses.
"""

from __future__ import annotations
import time
from dataclasses import dataclass, field
import numpy as np

# From Dataset API: Frame.load() → (600, 960, 3) uint8 RGB
FRAME_HEIGHT = 600
FRAME_WIDTH = 960
FRAME_CHANNELS = 3

# From Dataset API: ds.camera_names → ['wrist_left', 'wrist_right', 'ceiling', 'head']
# 'head' is the ZED stereo camera — one stream, SDK handles stereo internally
CAMERA_NAMES = ["wrist_left", "wrist_right", "ceiling", "head"]


@dataclass
class CameraFrame:
    """One captured frame from one camera."""
    camera_name: str
    timestamp_ns: int   # time.time_ns() at capture — used as {ns}.jpeg filename
    timestamp: float    # timestamp_ns / 1e9, matches Dataset API Frame.timestamp
    frame_index: int
    frame: np.ndarray   # (600, 960, 3) uint8 RGB

    @classmethod
    def create(
        cls,
        camera_name: str,
        frame_index: int,
        frame: np.ndarray,
        timestamp_ns: int | None = None,
    ) -> "CameraFrame":
        # Always derive timestamp from timestamp_ns — never call both time.time_ns()
        # and time.time() separately or they'll be microseconds apart
        ts_ns = timestamp_ns if timestamp_ns is not None else time.time_ns()
        return cls(
            camera_name=camera_name,
            timestamp_ns=ts_ns,
            timestamp=ts_ns / 1e9,
            frame_index=frame_index,
            frame=frame,
        )


@dataclass
class SyncedFrameSet:
    """One aligned snapshot across all 4 cameras."""
    timestamp: float                      # wall-clock reference for storage layer
    frames: dict[str, CameraFrame] = field(default_factory=dict)
    max_jitter_ms: float = 0.0            # spread between earliest and latest frame (ms)
    dropped_count: int = 0                # frames discarded while draining queues

    def __post_init__(self) -> None:
        if self.frames:
            timestamps = [f.timestamp for f in self.frames.values()]
            self.max_jitter_ms = (max(timestamps) - min(timestamps)) * 1000.0

    def is_complete(self) -> bool:
        return set(self.frames.keys()) == set(CAMERA_NAMES)

    def summary(self) -> str:
        lines = [
            f"SyncedFrameSet @ t={self.timestamp:.6f}  "
            f"jitter={self.max_jitter_ms:.1f}ms  "
            f"dropped={self.dropped_count}"
        ]
        for name in CAMERA_NAMES:
            if name in self.frames:
                f = self.frames[name]
                lines.append(
                    f"  {name:<14} frame={f.frame_index:05d}  t={f.timestamp:.6f}"
                )
            else:
                lines.append(f"  {name:<14} MISSING")
        return "\n".join(lines)