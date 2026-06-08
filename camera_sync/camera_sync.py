"""
camera_sync/camera_sync.py — Multi-camera frame synchronizer.

Strategy: nearest-frame matching with a staleness bound.
Same approach as ROS ApproximateTimeSynchronizer.

On each tick:
  1. Drain each camera queue — keep only the freshest frame
  2. Check all 4 frames are within STALENESS_BOUND_S of each other
  3. Emit SyncedFrameSet if yes, drop and log if no
"""

from __future__ import annotations
import queue
import threading
import time
from typing import Callable

from camera_sync.frame import CAMERA_NAMES, CameraFrame, SyncedFrameSet
from camera_sync.mock_camera import MockCamera

# ~1.5 frame periods at 30Hz — reasonable for USB cameras without a hardware sync trigger.
# With a GPIO trigger this could be tightened to ~5ms.
STALENESS_BOUND_S = 0.050

# Slightly faster than camera rate (30.03Hz) so the synchronizer reliably
# finds a fresh frame each tick without racing the camera threads.
SYNC_HZ = 30.5
SYNC_INTERVAL = 1.0 / SYNC_HZ


class CameraSynchronizer:
    """Aligns frames from 4 independent camera streams."""

    def __init__(
        self,
        cameras: dict[str, MockCamera],
        on_drop: Callable[[str], None] | None = None,
    ) -> None:
        missing = set(CAMERA_NAMES) - set(cameras.keys())
        if missing:
            raise ValueError(f"Missing cameras: {missing}")

        self._cameras = cameras
        self._on_drop = on_drop or (lambda msg: None)
        self.output_queue: queue.Queue[SyncedFrameSet] = queue.Queue(
            maxsize=int(SYNC_HZ * 2)
        )
        self._latest: dict[str, CameraFrame | None] = {n: None for n in CAMERA_NAMES}

        self._total_ticks = 0
        self._dropped_ticks = 0
        self._total_drained = 0

        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._sync_loop,
            daemon=True,
            name="CameraSynchronizer",
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    @property
    def drop_rate(self) -> float:
        if self._total_ticks == 0:
            return 0.0
        return self._dropped_ticks / self._total_ticks

    def _drain_camera(self, name: str) -> int:
        """Drain queue, keep only the freshest frame. Returns number discarded."""
        cam_queue = self._cameras[name].latest_queue
        latest = None
        discarded = 0
        while True:
            try:
                frame = cam_queue.get_nowait()
                if latest is not None:
                    discarded += 1
                latest = frame
            except queue.Empty:
                break
        if latest is not None:
            self._latest[name] = latest
        return discarded

    def _sync_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            self._total_ticks += 1
            tick_time = time.time()
            total_drained = sum(self._drain_camera(n) for n in CAMERA_NAMES)
            self._total_drained += total_drained

            missing = [n for n in CAMERA_NAMES if self._latest[n] is None]
            if missing:
                self._dropped_ticks += 1
                self._on_drop(f"[Sync] tick {self._total_ticks}: no frame yet from {missing}")
            else:
                timestamps = {n: self._latest[n].timestamp for n in CAMERA_NAMES}
                spread_s = max(timestamps.values()) - min(timestamps.values())

                if spread_s > STALENESS_BOUND_S:
                    self._dropped_ticks += 1
                    lagging = [n for n, t in timestamps.items() if t == min(timestamps.values())]
                    self._on_drop(
                        f"[Sync] tick {self._total_ticks}: spread {spread_s*1000:.1f}ms "
                        f"> bound {STALENESS_BOUND_S*1000:.0f}ms — lagging: {lagging}"
                    )
                else:
                    synced = SyncedFrameSet(
                        timestamp=tick_time,
                        frames={n: self._latest[n] for n in CAMERA_NAMES},
                        dropped_count=total_drained,
                    )
                    try:
                        self.output_queue.put_nowait(synced)
                    except queue.Full:
                        self._dropped_ticks += 1
                        self._on_drop(f"[Sync] tick {self._total_ticks}: output_queue full")

            next_tick += SYNC_INTERVAL
            remaining = next_tick - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)

    def stats_summary(self) -> str:
        return (
            f"Sync stats: ticks={self._total_ticks}  "
            f"dropped={self._dropped_ticks}  "
            f"drop_rate={self.drop_rate:.1%}  "
            f"frames_drained={self._total_drained}"
        )