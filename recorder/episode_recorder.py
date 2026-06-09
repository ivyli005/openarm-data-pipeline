"""
recorder/episode_recorder.py — Buffers joint states and streams camera
frames to disk during recording. Camera frames are written immediately
on arrival to avoid buffering ~1GB of raw numpy arrays in memory.
"""

import queue
import threading
import time
from collections import defaultdict
from pathlib import Path

from can_reader.joint_state import JointState
from camera_sync.frame import CAMERA_NAMES, CameraFrame, SyncedFrameSet
from recorder.writer import write_jpeg_frame, write_obs_parquet, write_action_parquet, write_metadata_yaml


class EpisodeRecorder:
    """
    Consumes from CAN and camera queues when recording is active.
    Camera frames are written to disk immediately (not buffered).
    Joint states are buffered in memory (small — float32 arrays only).
    """

    def __init__(
        self,
        can_queue: queue.Queue,
        camera_queue: queue.Queue,
        dataset_path: Path,
    ) -> None:
        self._can_queue = can_queue
        self._camera_queue = camera_queue
        self._dataset_path = dataset_path

        self._recording = False
        self._lock = threading.Lock()

        self._joint_states: list[JointState] = []
        self._frame_counts: dict[str, int] = defaultdict(int)
        self._episode_path: Path | None = None
        self._start_time: float | None = None
        self._episode_id: int = 0

        self._stop_event = threading.Event()
        self._can_thread = threading.Thread(
            target=self._consume_can, daemon=True, name="Recorder-CAN"
        )
        self._cam_thread = threading.Thread(
            target=self._consume_camera, daemon=True, name="Recorder-Camera"
        )

    def start_threads(self) -> None:
        self._can_thread.start()
        self._cam_thread.start()

    def stop_threads(self) -> None:
        self._stop_event.set()

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def current_episode_id(self) -> int | None:
        return self._episode_id if self._recording else None

    def start_recording(self) -> int:
        with self._lock:
            if self._recording:
                raise RuntimeError("Already recording")

            # Create episode directory structure upfront
            episode_path = self._dataset_path / "episodes" / str(self._episode_id)
            episode_path.mkdir(parents=True, exist_ok=True)
            for name in CAMERA_NAMES:
                (episode_path / "cameras" / name).mkdir(parents=True, exist_ok=True)

            self._episode_path = episode_path
            self._joint_states = []
            self._frame_counts = defaultdict(int)
            self._start_time = time.time()
            self._recording = True
            print(f"[Recorder] Started episode {self._episode_id}")
            return self._episode_id

    def stop_recording(self) -> dict:
        with self._lock:
            if not self._recording:
                raise RuntimeError("Not recording")
            self._recording = False
            episode_path = self._episode_path
            joint_states = self._joint_states[:]
            frame_counts = dict(self._frame_counts)

        # Write parquet files (joint states only — frames already on disk)
        print(f"[Recorder] Writing parquet for episode {self._episode_id}...")
        write_obs_parquet(joint_states, episode_path)
        write_action_parquet(joint_states, episode_path)

        end_time = time.time()
        write_metadata_yaml(self._episode_id, self._start_time, end_time, self._dataset_path)

        meta = {
            "id": self._episode_id,
            "path": str(episode_path),
            "start_time": self._start_time,
            "end_time": end_time,
            "duration_s": round(end_time - self._start_time, 2),
            "joint_state_count": len(joint_states),
            "frame_counts": {name: frame_counts.get(name, 0) for name in CAMERA_NAMES},
        }

        print(
            f"[Recorder] Episode {self._episode_id} done — "
            f"{meta['joint_state_count']} joint states, "
            f"{meta['duration_s']}s, "
            f"frames: {frame_counts}"
        )
        self._episode_id += 1
        return meta

    def _consume_can(self) -> None:
        while not self._stop_event.is_set():
            try:
                joint_state: JointState = self._can_queue.get(timeout=0.5)
                with self._lock:
                    if self._recording:
                        self._joint_states.append(joint_state)
            except queue.Empty:
                continue

    def _consume_camera(self) -> None:
        while not self._stop_event.is_set():
            try:
                synced: SyncedFrameSet = self._camera_queue.get(timeout=0.5)
                with self._lock:
                    if not self._recording or self._episode_path is None:
                        continue
                    episode_path = self._episode_path

                # Write frames outside the lock — disk I/O shouldn't block recording state
                for name, frame in synced.frames.items():
                    write_jpeg_frame(frame, episode_path)
                    with self._lock:
                        self._frame_counts[name] += 1

            except queue.Empty:
                continue