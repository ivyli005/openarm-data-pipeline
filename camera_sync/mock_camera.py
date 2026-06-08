"""
camera_sync/mock_camera.py — Simulated camera streams at 30.03Hz.

On real hardware, replace _generate_frame() with:
  Arducam:  ret, bgr = cv2.VideoCapture(f"/dev/camera_{name}").read()
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
  ZED head: zed.retrieve_image(mat, sl.VIEW.LEFT)
            frame = mat.get_data()[:, :, :3]
            # ZED clock note: zed.get_timestamp(sl.TIME_REFERENCE.IMAGE) uses
            # a device-internal clock that drifts from wall clock. Fix: average
            # time.time() - zed.get_timestamp(TIME_REFERENCE.CURRENT) over 100
            # samples at startup and apply as a fixed offset to ZED timestamps.
            # Sufficient for 30s episodes. In this mock all cameras use time.time_ns().
"""

from __future__ import annotations
import queue
import random
import threading
import time
import numpy as np

from camera_sync.frame import CAMERA_NAMES, FRAME_CHANNELS, FRAME_HEIGHT, FRAME_WIDTH, CameraFrame

# From Dataset API: ds.meta.frequencies.cameras = 30.303030303030305 for all 4 cameras
CAMERA_HZ = 30.303030303030305
CAMERA_INTERVAL = 1.0 / CAMERA_HZ

# Real Arducam USB3 cameras show ~1-5ms delivery jitter at the OS level
JITTER_MIN_S = -0.002
JITTER_MAX_S = +0.002

# 2 seconds of frame buffer per camera
QUEUE_DEPTH = int(CAMERA_HZ * 2)


def _generate_frame(camera_name: str, frame_index: int) -> np.ndarray:
    """
    Generate a (600, 960, 3) uint8 RGB frame with gradient + text overlay.

    Gradient is seeded from frame_index so consecutive frames differ —
    identical frames would compress to identical bytes and hide I/O bugs.
    Per-camera hue makes streams visually distinguishable in the dashboard.
    """
    hue_map = {
        "wrist_left":  (60,  120, 200),
        "wrist_right": (200, 80,  60),
        "ceiling":     (60,  180, 80),
        "head":        (180, 140, 60),
    }
    base_r, base_g, base_b = hue_map.get(camera_name, (128, 128, 128))

    frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, FRAME_CHANNELS), dtype=np.uint8)
    x = np.linspace(0, 255, FRAME_WIDTH, dtype=np.float32)
    y_mod = np.sin(
        np.linspace(0, np.pi, FRAME_HEIGHT) + frame_index * 0.05
    ).reshape(-1, 1) * 40

    scale = x / 255.0
    frame[:, :, 0] = np.clip(base_r * scale + y_mod, 0, 255).astype(np.uint8)
    frame[:, :, 1] = np.clip(base_g * scale + y_mod, 0, 255).astype(np.uint8)
    frame[:, :, 2] = np.clip(base_b * scale + y_mod, 0, 255).astype(np.uint8)

    _write_label(frame, camera_name, frame_index)
    return frame


# Minimal 5x3 pixel font — avoids cv2/PIL dependency for text overlay
_FONT: dict[str, list[str]] = {
    "A": ["010","101","111","101","101"], "B": ["110","101","110","101","110"],
    "C": ["011","100","100","100","011"], "D": ["110","101","101","101","110"],
    "E": ["111","100","110","100","111"], "F": ["111","100","110","100","100"],
    "G": ["011","100","101","101","011"], "H": ["101","101","111","101","101"],
    "I": ["111","010","010","010","111"], "J": ["001","001","001","101","010"],
    "K": ["101","101","110","101","101"], "L": ["100","100","100","100","111"],
    "M": ["101","111","101","101","101"], "N": ["101","111","111","111","101"],
    "O": ["010","101","101","101","010"], "P": ["110","101","110","100","100"],
    "Q": ["010","101","101","111","011"], "R": ["110","101","110","101","101"],
    "S": ["011","100","010","001","110"], "T": ["111","010","010","010","010"],
    "U": ["101","101","101","101","010"], "V": ["101","101","101","010","010"],
    "W": ["101","101","101","111","101"], "X": ["101","101","010","101","101"],
    "Y": ["101","101","010","010","010"], "Z": ["111","001","010","100","111"],
    "0": ["010","101","101","101","010"], "1": ["010","110","010","010","111"],
    "2": ["110","001","010","100","111"], "3": ["110","001","010","001","110"],
    "4": ["101","101","111","001","001"], "5": ["111","100","110","001","110"],
    "6": ["010","100","110","101","010"], "7": ["111","001","010","010","010"],
    "8": ["010","101","010","101","010"], "9": ["010","101","011","001","010"],
    "_": ["000","000","000","000","111"], " ": ["000","000","000","000","000"],
    ".": ["000","000","000","000","010"], ":": ["000","010","000","010","000"],
    "=": ["000","111","000","111","000"], "-": ["000","000","111","000","000"],
}
_SCALE = 2  # each logical pixel → 2×2 block


def _write_label(frame: np.ndarray, camera_name: str, frame_index: int) -> None:
    lines = [
        camera_name.upper(),
        f"FRAME={frame_index:05d}",
        f"T={time.time():.3f}",
        "30.03 HZ",
    ]
    y = 8
    for line in lines:
        _write_string(frame, line, x=8, y=y, color=(255, 255, 255))
        y += (5 + 2) * _SCALE


def _write_string(frame, text, x, y, color):
    x_cursor = x
    for ch in text.upper():
        bitmap = _FONT.get(ch, _FONT[" "])
        _write_char(frame, bitmap, x_cursor, y, color)
        x_cursor += (3 + 1) * _SCALE


def _write_char(frame, bitmap, x, y, color):
    h, w = frame.shape[:2]
    for row_i, row in enumerate(bitmap):
        for col_i, px in enumerate(row):
            if px == "1":
                for dy in range(_SCALE):
                    for dx in range(_SCALE):
                        ry, rx = y + row_i * _SCALE + dy, x + col_i * _SCALE + dx
                        if 0 <= ry < h and 0 <= rx < w:
                            frame[ry, rx] = color


class MockCamera:
    """One simulated camera stream at ~30.03Hz."""

    def __init__(self, camera_name: str) -> None:
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"Unknown camera '{camera_name}'. Expected: {CAMERA_NAMES}")
        self.camera_name = camera_name
        self.latest_queue: queue.Queue[CameraFrame] = queue.Queue(maxsize=QUEUE_DEPTH)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"MockCamera-{camera_name}",
        )
        self._frame_index = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def _capture_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            # Timestamp at capture time, not queue-put time — matches real driver behaviour
            timestamp_ns = time.time_ns()
            frame_array = _generate_frame(self.camera_name, self._frame_index)
            camera_frame = CameraFrame.create(
                camera_name=self.camera_name,
                frame_index=self._frame_index,
                frame=frame_array,
                timestamp_ns=timestamp_ns,
            )

            # Drop oldest if full — prefer fresh frames over stale ones
            if self.latest_queue.full():
                try:
                    self.latest_queue.get_nowait()
                except queue.Empty:
                    pass
            try:
                self.latest_queue.put_nowait(camera_frame)
            except queue.Full:
                pass

            self._frame_index += 1

            # Sleep with jitter to simulate USB scheduling variance
            next_tick += CAMERA_INTERVAL
            remaining = next_tick + random.uniform(JITTER_MIN_S, JITTER_MAX_S) - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)