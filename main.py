# main.py — OpenArm 2.0 data collection pipeline
#
# Starts:
#   - CAN pipeline (Task 2): MockCANPublisher + CANReader at 250Hz
#   - Camera pipeline (Task 3): 4x MockCamera + CameraSynchronizer at 30Hz
#   - EpisodeRecorder (Task 4): buffers data, writes on API trigger
#   - FastAPI server (Task 4): REST API on http://localhost:8000
#
# Recording triggered via API:
#   POST /episodes/start  → begin recording
#   POST /episodes/stop   → write episode to disk
#   GET  /episodes        → list episodes
#   GET  /episodes/{id}/download → download as .tar.gz

import signal
import sys
import threading
import time
from pathlib import Path

import uvicorn

from api.app import app, set_recorder
from can_reader.can_reader import CANReader
from can_reader.mock_publisher import MockCANPublisher
from camera_sync import CAMERA_NAMES, CameraSynchronizer, MockCamera
from recorder.episode_recorder import EpisodeRecorder

DATASET_PATH = Path("data")


def main() -> None:
    print("=" * 65)
    print("  OpenArm 2.0 — Data Collection Pipeline (Mock)")
    print("  Task 2: CAN      — 250Hz, 8 motors × 2 arms")
    print("  Task 3: Cameras  — 30.03Hz, 4 cameras")
    print("  Task 4: Storage  — OpenArm native format + REST API")
    print("  API:    http://localhost:8000")
    print("  Docs:   http://localhost:8000/docs")
    print("=" * 65)

    # ── Task 2: CAN pipeline ──────────────────────────────────────────
    publisher = MockCANPublisher()
    reader = CANReader()

    # ── Task 3: Camera pipeline ───────────────────────────────────────
    cameras = {name: MockCamera(name) for name in CAMERA_NAMES}
    synchronizer = CameraSynchronizer(cameras=cameras)

    # ── Task 4: Recorder ─────────────────────────────────────────────
    recorder = EpisodeRecorder(
        can_queue=reader.output_queue,
        camera_queue=synchronizer.output_queue,
        dataset_path=DATASET_PATH,
    )
    # Inject recorder before uvicorn starts — uses module-level state dict
    # so the reference survives uvicorn's import
    set_recorder(recorder)

    # ── Shutdown ──────────────────────────────────────────────────────
    def shutdown(sig, frame):
        print("\n[Main] Shutting down...")
        if recorder.is_recording:
            recorder.stop_recording()
        recorder.stop_threads()
        synchronizer.stop()
        for cam in cameras.values():
            cam.stop()
        reader.stop()
        publisher.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    # ── Start pipelines ───────────────────────────────────────────────
    publisher.start()
    time.sleep(0.1)
    reader.start()

    for cam in cameras.values():
        cam.start()
    time.sleep(0.1)
    synchronizer.start()

    recorder.start_threads()

    print("\n[Main] All pipelines running")
    print("[Main] Use the API to start/stop recording:")
    print("         curl -X POST http://localhost:8000/episodes/start")
    print("         curl -X POST http://localhost:8000/episodes/stop")
    print("         curl http://localhost:8000/episodes\n")

    # Blocks until Ctrl+C
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")


if __name__ == "__main__":
    main()