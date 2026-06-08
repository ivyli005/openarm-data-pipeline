# main.py
#
# Entry point for the OpenArm 2.0 data collection pipeline.
# Runs Task 2 (CAN reading) and Task 3 (camera sync) together.
#
# Architecture:
#   Task 2 — CAN pipeline
#     MockCANPublisher  → vcan0 (right arm) + vcan1 (left arm) at 250Hz
#     CANReader         → reads + parses → JointState objects → output_queue
#
#   Task 3 — Camera pipeline
#     MockCamera × 4   → wrist_left, wrist_right, ceiling, head at ~30.03Hz
#     CameraSynchronizer → aligns frames → SyncedFrameSet → output_queue
#
#   Both pipelines run fully in parallel (separate threads).
#   main.py consumes from both queues and prints live output.
#
# Display rate:
#   CAN:    print every 25th JointState  → ~10 prints/sec (250Hz / 25)
#   Camera: print every SyncedFrameSet  → ~30 prints/sec
#   Both are downsampled for readability — humans can't read 250Hz numbers.
#
# To run:
#   python3 main.py
#
# Press Ctrl+C to stop cleanly.

import signal
import sys
import threading
import time

from can_reader.mock_publisher import MockCANPublisher
from can_reader.can_reader import CANReader
from camera_sync import CameraSynchronizer, MockCamera, CAMERA_NAMES


def run_can_consumer(reader: CANReader, stop_event: threading.Event) -> None:
    """
    Consumes JointState objects from the CAN reader queue and prints them.
    Runs in its own thread so it doesn't block the camera consumer.
    Prints every 25th frame (~10/sec) so the terminal stays readable.
    """
    frame_count = 0
    start_time = time.time()

    while not stop_event.is_set():
        try:
            joint_state = reader.output_queue.get(timeout=0.5)
            frame_count += 1

            if frame_count % 25 == 0:
                elapsed = time.time() - start_time
                hz = frame_count / elapsed if elapsed > 0 else 0
                print(f"\n[CAN] frame={frame_count}  measured={hz:.1f}Hz  "
                      f"target=250Hz")
                joint_state.pretty_print()

        except Exception:
            # timeout or stop — loop again
            continue


def run_camera_consumer(
    synchronizer: CameraSynchronizer,
    stop_event: threading.Event,
) -> None:
    """
    Consumes SyncedFrameSet objects from the synchronizer queue and prints them.
    Runs in its own thread so it doesn't block the CAN consumer.

    Prints every synced set at ~30Hz — this is the right display rate for
    camera data.  In Task 5 (dashboard) these sets feed the MJPEG stream.
    """
    sync_count = 0

    while not stop_event.is_set():
        try:
            synced = synchronizer.output_queue.get(timeout=0.5)
            sync_count += 1

            # Print every 10th synced set (~3/sec) to keep terminal readable
            if sync_count % 10 == 0:
                print(f"\n[CAM] {synced.summary()}")
                print(f"[CAM] {synchronizer.stats_summary()}")

        except Exception:
            continue


def main() -> None:
    print("=" * 65)
    print("  OpenArm 2.0 — Data Collection Pipeline (Mock)")
    print("  Task 2: CAN reading   — 250Hz, 8 motors × 2 arms")
    print("  Task 3: Camera sync   — 30.03Hz, 4 cameras, ±50ms bound")
    print("  Interfaces: vcan0 (right), vcan1 (left)")
    print("=" * 65)

    # ------------------------------------------------------------------ #
    # Task 2: CAN pipeline
    # ------------------------------------------------------------------ #
    publisher = MockCANPublisher()
    reader = CANReader()

    # ------------------------------------------------------------------ #
    # Task 3: Camera pipeline
    # ------------------------------------------------------------------ #
    # Instantiate one MockCamera per stream.
    # Names match the real OpenArm dataset spec exactly:
    #   ds.camera_names → ['wrist_left', 'wrist_right', 'ceiling', 'head']
    # 'head' = ZED stereo camera (one stream; SDK handles stereo internally)
    cameras = {name: MockCamera(name) for name in CAMERA_NAMES}
    synchronizer = CameraSynchronizer(
        cameras=cameras,
        on_drop=lambda msg: None,  # silence drop logs in normal output;
                                   # swap for `print` to debug sync issues
    )

    # ------------------------------------------------------------------ #
    # Clean shutdown on Ctrl+C
    # ------------------------------------------------------------------ #
    stop_event = threading.Event()

    def shutdown(sig, frame):
        print("\n\n[Main] Shutting down...")
        stop_event.set()
        reader.stop()
        publisher.stop()
        for cam in cameras.values():
            cam.stop()
        synchronizer.stop()
        print("[Main] Final sync stats:")
        print(f"       {synchronizer.stats_summary()}")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    # ------------------------------------------------------------------ #
    # Start all components
    # Order matters:
    #   1. CAN publisher first (frames must exist before reader starts)
    #   2. CAN reader
    #   3. Cameras (independent — order doesn't matter between them)
    #   4. Synchronizer last (needs cameras already running)
    # ------------------------------------------------------------------ #
    publisher.start()
    time.sleep(0.1)   # let publisher warm up before reader starts
    reader.start()

    for cam in cameras.values():
        cam.start()
    time.sleep(0.1)   # let cameras produce at least one frame before sync starts
    synchronizer.start()

    print("\n[Main] All pipelines running — press Ctrl+C to stop\n")

    # ------------------------------------------------------------------ #
    # Consumer threads — one for CAN, one for cameras
    # Both run as daemon threads; main thread just waits for Ctrl+C
    # ------------------------------------------------------------------ #
    can_thread = threading.Thread(
        target=run_can_consumer,
        args=(reader, stop_event),
        daemon=True,
        name="CANConsumer",
    )
    cam_thread = threading.Thread(
        target=run_camera_consumer,
        args=(synchronizer, stop_event),
        daemon=True,
        name="CameraConsumer",
    )

    can_thread.start()
    cam_thread.start()

    # Keep main thread alive
    while not stop_event.is_set():
        time.sleep(0.5)


if __name__ == "__main__":
    main()