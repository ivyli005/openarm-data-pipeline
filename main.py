# main.py
#
# Entry point for the OpenArm data collection pipeline.
# Ties together the mock CAN publisher and reader for Task 2.
#
# What this does:
#   1. Starts MockCANPublisher — generates fake motor frames at 250Hz
#      on vcan0 (right arm) and vcan1 (left arm)
#   2. Starts CANReader — reads and parses those frames back
#   3. Prints live joint states to terminal to verify everything works
#
# To run:
#   python3 main.py
#
# Expected output:
#   Joint states printing at ~250Hz showing position/velocity/torque
#   for all 8 joints on both arms
#
# Press Ctrl+C to stop.

import time
import signal
import sys

from can_reader.mock_publisher import MockCANPublisher
from can_reader.can_reader import CANReader


def main():
    print("=" * 60)
    print("  OpenArm 2.0 — CAN Data Collection Pipeline (Mock)")
    print("  Task 2: CAN Data Reading")
    print("  Interfaces: vcan0 (right arm), vcan1 (left arm)")
    print("  Rate: 250Hz | Motors: 8 per arm | Format: Damiao MIT")
    print("=" * 60)

    publisher = MockCANPublisher()
    reader    = CANReader()

    # Clean shutdown on Ctrl+C
    def shutdown(sig, frame):
        print("\n[Main] Shutting down...")
        reader.stop()
        publisher.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    # Start publisher first so frames are ready when reader starts
    publisher.start()
    time.sleep(0.1)  # small delay to let publisher warm up
    reader.start()

    print("\n[Main] Pipeline running — press Ctrl+C to stop\n")

    # Print joint states as they arrive
    frame_count = 0
    start_time  = time.time()

    while True:
        try:
            # Block until next joint state arrives (timeout 1s)
            joint_state = reader.output_queue.get(timeout=1.0)
            frame_count += 1

            # Print every 25th frame so terminal isn't flooded
            # (250Hz / 25 = ~10 prints per second — readable)
            if frame_count % 25 == 0:
                elapsed = time.time() - start_time
                hz = frame_count / elapsed
                print(f"[Main] {frame_count} frames | {hz:.1f} Hz")
                joint_state.pretty_print()

        except Exception:
            print("[Main] Waiting for frames...")


if __name__ == "__main__":
    main()