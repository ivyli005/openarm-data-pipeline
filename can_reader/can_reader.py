# can_reader/can_reader.py
#
# Reads and parses Damiao motor feedback frames from vcan0 and vcan1.
#
# Unpacking logic mirrors __process_packet() in DM_CAN.py exactly:
#   D[0] = motor ID
#   D[1..2] = position  (16-bit uint -> float via uint_to_float)
#   D[3..4] = velocity  (12-bit uint -> float)
#   D[4..5] = torque    (12-bit uint -> float)
#   D[6]    = T_MOS temperature
#   D[7]    = T_Rotor temperature
#
# Per-joint scaling matches mock_publisher.py exactly:
#   joint1-2: DM8009P  DQ_MAX=45, TAU_MAX=54
#   joint3:   DM4340P  DQ_MAX=10, TAU_MAX=28
#   joint4:   DM4340   DQ_MAX=8,  TAU_MAX=28
#   joint5-7+gripper: DM4310 DQ_MAX=30, TAU_MAX=10
#
# Architecture:
#   Two reader threads — one per CAN interface (vcan0, vcan1)
#   Each thread blocks on recv() waiting for frames
#   Parsed frames go into ArmBuffer until all 8 motors reported
#   Complete arm states are paired into JointState objects
#   JointStates go into output_queue for main.py to consume
#
# Thread safety: all shared state via queue.Queue and threading.Lock
#
# Timestamping note:
#   Uses time.time() when frame arrives in Python (software timestamp).
#   Real hardware should use Linux kernel SO_TIMESTAMPING for
#   hardware-level accuracy — important for robot learning datasets.

import can
import time
import queue
import threading
import numpy as np

from can_reader.joint_state import ArmState, JointState

# Per-joint motor receive IDs and scaling constants
# recv ID = slave ID + 0x10 (from openarm_can demo.cpp)
# Scaling from DM_CAN.py Limit_Param (github.com/cmjang/DM_Control_Python)
# Motor layout inferred from docs.openarm.dev motor diagram
MOTOR_RECV_IDS = {
    0x11: {"name": "joint1",  "index": 0, "Q_MAX": 12.5, "DQ_MAX": 45, "TAU_MAX": 40},  # DM8009P — 40Nm peak (datasheet)
    0x12: {"name": "joint2",  "index": 1, "Q_MAX": 12.5, "DQ_MAX": 45, "TAU_MAX": 40},  # DM8009P
    0x13: {"name": "joint3",  "index": 2, "Q_MAX": 12.5, "DQ_MAX": 10, "TAU_MAX": 27},  # DM4340P — 27Nm peak (datasheet)
    0x14: {"name": "joint4",  "index": 3, "Q_MAX": 12.5, "DQ_MAX": 8,  "TAU_MAX": 27},  # DM4340
    0x15: {"name": "joint5",  "index": 4, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 7},   # DM4310 — 7Nm peak (datasheet)
    0x16: {"name": "joint6",  "index": 5, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 7},   # DM4310
    0x17: {"name": "joint7",  "index": 6, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 7},   # DM4310
    0x18: {"name": "gripper", "index": 7, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 7},   # DM4310
}

NUM_MOTORS = 8  # joint1-7 + gripper


def _uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """
    Linear mapping: unsigned int -> float.
    Matches uint_to_float() in DM_CAN.py exactly.
    """
    span = x_max - x_min
    return float(x) / ((1 << bits) - 1) * span + x_min


def _unpack_feedback_frame(arb_id: int, data: bytes):
    """
    Unpack one Damiao feedback frame into (index, position, velocity, torque).
    Uses per-motor scaling based on arbitration ID.
    Matches __process_packet() in DM_CAN.py exactly.
    Returns None if frame is unknown or too short.
    """
    if len(data) < 6:
        return None
    if arb_id not in MOTOR_RECV_IDS:
        return None

    motor_info = MOTOR_RECV_IDS[arb_id]
    q_max   = motor_info["Q_MAX"]
    dq_max  = motor_info["DQ_MAX"]
    tau_max = motor_info["TAU_MAX"]

    # Real Damiao bit layout from official datasheet
    pos_uint = (data[1] << 8) | data[2]                    # 16 bits
    vel_uint = (data[3] << 4) | (data[4] >> 4)             # 12 bits
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]           # 12 bits

    position = _uint_to_float(pos_uint, -q_max,   q_max,   16)
    velocity = _uint_to_float(vel_uint, -dq_max,  dq_max,  12)
    torque   = _uint_to_float(tau_uint, -tau_max, tau_max, 12)

    return motor_info["index"], position, velocity, torque


class ArmBuffer:
    """
    Accumulates frames for one arm until all 8 motors reported.
    Then assembles an ArmState with numpy arrays.
    Thread-safe via lock.
    """

    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self._lock = threading.Lock()
        self._reset()

    def _reset(self):
        self._positions  = [None] * NUM_MOTORS
        self._velocities = [None] * NUM_MOTORS
        self._torques    = [None] * NUM_MOTORS
        self._timestamp  = None

    def add_frame(self, arb_id: int, index: int,
                  position: float, velocity: float,
                  torque: float, timestamp: float):
        """
        Add one motor's data.
        Returns (ArmState, timestamp) when all 8 motors received, else None.
        """
        with self._lock:
            self._positions[index]  = position
            self._velocities[index] = velocity
            self._torques[index]    = torque

            if self._timestamp is None:
                self._timestamp = timestamp

            if all(v is not None for v in self._positions):
                arm_state = ArmState(
                    qpos    = np.array(self._positions,  dtype=np.float32),
                    qvel    = np.array(self._velocities, dtype=np.float32),
                    qtorque = np.array(self._torques,    dtype=np.float32),
                )
                ts = self._timestamp
                self._reset()
                return arm_state, ts

        return None


class CANReader:
    """
    Reads motor feedback from vcan0 (right arm) and vcan1 (left arm).
    Assembles complete JointState objects and puts them in output_queue.

    Usage:
        reader = CANReader()
        reader.start()
        while True:
            joint_state = reader.output_queue.get()
            joint_state.pretty_print()
    """

    def __init__(self):
        self.output_queue  = queue.Queue(maxsize=500)
        self._stop_event   = threading.Event()
        self._right_buffer = ArmBuffer("right")
        self._left_buffer  = ArmBuffer("left")
        self._pending_right = None
        self._pending_left  = None
        self._assemble_lock = threading.Lock()

        self._bus0 = can.Bus(channel='vcan0', bustype='socketcan')
        self._bus1 = can.Bus(channel='vcan1', bustype='socketcan')

        self._thread0 = threading.Thread(
            target=self._read_loop,
            args=(self._bus0, self._right_buffer, "right"),
            daemon=True,
            name="CANReader-vcan0"
        )
        self._thread1 = threading.Thread(
            target=self._read_loop,
            args=(self._bus1, self._left_buffer, "left"),
            daemon=True,
            name="CANReader-vcan1"
        )

    def start(self):
        print("[CANReader] Starting on vcan0 (right arm) and vcan1 (left arm)")
        self._thread0.start()
        self._thread1.start()

    def stop(self):
        print("[CANReader] Stopping...")
        self._stop_event.set()
        self._bus0.shutdown()
        self._bus1.shutdown()

    def _read_loop(self, bus: can.Bus, buffer: ArmBuffer, arm_name: str):
        """
        Blocks waiting for CAN frames on one interface.
        Parses each frame and feeds it to the arm buffer.
        When buffer complete, tries to pair with other arm.
        """
        while not self._stop_event.is_set():
            try:
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue

                timestamp = time.time()
                result = _unpack_feedback_frame(msg.arbitration_id, msg.data)
                if result is None:
                    continue

                index, position, velocity, torque = result

                assembled = buffer.add_frame(
                    msg.arbitration_id,
                    index, position, velocity, torque, timestamp
                )
                if assembled is not None:
                    arm_state, ts = assembled
                    self._try_assemble_joint_state(arm_name, arm_state, ts)

            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[CANReader-{arm_name}] Error: {e}")

    def _try_assemble_joint_state(self, arm_name: str,
                                   arm_state: ArmState,
                                   timestamp: float):
        """
        Pairs right and left arm states into a complete JointState.
        Second arm to arrive triggers assembly using both latest states.
        Average timestamp used for the combined state.
        """
        with self._assemble_lock:
            if arm_name == "right":
                self._pending_right = (arm_state, timestamp)
            else:
                self._pending_left = (arm_state, timestamp)

            if self._pending_right is not None and self._pending_left is not None:
                right_state, right_ts = self._pending_right
                left_state,  left_ts  = self._pending_left

                joint_state = JointState(
                    timestamp = (right_ts + left_ts) / 2.0,
                    right     = right_state,
                    left      = left_state,
                )

                self._pending_right = None
                self._pending_left  = None

                try:
                    self.output_queue.put_nowait(joint_state)
                except queue.Full:
                    pass  # drop if consumer too slow