# can_reader/mock_publisher.py
#
# Simulates OpenArm motor feedback over CAN FD at 250Hz.
#
# Implements the REAL Damiao Motor feedback frame format.
# Source: Official Damiao DM4310 datasheet + DM_CAN.py
#         (github.com/cmjang/DM_Control_Python)
#
# Feedback frame format (8 bytes) — same across ALL Damiao motor models:
#   D[0] = Motor ID | (ERR << 4)
#   D[1] = POS[15:8]             position high byte
#   D[2] = POS[7:0]              position low byte
#   D[3] = VEL[11:4]             velocity high 8 bits
#   D[4] = VEL[3:0] | T[11:8]   velocity low 4 bits + torque high 4 bits
#   D[5] = T[7:0]                torque low byte
#   D[6] = T_MOS                 MOSFET temperature (mock: 25)
#   D[7] = T_Rotor               rotor temperature  (mock: 25)
#
# Motor layout from: docs.openarm.dev/hardware/specifications/motor
#   joint1-2: DM8009P  (shoulder — highest torque)
#   joint3:   DM4340P
#   joint4:   DM4340
#   joint5-7 + gripper: DM4310 (wrist — lowest torque)
#
# Per-joint scaling from DM_CAN.py Limit_Param:
#   DM8009P: Q=12.5 rad, DQ=45 rad/s, TAU=54 Nm
#   DM4340P: Q=12.5 rad, DQ=10 rad/s, TAU=28 Nm
#   DM4340:  Q=12.5 rad, DQ=8  rad/s, TAU=28 Nm
#   DM4310:  Q=12.5 rad, DQ=30 rad/s, TAU=10 Nm
#
# CAN IDs from openarm_can demo.cpp:
#   Feedback (recv) IDs: 0x11-0x18 (slave ID + 0x10)
#   Right arm -> vcan0 (real: can0)
#   Left arm  -> vcan1 (real: can1)
#
# NOTE: Software mock. Real hardware differences:
#   - Real frames come from physical Damiao motors
#   - Real datasheet says 1Mbps standard CAN; OpenArm uses 5Mbps CAN FD
#   - Real timestamps use Linux kernel SO_TIMESTAMPING
#   - Bitrate set at OS level via openarm-can-cli, not in code

import can
import time
import threading
import math
import numpy as np

# Per-joint motor configuration
# Layout inferred from docs.openarm.dev motor diagram (2x DM8009P shoulder,
# 1x DM4340P, 1x DM4340, 4x DM4310 wrist/gripper)
# Scaling from DM_CAN.py Limit_Param (github.com/cmjang/DM_Control_Python)
MOTORS = [
    {"name": "joint1",  "slave": 0x01, "recv": 0x11, "Q_MAX": 12.5, "DQ_MAX": 45, "TAU_MAX": 54},  # DM8009P
    {"name": "joint2",  "slave": 0x02, "recv": 0x12, "Q_MAX": 12.5, "DQ_MAX": 45, "TAU_MAX": 54},  # DM8009P
    {"name": "joint3",  "slave": 0x03, "recv": 0x13, "Q_MAX": 12.5, "DQ_MAX": 10, "TAU_MAX": 28},  # DM4340P
    {"name": "joint4",  "slave": 0x04, "recv": 0x14, "Q_MAX": 12.5, "DQ_MAX": 8,  "TAU_MAX": 28},  # DM4340
    {"name": "joint5",  "slave": 0x05, "recv": 0x15, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 10},  # DM4310
    {"name": "joint6",  "slave": 0x06, "recv": 0x16, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 10},  # DM4310
    {"name": "joint7",  "slave": 0x07, "recv": 0x17, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 10},  # DM4310
    {"name": "gripper", "slave": 0x08, "recv": 0x18, "Q_MAX": 12.5, "DQ_MAX": 30, "TAU_MAX": 10},  # DM4310
]

# 250Hz per OpenArm dataset spec (frequencies.action.arms = 250.0)
PUBLISH_RATE_HZ  = 250
PUBLISH_INTERVAL = 1.0 / PUBLISH_RATE_HZ  # 0.004 seconds


def _float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """
    Linear mapping: float -> unsigned int.
    Matches float_to_uint() in DM_CAN.py exactly.
    """
    x = max(x_min, min(x_max, x))  # clamp to valid range
    span = x_max - x_min
    return int((x - x_min) / span * ((1 << bits) - 1))


def _pack_feedback_frame(motor_id: int,
                          position: float,
                          velocity: float,
                          torque: float,
                          q_max: float,
                          dq_max: float,
                          tau_max: float,
                          temp_mos: int = 25,
                          temp_rotor: int = 25) -> bytes:
    """
    Pack one motor feedback frame using the real Damiao byte format.
    Matches __process_packet() unpacking in DM_CAN.py exactly.

    D[0] = motor_id (no error in mock)
    D[1] = POS[15:8]
    D[2] = POS[7:0]
    D[3] = VEL[11:4]
    D[4] = VEL[3:0] | T[11:8]
    D[5] = T[7:0]
    D[6] = T_MOS
    D[7] = T_Rotor
    """
    pos_uint = _float_to_uint(position, -q_max,   q_max,   16)
    vel_uint = _float_to_uint(velocity, -dq_max,  dq_max,  12)
    tau_uint = _float_to_uint(torque,   -tau_max, tau_max, 12)

    data = bytearray(8)
    data[0] = motor_id & 0xFF
    data[1] = (pos_uint >> 8) & 0xFF
    data[2] =  pos_uint       & 0xFF
    data[3] = (vel_uint >> 4) & 0xFF
    data[4] = ((vel_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
    data[5] =  tau_uint       & 0xFF
    data[6] = temp_mos   & 0xFF
    data[7] = temp_rotor & 0xFF

    return bytes(data)


def _make_position(t: float, joint_index: int, arm: str, q_max: float) -> float:
    """Realistic joint position — sine wave scaled to motor range."""
    offset = 0.0 if arm == "right" else math.pi
    return math.sin(t * 0.5 + offset + joint_index * 0.3) * (q_max * 0.3)


def _make_velocity(t: float, joint_index: int, arm: str, dq_max: float) -> float:
    """Velocity = derivative of position, scaled to motor range."""
    offset = 0.0 if arm == "right" else math.pi
    return 0.5 * math.cos(t * 0.5 + offset + joint_index * 0.3) * (dq_max * 0.1)


def _make_torque(t: float, joint_index: int, tau_max: float) -> float:
    """Small realistic torque with noise, scaled to motor range."""
    base  = 0.1 * math.sin(t * 0.5 + joint_index * 0.3) * tau_max * 0.1
    noise = np.random.normal(0, 0.01)
    return float(base + noise)


class MockCANPublisher:
    """
    Publishes simulated Damiao motor feedback frames at 250Hz.
    One frame per motor (8 motors per arm).
    Right arm -> vcan0   Left arm -> vcan1
    """

    def __init__(self):
        self.bus0 = can.Bus(channel='vcan0', bustype='socketcan')
        self.bus1 = can.Bus(channel='vcan1', bustype='socketcan')
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._publish_loop,
            daemon=True,
            name="MockCANPublisher"
        )

    def start(self):
        print("[MockPublisher] Starting 250Hz on vcan0 (right) and vcan1 (left)")
        print(f"[MockPublisher] {len(MOTORS)} motors per arm — real Damiao MIT frame format")
        print("[MockPublisher] Per-joint scaling: DM8009P(j1-2), DM4340P(j3), DM4340(j4), DM4310(j5-7+gripper)")
        self._thread.start()

    def stop(self):
        print("[MockPublisher] Stopping...")
        self._stop_event.set()
        self.bus0.shutdown()
        self.bus1.shutdown()

    def _publish_loop(self):
        """Publish loop — maintains 250Hz by sleeping remainder of each interval."""
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            t = time.time()

            for i, motor in enumerate(MOTORS):
                recv_id  = motor["recv"]
                slave_id = motor["slave"]
                q_max    = motor["Q_MAX"]
                dq_max   = motor["DQ_MAX"]
                tau_max  = motor["TAU_MAX"]

                # Right arm on vcan0
                right_frame = _pack_feedback_frame(
                    motor_id = slave_id,
                    position = _make_position(t, i, "right", q_max),
                    velocity = _make_velocity(t, i, "right", dq_max),
                    torque   = _make_torque(t, i, tau_max),
                    q_max    = q_max,
                    dq_max   = dq_max,
                    tau_max  = tau_max,
                )
                self._send(self.bus0, recv_id, right_frame)

                # Left arm on vcan1
                left_frame = _pack_feedback_frame(
                    motor_id = slave_id,
                    position = _make_position(t, i, "left", q_max),
                    velocity = _make_velocity(t, i, "left", dq_max),
                    torque   = _make_torque(t, i, tau_max),
                    q_max    = q_max,
                    dq_max   = dq_max,
                    tau_max  = tau_max,
                )
                self._send(self.bus1, recv_id, left_frame)

            # Sleep remaining time to maintain 250Hz
            elapsed = time.monotonic() - loop_start
            sleep_time = PUBLISH_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _send(self, bus: can.Bus, arb_id: int, data: bytes):
        msg = can.Message(
            arbitration_id = arb_id,
            data           = data,
            is_fd          = False,
            is_extended_id = False
        )
        try:
            bus.send(msg)
        except can.CanError as e:
            print(f"[MockPublisher] Send error on ID {hex(arb_id)}: {e}")