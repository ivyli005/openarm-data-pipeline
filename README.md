# OpenArm 2.0 — Data Collection Pipeline

Software mock of the OpenArm 2.0 data collection pipeline. Built without hardware access — all CAN and camera data is simulated in software.

---

## Task 1: CAN Interface Setup

Configured virtual CAN interfaces `vcan0` and `vcan1` on Ubuntu 22.04 (ARM64 VM on Apple Silicon Mac).

Both interfaces confirmed UP via `ip link show`. Screenshot included.

**Real hardware steps** (requires physical OpenArm + CAN adapter):
```bash
sudo apt install libopenarm-can-dev openarm-can-utils
openarm-can-cli can_configure          # sets 5Mbps CAN FD
openarm-can-cli -i can0 set_zero --arm
openarm-can-cli -i can1 set_zero --arm
```

**Limitation:** `openarm-can-cli` requires physical hardware. Zero position step not executable without the arm.

---

## Task 2: CAN Data Reading

**Status: Complete (software mock)**

Multi-threaded pipeline reading joint states from both arms at 250Hz.

**Architecture:**
- `MockCANPublisher` — generates Damiao MIT feedback frames at 250Hz on vcan0/vcan1
- `CANReader` — two threads (one per interface) parse frames and assemble JointStates
- Output: `arms/right/qpos`, `arms/right/qvel`, `arms/right/qtorque` (and left) — shape (8,) float32

**Damiao MIT frame format** (sourced from official Damiao datasheets at https://damiao.enactic.ai/en/products/hardware/dm-j4310-2ec-v1.1):
- Position: 16-bit, ±12.5 rad (same across all motor models)
- Velocity: 12-bit, per motor type
- Torque: 12-bit, per motor type

**Per-joint motor types** (layout inferred from hardware diagram at docs.openarm.dev, TAU_MAX from official datasheets):

| Joints | Motor | DQ_MAX | TAU_MAX (datasheet peak) |
|---|---|---|---|
| joint1-2 | DM8009P (shoulder) | 45 rad/s | 40 Nm |
| joint3 | DM4340P | 10 rad/s | 27 Nm |
| joint4 | DM4340 | 8 rad/s | 27 Nm |
| joint5-7 + gripper | DM4310 (wrist) | 30 rad/s | 7 Nm |


**Limitations:**
- Software timestamps (`time.time()`) — real hardware uses kernel `SO_TIMESTAMPING`
- vcan doesn't support CAN FD (tried but failed) — mock uses standard CAN frames (8 bytes fits fine)
- Mock rate: ~220Hz vs target 250Hz (Python/VM overhead), would not be an issue for real hardware. 
- Per-joint motor mapping inferred from hardware diagram https://docs.openarm.dev/hardware/openarm-2.0/motor — not officially documented

---

## Task 3: Multi-Camera Synchronization

*Coming soon*

---

## Task 4: Data Storage Backend

*Coming soon*

---

## Task 5: Monitoring Dashboard

*Coming soon*
