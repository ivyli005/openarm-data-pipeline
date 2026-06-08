# OpenArm 2.0 — Data Collection Pipeline

Software mock of the OpenArm 2.0 data collection pipeline for teleoperation data collection. Built without hardware access using Python, python-can, and SocketCAN on Ubuntu 22.04. All CAN and camera data is simulated in software with real hardware protocols implemented where possible.

---

## Task 1: CAN Interface Setup

Configured virtual CAN interfaces `vcan0` and `vcan1` on Ubuntu 22.04 LTS (ARM64 VM in UTM on Apple Silicon Mac). A Linux VM is required because SocketCAN is a Linux kernel feature — macOS has no native CAN support.

Both interfaces confirmed UP via `ip link show` — see `docs/screenshots/task1_vcan_up.png`.

**Real hardware sequence** (from docs.openarm.dev/setup):
```bash
sudo apt install libopenarm-can-dev openarm-can-utils
openarm-can-cli can_configure          # auto-configures CAN FD at 5Mbps on can0 + can1
openarm-can-cli -i can0 set_zero --arm # set zero position right arm
openarm-can-cli -i can1 set_zero --arm # set zero position left arm
```

**Limitations:** `openarm-can-cli` requires a physical CAN adapter and OpenArm hardware. Zero position calibration cannot be executed without the arm.

---

## Task 2: CAN Data Reading

**Status: Complete (software mock — no hardware access)**

Multi-threaded pipeline reading joint states from both arms at 250Hz, matching the real OpenArm action frequency (`frequencies.action.arms = 250.0` from OpenArm Dataset API docs).

**Architecture:**
- `MockCANPublisher` — generates real Damiao MIT feedback frames at 250Hz on vcan0 (right arm) and vcan1 (left arm)
- `CANReader` — two threads (one per interface) block on `socket.recv()`, parse frames using real Damiao bit layout, assemble complete JointStates
- Output: `arms/right/qpos`, `arms/right/qvel`, `arms/right/qtorque` (and left) — shape (8,) float32, matching real OpenArm dataset format

**Verified running at ~220Hz with correct joint values:**

[t=1780950268.791]
right qpos:    [ 3.174  2.442  1.492  0.409 -0.711 -1.767 -2.666 -3.326]
right qvel:    [-1.22  -1.725 -0.462 -0.4   -1.487 -1.326 -1.062 -0.696]
right qtorque: [ 0.435  0.356  0.089  0.021 -0.012 -0.046 -0.081 -0.085]

**Real Damiao MIT feedback frame format** — sourced from official Damiao DM4310 datasheet https://damiao.enactic.ai/en/products/hardware/dm-j4310-2ec-v1.1. Format is identical across all Damiao motor models:

| Byte | Content |
|---|---|
| D[0] | Motor ID |
| D[1-2] | Position — 16-bit uint, ±12.5 rad |
| D[3-4] | Velocity — 12-bit uint, split across bytes |
| D[4-5] | Torque — 12-bit uint, split across bytes |
| D[6] | T_MOS — MOSFET temperature |
| D[7] | T_Rotor — rotor temperature |

**Per-joint motor configuration** — layout from hardware diagram at docs.openarm.dev/hardware/openarm-2.0/motor , TAU_MAX from official Damiao datasheets:

| Joints | Motor | DQ_MAX | TAU_MAX |
|---|---|---|---|
| joint1-2 | DM8009P (shoulder) | 45 rad/s | 40 Nm |
| joint3 | DM4340P | 10 rad/s | 27 Nm |
| joint4 | DM4340 | 8 rad/s | 27 Nm |
| joint5-7 + gripper | DM4310 (wrist) | 30 rad/s | 7 Nm |

**CAN IDs** sourced from openarm_can demo.cpp (github.com/enactic/openarm_can): feedback IDs 0x11–0x18, right arm on can0/vcan0, left arm on can1/vcan1.

**Limitations:**
- Software timestamps (time.time()) — real hardware uses kernel SO_TIMESTAMPING
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