# OpenArm 2.0 — Data Collection Pipeline

Take-home project for DeepAware AI / Robotics Center of Silicon Valley.

---
## Quick Start

**Requirements:** Ubuntu 22.04, Python 3.10+

```bash
# 1. Set up virtual CAN interfaces
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
sudo ip link add dev vcan1 type vcan && sudo ip link set up vcan1

# 2. Install dependencies
pip3 install python-can fastapi uvicorn pandas pyarrow Pillow pyyaml

# 3. Run
cd openarm-project
python3 main.py
```

Open `http://localhost:8000/docs` in your browser. Use **POST /episodes/start** and **POST /episodes/stop** to record an episode. Recorded data lands in `data/episodes/`.

---

## Task 1: CAN Interface Setup

Configured virtual CAN interfaces `vcan0` and `vcan1` on Ubuntu 22.04 LTS (ARM64 VM in UTM on Apple Silicon Mac). A Linux VM is required — SocketCAN is a Linux kernel feature, macOS has no native CAN support.

Both interfaces confirmed UP via `ip link show` — see `docs/screenshots/Task1.png`.

**Real hardware sequence** (from [docs.openarm.dev/setup](https://docs.openarm.dev/setup)):

```bash
sudo apt install libopenarm-can-dev openarm-can-utils
openarm-can-cli can_configure          # auto-configures CAN FD at 5Mbps on can0 + can1
openarm-can-cli -i can0 set_zero --arm # set zero position right arm
openarm-can-cli -i can1 set_zero --arm # set zero position left arm
```

**Limitations:**
- `openarm-can-cli` requires a physical CAN adapter and OpenArm hardware
- Zero position calibration cannot be executed without the arm

---

## Task 2: CAN Data Reading

**Status:** Complete (software mock — no hardware access)

Multi-threaded pipeline reading joint states from both arms at 250Hz, matching the real OpenArm action frequency (`frequencies.action.arms = 250.0` from [OpenArm Dataset API](https://docs.openarm.dev/dataset/api)).

**Architecture:**
- `MockCANPublisher` — generates real Damiao MIT feedback frames at 250Hz on `vcan0` (right arm) and `vcan1` (left arm)
- `CANReader` — two threads (one per interface) block on `socket.recv()`, parse frames using real Damiao bit layout, assemble complete `JointState` objects
- Output: `arms/right/qpos`, `arms/right/qvel`, `arms/right/qtorque` (and left) — shape `(8,)` float32, matching real OpenArm dataset format

**Verified output (~220Hz):**

```
[t=1780950268.791]
  right qpos:    [ 3.174  2.442  1.492  0.409 -0.711 -1.767 -2.666 -3.326]
  right qvel:    [-1.220 -1.725 -0.462 -0.400 -1.487 -1.326 -1.062 -0.696]
  right qtorque: [ 0.435  0.356  0.089  0.021 -0.012 -0.046 -0.081 -0.085]
```

**Damiao MIT feedback frame format** — sourced from [official Damiao DM4310 datasheet](https://damiao.enactic.ai/en/products/hardware/dm-j4310-2ec-v1.1). Format is identical across all Damiao motor models:

| Byte | Content |
|------|---------|
| D[0] | Motor ID |
| D[1-2] | Position — 16-bit uint, ±12.5 rad |
| D[3-4] | Velocity — 12-bit uint, split across bytes |
| D[4-5] | Torque — 12-bit uint, split across bytes |
| D[6] | T_MOS — MOSFET temperature |
| D[7] | T_Rotor — rotor temperature |

**Per-joint motor configuration** — layout from [hardware diagram](https://docs.openarm.dev/hardware/openarm-2.0/motor), TAU_MAX from official Damiao datasheets:

| Joints | Motor | DQ_MAX | TAU_MAX |
|--------|-------|--------|---------|
| joint1-2 | DM8009P (shoulder) | 45 rad/s | 40 Nm |
| joint3 | DM4340P | 10 rad/s | 27 Nm |
| joint4 | DM4340 | 8 rad/s | 27 Nm |
| joint5-7 + gripper | DM4310 (wrist) | 30 rad/s | 7 Nm |

CAN IDs sourced from [openarm_can demo.cpp](https://github.com/enactic/openarm_can): feedback IDs `0x11–0x18`, right arm on `can0/vcan0`, left arm on `can1/vcan1`.

**Limitations:**
- Software timestamps (`time.time()`) — real hardware uses kernel `SO_TIMESTAMPING`
- `vcan` doesn't support CAN FD (tried but failed) — mock uses standard CAN frames (8-byte frame fits fine)
- Mock rate ~220Hz vs target 250Hz — Python/VM threading overhead; not an issue on real hardware
- Per-joint motor mapping inferred from [hardware diagram](https://docs.openarm.dev/hardware/openarm-2.0/motor) — not officially documented by Enactic

---

## Task 3: Multi-Camera Synchronization

**Status:** Complete (software mock — no hardware access)

4 camera streams matching the real OpenArm spec (`ds.camera_names → ['wrist_left', 'wrist_right', 'ceiling', 'head']`). The ZED stereo head is one stream — the SDK handles stereo internally.

**Architecture:**
- `MockCamera` — one thread per camera, generates `(600, 960, 3)` uint8 RGB frames at `30.303030Hz` (from `ds.meta.frequencies.cameras`)
- `CameraSynchronizer` — nearest-frame matching with ±50ms staleness bound; emits one `SyncedFrameSet` per tick when all 4 cameras are within bound
- Frame timestamps use `time.time_ns()` at capture time (not queue-put time), matching the real `{ns}.jpeg` filename format from the Dataset API

**Sync strategy:**

On each tick, drain each camera queue to get the freshest frame. Accept the set if all 4 timestamps are within 50ms of each other — the same approach as ROS `ApproximateTimeSynchronizer`. The 50ms bound (~1.5 frame periods at 30Hz) is a design choice; a hardware GPIO sync trigger would tighten this to ~5ms.

**Alignment with joint state data:**

No explicit CAN↔camera alignment logic is needed during recording. Both pipelines use `time.time()` on the same host. The Dataset API `Sampler` handles alignment at read time via `np.searchsorted` — it picks the camera frame with the smallest timestamp `>=` the sample timestamp. Maximum misalignment is one frame period (~33ms).

**Verified output:**

```
[CAM] SyncedFrameSet @ t=1780956580.189  jitter=3.2ms  dropped=0
  wrist_left    frame=00121  t=1780956580.170892
  wrist_right   frame=00121  t=1780956580.170797
  ceiling       frame=00121  t=1780956580.172117
  head          frame=00121  t=1780956580.173987
[CAM] Sync stats: ticks=120  dropped=0  drop_rate=0.0%  frames_drained=40
```

**Limitations / real hardware notes:**
- **ZED clock drift** — ZED SDK timestamps frames via `zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)` using a device-internal clock that drifts from wall clock. Fix: average `time.time() - zed.get_timestamp(TIME_REFERENCE.CURRENT)` over 100 samples at startup and apply as a fixed offset. Sufficient for 30s episodes. Not implemented in mock — all cameras share `time.time_ns()`.
- **No hardware sync trigger** — cameras free-run independently with ±2ms simulated USB jitter. A GPIO trigger on real hardware reduces this to ~0.1ms.
- **VM jitter artificially low** — measured 3.2–3.4ms inter-camera spread. Real USB cameras typically show 5–15ms; the ±50ms bound handles this comfortably.
- **Arducam serial numbers** — real setup requires `ArducamUvcConfigUpdateTool` to assign `CELL1_CAM_RIGHT`, `CELL1_CAM_LEFT`, `CELL1_CAM_CEILING` and udev rules to create stable `/dev/camera_*` symlinks. See [docs.openarm.dev/setup](https://docs.openarm.dev/setup).


---

## Task 4: Data Storage Backend

**Status:** Complete

Stores recorded episodes (joint states + camera frames) to disk and exposes them via a REST API.

### Storage format — native OpenArm 0.3.0

Chose the native OpenArm format over HDF5, zarr, or MCAP because the [Dataset API](https://docs.openarm.dev/dataset/api) defines an exact on-disk layout that the real `openarm_dataset` library reads directly. Using any other format would mean the stored data is incompatible with the training pipeline.

**Why not HDF5?** Common in robot learning but not what OpenArm uses. Would require a conversion step before training.

**Why not MCAP?** Good for replay/debugging (used by Foxglove) but not the OpenArm format. Worth considering for a separate replay tool.

On-disk layout per episode:

```
data/
├── metadata.yaml                          ← dataset info (cameras, frequencies, version)
├── episodes.db                            ← SQLite index for fast API queries
└── episodes/
    └── 0/
        ├── obs/state.parquet              ← joint states: qpos, qvel, qtorque for both arms
        ├── action/state.parquet           ← qpos only (per Dataset API spec)
        └── cameras/
            ├── wrist_left/
            │   └── 1780963485244807112.jpeg   ← filename = nanosecond timestamp
            ├── wrist_right/
            ├── ceiling/
            └── head/
```

The JPEG filenames are nanosecond timestamps (`time.time_ns()`). The Dataset API's `Camera.load_timestamps()` decodes them directly from filenames as `int(stem) / 1e9`. This is how camera frames get aligned with joint states at training time.

**Why SQLite?** Lightweight, no server setup needed, sufficient for a single-node collection system. The parquet files hold the actual sensor data — SQLite just holds the searchable index (episode id, duration, frame counts, timestamps).

**Atomic writes** — camera frames are written to disk one by one during recording (not buffered in memory). Writing all frames at stop time caused the VM to run out of memory (~1GB of raw numpy arrays for a 30s episode). Writing JPEGs live during recording keeps memory flat. Joint states are still buffered (tiny — just float32 arrays). On stop, only two small parquet files need to be written.

### REST API

Built with FastAPI. Auto-generated interactive docs at `http://localhost:8000/docs`.

| Endpoint | What it does |
|----------|-------------|
| `GET /status` | Is recording currently active? |
| `POST /episodes/start` | Start buffering data |
| `POST /episodes/stop` | Stop + write episode to disk |
| `GET /episodes` | List all recorded episodes |
| `GET /episodes/{id}` | Metadata for one episode |
| `GET /episodes/{id}/download` | Download episode as `.tar.gz` |

The download endpoint streams the `.tar.gz` in 64KB chunks rather than loading the whole episode into memory — a 30s episode with 4 cameras is ~50MB+.

### How to run

```bash
python3 main.py
```

Open `http://localhost:8000/docs` in your browser. All endpoints are interactive there.

Or use curl in a second terminal:

```bash
curl -X POST http://localhost:8000/episodes/start
sleep 5
curl -X POST http://localhost:8000/episodes/stop
curl http://localhost:8000/episodes
```

### Where to see the results

**API response** — the `/episodes/stop` call returns immediately with episode stats:
```json
{
  "id": 0,
  "duration_s": 28.91,
  "joint_states": 6249,
  "frame_counts": {"wrist_left": 870, "wrist_right": 870, "ceiling": 870, "head": 870}
}
```

**Files on disk** — in VSCode file explorer, open the `data/` folder:
- Click any `.jpeg` file to preview a camera frame
- Run `python3 -c "import pandas as pd; print(pd.read_parquet('data/episodes/0/obs/state.parquet').head())"` to inspect joint states

**Terminal** — shows write progress:
```
[Recorder] Started episode 0
[Recorder] Writing parquet for episode 0...
[Recorder] Episode 0 done — 6249 joint states, 28.91s
```

### Verified output

```
POST /episodes/start → {"recording": true, "episode_id": 0}
POST /episodes/stop  → {"id": 0, "duration_s": 28.91, "joint_states": 6249,
                        "frame_counts": {"wrist_left": 870, "wrist_right": 870,
                        "ceiling": 870, "head": 870}}
GET  /episodes       → same as above, queryable anytime
```


---

## Task 5: Monitoring Dashboard

*Coming soon*


## Given More Time / Real Hardware

**CAN / joint states:**
- Kernel `SO_TIMESTAMPING` for hardware-level CAN timestamps — `time.time()` is not accurate enough for robot learning datasets
- C++ CAN reader to hit exact 250Hz — Python threading overhead caps us at ~220Hz

**Cameras:**
- Replace mock with real drivers: `cv2.VideoCapture` for Arducams, `pyzed` SDK for ZED head camera
- Hardware GPIO sync trigger — reduces inter-camera jitter from ~5ms to ~0.1ms
- ZED clock offset correction at startup to align device-internal clock with host wall clock

**Storage / API:**
- Stream `.tar.gz` download directly from disk — currently builds full archive in memory first
- Episode `success` flag update endpoint so operators can mark episodes good/bad after review
- Convert to LeRobot v2.1 format via `Dataset.write(format="lerobot_v2.1")` for direct ACT policy training and upload to Hugging Face