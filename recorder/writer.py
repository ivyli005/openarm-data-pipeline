"""
recorder/writer.py — Writes episode data in native OpenArm format.

On-disk layout matches Dataset API spec (docs.openarm.dev/dataset/api):
    episodes/{id}/
        obs/state.parquet
        action/state.parquet
        cameras/{name}/{timestamp_ns}.jpeg
"""

import time
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from PIL import Image

from can_reader.joint_state import JointState
from camera_sync.frame import CameraFrame

# From Dataset API: embodiment.joints = ('joint1'...'joint7', 'gripper')
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"]


def write_jpeg_frame(frame: CameraFrame, episode_path: Path) -> None:
    """
    Write one camera frame to disk immediately.
    Filename = {timestamp_ns}.jpeg — matches Dataset API Camera.load_timestamps()
    which decodes timestamps as int(stem) / 1e9.
    Called during recording, not at stop time, to avoid memory buildup.
    """
    cam_dir = episode_path / "cameras" / frame.camera_name
    img = Image.fromarray(frame.frame, mode="RGB")
    # quality=85: good balance between file size and image quality for robot learning
    img.save(cam_dir / f"{frame.timestamp_ns}.jpeg", format="JPEG", quality=85)


def _arm_df(states: list[JointState], side: str, attr: str) -> pd.DataFrame:
    """
    Build a DataFrame for one arm / one attribute.
    Index = timestamp, columns = JOINT_NAMES.
    Matches Dataset API: obs["arms/right/qpos"] → shape (N, 8).
    """
    timestamps = [s.timestamp for s in states]
    data = [getattr(getattr(s, side), attr) for s in states]
    return pd.DataFrame(
        np.array(data, dtype=np.float32),
        index=pd.Index(timestamps, name="timestamp"),
        columns=JOINT_NAMES,
    )


def write_obs_parquet(states: list[JointState], episode_path: Path) -> None:
    obs_dir = episode_path / "obs"
    obs_dir.mkdir(parents=True, exist_ok=True)

    frames = {}
    for side in ("right", "left"):
        for attr in ("qpos", "qvel", "qtorque"):
            df = _arm_df(states, side, attr)
            df.columns = [f"{side}_{attr}_{j}" for j in JOINT_NAMES]
            frames[f"{side}_{attr}"] = df

    pd.concat(frames.values(), axis=1).to_parquet(obs_dir / "state.parquet", index=True)


def write_action_parquet(states: list[JointState], episode_path: Path) -> None:
    # Per Dataset API spec: action stores qpos only
    action_dir = episode_path / "action"
    action_dir.mkdir(parents=True, exist_ok=True)

    frames = {}
    for side in ("right", "left"):
        df = _arm_df(states, side, "qpos")
        df.columns = [f"{side}_qpos_{j}" for j in JOINT_NAMES]
        frames[f"{side}_qpos"] = df

    pd.concat(frames.values(), axis=1).to_parquet(action_dir / "state.parquet", index=True)


def write_metadata_yaml(
    episode_id: int,
    start_time: float,
    end_time: float,
    dataset_path: Path,
) -> None:
    """
    Write/update metadata.yaml at dataset root.
    Re-written on each new episode to append to episodes list.
    """
    meta_path = dataset_path / "metadata.yaml"

    if meta_path.exists():
        with open(meta_path) as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {
            "version": "0.3.0",
            "operator": "mock",
            "operation_type": "teleop",
            "location": "mock",
            "tasks": [{"prompt": "Mock teleoperation", "description": "Mock episode"}],
            "episodes": [],
            "equipment": {
                "id": "OpenArm",
                "version": "2.0",
                "embodiments": {
                    "arms": {"id": "OpenArm", "version": "2.0"}
                },
                "perceptions": {
                    "cameras": {
                        "wrist_left":  {"name": "wrist_left"},
                        "wrist_right": {"name": "wrist_right"},
                        "ceiling":     {"name": "ceiling"},
                        "head":        {"name": "head"},
                    }
                },
            },
            "frequencies": {
                # From Dataset API: ds.meta.frequencies
                "cameras": {
                    "wrist_left":  30.303030303030305,
                    "wrist_right": 30.303030303030305,
                    "ceiling":     30.303030303030305,
                    "head":        30.303030303030305,
                },
                "obs":    {"arms": {"left": 250.0, "right": 250.0}},
                "action": {"arms": {"left": 250.0, "right": 250.0}},
            },
        }

    data["episodes"].append({
        "id": str(episode_id),
        "success": False,
        "task_index": 0,
    })

    with open(meta_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)