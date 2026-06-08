# can_reader/joint_state.py
# Data structure representing a single joint state reading from one arm.
# Mirrors the real OpenArm format: 8 values per arm (joint1-7 + gripper)

import time
from dataclasses import dataclass, field
import numpy as np


@dataclass
class ArmState:
    """State for one arm (left or right) — 8 joints: joint1-7 + gripper."""
    qpos: np.ndarray    # position, shape (8,)
    qvel: np.ndarray    # velocity, shape (8,)
    qtorque: np.ndarray # torque,   shape (8,)

    def __post_init__(self):
        # ensure all arrays are float32, shape (8,)
        self.qpos    = np.asarray(self.qpos,    dtype=np.float32)
        self.qvel    = np.asarray(self.qvel,    dtype=np.float32)
        self.qtorque = np.asarray(self.qtorque, dtype=np.float32)
        assert self.qpos.shape == (8,), "qpos must have 8 values"
        assert self.qvel.shape == (8,), "qvel must have 8 values"
        assert self.qtorque.shape == (8,), "qtorque must have 8 values"


@dataclass
class JointState:
    """
    Combined state of both arms at a single timestep.
    Matches the real OpenArm obs keys:
      arms/right/qpos, arms/right/qvel, arms/right/qtorque
      arms/left/qpos,  arms/left/qvel,  arms/left/qtorque
    """
    timestamp: float          # Unix time in seconds (time.time())
    right: ArmState           # right arm — comes from can0
    left: ArmState            # left arm  — comes from can1

    def to_dict(self):
        """Return data in the real OpenArm key format."""
        return {
            "timestamp": self.timestamp,
            "arms/right/qpos":    self.right.qpos,
            "arms/right/qvel":    self.right.qvel,
            "arms/right/qtorque": self.right.qtorque,
            "arms/left/qpos":     self.left.qpos,
            "arms/left/qvel":     self.left.qvel,
            "arms/left/qtorque":  self.left.qtorque,
        }

    def pretty_print(self):
        """Human-readable output for terminal monitoring."""
        print(f"\n[t={self.timestamp:.3f}]")
        print(f"  right qpos:    {np.round(self.right.qpos, 3)}")
        print(f"  right qvel:    {np.round(self.right.qvel, 3)}")
        print(f"  right qtorque: {np.round(self.right.qtorque, 3)}")
        print(f"  left  qpos:    {np.round(self.left.qpos, 3)}")
        print(f"  left  qvel:    {np.round(self.left.qvel, 3)}")
        print(f"  left  qtorque: {np.round(self.left.qtorque, 3)}")