"""Shared datatypes for XR observation and sparse extremity targets.

Design rule: every embodiment (dual-arm or humanoid) consumes SparseTarget.
Joint-space retargeting, if any, lives inside a Backend — never as the bus format.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

# SE(3) pose: xyz + quaternion xyzw
POSE_DIM = 7


def identity_pose() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def pose_to_matrix(pose: np.ndarray) -> np.ndarray:
    """Convert xyz + quat(xyzw) to 4x4 homogeneous matrix."""
    x, y, z, qx, qy, qz, qw = pose
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    R = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_pose(T: np.ndarray) -> np.ndarray:
    """Convert 4x4 matrix to xyz + quat(xyzw)."""
    R = T[:3, :3]
    t = T[:3, 3]
    trace = np.trace(R)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    return np.concatenate([t, q])


def invert_pose(pose: np.ndarray) -> np.ndarray:
    return matrix_to_pose(np.linalg.inv(pose_to_matrix(pose)))


def multiply_poses(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return matrix_to_pose(pose_to_matrix(a) @ pose_to_matrix(b))


@dataclass
class ButtonState:
    left_grip: float = 0.0
    right_grip: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    button_a: bool = False
    button_b: bool = False
    button_x: bool = False
    button_y: bool = False
    left_menu: bool = False
    right_menu: bool = False
    left_axis: tuple[float, float] = (0.0, 0.0)
    right_axis: tuple[float, float] = (0.0, 0.0)
    left_axis_click: bool = False
    right_axis_click: bool = False


@dataclass
class XRObservation:
    """Raw XR stream — embodiment-agnostic.

    body_joints: optional 24x7 Pico full-body poses (Enterprise + trackers).
    trackers: optional dict serial -> pose (ankle / object trackers).
    """

    timestamp_ns: int
    head: np.ndarray = field(default_factory=identity_pose)
    left_controller: np.ndarray = field(default_factory=identity_pose)
    right_controller: np.ndarray = field(default_factory=identity_pose)
    left_hand: Optional[np.ndarray] = None  # wrist or 26-joint flattened later
    right_hand: Optional[np.ndarray] = None
    body_joints: Optional[np.ndarray] = None  # (24, 7)
    trackers: Dict[str, np.ndarray] = field(default_factory=dict)
    buttons: ButtonState = field(default_factory=ButtonState)
    valid: bool = True
    stream_ok: bool = True  # False when XR link looks stale / reconnecting
    reconnected: bool = False  # one-shot: True on the frame after successful reconnect


# Canonical extremity / keypoint names used across dual-arm and humanoid.
EXTREMITY_LEFT_EE = "left_ee"
EXTREMITY_RIGHT_EE = "right_ee"
EXTREMITY_HEAD = "head"
EXTREMITY_PELVIS = "pelvis"
EXTREMITY_LEFT_FOOT = "left_foot"
EXTREMITY_RIGHT_FOOT = "right_foot"
EXTREMITY_LEFT_ELBOW = "left_elbow"
EXTREMITY_RIGHT_ELBOW = "right_elbow"


@dataclass
class SparseTarget:
    """Unified control bus: sparse SE(3) extremities + scalar auxiliaries.

    Dual-arm typically fills left_ee / right_ee / head + grippers.
    Humanoid additionally fills pelvis / feet and may set root_height / root_vel.
    """

    timestamp_ns: int
    extremities: Dict[str, np.ndarray] = field(default_factory=dict)
    # Active flags: False means "hold previous / do not update this link"
    active: Dict[str, bool] = field(default_factory=dict)
    left_gripper: float = 0.0  # 0 open .. 1 closed
    right_gripper: float = 0.0
    root_height: Optional[float] = None
    root_lin_vel: Optional[np.ndarray] = None  # (3,)
    root_ang_vel: Optional[np.ndarray] = None  # (3,)
    # Optional dense keypoints in robot/base frame for policy backends (HuMiT-style)
    keypoints: Optional[Dict[str, np.ndarray]] = None
    meta: Dict[str, float] = field(default_factory=dict)

    def get_pose(self, name: str) -> Optional[np.ndarray]:
        return self.extremities.get(name)

    def is_active(self, name: str) -> bool:
        return bool(self.active.get(name, False))
