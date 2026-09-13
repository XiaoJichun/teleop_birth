"""Unitree G1 backend skeleton — same SparseTarget bus as dual-arm.

Phase-2: wire Unitree SDK / HuMiT / MOSAIC / xr_teleoperate behind this class
without changing TeleopLoop or XRSource.
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from teleop_birth.embodiment.base import BackendState, RobotBackend
from teleop_birth.schema import (
    EXTREMITY_HEAD,
    EXTREMITY_LEFT_EE,
    EXTREMITY_LEFT_FOOT,
    EXTREMITY_PELVIS,
    EXTREMITY_RIGHT_EE,
    EXTREMITY_RIGHT_FOOT,
    SparseTarget,
    identity_pose,
)


class G1BackendStub(RobotBackend):
    """Holds last SparseTarget and exposes feedback placeholders.

    Replace `apply()` internals with:
      - Unitree DDS low-level, or
      - HuMiT / MOSAIC ONNX policy consuming target.keypoints / extremities
    """

    def __init__(self, mode: str = "upper_body") -> None:
        self.mode = mode  # upper_body | whole_body
        self._pose: Dict[str, np.ndarray] = {
            EXTREMITY_LEFT_EE: identity_pose(),
            EXTREMITY_RIGHT_EE: identity_pose(),
            EXTREMITY_HEAD: identity_pose(),
            EXTREMITY_PELVIS: identity_pose(),
            EXTREMITY_LEFT_FOOT: identity_pose(),
            EXTREMITY_RIGHT_FOOT: identity_pose(),
        }
        self._last: Optional[SparseTarget] = None
        self.connected = False

    def connect(self) -> None:
        # Future: channel init / onnx load
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def get_feedback(self) -> Dict[str, np.ndarray]:
        return {k: v.copy() for k, v in self._pose.items()}

    def apply(self, target: SparseTarget) -> BackendState:
        if not self.connected:
            raise RuntimeError("G1BackendStub not connected")
        self._last = target
        for name, pose in target.extremities.items():
            if target.is_active(name):
                self._pose[name] = pose.copy()
        # TODO: map SparseTarget → G1 joint command / policy observation
        return BackendState(
            extremities=self.get_feedback(),
            left_gripper=target.left_gripper,
            right_gripper=target.right_gripper,
            extra={"g1_mode": 1.0 if self.mode == "whole_body" else 0.0},
        )

    @property
    def last_target(self) -> Optional[SparseTarget]:
        return self._last
