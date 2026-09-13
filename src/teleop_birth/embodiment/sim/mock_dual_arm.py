"""Mock dual-arm embodiment (in-memory) — protocol bring-up without physics."""

from __future__ import annotations

from typing import Dict, List

import numpy as np

from teleop_birth.embodiment.base import BackendState, RobotBackend
from teleop_birth.schema import (
    EXTREMITY_HEAD,
    EXTREMITY_LEFT_EE,
    EXTREMITY_RIGHT_EE,
    SparseTarget,
)


class MockDualArmBackend(RobotBackend):
    def __init__(
        self,
        left0: np.ndarray,
        right0: np.ndarray,
        head0: np.ndarray,
    ) -> None:
        self._home = {
            EXTREMITY_LEFT_EE: np.asarray(left0, dtype=np.float64).copy(),
            EXTREMITY_RIGHT_EE: np.asarray(right0, dtype=np.float64).copy(),
            EXTREMITY_HEAD: np.asarray(head0, dtype=np.float64).copy(),
        }
        self._pose = {k: v.copy() for k, v in self._home.items()}
        self._left_gripper = 0.0
        self._right_gripper = 0.0
        self._history: List[BackendState] = []

    def connect(self) -> None:
        return

    def disconnect(self) -> None:
        return

    def get_feedback(self) -> Dict[str, np.ndarray]:
        return {k: v.copy() for k, v in self._pose.items()}

    def apply(self, target: SparseTarget) -> BackendState:
        meta = target.meta or {}
        if bool(meta.get("home_left")):
            self._pose[EXTREMITY_LEFT_EE] = self._home[EXTREMITY_LEFT_EE].copy()
        if bool(meta.get("home_right")):
            self._pose[EXTREMITY_RIGHT_EE] = self._home[EXTREMITY_RIGHT_EE].copy()
        for name in (EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, EXTREMITY_HEAD):
            if target.is_active(name) and name in target.extremities:
                self._pose[name] = target.extremities[name].copy()
        self._left_gripper = target.left_gripper
        self._right_gripper = target.right_gripper
        state = BackendState(
            extremities={k: v.copy() for k, v in self._pose.items()},
            left_gripper=self._left_gripper,
            right_gripper=self._right_gripper,
        )
        self._history.append(state)
        return state

    @property
    def history(self) -> List[BackendState]:
        return self._history
