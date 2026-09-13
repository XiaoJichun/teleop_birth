"""脚本化末端笛卡尔运动（走现有 IK 后端）。"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np

from teleop_birth.embodiment.base import BackendState, RobotBackend
from teleop_birth.schema import (
    EXTREMITY_LEFT_EE,
    EXTREMITY_RIGHT_EE,
    SparseTarget,
    identity_pose,
)


class ScriptedEEMotion:
    def __init__(
        self,
        backend: RobotBackend,
        *,
        arm: str = "right",
        steps_per_segment: int = 40,
        hold_steps: int = 0,
    ) -> None:
        self.backend = backend
        self.arm = arm.lower()
        self.steps_per_segment = max(2, int(steps_per_segment))
        # 默认 0：段与段之间不停顿，轨迹更流畅
        self.hold_steps = max(0, int(hold_steps))
        self._key = EXTREMITY_RIGHT_EE if self.arm.startswith("r") else EXTREMITY_LEFT_EE

    def _current_ee(self) -> np.ndarray:
        fb = self.backend.get_feedback()
        return np.asarray(fb[self._key], dtype=np.float64).copy()

    def _apply(
        self,
        pose: np.ndarray,
        gripper: float,
        on_step: Optional[Callable[[BackendState, SparseTarget], None]] = None,
    ) -> BackendState:
        other = EXTREMITY_LEFT_EE if self._key == EXTREMITY_RIGHT_EE else EXTREMITY_RIGHT_EE
        fb = self.backend.get_feedback()
        extremities = {
            self._key: np.asarray(pose, dtype=np.float64),
            other: np.asarray(fb.get(other, identity_pose()), dtype=np.float64),
        }
        left_g = float(gripper) if self._key == EXTREMITY_LEFT_EE else 0.0
        right_g = float(gripper) if self._key == EXTREMITY_RIGHT_EE else 0.0
        target = SparseTarget(
            timestamp_ns=0,
            extremities=extremities,
            active={self._key: True, other: False},
            left_gripper=left_g,
            right_gripper=right_g,
        )
        state = self.backend.apply(target)
        if on_step is not None:
            on_step(state, target)
        return state

    def go_to(
        self,
        goal: np.ndarray,
        gripper: float,
        *,
        on_step: Optional[Callable[[BackendState, SparseTarget], None]] = None,
        n_steps: int | None = None,
        gripper_start: float | None = None,
    ) -> None:
        start = self._current_ee()
        goal = np.asarray(goal, dtype=np.float64)
        n = int(n_steps or self.steps_per_segment)
        g0 = float(gripper if gripper_start is None else gripper_start)
        g1 = float(gripper)
        for i in range(1, n + 1):
            a = i / n
            # 平滑步进（ease-in-out）减少段首段尾顿挫感
            s = a * a * (3.0 - 2.0 * a)
            pose = start.copy()
            pose[:3] = (1.0 - s) * start[:3] + s * goal[:3]
            pose[3:] = goal[3:]
            g = (1.0 - s) * g0 + s * g1
            self._apply(pose, g, on_step=on_step)

    def hold(
        self,
        pose: np.ndarray,
        gripper: float,
        *,
        on_step: Optional[Callable[[BackendState, SparseTarget], None]] = None,
        n_steps: int | None = None,
    ) -> None:
        n = int(self.hold_steps if n_steps is None else n_steps)
        for _ in range(max(0, n)):
            self._apply(pose, gripper, on_step=on_step)

    def execute_waypoints(
        self,
        poses: list[np.ndarray],
        grippers: list[float],
        *,
        on_step: Optional[Callable[[BackendState, SparseTarget], None]] = None,
        on_segment: Optional[Callable[[int, str], None]] = None,
        labels: list[str] | None = None,
        steps_per_segment: list[int] | None = None,
    ) -> None:
        labels = labels or [f"seg_{i}" for i in range(len(poses))]
        prev_g = float(grippers[0]) if grippers else 0.0
        for i, (pose, g, lab) in enumerate(zip(poses, grippers, labels)):
            if on_segment is not None:
                on_segment(i, lab)
            n = None
            if steps_per_segment is not None and i < len(steps_per_segment):
                n = int(steps_per_segment[i])
            self.go_to(pose, g, on_step=on_step, n_steps=n, gripper_start=prev_g)
            if self.hold_steps > 0:
                self.hold(pose, g, on_step=on_step)
            prev_g = float(g)
