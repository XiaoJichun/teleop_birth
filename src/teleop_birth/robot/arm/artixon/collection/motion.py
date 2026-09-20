"""双臂脚本运动。"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np

from teleop_birth.robot.arm.artixon.backend import DualArtixOnBackend


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    a = np.asarray(q0, dtype=np.float64).reshape(4)
    b = np.asarray(q1, dtype=np.float64).reshape(4)
    a = a / max(float(np.linalg.norm(a)), 1e-12)
    b = b / max(float(np.linalg.norm(b)), 1e-12)
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        out = a + t * (b - a)
        return out / max(float(np.linalg.norm(out)), 1e-12)
    theta0 = float(np.arccos(dot))
    theta = theta0 * t
    s0 = float(np.sin(theta0))
    w0 = float(np.sin(theta0 - theta) / s0)
    w1 = float(np.sin(theta) / s0)
    return w0 * a + w1 * b


class DualScriptedMotion:
    def __init__(self, backend: DualArtixOnBackend, *, steps_per_segment: int = 35) -> None:
        self.backend = backend
        self.steps_per_segment = max(2, int(steps_per_segment))

    def go_to(
        self,
        *,
        left: np.ndarray | None = None,
        right: np.ndarray | None = None,
        left_g: float | None = None,
        right_g: float | None = None,
        n_steps: int | None = None,
        on_step: Optional[Callable[[dict[str, np.ndarray], float, float], None]] = None,
        left_g0: float | None = None,
        right_g0: float | None = None,
    ) -> None:
        n = int(n_steps or self.steps_per_segment)
        l0 = self.backend.tcp_pose("left") if left is not None else None
        r0 = self.backend.tcp_pose("right") if right is not None else None
        lg0 = float(left_g0 if left_g0 is not None else (left_g or 0.0))
        rg0 = float(right_g0 if right_g0 is not None else (right_g or 0.0))
        lg1 = float(left_g if left_g is not None else lg0)
        rg1 = float(right_g if right_g is not None else rg0)
        for i in range(1, n + 1):
            a = i / n
            s = a * a * (3.0 - 2.0 * a)
            lp = None
            rp = None
            if left is not None and l0 is not None:
                lp = l0.copy()
                lp[:3] = (1 - s) * l0[:3] + s * left[:3]
                lp[3:] = _slerp(l0[3:], left[3:], s)
            if right is not None and r0 is not None:
                rp = r0.copy()
                rp[:3] = (1 - s) * r0[:3] + s * right[:3]
                rp[3:] = _slerp(r0[3:], right[3:], s)
            lg = (1 - s) * lg0 + s * lg1
            rg = (1 - s) * rg0 + s * rg1
            ees = self.backend.step_to_poses(
                left=lp, right=rp, left_gripper=lg, right_gripper=rg
            )
            if on_step is not None:
                on_step(ees, lg, rg)
