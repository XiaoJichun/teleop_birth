"""Clutch, filtering, and latency helpers shared by all embodiments."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional, Tuple

import numpy as np

from birth_teleop.schema import (
    invert_pose,
    matrix_to_pose,
    multiply_poses,
    pose_to_matrix,
)


@dataclass
class ClutchState:
    engaged: bool = False
    press_t0: Optional[float] = None
    xr_ref: Optional[np.ndarray] = None
    robot_ref: Optional[np.ndarray] = None


class RelativeClutch:
    """Hold-to-follow relative teleop.

    Modes (YAML clutch_mode):
      - long_press_grip: must hold grip >= hold_s then keep holding to teleop (default for Pico)
      - grip: engage while grip >= threshold (no long-press delay)
      - grip_or_trigger: grip or trigger
      - always: always engaged (debug)

    position_only: map translation only; keep EE orientation at engage.
    """

    def __init__(
        self,
        grip_threshold: float,
        mode: str = "long_press_grip",
        position_only: bool = True,
        hold_s: float = 0.35,
    ) -> None:
        self.grip_threshold = float(grip_threshold)
        self.mode = str(mode).lower()
        self.position_only = bool(position_only)
        self.hold_s = float(max(0.0, hold_s))
        self._states: Dict[str, ClutchState] = {}

    def reset(self, name: str) -> None:
        self._states[name] = ClutchState()

    def _raw_down(self, grip: float, trigger: float = 0.0) -> bool:
        if self.mode in ("always", "on", "continuous"):
            return True
        if self.mode in ("grip_or_trigger", "any"):
            return max(grip, trigger) >= self.grip_threshold
        # grip / long_press_grip
        return grip >= self.grip_threshold

    def _want(self, name: str, grip: float, trigger: float = 0.0) -> bool:
        st = self._states.setdefault(name, ClutchState())
        down = self._raw_down(grip, trigger)
        now = time.perf_counter()
        if self.mode in ("always", "on", "continuous"):
            return True
        if not down:
            st.press_t0 = None
            return False
        if self.mode in ("long_press_grip", "long_press", "hold"):
            if st.press_t0 is None:
                st.press_t0 = now
            return (now - st.press_t0) >= self.hold_s
        # immediate grip / grip_or_trigger
        return True

    def update(
        self,
        name: str,
        grip: float,
        xr_pose: np.ndarray,
        robot_pose: np.ndarray,
        trigger: float = 0.0,
    ) -> Tuple[bool, Optional[np.ndarray]]:
        st = self._states.setdefault(name, ClutchState())
        want = self._want(name, grip, trigger)
        if want and not st.engaged:
            st.engaged = True
            st.xr_ref = xr_pose.copy()
            st.robot_ref = robot_pose.copy()
        if not want:
            st.engaged = False
            st.xr_ref = None
            st.robot_ref = None
            return False, None
        assert st.xr_ref is not None and st.robot_ref is not None
        if self.position_only:
            target = st.robot_ref.copy()
            target[:3] = st.robot_ref[:3] + (xr_pose[:3] - st.xr_ref[:3])
            return True, target
        delta = multiply_poses(invert_pose(st.xr_ref), xr_pose)
        target = multiply_poses(st.robot_ref, delta)
        return True, target


class PoseLowPass:
    """Exponential low-pass on translation; slerp-like blend on quaternion."""

    def __init__(self, alpha: float) -> None:
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self._prev: Dict[str, np.ndarray] = {}

    def reset(self, name: Optional[str] = None) -> None:
        if name is None:
            self._prev.clear()
        else:
            self._prev.pop(name, None)

    def filter(self, name: str, pose: np.ndarray) -> np.ndarray:
        prev = self._prev.get(name)
        if prev is None:
            self._prev[name] = pose.copy()
            return pose.copy()
        a = self.alpha
        out = prev.copy()
        out[:3] = (1 - a) * prev[:3] + a * pose[:3]
        # Naive quat blend + renormalize (good enough for small steps)
        q = (1 - a) * prev[3:] + a * pose[3:]
        if np.dot(prev[3:], pose[3:]) < 0:
            q = (1 - a) * prev[3:] - a * pose[3:]
        n = np.linalg.norm(q)
        out[3:] = q / max(n, 1e-12)
        self._prev[name] = out.copy()
        return out


@dataclass
class LatencySample:
    stamp_ns: int
    loop_ms: float
    xr_to_cmd_ms: float


class LatencyProbe:
    """Rolling latency stats for the control loop (paper measurement hook)."""

    def __init__(self, window: int) -> None:
        self._loop_ms: Deque[float] = deque(maxlen=window)
        self._xr_to_cmd_ms: Deque[float] = deque(maxlen=window)
        self._last_loop_t = time.perf_counter()

    def tick(self, xr_timestamp_ns: int) -> LatencySample:
        now = time.perf_counter()
        loop_ms = (now - self._last_loop_t) * 1e3
        self._last_loop_t = now
        now_ns = time.time_ns()
        xr_to_cmd_ms = max(0.0, (now_ns - xr_timestamp_ns) / 1e6)
        self._loop_ms.append(loop_ms)
        self._xr_to_cmd_ms.append(xr_to_cmd_ms)
        return LatencySample(stamp_ns=now_ns, loop_ms=loop_ms, xr_to_cmd_ms=xr_to_cmd_ms)

    def summary(self) -> Dict[str, float]:
        def _stats(xs: Deque[float]) -> Dict[str, float]:
            if not xs:
                return {"mean": 0.0, "p50": 0.0, "p95": 0.0}
            arr = np.asarray(xs, dtype=np.float64)
            return {
                "mean": float(arr.mean()),
                "p50": float(np.percentile(arr, 50)),
                "p95": float(np.percentile(arr, 95)),
            }

        loop = _stats(self._loop_ms)
        xr = _stats(self._xr_to_cmd_ms)
        return {
            "loop_ms_mean": loop["mean"],
            "loop_ms_p50": loop["p50"],
            "loop_ms_p95": loop["p95"],
            "xr_to_cmd_ms_mean": xr["mean"],
            "xr_to_cmd_ms_p50": xr["p50"],
            "xr_to_cmd_ms_p95": xr["p95"],
        }


def estimate_pose_velocity(
    prev: Optional[np.ndarray],
    curr: np.ndarray,
    dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (linear_vel xyz, angular_vel approx) for ExtremControl-style feedforward."""
    if prev is None or dt <= 1e-6:
        return np.zeros(3), np.zeros(3)
    lin = (curr[:3] - prev[:3]) / dt
    # Small-angle angular velocity from relative rotation
    T_rel = np.linalg.inv(pose_to_matrix(prev)) @ pose_to_matrix(curr)
    R = T_rel[:3, :3]
    # vee(log R) approx for small dt
    w = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ]
    ) / (2.0 * dt)
    return lin, w
