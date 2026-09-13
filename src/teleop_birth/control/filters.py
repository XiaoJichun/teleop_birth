"""Clutch, filtering, and latency helpers shared by all embodiments."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional, Sequence, Tuple

import numpy as np

from teleop_birth.schema import (
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
    # track 姿态：R_cmd = R_ori_offset @ R_a @ R_xr @ R_a.T，接合时锁定使 cmd=robot_ref
    R_ori_offset: Optional[np.ndarray] = None


def openxr_yup_to_mujoco_zup() -> np.ndarray:
    """OpenXR (Y-up, -Z forward) → MuJoCo (Z-up, +Y forward/table).

    XR right(+X)→MJ +X, XR up(+Y)→MJ +Z, XR forward(-Z)→MJ +Y
    """
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )


def resolve_xr_align(name: str | None, matrix: list | None) -> np.ndarray:
    if matrix is not None:
        R = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        return R
    key = (name or "openxr_yup_to_zup").lower()
    if key in ("identity", "none", "raw"):
        return np.eye(3, dtype=np.float64)
    if key in ("openxr_yup_to_zup", "openxr", "pico", "default"):
        return openxr_yup_to_mujoco_zup()
    raise ValueError(f"未知 xr_align 预设: {name}")


class RelativeClutch:
    """Hold-to-follow relative teleop.

    delta_frame:
      - world: 平移增量在对齐后的世界系叠加（推荐）
      - local: 旧局部系相对（易拧反）

    ori_mode（仅 position_only=false）:
      - world_delta: 把手柄的世界系相对旋转左乘到机器人参考上（默认；房间拧腕方向一致）
      - track: 末端跟踪「轴对齐后的手柄姿态」，接合锁偏移；TCP 与手柄轴不对齐时易觉「拧反」
    """

    def __init__(
        self,
        grip_threshold: float,
        mode: str = "long_press_grip",
        position_only: bool = True,
        hold_s: float = 0.35,
        delta_frame: str = "world",
        xr_align: Optional[np.ndarray] = None,
        ori_mode: str = "world_delta",
        # 可选：手柄本地固定旋转（xyzw），再参与姿态跟踪，用于 grip→aim 轴修正
        controller_quat_xyzw: Optional[Sequence[float]] = None,
    ) -> None:
        self.grip_threshold = float(grip_threshold)
        self.mode = str(mode).lower()
        self.position_only = bool(position_only)
        self.hold_s = float(max(0.0, hold_s))
        self.delta_frame = str(delta_frame).lower()
        self.ori_mode = str(ori_mode).lower()
        self.R_align = (
            np.asarray(xr_align, dtype=np.float64).reshape(3, 3)
            if xr_align is not None
            else openxr_yup_to_mujoco_zup()
        )
        if controller_quat_xyzw is None:
            self.R_ctrl = np.eye(3, dtype=np.float64)
        else:
            q = np.asarray(controller_quat_xyzw, dtype=np.float64).reshape(4)
            # xyzw → 临时 pose
            p = np.zeros(7, dtype=np.float64)
            p[3:] = q
            self.R_ctrl = pose_to_matrix(p)[:3, :3].copy()
        self._states: Dict[str, ClutchState] = {}

    def reset(self, name: str) -> None:
        self._states[name] = ClutchState()

    def _R_xr_mj(self, xr_pose: np.ndarray) -> np.ndarray:
        """手柄姿态：可选本地修正后，再改基到 MuJoCo 世界系。"""
        R_x = pose_to_matrix(xr_pose)[:3, :3] @ self.R_ctrl
        R_a = self.R_align
        return R_a @ R_x @ R_a.T

    def _raw_down(self, grip: float, trigger: float = 0.0) -> bool:
        if self.mode in ("always", "on", "continuous"):
            return True
        if self.mode in ("grip_or_trigger", "any"):
            return max(grip, trigger) >= self.grip_threshold
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
        return True

    def _map_world(self, st: ClutchState, xr_now: np.ndarray, scale: float) -> np.ndarray:
        """World-frame position Δ + orientation (track or world_delta)."""
        assert st.xr_ref is not None and st.robot_ref is not None
        R_a = self.R_align
        p0 = st.xr_ref[:3]
        p1 = xr_now[:3]
        dp_mj = scale * (R_a @ (p1 - p0))

        T_r0 = pose_to_matrix(st.robot_ref)
        p_t = T_r0[:3, 3] + dp_mj
        R_r0 = T_r0[:3, :3]

        if self.position_only:
            T = np.eye(4)
            T[:3, :3] = R_r0
            T[:3, 3] = p_t
            return matrix_to_pose(T)

        if self.ori_mode in ("track", "follow", "offset"):
            # R_cmd = R_offset @ R_xr_mj，接合时 R_offset 使 cmd = R_r0
            assert st.R_ori_offset is not None
            R_t = st.R_ori_offset @ self._R_xr_mj(xr_now)
        else:
            # world_delta：空间相对旋转左乘
            R_x0 = pose_to_matrix(st.xr_ref)[:3, :3] @ self.R_ctrl
            R_x1 = pose_to_matrix(xr_now)[:3, :3] @ self.R_ctrl
            R_delta_xr = R_x1 @ R_x0.T
            R_delta_mj = R_a @ R_delta_xr @ R_a.T
            R_t = R_delta_mj @ R_r0

        T = np.eye(4)
        T[:3, :3] = R_t
        T[:3, 3] = p_t
        return matrix_to_pose(T)

    def _map_local(self, st: ClutchState, xr_now: np.ndarray, scale: float) -> np.ndarray:
        """旧局部系相对（不直观，保留兼容）。"""
        assert st.xr_ref is not None and st.robot_ref is not None
        xr_ref = st.xr_ref
        robot_ref = st.robot_ref
        if self.position_only:
            target = robot_ref.copy()
            dp = scale * (self.R_align @ (xr_now[:3] - xr_ref[:3]))
            target[:3] = robot_ref[:3] + dp
            return target
        if self.ori_mode in ("track", "follow", "offset"):
            # 平移仍用对齐世界 Δ；姿态用 track
            return self._map_world(st, xr_now, scale)
        delta = multiply_poses(invert_pose(xr_ref), xr_now)
        T = pose_to_matrix(delta)
        T[:3, 3] = scale * (self.R_align @ T[:3, 3])
        R_d = self.R_align @ T[:3, :3] @ self.R_align.T
        T[:3, :3] = R_d
        return multiply_poses(robot_ref, matrix_to_pose(T))

    def update(
        self,
        name: str,
        grip: float,
        xr_pose: np.ndarray,
        robot_pose: np.ndarray,
        trigger: float = 0.0,
        scale: float = 1.0,
    ) -> Tuple[bool, Optional[np.ndarray]]:
        st = self._states.setdefault(name, ClutchState())
        want = self._want(name, grip, trigger)
        if want and not st.engaged:
            st.engaged = True
            st.xr_ref = xr_pose.copy()
            st.robot_ref = robot_pose.copy()
            if not self.position_only:
                R_r0 = pose_to_matrix(robot_pose)[:3, :3]
                R_x0_mj = self._R_xr_mj(xr_pose)
                st.R_ori_offset = R_r0 @ R_x0_mj.T
            else:
                st.R_ori_offset = None
        if not want:
            st.engaged = False
            st.xr_ref = None
            st.robot_ref = None
            st.R_ori_offset = None
            return False, None
        assert st.xr_ref is not None and st.robot_ref is not None
        s = float(scale)
        if self.delta_frame in ("local", "body", "ee") and self.position_only:
            return True, self._map_local(st, xr_pose, s)
        if self.delta_frame in ("local", "body", "ee") and self.ori_mode not in (
            "track",
            "follow",
            "offset",
        ):
            return True, self._map_local(st, xr_pose, s)
        return True, self._map_world(st, xr_pose, s)


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
