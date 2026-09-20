"""Clutch, filtering, and latency helpers shared by all embodiments."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional, Sequence, Tuple

import numpy as np

from teleop_birth.teleop.schema import matrix_to_pose, pose_to_matrix


@dataclass
class ClutchState:
    engaged: bool = False
    press_t0: Optional[float] = None
    xr_ref: Optional[np.ndarray] = None
    robot_ref: Optional[np.ndarray] = None
    last_xr: Optional[np.ndarray] = None
    still_since: Optional[float] = None
    # 本帧是否因为「位姿卡住后跳变」而重新锚定
    reanchored: bool = False


def openxr_grip_to_tcp_body() -> np.ndarray:
    """OpenXR grip 体轴 → 朝下 TCP 的体轴（右手系）。委托 frame.orientation。"""
    from teleop_birth.teleop.frame.orientation import default_grip_to_tcp_M

    return default_grip_to_tcp_M()


def _axis_angle(R: np.ndarray) -> np.ndarray:
    from teleop_birth.teleop.frame.orientation import axis_angle

    return axis_angle(R)


def _rot_from_aa(w: np.ndarray) -> np.ndarray:
    from teleop_birth.teleop.frame.orientation import rot_from_aa

    return rot_from_aa(w)


def openxr_yup_to_mujoco_zup() -> np.ndarray:
    """OpenXR (Y-up, -Z forward) → MuJoCo (Z-up, +Y forward/table).

    XR right(+X)→MJ +X, XR up(+Y)→MJ +Z, XR forward(-Z)→MJ +Y
    实现委托给 teleop.frame.build_ra（与 XrAlignPlugin 同一套）。
    """
    from teleop_birth.teleop.frame.axes import build_ra

    return build_ra(xr_right_to="+x", xr_up_to="+z", xr_forward_to="+y")


def resolve_xr_align(
    name: str | None,
    matrix: list | None,
    *,
    profile_id: str | None = None,
    mapping: dict | None = None,
) -> np.ndarray:
    """解析 R_a：矩阵 > 语义 mapping > arm profile > 具名预设。"""
    from teleop_birth.teleop.frame.profiles import resolve_align_matrix

    return resolve_align_matrix(
        name,
        matrix,
        profile_id=profile_id,
        mapping=mapping,
    )


class RelativeClutch:
    """Hold-to-follow relative teleop.

    平移始终在对齐后的房间系叠加。姿态：房间竖直转跟手柄的房间 yaw，
    拧腕/侧倾贴到夹爪体轴。position_only 时锁住按下瞬间的 TCP 姿态。
    """

    def __init__(
        self,
        grip_threshold: float,
        mode: str = "long_press_grip",
        position_only: bool = True,
        hold_s: float = 0.35,
        xr_align: Optional[np.ndarray] = None,
        # 可选：手柄本地固定旋转（xyzw），再参与姿态跟踪，用于 grip→aim 轴修正
        controller_quat_xyzw: Optional[Sequence[float]] = None,
        # 握把→夹爪体轴 M（尖端朝下 / 朝前等剖面）；None = tip_down 默认
        grip_to_tcp_M: Optional[np.ndarray] = None,
        # SDK 位姿卡住后再跳变：不把这一跳当成人手移动（150Hz 下 >4cm/帧是追踪跳变）
        pose_still_s: float = 0.4,
        # 0 = 不丢弃跳变。丢掉「卡住后的大跳」会让刚开始的大位移完全不带动机械臂。
        reanchor_jump_m: float = 0.0,
        still_eps_m: float = 0.002,
    ) -> None:
        self.grip_threshold = float(grip_threshold)
        self.mode = str(mode).lower()
        self.position_only = bool(position_only)
        self.hold_s = float(max(0.0, hold_s))
        self.pose_still_s = float(max(0.0, pose_still_s))
        self.reanchor_jump_m = float(max(0.0, reanchor_jump_m))
        self.still_eps_m = float(max(0.0, still_eps_m))
        self.R_align = (
            np.asarray(xr_align, dtype=np.float64).reshape(3, 3)
            if xr_align is not None
            else openxr_yup_to_mujoco_zup()
        )
        if controller_quat_xyzw is None:
            self.R_ctrl = np.eye(3, dtype=np.float64)
        else:
            from teleop_birth.teleop.frame.orientation import quat_xyzw_to_R

            self.R_ctrl = quat_xyzw_to_R(controller_quat_xyzw)
        from teleop_birth.teleop.frame.orientation import WristOrientationMap

        self._orient = WristOrientationMap(
            R_a=self.R_align,
            R_ctrl=self.R_ctrl,
            M=grip_to_tcp_M,
            position_only=self.position_only,
        )
        self._states: Dict[str, ClutchState] = {}

    def reset(self, name: str) -> None:
        self._states[name] = ClutchState()

    def _raw_down(self, grip: float, trigger: float = 0.0) -> bool:
        if self.mode == "always":
            return True
        if self.mode == "grip_or_trigger":
            return max(grip, trigger) >= self.grip_threshold
        return grip >= self.grip_threshold

    def _want(self, name: str, grip: float, trigger: float = 0.0) -> bool:
        st = self._states.setdefault(name, ClutchState())
        down = self._raw_down(grip, trigger)
        now = time.perf_counter()
        if self.mode == "always":
            return True
        if not down:
            st.press_t0 = None
            return False
        if self.mode == "long_press_grip":
            if st.press_t0 is None:
                st.press_t0 = now
            return (now - st.press_t0) >= self.hold_s
        return True

    def _map(self, st: ClutchState, xr_now: np.ndarray, scale: float) -> np.ndarray:
        """房间系平移 + 腕部姿态（竖直转走房间 yaw，拧腕贴夹爪体轴）。"""
        assert st.xr_ref is not None and st.robot_ref is not None
        R_a = self.R_align
        dp_mj = scale * (R_a @ (xr_now[:3] - st.xr_ref[:3]))

        T_r0 = pose_to_matrix(st.robot_ref)
        p_t = T_r0[:3, 3] + dp_mj
        R_r0 = T_r0[:3, :3]

        if self.position_only:
            T = np.eye(4)
            T[:3, :3] = R_r0
            T[:3, 3] = p_t
            return matrix_to_pose(T)

        # 姿态：委托 WristOrientationMap（与坐标系插件 / 文档第 2 步同一套）
        self._orient.position_only = False
        self._orient.R_a = self.R_align
        self._orient.R_ctrl = self.R_ctrl
        ori = self._orient.map_from_poses(
            xr_ref_pose=st.xr_ref,
            xr_now_pose=xr_now,
            robot_ref_pose=st.robot_ref,
        )
        T = np.eye(4)
        T[:3, :3] = ori.R_t
        T[:3, 3] = p_t
        return matrix_to_pose(T)

    def _latch(self, st: ClutchState, xr_pose: np.ndarray, robot_pose: np.ndarray) -> None:
        st.engaged = True
        st.xr_ref = xr_pose.copy()
        st.robot_ref = robot_pose.copy()
        st.last_xr = xr_pose.copy()
        st.still_since = time.perf_counter()

    def _maybe_reanchor(self, st: ClutchState, xr_pose: np.ndarray, robot_pose: np.ndarray) -> None:
        """位姿长时间不变后再跳：丢掉这一跳，从当前帧重新锚定。

        相对模式只跟 SDK 位姿增量。Grip 已经按下时，若 SDK 仍在重复同一帧，
        增量是 0，机械臂不会动。追踪开始时常见一次大跳，不能当成手的位移。
        连续小幅移动不重置，避免把正常遥操吃掉。
        """
        st.reanchored = False
        if st.last_xr is None or self.reanchor_jump_m <= 0:
            st.last_xr = xr_pose.copy()
            return
        dp = float(np.linalg.norm(xr_pose[:3] - st.last_xr[:3]))
        now = time.perf_counter()
        if dp <= self.still_eps_m:
            if st.still_since is None:
                st.still_since = now
            st.last_xr = xr_pose.copy()
            return
        frozen = st.still_since is not None and (now - st.still_since) >= self.pose_still_s
        st.still_since = None
        st.last_xr = xr_pose.copy()
        if not frozen or dp < self.reanchor_jump_m:
            return
        self._latch(st, xr_pose, robot_pose)
        st.reanchored = True

    def was_reanchored(self, name: str) -> bool:
        st = self._states.get(name)
        return bool(st is not None and st.reanchored)

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
        st.reanchored = False
        want = self._want(name, grip, trigger)
        if want and not st.engaged:
            self._latch(st, xr_pose, robot_pose)
        if not want:
            st.engaged = False
            st.xr_ref = None
            st.robot_ref = None
            st.last_xr = None
            st.still_since = None
            return False, None
        assert st.xr_ref is not None and st.robot_ref is not None
        self._maybe_reanchor(st, xr_pose, robot_pose)
        return True, self._map(st, xr_pose, float(scale))


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
