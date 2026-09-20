"""腕部姿态映射：房间 yaw + 夹爪体轴拧腕（第 2 步）。

统一符号（与 doc/遥操作全链路详解.md §5.3 一致）：

  R_x0, R_x1  — 按下时 / 现在的手柄旋转（已乘 R_ctrl）
  R_a         — OpenXR → 房间轴对齐
  R_r0        — 按下时 TCP 旋转
  R_Δ^MJ      — 手柄相对旋转改到房间系
  ψ, R_yaw    — 房间竖直转（绕 +Z）
  w, M        — 手柄体轴角增量；握把→TCP 体轴
  R_tool      — 拧腕/侧倾（右乘到夹爪）
  R_t         — 目标 TCP 旋转
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from teleop_birth.teleop.schema import pose_to_matrix

__all__ = [
    "WristOrientResult",
    "WristOrientationMap",
    "default_grip_to_tcp_M",
    "axis_angle",
    "rot_from_aa",
    "quat_xyzw_to_R",
]


def default_grip_to_tcp_M() -> np.ndarray:
    """默认 tip_down 剖面的 M（委托 grip_tcp，可 YAML / 插件覆盖）。"""
    from teleop_birth.teleop.frame.grip_tcp import default_grip_to_tcp_M as _default

    return _default()


def axis_angle(R: np.ndarray) -> np.ndarray:
    cos_a = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    ang = float(np.arccos(cos_a))
    if ang < 1e-8:
        return np.zeros(3, dtype=np.float64)
    axis = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=np.float64,
    )
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.zeros(3, dtype=np.float64)
    return axis * (ang / n)


def rot_from_aa(w: np.ndarray) -> np.ndarray:
    ang = float(np.linalg.norm(w))
    if ang < 1e-10:
        return np.eye(3, dtype=np.float64)
    axis = w / ang
    K = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3) + np.sin(ang) * K + (1.0 - np.cos(ang)) * (K @ K)


def quat_xyzw_to_R(quat_xyzw: Sequence[float] | None) -> np.ndarray:
    if quat_xyzw is None:
        return np.eye(3, dtype=np.float64)
    p = np.zeros(7, dtype=np.float64)
    p[3:] = np.asarray(quat_xyzw, dtype=np.float64).reshape(4)
    return pose_to_matrix(p)[:3, :3].copy()


@dataclass
class WristOrientResult:
    """一步步中间量，方便 UI / 文档对照。"""

    R_x0: np.ndarray
    R_x1: np.ndarray
    R_delta_mj: np.ndarray
    psi: float
    R_yaw: np.ndarray
    w: np.ndarray
    R_tool: np.ndarray
    R_t: np.ndarray
    R_r0: np.ndarray
    R_a: np.ndarray
    M: np.ndarray

    def explain_zh(self) -> list[str]:
        deg = float(np.degrees(self.psi))
        return [
            "第 2 步：姿态（与平移分开算）",
            f"  ① 手柄旋转（含 R_ctrl）：R_x0、R_x1",
            f"  ② 相对旋转改到房间系：R_Δ^MJ = R_a (R_x1 R_x0ᵀ) R_aᵀ",
            f"  ③ 竖直转角 ψ = atan2(...) = {self.psi:.4f} rad ({deg:.1f}°)",
            f"  ④ R_yaw = Rot_z(ψ) 左乘到按下时的 TCP",
            f"  ⑤ 手柄体轴增量 w（已清零绕「上」的 wy）= {np.array2string(self.w, precision=3)}",
            f"  ⑥ R_tool = Rot(M w) 右乘到夹爪体轴",
            "  ⑦ 合成：R_t = R_yaw @ R_r0 @ R_tool",
        ]


class WristOrientationMap:
    """手柄相对旋转 → TCP 目标旋转（房间 yaw + 夹爪体轴）。"""

    def __init__(
        self,
        *,
        R_a: Optional[np.ndarray] = None,
        R_ctrl: Optional[np.ndarray] = None,
        controller_quat_xyzw: Optional[Sequence[float]] = None,
        M: Optional[np.ndarray] = None,
        position_only: bool = False,
    ) -> None:
        if R_a is None:
            from teleop_birth.teleop.frame.axes import build_ra

            R_a = build_ra(xr_right_to="+x", xr_up_to="+z", xr_forward_to="+y")
        self.R_a = np.asarray(R_a, dtype=np.float64).reshape(3, 3)
        if R_ctrl is not None:
            self.R_ctrl = np.asarray(R_ctrl, dtype=np.float64).reshape(3, 3)
        else:
            self.R_ctrl = quat_xyzw_to_R(controller_quat_xyzw)
        self.M = (
            np.asarray(M, dtype=np.float64).reshape(3, 3)
            if M is not None
            else default_grip_to_tcp_M()
        )
        self.position_only = bool(position_only)

    def map(
        self,
        *,
        R_xr0: np.ndarray,
        R_xr1: np.ndarray,
        R_r0: np.ndarray,
    ) -> WristOrientResult:
        """从按下手柄旋转 / 当前手柄旋转 / 按下 TCP 旋转 → R_t。"""
        R_r0 = np.asarray(R_r0, dtype=np.float64).reshape(3, 3)
        if self.position_only:
            I = np.eye(3, dtype=np.float64)
            return WristOrientResult(
                R_x0=I,
                R_x1=I,
                R_delta_mj=I,
                psi=0.0,
                R_yaw=I,
                w=np.zeros(3),
                R_tool=I,
                R_t=R_r0.copy(),
                R_r0=R_r0,
                R_a=self.R_a,
                M=self.M,
            )

        R_x0 = np.asarray(R_xr0, dtype=np.float64).reshape(3, 3) @ self.R_ctrl
        R_x1 = np.asarray(R_xr1, dtype=np.float64).reshape(3, 3) @ self.R_ctrl
        R_a = self.R_a
        R_delta_mj = R_a @ (R_x1 @ R_x0.T) @ R_a.T
        psi = float(np.arctan2(R_delta_mj[1, 0], R_delta_mj[0, 0]))
        c, s = float(np.cos(psi)), float(np.sin(psi))
        R_yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        w = axis_angle(R_x0.T @ R_x1)
        w[1] = 0.0  # 绕手柄「上」已由 ψ 负责
        R_tool = rot_from_aa(self.M @ w)
        R_t = R_yaw @ R_r0 @ R_tool
        return WristOrientResult(
            R_x0=R_x0,
            R_x1=R_x1,
            R_delta_mj=R_delta_mj,
            psi=psi,
            R_yaw=R_yaw,
            w=w,
            R_tool=R_tool,
            R_t=R_t,
            R_r0=R_r0,
            R_a=R_a,
            M=self.M,
        )

    def map_from_poses(
        self,
        *,
        xr_ref_pose: np.ndarray,
        xr_now_pose: np.ndarray,
        robot_ref_pose: np.ndarray,
    ) -> WristOrientResult:
        R_xr0 = pose_to_matrix(xr_ref_pose)[:3, :3]
        R_xr1 = pose_to_matrix(xr_now_pose)[:3, :3]
        R_r0 = pose_to_matrix(robot_ref_pose)[:3, :3]
        return self.map(R_xr0=R_xr0, R_xr1=R_xr1, R_r0=R_r0)
