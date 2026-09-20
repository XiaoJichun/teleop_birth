"""Arm-6A 抓取几何：TCP 沿工具 +X 接近（非 Robotiq 的 +Z）。"""

from __future__ import annotations

import numpy as np

from teleop_birth.robot.arm.artixon.ik_dls import matrix_to_pose, pose_to_matrix

# 指垫中心相对 tcp_pinch（tcp 已放在指缝中心时用 0）
DEFAULT_PAD_OFFSET_XYZ = (0.0, 0.0, 0.0)


def top_down_quat_xyzw(*, open_axis_world: np.ndarray | None = None) -> np.ndarray:
    """俯抓姿态：工具 +X 朝下，+Y 为夹爪开合（默认世界 +Y）。"""
    x = np.array([0.0, 0.0, -1.0], dtype=np.float64)  # approach down
    if open_axis_world is None:
        y = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    else:
        y = np.asarray(open_axis_world, dtype=np.float64).reshape(3)
        y[2] = 0.0
        n = float(np.linalg.norm(y))
        y = y / n if n > 1e-8 else np.array([0.0, 1.0, 0.0], dtype=np.float64)
    z = np.cross(x, y)
    z /= max(float(np.linalg.norm(z)), 1e-8)
    y = np.cross(z, x)
    y /= max(float(np.linalg.norm(y)), 1e-8)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.column_stack([x, y, z])
    return matrix_to_pose(T)[3:].copy()


def pinch_pose_for_object(
    object_pos: np.ndarray,
    *,
    quat_xyzw: np.ndarray,
    grasp_offset_xyz: np.ndarray | list[float] = DEFAULT_PAD_OFFSET_XYZ,
) -> np.ndarray:
    """使指垫中心对齐物体中心的 TCP 位姿。"""
    offset = np.asarray(grasp_offset_xyz, dtype=np.float64).reshape(3)
    T = pose_to_matrix(
        np.array(
            [
                float(object_pos[0]),
                float(object_pos[1]),
                float(object_pos[2]),
                float(quat_xyzw[0]),
                float(quat_xyzw[1]),
                float(quat_xyzw[2]),
                float(quat_xyzw[3]),
            ],
            dtype=np.float64,
        )
    )
    # ideal_obj = tcp + R @ offset  →  tcp = obj - R @ offset
    T[:3, 3] = np.asarray(object_pos, dtype=np.float64) - T[:3, :3] @ offset
    return matrix_to_pose(T)


def gripper_close_for_width(width_m: float, *, max_open_m: float = 0.0975, pad_skin_m: float = 0.022) -> float:
    """物体全宽 → 夹爪闭合量：薄垫停在盒外侧，不插进盒子。"""
    half = 0.5 * float(np.clip(width_m, 0.02, max_open_m))
    half_gap = float(np.clip(half + pad_skin_m, 0.018, 0.04875))
    open_q, close_q = 0.04875, 0.006
    g = (half_gap - open_q) / (close_q - open_q)
    return float(np.clip(g, 0.02, 0.55))
