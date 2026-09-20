"""抓取几何：指垫偏移、朝下姿态、半合开度（无场景依赖，避免循环 import）。"""

from __future__ import annotations

import numpy as np

from teleop_birth.teleop.schema import matrix_to_pose, pose_to_matrix

# Robotiq 2F-85：硅胶指垫中心相对 g_pinch 的工具系偏移（张开时实测 ~ -0.033）
DEFAULT_PAD_OFFSET_XYZ = (0.0, 0.0, -0.033)


def top_down_quat_xyzw(*, pad_axis_xy: np.ndarray | None = None) -> np.ndarray:
    """工具系：+Z 朝下，+Y 为夹爪开合方向（水平）。"""
    z = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    if pad_axis_xy is None:
        y = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        y = np.asarray(pad_axis_xy, dtype=np.float64).reshape(3)
        y[2] = 0.0
        n = float(np.linalg.norm(y))
        y = y / n if n > 1e-8 else np.array([1.0, 0.0, 0.0], dtype=np.float64)
    x = np.cross(y, z)
    x /= max(float(np.linalg.norm(x)), 1e-8)
    y = np.cross(z, x)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.column_stack([x, y, z])
    return matrix_to_pose(T)[3:].copy()


def pinch_pose_for_object(
    object_pos: np.ndarray,
    *,
    quat_xyzw: np.ndarray,
    grasp_offset_xyz: np.ndarray | list[float] = DEFAULT_PAD_OFFSET_XYZ,
) -> np.ndarray:
    """使指垫中心落在物体中心的 TCP 位姿。"""
    offset = np.asarray(grasp_offset_xyz, dtype=np.float64).reshape(3)
    R = pose_to_matrix(
        np.array(
            [
                object_pos[0],
                object_pos[1],
                object_pos[2],
                quat_xyzw[0],
                quat_xyzw[1],
                quat_xyzw[2],
                quat_xyzw[3],
            ],
            dtype=np.float64,
        )
    )[:3, :3]
    pinch = np.asarray(object_pos, dtype=np.float64).reshape(3) - R @ offset
    return np.array(
        [pinch[0], pinch[1], pinch[2], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]],
        dtype=np.float64,
    )


def gripper_close_for_width(width_m: float) -> float:
    """按物体宽度估计 2F-85 闭合指令（0 开…1 合死）。明显小于宽度以产生夹紧力。"""
    # 目标指缝约物体宽度的 55%，形成足够法向力再靠摩擦抬起
    target_gap = max(0.012, float(width_m) * 0.55)
    gap_open, gap_closed = 0.0984, 0.0149
    alpha = (gap_open - target_gap) / (gap_open - gap_closed)
    return float(np.clip(alpha, 0.70, 0.96))
