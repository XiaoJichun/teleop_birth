"""语义轴向 → 轴对齐矩阵 R_a。

OpenXR：+X 右、+Y 上、−Z 前。
目标房间系（常见 MuJoCo 桌面）：+X 右、+Y 前/桌深、+Z 上。

R_a 满足 v_room = R_a @ v_XR，只换轴名，不平移原点。
列含义：第 1 列 = XR +X 去向，第 2 列 = XR +Y 去向，第 3 列 = XR +Z 去向。
"""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np

__all__ = [
    "AXIS_ALIASES",
    "parse_axis",
    "build_ra",
    "build_ra_from_dict",
    "validate_ra",
    "openxr_semantic_basis",
]

# 语义名 → 单位向量（目标坐标系下）
AXIS_ALIASES: dict[str, tuple[float, float, float]] = {
    "+x": (1.0, 0.0, 0.0),
    "x": (1.0, 0.0, 0.0),
    "right": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "left": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "forward": (0.0, 1.0, 0.0),
    "front": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "back": (0.0, -1.0, 0.0),
    "backward": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "z": (0.0, 0.0, 1.0),
    "up": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
    "down": (0.0, 0.0, -1.0),
}


def parse_axis(spec: str | Sequence[float]) -> np.ndarray:
    """把 '+Z' / 'up' / [0,0,1] 解析成单位向量。"""
    if isinstance(spec, str):
        key = spec.strip().lower().replace(" ", "")
        if key not in AXIS_ALIASES:
            raise ValueError(f"未知轴向 {spec!r}，可选 {sorted(AXIS_ALIASES)}")
        v = np.asarray(AXIS_ALIASES[key], dtype=np.float64)
    else:
        v = np.asarray(spec, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError(f"轴向向量接近零: {spec}")
    return v / n


def openxr_semantic_basis() -> dict[str, np.ndarray]:
    """头显跟踪空间里的语义方向（右 / 上 / 前）。"""
    return {
        "right": np.array([1.0, 0.0, 0.0], dtype=np.float64),
        "up": np.array([0.0, 1.0, 0.0], dtype=np.float64),
        "forward": np.array([0.0, 0.0, -1.0], dtype=np.float64),
    }


def build_ra(
    *,
    xr_right_to: str | Sequence[float],
    xr_up_to: str | Sequence[float],
    xr_forward_to: str | Sequence[float],
    atol: float = 1e-6,
) -> np.ndarray:
    """按「映射后需求」生成 R_a。

    参数是 OpenXR 语义方向落到目标房间系的去向：
      xr_right_to   ← XR +X 应变成房间的哪根轴
      xr_up_to      ← XR +Y
      xr_forward_to ← XR 前向 (−Z) 应变成房间的哪根轴

    因此：
      R_a @ e_x = right_to
      R_a @ e_y = up_to
      R_a @ (−e_z) = forward_to  ⇒  R_a 第 3 列 = −forward_to
    """
    c0 = parse_axis(xr_right_to)
    c1 = parse_axis(xr_up_to)
    fwd = parse_axis(xr_forward_to)
    c2 = -fwd
    R = np.column_stack([c0, c1, c2])
    validate_ra(R, atol=atol)
    return R


def build_ra_from_dict(mapping: Mapping[str, str | Sequence[float]], **kwargs) -> np.ndarray:
    """从 dict 生成 R_a。键：right/up/forward（或 xr_right/xr_up/xr_forward）。"""
    def _get(*keys: str) -> str | Sequence[float]:
        for k in keys:
            if k in mapping:
                return mapping[k]
        raise KeyError(f"缺少轴向映射键，需要其一: {keys}")

    return build_ra(
        xr_right_to=_get("right", "xr_right", "xr_right_to"),
        xr_up_to=_get("up", "xr_up", "xr_up_to"),
        xr_forward_to=_get("forward", "xr_forward", "xr_forward_to"),
        **kwargs,
    )


def validate_ra(R: np.ndarray, *, atol: float = 1e-6) -> None:
    """要求近似正交且 det=+1（右手系）。"""
    M = np.asarray(R, dtype=np.float64).reshape(3, 3)
    if not np.allclose(M @ M.T, np.eye(3), atol=atol):
        raise ValueError(f"R_a 不正交:\n{M}")
    det = float(np.linalg.det(M))
    if abs(det - 1.0) > atol:
        raise ValueError(f"R_a 行列式应为 +1（右手系），得到 {det}")
