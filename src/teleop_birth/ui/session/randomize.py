"""随机化区域：相对机器人 base 的平面/立体盒子（防碰撞采样）。"""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


@dataclass
class RandomizeRegion:
    """相对机器人 base 的放置盒。

    offset: 区域中心相对 base（m）
    size: 边长 / 面积范围（m）；2D 时 z 厚度用默认桌面高度层
    dim: 2 | 3
    """

    offset: tuple[float, ...]
    size: tuple[float, ...]
    dim: int = 2
    min_separation_m: float = 0.08
    base_xy: tuple[float, float] = (0.0, 0.0)
    base_z: float = 0.42  # 接触点 / 桌面参考

    def world_bounds(self) -> dict[str, float]:
        ox, oy = float(self.offset[0]), float(self.offset[1])
        sx, sy = float(self.size[0]), float(self.size[1])
        bx, by = self.base_xy
        half_x, half_y = abs(sx) * 0.5, abs(sy) * 0.5
        cx, cy = bx + ox, by + oy
        out = {
            "x_min": cx - half_x,
            "x_max": cx + half_x,
            "y_min": cy - half_y,
            "y_max": cy + half_y,
            "z": float(self.base_z + 0.025),
            "min_separation_m": self.min_separation_m,
        }
        if self.dim >= 3 and len(self.offset) >= 3 and len(self.size) >= 3:
            oz, sz = float(self.offset[2]), float(self.size[2])
            cz = self.base_z + oz
            out["z_min"] = cz - abs(sz) * 0.5
            out["z_max"] = cz + abs(sz) * 0.5
            out["z"] = cz
        return out

    def to_dict(self) -> dict[str, Any]:
        return {
            "offset": list(self.offset),
            "size": list(self.size),
            "dim": self.dim,
            "min_separation_m": self.min_separation_m,
            "base_xy": list(self.base_xy),
            "base_z": self.base_z,
            "world_bounds": self.world_bounds(),
        }


def _to_meters(v: float) -> float:
    """数值 ≥10 视为毫米，否则米。"""
    x = float(v)
    return x / 1000.0 if abs(x) >= 10.0 else x


def parse_region_text(raw: str) -> tuple[list[float], list[float]]:
    """解析 [[ox,oy], [sx,sy]] 或三维；支持中文逗号。"""
    s = (raw or "").strip().replace("，", ",").replace("［", "[").replace("］", "]")
    s = re.sub(r"\s+", "", s)
    try:
        data = ast.literal_eval(s)
    except (SyntaxError, ValueError) as exc:
        raise ValueError(f"无法解析区域数组: {raw!r}") from exc
    if not (isinstance(data, (list, tuple)) and len(data) == 2):
        raise ValueError("格式应为 [[offset...], [size...]]")
    off, size = list(data[0]), list(data[1])
    if len(off) not in (2, 3) or len(off) != len(size):
        raise ValueError("offset 与 size 维数须同为 2 或 3")
    return [_to_meters(x) for x in off], [_to_meters(x) for x in size]


def region_from_arrays(
    offset: Sequence[float],
    size: Sequence[float],
    *,
    base_xy: Sequence[float] = (0.0, 0.0),
    base_z: float = 0.42,
    min_separation_m: float = 0.08,
) -> RandomizeRegion:
    dim = 3 if len(offset) >= 3 else 2
    return RandomizeRegion(
        offset=tuple(float(x) for x in offset),
        size=tuple(float(x) for x in size),
        dim=dim,
        min_separation_m=float(min_separation_m),
        base_xy=(float(base_xy[0]), float(base_xy[1])),
        base_z=float(base_z),
    )


def sample_non_colliding(
    region: RandomizeRegion,
    n: int,
    *,
    radii: Sequence[float] | None = None,
    seed: int = 0,
    max_tries: int = 500,
) -> list[np.ndarray]:
    """在区域内采样 n 个点，保证两两距离 ≥ max(min_sep, ri+rj)。"""
    rng = np.random.default_rng(seed)
    b = region.world_bounds()
    rads = list(radii) if radii is not None else [region.min_separation_m * 0.5] * n
    pts: list[np.ndarray] = []
    for i in range(n):
        ok = False
        for _ in range(max_tries):
            x = float(rng.uniform(b["x_min"], b["x_max"]))
            y = float(rng.uniform(b["y_min"], b["y_max"]))
            if "z_min" in b:
                z = float(rng.uniform(b["z_min"], b["z_max"]))
            else:
                z = float(b["z"])
            p = np.array([x, y, z], dtype=np.float64)
            good = True
            for j, q in enumerate(pts):
                need = max(region.min_separation_m, float(rads[i]) + float(rads[j]))
                if float(np.linalg.norm(p[:2] - q[:2])) < need:
                    good = False
                    break
            if good:
                pts.append(p)
                ok = True
                break
        if not ok:
            raise RuntimeError(f"无法在区域内为第 {i} 个物体找到无碰撞位置，请放大区域或减小分离距离")
    return pts
