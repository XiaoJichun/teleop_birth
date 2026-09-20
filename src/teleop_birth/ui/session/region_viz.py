"""MuJoCo 随机化区域可视化：半透明盒子 + 可切换显隐。"""

from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any, Optional

import numpy as np

from teleop_birth.ui.session.randomize import RandomizeRegion


class RegionOverlay:
    """把 RandomizeRegion 画到 viewer.user_scn；线程安全切换 visible。"""

    def __init__(self, region: RandomizeRegion | dict[str, Any] | None = None) -> None:
        self._lock = threading.Lock()
        self._visible = True
        self._region: RandomizeRegion | None = None
        if region is not None:
            self.set_region(region)

    @property
    def visible(self) -> bool:
        with self._lock:
            return self._visible

    def set_visible(self, on: bool) -> None:
        with self._lock:
            self._visible = bool(on)

    def toggle(self) -> bool:
        with self._lock:
            self._visible = not self._visible
            return self._visible

    def set_region(self, region: RandomizeRegion | dict[str, Any]) -> None:
        with self._lock:
            if isinstance(region, RandomizeRegion):
                self._region = region
            else:
                self._region = _region_from_dict(region)

    def clear_region(self) -> None:
        with self._lock:
            self._region = None

    def apply(self, viewer) -> None:
        """在 viewer.sync() 前调用；写入 user_scn。"""
        if viewer is None:
            return

        with self._lock:
            visible = self._visible
            region = self._region

        try:
            lock = viewer.lock()
        except Exception:
            return
        with lock:
            scn = getattr(viewer, "user_scn", None)
            if scn is None:
                return
            scn.ngeom = 0
            if not visible or region is None:
                return
            _add_region_box(scn, region)


def _region_from_dict(data: dict[str, Any]) -> RandomizeRegion:
    from teleop_birth.ui.session.round_layouts import region_from_plan_dict

    r = region_from_plan_dict(data)
    if r is None:
        raise ValueError("无法从 dict 构造 RandomizeRegion")
    return r


def _add_region_box(scn, region: RandomizeRegion) -> None:
    import mujoco

    b = region.world_bounds()
    x0, x1 = float(b["x_min"]), float(b["x_max"])
    y0, y1 = float(b["y_min"]), float(b["y_max"])
    if "z_min" in b and "z_max" in b:
        z0, z1 = float(b["z_min"]), float(b["z_max"])
    else:
        z = float(b.get("z", region.base_z + 0.025))
        z0, z1 = z - 0.002, z + 0.012

    cx = 0.5 * (x0 + x1)
    cy = 0.5 * (y0 + y1)
    cz = 0.5 * (z0 + z1)
    hx = max(1e-4, 0.5 * abs(x1 - x0))
    hy = max(1e-4, 0.5 * abs(y1 - y0))
    hz = max(1e-4, 0.5 * abs(z1 - z0))

    if scn.ngeom >= scn.maxgeom:
        return
    i = scn.ngeom
    scn.ngeom = i + 1
    g = scn.geoms[i]
    mujoco.mjv_initGeom(
        g,
        mujoco.mjtGeom.mjGEOM_BOX,
        np.array([hx, hy, hz], dtype=np.float64),
        np.array([cx, cy, cz], dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(9),
        np.array([0.15, 0.85, 0.35, 0.28], dtype=np.float64),
    )
    corners = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]
    for a, bidx in edges:
        if scn.ngeom >= scn.maxgeom:
            break
        p1 = np.array(corners[a], dtype=np.float64)
        p2 = np.array(corners[bidx], dtype=np.float64)
        i = scn.ngeom
        scn.ngeom = i + 1
        g = scn.geoms[i]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.zeros(9),
            np.array([0.05, 0.95, 0.25, 0.95], dtype=np.float64),
        )
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.003, p1, p2)


_GLOBAL: Optional[RegionOverlay] = None
_GLOBAL_LOCK = threading.Lock()


def get_region_overlay() -> RegionOverlay:
    global _GLOBAL
    with _GLOBAL_LOCK:
        if _GLOBAL is None:
            _GLOBAL = RegionOverlay()
        return _GLOBAL


def attach_region_from_env_or_dict(data: dict[str, Any] | None = None) -> RegionOverlay | None:
    """从 dict / TELEOP_PLAN_JSON 挂载区域到全局 overlay。"""
    region = None
    if isinstance(data, dict):
        region = data.get("region") or data.get("randomize_region")
        if region is None and ("world_bounds" in data or "offset" in data):
            region = data
    if region is None:
        plan = os.environ.get("TELEOP_PLAN_JSON", "").strip()
        if plan and Path(plan).is_file():
            try:
                payload = json.loads(Path(plan).read_text(encoding="utf-8"))
                region = payload.get("region")
            except Exception:
                region = None
    if not region:
        return None
    ov = get_region_overlay()
    ov.set_region(region)
    ov.set_visible(True)
    return ov
