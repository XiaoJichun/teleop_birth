"""MuJoCo：显示「桌面接触 base」与「转换后 URDF base」。

约定（与 desktop_base 插件一致）：
  T_world_contact — 底座与桌面贴合处（机器人 base / 接触系）
  T_world_urdf    — 经 T_contact_urdf 抬升后的 URDF 原点（转换后 base）
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence

import numpy as np


@dataclass
class BaseFramePose:
    """一个 base 坐标系：原点 + 3x3 旋转（列向量为轴）。"""

    name: str
    origin: np.ndarray  # (3,)
    R: np.ndarray  # (3,3)
    kind: str  # "contact" | "urdf"
    label: str = ""


class BaseFrameOverlay:
    """把 contact / URDF base 轴画到 viewer.user_scn。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._visible = True
        self._frames: list[BaseFramePose] = []
        self._axis_len = 0.12

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

    def clear(self) -> None:
        with self._lock:
            self._frames = []

    def has_frames(self) -> bool:
        with self._lock:
            return bool(self._frames)

    def set_frames(self, frames: Sequence[BaseFramePose]) -> None:
        with self._lock:
            self._frames = list(frames)

    def set_from_scene(self, scene_cfg: Mapping[str, Any] | None) -> None:
        """从场景 dict 解析 dual-UR / 单臂 desktop_base，写入帧列表。"""
        self.set_frames(frames_from_scene(scene_cfg or {}))

    def append_to_scn(self, scn) -> None:
        """追加几何到已清空或未清空的 user_scn（不重置 ngeom）。"""
        with self._lock:
            visible = self._visible
            frames = list(self._frames)
            axis_len = float(self._axis_len)
        if not visible or not frames:
            return
        for fr in frames:
            _draw_frame(scn, fr, axis_len=axis_len)


_GLOBAL: Optional[BaseFrameOverlay] = None
_GLOBAL_LOCK = threading.Lock()


def get_base_frame_overlay() -> BaseFrameOverlay:
    global _GLOBAL
    with _GLOBAL_LOCK:
        if _GLOBAL is None:
            _GLOBAL = BaseFrameOverlay()
        return _GLOBAL


def frames_from_scene(scene_cfg: Mapping[str, Any]) -> list[BaseFramePose]:
    """解析场景 → contact / urdf 帧（双臂则左右各一对）。"""
    from teleop_birth.teleop.frame.desktop_base import (
        T_world_contact,
        T_world_urdf,
        get_desktop_base,
        table_top_z_from_scene,
    )

    db = dict(scene_cfg.get("desktop_base") or {})
    arm_id = str(db.get("arm_id") or scene_cfg.get("robot_id") or "ur5e")
    try:
        top = float(db["table_top_z"]) if "table_top_z" in db else table_top_z_from_scene(scene_cfg)
    except Exception:
        top = float(scene_cfg.get("table_top_z") or scene_cfg.get("base_height") or 0.42)
    xy = list(db.get("xy") or [0.0, 0.0])
    if len(xy) < 2:
        xy = [0.0, 0.0]
    yaw = float(db.get("yaw_rad") or 0.0)
    try:
        spec = get_desktop_base(arm_id)
    except Exception:
        spec = arm_id

    frames: list[BaseFramePose] = []
    # 双 UR：±base_x；ArtiXon：left_arm_xy / right_arm_xy；否则单点 desktop_base.xy
    base_x = scene_cfg.get("base_x")
    left_xy = scene_cfg.get("left_arm_xy")
    right_xy = scene_cfg.get("right_arm_xy")
    if left_xy is not None and right_xy is not None:
        sides = (
            ("L", float(left_xy[0]), float(left_xy[1])),
            ("R", float(right_xy[0]), float(right_xy[1])),
        )
    elif base_x is not None and float(base_x) > 1e-6:
        half = float(base_x)
        sides = (
            ("L", float(xy[0]) - half, float(xy[1])),
            ("R", float(xy[0]) + half, float(xy[1])),
        )
    else:
        sides = (("base", float(xy[0]), float(xy[1])),)

    for tag, ox, oy in sides:
        Tc = T_world_contact(top, [ox, oy], yaw_rad=yaw)
        Tu = T_world_urdf(top, [ox, oy], spec, yaw_rad=yaw)
        prefix = f"{tag}_" if tag in ("L", "R") else ""
        frames.append(
            BaseFramePose(
                name=f"{prefix}contact",
                origin=Tc[:3, 3].copy(),
                R=Tc[:3, :3].copy(),
                kind="contact",
                label=f"{tag} 桌面接触 base",
            )
        )
        frames.append(
            BaseFramePose(
                name=f"{prefix}urdf",
                origin=Tu[:3, 3].copy(),
                R=Tu[:3, :3].copy(),
                kind="urdf",
                label=f"{tag} 转换后 URDF base",
            )
        )
    return frames


def _draw_frame(scn, fr: BaseFramePose, *, axis_len: float) -> None:
    import mujoco

    o = np.asarray(fr.origin, dtype=np.float64).reshape(3)
    R = np.asarray(fr.R, dtype=np.float64).reshape(3, 3)
    # 接触=青色扁盘（贴桌）；URDF=品红球 + 更长轴（即使 z 重合也能区分）
    if fr.kind == "contact":
        if scn.ngeom < scn.maxgeom:
            i = scn.ngeom
            scn.ngeom = i + 1
            g = scn.geoms[i]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                np.array([0.035, 0.004, 0.0], dtype=np.float64),
                o,
                R.reshape(9),
                np.array([0.05, 0.85, 0.95, 0.85], dtype=np.float64),
            )
        length = float(axis_len) * 0.85
        radius = 0.004
    else:
        if scn.ngeom < scn.maxgeom:
            i = scn.ngeom
            scn.ngeom = i + 1
            g = scn.geoms[i]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.016, 0, 0], dtype=np.float64),
                o,
                np.eye(3, dtype=np.float64).reshape(9),
                np.array([0.95, 0.25, 0.75, 0.95], dtype=np.float64),
            )
        length = float(axis_len) * 1.2
        radius = 0.005

    colors = (
        np.array([0.95, 0.15, 0.12, 0.98], dtype=np.float64),
        np.array([0.15, 0.85, 0.20, 0.98], dtype=np.float64),
        np.array([0.20, 0.40, 0.98, 0.98], dtype=np.float64),
    )
    for axi in range(3):
        if scn.ngeom >= scn.maxgeom:
            break
        p1 = o.copy()
        p2 = o + R[:, axi] * length
        i = scn.ngeom
        scn.ngeom = i + 1
        g = scn.geoms[i]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.zeros(9),
            colors[axi],
        )
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, p1, p2)


def draw_contact_urdf_links(scn, frames: Sequence[BaseFramePose]) -> None:
    """同名前缀的 contact→urdf 画一条黄色连线。"""
    import mujoco

    by: dict[str, dict[str, BaseFramePose]] = {}
    for fr in frames:
        key = fr.name.rsplit("_", 1)[0] if "_" in fr.name else "base"
        if fr.name.endswith("contact") or fr.kind == "contact":
            key = fr.name.replace("_contact", "").replace("contact", "base")
        if fr.name.endswith("urdf") or fr.kind == "urdf":
            key = fr.name.replace("_urdf", "").replace("urdf", "base")
        # 更稳：用 label 前缀 L/R/base
        tag = fr.label.split()[0] if fr.label else "base"
        by.setdefault(tag, {})[fr.kind] = fr

    for pair in by.values():
        if "contact" not in pair or "urdf" not in pair:
            continue
        if scn.ngeom >= scn.maxgeom:
            break
        p1 = np.asarray(pair["contact"].origin, dtype=np.float64)
        p2 = np.asarray(pair["urdf"].origin, dtype=np.float64)
        i = scn.ngeom
        scn.ngeom = i + 1
        g = scn.geoms[i]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.zeros(9),
            np.array([0.95, 0.85, 0.1, 0.9], dtype=np.float64),
        )
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.003, p1, p2)


def apply_viewer_overlays(viewer) -> None:
    """统一写入 user_scn：随机化区域 + base 坐标系（单次清屏）。"""
    if viewer is None:
        return
    try:
        lock = viewer.lock()
    except Exception:
        return
    with lock:
        scn = getattr(viewer, "user_scn", None)
        if scn is None:
            return
        scn.ngeom = 0

        # 区域（半透明绿盒）
        try:
            from teleop_birth.ui.session.region_viz import get_region_overlay

            ov = get_region_overlay()
            with ov._lock:
                visible = ov._visible
                region = ov._region
            if visible and region is not None:
                from teleop_birth.ui.session.region_viz import _add_region_box

                _add_region_box(scn, region)
        except Exception:
            pass

        # base 帧
        try:
            bov = get_base_frame_overlay()
            with bov._lock:
                frames = list(bov._frames)
                vis = bov._visible
            if vis and frames:
                bov.append_to_scn(scn)
                draw_contact_urdf_links(scn, frames)
        except Exception:
            pass


def add_base_frame_geoms_to_spec(
    scene,
    scene_cfg: Mapping[str, Any],
) -> int:
    """在 MjSpec 中加入 contact/urdf 小球（编译进模型，相机页也能看到）。"""
    import mujoco

    frames = frames_from_scene(scene_cfg)
    n = 0
    for fr in frames:
        o = [float(x) for x in fr.origin]
        R = np.asarray(fr.R, dtype=np.float64)
        if fr.kind == "contact":
            # 贴桌青色扁圆柱
            try:
                scene.worldbody.add_geom(
                    name=f"viz_{fr.name}_contact",
                    type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=[0.035, 0.004],
                    pos=o,
                    rgba=[0.05, 0.85, 0.95, 0.85],
                    contype=0,
                    conaffinity=0,
                )
                n += 1
            except Exception:
                continue
            length = 0.08
        else:
            try:
                scene.worldbody.add_geom(
                    name=f"viz_{fr.name}_urdf",
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.014],
                    pos=o,
                    rgba=[0.95, 0.25, 0.75, 0.9],
                    contype=0,
                    conaffinity=0,
                )
                n += 1
            except Exception:
                continue
            length = 0.10
        axis_rgba = (
            [0.9, 0.1, 0.1, 0.85],
            [0.1, 0.8, 0.15, 0.85],
            [0.15, 0.3, 0.95, 0.85],
        )
        for axi, col in enumerate(axis_rgba):
            mid = (np.asarray(o, dtype=np.float64) + R[:, axi] * (length * 0.5)).tolist()
            quat = _quat_z_to(R[:, axi])
            try:
                scene.worldbody.add_geom(
                    name=f"viz_{fr.name}_ax{axi}",
                    type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                    size=[0.0035, length * 0.5],
                    pos=[float(mid[0]), float(mid[1]), float(mid[2])],
                    quat=quat,
                    rgba=list(col),
                    contype=0,
                    conaffinity=0,
                )
                n += 1
            except Exception:
                continue
    return n


def _quat_z_to(v: np.ndarray) -> list[float]:
    """返回 wxyz，使局部 +Z 对齐到世界方向 v。"""
    v = np.asarray(v, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    v = v / n
    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if float(np.dot(z, v)) > 0.999999:
        return [1.0, 0.0, 0.0, 0.0]
    if float(np.dot(z, v)) < -0.999999:
        return [0.0, 1.0, 0.0, 0.0]
    ax = np.cross(z, v)
    ax = ax / float(np.linalg.norm(ax))
    ang = float(np.arccos(np.clip(np.dot(z, v), -1.0, 1.0)))
    s = float(np.sin(ang * 0.5))
    return [float(np.cos(ang * 0.5)), float(ax[0] * s), float(ax[1] * s), float(ax[2] * s)]
