"""MuJoCo 多相机：第三人称（世界系）+ 腕部（site 跟随）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np


@dataclass
class CameraViewSpec:
    name: str
    type: str = "world"  # world | site
    pos: list[float] = field(default_factory=lambda: [0.0, -0.55, 1.2])
    lookat: list[float] = field(default_factory=lambda: [0.15, 0.4, 0.45])
    fovy_deg: float = 55.0
    site: str | None = None
    offset_xyz: list[float] = field(default_factory=lambda: [0.0, 0.04, -0.1])
    lookat_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.08])
    topic: str | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, topic: str | None = None) -> "CameraViewSpec":
        return cls(
            name=str(data["name"]),
            type=str(data.get("type") or "world").lower(),
            pos=[float(x) for x in (data.get("pos") or [0.0, -0.55, 1.2])],
            lookat=[float(x) for x in (data.get("lookat") or [0.15, 0.4, 0.45])],
            fovy_deg=float(data.get("fovy_deg", 55.0)),
            site=(str(data["site"]) if data.get("site") else None),
            offset_xyz=[float(x) for x in (data.get("offset_xyz") or [0.0, 0.04, -0.1])],
            lookat_xyz=[float(x) for x in (data.get("lookat_xyz") or [0.0, 0.0, 0.08])],
            topic=topic or data.get("topic"),
        )


@dataclass
class MultiCameraConfig:
    enabled: bool = True
    width: int = 640
    height: int = 480
    every_n_steps: int = 2
    jpeg_quality: int = 85
    views: list[CameraViewSpec] = field(default_factory=list)

    @classmethod
    def from_recording_dict(cls, recording: dict[str, Any]) -> "MultiCameraConfig":
        cam = dict(recording.get("cameras") or {})
        topic_map = dict((recording.get("topics") or {}).get("cameras") or {})
        views: list[CameraViewSpec] = []
        for raw in cam.get("views") or []:
            if not isinstance(raw, dict):
                continue
            name = str(raw.get("name") or "")
            views.append(CameraViewSpec.from_dict(raw, topic=topic_map.get(name)))
        if not views:
            # 无配置时默认：第三人称 + 双腕
            views = [
                CameraViewSpec.from_dict(
                    {
                        "name": "third_person",
                        "type": "world",
                        "pos": [0.05, -0.55, 1.20],
                        "lookat": [0.18, 0.40, 0.45],
                        "fovy_deg": 55.0,
                    },
                    topic=topic_map.get("third_person"),
                ),
                CameraViewSpec.from_dict(
                    {
                        "name": "wrist_left",
                        "type": "site",
                        "site": "L_g_pinch",
                        "offset_xyz": [0.0, 0.04, -0.10],
                        "lookat_xyz": [0.0, 0.0, 0.08],
                        "fovy_deg": 70.0,
                    },
                    topic=topic_map.get("wrist_left"),
                ),
                CameraViewSpec.from_dict(
                    {
                        "name": "wrist_right",
                        "type": "site",
                        "site": "R_g_pinch",
                        "offset_xyz": [0.0, 0.04, -0.10],
                        "lookat_xyz": [0.0, 0.0, 0.08],
                        "fovy_deg": 70.0,
                    },
                    topic=topic_map.get("wrist_right"),
                ),
            ]
        return cls(
            enabled=bool(cam.get("enabled", True)),
            width=int(cam.get("width", 640)),
            height=int(cam.get("height", 480)),
            every_n_steps=int(cam.get("every_n_steps", 2)),
            jpeg_quality=int(cam.get("jpeg_quality", 85)),
            views=views,
        )


class MultiCameraCapture:
    """共享一个 Renderer，按 views 依次出 JPEG。"""

    def __init__(self, model, data, cfg: MultiCameraConfig) -> None:
        self.model = model
        self.data = data
        self.cfg = cfg
        self._renderer = None
        self._step = 0
        self._site_ids: dict[str, int] = {}
        if cfg.enabled and cfg.views:
            import mujoco

            self._renderer = mujoco.Renderer(
                model, height=int(cfg.height), width=int(cfg.width)
            )
            for view in cfg.views:
                if view.type == "site" and view.site:
                    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, view.site)
                    if sid < 0:
                        raise KeyError(f"腕部相机 site 不存在: {view.site}")
                    self._site_ids[view.name] = sid

    def close(self) -> None:
        self._renderer = None

    def _world_pos_lookat(self, view: CameraViewSpec) -> tuple[np.ndarray, np.ndarray]:
        if view.type != "site":
            return (
                np.asarray(view.pos, dtype=np.float64),
                np.asarray(view.lookat, dtype=np.float64),
            )
        sid = self._site_ids[view.name]
        origin = self.data.site_xpos[sid].copy()
        R = self.data.site_xmat[sid].reshape(3, 3).copy()
        off = np.asarray(view.offset_xyz, dtype=np.float64)
        look_local = np.asarray(view.lookat_xyz, dtype=np.float64)
        pos = origin + R @ off
        lookat = origin + R @ look_local
        return pos, lookat

    def _apply_view(self, view: CameraViewSpec) -> None:
        import mujoco

        assert self._renderer is not None
        pos, lookat = self._world_pos_lookat(view)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        delta = pos - lookat
        dist = float(np.linalg.norm(delta))
        cam.lookat[:] = lookat
        cam.distance = max(dist, 0.05)
        cam.azimuth = float(np.degrees(np.arctan2(delta[1], delta[0])) - 90.0)
        horiz = float(np.linalg.norm(delta[:2]))
        cam.elevation = float(-np.degrees(np.arctan2(delta[2], max(horiz, 1e-6))))
        self._renderer.update_scene(self.data, camera=cam)

    def _jpeg_from_rgb(self, rgb: np.ndarray) -> bytes:
        from io import BytesIO

        from PIL import Image

        img = Image.fromarray(np.asarray(rgb, dtype=np.uint8), mode="RGB")
        buf = BytesIO()
        img.save(buf, format="JPEG", quality=int(self.cfg.jpeg_quality))
        return buf.getvalue()

    def maybe_frames(self) -> dict[str, bytes]:
        """返回 {camera_name: jpeg_bytes}；本步不采样则空 dict。"""
        if self._renderer is None or not self.cfg.enabled or not self.cfg.views:
            return {}
        self._step += 1
        if (self._step - 1) % max(1, int(self.cfg.every_n_steps)) != 0:
            return {}
        out: dict[str, bytes] = {}
        for view in self.cfg.views:
            self._apply_view(view)
            rgb = self._renderer.render()
            out[view.name] = self._jpeg_from_rgb(rgb)
        return out


# 兼容旧单相机 API
CameraConfig = MultiCameraConfig
MujocoCameraCapture = MultiCameraCapture
