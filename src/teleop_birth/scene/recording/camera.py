"""MuJoCo 多相机：第三人称（世界系）+ 腕部（site 跟随）。

默认机位约定（可由资产 / CameraPlugin 覆盖）：
  - 腕部：Intel RealSense D405
  - 第三人称：Intel RealSense D455
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

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
    # 针孔内参（可选；采集 JPEG 时主要用于元数据）
    intrinsics: dict[str, float] | None = None
    camera_asset: str | None = None
    role: str | None = None
    T_parent_cam: list[list[float]] | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, topic: str | None = None) -> "CameraViewSpec":
        K = data.get("intrinsics")
        if isinstance(K, Mapping):
            intrinsics = {
                "fx": float(K.get("fx", 0.0)),
                "fy": float(K.get("fy", 0.0)),
                "cx": float(K.get("cx", 0.0)),
                "cy": float(K.get("cy", 0.0)),
            }
        else:
            intrinsics = None
        T = data.get("T_parent_cam") or (data.get("extrinsics") or {}).get("T")
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
            intrinsics=intrinsics,
            camera_asset=(str(data["camera_asset"]) if data.get("camera_asset") else None),
            role=(str(data["role"]) if data.get("role") else None),
            T_parent_cam=([list(map(float, row)) for row in T] if T is not None else None),
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
            # 无配置时默认：D455 第三人称 + 双腕 D405
            views = [
                CameraViewSpec.from_dict(
                    {
                        "name": "third_person",
                        "type": "world",
                        "role": "third_person",
                        "camera_asset": "realsense_d455",
                        "pos": [0.05, -0.55, 1.20],
                        "lookat": [0.18, 0.40, 0.45],
                        "fovy_deg": 58.0,
                        "intrinsics": {"fx": 390.0, "fy": 390.0, "cx": 320.0, "cy": 240.0},
                    },
                    topic=topic_map.get("third_person"),
                ),
                CameraViewSpec.from_dict(
                    {
                        "name": "wrist_left",
                        "type": "site",
                        "role": "wrist",
                        "camera_asset": "realsense_d405",
                        "site": "L_g_pinch",
                        "offset_xyz": [0.0, 0.04, -0.10],
                        "lookat_xyz": [0.0, 0.0, 0.08],
                        "fovy_deg": 58.0,
                        "intrinsics": {"fx": 430.0, "fy": 430.0, "cx": 320.0, "cy": 240.0},
                    },
                    topic=topic_map.get("wrist_left"),
                ),
                CameraViewSpec.from_dict(
                    {
                        "name": "wrist_right",
                        "type": "site",
                        "role": "wrist",
                        "camera_asset": "realsense_d405",
                        "site": "R_g_pinch",
                        "offset_xyz": [0.0, 0.04, -0.10],
                        "lookat_xyz": [0.0, 0.0, 0.08],
                        "fovy_deg": 58.0,
                        "intrinsics": {"fx": 430.0, "fy": 430.0, "cx": 320.0, "cy": 240.0},
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

    @classmethod
    def from_scene(
        cls,
        project_root: Path | str,
        scene_cfg: Mapping[str, Any],
        *,
        recording: Mapping[str, Any] | None = None,
    ) -> "MultiCameraConfig":
        """用 CameraPlugin 解析场景 cameras（自动 D405/D455）再合成采集配置。"""
        from teleop_birth.scene.plugins.camera import CameraPlugin

        plug = CameraPlugin()
        cam_defaults = dict(scene_cfg.get("camera") or {})
        defaults = {
            "wrist": cam_defaults.get("wrist_asset") or "realsense_d405",
            "third_person": cam_defaults.get("third_person_asset") or "realsense_d455",
            "default_mount": cam_defaults.get("default_mount") or "wrist_camera_bracket",
            "width": int((recording or {}).get("cameras", {}).get("width", 640))
            if recording
            else 640,
            "height": int((recording or {}).get("cameras", {}).get("height", 480))
            if recording
            else 480,
        }
        resolved = plug.resolve_views(
            Path(project_root),
            list(scene_cfg.get("cameras") or []),
            defaults=defaults,
        )
        rec = dict(recording or {})
        cam_block = dict(rec.get("cameras") or {})
        topic_map = dict((rec.get("topics") or {}).get("cameras") or {})
        rec_views = {
            str(v.get("name")): v
            for v in (cam_block.get("views") or [])
            if isinstance(v, dict)
        }
        views: list[CameraViewSpec] = []
        for r in resolved:
            name = str(r["name"])
            merged = dict(r)
            if name in rec_views:
                for k in ("pos", "lookat", "site", "offset_xyz", "lookat_xyz", "fovy_deg", "type"):
                    if k in rec_views[name] and rec_views[name][k] is not None:
                        merged[k] = rec_views[name][k]
            views.append(CameraViewSpec.from_dict(merged, topic=topic_map.get(name)))
        return cls(
            enabled=bool(cam_block.get("enabled", True)),
            width=int(cam_block.get("width", defaults["width"])),
            height=int(cam_block.get("height", defaults["height"])),
            every_n_steps=int(cam_block.get("every_n_steps", 2)),
            jpeg_quality=int(cam_block.get("jpeg_quality", 85)),
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

            renderer_cls = getattr(mujoco, "Renderer", None)
            if renderer_cls is None:
                self.cfg = MultiCameraConfig(
                    enabled=False,
                    width=cfg.width,
                    height=cfg.height,
                    every_n_steps=cfg.every_n_steps,
                    jpeg_quality=cfg.jpeg_quality,
                    views=[],
                )
            else:
                self._renderer = renderer_cls(
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

    def capture_rgb(self) -> dict[str, np.ndarray]:
        """立即渲染所有相机，返回 {name: HxWx3 uint8}（推理/通讯用）。"""
        if self._renderer is None or not self.cfg.enabled or not self.cfg.views:
            return {}
        out: dict[str, np.ndarray] = {}
        for view in self.cfg.views:
            self._apply_view(view)
            rgb = np.asarray(self._renderer.render(), dtype=np.uint8).copy()
            out[view.name] = rgb
        return out

    def calibration_dict(self) -> dict[str, Any]:
        """导出各相机内参/外参（供 MCAP 元数据或标定落盘）。"""
        out: dict[str, Any] = {}
        for v in self.cfg.views:
            out[v.name] = {
                "camera_asset": v.camera_asset,
                "role": v.role,
                "type": v.type,
                "fovy_deg": v.fovy_deg,
                "intrinsics": v.intrinsics,
                "T_parent_cam": v.T_parent_cam,
                "site": v.site,
                "pos": v.pos,
                "lookat": v.lookat,
                "offset_xyz": v.offset_xyz,
                "lookat_xyz": v.lookat_xyz,
            }
        return out


# 兼容旧单相机 API
CameraConfig = MultiCameraConfig
MujocoCameraCapture = MultiCameraCapture
