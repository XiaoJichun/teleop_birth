"""相机插件：按角色自动选 D405（腕部）/ D455（第三人称），解析内参与外参。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
import yaml

from teleop_birth.scene.assets import AssetRegistry
from teleop_birth.scene.plugins.base import MountSpec, Plugin, PluginContext, register
from teleop_birth.scene.plugins.camera.tuner import run_camera_tuner

# 约定：腕部 → Intel RealSense D405；第三人称 → D455
DEFAULT_WRIST_ASSET = "realsense_d405"
DEFAULT_THIRD_PERSON_ASSET = "realsense_d455"
DEFAULT_WRIST_MOUNT = "wrist_camera_bracket"


def _infer_role(cam: Mapping[str, Any]) -> str:
    """从 name/type/role 推断相机角色。"""
    explicit = str(cam.get("role") or "").strip().lower()
    if explicit in ("wrist", "third_person", "third", "ego"):
        return "wrist" if explicit in ("wrist", "ego") else "third_person"
    name = str(cam.get("name") or "").lower()
    ctype = str(cam.get("type") or "").lower()
    if "wrist" in name or ctype == "site":
        return "wrist"
    if "third" in name or ctype in ("world", "free"):
        return "third_person"
    return "third_person"


def _default_asset_for_role(role: str) -> str:
    return DEFAULT_WRIST_ASSET if role == "wrist" else DEFAULT_THIRD_PERSON_ASSET


def _quat_wxyz_to_R(q: Sequence[float]) -> np.ndarray:
    """MuJoCo 常用 wxyz → 3×3。"""
    w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    n = (w * w + x * x + y * y + z * z) ** 0.5
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _T_from_pos_quat_wxyz(pos: Sequence[float], quat_wxyz: Sequence[float]) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _quat_wxyz_to_R(quat_wxyz)
    T[0, 3] = float(pos[0])
    T[1, 3] = float(pos[1])
    T[2, 3] = float(pos[2])
    return T


def _K_from_intrinsics(
    intrinsics: Mapping[str, Any] | None,
    *,
    width: int,
    height: int,
    fovy_deg: float | None,
) -> dict[str, float]:
    """返回 fx, fy, cx, cy；缺省时由 fovy 估算。"""
    if intrinsics:
        return {
            "fx": float(intrinsics.get("fx", 0.0)),
            "fy": float(intrinsics.get("fy", 0.0)),
            "cx": float(intrinsics.get("cx", width * 0.5)),
            "cy": float(intrinsics.get("cy", height * 0.5)),
        }
    fy = float(height) / (2.0 * np.tan(np.radians(float(fovy_deg or 55.0) * 0.5)))
    fx = fy
    return {"fx": fx, "fy": fy, "cx": float(width) * 0.5, "cy": float(height) * 0.5}


@register
class CameraPlugin(Plugin):
    """相机：资产内参 + 支架外参；腕部默认 D405，第三人称默认 D455。"""

    id = "camera"
    title = "相机"

    def list_available(self, project_root: Path) -> list[dict[str, Any]]:
        reg = AssetRegistry(project_root)
        out = []
        for key in reg.list_keys("cameras"):
            ref = reg.resolve("cameras", key)
            out.append(
                {
                    "id": key,
                    "title": ref.meta.get("title") or key,
                    "path": str(ref.path) if ref.path else None,
                    "exists": bool(ref.path and ref.path.is_file()),
                    "role": (ref.meta.get("role") if isinstance(ref.meta, dict) else None),
                }
            )
        return out

    def load_camera_asset(self, project_root: Path, camera_id: str) -> dict[str, Any]:
        reg = AssetRegistry(project_root)
        # 兼容 catalog id 与短名
        aliases = {
            "d405": DEFAULT_WRIST_ASSET,
            "realsense_d405": DEFAULT_WRIST_ASSET,
            "d455": DEFAULT_THIRD_PERSON_ASSET,
            "realsense_d455": DEFAULT_THIRD_PERSON_ASSET,
        }
        cid = aliases.get(str(camera_id).lower(), camera_id)
        ref = reg.resolve("cameras", cid)
        if ref.path and ref.path.is_file():
            data = yaml.safe_load(ref.path.read_text(encoding="utf-8")) or {}
            data.setdefault("id", cid)
            return data
        return dict(ref.meta)

    def load_mount(self, project_root: Path, mount_id: str) -> MountSpec:
        reg = AssetRegistry(project_root)
        ref = reg.resolve("mounts", mount_id)
        raw = {}
        if ref.path and ref.path.is_file():
            raw = yaml.safe_load(ref.path.read_text(encoding="utf-8")) or {}
        pos = tuple(float(x) for x in (raw.get("pos") or [0, 0, 0]))
        quat = tuple(float(x) for x in (raw.get("quat_wxyz") or [1, 0, 0, 0]))
        return MountSpec(
            parent=str(raw.get("parent") or ""),
            parent_kind=str(raw.get("parent_kind") or "site"),
            pos=pos,  # type: ignore[arg-type]
            quat_wxyz=quat,  # type: ignore[arg-type]
            extra=raw,
        )

    def resolve_one(
        self,
        project_root: Path,
        cam: Mapping[str, Any],
        *,
        defaults: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """解析单路相机：角色 → 资产 → K + 外参 T。"""
        defaults = dict(defaults or {})
        role = _infer_role(cam)
        asset_id = str(
            cam.get("camera_asset")
            or cam.get("asset")
            or defaults.get(role)
            or _default_asset_for_role(role)
        )
        asset = self.load_camera_asset(project_root, asset_id)
        width = int(cam.get("width") or asset.get("width") or defaults.get("width") or 640)
        height = int(cam.get("height") or asset.get("height") or defaults.get("height") or 480)
        fovy = float(cam.get("fovy_deg") or asset.get("fovy_deg") or 55.0)
        K = _K_from_intrinsics(
            dict(cam.get("intrinsics") or asset.get("intrinsics") or {}),
            width=width,
            height=height,
            fovy_deg=fovy,
        )

        mount_id = str(cam.get("mount") or defaults.get("default_mount") or "")
        mount_raw: dict[str, Any] = {}
        T_parent_cam = np.eye(4, dtype=np.float64)
        offset_xyz = list(cam.get("offset_xyz") or [])
        lookat_xyz = list(cam.get("lookat_xyz") or [])
        if role == "wrist":
            mid = mount_id or DEFAULT_WRIST_MOUNT
            try:
                mspec = self.load_mount(project_root, mid)
                mount_raw = dict(mspec.extra or {})
                mount_id = mid
                if not offset_xyz:
                    offset_xyz = [float(x) for x in mspec.pos]
                look_local = mount_raw.get("lookat_local")
                if not lookat_xyz and look_local is not None:
                    lookat_xyz = [float(x) for x in look_local]
                T_parent_cam = _T_from_pos_quat_wxyz(mspec.pos, mspec.quat_wxyz)
            except KeyError:
                mount_id = mid
                if not offset_xyz:
                    offset_xyz = [0.0, 0.04, -0.10]
                if not lookat_xyz:
                    lookat_xyz = [0.0, 0.0, 0.08]
                T_parent_cam = np.eye(4, dtype=np.float64)
                T_parent_cam[:3, 3] = offset_xyz
        else:
            # 第三人称：世界系 pos/lookat；外参相对世界 = 相机位姿
            pos = list(cam.get("pos") or asset.get("default_pos") or [0.05, -0.55, 1.20])
            lookat = list(cam.get("lookat") or [0.18, 0.40, 0.45])
            # 由 lookat 构造粗外参（仅平移 + 朝向近似；采集仍用 pos/lookat）
            T_parent_cam = np.eye(4, dtype=np.float64)
            T_parent_cam[:3, 3] = [float(pos[0]), float(pos[1]), float(pos[2])]
            offset_xyz = offset_xyz or [0.0, 0.0, 0.0]
            lookat_xyz = lookat_xyz or lookat

        out: dict[str, Any] = {
            "name": str(cam.get("name") or role),
            "type": str(cam.get("type") or ("site" if role == "wrist" else "world")).lower(),
            "role": role,
            "camera_asset": asset_id,
            "mount": mount_id or None,
            "site": cam.get("site"),
            "pos": list(cam.get("pos") or asset.get("default_pos") or [0.05, -0.55, 1.20]),
            "lookat": list(cam.get("lookat") or [0.18, 0.40, 0.45]),
            "offset_xyz": [float(x) for x in (offset_xyz or [0.0, 0.04, -0.10])],
            "lookat_xyz": [float(x) for x in (lookat_xyz or [0.0, 0.0, 0.08])],
            "fovy_deg": fovy,
            "width": width,
            "height": height,
            "intrinsics": K,
            "K": [[K["fx"], 0.0, K["cx"]], [0.0, K["fy"], K["cy"]], [0.0, 0.0, 1.0]],
            "T_parent_cam": T_parent_cam.tolist(),
            "extrinsics": {
                "parent": str(cam.get("site") or ("world" if role == "third_person" else "")),
                "parent_kind": "site" if role == "wrist" else "world",
                "pos": [float(T_parent_cam[0, 3]), float(T_parent_cam[1, 3]), float(T_parent_cam[2, 3])],
                "quat_wxyz": list(
                    (mount_raw.get("quat_wxyz") if mount_raw else None) or [1.0, 0.0, 0.0, 0.0]
                ),
                "T": T_parent_cam.tolist(),
            },
            "topic": cam.get("topic"),
            "asset_title": asset.get("title"),
            "notes": asset.get("notes"),
        }
        return out

    def resolve_views(
        self,
        project_root: Path,
        cameras: Sequence[Mapping[str, Any]] | None,
        *,
        defaults: Mapping[str, Any] | None = None,
    ) -> list[dict[str, Any]]:
        """批量解析场景 cameras 列表。空列表时给默认 D455 + 双腕 D405。"""
        defaults = dict(defaults or {})
        cams = list(cameras or [])
        if not cams:
            cams = [
                {"name": "third_person", "type": "world", "role": "third_person"},
                {
                    "name": "wrist_left",
                    "type": "site",
                    "role": "wrist",
                    "site": "L_g_pinch",
                    "mount": DEFAULT_WRIST_MOUNT,
                },
                {
                    "name": "wrist_right",
                    "type": "site",
                    "role": "wrist",
                    "site": "R_g_pinch",
                    "mount": DEFAULT_WRIST_MOUNT,
                },
            ]
        return [self.resolve_one(project_root, c, defaults=defaults) for c in cams]

    def attach(self, ctx: PluginContext, mount=None) -> None:
        plugin_defaults = {}
        # configs/plugins/default.yaml 可经 scene 合并；也支持 scene.camera
        cam_cfg = dict(ctx.scene_cfg.get("camera") or {})
        if cam_cfg.get("wrist_asset"):
            plugin_defaults["wrist"] = cam_cfg["wrist_asset"]
        if cam_cfg.get("third_person_asset"):
            plugin_defaults["third_person"] = cam_cfg["third_person_asset"]
        if cam_cfg.get("default_mount"):
            plugin_defaults["default_mount"] = cam_cfg["default_mount"]
        # 未写时：腕 D405 / 第三人称 D455
        plugin_defaults.setdefault("wrist", DEFAULT_WRIST_ASSET)
        plugin_defaults.setdefault("third_person", DEFAULT_THIRD_PERSON_ASSET)
        plugin_defaults.setdefault("default_mount", DEFAULT_WRIST_MOUNT)

        raw = list(ctx.name_map.get("camera_specs") or ctx.scene_cfg.get("cameras") or [])
        resolved = self.resolve_views(ctx.project_root, raw, defaults=plugin_defaults)
        ctx.extras["cameras"] = resolved
        ctx.extras["camera_intrinsics"] = {c["name"]: c["intrinsics"] for c in resolved}
        ctx.extras["camera_extrinsics"] = {c["name"]: c["extrinsics"] for c in resolved}
        ctx.name_map["camera_specs"] = resolved
        ctx.name_map["camera_K"] = {c["name"]: c["K"] for c in resolved}

    def calibrate(
        self,
        project_root: Path,
        cameras: Sequence[Mapping[str, Any]] | None = None,
        *,
        session_dir: Path | None = None,
        override_rel: str | None = "scene/calibration/override.yaml",
        defaults: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """采集前标定：解析机位 → 校验 → 可选落盘。"""
        from teleop_birth.scene.recording.calibration import CameraCalibration, build_calibration
        from teleop_birth.scene.recording.camera import CameraViewSpec, MultiCameraConfig

        defaults = dict(defaults or {})
        defaults.setdefault("wrist", DEFAULT_WRIST_ASSET)
        defaults.setdefault("third_person", DEFAULT_THIRD_PERSON_ASSET)
        defaults.setdefault("default_mount", DEFAULT_WRIST_MOUNT)
        resolved = self.resolve_views(project_root, cameras, defaults=defaults)
        views = [CameraViewSpec.from_dict(r) for r in resolved]
        cfg = MultiCameraConfig(views=views, width=640, height=480)
        if session_dir is None:
            return build_calibration(views=views, width=640, height=480, source="plugin_resolve")
        gate = CameraCalibration(project_root)
        return gate.run(cfg, session_dir=Path(session_dir), override_rel=override_rel)

    def tune(self, project_root: Path, scene_yaml: Path | None = None, *, select=None) -> None:
        run_camera_tuner(project_root, scene_yaml, select=select)
