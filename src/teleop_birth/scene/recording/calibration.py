"""相机内参/外参标定会话：采集前解析 → 落盘 → 写入数据集。

仿真默认：从 CameraPlugin + D405/D455 资产解析（工厂近似）。
真机/覆盖：放 `configs/scene/calibration/override.yaml` 或会话 JSON，同名字段覆盖。
"""

from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from teleop_birth.scene.recording.camera import CameraViewSpec, MultiCameraConfig

__all__ = [
    "CameraCalibration",
    "apply_calibration_to_config",
    "build_calibration",
    "load_calibration",
    "load_override_yaml",
    "save_calibration",
    "validate_calibration",
]


def _K_matrix(intrinsics: Mapping[str, float]) -> list[list[float]]:
    fx = float(intrinsics.get("fx", 0.0))
    fy = float(intrinsics.get("fy", 0.0))
    cx = float(intrinsics.get("cx", 0.0))
    cy = float(intrinsics.get("cy", 0.0))
    return [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]


def _cam_entry_from_view(view: CameraViewSpec | Mapping[str, Any]) -> dict[str, Any]:
    if isinstance(view, CameraViewSpec):
        K = view.intrinsics or {}
        T = view.T_parent_cam
        return {
            "name": view.name,
            "role": view.role,
            "type": view.type,
            "camera_asset": view.camera_asset,
            "site": view.site,
            "fovy_deg": float(view.fovy_deg),
            "intrinsics": {
                "fx": float(K.get("fx", 0.0)),
                "fy": float(K.get("fy", 0.0)),
                "cx": float(K.get("cx", 0.0)),
                "cy": float(K.get("cy", 0.0)),
            },
            "K": _K_matrix(K) if K else None,
            "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
            "extrinsics": {
                "parent": view.site if view.type == "site" else "world",
                "parent_kind": "site" if view.type == "site" else "world",
                "T": T,
                "pos": list(view.offset_xyz) if view.type == "site" else list(view.pos),
                "lookat": list(view.lookat_xyz) if view.type == "site" else list(view.lookat),
            },
            "pos": list(view.pos),
            "lookat": list(view.lookat),
            "offset_xyz": list(view.offset_xyz),
            "lookat_xyz": list(view.lookat_xyz),
        }
    # already dict (from CameraPlugin.resolve)
    d = dict(view)
    K = dict(d.get("intrinsics") or {})
    return {
        "name": str(d.get("name") or ""),
        "role": d.get("role"),
        "type": d.get("type"),
        "camera_asset": d.get("camera_asset"),
        "site": d.get("site"),
        "fovy_deg": float(d.get("fovy_deg", 55.0)),
        "intrinsics": {
            "fx": float(K.get("fx", 0.0)),
            "fy": float(K.get("fy", 0.0)),
            "cx": float(K.get("cx", 0.0)),
            "cy": float(K.get("cy", 0.0)),
        },
        "K": d.get("K") or (_K_matrix(K) if K else None),
        "distortion": list(d.get("distortion") or [0.0, 0.0, 0.0, 0.0, 0.0]),
        "extrinsics": dict(d.get("extrinsics") or {"T": d.get("T_parent_cam")}),
        "pos": list(d.get("pos") or []),
        "lookat": list(d.get("lookat") or []),
        "offset_xyz": list(d.get("offset_xyz") or []),
        "lookat_xyz": list(d.get("lookat_xyz") or []),
    }


def load_override_yaml(path: Path | str | None) -> dict[str, Any]:
    """覆盖文件格式：

    cameras:
      wrist_right:
        intrinsics: {fx, fy, cx, cy}
        extrinsics: {T: [[...]]}   # 或 pos + quat_wxyz
    """
    if not path:
        return {}
    p = Path(path)
    if not p.is_file():
        return {}
    raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        return {}
    return dict(raw.get("cameras") or raw)


def _merge_override(entry: dict[str, Any], ov: Mapping[str, Any]) -> dict[str, Any]:
    out = deepcopy(entry)
    if "intrinsics" in ov and isinstance(ov["intrinsics"], Mapping):
        out["intrinsics"] = {
            "fx": float(ov["intrinsics"].get("fx", out["intrinsics"]["fx"])),
            "fy": float(ov["intrinsics"].get("fy", out["intrinsics"]["fy"])),
            "cx": float(ov["intrinsics"].get("cx", out["intrinsics"]["cx"])),
            "cy": float(ov["intrinsics"].get("cy", out["intrinsics"]["cy"])),
        }
        out["K"] = _K_matrix(out["intrinsics"])
    if "K" in ov and ov["K"] is not None:
        out["K"] = [list(map(float, row)) for row in ov["K"]]
        out["intrinsics"] = {
            "fx": float(out["K"][0][0]),
            "fy": float(out["K"][1][1]),
            "cx": float(out["K"][0][2]),
            "cy": float(out["K"][1][2]),
        }
    if "distortion" in ov:
        out["distortion"] = [float(x) for x in ov["distortion"]]
    if "extrinsics" in ov and isinstance(ov["extrinsics"], Mapping):
        ex = dict(out.get("extrinsics") or {})
        ex.update(dict(ov["extrinsics"]))
        out["extrinsics"] = ex
    if "T_parent_cam" in ov:
        out.setdefault("extrinsics", {})["T"] = [
            list(map(float, row)) for row in ov["T_parent_cam"]
        ]
    for k in ("fovy_deg", "pos", "lookat", "offset_xyz", "lookat_xyz", "site", "camera_asset"):
        if k in ov and ov[k] is not None:
            out[k] = ov[k]
    return out


def build_calibration(
    *,
    views: Sequence[CameraViewSpec | Mapping[str, Any]],
    width: int = 640,
    height: int = 480,
    source: str = "plugin_resolve",
    override: Mapping[str, Any] | None = None,
    notes: str = "",
) -> dict[str, Any]:
    """从已解析机位构建标定字典（可叠加 override）。"""
    cameras: dict[str, Any] = {}
    ov_all = dict(override or {})
    for v in views:
        entry = _cam_entry_from_view(v)
        name = str(entry["name"])
        if name in ov_all:
            entry = _merge_override(entry, dict(ov_all[name]))
        cameras[name] = entry
    return {
        "version": 1,
        "source": source,
        "timestamp_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "width": int(width),
        "height": int(height),
        "notes": notes
        or "仿真：D405(腕)/D455(第三人称) 资产近似；真机请用 override 覆盖厂家标定",
        "cameras": cameras,
    }


def validate_calibration(calib: Mapping[str, Any]) -> list[str]:
    errs: list[str] = []
    cams = calib.get("cameras")
    if not isinstance(cams, Mapping) or not cams:
        return ["calibration.cameras 为空"]
    for name, c in cams.items():
        if not isinstance(c, Mapping):
            errs.append(f"{name}: 非法条目")
            continue
        K = c.get("intrinsics") or {}
        for key in ("fx", "fy", "cx", "cy"):
            try:
                if float(K.get(key, 0.0)) == 0.0 and key in ("fx", "fy"):
                    errs.append(f"{name}: intrinsics.{key} 无效")
            except (TypeError, ValueError):
                errs.append(f"{name}: intrinsics.{key} 非数值")
        ex = c.get("extrinsics") or {}
        T = ex.get("T") if isinstance(ex, Mapping) else None
        if T is None:
            errs.append(f"{name}: 缺少 extrinsics.T")
        elif not (isinstance(T, list) and len(T) == 4 and all(len(r) == 4 for r in T)):
            errs.append(f"{name}: extrinsics.T 应为 4×4")
    return errs


def save_calibration(calib: Mapping[str, Any], path: Path | str) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(calib, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return p


def load_calibration(path: Path | str) -> dict[str, Any]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise TypeError(f"calibration JSON 应为 object: {path}")
    return raw


def apply_calibration_to_config(cfg: MultiCameraConfig, calib: Mapping[str, Any]) -> MultiCameraConfig:
    """把标定结果写回 MultiCameraConfig.views（采集渲染用同一套 K/外参）。"""
    cams = dict(calib.get("cameras") or {})
    new_views: list[CameraViewSpec] = []
    for v in cfg.views:
        c = cams.get(v.name)
        if not c:
            new_views.append(v)
            continue
        data = {
            "name": v.name,
            "type": c.get("type") or v.type,
            "role": c.get("role") or v.role,
            "camera_asset": c.get("camera_asset") or v.camera_asset,
            "site": c.get("site") if c.get("site") is not None else v.site,
            "pos": c.get("pos") or v.pos,
            "lookat": c.get("lookat") or v.lookat,
            "offset_xyz": c.get("offset_xyz") or v.offset_xyz,
            "lookat_xyz": c.get("lookat_xyz") or v.lookat_xyz,
            "fovy_deg": float(c.get("fovy_deg", v.fovy_deg)),
            "intrinsics": c.get("intrinsics") or v.intrinsics,
            "T_parent_cam": (c.get("extrinsics") or {}).get("T") or v.T_parent_cam,
            "topic": v.topic,
        }
        new_views.append(CameraViewSpec.from_dict(data, topic=v.topic))
    return MultiCameraConfig(
        enabled=cfg.enabled,
        width=int(calib.get("width", cfg.width)),
        height=int(calib.get("height", cfg.height)),
        every_n_steps=cfg.every_n_steps,
        jpeg_quality=cfg.jpeg_quality,
        views=new_views,
    )


class CameraCalibration:
    """采集前标定门面：解析 → 校验 → 落盘 → 应用到 config。"""

    def __init__(self, project_root: Path) -> None:
        self.project_root = Path(project_root)
        self.payload: dict[str, Any] | None = None
        self.path: Path | None = None

    def run(
        self,
        camera_cfg: MultiCameraConfig,
        *,
        session_dir: Path,
        override_rel: str | None = "scene/calibration/override.yaml",
        existing_json: str | Path | None = None,
    ) -> dict[str, Any]:
        """执行标定步骤。

        优先级：existing_json > override.yaml 叠加 plugin 解析 > 仅 plugin 解析。
        """
        session_dir = Path(session_dir)
        session_dir.mkdir(parents=True, exist_ok=True)

        override: dict[str, Any] = {}
        source = "plugin_resolve"
        if existing_json:
            p = Path(existing_json)
            if not p.is_file():
                p = self.project_root / existing_json
            if p.is_file():
                self.payload = load_calibration(p)
                self.path = save_calibration(self.payload, session_dir / "calibration.json")
                return self.payload

        if override_rel:
            ov_path = self.project_root / "configs" / override_rel
            if not ov_path.is_file():
                ov_path = self.project_root / override_rel
            override = load_override_yaml(ov_path if ov_path.is_file() else None)
            if override:
                source = "plugin_resolve+override"

        self.payload = build_calibration(
            views=camera_cfg.views,
            width=camera_cfg.width,
            height=camera_cfg.height,
            source=source,
            override=override,
        )
        errs = validate_calibration(self.payload)
        if errs:
            raise ValueError("相机标定校验失败:\n  - " + "\n  - ".join(errs))
        self.path = save_calibration(self.payload, session_dir / "calibration.json")
        return self.payload

    def summary_lines(self) -> list[str]:
        if not self.payload:
            return ["(无标定)"]
        lines = [
            f"source={self.payload.get('source')}  "
            f"{self.payload.get('width')}x{self.payload.get('height')}  "
            f"ts={self.payload.get('timestamp_utc')}"
        ]
        for name, c in (self.payload.get("cameras") or {}).items():
            K = c.get("intrinsics") or {}
            asset = c.get("camera_asset") or "?"
            lines.append(
                f"  [{name}] {asset}  "
                f"K=({K.get('fx'):.1f},{K.get('fy'):.1f},{K.get('cx'):.1f},{K.get('cy'):.1f})"
            )
        if self.path:
            lines.append(f"  → {self.path}")
        return lines
