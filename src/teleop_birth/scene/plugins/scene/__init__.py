"""场景组装插件：按 YAML 清单调度臂/夹爪/相机/传感器/物理。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from teleop_birth.scene.assets import AssetRegistry
from teleop_birth.scene.plugins.base import Plugin, PluginContext, register


@register
class ScenePlugin(Plugin):
    id = "scene"
    title = "场景组装"

    def validate(self, ctx: PluginContext) -> list[str]:
        errs: list[str] = []
        kind = str((ctx.scene_cfg.get("backend") or {}).get("type") or ctx.scene_cfg.get("type") or "")
        if not kind:
            errs.append("场景缺少 type/backend.type（dual_ur5e | logistics_x2arm）")
        return errs

    def compose(self, ctx: PluginContext) -> tuple[Any, dict[str, Any]]:
        """返回 (model_or_backend_factory_hint, name_map)。

        实际 MjModel 由 adapter 构建，避免此处硬绑 MuJoCo 生命周期。
        """
        cfg = ctx.scene_cfg
        stype = str(
            (cfg.get("backend") or {}).get("type")
            or cfg.get("type")
            or "dual_ur5e"
        ).lower()
        ctx.name_map["scene_type"] = stype
        ctx.name_map["scene_id"] = str(cfg.get("id") or stype)
        reg = AssetRegistry(ctx.project_root)
        ctx.extras["registry"] = reg

        if stype in ("dual_ur5e", "mujoco_dual_ur", "dual_ur"):
            ctx.name_map["adapter"] = "dual_ur5e"
        elif stype in ("logistics_x2arm", "logistics", "artixon"):
            ctx.name_map["adapter"] = "logistics_x2arm"
        else:
            raise ValueError(f"未知场景类型: {stype}")

        # 挂载声明留给各子插件
        for cam in cfg.get("cameras") or []:
            ctx.name_map.setdefault("camera_specs", []).append(dict(cam))
        for ee in cfg.get("grippers") or cfg.get("end_effectors") or []:
            ctx.name_map.setdefault("gripper_specs", []).append(dict(ee))
        for sens in cfg.get("sensors") or []:
            ctx.name_map.setdefault("sensor_specs", []).append(dict(sens))
        ctx.name_map["physics"] = dict(cfg.get("physics") or {})
        return stype, ctx.name_map


def load_scene_yaml(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    assert isinstance(raw, dict)
    return raw


def compose_scene(project_root: Path, scene_yaml: Path | dict[str, Any]) -> PluginContext:
    cfg = load_scene_yaml(scene_yaml) if isinstance(scene_yaml, Path) else dict(scene_yaml)
    ctx = PluginContext(project_root=Path(project_root), scene_cfg=cfg)
    ScenePlugin().compose(ctx)
    # 桌面 base / 相机内参外参：即插即用写入 ctx.extras
    from teleop_birth.scene.plugins.frame import DesktopBasePlugin
    from teleop_birth.scene.plugins.camera import CameraPlugin

    DesktopBasePlugin().attach(ctx)
    CameraPlugin().attach(ctx)
    return ctx
