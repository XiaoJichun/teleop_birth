"""夹爪 / 灵巧手插件：从 catalog / menagerie 解析真实模型。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from teleop_birth.scene.assets import AssetRegistry
from teleop_birth.scene.plugins.base import Plugin, PluginContext, register


@register
class GripperPlugin(Plugin):
    id = "gripper"
    title = "夹爪 / 灵巧手"

    def list_available(
        self, project_root: Path, *, robot_id: str | None = None
    ) -> list[dict[str, Any]]:
        """列出夹爪；若给 robot_id 则只返回与该机兼容的项。"""
        from teleop_birth.robot.registry import grippers_compatible_with

        reg = AssetRegistry(project_root)
        allow: set[str] | None = None
        if robot_id:
            allow = set(grippers_compatible_with(robot_id))
        out = []
        for key in reg.list_keys("grippers"):
            if allow is not None and key not in allow:
                continue
            ref = reg.resolve("grippers", key)
            # catalog 侧 compatible_robots 再校验一层（防登记不一致）
            cron = ref.meta.get("compatible_robots")
            if robot_id and isinstance(cron, (list, tuple)) and cron:
                if robot_id not in [str(x) for x in cron]:
                    continue
            out.append(
                {
                    "id": key,
                    "title": ref.meta.get("title") or key,
                    "path": str(ref.path) if ref.path else None,
                    "exists": bool(ref.path and ref.path.is_file()),
                    "builtin": bool(ref.meta.get("builtin")),
                    "source": ref.source,
                    "compatible_robots": list(cron or []),
                }
            )
        return out

    def validate(self, ctx: PluginContext) -> list[str]:
        errs: list[str] = []
        reg = AssetRegistry(ctx.project_root)
        for g in ctx.name_map.get("gripper_specs") or ctx.scene_cfg.get("grippers") or []:
            gid = str(g.get("id") or g.get("type") or "")
            if not gid:
                continue
            try:
                ref = reg.resolve("grippers", gid)
            except KeyError:
                errs.append(f"未知夹爪: {gid}")
                continue
            if ref.meta.get("builtin"):
                continue
            if ref.path and not ref.path.is_file():
                errs.append(
                    f"夹爪资产缺失 {gid}: {ref.path} （{reg.bootstrap_hint(ref.source)}）"
                )
        return errs

    def attach(self, ctx: PluginContext, mount=None) -> None:
        specs = list(ctx.name_map.get("gripper_specs") or ctx.scene_cfg.get("grippers") or [])
        ctx.extras["grippers"] = specs
        # 若场景声明了夹爪 id，尝试映射到 MuJoCo scene_cfg.gripper_xml
        gid = ""
        if specs:
            gid = str(specs[0].get("id") or specs[0].get("type") or "")
        if not gid:
            gid = str(ctx.scene_cfg.get("gripper_id") or "")
        if gid:
            from teleop_birth.ui.scene_builder.gripper_util import apply_gripper_to_scene_cfg

            apply_gripper_to_scene_cfg(ctx.project_root, ctx.scene_cfg, gid)
            warn = ctx.scene_cfg.pop("_gripper_warn", None)
            applied = ctx.scene_cfg.pop("_gripper_applied", None)
            if applied:
                ctx.extras["gripper_xml"] = ctx.scene_cfg.get("gripper_xml")
                ctx.name_map["gripper_applied"] = applied
            if warn:
                ctx.extras.setdefault("warnings", []).append(warn)
