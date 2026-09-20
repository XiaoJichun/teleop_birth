"""物理 / 防穿模插件。"""

from __future__ import annotations

from teleop_birth.scene.plugins.base import Plugin, PluginContext, register
from teleop_birth.scene.plugins.physics.anti_penetration import (
    PROP_CONAFFINITY,
    PROP_CONTYPE,
    AntiPenetrationManager,
)

__all__ = [
    "AntiPenetrationManager",
    "PROP_CONTYPE",
    "PROP_CONAFFINITY",
    "PhysicsPlugin",
]


@register
class PhysicsPlugin(Plugin):
    id = "physics"
    title = "防穿模 / 碰撞"

    def attach(self, ctx: PluginContext, mount=None) -> None:
        phys = dict(ctx.name_map.get("physics") or ctx.scene_cfg.get("physics") or {})
        if not bool(phys.get("anti_penetration", True)):
            return
        # 运行时由 backend/collector 创建 Manager；此处只登记配置
        ctx.extras["physics_cfg"] = {
            "anti_penetration": True,
            "min_separation_m": float(phys.get("min_separation_m", 0.08)),
            "tcp_clearance_m": float(phys.get("tcp_clearance_m", 0.06)),
            "table_top_z": phys.get("table_top_z"),
        }
