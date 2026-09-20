"""传感器插件（TACTO DIGIT 等）。"""

from __future__ import annotations

from teleop_birth.scene.plugins.base import Plugin, PluginContext, register
from teleop_birth.scene.plugins.sensor.tacto_bridge import (
    MujocoTactoSensors,
    TactileSensorSpec,
    TactoTouchBundle,
)

__all__ = [
    "MujocoTactoSensors",
    "TactileSensorSpec",
    "TactoTouchBundle",
    "SensorPlugin",
]


@register
class SensorPlugin(Plugin):
    id = "sensor"
    title = "触觉 / 传感器"

    def validate(self, ctx: PluginContext) -> list[str]:
        specs = list(ctx.name_map.get("sensor_specs") or ctx.scene_cfg.get("sensors") or [])
        if not specs:
            return []
        tacto = ctx.project_root / "third_party" / "tacto" / "tacto" / "__init__.py"
        if not tacto.is_file():
            return ["缺少 third_party/tacto，请: bash scripts/bootstrap_third_party.sh"]
        return []

    def attach(self, ctx: PluginContext, mount=None) -> None:
        specs = list(ctx.name_map.get("sensor_specs") or ctx.scene_cfg.get("sensors") or [])
        ctx.extras["sensors"] = specs

    def build_runtime(
        self,
        model,
        data,
        *,
        repo_root,
        sensor_specs: list | None = None,
        show_window: bool = True,
        enabled: bool = True,
    ) -> MujocoTactoSensors | None:
        specs_cfg = list(sensor_specs or [])
        if not enabled:
            return None
        if specs_cfg:
            specs = [
                TactileSensorSpec(
                    name=str(s.get("name") or s.get("id") or f"digit_{i}"),
                    body_name=str(s["parent"]),
                    local_origin=tuple(s.get("local_origin") or (0.028, 0.0, 0.0)),
                    local_normal=tuple(s.get("local_normal") or (0.0, 1.0, 0.0)),
                )
                for i, s in enumerate(specs_cfg)
                if s.get("parent")
            ]
        else:
            specs = MujocoTactoSensors.default_arm6a_specs()
        if not specs:
            return None
        try:
            return MujocoTactoSensors(
                model,
                data,
                specs,
                repo_root=repo_root,
                show_window=show_window,
                enabled=True,
            )
        except Exception as exc:
            print(f"[sensor] TACTO 初始化失败: {exc}")
            return None
