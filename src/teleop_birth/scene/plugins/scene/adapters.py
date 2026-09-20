"""场景适配器：把场景清单映射到现有 builder/backend。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

from teleop_birth.scene.plugins.scene import compose_scene


def resolve_adapter(project_root: Path, scene_yaml: Path | dict[str, Any]) -> dict[str, Any]:
    ctx = compose_scene(project_root, scene_yaml)
    adapter = str(ctx.name_map.get("adapter") or "")
    out: dict[str, Any] = {
        "ctx": ctx,
        "adapter": adapter,
        "teleop_runner": None,
        "collect_runner": None,
    }
    if adapter == "dual_ur5e":
        out["teleop_runner"] = "start_pico_sim.sh"
        out["collect_kind"] = "dual_ur"
    elif adapter == "logistics_x2arm":
        out["teleop_runner"] = "start_logistics_teleop.sh"
        out["collect_kind"] = "logistics"
        out["module_config"] = (
            (ctx.scene_cfg.get("backend") or {}).get("module_config")
            or "configs/robots/artixon_logistics_scene.yaml"
        )
    return out
