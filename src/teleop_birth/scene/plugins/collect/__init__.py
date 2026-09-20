"""采集插件：场景无关门面（双 UR / 物流策略可插拔）。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Protocol

from teleop_birth.scene.plugins.base import Plugin, PluginContext, register
from teleop_birth.scene.config import CollectConfig
from teleop_birth.scene.runner import SceneOpsRunner


class CollectPolicy(Protocol):
    def run(self, ctx: Any) -> str: ...


_POLICIES: dict[str, Callable[[], CollectPolicy]] = {}


def register_policy(name: str, factory: Callable[[], CollectPolicy]) -> None:
    _POLICIES[name] = factory


@register
class CollectPlugin(Plugin):
    id = "collect"
    title = "自动采集"

    def list_policies(self) -> list[str]:
        names = {"pick_place", "pick_flip_place"}
        names.update(_POLICIES.keys())
        return sorted(names)

    def attach(self, ctx: PluginContext, mount=None) -> None:
        ctx.extras["collect"] = dict(ctx.scene_cfg.get("collect") or {})

    def run_dual_ur(self, project_root: Path, collect_cfg: CollectConfig | Any) -> None:
        SceneOpsRunner(project_root).run_collect(collect_cfg)

    def run_logistics(self, project_root: Path) -> int:
        import subprocess
        import sys

        script = project_root / "scripts" / "robots" / "artixon_collect.py"
        return subprocess.call([sys.executable, str(script)], cwd=project_root)
